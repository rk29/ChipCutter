use chip_cam::{ResolvedStockGeometry, SegmentMotion3D, StockAxis, ToolpathSegment3D};
use chip_math::DVec2;
use chip_tools::CutterGeometry;
use rayon::prelude::*;

use crate::{cutter_profile::RadialCutterProfile, timeline::SimOperationInput};

use super::{
    SetupSimulationResult, SimError, SimResolution, StockPlaybackCursor, target_cursor_from_snap,
};

pub const UNCUT_OP_IDX: u16 = u16::MAX;

/// High-resolution 2D Z-Dexel heightmap aligned with a Setup's WCS stock bounds (`06-simulation.md` §3).
#[derive(Debug, Clone)]
pub struct HeightfieldStock {
    pub nx: usize,
    pub ny: usize,
    pub origin_x_mm: f64,
    pub origin_y_mm: f64,
    pub dx_mm: f64,
    pub dy_mm: f64,
    pub max_initial_top_z_mm: f64,
    pub min_bottom_z_mm: f64,
    pub in_stock: Vec<bool>,
    pub z_stock: Vec<f32>,
    pub z_bottom: Vec<f32>,
    pub z_target: Vec<f32>,
    /// `u16::MAX` indicates unmachined raw stock; `0..N` indicates the last operation index that cut this cell.
    pub last_op_idx: Vec<u16>,
}

impl HeightfieldStock {
    /// Initializes a Z-dexel stock grid from a [`ResolvedSetup`]'s stock solid (`Box` or `Cylinder`).
    pub fn from_setup_stock(
        stock: &ResolvedStockGeometry,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let aabb = stock.aabb_wcs();
        if !aabb.is_valid() {
            return Err(SimError::InvalidStockBounds);
        }
        let size = aabb.size();
        if size.x <= 1e-4 || size.y <= 1e-4 || size.z <= 1e-4 {
            return Err(SimError::InvalidStockBounds);
        }

        let max_dim = resolution.max_grid_dim() as f64;
        let longest_xy = size.x.max(size.y);
        let target_cell = longest_xy / max_dim;
        let nx = ((size.x / target_cell).round() as usize).clamp(8, resolution.max_grid_dim());
        let ny = ((size.y / target_cell).round() as usize).clamp(8, resolution.max_grid_dim());
        let dx_mm = size.x / (nx as f64);
        let dy_mm = size.y / (ny as f64);

        let total = nx * ny;
        let mut in_stock = vec![false; total];
        let mut z_stock = vec![f32::NEG_INFINITY; total];
        let mut z_bottom = vec![f32::NEG_INFINITY; total];
        let z_target = vec![f32::NEG_INFINITY; total];
        let last_op_idx = vec![UNCUT_OP_IDX; total];

        match *stock {
            ResolvedStockGeometry::Box { aabb_wcs } => {
                in_stock.fill(true);
                z_stock.fill(aabb_wcs.max.z as f32);
                z_bottom.fill(aabb_wcs.min.z as f32);
            }
            ResolvedStockGeometry::Cylinder {
                axis,
                center_wcs,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                ..
            } => {
                let r2 = radius_mm * radius_mm;
                for j in 0..ny {
                    let y = aabb.min.y + (j as f64 + 0.5) * dy_mm;
                    let row = j * nx;
                    for i in 0..nx {
                        let x = aabb.min.x + (i as f64 + 0.5) * dx_mm;
                        let idx = row + i;
                        match axis {
                            StockAxis::Z => {
                                let d2 = (x - center_wcs.x).powi(2) + (y - center_wcs.y).powi(2);
                                if d2 <= r2 {
                                    in_stock[idx] = true;
                                    z_stock[idx] = max_axial_mm as f32;
                                    z_bottom[idx] = min_axial_mm as f32;
                                }
                            }
                            StockAxis::X => {
                                let dy_c = y - center_wcs.y;
                                if x >= min_axial_mm && x <= max_axial_mm && dy_c * dy_c <= r2 {
                                    let dz = (r2 - dy_c * dy_c).max(0.0).sqrt();
                                    in_stock[idx] = true;
                                    z_stock[idx] = (center_wcs.z + dz) as f32;
                                    z_bottom[idx] = (center_wcs.z - dz) as f32;
                                }
                            }
                            StockAxis::Y => {
                                let dx_c = x - center_wcs.x;
                                if y >= min_axial_mm && y <= max_axial_mm && dx_c * dx_c <= r2 {
                                    let dz = (r2 - dx_c * dx_c).max(0.0).sqrt();
                                    in_stock[idx] = true;
                                    z_stock[idx] = (center_wcs.z + dz) as f32;
                                    z_bottom[idx] = (center_wcs.z - dz) as f32;
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(Self {
            nx,
            ny,
            origin_x_mm: aabb.min.x,
            origin_y_mm: aabb.min.y,
            dx_mm,
            dy_mm,
            max_initial_top_z_mm: aabb.max.z,
            min_bottom_z_mm: aabb.min.z,
            in_stock,
            z_stock,
            z_bottom,
            z_target,
            last_op_idx,
        })
    }

    #[inline]
    pub fn cell_center_xy(&self, i: usize, j: usize) -> DVec2 {
        DVec2::new(
            self.origin_x_mm + (i as f64 + 0.5) * self.dx_mm,
            self.origin_y_mm + (j as f64 + 0.5) * self.dy_mm,
        )
    }

    /// Samples remaining `z_stock` height (`mm`) at WCS `(x, y)` (`None` if outside stock or drilled through).
    pub fn sample_stock_z(&self, x_wcs: f64, y_wcs: f64) -> Option<f64> {
        if x_wcs < self.origin_x_mm || y_wcs < self.origin_y_mm {
            return None;
        }
        let i = ((x_wcs - self.origin_x_mm) / self.dx_mm).floor() as isize;
        let j = ((y_wcs - self.origin_y_mm) / self.dy_mm).floor() as isize;
        if i < 0 || j < 0 || (i as usize) >= self.nx || (j as usize) >= self.ny {
            return None;
        }
        let idx = (j as usize) * self.nx + (i as usize);
        if !self.in_stock[idx] {
            return None;
        }
        let z = f64::from(self.z_stock[idx]);
        let zb = f64::from(self.z_bottom[idx]);
        if z <= zb + 1e-5 { None } else { Some(z) }
    }

    /// Samples the maximum remaining `z_stock` height (`mm`) across all cells within `radius_mm`
    /// of `(cx, cy)` (always including the center cell; `None` if all cells are empty/through-drilled).
    pub fn sample_max_stock_z_in_disk(&self, cx: f64, cy: f64, radius_mm: f64) -> Option<f64> {
        let mut best = self.sample_stock_z(cx, cy);
        let r = radius_mm.max(0.0);
        if let (Some((i0, i1)), Some((j0, j1))) = (
            self.index_span_x(cx - r, cx + r),
            self.index_span_y(cy - r, cy + r),
        ) {
            let r2 = r * r;
            for j in j0..=j1 {
                let py = self.origin_y_mm + (j as f64 + 0.5) * self.dy_mm;
                let dy = py - cy;
                let row = j * self.nx;
                for i in i0..=i1 {
                    let px = self.origin_x_mm + (i as f64 + 0.5) * self.dx_mm;
                    let dx = px - cx;
                    if dx * dx + dy * dy <= r2 + 1e-9 {
                        let idx = row + i;
                        if self.in_stock[idx] {
                            let z = f64::from(self.z_stock[idx]);
                            let zb = f64::from(self.z_bottom[idx]);
                            if z > zb + 1e-5 && best.is_none_or(|b| z > b) {
                                best = Some(z);
                            }
                        }
                    }
                }
            }
        }
        best
    }

    /// Computes total remaining stock volume (`mm³`) across all cells.
    pub fn compute_volume_mm3(&self) -> f64 {
        let cell_area = self.dx_mm * self.dy_mm;
        let sum_h: f64 = self
            .in_stock
            .par_iter()
            .zip(self.z_stock.par_iter())
            .zip(self.z_bottom.par_iter())
            .map(|((&inside, &zt), &zb)| {
                if inside && zt > zb {
                    f64::from(zt - zb)
                } else {
                    0.0
                }
            })
            .sum();
        sum_h * cell_area
    }

    pub(super) fn index_span_x(&self, min_x: f64, max_x: f64) -> Option<(usize, usize)> {
        if max_x < self.origin_x_mm || min_x > self.origin_x_mm + (self.nx as f64) * self.dx_mm {
            return None;
        }
        let i0 = (((min_x - self.origin_x_mm) / self.dx_mm).floor() as isize).max(0) as usize;
        let i1 = (((max_x - self.origin_x_mm) / self.dx_mm).ceil() as isize)
            .clamp(0, (self.nx - 1) as isize) as usize;
        if i0 <= i1 { Some((i0, i1)) } else { None }
    }

    pub(super) fn index_span_y(&self, min_y: f64, max_y: f64) -> Option<(usize, usize)> {
        if max_y < self.origin_y_mm || min_y > self.origin_y_mm + (self.ny as f64) * self.dy_mm {
            return None;
        }
        let j0 = (((min_y - self.origin_y_mm) / self.dy_mm).floor() as isize).max(0) as usize;
        let j1 = (((max_y - self.origin_y_mm) / self.dy_mm).ceil() as isize)
            .clamp(0, (self.ny - 1) as isize) as usize;
        if j0 <= j1 { Some((j0, j1)) } else { None }
    }
}

impl SetupSimulationResult {
    /// Reconstructs the exact `HeightfieldStock` state at `time_sec` by restoring the nearest
    /// preceding [`StockCheckpoint`] and sweeping only the short tail delta up to `time_sec`!
    pub fn reconstruct_stock_at_time(
        &self,
        ops: &[SimOperationInput<'_>],
        time_sec: f64,
    ) -> HeightfieldStock {
        let mut stock = self.final_stock.clone();
        let mut cursor = None;
        if self.operation_profiles.is_empty() && !ops.is_empty() {
            let profiles: Vec<(RadialCutterProfile, bool)> = ops
                .iter()
                .map(|o| {
                    (
                        RadialCutterProfile::from_tool(o.tool),
                        matches!(o.tool.geometry, CutterGeometry::FlatEndmill { .. }),
                    )
                })
                .collect();
            self.sync_stock_to_time_with_profiles(&mut stock, &mut cursor, &profiles, time_sec);
        } else {
            self.sync_stock_to_time(&mut stock, &mut cursor, time_sec);
        }
        stock
    }

    /// Synchronizes a reusable working [`HeightfieldStock`] to the exact timeline state at
    /// `time_sec`, advancing incrementally when moving forward along the timeline and restoring
    /// from the nearest [`StockCheckpoint`] when scrubbing backward or jumping past a checkpoint.
    ///
    /// Returns `true` if `stock` changed (and any display mesh should be re-tessellated).
    pub fn sync_stock_to_time(
        &self,
        stock: &mut HeightfieldStock,
        cursor: &mut Option<StockPlaybackCursor>,
        time_sec: f64,
    ) -> bool {
        self.sync_stock_to_time_with_profiles(stock, cursor, &self.operation_profiles, time_sec)
    }

    fn sync_stock_to_time_with_profiles(
        &self,
        stock: &mut HeightfieldStock,
        cursor: &mut Option<StockPlaybackCursor>,
        profiles: &[(RadialCutterProfile, bool)],
        time_sec: f64,
    ) -> bool {
        let total_spans = self.timeline.motion_spans.len();
        let target = self.timeline.sample_at_time(time_sec).map_or(
            StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            },
            |s| target_cursor_from_snap(&s, total_spans),
        );

        if let Some(cur) = *cursor
            && cur.seg_idx == target.seg_idx
            && (target.seg_u - cur.seg_u).abs() <= 1e-6
        {
            return false;
        }

        if target.seg_idx >= total_spans {
            stock.z_stock.clone_from(&self.final_stock.z_stock);
            stock.last_op_idx.clone_from(&self.final_stock.last_op_idx);
            *cursor = Some(StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            });
            return true;
        }

        let cp = self
            .checkpoints
            .iter()
            .rfind(|c| c.completed_segments <= target.seg_idx);

        let (mut cur_seg, cur_u, mut changed) = if let Some(cur) = *cursor
            && (target.seg_idx > cur.seg_idx
                || (target.seg_idx == cur.seg_idx && target.seg_u >= cur.seg_u))
        {
            if let Some(cp) = cp
                && cp.completed_segments > cur.seg_idx
            {
                stock.z_stock.clone_from(&cp.z_stock);
                stock.last_op_idx.clone_from(&cp.last_op_idx);
                (cp.completed_segments, 0.0, true)
            } else {
                (cur.seg_idx, cur.seg_u, false)
            }
        } else if let Some(cp) = cp {
            stock.z_stock.clone_from(&cp.z_stock);
            stock.last_op_idx.clone_from(&cp.last_op_idx);
            (cp.completed_segments, 0.0, true)
        } else {
            stock.z_stock.clone_from(&self.final_stock.z_stock);
            stock.last_op_idx.clone_from(&self.final_stock.last_op_idx);
            (0, 0.0, true)
        };

        if cur_seg == target.seg_idx {
            if let Some(span) = self.timeline.motion_spans.get(cur_seg)
                && let Some(&(prof, is_flat)) = profiles.get(span.op_idx)
                && let Some(sub_seg) = slice_segment_range(&span.segment, cur_u, target.seg_u)
            {
                changed |=
                    stock.sweep_segment_mutate_only(&sub_seg, &prof, is_flat, span.op_idx as u16);
            }
        } else {
            if cur_u > 1e-6 {
                if let Some(span) = self.timeline.motion_spans.get(cur_seg)
                    && let Some(&(prof, is_flat)) = profiles.get(span.op_idx)
                    && let Some(sub_seg) = slice_segment_range(&span.segment, cur_u, 1.0)
                {
                    changed |= stock.sweep_segment_mutate_only(
                        &sub_seg,
                        &prof,
                        is_flat,
                        span.op_idx as u16,
                    );
                }
                cur_seg += 1;
            }

            for span in &self.timeline.motion_spans[cur_seg..target.seg_idx] {
                if let Some(&(prof, is_flat)) = profiles.get(span.op_idx) {
                    changed |= stock.sweep_segment_mutate_only(
                        &span.segment,
                        &prof,
                        is_flat,
                        span.op_idx as u16,
                    );
                }
            }

            if target.seg_u > 1e-6
                && let Some(span) = self.timeline.motion_spans.get(target.seg_idx)
                && let Some(&(prof, is_flat)) = profiles.get(span.op_idx)
                && let Some(sub_seg) = slice_segment_range(&span.segment, 0.0, target.seg_u)
            {
                changed |=
                    stock.sweep_segment_mutate_only(&sub_seg, &prof, is_flat, span.op_idx as u16);
            }
        }

        *cursor = Some(target);
        changed
    }
}

pub(crate) fn slice_segment_range(
    seg: &ToolpathSegment3D,
    u_start: f64,
    u_end: f64,
) -> Option<ToolpathSegment3D> {
    let a = u_start.clamp(0.0, 1.0);
    let b = u_end.clamp(0.0, 1.0);
    if b - a <= 1e-6 {
        return None;
    }
    let start_wcs = seg.sample_point(a);
    let end_wcs = seg.sample_point(b);
    let motion = match seg.motion {
        SegmentMotion3D::Rapid => SegmentMotion3D::Rapid,
        SegmentMotion3D::Linear { feed_mm_min, role } => {
            SegmentMotion3D::Linear { feed_mm_min, role }
        }
        SegmentMotion3D::Arc {
            plane,
            dir,
            center_wcs,
            radius_mm,
            signed_sweep_rad,
            feed_mm_min,
            role,
        } => SegmentMotion3D::Arc {
            plane,
            dir,
            center_wcs,
            radius_mm,
            signed_sweep_rad: signed_sweep_rad * (b - a),
            feed_mm_min,
            role,
        },
    };
    Some(ToolpathSegment3D {
        start_wcs,
        end_wcs,
        motion,
        start_rotary_deg: seg.sample_rotary_deg(a),
        end_rotary_deg: seg.sample_rotary_deg(b),
    })
}
