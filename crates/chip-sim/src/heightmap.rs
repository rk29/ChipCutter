use std::sync::atomic::{AtomicBool, Ordering};

use chip_cad::{BodyId, CadDocument, SurfaceKind};
use chip_cam::{
    ArcPlane, ExtractedFeature25D, GeometrySelectionItem, ResolvedSetup, ResolvedStockGeometry,
    SegmentMotion3D, StockAxis, ToolpathSegment3D, extract_selection_features,
};
use chip_math::{Aabb2, Aabb3, ArcPolyline2D, DVec2, DVec3};
use chip_tools::CutterGeometry;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    cutter_profile::{RadialCutterProfile, SimColoredVertex},
    timeline::{MachineKinematicsConfig, MotionSpan, SimOperationInput, SimulationTimeline},
    verifier::{SimDiagnostic, SimIssueKind, SimVerificationConfig},
};

/// Errors raised during heightmap simulation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SimError {
    #[error("simulation cancelled")]
    Cancelled,
    #[error("resolved stock bounding box is invalid or degenerate")]
    InvalidStockBounds,
}

/// Configurable Z-dexel grid resolution (`06-simulation.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimResolution {
    /// `~512 × 512` cells (instant interactive updates).
    #[default]
    Standard,
    /// `~1024 × 1024` cells (high-precision verification).
    Fine,
    /// `~2048 × 2048` cells (ultra-fine inspection).
    Ultra,
}

impl SimResolution {
    pub const ALL: [Self; 3] = [Self::Standard, Self::Fine, Self::Ultra];

    pub const fn max_grid_dim(self) -> usize {
        match self {
            Self::Standard => 512,
            Self::Fine => 1024,
            Self::Ultra => 2048,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Standard => "Standard (512)",
            Self::Fine => "Fine (1024)",
            Self::Ultra => "Ultra (2048)",
        }
    }
}

/// Triangulated 3D machined stock mesh (`pos_mm`, `normal`, `color_rgba` in Setup WCS).
#[derive(Debug, Clone, Default)]
pub struct SimStockMesh {
    pub vertices: Vec<SimColoredVertex>,
    pub indices: Vec<u32>,
}

/// Snapshot of `z_stock` and `last_op_idx` after completing `completed_segments` motion spans,
/// enabling `< 4 ms` interactive timeline scrubbing to any timestamp.
#[derive(Debug, Clone)]
pub struct StockCheckpoint {
    pub completed_segments: usize,
    pub z_stock: Vec<f32>,
    pub last_op_idx: Vec<u16>,
}

/// Complete output of simulating a Setup (`timeline`, `final_stock`, `checkpoints`, `diagnostics`).
#[derive(Debug, Clone)]
pub struct SetupSimulationResult {
    pub resolution: SimResolution,
    pub timeline: SimulationTimeline,
    pub initial_stock_volume_mm3: f64,
    pub remaining_stock_volume_mm3: f64,
    pub removed_volume_mm3: f64,
    pub final_stock: HeightfieldStock,
    pub checkpoints: Vec<StockCheckpoint>,
    pub diagnostics: Vec<SimDiagnostic>,
}

impl SetupSimulationResult {
    pub fn critical_error_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == crate::verifier::SimSeverity::CriticalError)
            .count()
    }

    pub fn warning_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == crate::verifier::SimSeverity::Warning)
            .count()
    }

    pub fn has_critical_errors(&self) -> bool {
        self.critical_error_count() > 0
    }

    /// Reconstructs the exact `HeightfieldStock` state at `time_sec` by restoring the nearest
    /// preceding [`StockCheckpoint`] and sweeping only the short tail delta up to `time_sec`!
    pub fn reconstruct_stock_at_time(
        &self,
        ops: &[SimOperationInput<'_>],
        time_sec: f64,
    ) -> HeightfieldStock {
        let Some(snap) = self.timeline.sample_at_time(time_sec) else {
            return self.final_stock.clone();
        };
        if snap.completed_segments >= self.timeline.motion_spans.len() {
            return self.final_stock.clone();
        }

        let target_full_segs = snap.global_seg_idx;
        let cp = self
            .checkpoints
            .iter()
            .rfind(|c| c.completed_segments <= target_full_segs);

        let mut stock = self.final_stock.clone();
        let start_seg = if let Some(cp) = cp {
            stock.z_stock.clone_from(&cp.z_stock);
            stock.last_op_idx.clone_from(&cp.last_op_idx);
            cp.completed_segments
        } else {
            0
        };

        let profiles: Vec<(RadialCutterProfile, bool)> = ops
            .iter()
            .map(|o| {
                (
                    RadialCutterProfile::from_tool(o.tool),
                    matches!(o.tool.geometry, CutterGeometry::FlatEndmill { .. }),
                )
            })
            .collect();

        for span in &self.timeline.motion_spans[start_seg..target_full_segs] {
            if let Some(&(prof, is_flat)) = profiles.get(span.op_idx) {
                stock.sweep_segment_mutate_only(&span.segment, &prof, is_flat, span.op_idx as u16);
            }
        }

        // Apply fractional active segment `(0 .. seg_progress_t)` if mid-move
        if snap.seg_progress_t > 1e-4
            && let Some(span) = self.timeline.motion_spans.get(snap.global_seg_idx)
            && let Some(&(prof, is_flat)) = profiles.get(span.op_idx)
            && let Some(sub_seg) = slice_segment_prefix(&span.segment, snap.seg_progress_t)
        {
            stock.sweep_segment_mutate_only(&sub_seg, &prof, is_flat, span.op_idx as u16);
        }

        stock
    }
}

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

pub const UNCUT_OP_IDX: u16 = u16::MAX;

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

    /// Rasterizes the upper `+Z` envelope of `doc` (filtered to `target_bodies`, or all bodies if empty) onto `self.z_target`.
    ///
    /// Uses exact analytic `ArcPolyline2D::contains_point` for horizontal planar faces (`n_wcs.z > 0.999`)
    /// so circular holes, counterbores, and bosses suffer **zero** polygonal chord sag, combined with
    /// exact analytic surface `z(x, y)` evaluation on sloped planar/chamfer faces and barycentric Z on freeform faces.
    pub fn rasterize_target_brep(
        &mut self,
        doc: &CadDocument,
        setup: &ResolvedSetup,
        target_bodies: &[BodyId],
    ) {
        self.z_target.fill(f32::NEG_INFINITY);
        let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);

        // 1. Collect upward-facing horizontal planar faces with exact analytic `ArcPolyline2D` boundaries
        struct AnalyticHorizFace {
            outer: ArcPolyline2D,
            inners: Vec<ArcPolyline2D>,
            aabb: Aabb2,
            z_wcs: f32,
        }
        let mut horiz_faces: Vec<AnalyticHorizFace> = Vec::new();
        let mut handled_horiz_face_ids = vec![false; doc.faces.len()];

        for face in &doc.faces {
            if !include_body(face.body_id) {
                continue;
            }
            if let SurfaceKind::Plane { normal, .. } = face.surface {
                let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
                if n_wcs.z > 0.999
                    && let Some(binding) = doc.bind_face(face.id)
                    && let Ok(summary) = extract_selection_features(
                        doc,
                        setup,
                        &[GeometrySelectionItem::PlanarFace { binding }],
                    )
                {
                    for feat in summary.features {
                        if let ExtractedFeature25D::PlanarFace {
                            face_id,
                            outer_boundary_wcs,
                            inner_islands_wcs,
                            face_z_wcs,
                            ..
                        } = feat
                            && face_id == face.id
                            && !outer_boundary_wcs.segments.is_empty()
                        {
                            let aabb = outer_boundary_wcs.exact_aabb();
                            if aabb.is_valid() {
                                horiz_faces.push(AnalyticHorizFace {
                                    outer: outer_boundary_wcs,
                                    inners: inner_islands_wcs,
                                    aabb,
                                    z_wcs: face_z_wcs as f32,
                                });
                                if (face.id.0 as usize) < handled_horiz_face_ids.len() {
                                    handled_horiz_face_ids[face.id.0 as usize] = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Parallel row evaluation for all exact horizontal B-Rep faces
        if !horiz_faces.is_empty() {
            let nx = self.nx;
            let ox = self.origin_x_mm;
            let oy = self.origin_y_mm;
            let dx = self.dx_mm;
            let dy = self.dy_mm;

            self.z_target
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row_z)| {
                    let y = oy + (j as f64 + 0.5) * dy;
                    for hf in &horiz_faces {
                        if y < hf.aabb.min.y - 1e-6 || y > hf.aabb.max.y + 1e-6 {
                            continue;
                        }
                        let i0 = (((hf.aabb.min.x - ox) / dx).floor() as isize).max(0) as usize;
                        let i1 = (((hf.aabb.max.x - ox) / dx).ceil() as isize)
                            .clamp(0, (nx - 1) as isize) as usize;
                        for (off, z_cell) in row_z[i0..=i1].iter_mut().enumerate() {
                            if *z_cell >= hf.z_wcs {
                                continue;
                            }
                            let i = i0 + off;
                            let pt = DVec2::new(ox + (i as f64 + 0.5) * dx, y);
                            if hf.outer.contains_point(pt)
                                && !hf.inners.iter().any(|inn| inn.contains_point(pt))
                            {
                                *z_cell = (*z_cell).max(hf.z_wcs);
                            }
                        }
                    }
                });
        }

        // 2. Rasterize sloped / conical / toroidal / freeform upward-facing triangles
        for tri in &doc.render_mesh.pick_triangles {
            let fid_idx = tri.face_id.0 as usize;
            if fid_idx < handled_horiz_face_ids.len() && handled_horiz_face_ids[fid_idx] {
                continue;
            }
            let Some(face) = doc.face(tri.face_id) else {
                continue;
            };
            if !include_body(face.body_id) {
                continue;
            }

            let v0 = setup.frame.world_to_local_point(tri.v0_mm);
            let v1 = setup.frame.world_to_local_point(tri.v1_mm);
            let v2 = setup.frame.world_to_local_point(tri.v2_mm);

            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let tri_n = e1.cross(e2);
            let area2_xy = tri_n.z;
            // Skip vertical or downward-facing triangles (`area2_xy.abs() <= 1e-8` is vertical in XY)
            if area2_xy.abs() <= 1e-8 {
                continue;
            }
            let n_unit = tri_n.normalize_or_zero();
            if n_unit.z <= 1e-4 {
                continue;
            }

            let analytic_plane = if let SurfaceKind::Plane { origin_mm, normal } = face.surface {
                let o_wcs = setup.frame.world_to_local_point(origin_mm);
                let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
                if n_wcs.z.abs() > 1e-4 {
                    Some((o_wcs, n_wcs))
                } else {
                    None
                }
            } else {
                None
            };

            let min_x = v0.x.min(v1.x).min(v2.x);
            let max_x = v0.x.max(v1.x).max(v2.x);
            let min_y = v0.y.min(v1.y).min(v2.y);
            let max_y = v0.y.max(v1.y).max(v2.y);

            let (Some((i0, i1)), Some((j0, j1))) = (
                self.index_span_x(min_x, max_x),
                self.index_span_y(min_y, max_y),
            ) else {
                continue;
            };

            let inv_area2 = 1.0 / area2_xy;
            for j in j0..=j1 {
                let py = self.origin_y_mm + (j as f64 + 0.5) * self.dy_mm;
                let row = j * self.nx;
                for i in i0..=i1 {
                    let px = self.origin_x_mm + (i as f64 + 0.5) * self.dx_mm;
                    let w0 = ((v1.x - px) * (v2.y - py) - (v1.y - py) * (v2.x - px)) * inv_area2;
                    let w1 = ((v2.x - px) * (v0.y - py) - (v2.y - py) * (v0.x - px)) * inv_area2;
                    let w2 = 1.0 - w0 - w1;
                    if w0 >= -1e-6 && w1 >= -1e-6 && w2 >= -1e-6 {
                        let z_val = if let Some((o_wcs, n_wcs)) = analytic_plane {
                            o_wcs.z
                                - (n_wcs.x * (px - o_wcs.x) + n_wcs.y * (py - o_wcs.y)) / n_wcs.z
                        } else {
                            w0 * v0.z + w1 * v1.z + w2 * v2.z
                        };
                        let zf = z_val as f32;
                        if zf > self.z_target[row + i] {
                            self.z_target[row + i] = zf;
                        }
                    }
                }
            }
        }
    }

    fn index_span_x(&self, min_x: f64, max_x: f64) -> Option<(usize, usize)> {
        if max_x < self.origin_x_mm || min_x > self.origin_x_mm + (self.nx as f64) * self.dx_mm {
            return None;
        }
        let i0 = (((min_x - self.origin_x_mm) / self.dx_mm).floor() as isize).max(0) as usize;
        let i1 = (((max_x - self.origin_x_mm) / self.dx_mm).ceil() as isize)
            .clamp(0, (self.nx - 1) as isize) as usize;
        if i0 <= i1 { Some((i0, i1)) } else { None }
    }

    fn index_span_y(&self, min_y: f64, max_y: f64) -> Option<(usize, usize)> {
        if max_y < self.origin_y_mm || min_y > self.origin_y_mm + (self.ny as f64) * self.dy_mm {
            return None;
        }
        let j0 = (((min_y - self.origin_y_mm) / self.dy_mm).floor() as isize).max(0) as usize;
        let j1 = (((max_y - self.origin_y_mm) / self.dy_mm).ceil() as isize)
            .clamp(0, (self.ny - 1) as isize) as usize;
        if j0 <= j1 { Some((j0, j1)) } else { None }
    }

    /// Fast mutation-only sweep used when reconstructing intermediate timeline scrub positions.
    pub fn sweep_segment_mutate_only(
        &mut self,
        seg: &ToolpathSegment3D,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
        op_idx: u16,
    ) {
        let primitives = decompose_segment_primitives(seg);
        for prim in primitives {
            self.apply_primitive_mutate(&prim, prof, is_flat_endmill, op_idx);
        }
    }

    fn apply_primitive_mutate(
        &mut self,
        prim: &SweepPrimitive,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
        op_idx: u16,
    ) {
        let r_cut = prof.cutting_radius_mm;
        let aabb = prim.xy_bounds(r_cut);
        let (Some((i0, i1)), Some((j0, j1))) = (
            self.index_span_x(aabb.min.x, aabb.max.x),
            self.index_span_y(aabb.min.y, aabb.max.y),
        ) else {
            return;
        };

        let min_possible_z = prim.min_tip_z() as f32;
        for j in j0..=j1 {
            let py = self.origin_y_mm + (j as f64 + 0.5) * self.dy_mm;
            let row = j * self.nx;
            for i in i0..=i1 {
                let idx = row + i;
                if !self.in_stock[idx] || self.z_stock[idx] <= min_possible_z {
                    continue;
                }
                let px = self.origin_x_mm + (i as f64 + 0.5) * self.dx_mm;
                if let Some(hit) = prim.eval_cutter_at_xy(px, py, prof, is_flat_endmill) {
                    let z_cut_f = hit.z_cut_mm as f32;
                    if z_cut_f < self.z_stock[idx] {
                        let zb = self.z_bottom[idx];
                        self.z_stock[idx] = if z_cut_f <= zb {
                            f32::NEG_INFINITY
                        } else {
                            z_cut_f
                        };
                        self.last_op_idx[idx] = op_idx;
                    }
                }
            }
        }
    }

    /// Sweeps a single [`MotionSpan`] across `self`, evaluating all 5 safety checks (`06-simulation.md` §4)
    /// and updating `z_stock` and `last_op_idx`.
    #[allow(clippy::too_many_arguments)]
    fn sweep_span_with_verification(
        &mut self,
        span: &MotionSpan,
        op_id: &str,
        op_name: &str,
        tool_number: u32,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
        cfg: &SimVerificationConfig,
        out_hits: &mut Vec<SegmentRuleHit>,
    ) {
        let seg = &span.segment;
        let is_rapid = matches!(seg.motion, SegmentMotion3D::Rapid);

        // Rule 5: Excessive / Non-Center-Cutting Vertical Plunge (`ΔXY == 0`)
        if !is_rapid {
            let dxy = (seg.end_wcs.x - seg.start_wcs.x).hypot(seg.end_wcs.y - seg.start_wcs.y);
            if dxy <= 1e-4
                && seg.end_wcs.z < seg.start_wcs.z - 1e-4
                && let Some(stock_z_here) = self.sample_stock_z(seg.start_wcs.x, seg.start_wcs.y)
            {
                let entry_z = stock_z_here.min(seg.start_wcs.z);
                let plunge_in_mat = entry_z - seg.end_wcs.z;
                if plunge_in_mat > 0.05 {
                    if !prof.is_center_cutting {
                        out_hits.push(SegmentRuleHit {
                            kind: SimIssueKind::UnsafeVerticalPlunge,
                            span_idx: span.global_seg_idx,
                            op_idx: span.op_idx,
                            op_id: op_id.to_string(),
                            op_name: op_name.to_string(),
                            tool_number,
                            time_sec: span.t_start_sec,
                            location_wcs: DVec3::new(seg.end_wcs.x, seg.end_wcs.y, seg.end_wcs.z),
                            excess_mm: plunge_in_mat,
                            detail: format!(
                                "Non-center-cutting tool T{tool_number} plunged vertically {plunge_in_mat:.2} mm into stock (use Helical or Linear ramp)."
                            ),
                        });
                    } else if plunge_in_mat > prof.max_safe_plunge_depth_mm + 0.05 {
                        let excess = plunge_in_mat - prof.max_safe_plunge_depth_mm;
                        out_hits.push(SegmentRuleHit {
                            kind: SimIssueKind::UnsafeVerticalPlunge,
                            span_idx: span.global_seg_idx,
                            op_idx: span.op_idx,
                            op_id: op_id.to_string(),
                            op_name: op_name.to_string(),
                            tool_number,
                            time_sec: span.t_start_sec,
                            location_wcs: DVec3::new(seg.end_wcs.x, seg.end_wcs.y, seg.end_wcs.z),
                            excess_mm: excess,
                            detail: format!(
                                "Vertical plunge depth ({plunge_in_mat:.2} mm) exceeds safe axial plunge threshold ({:.2} mm) on T{tool_number}.",
                                prof.max_safe_plunge_depth_mm
                            ),
                        });
                    }
                }
            }
        }

        let primitives = decompose_segment_primitives(seg);
        let r_inner_skin = (prof.cutting_radius_mm - cfg.radial_wall_skin_mm).max(0.0);

        let mut worst_rapid: Option<(f64, DVec3)> = None;
        let mut worst_gouge: Option<(f64, DVec3)> = None;
        let mut worst_flute: Option<(f64, DVec3)> = None;
        let mut worst_holder: Option<(f64, DVec3)> = None;

        for prim in &primitives {
            let check_holder =
                prim.min_tip_z() + prof.stickout_length_mm < self.max_initial_top_z_mm - 0.02;
            let sweep_r = if check_holder {
                prof.max_assembly_radius_mm()
            } else {
                prof.cutting_radius_mm.max(prof.shank_radius_mm)
            };
            let aabb = prim.xy_bounds(sweep_r);
            let (Some((i0, i1)), Some((j0, j1))) = (
                self.index_span_x(aabb.min.x, aabb.max.x),
                self.index_span_y(aabb.min.y, aabb.max.y),
            ) else {
                continue;
            };

            let row_count = j1 - j0 + 1;
            let ox = self.origin_x_mm;
            let oy = self.origin_y_mm;
            let dx = self.dx_mm;
            let dy = self.dy_mm;
            let nx = self.nx;
            let op_u16 = span.op_idx as u16;

            // Parallelize across grid rows via rayon when the swept footprint spans >= 24 rows
            if row_count >= 24 {
                let z_stock_slice = &mut self.z_stock[j0 * nx..(j1 + 1) * nx];
                let last_op_slice = &mut self.last_op_idx[j0 * nx..(j1 + 1) * nx];
                let in_stock_ref = &self.in_stock[j0 * nx..(j1 + 1) * nx];
                let z_bot_ref = &self.z_bottom[j0 * nx..(j1 + 1) * nx];
                let z_tgt_ref = &self.z_target[j0 * nx..(j1 + 1) * nx];

                let row_reducers: Vec<RowViolationSummary> = z_stock_slice
                    .par_chunks_mut(nx)
                    .zip(last_op_slice.par_chunks_mut(nx))
                    .enumerate()
                    .map(|(rel_j, (row_zs, row_op))| {
                        let j = j0 + rel_j;
                        let py = oy + (j as f64 + 0.5) * dy;
                        let row_base = rel_j * nx;
                        eval_row_cells(
                            i0,
                            i1,
                            ox,
                            dx,
                            py,
                            &in_stock_ref[row_base..row_base + nx],
                            &z_bot_ref[row_base..row_base + nx],
                            &z_tgt_ref[row_base..row_base + nx],
                            row_zs,
                            row_op,
                            prim,
                            prof,
                            is_flat_endmill,
                            is_rapid,
                            check_holder,
                            r_inner_skin,
                            cfg,
                            op_u16,
                        )
                    })
                    .collect();

                for rr in row_reducers {
                    merge_violation(&mut worst_rapid, rr.rapid);
                    merge_violation(&mut worst_gouge, rr.gouge);
                    merge_violation(&mut worst_flute, rr.flute);
                    merge_violation(&mut worst_holder, rr.holder);
                }
            } else {
                for j in j0..=j1 {
                    let py = oy + (j as f64 + 0.5) * dy;
                    let base = j * nx;
                    let rr = eval_row_cells(
                        i0,
                        i1,
                        ox,
                        dx,
                        py,
                        &self.in_stock[base..base + nx],
                        &self.z_bottom[base..base + nx],
                        &self.z_target[base..base + nx],
                        &mut self.z_stock[base..base + nx],
                        &mut self.last_op_idx[base..base + nx],
                        prim,
                        prof,
                        is_flat_endmill,
                        is_rapid,
                        check_holder,
                        r_inner_skin,
                        cfg,
                        op_u16,
                    );
                    merge_violation(&mut worst_rapid, rr.rapid);
                    merge_violation(&mut worst_gouge, rr.gouge);
                    merge_violation(&mut worst_flute, rr.flute);
                    merge_violation(&mut worst_holder, rr.holder);
                }
            }
        }

        if let Some((depth, loc)) = worst_rapid {
            out_hits.push(SegmentRuleHit {
                kind: SimIssueKind::RapidIntoStock,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "G0 rapid move crashed {depth:.3} mm into remaining stock at WCS ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((depth, loc)) = worst_gouge {
            out_hits.push(SegmentRuleHit {
                kind: SimIssueKind::TargetModelGouge,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Tool T{tool_number} gouged target model by {depth:.3} mm at WCS ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((depth, loc)) = worst_flute {
            out_hits.push(SegmentRuleHit {
                kind: SimIssueKind::FluteLengthExceeded,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Engagement depth exceeded T{tool_number} flute length ({:.2} mm) by {depth:.2} mm (shank rub).",
                    prof.flute_length_mm
                ),
            });
        }
        if let Some((depth, loc)) = worst_holder {
            out_hits.push(SegmentRuleHit {
                kind: SimIssueKind::HolderCollision,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Collet/holder of T{tool_number} collided {depth:.2} mm into stock (stickout {:.2} mm insufficient).",
                    prof.stickout_length_mm
                ),
            });
        }
    }
}

#[derive(Default)]
struct RowViolationSummary {
    rapid: Option<(f64, DVec3)>,
    gouge: Option<(f64, DVec3)>,
    flute: Option<(f64, DVec3)>,
    holder: Option<(f64, DVec3)>,
}

fn merge_violation(dst: &mut Option<(f64, DVec3)>, cand: Option<(f64, DVec3)>) {
    if let Some((c_val, c_loc)) = cand
        && dst.is_none_or(|(cur, _)| c_val > cur)
    {
        *dst = Some((c_val, c_loc));
    }
}

#[allow(clippy::too_many_arguments)]
fn eval_row_cells(
    i0: usize,
    i1: usize,
    ox: f64,
    dx: f64,
    py: f64,
    in_stock: &[bool],
    z_bottom: &[f32],
    z_target: &[f32],
    z_stock: &mut [f32],
    last_op: &mut [u16],
    prim: &SweepPrimitive,
    prof: &RadialCutterProfile,
    is_flat_endmill: bool,
    is_rapid: bool,
    check_holder: bool,
    r_inner_skin: f64,
    cfg: &SimVerificationConfig,
    op_idx: u16,
) -> RowViolationSummary {
    let mut out = RowViolationSummary::default();
    let shank_skin = (prof.shank_radius_mm - cfg.radial_wall_skin_mm).max(0.0);
    let holder_skin = (prof.holder_radius_mm - cfg.radial_wall_skin_mm).max(0.0);

    for i in i0..=i1 {
        let px = ox + (i as f64 + 0.5) * dx;
        let zs_before = z_stock[i];
        let has_stock = in_stock[i] && zs_before > z_bottom[i];
        let zt = z_target[i];
        let has_target = zt.is_finite();

        if !has_stock && !has_target {
            continue;
        }

        // Check holder collision when deep enough (`r <= holder_skin`)
        if check_holder
            && has_stock
            && let Some((r_axis, z_tip_min)) = prim.min_tip_z_within_radius(px, py, holder_skin)
            && r_axis <= holder_skin
        {
            let holder_z = z_tip_min + prof.stickout_length_mm;
            let pen = f64::from(zs_before) - holder_z;
            if pen > 0.03 {
                merge_violation(&mut out.holder, Some((pen, DVec3::new(px, py, holder_z))));
            }
        }

        // Evaluate cutter cutting envelope
        if let Some(hit) = prim.eval_cutter_at_xy(px, py, prof, is_flat_endmill) {
            let z_cut = hit.z_cut_mm;

            // Rule 2: Target Model Gouge (checked inside radial skin to avoid tangential vertical-wall ties)
            if has_target && hit.r_mm <= r_inner_skin {
                let gouge = f64::from(zt) - z_cut;
                if gouge > cfg.gouge_tol_mm {
                    merge_violation(&mut out.gouge, Some((gouge, DVec3::new(px, py, z_cut))));
                }
            }

            if has_stock {
                let stock_z_f64 = f64::from(zs_before);
                let cut_depth = stock_z_f64 - z_cut;

                // Rule 1: Rapid-into-Stock Crash
                if is_rapid && hit.r_mm <= r_inner_skin && cut_depth > cfg.rapid_crash_tol_mm {
                    merge_violation(&mut out.rapid, Some((cut_depth, DVec3::new(px, py, z_cut))));
                }

                // Rule 3: Flute Length Exceeded (Shank Rub)
                if hit.r_mm <= r_inner_skin.max(shank_skin) && cut_depth > 0.02 {
                    let engagement = stock_z_f64 - hit.z_tip_mm;
                    let excess_flute = engagement - prof.flute_length_mm;
                    if excess_flute > 0.05 {
                        merge_violation(
                            &mut out.flute,
                            Some((excess_flute, DVec3::new(px, py, hit.z_tip_mm))),
                        );
                    }
                }

                // Mutate remaining heightmap stock!
                let z_cut_f32 = z_cut as f32;
                if z_cut_f32 < zs_before {
                    z_stock[i] = if z_cut_f32 <= z_bottom[i] {
                        f32::NEG_INFINITY
                    } else {
                        z_cut_f32
                    };
                    last_op[i] = op_idx;
                }
            }
        } else if has_stock && prof.shank_radius_mm > prof.cutting_radius_mm + 1e-4 {
            // Reduced-neck / small cutter with wider shank (`cutting_r < r <= shank_skin`)
            if let Some((r_axis, z_tip_min)) = prim.min_tip_z_within_radius(px, py, shank_skin)
                && r_axis <= shank_skin
            {
                let shank_bottom_z = z_tip_min + prof.flute_length_mm;
                let rub = f64::from(zs_before) - shank_bottom_z;
                if rub > 0.05 {
                    merge_violation(
                        &mut out.flute,
                        Some((rub, DVec3::new(px, py, shank_bottom_z))),
                    );
                }
            }
        }
    }

    out
}

#[derive(Clone, Copy)]
struct CutterCellHit {
    z_cut_mm: f64,
    z_tip_mm: f64,
    r_mm: f64,
}

#[derive(Clone, Copy)]
enum SweepPrimitive {
    Line3D {
        start: DVec3,
        end: DVec3,
    },
    PlanarArcXY {
        start: DVec3,
        end: DVec3,
        center_xy: DVec2,
        radius_mm: f64,
        a0_rad: f64,
        signed_sweep_rad: f64,
    },
}

impl SweepPrimitive {
    fn min_tip_z(&self) -> f64 {
        match *self {
            Self::Line3D { start, end } => start.z.min(end.z),
            Self::PlanarArcXY { start, .. } => start.z,
        }
    }

    fn xy_bounds(&self, expand_r: f64) -> Aabb2 {
        match *self {
            Self::Line3D { start, end } => Aabb2 {
                min: DVec2::new(start.x.min(end.x) - expand_r, start.y.min(end.y) - expand_r),
                max: DVec2::new(start.x.max(end.x) + expand_r, start.y.max(end.y) + expand_r),
            },
            Self::PlanarArcXY {
                center_xy,
                radius_mm,
                ..
            } => {
                let r_tot = radius_mm + expand_r;
                Aabb2 {
                    min: DVec2::new(center_xy.x - r_tot, center_xy.y - r_tot),
                    max: DVec2::new(center_xy.x + r_tot, center_xy.y + r_tot),
                }
            }
        }
    }

    fn min_tip_z_within_radius(&self, px: f64, py: f64, r_max: f64) -> Option<(f64, f64)> {
        if r_max <= 0.0 {
            return None;
        }
        match *self {
            Self::Line3D { start, end } => {
                let dx = end.x - start.x;
                let dy = end.y - start.y;
                let l2 = dx * dx + dy * dy;
                if l2 <= 1e-12 {
                    let r = (px - start.x).hypot(py - start.y);
                    return (r <= r_max).then(|| (r, start.z.min(end.z)));
                }
                let tc = ((px - start.x) * dx + (py - start.y) * dy) / l2;
                let qx = start.x + tc * dx;
                let qy = start.y + tc * dy;
                let d_perp_sq = (px - qx).powi(2) + (py - qy).powi(2);
                let r2 = r_max * r_max;
                if d_perp_sq > r2 {
                    return None;
                }
                let dt = ((r2 - d_perp_sq) / l2).max(0.0).sqrt();
                let t_lo = (tc - dt).clamp(0.0, 1.0);
                let t_hi = (tc + dt).clamp(0.0, 1.0);
                if t_lo > t_hi {
                    return None;
                }
                let t_best = if end.z >= start.z { t_lo } else { t_hi };
                let t_clamped = tc.clamp(0.0, 1.0);
                let r_min =
                    (px - (start.x + t_clamped * dx)).hypot(py - (start.y + t_clamped * dy));
                let z_min = start.z + t_best * (end.z - start.z);
                Some((r_min, z_min))
            }
            Self::PlanarArcXY {
                start,
                end,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
            } => {
                let r = planar_arc_xy_distance(
                    px,
                    py,
                    start,
                    end,
                    center_xy,
                    radius_mm,
                    a0_rad,
                    signed_sweep_rad,
                );
                (r <= r_max).then_some((r, start.z))
            }
        }
    }

    fn eval_cutter_at_xy(
        &self,
        px: f64,
        py: f64,
        prof: &RadialCutterProfile,
        is_flat_endmill: bool,
    ) -> Option<CutterCellHit> {
        let r_max = prof.cutting_radius_mm;
        match *self {
            Self::Line3D { start, end } => {
                let dx = end.x - start.x;
                let dy = end.y - start.y;
                let dz = end.z - start.z;
                let l2 = dx * dx + dy * dy;

                if l2 <= 1e-12 {
                    // Pure vertical move
                    let r = (px - start.x).hypot(py - start.y);
                    let p_dz = prof.tip_to_envelope_dz(r)?;
                    let z_tip = start.z.min(end.z);
                    return Some(CutterCellHit {
                        z_cut_mm: z_tip + p_dz,
                        z_tip_mm: z_tip,
                        r_mm: r,
                    });
                }

                if dz.abs() <= 1e-8 {
                    // Horizontal 2D linear move (exact closed-form)
                    let t = (((px - start.x) * dx + (py - start.y) * dy) / l2).clamp(0.0, 1.0);
                    let qx = start.x + t * dx;
                    let qy = start.y + t * dy;
                    let r = (px - qx).hypot(py - qy);
                    let p_dz = prof.tip_to_envelope_dz(r)?;
                    return Some(CutterCellHit {
                        z_cut_mm: start.z + p_dz,
                        z_tip_mm: start.z,
                        r_mm: r,
                    });
                }

                // Sloped 3D linear move (exact `[t_lo, t_hi]` interval inside cylinder of radius `r_max`)
                let tc = ((px - start.x) * dx + (py - start.y) * dy) / l2;
                let qx_inf = start.x + tc * dx;
                let qy_inf = start.y + tc * dy;
                let d_perp_sq = (px - qx_inf).powi(2) + (py - qy_inf).powi(2);
                let r2 = r_max * r_max;
                if d_perp_sq > r2 {
                    return None;
                }
                let dt = ((r2 - d_perp_sq) / l2).max(0.0).sqrt();
                let t_lo = (tc - dt).clamp(0.0, 1.0);
                let t_hi = (tc + dt).clamp(0.0, 1.0);
                if t_lo > t_hi {
                    return None;
                }

                // Minimum radial distance on `[t_lo, t_hi]` occurs at `tc.clamp(t_lo, t_hi)`
                let t_min_r = tc.clamp(t_lo, t_hi);
                let min_r = (px - (start.x + t_min_r * dx)).hypot(py - (start.y + t_min_r * dy));

                if is_flat_endmill {
                    // Exact analytical minimum for flat cylinder!
                    let t_opt = if dz >= 0.0 { t_lo } else { t_hi };
                    let z_tip = start.z + t_opt * dz;
                    return Some(CutterCellHit {
                        z_cut_mm: z_tip,
                        z_tip_mm: z_tip,
                        r_mm: min_r,
                    });
                }

                // Shaped cutter on sloped 3D ramp: sample `[t_lo, t_hi]` + `t_min_r`
                let mut best: Option<CutterCellHit> = None;
                let samples = 6usize;
                for s in 0..=samples {
                    let t = t_lo + (t_hi - t_lo) * (s as f64 / samples as f64);
                    let r = (px - (start.x + t * dx)).hypot(py - (start.y + t * dy));
                    if let Some(p_dz) = prof.tip_to_envelope_dz(r) {
                        let z_tip = start.z + t * dz;
                        let z_cut = z_tip + p_dz;
                        if best.is_none_or(|b| z_cut < b.z_cut_mm) {
                            best = Some(CutterCellHit {
                                z_cut_mm: z_cut,
                                z_tip_mm: z_tip,
                                r_mm: min_r.min(r),
                            });
                        }
                    }
                }
                {
                    let r = min_r;
                    if let Some(p_dz) = prof.tip_to_envelope_dz(r) {
                        let z_tip = start.z + t_min_r * dz;
                        let z_cut = z_tip + p_dz;
                        if best.is_none_or(|b| z_cut < b.z_cut_mm) {
                            best = Some(CutterCellHit {
                                z_cut_mm: z_cut,
                                z_tip_mm: z_tip,
                                r_mm: r,
                            });
                        }
                    }
                }
                best
            }
            Self::PlanarArcXY {
                start,
                end,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
            } => {
                let r = planar_arc_xy_distance(
                    px,
                    py,
                    start,
                    end,
                    center_xy,
                    radius_mm,
                    a0_rad,
                    signed_sweep_rad,
                );
                let p_dz = prof.tip_to_envelope_dz(r)?;
                Some(CutterCellHit {
                    z_cut_mm: start.z + p_dz,
                    z_tip_mm: start.z,
                    r_mm: r,
                })
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
#[inline]
fn planar_arc_xy_distance(
    px: f64,
    py: f64,
    start: DVec3,
    end: DVec3,
    center_xy: DVec2,
    radius_mm: f64,
    a0_rad: f64,
    signed_sweep_rad: f64,
) -> f64 {
    let sweep_abs = signed_sweep_rad.abs();
    let dx = px - center_xy.x;
    let dy = py - center_xy.y;
    let dist_c = dx.hypot(dy);

    if sweep_abs >= std::f64::consts::TAU - 1e-6 {
        return (dist_c - radius_mm).abs();
    }
    if dist_c > 1e-9 {
        let ang = dy.atan2(dx);
        let rel = if signed_sweep_rad >= 0.0 {
            (ang - a0_rad).rem_euclid(std::f64::consts::TAU)
        } else {
            (a0_rad - ang).rem_euclid(std::f64::consts::TAU)
        };
        if rel <= sweep_abs {
            return (dist_c - radius_mm).abs();
        }
    }
    let d0 = (px - start.x).hypot(py - start.y);
    let d1 = (px - end.x).hypot(py - end.y);
    d0.min(d1)
}

fn decompose_segment_primitives(seg: &ToolpathSegment3D) -> Vec<SweepPrimitive> {
    match seg.motion {
        SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => vec![SweepPrimitive::Line3D {
            start: seg.start_wcs,
            end: seg.end_wcs,
        }],
        SegmentMotion3D::Arc {
            plane,
            center_wcs,
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            if plane == ArcPlane::XY && (seg.end_wcs.z - seg.start_wcs.z).abs() <= 1e-7 {
                let c2 = DVec2::new(center_wcs.x, center_wcs.y);
                let a0 = (seg.start_wcs.y - c2.y).atan2(seg.start_wcs.x - c2.x);
                vec![SweepPrimitive::PlanarArcXY {
                    start: seg.start_wcs,
                    end: seg.end_wcs,
                    center_xy: c2,
                    radius_mm,
                    a0_rad: a0,
                    signed_sweep_rad,
                }]
            } else {
                // Helical XY arc or vertical G18/G19 lead arc: subdivide at <= 3° (`0.05 rad`) so sagitta < 0.002 mm
                let steps = ((signed_sweep_rad.abs() / 0.05).ceil() as usize).clamp(4, 2_048);
                let mut out = Vec::with_capacity(steps);
                let mut prev = seg.start_wcs;
                for k in 1..=steps {
                    let next = seg.sample_point(k as f64 / steps as f64);
                    out.push(SweepPrimitive::Line3D {
                        start: prev,
                        end: next,
                    });
                    prev = next;
                }
                out
            }
        }
    }
}

fn slice_segment_prefix(seg: &ToolpathSegment3D, u: f64) -> Option<ToolpathSegment3D> {
    let u_clamped = u.clamp(0.0, 1.0);
    if u_clamped <= 1e-6 {
        return None;
    }
    let end_wcs = seg.sample_point(u_clamped);
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
            signed_sweep_rad: signed_sweep_rad * u_clamped,
            feed_mm_min,
            role,
        },
    };
    Some(ToolpathSegment3D {
        start_wcs: seg.start_wcs,
        end_wcs,
        motion,
    })
}

struct SegmentRuleHit {
    kind: SimIssueKind,
    span_idx: usize,
    op_idx: usize,
    op_id: String,
    op_name: String,
    tool_number: u32,
    time_sec: f64,
    location_wcs: DVec3,
    excess_mm: f64,
    detail: String,
}

/// Executes full heightmap stock removal, periodic checkpoint capture, and 5-rule crash/gouge
/// verification across all active Setup operations (`06-simulation.md`).
#[allow(clippy::too_many_arguments)]
pub fn run_setup_simulation(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    target_bodies: &[BodyId],
    ops: &[SimOperationInput<'_>],
    resolution: SimResolution,
    kinematics: MachineKinematicsConfig,
    verify_cfg: SimVerificationConfig,
    cancel: Option<&AtomicBool>,
) -> Result<SetupSimulationResult, SimError> {
    let check_cancel = || -> Result<(), SimError> {
        if let Some(flag) = cancel
            && flag.load(Ordering::Relaxed)
        {
            return Err(SimError::Cancelled);
        }
        Ok(())
    };

    check_cancel()?;
    let timeline = SimulationTimeline::from_operations(ops, kinematics);
    let mut stock = HeightfieldStock::from_setup_stock(&setup.stock, resolution)?;
    let initial_stock_volume_mm3 = stock.compute_volume_mm3();

    check_cancel()?;
    stock.rasterize_target_brep(doc, setup, target_bodies);

    // Save initial checkpoint (`completed_segments = 0`) plus bounded periodic checkpoints (`<= 24` total)
    let total_segs = timeline.motion_spans.len();
    let checkpoint_stride = (total_segs / 20).max(128);
    let mut checkpoints = Vec::new();
    checkpoints.push(StockCheckpoint {
        completed_segments: 0,
        z_stock: stock.z_stock.clone(),
        last_op_idx: stock.last_op_idx.clone(),
    });

    let profiles: Vec<(RadialCutterProfile, bool)> = ops
        .iter()
        .map(|o| {
            (
                RadialCutterProfile::from_tool(o.tool),
                matches!(o.tool.geometry, CutterGeometry::FlatEndmill { .. }),
            )
        })
        .collect();

    let mut raw_hits: Vec<SegmentRuleHit> = Vec::new();

    for (idx, span) in timeline.motion_spans.iter().enumerate() {
        if idx % 32 == 0 {
            check_cancel()?;
        }
        let op_in = &ops[span.op_idx];
        let (prof, is_flat) = profiles[span.op_idx];
        let mut op_verify = verify_cfg;
        op_verify.gouge_tol_mm += op_in.intentional_edge_break_mm.max(0.0);

        stock.sweep_span_with_verification(
            span,
            op_in.op_id,
            op_in.op_name,
            op_in.tool.tool_number,
            &prof,
            is_flat,
            &op_verify,
            &mut raw_hits,
        );

        let completed = idx + 1;
        let is_op_end = timeline.operations[span.op_idx].end_global_seg == completed;
        if (completed % checkpoint_stride == 0 || is_op_end) && completed < total_segs {
            checkpoints.push(StockCheckpoint {
                completed_segments: completed,
                z_stock: stock.z_stock.clone(),
                last_op_idx: stock.last_op_idx.clone(),
            });
        }
    }

    let remaining_stock_volume_mm3 = stock.compute_volume_mm3();
    let removed_volume_mm3 = (initial_stock_volume_mm3 - remaining_stock_volume_mm3).max(0.0);

    // Coalesce consecutive segment hits of the same `(op_idx, kind)` so each physical crash/gouge
    // produces a crisp clickable diagnostic pointing to its onset while reporting peak depth!
    let diagnostics = coalesce_rule_hits(raw_hits);

    Ok(SetupSimulationResult {
        resolution,
        timeline,
        initial_stock_volume_mm3,
        remaining_stock_volume_mm3,
        removed_volume_mm3,
        final_stock: stock,
        checkpoints,
        diagnostics,
    })
}

fn coalesce_rule_hits(hits: Vec<SegmentRuleHit>) -> Vec<SimDiagnostic> {
    let mut out: Vec<SimDiagnostic> = Vec::new();
    let mut last_seg_by_diag: Vec<usize> = Vec::new();

    for hit in hits {
        // Merge into an existing diagnostic of the same `(op_idx, kind)` if within 16 segments (same pass)
        // or if we already emitted >= 3 diagnostics of that `(op_idx, kind)`!
        let mut merged = false;
        let existing_count = out
            .iter()
            .filter(|d| d.op_idx == hit.op_idx && d.kind == hit.kind)
            .count();

        for (di, diag) in out.iter_mut().enumerate().rev() {
            if diag.op_idx == hit.op_idx
                && diag.kind == hit.kind
                && (hit.span_idx <= last_seg_by_diag[di] + 16 || existing_count >= 3)
            {
                last_seg_by_diag[di] = hit.span_idx;
                if hit.excess_mm > diag.max_excess_mm {
                    diag.max_excess_mm = hit.excess_mm;
                    diag.location_wcs = hit.location_wcs;
                    diag.message = hit.detail.clone();
                }
                merged = true;
                break;
            }
        }

        if !merged {
            last_seg_by_diag.push(hit.span_idx);
            out.push(SimDiagnostic {
                severity: hit.kind.default_severity(),
                kind: hit.kind,
                op_idx: hit.op_idx,
                op_id: hit.op_id,
                op_name: hit.op_name,
                tool_number: hit.tool_number,
                global_seg_idx: hit.span_idx,
                timeline_time_sec: hit.time_sec,
                location_wcs: hit.location_wcs,
                max_excess_mm: hit.excess_mm,
                message: hit.detail,
            });
        }
    }
    out
}

/// Distinct operation color palette for **Color Stock by Operation / Tool** (`06-simulation.md` §3).
pub const OPERATION_STOCK_PALETTE: [[f32; 4]; 8] = [
    [0.34, 0.66, 0.94, 1.0], // Azure (Op 1 - e.g. Facing)
    [0.24, 0.80, 0.62, 1.0], // Mint Emerald (Op 2 - Pocket)
    [0.92, 0.64, 0.26, 1.0], // Warm Amber (Op 3 - Step)
    [0.72, 0.48, 0.94, 1.0], // Violet (Op 4 - Bore)
    [0.94, 0.44, 0.56, 1.0], // Coral Rose (Op 5 - Drill)
    [0.28, 0.80, 0.86, 1.0], // Cyan Teal (Op 6 - Contour)
    [0.86, 0.80, 0.30, 1.0], // Citrine (Op 7 - Chamfer)
    [0.58, 0.78, 0.40, 1.0], // Lime Sage (Op 8+)
];

const RAW_STOCK_TOP_COLOR: [f32; 4] = [0.58, 0.62, 0.67, 1.0];
const RAW_STOCK_SIDE_COLOR: [f32; 4] = [0.50, 0.54, 0.59, 1.0];
const MACHINED_METAL_COLOR: [f32; 4] = [0.76, 0.80, 0.85, 1.0];

#[inline]
fn stock_cell_color(op_idx: u16, color_by_op: bool, is_side_wall: bool) -> [f32; 4] {
    if op_idx == UNCUT_OP_IDX {
        if is_side_wall {
            RAW_STOCK_SIDE_COLOR
        } else {
            RAW_STOCK_TOP_COLOR
        }
    } else if color_by_op {
        OPERATION_STOCK_PALETTE[(op_idx as usize) % OPERATION_STOCK_PALETTE.len()]
    } else {
        MACHINED_METAL_COLOR
    }
}

/// Triangulates a [`HeightfieldStock`] into a crisp-walled 3D solid mesh with greedy X-row quad
/// merging on flat regions, sharp vertical step risers on pocket/boss/tab walls, and outer side walls.
pub fn tessellate_stock_mesh(stock: &HeightfieldStock, color_by_op: bool) -> SimStockMesh {
    const MAX_VISUAL_DIM: usize = 512;
    let stride = (stock.nx.max(stock.ny)).div_ceil(MAX_VISUAL_DIM).max(1);
    let vx = stock.nx / stride;
    let vy = stock.ny / stride;
    if vx == 0 || vy == 0 {
        return SimStockMesh::default();
    }

    let vdx = (stock.nx as f64 * stock.dx_mm) / (vx as f64);
    let vdy = (stock.ny as f64 * stock.dy_mm) / (vy as f64);

    // Downsample (stride == 1 is a direct copy of active cell heights)
    let vtotal = vx * vy;
    let mut v_active = vec![false; vtotal];
    let mut v_ztop = vec![f32::NEG_INFINITY; vtotal];
    let mut v_zbot = vec![f32::NEG_INFINITY; vtotal];
    let mut v_op = vec![UNCUT_OP_IDX; vtotal];

    for vj in 0..vy {
        let sj = (vj * stride + stride / 2).min(stock.ny - 1);
        for vi in 0..vx {
            let si = (vi * stride + stride / 2).min(stock.nx - 1);
            let sidx = sj * stock.nx + si;
            let vidx = vj * vx + vi;
            let zt = stock.z_stock[sidx];
            let zb = stock.z_bottom[sidx];
            if stock.in_stock[sidx] && zt > zb + 1e-4 {
                v_active[vidx] = true;
                v_ztop[vidx] = zt;
                v_zbot[vidx] = zb;
                v_op[vidx] = stock.last_op_idx[sidx];
            }
        }
    }

    let mut mesh = SimStockMesh {
        vertices: Vec::with_capacity(vtotal / 2),
        indices: Vec::with_capacity(vtotal),
    };

    let push_quad = |mesh: &mut SimStockMesh,
                     p0: [f32; 3],
                     p1: [f32; 3],
                     p2: [f32; 3],
                     p3: [f32; 3],
                     normal: [f32; 3],
                     color: [f32; 4]| {
        let base = mesh.vertices.len() as u32;
        mesh.vertices.extend_from_slice(&[
            SimColoredVertex {
                pos_mm: p0,
                normal,
                color_rgba: color,
            },
            SimColoredVertex {
                pos_mm: p1,
                normal,
                color_rgba: color,
            },
            SimColoredVertex {
                pos_mm: p2,
                normal,
                color_rgba: color,
            },
            SimColoredVertex {
                pos_mm: p3,
                normal,
                color_rgba: color,
            },
        ]);
        mesh.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    };

    // 1. Greedy X-run merged top faces + smooth sloped normals on chamfers/ball grooves
    let slope_max_dz = (1.6 * vdx.max(vdy)) as f32;
    for vj in 0..vy {
        let y0 = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        let y1 = (stock.origin_y_mm + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;

        let mut vi = 0usize;
        while vi < vx {
            let idx = row + vi;
            if !v_active[idx] {
                vi += 1;
                continue;
            }
            let z0 = v_ztop[idx];
            let op0 = v_op[idx];

            // Compute local finite-difference normal to detect sloped chamfer/ball surfaces vs flat floors
            let zl = if vi > 0 && v_active[idx - 1] && (v_ztop[idx - 1] - z0).abs() <= slope_max_dz
            {
                v_ztop[idx - 1]
            } else {
                z0
            };
            let zr =
                if vi + 1 < vx && v_active[idx + 1] && (v_ztop[idx + 1] - z0).abs() <= slope_max_dz
                {
                    v_ztop[idx + 1]
                } else {
                    z0
                };
            let zd =
                if vj > 0 && v_active[idx - vx] && (v_ztop[idx - vx] - z0).abs() <= slope_max_dz {
                    v_ztop[idx - vx]
                } else {
                    z0
                };
            let zu = if vj + 1 < vy
                && v_active[idx + vx]
                && (v_ztop[idx + vx] - z0).abs() <= slope_max_dz
            {
                v_ztop[idx + vx]
            } else {
                z0
            };

            let dz_dx = (zr - zl) / (2.0 * vdx as f32);
            let dz_dy = (zu - zd) / (2.0 * vdy as f32);
            let is_flat = dz_dx.abs() < 1e-3 && dz_dy.abs() < 1e-3;

            let mut run_end = vi + 1;
            if is_flat {
                while run_end < vx {
                    let r_idx = row + run_end;
                    if !v_active[r_idx]
                        || (v_ztop[r_idx] - z0).abs() > 2e-4
                        || (color_by_op && v_op[r_idx] != op0)
                        || (!color_by_op && (v_op[r_idx] == UNCUT_OP_IDX) != (op0 == UNCUT_OP_IDX))
                    {
                        break;
                    }
                    run_end += 1;
                }
            }

            let normal = if is_flat {
                [0.0, 0.0, 1.0]
            } else {
                let len = (dz_dx * dz_dx + dz_dy * dz_dy + 1.0).sqrt();
                [-dz_dx / len, -dz_dy / len, 1.0 / len]
            };

            let x0 = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
            let x1 = (stock.origin_x_mm + (run_end as f64) * vdx) as f32;
            let col = stock_cell_color(op0, color_by_op, false);
            push_quad(
                &mut mesh,
                [x0, y0, z0],
                [x1, y0, z0],
                [x1, y1, z0],
                [x0, y1, z0],
                normal,
                col,
            );

            vi = run_end;
        }
    }

    // 2. Crisp vertical step risers & outer stock/hole walls (greedy merged along Y for ±X walls, along X for ±Y walls!)
    // -X / +X vertical risers (normal `[-1, 0, 0]` and `[+1, 0, 0]`)
    for vi in 0..=vx {
        let x = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
        for vj in 0..vy {
            let y0 = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
            let y1 = (stock.origin_y_mm + ((vj + 1) as f64) * vdy) as f32;

            let left = if vi > 0 && v_active[vj * vx + (vi - 1)] {
                Some((
                    v_zbot[vj * vx + (vi - 1)],
                    v_ztop[vj * vx + (vi - 1)],
                    v_op[vj * vx + (vi - 1)],
                ))
            } else {
                None
            };
            let right = if vi < vx && v_active[vj * vx + vi] {
                Some((
                    v_zbot[vj * vx + vi],
                    v_ztop[vj * vx + vi],
                    v_op[vj * vx + vi],
                ))
            } else {
                None
            };

            match (left, right) {
                (Some((lb, lt, lop)), None) => {
                    let col = stock_cell_color(lop, color_by_op, true);
                    push_quad(
                        &mut mesh,
                        [x, y0, lb],
                        [x, y1, lb],
                        [x, y1, lt],
                        [x, y0, lt],
                        [1.0, 0.0, 0.0],
                        col,
                    );
                }
                (None, Some((rb, rt, rop))) => {
                    let col = stock_cell_color(rop, color_by_op, true);
                    push_quad(
                        &mut mesh,
                        [x, y1, rb],
                        [x, y0, rb],
                        [x, y0, rt],
                        [x, y1, rt],
                        [-1.0, 0.0, 0.0],
                        col,
                    );
                }
                (Some((lb, lt, _lop)), Some((rb, rt, rop))) => {
                    if lt > rt + 2e-4 {
                        let z_lo = rt.max(lb);
                        let col = stock_cell_color(rop, color_by_op, false);
                        push_quad(
                            &mut mesh,
                            [x, y0, z_lo],
                            [x, y1, z_lo],
                            [x, y1, lt],
                            [x, y0, lt],
                            [1.0, 0.0, 0.0],
                            col,
                        );
                    } else if rt > lt + 2e-4 {
                        let z_lo = lt.max(rb);
                        let l_op = v_op[vj * vx + (vi - 1)];
                        let col = stock_cell_color(l_op, color_by_op, false);
                        push_quad(
                            &mut mesh,
                            [x, y1, z_lo],
                            [x, y0, z_lo],
                            [x, y0, rt],
                            [x, y1, rt],
                            [-1.0, 0.0, 0.0],
                            col,
                        );
                    }
                }
                (None, None) => {}
            }
        }
    }

    // -Y / +Y vertical risers (normal `[0, -1, 0]` and `[0, +1, 0]`)
    for vj in 0..=vy {
        let y = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        for vi in 0..vx {
            let x0 = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
            let x1 = (stock.origin_x_mm + ((vi + 1) as f64) * vdx) as f32;

            let front = if vj > 0 && v_active[(vj - 1) * vx + vi] {
                Some((
                    v_zbot[(vj - 1) * vx + vi],
                    v_ztop[(vj - 1) * vx + vi],
                    v_op[(vj - 1) * vx + vi],
                ))
            } else {
                None
            };
            let back = if vj < vy && v_active[vj * vx + vi] {
                Some((
                    v_zbot[vj * vx + vi],
                    v_ztop[vj * vx + vi],
                    v_op[vj * vx + vi],
                ))
            } else {
                None
            };

            match (front, back) {
                (Some((fb, ft, fop)), None) => {
                    let col = stock_cell_color(fop, color_by_op, true);
                    push_quad(
                        &mut mesh,
                        [x1, y, fb],
                        [x0, y, fb],
                        [x0, y, ft],
                        [x1, y, ft],
                        [0.0, 1.0, 0.0],
                        col,
                    );
                }
                (None, Some((bb, bt, bop))) => {
                    let col = stock_cell_color(bop, color_by_op, true);
                    push_quad(
                        &mut mesh,
                        [x0, y, bb],
                        [x1, y, bb],
                        [x1, y, bt],
                        [x0, y, bt],
                        [0.0, -1.0, 0.0],
                        col,
                    );
                }
                (Some((fb, ft, _fop)), Some((bb, bt, bop))) => {
                    if ft > bt + 2e-4 {
                        let z_lo = bt.max(fb);
                        let col = stock_cell_color(bop, color_by_op, false);
                        push_quad(
                            &mut mesh,
                            [x1, y, z_lo],
                            [x0, y, z_lo],
                            [x0, y, ft],
                            [x1, y, ft],
                            [0.0, 1.0, 0.0],
                            col,
                        );
                    } else if bt > ft + 2e-4 {
                        let z_lo = ft.max(bb);
                        let f_op = v_op[(vj - 1) * vx + vi];
                        let col = stock_cell_color(f_op, color_by_op, false);
                        push_quad(
                            &mut mesh,
                            [x0, y, z_lo],
                            [x1, y, z_lo],
                            [x1, y, bt],
                            [x0, y, bt],
                            [0.0, -1.0, 0.0],
                            col,
                        );
                    }
                }
                (None, None) => {}
            }
        }
    }

    // 3. Greedy X-run merged bottom faces (`normal = [0, 0, -1]`) so stock mesh is a closed solid from below
    for vj in 0..vy {
        let y0 = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        let y1 = (stock.origin_y_mm + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;
        let mut vi = 0usize;
        while vi < vx {
            let idx = row + vi;
            if !v_active[idx] {
                vi += 1;
                continue;
            }
            let zb0 = v_zbot[idx];
            let mut run_end = vi + 1;
            while run_end < vx
                && v_active[row + run_end]
                && (v_zbot[row + run_end] - zb0).abs() <= 2e-4
            {
                run_end += 1;
            }
            let x0 = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
            let x1 = (stock.origin_x_mm + (run_end as f64) * vdx) as f32;
            push_quad(
                &mut mesh,
                [x0, y1, zb0],
                [x1, y1, zb0],
                [x1, y0, zb0],
                [x0, y0, zb0],
                [0.0, 0.0, -1.0],
                RAW_STOCK_SIDE_COLOR,
            );
            vi = run_end;
        }
    }

    let _ = Aabb3::EMPTY;
    mesh
}
