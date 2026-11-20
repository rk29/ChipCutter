use std::sync::atomic::Ordering;

use chip_cam::{ArcPlane, SegmentMotion3D, ToolpathSegment3D};
use chip_math::{Aabb2, DVec2, DVec3};
use chip_tools::CutterGeometry;
use rayon::prelude::*;

use crate::{
    cutter_profile::RadialCutterProfile,
    timeline::{MotionSpan, SimulationTimeline},
    verifier::{SimDiagnostic, SimIssueKind, SimVerificationConfig},
};

use super::{
    HeightfieldStock, SetupSimulationInput, SetupSimulationResult, SimError, StockCheckpoint,
    target_rasterizer::effective_verify_config_for_op,
};

#[derive(Clone, Copy)]
struct SegmentSweepMeta<'a> {
    op_id: &'a str,
    op_name: &'a str,
    tool_number: u32,
    prof: &'a RadialCutterProfile,
    is_flat_endmill: bool,
    cfg: &'a SimVerificationConfig,
}

#[derive(Clone, Copy)]
struct SweepEnv<'a> {
    i0: usize,
    i1: usize,
    ox: f64,
    dx: f64,
    prim: &'a SweepPrimitive,
    prof: &'a RadialCutterProfile,
    is_flat_endmill: bool,
    is_rapid: bool,
    check_holder: bool,
    r_inner_skin: f64,
    cfg: &'a SimVerificationConfig,
    op_idx: u16,
}

struct RowCellsMut<'a> {
    in_stock: &'a [bool],
    z_bottom: &'a [f32],
    z_target: &'a [f32],
    z_stock: &'a mut [f32],
    last_op: &'a mut [u16],
}

impl HeightfieldStock {
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
    fn sweep_span_with_verification(
        &mut self,
        span: &MotionSpan,
        meta: &SegmentSweepMeta<'_>,
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
                    if !meta.prof.is_center_cutting {
                        out_hits.push(SegmentRuleHit {
                            kind: SimIssueKind::UnsafeVerticalPlunge,
                            span_idx: span.global_seg_idx,
                            op_idx: span.op_idx,
                            op_id: meta.op_id.to_string(),
                            op_name: meta.op_name.to_string(),
                            tool_number: meta.tool_number,
                            time_sec: span.t_start_sec,
                            location_wcs: DVec3::new(seg.end_wcs.x, seg.end_wcs.y, seg.end_wcs.z),
                            excess_mm: plunge_in_mat,
                            detail: format!(
                                "Non-center-cutting tool T{} plunged vertically {plunge_in_mat:.2} mm into stock (use Helical or Linear ramp).",
                                meta.tool_number
                            ),
                        });
                    } else if plunge_in_mat > meta.prof.max_safe_plunge_depth_mm + 0.05 {
                        let excess = plunge_in_mat - meta.prof.max_safe_plunge_depth_mm;
                        out_hits.push(SegmentRuleHit {
                            kind: SimIssueKind::UnsafeVerticalPlunge,
                            span_idx: span.global_seg_idx,
                            op_idx: span.op_idx,
                            op_id: meta.op_id.to_string(),
                            op_name: meta.op_name.to_string(),
                            tool_number: meta.tool_number,
                            time_sec: span.t_start_sec,
                            location_wcs: DVec3::new(seg.end_wcs.x, seg.end_wcs.y, seg.end_wcs.z),
                            excess_mm: excess,
                            detail: format!(
                                "Vertical plunge depth ({plunge_in_mat:.2} mm) exceeds safe axial plunge threshold ({:.2} mm) on T{}.",
                                meta.prof.max_safe_plunge_depth_mm,
                                meta.tool_number
                            ),
                        });
                    }
                }
            }
        }

        let primitives = decompose_segment_primitives(seg);
        let r_inner_skin = (meta.prof.cutting_radius_mm - meta.cfg.radial_wall_skin_mm).max(0.0);

        let mut worst_rapid: Option<(f64, DVec3)> = None;
        let mut worst_gouge: Option<(f64, DVec3)> = None;
        let mut worst_flute: Option<(f64, DVec3)> = None;
        let mut worst_holder: Option<(f64, DVec3)> = None;

        for prim in &primitives {
            let check_holder =
                prim.min_tip_z() + meta.prof.stickout_length_mm < self.max_initial_top_z_mm - 0.02;
            let sweep_r = if check_holder {
                meta.prof.max_assembly_radius_mm()
            } else {
                meta.prof.cutting_radius_mm.max(meta.prof.shank_radius_mm)
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

            let env = SweepEnv {
                i0,
                i1,
                ox,
                dx,
                prim,
                prof: meta.prof,
                is_flat_endmill: meta.is_flat_endmill,
                is_rapid,
                check_holder,
                r_inner_skin,
                cfg: meta.cfg,
                op_idx: op_u16,
            };

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
                            py,
                            RowCellsMut {
                                in_stock: &in_stock_ref[row_base..row_base + nx],
                                z_bottom: &z_bot_ref[row_base..row_base + nx],
                                z_target: &z_tgt_ref[row_base..row_base + nx],
                                z_stock: row_zs,
                                last_op: row_op,
                            },
                            &env,
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
                        py,
                        RowCellsMut {
                            in_stock: &self.in_stock[base..base + nx],
                            z_bottom: &self.z_bottom[base..base + nx],
                            z_target: &self.z_target[base..base + nx],
                            z_stock: &mut self.z_stock[base..base + nx],
                            last_op: &mut self.last_op_idx[base..base + nx],
                        },
                        &env,
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
                op_id: meta.op_id.to_string(),
                op_name: meta.op_name.to_string(),
                tool_number: meta.tool_number,
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
                op_id: meta.op_id.to_string(),
                op_name: meta.op_name.to_string(),
                tool_number: meta.tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Tool T{} gouged target model by {depth:.3} mm at WCS ({:.2}, {:.2}, {:.2}).",
                    meta.tool_number, loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((depth, loc)) = worst_flute {
            out_hits.push(SegmentRuleHit {
                kind: SimIssueKind::FluteLengthExceeded,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: meta.op_id.to_string(),
                op_name: meta.op_name.to_string(),
                tool_number: meta.tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Engagement depth exceeded T{} flute length ({:.2} mm) by {depth:.2} mm (shank rub).",
                    meta.tool_number, meta.prof.flute_length_mm
                ),
            });
        }
        if let Some((depth, loc)) = worst_holder {
            out_hits.push(SegmentRuleHit {
                kind: SimIssueKind::HolderCollision,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: meta.op_id.to_string(),
                op_name: meta.op_name.to_string(),
                tool_number: meta.tool_number,
                time_sec: span.t_start_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Collet/holder of T{} collided {depth:.2} mm into stock (stickout {:.2} mm insufficient).",
                    meta.tool_number, meta.prof.stickout_length_mm
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

fn eval_row_cells(py: f64, row: RowCellsMut<'_>, env: &SweepEnv<'_>) -> RowViolationSummary {
    let mut out = RowViolationSummary::default();
    let shank_skin = (env.prof.shank_radius_mm - env.cfg.radial_wall_skin_mm).max(0.0);
    let holder_skin = (env.prof.holder_radius_mm - env.cfg.radial_wall_skin_mm).max(0.0);

    for i in env.i0..=env.i1 {
        let px = env.ox + (i as f64 + 0.5) * env.dx;
        let zs_before = row.z_stock[i];
        let has_stock = row.in_stock[i] && zs_before > row.z_bottom[i];
        let zt = row.z_target[i];
        let has_target = zt.is_finite();

        if !has_stock && !has_target {
            continue;
        }

        // Check holder collision when deep enough (`r <= holder_skin`)
        if env.check_holder
            && has_stock
            && let Some((r_axis, z_tip_min)) = env.prim.min_tip_z_within_radius(px, py, holder_skin)
            && r_axis <= holder_skin
        {
            let holder_z = z_tip_min + env.prof.stickout_length_mm;
            let pen = f64::from(zs_before) - holder_z;
            if pen > 0.03 {
                merge_violation(&mut out.holder, Some((pen, DVec3::new(px, py, holder_z))));
            }
        }

        // Evaluate cutter cutting envelope
        if let Some(hit) = env
            .prim
            .eval_cutter_at_xy(px, py, env.prof, env.is_flat_endmill)
        {
            let z_cut = hit.z_cut_mm;

            // Rule 2: Target Model Gouge (checked inside radial skin to avoid tangential vertical-wall ties)
            if has_target && hit.r_mm <= env.r_inner_skin {
                let gouge = f64::from(zt) - z_cut;
                if gouge > env.cfg.gouge_tol_mm {
                    merge_violation(&mut out.gouge, Some((gouge, DVec3::new(px, py, z_cut))));
                }
            }

            if has_stock {
                let stock_z_f64 = f64::from(zs_before);
                let cut_depth = stock_z_f64 - z_cut;

                // Rule 1: Rapid-into-Stock Crash
                if env.is_rapid
                    && hit.r_mm <= env.r_inner_skin
                    && cut_depth > env.cfg.rapid_crash_tol_mm
                {
                    merge_violation(&mut out.rapid, Some((cut_depth, DVec3::new(px, py, z_cut))));
                }

                // Rule 3: Flute Length Exceeded (Shank Rub)
                if hit.r_mm <= env.r_inner_skin.max(shank_skin) && cut_depth > 0.02 {
                    let engagement = stock_z_f64 - hit.z_tip_mm;
                    let excess_flute = engagement - env.prof.flute_length_mm;
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
                    row.z_stock[i] = if z_cut_f32 <= row.z_bottom[i] {
                        f32::NEG_INFINITY
                    } else {
                        z_cut_f32
                    };
                    row.last_op[i] = env.op_idx;
                }
            }
        } else if has_stock && env.prof.shank_radius_mm > env.prof.cutting_radius_mm + 1e-4 {
            // Reduced-neck / small cutter with wider shank (`cutting_r < r <= shank_skin`)
            if let Some((r_axis, z_tip_min)) = env.prim.min_tip_z_within_radius(px, py, shank_skin)
                && r_axis <= shank_skin
            {
                let shank_bottom_z = z_tip_min + env.prof.flute_length_mm;
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
struct PlanarArcGeom {
    start: DVec3,
    end: DVec3,
    center_xy: DVec2,
    radius_mm: f64,
    a0_rad: f64,
    signed_sweep_rad: f64,
}

#[derive(Clone, Copy)]
enum SweepPrimitive {
    Line3D { start: DVec3, end: DVec3 },
    PlanarArcXY(PlanarArcGeom),
}

impl SweepPrimitive {
    fn min_tip_z(&self) -> f64 {
        match *self {
            Self::Line3D { start, end } => start.z.min(end.z),
            Self::PlanarArcXY(arc) => arc.start.z,
        }
    }

    fn xy_bounds(&self, expand_r: f64) -> Aabb2 {
        match *self {
            Self::Line3D { start, end } => Aabb2 {
                min: DVec2::new(start.x.min(end.x) - expand_r, start.y.min(end.y) - expand_r),
                max: DVec2::new(start.x.max(end.x) + expand_r, start.y.max(end.y) + expand_r),
            },
            Self::PlanarArcXY(arc) => {
                let r_tot = arc.radius_mm + expand_r;
                Aabb2 {
                    min: DVec2::new(arc.center_xy.x - r_tot, arc.center_xy.y - r_tot),
                    max: DVec2::new(arc.center_xy.x + r_tot, arc.center_xy.y + r_tot),
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
            Self::PlanarArcXY(arc) => {
                let r = planar_arc_xy_distance(px, py, &arc);
                (r <= r_max).then_some((r, arc.start.z))
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

                // Shaped cutter on sloped 3D ramp: evaluate `[t_lo, t_hi]`, `t_min_r`, AND exact
                // closed-form stationary point `t*` for conical V-bits/chamfer cutters!
                let mut best: Option<CutterCellHit> = None;
                let mut consider_t = |t_cand: f64| {
                    let t = t_cand.clamp(t_lo, t_hi);
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
                };

                let samples = 6usize;
                for s in 0..=samples {
                    consider_t(t_lo + (t_hi - t_lo) * (s as f64 / samples as f64));
                }
                consider_t(t_min_r);

                if let Some(k_cot) = prof.conical_slope_cot_theta() {
                    let denom_sq = k_cot * k_cot * l2 - dz * dz;
                    if denom_sq > 1e-12 {
                        let d_perp = d_perp_sq.max(0.0).sqrt();
                        let l_xy = l2.sqrt();
                        let t_star =
                            tc - dz.signum() * dz.abs() * d_perp / (l_xy * denom_sq.sqrt());
                        consider_t(t_star);
                    }
                }
                best
            }
            Self::PlanarArcXY(arc) => {
                let r = planar_arc_xy_distance(px, py, &arc);
                let p_dz = prof.tip_to_envelope_dz(r)?;
                Some(CutterCellHit {
                    z_cut_mm: arc.start.z + p_dz,
                    z_tip_mm: arc.start.z,
                    r_mm: r,
                })
            }
        }
    }
}

#[inline]
fn planar_arc_xy_distance(px: f64, py: f64, arc: &PlanarArcGeom) -> f64 {
    let sweep_abs = arc.signed_sweep_rad.abs();
    let dx = px - arc.center_xy.x;
    let dy = py - arc.center_xy.y;
    let dist_c = dx.hypot(dy);

    if sweep_abs >= std::f64::consts::TAU - 1e-6 {
        return (dist_c - arc.radius_mm).abs();
    }
    if dist_c > 1e-9 {
        let ang = dy.atan2(dx);
        let rel = if arc.signed_sweep_rad >= 0.0 {
            (ang - arc.a0_rad).rem_euclid(std::f64::consts::TAU)
        } else {
            (arc.a0_rad - ang).rem_euclid(std::f64::consts::TAU)
        };
        if rel <= sweep_abs {
            return (dist_c - arc.radius_mm).abs();
        }
    }
    let d0 = (px - arc.start.x).hypot(py - arc.start.y);
    let d1 = (px - arc.end.x).hypot(py - arc.end.y);
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
                vec![SweepPrimitive::PlanarArcXY(PlanarArcGeom {
                    start: seg.start_wcs,
                    end: seg.end_wcs,
                    center_xy: c2,
                    radius_mm,
                    a0_rad: a0,
                    signed_sweep_rad,
                })]
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
pub fn run_setup_simulation(
    input: &SetupSimulationInput<'_>,
) -> Result<SetupSimulationResult, SimError> {
    let check_cancel = || -> Result<(), SimError> {
        if let Some(flag) = input.cancel
            && flag.load(Ordering::Relaxed)
        {
            return Err(SimError::Cancelled);
        }
        Ok(())
    };

    check_cancel()?;
    if let Some(rotary) = input.setup.rotary {
        return crate::cylindrical::run_cylindrical_setup_simulation(input, rotary);
    }
    let timeline = SimulationTimeline::from_operations(input.operations, input.kinematics);
    let mut stock = HeightfieldStock::from_setup_stock(&input.setup.stock, input.resolution)?;
    let initial_stock_volume_mm3 = stock.compute_volume_mm3();

    check_cancel()?;
    stock.rasterize_target_brep(input.doc, input.setup, input.target_bodies);

    // Save initial checkpoint (`completed_segments = 0`) plus bounded periodic checkpoints (`<= 24` total)
    let total_segs = timeline.motion_spans.len();
    let checkpoint_stride = (total_segs / 20).max(128);
    let mut checkpoints = Vec::new();
    checkpoints.push(StockCheckpoint {
        completed_segments: 0,
        z_stock: stock.z_stock.clone(),
        last_op_idx: stock.last_op_idx.clone(),
    });

    let profiles: Vec<(RadialCutterProfile, bool)> = input
        .operations
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
        let op_in = &input.operations[span.op_idx];
        let (prof, is_flat) = profiles[span.op_idx];
        let op_verify =
            effective_verify_config_for_op(input.verify_config, op_in.intentional_edge_break_mm);

        stock.sweep_span_with_verification(
            span,
            &SegmentSweepMeta {
                op_id: op_in.op_id,
                op_name: op_in.op_name,
                tool_number: op_in.tool.tool_number,
                prof: &prof,
                is_flat_endmill: is_flat,
                cfg: &op_verify,
            },
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
        resolution: input.resolution,
        timeline,
        initial_stock_volume_mm3,
        remaining_stock_volume_mm3,
        removed_volume_mm3,
        final_stock: stock,
        final_cylindrical_stock: None,
        checkpoints,
        cylindrical_checkpoints: Vec::new(),
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
