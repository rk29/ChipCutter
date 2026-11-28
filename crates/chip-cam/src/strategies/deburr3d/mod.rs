//! **3D Edge Deburring & Non-Planar Chamfering** (`OperationKind::Deburr3D`).
//!
//! Extracts exact `f64` 3D B-Rep edge chains and adjacent-face outward normals `(n1, n2)` in Setup
//! WCS, solves cutter-specific 3D contact kinematics (`ChamferMill`, `BallEndmill`, and
//! `LollipopEndmill`, including underside back-chamfering on downward-facing overhang rims),
//! trims colliding ends/sub-segments away from neighboring vertical walls, stepped bosses, and
//! inside corners using `chip-bvh` [`FlatBvh`] with binary boundary refinement (`<= 0.02 mm`),
//! and emits smooth tangent arc lead-in/lead-out transitions with verified safe vertical plunges.

mod contact;
mod emit;
mod extract;
mod trim;

use std::collections::HashMap;

use chip_cad::{EdgeId, FaceId};
use chip_math::{DVec2, DVec3};
use chip_tools::{AnalyticCutterEnvelope, CutterGeometry};

use crate::{
    cl_data::MotionRole,
    params::Deburr3DParams,
    rest2d::RegionSet2D,
    strategies::StrategyExecContext,
    surface3d::{SurfaceMesh3D, resolve_3d_boundary_config_detailed},
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

pub use contact::solve_deburr_tool_tip;

use emit::emit_deburr_subchain;
use extract::{
    FaceTriLookup, apply_open_chain_extension, auto_detect_sharp_edge_chains,
    extract_selected_edge_chains, orient_chain_for_cut_direction,
};
use trim::trim_chain_against_target;

/// Single exact `f64` 3D sample along a B-Rep edge in Setup WCS coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeburrSample3D {
    pub pos_wcs: DVec3,
    pub tangent: DVec3,
    pub n1: DVec3,
    pub n2: DVec3,
    pub bisector_out: DVec3,
    pub is_underside: bool,
    pub face1: FaceId,
    pub face2: FaceId,
}

/// Exact 3D cutter contact solution at a [`DeburrSample3D`] station.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeburrContactSolution {
    /// Programmed cutter tip position `(X, Y, Z)` in Setup WCS millimeters.
    pub tip_wcs: DVec3,
    /// Point on the intentional bevel surface in Setup WCS.
    pub bevel_point_wcs: DVec3,
    /// Unit outward normal of the bevel plane cut by the tool at this station.
    pub bevel_normal_wcs: DVec3,
    /// Horizontal unit direction pointing from the edge outward toward the cutter axis.
    pub outward_xy: DVec2,
    /// Maximum local tangent contact chord (`mm`) along the edge for bevel-wedge exemption.
    pub contact_chord_mm: f64,
}

/// Extracted continuous chain of 3D edge samples in Setup WCS.
#[derive(Debug, Clone)]
pub(super) struct DeburrEdgeChain {
    pub(super) samples: Vec<DeburrSample3D>,
    pub(super) closed: bool,
    pub(super) is_underside: bool,
    pub(super) user_reversed: bool,
}

/// Solved collision-free station ready for toolpath emission.
#[derive(Debug, Clone, Copy)]
pub(super) struct SolvedStation {
    pub(super) sample: DeburrSample3D,
    pub(super) sol: DeburrContactSolution,
}

/// Collision-free sub-chain produced by trimming a [`DeburrEdgeChain`] against neighbor walls.
pub(super) struct TrimmedSubChain {
    pub(super) stations: Vec<SolvedStation>,
    pub(super) closed: bool,
    pub(super) is_underside: bool,
}

/// Generates **3D Edge Deburring & Non-Planar Chamfering** (`OperationKind::Deburr3D`) toolpaths.
pub fn generate_deburr3d(
    ctx: &StrategyExecContext<'_>,
    params: &Deburr3DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;
    if !ctx.doc.has_3d_geometry() {
        return Err(CamError::EmptyCadDocument);
    }

    // Validate parameters & tool compatibility
    let chamfer_w = params.chamfer_width.value_mm;
    if !chamfer_w.is_finite() || chamfer_w <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.chamfer_width",
            value: chamfer_w,
            reason: "must be finite and > 0.0001 mm",
        });
    }
    let tip_offset = params.tip_offset.value_mm;
    if !tip_offset.is_finite() || tip_offset < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.tip_offset",
            value: tip_offset,
            reason: "must be finite and >= 0.0 mm",
        });
    }
    let wall_clearance = params.wall_clearance.value_mm;
    if !wall_clearance.is_finite() || wall_clearance < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.wall_clearance",
            value: wall_clearance,
            reason: "must be finite and >= 0.0 mm",
        });
    }
    let edge_extension = params.edge_extension.value_mm;
    if !edge_extension.is_finite() || edge_extension < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.edge_extension",
            value: edge_extension,
            reason: "must be finite and >= 0.0 mm",
        });
    }
    let chord_tol = params.chord_tolerance_mm;
    if !chord_tol.is_finite() || chord_tol <= 1e-5 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.chord_tolerance_mm",
            value: chord_tol,
            reason: "must be finite and > 0.00001 mm",
        });
    }
    if !params.min_dihedral_angle_deg.is_finite()
        || params.min_dihedral_angle_deg <= 0.0
        || params.min_dihedral_angle_deg >= 179.0
    {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.min_dihedral_angle_deg",
            value: params.min_dihedral_angle_deg,
            reason: "must be within (0°, 179°)",
        });
    }

    match ctx.tool.geometry {
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => {
            if params.allow_underside_back_chamfer {
                return Err(CamError::InvalidChamferGeometry {
                    reason: "Underside back-chamfering requires a Lollipop Endmill cutter."
                        .to_string(),
                });
            }
            let half_a = (0.5 * included_angle_deg).clamp(5.0, 85.0).to_radians();
            let max_flute_radial = 0.5 * (diameter_mm - tip_diameter_mm).max(0.0);
            if tip_offset * half_a.tan() + chamfer_w > max_flute_radial + 1e-3 {
                return Err(CamError::InvalidChamferGeometry {
                    reason: format!(
                        "chamfer width ({chamfer_w:.2} mm) + tip_offset radial shift ({:.2} mm) exceeds cutter cone radial span ({max_flute_radial:.2} mm)",
                        tip_offset * half_a.tan()
                    ),
                });
            }
        }
        CutterGeometry::BallEndmill { .. } => {
            if params.allow_underside_back_chamfer {
                return Err(CamError::InvalidChamferGeometry {
                    reason: "Underside back-chamfering requires a Lollipop Endmill cutter."
                        .to_string(),
                });
            }
        }
        CutterGeometry::LollipopEndmill { .. } => {}
        _ => {
            return Err(CamError::IncompatibleTool {
                op_name: ctx.op.name.clone(),
                op_kind: ctx.op.kind.label(),
                tool_name: ctx.tool.name.clone(),
                tool_kind: ctx.tool.geometry.kind_label(),
            });
        }
    }

    let lookup = FaceTriLookup::build(ctx.doc);
    let poly_by_edge: HashMap<EdgeId, &[DVec3]> = ctx
        .doc
        .render_mesh
        .edge_polylines
        .iter()
        .map(|ep| (ep.edge_id, ep.points_mm.as_slice()))
        .collect();

    // 1. Extract explicit or auto-detected 3D B-Rep edge chains in Setup WCS
    let use_auto_detect = params.auto_detect_edges || ctx.op.selections.is_empty();
    let mut chains = if use_auto_detect {
        auto_detect_sharp_edge_chains(ctx, params, &lookup, &poly_by_edge, chord_tol)?
    } else {
        extract_selected_edge_chains(ctx, params, &lookup, &poly_by_edge, chord_tol)?
    };

    if chains.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 2. Orient chains for Climb vs Conventional milling & apply optional open-edge extensions
    for chain in &mut chains {
        orient_chain_for_cut_direction(chain, params.climb_direction);
        if !chain.closed && edge_extension > 1e-4 {
            apply_open_chain_extension(chain, edge_extension);
        }
    }

    // 3. Build target B-Rep TriMesh3D + FlatBvh in Setup WCS for full-assembly collision trimming
    let target_surface =
        SurfaceMesh3D::from_tri_mesh(ctx.doc.build_wcs_tri_mesh(&[], &ctx.setup.frame));
    let target_mesh = &target_surface.mesh;
    let avoid_2d = ctx.resolve_avoid_2d(ctx.heights.bottom_z_wcs, ctx.heights.top_z_wcs);
    let had_avoid = !avoid_2d.effective_region.is_empty();
    let (xy_containment, avoid_keepout): (Option<RegionSet2D>, RegionSet2D) = if use_auto_detect {
        let tool_r = 0.5 * ctx.tool.geometry.cutting_diameter_mm().max(0.1);
        let mut resolved_boundary = resolve_3d_boundary_config_detailed(
            &target_surface,
            ctx.setup,
            &ctx.selection,
            &params.boundary,
            tool_r,
        );
        resolved_boundary.apply_avoid(avoid_2d);
        builder.warnings.extend(resolved_boundary.warnings);
        if resolved_boundary.effective_region.is_empty() {
            if had_avoid {
                builder.warnings.push(
                    "Avoid / Keep-Out regions excluded 100% of the cutting area.".to_string(),
                );
            }
            return Err(CamError::EmptyToolpath {
                op_name: ctx.op.name.clone(),
            });
        }
        (
            Some(resolved_boundary.effective_region),
            resolved_boundary.avoid_effective_region,
        )
    } else {
        builder.warnings.extend(avoid_2d.warnings);
        (None, avoid_2d.effective_region)
    };
    let avoid_keepout_opt = (!avoid_keepout.is_empty()).then_some(&avoid_keepout);
    let envelope = AnalyticCutterEnvelope::from_geometry(&ctx.tool.geometry);
    let stickout_mm = ctx
        .tool
        .stickout_length_mm
        .max(envelope.effective_reach_mm());

    // 4. Progressive width passes (+ optional spring pass)
    let n_passes = params.multiple_passes.clamp(1, 20);
    let mut pass_schedule: Vec<(f64, MotionRole)> = Vec::with_capacity(n_passes as usize + 1);
    for k in 1..=n_passes {
        let frac = (k as f64) / (n_passes as f64);
        pass_schedule.push((chamfer_w * frac, MotionRole::Cut));
    }
    if params.spring_pass {
        pass_schedule.push((chamfer_w, MotionRole::FinishCut));
    }

    let retract_z = ctx.safe_retract_z();
    let mut emitted_subchains = 0usize;
    let mut cand_buf = Vec::<usize>::with_capacity(64);

    for (w_pass, cut_role) in pass_schedule {
        ctx.check_cancel()?;
        for chain in &chains {
            ctx.check_cancel()?;

            let subchains = trim_chain_against_target(
                chain,
                ctx.tool,
                &envelope,
                stickout_mm,
                target_mesh,
                w_pass,
                tip_offset,
                wall_clearance,
                params.allow_underside_back_chamfer,
                xy_containment.as_ref(),
                avoid_keepout_opt,
                &mut cand_buf,
            );

            for sub in subchains {
                ctx.check_cancel()?;
                if emit_deburr_subchain(
                    ctx,
                    &sub,
                    &envelope,
                    stickout_mm,
                    target_mesh,
                    wall_clearance,
                    retract_z,
                    cut_role,
                    &mut cand_buf,
                    builder,
                ) {
                    emitted_subchains += 1;
                }
            }
        }
    }

    if emitted_subchains == 0 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;

    fn sample_90deg_top_edge() -> DeburrSample3D {
        DeburrSample3D {
            pos_wcs: DVec3::new(0.0, 0.0, 10.0),
            tangent: DVec3::Y,
            n1: DVec3::Z,
            n2: DVec3::X,
            bisector_out: DVec3::new(1.0, 0.0, 1.0).normalize(),
            is_underside: false,
            face1: FaceId(0),
            face2: FaceId(1),
        }
    }

    #[test]
    fn solve_deburr_tool_tip_exact_chamfer_ball_and_lollipop_kinematics() {
        let s = sample_90deg_top_edge();

        // 1. 90° Chamfer Mill (`D=6`, `tip=0`, `w=0.5`, `tip_offset=0.75`):
        //    Matches classic 2D chamfer kinematics: tip at `(x = +0.75, z = 10 - 0.5 - 0.75 = 8.75)`.
        let chamf = CutterGeometry::ChamferMill {
            diameter_mm: 6.0,
            tip_diameter_mm: 0.0,
            included_angle_deg: 90.0,
            flute_length_mm: 3.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        let sol_c = solve_deburr_tool_tip(&s, &chamf, 0.5, 0.75, 0.15, false).unwrap();
        assert!(
            approx_eq(sol_c.tip_wcs.x, 0.75, 1e-6),
            "got {:?}",
            sol_c.tip_wcs
        );
        assert!(
            approx_eq(sol_c.tip_wcs.z, 8.75, 1e-6),
            "got {:?}",
            sol_c.tip_wcs
        );

        // 2. Ø6 mm Ball Endmill (`R=3`, `w=0.5`, `d_bevel = 0.5 / sqrt(2)`):
        //    Sphere center at `p + (R - d_bevel) * b`, tip `R` below center.
        let ball = CutterGeometry::BallEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        };
        let sol_b = solve_deburr_tool_tip(&s, &ball, 0.5, 0.75, 0.15, false).unwrap();
        let d_bevel = 0.5 * std::f64::consts::FRAC_1_SQRT_2;
        let expected_c = s.pos_wcs + s.bisector_out * (3.0 - d_bevel);
        assert!(approx_eq(sol_b.tip_wcs.x, expected_c.x, 1e-6));
        assert!(approx_eq(sol_b.tip_wcs.z, expected_c.z - 3.0, 1e-6));

        // 3. Ø6 mm Lollipop Endmill (`R=3`, `r_neck=1.5`) on underside overhang rim (`b = (+1, 0, -1)/sqrt(2)`):
        let s_under = DeburrSample3D {
            pos_wcs: DVec3::new(20.0, 0.0, 10.0),
            tangent: DVec3::Y,
            n1: -DVec3::Z,
            n2: DVec3::X,
            bisector_out: DVec3::new(1.0, 0.0, -1.0).normalize(),
            is_underside: true,
            face1: FaceId(2),
            face2: FaceId(3),
        };
        let lolli = CutterGeometry::LollipopEndmill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.0,
            neck_length_mm: 18.0,
            flute_length_mm: 5.5,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        // Rejected when `allow_underside_back_chamfer == false`
        assert!(solve_deburr_tool_tip(&s_under, &lolli, 0.5, 0.75, 0.15, false).is_none());
        // Accepted when `allow_underside_back_chamfer == true` (horiz_reach ≈ 1.871 mm > 1.5 + 0.15)
        let sol_l = solve_deburr_tool_tip(&s_under, &lolli, 0.5, 0.75, 0.15, true).unwrap();
        let exp_l_center = s_under.pos_wcs + s_under.bisector_out * (3.0 - d_bevel);
        assert!(approx_eq(sol_l.tip_wcs.x, exp_l_center.x, 1e-6));
        assert!(approx_eq(sol_l.tip_wcs.z, exp_l_center.z - 3.0, 1e-6));
        // Rejected when neck is too thick (`r_neck = 2.0 > horiz_reach = 1.871`)
        let thick_lolli = CutterGeometry::LollipopEndmill {
            diameter_mm: 6.0,
            neck_diameter_mm: 4.0,
            neck_length_mm: 18.0,
            flute_length_mm: 5.2,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        assert!(solve_deburr_tool_tip(&s_under, &thick_lolli, 0.5, 0.75, 0.15, true).is_none());
    }
}
