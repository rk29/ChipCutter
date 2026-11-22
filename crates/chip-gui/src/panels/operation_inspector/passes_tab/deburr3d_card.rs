use chip_cam::Deburr3DParams;
use chip_math::DisplayUnit;
use chip_tools::CutterGeometry;

use super::PassItemCtx;
use crate::widgets::{BannerKind, inline_banner, section_card};

pub(super) struct Deburr3DParamsUiCtx<'a> {
    pub p: &'a mut Deburr3DParams,
    pub assigned_tool_geom: Option<&'a CutterGeometry>,
    pub has_explicit_selections: bool,
    pub display_unit: DisplayUnit,
    pub item: PassItemCtx<'a>,
}

/// Evaluates immediate inline tool compatibility and safety banners for a `Deburr3D` operation.
pub fn collect_deburr3d_banners(
    p: &Deburr3DParams,
    assigned_tool_geom: Option<&CutterGeometry>,
    display_unit: DisplayUnit,
) -> Vec<(BannerKind, String)> {
    let mut banners = Vec::new();
    let Some(geom) = assigned_tool_geom else {
        banners.push((
            BannerKind::Warn,
            "No deburring cutter assigned. Select a Chamfer Mill, Ball Endmill, or Lollipop Endmill in the Tool tab."
                .to_string(),
        ));
        return banners;
    };

    match *geom {
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => {
            if p.allow_underside_back_chamfer {
                banners.push((
                    BannerKind::Error,
                    "Underside Back-Chamfer requires a relieved-neck Lollipop Endmill (assigned: Chamfer Mill)."
                        .to_string(),
                ));
            }
            let half_a = (0.5 * included_angle_deg).clamp(5.0, 85.0).to_radians();
            let max_r = 0.5 * (diameter_mm - tip_diameter_mm).max(0.0);
            let req_r =
                p.tip_offset.value_mm.max(0.0) * half_a.tan() + p.chamfer_width.value_mm.max(0.0);
            if req_r > max_r + 1e-3 {
                banners.push((
                    BannerKind::Error,
                    format!(
                        "Chamfer width + tip offset radial reach ({}) exceeds chamfer mill flute capacity ({}). Reduce chamfer width or tip offset.",
                        display_unit.format_length(req_r),
                        display_unit.format_length(max_r),
                    ),
                ));
            } else if !p.allow_underside_back_chamfer {
                banners.push((
                    BannerKind::Info,
                    format!(
                        "Chamfer Mill ({included_angle_deg:.0}° cone): constant-width conical flank contact with {} tip clearance.",
                        display_unit.format_length(p.tip_offset.value_mm.max(0.0))
                    ),
                ));
            }
        }
        CutterGeometry::BallEndmill { diameter_mm, .. } => {
            if p.allow_underside_back_chamfer {
                banners.push((
                    BannerKind::Error,
                    "Underside Back-Chamfer requires a relieved-neck Lollipop Endmill (assigned: Ball Endmill)."
                        .to_string(),
                ));
            } else {
                banners.push((
                    BannerKind::Info,
                    format!(
                        "Ball Endmill (Ø{}): exact 3D spherical tangent contact along curved and sloped edges.",
                        display_unit.format_length(diameter_mm)
                    ),
                ));
            }
        }
        CutterGeometry::LollipopEndmill {
            diameter_mm,
            neck_diameter_mm,
            ..
        } => {
            let r = 0.5 * diameter_mm;
            let r_neck = 0.5 * neck_diameter_mm;
            let d_bevel = p.chamfer_width.value_mm.max(0.0) * std::f64::consts::FRAC_1_SQRT_2;
            let horiz_reach = (r - d_bevel) * std::f64::consts::FRAC_1_SQRT_2;
            if p.allow_underside_back_chamfer
                && horiz_reach < r_neck + p.wall_clearance.value_mm.max(0.0)
            {
                banners.push((
                    BannerKind::Warn,
                    format!(
                        "Lollipop neck radius ({}) + wall clearance ({}) may be too wide for 90° underside back-chamfering (horizontal reach {}).",
                        display_unit.format_length(r_neck),
                        display_unit.format_length(p.wall_clearance.value_mm.max(0.0)),
                        display_unit.format_length(horiz_reach.max(0.0)),
                    ),
                ));
            } else {
                let mode_str = if p.allow_underside_back_chamfer {
                    "top + underside overhang back-chamfering enabled"
                } else {
                    "top-accessible 3D edges (enable Underside Back-Chamfer for overhangs)"
                };
                banners.push((
                    BannerKind::Info,
                    format!(
                        "Lollipop Endmill (Ø{} / neck Ø{}): {mode_str}.",
                        display_unit.format_length(diameter_mm),
                        display_unit.format_length(neck_diameter_mm),
                    ),
                ));
            }
        }
        _ => {
            banners.push((
                BannerKind::Error,
                format!(
                    "Assigned tool is a {} -- 3D Edge Deburr requires a Chamfer Mill, Ball Endmill, or Lollipop Endmill.",
                    geom.kind_label()
                ),
            ));
        }
    }

    banners
}

pub(super) fn show_deburr3d_params(ui: &mut egui::Ui, ctx: Deburr3DParamsUiCtx<'_>) {
    let Deburr3DParamsUiCtx {
        p,
        assigned_tool_geom,
        has_explicit_selections,
        display_unit,
        mut item,
    } = ctx;
    let (setup_idx, op_idx) = item.ids();

    let banners = collect_deburr3d_banners(p, assigned_tool_geom, display_unit);
    for (kind, msg) in &banners {
        inline_banner(ui, *kind, msg);
        ui.add_space(4.0);
    }

    let is_chamfer_mill = matches!(
        assigned_tool_geom,
        None | Some(CutterGeometry::ChamferMill { .. })
    );

    section_card(
        ui,
        ("passes_deburr3d_card", setup_idx, op_idx),
        "3D Deburr Passes",
        true,
        |ui| {
            item.grid(ui, "passes_deburr3d_grid", |f| {
                f.set_tip(
                    "Constant edge-break / chamfer leg width measured along the adjacent B-Rep faces",
                );
                f.len("Chamfer Width", &mut p.chamfer_width, "Edit 3D Chamfer Width");

                if is_chamfer_mill {
                    f.set_tip(
                        "Axial distance above the pointed Chamfer Mill tip so cutting occurs on the stronger conical flank",
                    );
                    f.len("Tip Offset", &mut p.tip_offset, "Edit 3D Deburr Tip Offset");
                }

                f.set_tip(
                    "Minimum safety clearance preserved from neighboring vertical walls, stepped bosses, and inside corners (colliding edge ends are trimmed automatically)",
                );
                f.len(
                    "Wall Clearance",
                    &mut p.wall_clearance,
                    "Edit 3D Deburr Wall Clearance",
                );

                f.set_tip(
                    "Optional tangent extension beyond the open ends of non-closed 3D edges before neighbor-wall collision trimming",
                );
                f.len(
                    "Edge Extension",
                    &mut p.edge_extension,
                    "Edit 3D Deburr Edge Extension",
                );

                f.set_tip(
                    "Automatically detect all sharp convex 3D edges on the solid (also active automatically when no explicit edges are selected in Geometry)",
                );
                f.check(
                    "Auto-Detect Edges",
                    &mut p.auto_detect_edges,
                    "Scan Sharp Convex Edges",
                    "Toggle Auto-Detect Sharp Edges",
                );

                if p.auto_detect_edges || !has_explicit_selections {
                    f.set_tip(
                        "Minimum dihedral normal-change angle between adjacent faces to classify an edge as sharp (ignores smooth tangent fillet seams)",
                    );
                    f.drag(
                        "Min Edge Angle",
                        &mut p.min_dihedral_angle_deg,
                        1.0..=90.0,
                        (0.5, "°"),
                        "Edit Min Dihedral Edge Angle",
                    );

                    f.set_tip(
                        "Skip pure vertical side-wall edges parallel to Setup WCS ±Z during automatic edge detection",
                    );
                    f.check(
                        "Exclude Vertical",
                        &mut p.exclude_vertical_edges,
                        "Skip ±Z Vertical Edges",
                        "Toggle Exclude Vertical Edges",
                    );
                }

                f.set_tip(
                    "Machine downward-facing underside overhang rim edges from underneath using a relieved-neck Lollipop Endmill",
                );
                f.check(
                    "Underside Chamfer",
                    &mut p.allow_underside_back_chamfer,
                    "Back-Chamfer (Lollipop)",
                    "Toggle Underside Back-Chamfer",
                );

                f.set_tip(
                    "Prefer Climb milling (material on right for M3 spindle) vs. Conventional milling along deburred edges",
                );
                f.check(
                    "Climb Direction",
                    &mut p.climb_direction,
                    "Climb Milling",
                    "Toggle 3D Deburr Cut Direction",
                );

                f.set_tip(
                    "Maximum 3D chord-sagitta deviation when subdividing curved non-planar B-Rep edges",
                );
                f.drag(
                    "Chord Tolerance",
                    &mut p.chord_tolerance_mm,
                    0.001..=0.05,
                    (0.001, " mm"),
                    "Edit 3D Deburr Chord Tolerance",
                );

                f.set_tip(
                    "Number of progressive chamfer passes stepping smoothly up to the full Chamfer Width",
                );
                f.drag(
                    "Passes",
                    &mut p.multiple_passes,
                    1..=10,
                    (0.1, ""),
                    "Edit 3D Deburr Pass Count",
                );

                f.set_tip(
                    "Append an extra zero-offset finishing pass at full Chamfer Width to remove tool deflection",
                );
                f.check(
                    "Spring Pass",
                    &mut p.spring_pass,
                    "Add Finish Spring Pass",
                    "Toggle 3D Deburr Spring Pass",
                );
            });
        },
    );
}

#[cfg(test)]
mod tests {
    use chip_math::ParamLength;

    use super::*;

    #[test]
    fn deburr3d_banners_report_tool_compatibility_and_underside_requirements() {
        let mut p = Deburr3DParams::default();
        let flat = CutterGeometry::FlatEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let b_flat = collect_deburr3d_banners(&p, Some(&flat), DisplayUnit::Millimeter);
        assert!(b_flat.iter().any(|(k, _)| *k == BannerKind::Error));

        let chamf = CutterGeometry::ChamferMill {
            diameter_mm: 6.0,
            tip_diameter_mm: 0.0,
            included_angle_deg: 90.0,
            flute_length_mm: 3.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        p.allow_underside_back_chamfer = true;
        let b_under = collect_deburr3d_banners(&p, Some(&chamf), DisplayUnit::Millimeter);
        assert!(
            b_under
                .iter()
                .any(|(k, m)| *k == BannerKind::Error && m.contains("Lollipop"))
        );

        let lolli = CutterGeometry::LollipopEndmill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.0,
            neck_length_mm: 18.0,
            flute_length_mm: 5.5,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        p.chamfer_width = ParamLength::from_mm(0.5);
        let b_lolli = collect_deburr3d_banners(&p, Some(&lolli), DisplayUnit::Millimeter);
        assert!(b_lolli.iter().all(|(k, _)| *k == BannerKind::Info));
    }
}
