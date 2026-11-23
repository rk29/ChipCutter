use chip_cam::{
    ChamferTraceMode, HeightReference, Operation, OperationKind, ResolvedHeights,
    ResolvedRotaryConfig, ResolvedSetup, StrategyParams,
};
use chip_math::{DisplayUnit, EvalContext};

use crate::widgets::{
    BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
    section_card, static_card,
};

const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

pub struct HeightsTabCtx<'a> {
    pub op: &'a mut Operation,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub display_unit: DisplayUnit,
    pub eval_ctx: &'a EvalContext,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub inf_top: f64,
    pub inf_bot: f64,
    pub show_z_planes: &'a mut bool,
    pub mutated_label: &'a mut Option<&'static str>,
}

/// Returns a concise contextual explanation banner for strategies with specialized height semantics.
pub fn heights_contextual_notice(op: &Operation) -> Option<&'static str> {
    match op.kind {
        OperationKind::VCarve => Some(
            "V-Carve Bottom Height is governed by Flat Depth in [Passes]; only Top Height sets the carving surface plane.",
        ),
        OperationKind::ChamferTrace => match op.resolved_strategy() {
            StrategyParams::ChamferTrace(p) if p.mode == ChamferTraceMode::EdgeChamfer => Some(
                "Edge Chamfer Z depth is derived automatically from the selected chamfer edge + Tip Offset in [Passes].",
            ),
            _ => None,
        },
        OperationKind::Rotary2Linear | OperationKind::Rotary4AxisSim => Some(
            "4th-Axis Rotary Top Height and Bottom Height map directly to Outer Radius (R_max) and Core Floor Radius (R_min) measured from the rotary centerline.",
        ),
        _ => None,
    }
}

/// Computes the live equivalent `(R_max, R_min)` radii (`mm` from rotary centerline) matching
/// `chip_cam::resolve_rotary_window` for native 4th-axis rotary operations.
pub fn rotary_equivalent_radii_mm(
    rot: &ResolvedRotaryConfig,
    top_z_wcs: f64,
    bottom_z_wcs: f64,
) -> (f64, f64) {
    let stock_top_z = rot.center_wcs.z + rot.stock_outer_radius_mm;
    let r_min = (bottom_z_wcs - rot.center_wcs.z).max(0.0);
    let r_max = if top_z_wcs < stock_top_z - 1e-6 {
        (top_z_wcs - rot.center_wcs.z).clamp(r_min, rot.stock_outer_radius_mm.max(r_min))
    } else {
        rot.stock_outer_radius_mm.max(r_min)
    };
    (r_max, r_min)
}

pub fn show_heights_tab(ui: &mut egui::Ui, ctx: HeightsTabCtx<'_>) {
    let HeightsTabCtx {
        op,
        setup_idx,
        op_idx,
        display_unit,
        eval_ctx,
        resolved_setup,
        inf_top,
        inf_bot,
        show_z_planes,
        mutated_label,
    } = ctx;

    if let Some(notice) = heights_contextual_notice(op) {
        inline_banner(ui, BannerKind::Info, notice);
        ui.add_space(6.0);
    }

    let resolved_heights_preview = resolved_setup.map(|rs| {
        let mut h_eval = op.heights.clone();
        h_eval.reevaluate(eval_ctx);
        ResolvedHeights::resolve(
            &h_eval,
            &rs.model_aabb_wcs,
            &rs.stock.aabb_wcs(),
            rs.clearance_z_wcs,
            rs.retract_z_wcs,
            inf_top,
            inf_bot,
        )
    });

    let rotary_radii: Option<(f64, f64)> = if op.kind.is_rotary_native()
        && let Some(rot) = resolved_setup.and_then(|rs| rs.rotary)
        && let Some(Ok(rh)) = &resolved_heights_preview
    {
        Some(rotary_equivalent_radii_mm(
            &rot,
            rh.top_z_wcs,
            rh.bottom_z_wcs,
        ))
    } else {
        None
    };

    let planes = [
        (
            "Clearance",
            "Highest safe Z plane used for initial/final XY rapids above clamps and stock",
            0usize,
        ),
        (
            "Retract",
            "Intermediate rapid retract Z plane between cutting regions",
            1usize,
        ),
        (
            "Feed",
            "Z height where tool transitions from rapid G0 plunge to controlled feedrate",
            2usize,
        ),
        (
            "Top",
            "Upper Z surface where material cutting begins",
            3usize,
        ),
        (
            "Bottom",
            "Final target cut floor Z height (must be <= Top Height)",
            4usize,
        ),
    ];

    section_card(
        ui,
        ("heights_planes_card", setup_idx, op_idx),
        "Z-Height Planes & Offsets",
        true,
        |ui| {
            inspector_grid(ui, ("heights_planes_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "3D Preview",
                    Some("Render translucent Z-height reference planes in the 3D viewport"),
                    |ui| {
                        ui.checkbox(show_z_planes, "Show Z-Planes")
                            .on_hover_text("Render translucent color-coded Clearance, Retract, Feed, Top, and Bottom Z planes in the 3D viewport");
                    },
                );

                for (name, tip, which) in planes {
                    let spec = match which {
                        0 => &mut op.heights.clearance,
                        1 => &mut op.heights.retract,
                        2 => &mut op.heights.feed,
                        3 => &mut op.heights.top,
                        _ => &mut op.heights.bottom,
                    };
                    let ref_lbl = format!("{name} Ref");
                    let off_lbl = format!("{name} Offset");
                    grid_row(ui, &ref_lbl, Some(tip), |ui| {
                        egui::ComboBox::from_id_salt(("op_h_ref", setup_idx, op_idx, which))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(spec.reference.label())
                            .show_ui(ui, |ui| {
                                for r in HeightReference::OPERATION_REFS {
                                    if ui
                                        .selectable_value(&mut spec.reference, r, r.label())
                                        .on_hover_text(format!(
                                            "Measure {name} height relative to {}",
                                            r.label()
                                        ))
                                        .changed()
                                    {
                                        *mutated_label = Some("Change Operation Height Reference");
                                    }
                                }
                            })
                            .response
                            .on_hover_text(tip);
                    });
                    grid_row(
                        ui,
                        &off_lbl,
                        Some("Signed Z offset added to the selected height reference datum"),
                        |ui| {
                            ui.horizontal(|ui| {
                                if edit_param_length(
                                    ui,
                                    ("op_h_off", setup_idx, op_idx, which),
                                    &mut spec.offset,
                                    eval_ctx,
                                    INPUT_COL_WIDTH,
                                ) {
                                    *mutated_label = Some("Edit Operation Height Offset");
                                }
                                if let Some((r_max, r_min)) = rotary_radii {
                                    if which == 3 {
                                        ui.weak(format!(
                                            "R_max = {}",
                                            display_unit.format_length(r_max)
                                        ));
                                    } else if which == 4 {
                                        ui.weak(format!(
                                            "R_min = {}",
                                            display_unit.format_length(r_min)
                                        ));
                                    }
                                }
                            });
                        },
                    );
                }
            });
        },
    );

    if let Some(res) = resolved_heights_preview {
        static_card(ui, Some("Resolved WCS Z Hierarchy"), |ui| match res {
            Ok(rh) => {
                inline_banner(
                    ui,
                    BannerKind::Clean,
                    "Z-Height Hierarchy Valid (Clearance ≥ Retract ≥ Feed ≥ Top ≥ Bottom)",
                );
                ui.add_space(4.0);
                inspector_grid(ui, ("heights_resolved_grid", setup_idx, op_idx), |ui| {
                    for (idx, (lbl, val)) in [
                        ("Clearance Z", rh.clearance_z_wcs),
                        ("Retract Z", rh.retract_z_wcs),
                        ("Feed Z", rh.feed_z_wcs),
                        ("Top Z", rh.top_z_wcs),
                        ("Bottom Z", rh.bottom_z_wcs),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        grid_row(ui, lbl, None, |ui| {
                            ui.horizontal(|ui| {
                                ui.strong(display_unit.format_length(val));
                                if let Some((r_max, r_min)) = rotary_radii {
                                    if idx == 3 {
                                        ui.weak(format!(
                                            "  (Outer Radius R_max = {})",
                                            display_unit.format_length(r_max)
                                        ));
                                    } else if idx == 4 {
                                        ui.weak(format!(
                                            "  (Core Floor Radius R_min = {})",
                                            display_unit.format_length(r_min)
                                        ));
                                    }
                                }
                            });
                        });
                    }
                });
            }
            Err(err) => {
                inline_banner(ui, BannerKind::Error, err.to_string());
            }
        });
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{RotaryAxisKind, RotaryDirection, RotaryDriveMode, RotaryUnwindMode};
    use chip_math::DVec3;

    use super::*;

    #[test]
    fn heights_contextual_notices_cover_vcarve_edge_chamfer_and_rotary() {
        let vcarve = Operation::new("vc", "V-Carve", OperationKind::VCarve);
        let vc_msg = heights_contextual_notice(&vcarve).expect("VCarve must show contextual note");
        assert!(vc_msg.contains("Flat Depth"));

        let mut chamfer = Operation::new("ch", "Chamfer", OperationKind::ChamferTrace);
        let ch_msg =
            heights_contextual_notice(&chamfer).expect("EdgeChamfer must show contextual note");
        assert!(ch_msg.contains("Tip Offset"));

        if let StrategyParams::ChamferTrace(p) = chamfer.strategy_mut() {
            p.mode = ChamferTraceMode::CurveTrace;
        }
        assert!(
            heights_contextual_notice(&chamfer).is_none(),
            "CurveTrace uses standard heights and should not show EdgeChamfer banner"
        );

        for kind in [OperationKind::Rotary2Linear, OperationKind::Rotary4AxisSim] {
            let op = Operation::new("rot", "Rotary", kind);
            let msg = heights_contextual_notice(&op).expect("Rotary op must show radial hint");
            assert!(msg.contains("R_max") && msg.contains("R_min"));
        }

        let pocket = Operation::new("pkt", "Pocket", OperationKind::Pocket2D);
        assert!(heights_contextual_notice(&pocket).is_none());
    }

    #[test]
    fn rotary_equivalent_radii_matches_rotary_window_semantics() {
        let rot = ResolvedRotaryConfig {
            mode: RotaryDriveMode::default(),
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::default(),
            axis_letter: 'A',
            center_wcs: DVec3::new(0.0, 0.0, 10.0),
            stock_outer_radius_mm: 22.0,
            clearance_radius_mm: 28.0,
            stock_u_min: 0.0,
            stock_u_max: 80.0,
            chuck_end_positive: false,
            chuck_u_min: -10.0,
            chuck_u_max: 5.0,
            chuck_radius_mm: 40.0,
            unwind_mode: RotaryUnwindMode::default(),
        };

        // Top Z at stock top (`10 + 22 = 32`) and Bottom Z at `15` -> R_max = 22, R_min = 5
        let (r_max, r_min) = rotary_equivalent_radii_mm(&rot, 32.0, 15.0);
        assert!((r_max - 22.0).abs() < 1e-9);
        assert!((r_min - 5.0).abs() < 1e-9);

        // Top Z lowered to `26` (`R = 16`) and Bottom Z below centerline (`4` -> clamped to `0`)
        let (r_max2, r_min2) = rotary_equivalent_radii_mm(&rot, 26.0, 4.0);
        assert!((r_max2 - 16.0).abs() < 1e-9);
        assert!((r_min2 - 0.0).abs() < 1e-9);
    }
}
