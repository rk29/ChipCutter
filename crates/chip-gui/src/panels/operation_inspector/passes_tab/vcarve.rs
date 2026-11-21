use chip_cam::{VCarveClearingKind, VCarveInlayMode, VCarveParams};
use chip_math::ParamLength;
use chip_tools::ToolId;

use super::{COMBO_COL_WIDTH, PassItemCtx};
use crate::widgets::{BannerKind, inline_banner, section_card};

pub(super) struct VCarveParamsUiCtx<'a> {
    pub p: &'a mut VCarveParams,
    pub clearing_tool_candidates: &'a [(ToolId, String, Option<String>)],
    pub item: PassItemCtx<'a>,
}

pub(super) fn show_vcarve_params(ui: &mut egui::Ui, ctx: VCarveParamsUiCtx<'_>) {
    let VCarveParamsUiCtx {
        p,
        clearing_tool_candidates,
        mut item,
    } = ctx;
    let (setup_idx, op_idx) = item.ids();

    section_card(
        ui,
        ("vc_strategy_card", setup_idx, op_idx),
        "V-Carve Strategy",
        true,
        |ui| {
            let mut has_flat_depth = p.flat_depth.is_some();
            item.grid(ui, "vc_strategy_grid", |f| {
                f.set_tip("Clamp V-bit medial-axis carving at a flat floor depth (d_flat) for wide letters and V-inlays");
                f.row("Flat Floor Limit", |ui, mutated| {
                    if ui
                        .checkbox(&mut has_flat_depth, "Limit Max Depth")
                        .on_hover_text("Clamp V-bit depth at a flat floor (d_flat) for wide letters, signs, and V-inlays")
                        .changed()
                    {
                        p.flat_depth = if has_flat_depth {
                            Some(ParamLength::from_mm(3.0))
                        } else {
                            None
                        };
                        *mutated = Some("Toggle V-Carve Flat Depth");
                    }
                });

                if let Some(fd) = &mut p.flat_depth {
                    f.set_tip("Maximum flat pocket floor depth (d_flat) measured below Top Height");
                    f.len("Flat Depth", fd, "Edit V-Carve Flat Depth");
                    f.set_tip("Stepover used by the V-bit when clearing tight flat floor corners");
                    f.len("V-Bit Stepover", &mut p.vbit_flat_stepover, "Edit V-Bit Flat Stepover");
                }
            });

            if p.flat_depth.is_none() {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "Full-depth medial-axis V-carving: V-bit follows Voronoi bisectors with depth proportional to local stroke width.",
                );
            }
        },
    );

    if p.flat_depth.is_some() {
        section_card(
            ui,
            ("vc_clearance_card", setup_idx, op_idx),
            "Two-Stage Flat Area Clearance",
            true,
            |ui| {
                let current_lbl = match &p.clearing_tool_id {
                    None => "None (V-Bit Clears All)".to_string(),
                    Some(cid) => clearing_tool_candidates
                        .iter()
                        .find(|(id, _, _)| id == cid)
                        .map_or_else(|| cid.0.clone(), |(_, lbl, _)| lbl.clone()),
                };

                item.grid(ui, "vc_clearance_grid", |f| {
                    f.set_tip("Optional flat/bull-nose endmill for Stage 1 bulk flat-floor pocketing before Stage 2 V-bit bevels");
                    f.row("Clearing Tool", |ui, mutated| {
                        egui::ComboBox::from_id_salt(("vc_clear_tool", setup_idx, op_idx))
                            .selected_text(current_lbl)
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .show_ui(ui, |ui| {
                                let is_none = p.clearing_tool_id.is_none();
                                if ui
                                    .selectable_label(is_none, "None (V-Bit Clears All Flats)")
                                    .on_hover_text("Clear flat pocket areas using only the primary V-bit")
                                    .clicked()
                                    && !is_none
                                {
                                    p.clearing_tool_id = None;
                                    p.clearing_preset_id = None;
                                    *mutated = Some("Clear V-Carve Secondary Tool");
                                }
                                for (tid, lbl, default_preset) in clearing_tool_candidates {
                                    let is_sel = p.clearing_tool_id.as_ref() == Some(tid);
                                    if ui
                                        .selectable_label(is_sel, lbl)
                                        .on_hover_text("Use this flat/bull-nose endmill to clear wide flat interiors before the V-bit pass")
                                        .clicked()
                                        && !is_sel
                                    {
                                        p.clearing_tool_id = Some(tid.clone());
                                        p.clearing_preset_id = default_preset.clone();
                                        *mutated = Some("Select V-Carve Secondary Clearing Tool");
                                    }
                                }
                            })
                            .response
                            .on_hover_text("Optional secondary flat endmill for Stage 1 flat-area clearance before V-bit beveling");
                    });

                    if p.clearing_tool_id.is_some() {
                        f.set_tip("Clearing toolpath pattern for flat interior regions");
                        f.row("Stage 1 Pattern", |ui, mutated| {
                            for kind in VCarveClearingKind::ALL {
                                if ui
                                    .selectable_value(&mut p.clearing_strategy, kind, kind.label())
                                    .on_hover_text(format!("Clear flat V-carve pockets using {}", kind.label()))
                                    .changed()
                                {
                                    *mutated = Some("Change V-Carve Clearing Strategy");
                                }
                            }
                        });
                        f.set_tip("Radial stock left by Stage 1 clearing tool for seamless Stage 2 V-bit wall blending");
                        f.len("Blend Allowance", &mut p.clearing_radial_allowance, "Edit V-Carve Clearing Allowance");
                    }
                });
            },
        );
    }

    section_card(
        ui,
        ("vc_inlay_card", setup_idx, op_idx),
        "Mated Male V-Inlay Plug",
        true,
        |ui| {
            let is_male = matches!(p.inlay_mode, VCarveInlayMode::MalePlug { .. });
            let mut sel_male = is_male;

            item.grid(ui, "vc_inlay_grid", |f| {
                f.set_tip("Machine standard female V-cavity or a mirrored raised male plug with tapered wall engagement");
                f.row("Inlay Mode", |ui, mutated| {
                    if ui.selectable_value(&mut sel_male, false, "Female Cavity").clicked()
                        && is_male
                    {
                        p.inlay_mode = VCarveInlayMode::Disabled;
                        *mutated = Some("Set V-Carve Female Cavity Mode");
                    }
                    if ui
                        .selectable_value(&mut sel_male, true, "Male Plug")
                        .on_hover_text("Generates a mirrored raised male V-inlay plug with bevel engagement (start_depth) + bottom glue gap")
                        .clicked()
                        && !is_male
                    {
                        if p.flat_depth.is_none() {
                            p.flat_depth = Some(ParamLength::from_mm(3.0));
                        }
                        p.inlay_mode = VCarveInlayMode::MalePlug {
                            start_depth: ParamLength::from_mm(2.0),
                            glue_gap_bottom: ParamLength::from_mm(1.0),
                            border_margin: ParamLength::from_mm(5.0),
                            auto_mirror_x: true,
                        };
                        *mutated = Some("Enable V-Inlay Male Plug Mode");
                    }
                });

                if let VCarveInlayMode::MalePlug {
                    start_depth,
                    glue_gap_bottom,
                    border_margin,
                    auto_mirror_x,
                } = &mut p.inlay_mode
                {
                    f.set_tip("Axial engagement depth (start_depth) where male bevel mates into female cavity");
                    f.len("Start Depth", start_depth, "Edit V-Inlay Start Depth");
                    f.set_tip("Clearance gap beneath mated plug tip for glue squeeze-out");
                    f.len("Bottom Glue Gap", glue_gap_bottom, "Edit V-Inlay Glue Gap");
                    f.set_tip("Cleared frame width machined around the raised male inlay plug");
                    f.len("Border Margin", border_margin, "Edit V-Inlay Border Margin");
                    f.set_tip("Automatically mirror artwork across X so plug flips face-down into female pocket");
                    f.check("Mirror Artwork", auto_mirror_x, "Mirror X", "Toggle V-Inlay Auto Mirror X");
                }
            });

            if let VCarveInlayMode::MalePlug {
                start_depth,
                glue_gap_bottom,
                ..
            } = &p.inlay_mode
            {
                let total_mm = start_depth.value_mm + glue_gap_bottom.value_mm;
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    format!(
                        "Male plug pocket floor: {} (engagement + glue gap).",
                        item.eval_ctx.default_unit.format_length(total_mm)
                    ),
                );
            }
        },
    );
}
