use chip_cam::{BoxAnchor27, MeasureSnapTarget, MeasurementResult, ReachAnalysisResult};
use chip_math::{Aabb3, DisplayUnit, RigidFrame3};

use crate::{
    camera::{CadCamera, StandardView},
    panels::ViewportPickMode,
    theme::ThemeTokens,
    viewport::{HighlightState, MeasureToolState},
    widgets::{IconKind, hud_toggle_pill, icon_button, inline_icon, primary_icon_button},
};

/// Action emitted by the Floating Viewport View HUD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewportHudAction {
    FitToViewport,
    AlignToSetupTop,
    ToggleMeasureMode,
}

/// Action emitted by the floating Click-to-Measure Readout Card.
#[derive(Debug, Clone, PartialEq)]
pub enum MeasureCardAction {
    CopyText(String),
    Clear,
    SetOperationBottomZ(f64),
}

/// Action emitted by the Top-Center Active Pick / Sketch Guidance HUD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidanceHudAction {
    Done,
    ResetChainStart,
    Cancel,
}

/// Interactive 27-point 3D anchor dot overlay for picking a Stock or Model WCS Origin directly
/// inside the 3D viewport (`03-setups-wcs.md`).
pub fn draw_wcs_origin_27pt_overlay(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    camera: &CadCamera,
    wcs_frame: &RigidFrame3,
    box_wcs: &Aabb3,
    active_anchor: BoxAnchor27,
) -> Option<BoxAnchor27> {
    if !box_wcs.is_valid() {
        return None;
    }
    let tok = ThemeTokens::from_ui(ui);
    let painter = ui.painter_at(viewport_rect);
    let mut picked = None;

    for (idx, anchor) in BoxAnchor27::all_27().into_iter().enumerate() {
        let p_wcs = anchor.eval_on_aabb(box_wcs);
        let p_world = wcs_frame.local_to_world_point(p_wcs);
        let Some(screen_pt) = camera.project_world_to_screen(p_world, viewport_rect) else {
            continue;
        };
        if !viewport_rect.contains(screen_pt) {
            continue;
        }

        let hit_rect = egui::Rect::from_center_size(screen_pt, egui::vec2(14.0, 14.0));
        let resp = ui.interact(
            hit_rect,
            ui.scope_id().with(("wcs_anchor_27", idx)),
            egui::Sense::click(),
        );

        let is_active = anchor == active_anchor;
        let hovered = resp.hovered();
        let (radius, fill, stroke_col) = if is_active {
            (6.2, tok.dirty_fg, tok.card_bg)
        } else if hovered {
            (5.8, tok.accent_hover, tok.accent_on_fg)
        } else {
            (4.0, tok.card_bg.gamma_multiply(0.88), tok.border_strong)
        };

        painter.circle(screen_pt, radius, fill, egui::Stroke::new(1.3, stroke_col));

        if resp
            .on_hover_text(format!("Set WCS Origin (X0 Y0 Z0): {}", anchor.label()))
            .clicked()
        {
            picked = Some(anchor);
        }
    }

    picked
}

/// Draws an interactive 3D orientation gizmo + quick-snap ViewCube buttons in the top-right corner
/// of `viewport_rect`.
pub fn draw_view_cube_overlay(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    camera: &mut CadCamera,
) {
    let box_w = 152.0;
    let box_h = 152.0;
    if viewport_rect.width() < box_w + 24.0 || viewport_rect.height() < box_h + 24.0 {
        return;
    }

    let overlay_rect = egui::Rect::from_min_size(
        egui::pos2(
            viewport_rect.right() - box_w - 10.0,
            viewport_rect.top() + 10.0,
        ),
        egui::vec2(box_w, box_h),
    );

    let tok = ThemeTokens::from_ui(ui);
    let painter = ui.painter_at(overlay_rect);
    painter.rect(
        overlay_rect,
        6.0,
        tok.hud_bg,
        egui::Stroke::new(1.0, tok.hud_border),
        egui::StrokeKind::Inside,
    );

    let center = egui::pos2(overlay_rect.center().x, overlay_rect.top() + 44.0);
    let (_, _, _, view_mat) = camera.gpu_matrices(100, 100);
    let tf = &camera.turntable_frame;
    let x_dir = glam::Vec3::new(tf.x_axis.x as f32, tf.x_axis.y as f32, tf.x_axis.z as f32);
    let y_dir = glam::Vec3::new(tf.y_axis.x as f32, tf.y_axis.y as f32, tf.y_axis.z as f32);
    let z_dir = glam::Vec3::new(tf.z_axis.x as f32, tf.z_axis.y as f32, tf.z_axis.z as f32);

    let axes = [
        (
            x_dir,
            "X",
            egui::Color32::from_rgb(242, 78, 78),
            StandardView::Right,
        ),
        (
            y_dir,
            "Y",
            egui::Color32::from_rgb(76, 218, 108),
            StandardView::Back,
        ),
        (
            z_dir,
            "Z",
            egui::Color32::from_rgb(86, 152, 250),
            StandardView::Top,
        ),
    ];

    let arm_px = 28.0_f32;
    let mut sorted_axes: Vec<_> = axes
        .into_iter()
        .map(|(dir, label, col, snap_view)| {
            let v = view_mat.transform_vector3(dir);
            (v, label, col, snap_view)
        })
        .collect();
    sorted_axes.sort_by(|a, b| a.0.z.total_cmp(&b.0.z));

    let mut clicked_view: Option<StandardView> = None;

    for (v, label, col, snap_view) in sorted_axes {
        let tip = egui::pos2(center.x + v.x * arm_px, center.y - v.y * arm_px);
        painter.line_segment([center, tip], egui::Stroke::new(2.2, col));
        let badge_rect = egui::Rect::from_center_size(tip, egui::vec2(15.0, 15.0));
        let resp = ui.interact(
            badge_rect,
            ui.scope_id().with(("viewcube_axis", label)),
            egui::Sense::click(),
        );
        let fill = if resp.hovered() {
            egui::Color32::WHITE
        } else {
            col
        };
        painter.circle_filled(tip, 7.2, fill);
        painter.text(
            tip,
            egui::Align2::CENTER_CENTER,
            label,
            egui::FontId::proportional(10.0),
            egui::Color32::from_rgb(16, 18, 22),
        );
        if resp.clicked() {
            clicked_view = Some(snap_view);
        }
    }

    let btn_area = egui::Rect::from_min_max(
        egui::pos2(overlay_rect.left() + 6.0, overlay_rect.top() + 84.0),
        egui::pos2(overlay_rect.right() - 6.0, overlay_rect.bottom() - 6.0),
    );
    let mut child_ui = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(btn_area)
            .layout(egui::Layout::top_down(egui::Align::Center)),
    );
    child_ui.spacing_mut().item_spacing = egui::vec2(4.0, 4.0);
    child_ui.horizontal(|ui| {
        for (lbl, v) in [
            ("Top", StandardView::Top),
            ("Front", StandardView::Front),
            ("Right", StandardView::Right),
        ] {
            if ui
                .small_button(lbl)
                .on_hover_text(format!("Snap 3D camera to standard {lbl} view"))
                .clicked()
            {
                clicked_view = Some(v);
            }
        }
    });
    child_ui.horizontal(|ui| {
        for (lbl, v) in [
            ("Iso", StandardView::Isometric),
            ("Left", StandardView::Left),
            ("Bot", StandardView::Bottom),
        ] {
            if ui
                .small_button(lbl)
                .on_hover_text(format!("Snap 3D camera to standard {lbl} view"))
                .clicked()
            {
                clicked_view = Some(v);
            }
        }
    });

    if let Some(v) = clicked_view {
        camera.set_standard_view(v);
    }
}

/// Draws the translucent **Floating Viewport View HUD** anchored along the bottom center of
/// `viewport_rect`, plus the non-overlapping Setup / WCS / Units pill in the bottom-left or top-left.
pub fn draw_viewport_bottom_hud(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    camera: &mut CadCamera,
    highlight: &mut HighlightState,
    pick_mode: ViewportPickMode,
    setup_summary_label: &str,
) -> Option<ViewportHudAction> {
    let tok = ThemeTokens::from_ui(ui);
    let mut emitted = None;

    // 1. Subtle Setup + WCS + Units pill in top-left
    if viewport_rect.width() > 260.0 && viewport_rect.height() > 120.0 {
        let galley = ui.painter().layout_no_wrap(
            setup_summary_label.to_string(),
            egui::FontId::proportional(11.5),
            tok.text_secondary,
        );
        let pill_size = egui::vec2(galley.size().x + 16.0, 22.0);
        let resolved_pill_rect = egui::Rect::from_min_size(
            egui::pos2(viewport_rect.left() + 10.0, viewport_rect.top() + 10.0),
            pill_size,
        );
        let p = ui.painter_at(viewport_rect);
        p.rect(
            resolved_pill_rect,
            egui::CornerRadius::same(5),
            tok.hud_bg,
            egui::Stroke::new(1.0, tok.hud_border),
            egui::StrokeKind::Inside,
        );
        p.text(
            resolved_pill_rect.center(),
            egui::Align2::CENTER_CENTER,
            setup_summary_label,
            egui::FontId::proportional(11.5),
            tok.text_secondary,
        );
    }

    if viewport_rect.width() < 420.0 || viewport_rect.height() < 90.0 {
        return None;
    }

    // 2. Centered Floating Camera & Visibility HUD Bar along bottom edge
    let bar_w = (viewport_rect.width() - 24.0).min(710.0);
    let bar_h = 32.0;
    let bar_rect = egui::Rect::from_center_size(
        egui::pos2(viewport_rect.center().x, viewport_rect.bottom() - 21.0),
        egui::vec2(bar_w, bar_h),
    );

    ui.painter_at(viewport_rect).rect(
        bar_rect,
        egui::CornerRadius::same(7),
        tok.hud_bg,
        egui::Stroke::new(1.0, tok.hud_border),
        egui::StrokeKind::Inside,
    );

    let inner = bar_rect.shrink2(egui::vec2(8.0, 4.0));
    let mut hud_ui = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(inner)
            .layout(egui::Layout::left_to_right(egui::Align::Center)),
    );
    hud_ui.spacing_mut().item_spacing = egui::vec2(5.0, 2.0);

    let proj_lbl = if camera.projection.is_orthographic() {
        "Ortho"
    } else {
        "Persp"
    };
    if icon_button(
        &mut hud_ui,
        IconKind::OrthoCube,
        Some(proj_lbl),
        "Toggle Orthographic / Perspective Camera (P)",
    )
    .clicked()
    {
        camera.toggle_projection();
    }
    if icon_button(
        &mut hud_ui,
        IconKind::FitView,
        Some("Fit"),
        "Fit Scene to Viewport (F)",
    )
    .clicked()
    {
        emitted = Some(ViewportHudAction::FitToViewport);
    }
    if icon_button(
        &mut hud_ui,
        IconKind::TopView,
        Some("Top"),
        "Align Camera to Active Setup +Z Top",
    )
    .clicked()
    {
        emitted = Some(ViewportHudAction::AlignToSetupTop);
    }

    hud_ui.separator();

    let paths_icon = if highlight.show_toolpaths {
        IconKind::Eye
    } else {
        IconKind::EyeOff
    };
    let _ = hud_toggle_pill(
        &mut hud_ui,
        &mut highlight.show_toolpaths,
        Some(paths_icon),
        "Paths",
        "Toggle 3D Generated Toolpaths",
    );
    let _ = hud_toggle_pill(
        &mut hud_ui,
        &mut highlight.show_rapids,
        None,
        "Rapids",
        "Toggle G0 Rapid Moves",
    );
    let _ = hud_toggle_pill(
        &mut hud_ui,
        &mut highlight.show_stock,
        None,
        "Stock",
        "Toggle Translucent Setup Stock",
    );
    let _ = hud_toggle_pill(
        &mut hud_ui,
        &mut highlight.show_fixtures,
        None,
        "Fixtures",
        "Toggle Translucent 3D Workholding Fixtures (Vises, Clamps, Spoilboard)",
    );
    let _ = hud_toggle_pill(
        &mut hud_ui,
        &mut highlight.ghost_part,
        None,
        "Ghost",
        "Toggle Translucent Target Part Overlay",
    );

    hud_ui.separator();

    let mut measure_active = pick_mode == ViewportPickMode::Measure;
    if hud_toggle_pill(
        &mut hud_ui,
        &mut measure_active,
        Some(IconKind::Crosshair),
        "Measure",
        "Click-to-Measure Tool (M): inspect distances, diameters, depths, and face angles",
    ) {
        emitted = Some(ViewportHudAction::ToggleMeasureMode);
    }
    let _ = hud_toggle_pill(
        &mut hud_ui,
        &mut highlight.show_reach_heatmap,
        None,
        "Reach / Min-R",
        "Toggle Minimum-Radius & Flute/Stickout Reach Heatmap Overlay",
    );

    emitted
}

/// Draws a prominent top-center **Active Pick & Sketch Guidance HUD** whenever a 3D B-Rep pick mode,
/// manual holding-tab mode, or 2D sketch authoring tool is active (`08-ui-viewport.md`).
pub fn draw_viewport_guidance_hud(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    pick_mode: ViewportPickMode,
    context_badge: Option<&str>,
    avoid_target_label: Option<&str>,
) -> Option<GuidanceHudAction> {
    let (title, instruction, supports_done) = pick_mode.guidance_info()?;
    if viewport_rect.width() < 340.0 || viewport_rect.height() < 120.0 {
        return None;
    }

    let tok = ThemeTokens::from_ui(ui);
    let is_avoid_pick = avoid_target_label.is_some();
    let accent_stroke = if is_avoid_pick {
        tok.error_border
    } else {
        tok.accent
    };
    let accent_icon = if is_avoid_pick {
        tok.error_fg
    } else {
        tok.accent
    };
    // Reserve space for top-left Setup pill and top-right ViewCube (`162px`)
    let max_card_w = (viewport_rect.width() - 340.0).clamp(290.0, 560.0);
    let card_h = 48.0;
    let cx = (viewport_rect.center().x).min(viewport_rect.right() - 168.0 - 0.5 * max_card_w);
    let card_rect = egui::Rect::from_center_size(
        egui::pos2(
            cx.max(viewport_rect.left() + 0.5 * max_card_w + 10.0),
            viewport_rect.top() + 34.0,
        ),
        egui::vec2(max_card_w, card_h),
    );

    ui.painter_at(viewport_rect).rect(
        card_rect,
        egui::CornerRadius::same(7),
        tok.hud_bg,
        egui::Stroke::new(if is_avoid_pick { 1.6 } else { 1.2 }, accent_stroke),
        egui::StrokeKind::Inside,
    );

    let inner = card_rect.shrink2(egui::vec2(10.0, 5.0));
    let mut hud_ui = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(inner)
            .layout(egui::Layout::top_down(egui::Align::Min)),
    );
    hud_ui.spacing_mut().item_spacing = egui::vec2(6.0, 2.0);

    let mut action = None;
    let icon = match pick_mode {
        ViewportPickMode::SketchAuthor { .. } => IconKind::Sketch2D,
        _ => IconKind::Crosshair,
    };
    let has_chain_start = matches!(
        pick_mode,
        ViewportPickMode::OpOpenChain {
            start_edge: Some(_),
            ..
        }
    );

    hud_ui.horizontal(|ui| {
        inline_icon(ui, icon, accent_icon, 14.0);
        if let Some(avoid_lbl) = avoid_target_label {
            ui.colored_label(
                tok.error_fg,
                egui::RichText::new(format!("[{avoid_lbl}]")).strong(),
            );
        }
        ui.strong(title);
        if let Some(badge) = context_badge {
            ui.colored_label(
                if is_avoid_pick {
                    tok.error_fg
                } else {
                    tok.info_fg
                },
                format!("· {badge}"),
            );
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            if ui
                .small_button("Cancel (Esc)")
                .on_hover_text("Exit active pick/draw mode (Esc)")
                .clicked()
            {
                action = Some(GuidanceHudAction::Cancel);
            }
            if has_chain_start
                && ui
                    .small_button("Reset 1st")
                    .on_hover_text("Clear first chain edge and re-pick Start Edge")
                    .clicked()
            {
                action = Some(GuidanceHudAction::ResetChainStart);
            }
            if supports_done
                && primary_icon_button(ui, IconKind::StatusClean, "Done", "Finish selection")
                    .clicked()
            {
                action = Some(GuidanceHudAction::Done);
            }
        });
    });

    hud_ui.add(
        egui::Label::new(
            egui::RichText::new(instruction)
                .size(11.5)
                .color(tok.text_secondary),
        )
        .truncate(),
    );

    action
}

/// Draws a live hover snap tag near the hovered 3D anchor in Measure mode (plain ASCII only).
pub fn draw_measure_hover_label(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    camera: &CadCamera,
    wcs_frame: &RigidFrame3,
    target: &MeasureSnapTarget,
    unit: DisplayUnit,
) {
    let world_pt = wcs_frame.local_to_world_point(target.anchor_wcs());
    let Some(screen_pt) = camera.project_world_to_screen(world_pt, viewport_rect) else {
        return;
    };
    if !viewport_rect.contains(screen_pt) {
        return;
    }
    let tok = ThemeTokens::from_ui(ui);
    let text = match target {
        MeasureSnapTarget::BRepVertex { pos_wcs }
        | MeasureSnapTarget::SketchPoint { pos_wcs, .. }
        | MeasureSnapTarget::MeshPoint { pos_wcs, .. } => format!(
            "[{}] Z={}",
            target.short_label(),
            unit.format_length(pos_wcs.z)
        ),
        MeasureSnapTarget::BRepEdge {
            arc_radius_mm: Some(r),
            ..
        } => format!("[{}] R={}", target.short_label(), unit.format_length(*r)),
        MeasureSnapTarget::BRepEdge { length_mm, .. } => {
            format!(
                "[{}] L={}",
                target.short_label(),
                unit.format_length(*length_mm)
            )
        }
        MeasureSnapTarget::BRepPlanarFace { z_wcs, .. } => {
            format!(
                "[{}] Z={}",
                target.short_label(),
                unit.format_length(*z_wcs)
            )
        }
        MeasureSnapTarget::BRepCylindricalFace { diameter_mm, .. }
        | MeasureSnapTarget::SketchCircle { diameter_mm, .. } => format!(
            "[{}] Dia={}",
            target.short_label(),
            unit.format_length(*diameter_mm)
        ),
    };
    let painter = ui.painter_at(viewport_rect);
    let galley = painter.layout_no_wrap(text, egui::FontId::monospace(11.5), tok.text_primary);
    let pos = egui::pos2(screen_pt.x + 12.0, screen_pt.y - 26.0);
    let bg_rect = egui::Rect::from_min_size(pos, galley.size() + egui::vec2(10.0, 6.0));
    painter.rect(
        bg_rect,
        egui::CornerRadius::same(4),
        tok.hud_bg,
        egui::Stroke::new(1.0, tok.info_border),
        egui::StrokeKind::Inside,
    );
    painter.galley(pos + egui::vec2(5.0, 3.0), galley, tok.text_primary);
}

/// Draws the floating **Measurement Readout Card** in the bottom-left of the viewport when
/// Click-to-Measure has an active inspection or two-entity measurement (plain ASCII only).
pub fn draw_measure_readout_card(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    measure: &MeasureToolState,
    unit: DisplayUnit,
    focused_op_name: Option<&str>,
) -> Option<MeasureCardAction> {
    let result = measure.result.as_ref()?;
    if viewport_rect.width() < 340.0 || viewport_rect.height() < 180.0 {
        return None;
    }

    let tok = ThemeTokens::from_ui(ui);
    let card_w = (viewport_rect.width() - 24.0).clamp(260.0, 332.0);
    let card_h = if focused_op_name.is_some() && measure.suggested_bottom_z_wcs().is_some() {
        164.0
    } else {
        142.0
    };
    let card_rect = egui::Rect::from_min_size(
        egui::pos2(
            viewport_rect.left() + 12.0,
            viewport_rect.bottom() - card_h - 46.0,
        ),
        egui::vec2(card_w, card_h),
    );

    ui.painter_at(viewport_rect).rect(
        card_rect,
        egui::CornerRadius::same(7),
        tok.hud_bg,
        egui::Stroke::new(1.2, tok.info_border),
        egui::StrokeKind::Inside,
    );

    let inner = card_rect.shrink2(egui::vec2(10.0, 8.0));
    let mut card_ui = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(inner)
            .layout(egui::Layout::top_down(egui::Align::Min)),
    );
    card_ui.spacing_mut().item_spacing = egui::vec2(6.0, 3.0);

    let mut action = None;

    let heading = match result {
        MeasurementResult::SingleEntity { target, .. } => {
            format!("Inspect: {}", target.short_label())
        }
        MeasurementResult::PointToPoint { .. } => "Measure: Point to Point".to_string(),
        MeasurementResult::ParallelPlanes { .. } => "Measure: Parallel Planes".to_string(),
        MeasurementResult::AngledPlanes { .. } => "Measure: Angled Planes".to_string(),
        MeasurementResult::CylinderToCylinder { .. } => "Measure: Bore to Bore".to_string(),
        MeasurementResult::PointToPlane { .. } => "Measure: Point to Plane".to_string(),
    };

    card_ui.horizontal(|ui| {
        inline_icon(ui, IconKind::Crosshair, tok.info_fg, 14.0);
        ui.strong(
            egui::RichText::new(heading)
                .size(12.5)
                .color(tok.text_primary),
        );
    });
    card_ui.separator();

    let fmt = |v_mm: f64| unit.format_length(v_mm);
    match result {
        MeasurementResult::SingleEntity { target, .. } => match target {
            MeasureSnapTarget::BRepVertex { pos_wcs }
            | MeasureSnapTarget::SketchPoint { pos_wcs, .. }
            | MeasureSnapTarget::MeshPoint { pos_wcs, .. } => {
                measure_kv_row(
                    &mut card_ui,
                    "WCS X / Y",
                    &format!("{} , {}", fmt(pos_wcs.x), fmt(pos_wcs.y)),
                    tok,
                );
                measure_kv_row(&mut card_ui, "WCS Z", &fmt(pos_wcs.z), tok);
            }
            MeasureSnapTarget::BRepEdge {
                length_mm,
                arc_radius_mm,
                ..
            } => {
                if let Some(r) = arc_radius_mm {
                    measure_kv_row(
                        &mut card_ui,
                        "Radius / Dia",
                        &format!("R {} (Dia {})", fmt(*r), fmt(2.0 * *r)),
                        tok,
                    );
                }
                measure_kv_row(&mut card_ui, "Edge Length", &fmt(*length_mm), tok);
            }
            MeasureSnapTarget::BRepPlanarFace {
                z_wcs, area_mm2, ..
            } => {
                measure_kv_row(&mut card_ui, "Face Z (WCS)", &fmt(*z_wcs), tok);
                measure_kv_row(&mut card_ui, "Area", &format!("{area_mm2:.2} mm^2"), tok);
            }
            MeasureSnapTarget::BRepCylindricalFace {
                diameter_mm,
                depth_mm,
                top_z_wcs,
                bottom_z_wcs,
                ..
            } => {
                measure_kv_row(
                    &mut card_ui,
                    "Dia / Radius",
                    &format!("{} (R {})", fmt(*diameter_mm), fmt(0.5 * *diameter_mm)),
                    tok,
                );
                measure_kv_row(
                    &mut card_ui,
                    "Depth / Z",
                    &format!(
                        "{} [{} .. {}]",
                        fmt(*depth_mm),
                        fmt(*bottom_z_wcs),
                        fmt(*top_z_wcs)
                    ),
                    tok,
                );
            }
            MeasureSnapTarget::SketchCircle {
                center_wcs,
                diameter_mm,
                ..
            } => {
                measure_kv_row(
                    &mut card_ui,
                    "Dia / Radius",
                    &format!("{} (R {})", fmt(*diameter_mm), fmt(0.5 * *diameter_mm)),
                    tok,
                );
                measure_kv_row(
                    &mut card_ui,
                    "Center XY",
                    &format!("{} , {}", fmt(center_wcs.x), fmt(center_wcs.y)),
                    tok,
                );
            }
        },
        MeasurementResult::PointToPoint {
            distance_3d_mm,
            delta_wcs,
            distance_xy_mm,
            ..
        } => {
            measure_kv_row(
                &mut card_ui,
                "Dist 3D / XY",
                &format!("{}  (XY {})", fmt(*distance_3d_mm), fmt(*distance_xy_mm)),
                tok,
            );
            measure_kv_row(
                &mut card_ui,
                "dX / dY / dZ",
                &format!(
                    "{} / {} / {}",
                    fmt(delta_wcs.x),
                    fmt(delta_wcs.y),
                    fmt(delta_wcs.z)
                ),
                tok,
            );
        }
        MeasurementResult::ParallelPlanes {
            perpendicular_gap_mm,
            delta_z_wcs,
            distance_3d_mm,
            ..
        } => {
            measure_kv_row(
                &mut card_ui,
                "Perp Gap",
                &format!("{}  (dZ {})", fmt(*perpendicular_gap_mm), fmt(*delta_z_wcs)),
                tok,
            );
            measure_kv_row(&mut card_ui, "Dist 3D", &fmt(*distance_3d_mm), tok);
        }
        MeasurementResult::AngledPlanes {
            acute_angle_deg,
            supplementary_angle_deg,
            ..
        } => {
            measure_kv_row(
                &mut card_ui,
                "Dihedral",
                &format!("{acute_angle_deg:.3} deg"),
                tok,
            );
            measure_kv_row(
                &mut card_ui,
                "Supplement",
                &format!("{supplementary_angle_deg:.3} deg"),
                tok,
            );
        }
        MeasurementResult::CylinderToCylinder {
            diameter1_mm,
            diameter2_mm,
            center_distance_3d_mm,
            center_distance_xy_mm,
            min_wall_thickness_mm,
            ..
        } => {
            measure_kv_row(
                &mut card_ui,
                "Pitch XY / 3D",
                &format!(
                    "{}  (3D {})",
                    fmt(*center_distance_xy_mm),
                    fmt(*center_distance_3d_mm)
                ),
                tok,
            );
            let wall_str = min_wall_thickness_mm
                .map(|w| format!("Wall {}", fmt(w)))
                .unwrap_or_else(|| "Skew axes".to_string());
            measure_kv_row(
                &mut card_ui,
                "Dia1 / Dia2",
                &format!(
                    "{} / {} | {wall_str}",
                    fmt(*diameter1_mm),
                    fmt(*diameter2_mm)
                ),
                tok,
            );
        }
        MeasurementResult::PointToPlane {
            abs_distance_mm,
            signed_distance_mm,
            delta_z_wcs,
            ..
        } => {
            measure_kv_row(
                &mut card_ui,
                "Perp Dist",
                &format!(
                    "{} (signed {})",
                    fmt(*abs_distance_mm),
                    fmt(*signed_distance_mm)
                ),
                tok,
            );
            measure_kv_row(&mut card_ui, "dZ (WCS)", &fmt(*delta_z_wcs), tok);
        }
    }

    card_ui.add_space(2.0);
    card_ui.horizontal(|ui| {
        if ui
            .small_button("Copy")
            .on_hover_text("Copy formatted measurement summary to clipboard")
            .clicked()
        {
            action = Some(MeasureCardAction::CopyText(result.format_copy_text(unit)));
        }
        if ui
            .small_button("Clear")
            .on_hover_text("Clear anchors and start a new measurement")
            .clicked()
        {
            action = Some(MeasureCardAction::Clear);
        }
        if let (Some(op_name), Some(z_wcs)) = (focused_op_name, measure.suggested_bottom_z_wcs())
            && ui
                .small_button("Set Op Bottom Z")
                .on_hover_text(format!(
                    "Set Bottom Height of '{}' to WCS Z = {} (atomic Undo)",
                    op_name,
                    fmt(z_wcs)
                ))
                .clicked()
        {
            action = Some(MeasureCardAction::SetOperationBottomZ(z_wcs));
        }
    });

    action
}

fn measure_kv_row(ui: &mut egui::Ui, key: &str, val: &str, tok: ThemeTokens) {
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("{key}:"))
                .size(11.5)
                .color(tok.text_secondary),
        );
        ui.label(
            egui::RichText::new(val)
                .monospace()
                .size(11.5)
                .color(tok.text_primary),
        );
    });
}

/// Draws the compact **Minimum-Radius & Reach Heatmap Legend Pill** below the top-right ViewCube
/// whenever the Reach / Min-R overlay is enabled (plain ASCII only).
pub fn draw_reach_legend_overlay(
    ui: &mut egui::Ui,
    viewport_rect: egui::Rect,
    reach: Option<&ReachAnalysisResult>,
    is_computing: bool,
    unit: DisplayUnit,
) {
    let box_w = 188.0;
    let box_h = 104.0;
    if viewport_rect.width() < box_w + 24.0 || viewport_rect.height() < 290.0 {
        return;
    }

    let overlay_rect = egui::Rect::from_min_size(
        egui::pos2(
            viewport_rect.right() - box_w - 10.0,
            viewport_rect.top() + 170.0,
        ),
        egui::vec2(box_w, box_h),
    );

    let tok = ThemeTokens::from_ui(ui);
    let painter = ui.painter_at(overlay_rect);
    painter.rect(
        overlay_rect,
        egui::CornerRadius::same(6),
        tok.hud_bg,
        egui::Stroke::new(1.0, tok.hud_border),
        egui::StrokeKind::Inside,
    );

    let inner = overlay_rect.shrink2(egui::vec2(8.0, 6.0));
    let mut leg_ui = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(inner)
            .layout(egui::Layout::top_down(egui::Align::Min)),
    );
    leg_ui.spacing_mut().item_spacing = egui::vec2(4.0, 2.5);

    leg_ui.label(
        egui::RichText::new("Reach / Min-R Analysis")
            .size(11.5)
            .strong()
            .color(tok.text_primary),
    );

    if is_computing && reach.is_none() {
        leg_ui.label(
            egui::RichText::new("Analyzing geometry...")
                .size(11.0)
                .color(tok.text_secondary),
        );
        return;
    }

    let Some(res) = reach else {
        leg_ui.label(
            egui::RichText::new("Load model + tool to analyze")
                .size(11.0)
                .color(tok.text_muted),
        );
        return;
    };

    let min_r_str = match res.min_internal_radius_mm {
        Some(r) if r <= 1e-4 => "Sharp (0)".to_string(),
        Some(r) => unit.format_length(r),
        None => "None".to_string(),
    };
    leg_ui.label(
        egui::RichText::new(format!(
            "Tool Dia {} | Min R: {}",
            unit.format_length(res.tool_diameter_mm),
            min_r_str
        ))
        .size(10.5)
        .monospace()
        .color(tok.text_secondary),
    );
    leg_ui.label(
        egui::RichText::new(format!(
            "Max Depth: {} (LOC {})",
            unit.format_length(res.max_feature_depth_mm),
            unit.format_length(res.tool_effective_reach_mm)
        ))
        .size(10.5)
        .monospace()
        .color(tok.text_secondary),
    );

    let amber_col = egui::Color32::from_rgb(245, 168, 24);
    let crimson_col = egui::Color32::from_rgb(236, 44, 56);
    draw_legend_swatch_row(
        &mut leg_ui,
        amber_col,
        &format!("Amber: {} (Tight R / LOC)", res.amber_count),
        tok.text_primary,
    );
    draw_legend_swatch_row(
        &mut leg_ui,
        crimson_col,
        &format!("Crimson: {} (Blocked / Reach)", res.crimson_count),
        tok.text_primary,
    );
}

fn draw_legend_swatch_row(
    ui: &mut egui::Ui,
    swatch: egui::Color32,
    label: &str,
    fg: egui::Color32,
) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
        ui.painter()
            .rect_filled(rect, egui::CornerRadius::same(2), swatch);
        ui.label(egui::RichText::new(label).size(10.5).color(fg));
    });
}
