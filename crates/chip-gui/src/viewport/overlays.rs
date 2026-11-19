use chip_cam::BoxAnchor27;
use chip_math::{Aabb3, RigidFrame3};

use crate::{
    camera::{CadCamera, MousePreset, OrbitMode, StandardView},
    theme::ThemeTokens,
    viewport::HighlightState,
    widgets::{IconKind, hud_toggle_pill, icon_button},
};

/// Action emitted by the Floating Viewport View HUD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewportHudAction {
    FitToViewport,
    AlignToSetupTop,
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
            if ui.small_button(lbl).clicked() {
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
            if ui.small_button(lbl).clicked() {
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

    if viewport_rect.width() < 520.0 || viewport_rect.height() < 90.0 {
        return None;
    }

    // 2. Centered Floating Camera & Visibility HUD Bar along bottom edge
    let bar_w = (viewport_rect.width() - 24.0).min(640.0);
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
        &mut highlight.ghost_part,
        None,
        "Ghost",
        "Toggle Translucent Target Part Overlay",
    );

    if bar_w >= 580.0 {
        hud_ui.separator();
        egui::ComboBox::from_id_salt("vp_hud_mouse_preset")
            .selected_text(camera.mouse_preset.short_label())
            .width(86.0)
            .show_ui(&mut hud_ui, |ui| {
                for preset in MousePreset::ALL {
                    ui.selectable_value(&mut camera.mouse_preset, preset, preset.label());
                }
            });
        let orbit_short = match camera.orbit_mode {
            OrbitMode::TurntableZUp => "+Z Up",
            OrbitMode::FreeTrackball => "Free",
        };
        let next_orbit = match camera.orbit_mode {
            OrbitMode::TurntableZUp => OrbitMode::FreeTrackball,
            OrbitMode::FreeTrackball => OrbitMode::TurntableZUp,
        };
        if hud_ui
            .small_button(orbit_short)
            .on_hover_text(camera.orbit_mode.label())
            .clicked()
        {
            camera.set_orbit_mode(next_orbit);
        }
    }

    emitted
}
