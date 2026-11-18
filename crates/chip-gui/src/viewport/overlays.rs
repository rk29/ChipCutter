use chip_cam::BoxAnchor27;
use chip_math::{Aabb3, RigidFrame3};

use crate::camera::{CadCamera, StandardView};

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
            (
                6.2,
                egui::Color32::from_rgb(255, 204, 56),
                egui::Color32::WHITE,
            )
        } else if hovered {
            (
                5.8,
                egui::Color32::from_rgb(92, 224, 255),
                egui::Color32::WHITE,
            )
        } else {
            (
                4.0,
                egui::Color32::from_rgba_unmultiplied(215, 225, 240, 175),
                egui::Color32::from_rgb(28, 34, 42),
            )
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

    let painter = ui.painter_at(overlay_rect);
    painter.rect(
        overlay_rect,
        6.0,
        egui::Color32::from_rgba_unmultiplied(20, 24, 30, 210),
        egui::Stroke::new(1.0, egui::Color32::from_rgb(58, 66, 78)),
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
