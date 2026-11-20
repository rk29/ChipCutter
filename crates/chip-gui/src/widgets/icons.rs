use std::f32::consts::{FRAC_PI_4, PI, TAU};

use chip_cam::OperationKind;
use egui::{Color32, CornerRadius, Pos2, Rect, Response, Sense, Stroke, StrokeKind, Ui, Vec2};

use crate::theme::ThemeTokens;

/// Semantic status badge variants for CAM Browser rows, Diagnostics, and Simulation bars.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusBadgeKind {
    Clean,
    Dirty,
    Error,
    NeedsRepick,
    Suppressed,
}

/// Resolution-independent procedural vector icons rendered directly via `egui::Painter`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IconKind {
    // Status badges
    StatusClean,
    StatusDirty,
    StatusError,
    StatusNeedsRepick,
    StatusSuppressed,

    // Operations & entities
    Facing,
    Contour2D,
    Pocket2D,
    Adaptive2D,
    DrillBore,
    ChamferTrace,
    VCarve,
    Sketch2D,
    StepModel,
    Tool,

    // Workflow & file actions
    FolderOpen,
    Save,
    BundleZip,
    VectorImport,
    Reload,
    Undo,
    Redo,
    Plus,
    Generate,
    Simulate,
    PostGCode,
    CancelStop,

    // Simulation transport
    Play,
    Pause,
    SkipStart,
    StepBack,
    StepForward,
    SkipNextOp,
    SkipEnd,

    // Viewport & theme controls
    Sun,
    Moon,
    Eye,
    EyeOff,
    FitView,
    TopView,
    OrthoCube,
    Trash,
    Duplicate,
    InfoCircle,
    Search,
    Sparkles,
    Keyboard,
    Crosshair,
    ArrowUp,
    ArrowDown,
}

impl IconKind {
    pub const fn for_operation(kind: OperationKind, is_adaptive: bool) -> Self {
        if is_adaptive {
            return Self::Adaptive2D;
        }
        match kind {
            OperationKind::Facing => Self::Facing,
            OperationKind::Contour2D => Self::Contour2D,
            OperationKind::Pocket2D => Self::Pocket2D,
            OperationKind::DrillBore => Self::DrillBore,
            OperationKind::ChamferTrace => Self::ChamferTrace,
            OperationKind::VCarve => Self::VCarve,
            OperationKind::Adaptive3D => Self::Adaptive2D,
            OperationKind::Waterline3D => Self::Contour2D,
            OperationKind::Parallel3D => Self::Facing,
            OperationKind::Scallop3D => Self::Pocket2D,
            OperationKind::Pencil3D => Self::ChamferTrace,
            OperationKind::Rotary2Linear => Self::OrthoCube,
        }
    }

    pub const fn for_status(status: StatusBadgeKind) -> Self {
        match status {
            StatusBadgeKind::Clean => Self::StatusClean,
            StatusBadgeKind::Dirty => Self::StatusDirty,
            StatusBadgeKind::Error => Self::StatusError,
            StatusBadgeKind::NeedsRepick => Self::StatusNeedsRepick,
            StatusBadgeKind::Suppressed => Self::StatusSuppressed,
        }
    }
}

#[inline]
fn map_uv(rect: Rect, u: f32, v: f32) -> Pos2 {
    Pos2::new(
        rect.left() + u * rect.width(),
        rect.top() + v * rect.height(),
    )
}

fn draw_arc(
    painter: &egui::Painter,
    center: Pos2,
    radius: f32,
    start_rad: f32,
    end_rad: f32,
    steps: usize,
    stroke: Stroke,
) {
    let n = steps.max(4);
    let pts: Vec<Pos2> = (0..=n)
        .map(|i| {
            let t = (i as f32) / (n as f32);
            let a = start_rad + (end_rad - start_rad) * t;
            Pos2::new(center.x + radius * a.cos(), center.y + radius * a.sin())
        })
        .collect();
    painter.add(egui::Shape::line(pts, stroke));
}

/// Renders `icon` inside `rect` using anti-aliased `egui::Painter` primitives.
pub fn paint_icon(painter: &egui::Painter, rect: Rect, icon: IconKind, color: Color32) {
    let s = rect.width().min(rect.height()).max(8.0);
    let sq = Rect::from_center_size(rect.center(), Vec2::splat(s));
    let sw = (s * 0.115).clamp(1.25, 2.2);
    let stroke = Stroke::new(sw, color);
    let thin = Stroke::new((sw * 0.82).max(1.05), color);
    let p = |u: f32, v: f32| map_uv(sq, u, v);

    match icon {
        IconKind::StatusClean => {
            painter.circle_stroke(sq.center(), s * 0.42, thin);
            painter.add(egui::Shape::line(
                vec![p(0.28, 0.52), p(0.44, 0.68), p(0.73, 0.35)],
                stroke,
            ));
        }
        IconKind::StatusDirty => {
            draw_arc(
                painter,
                sq.center(),
                s * 0.36,
                -0.85 * PI,
                0.05 * PI,
                10,
                stroke,
            );
            draw_arc(
                painter,
                sq.center(),
                s * 0.36,
                0.15 * PI,
                1.05 * PI,
                10,
                stroke,
            );
            painter.add(egui::Shape::line(
                vec![p(0.72, 0.34), p(0.86, 0.52), p(0.68, 0.54)],
                thin,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.28, 0.66), p(0.14, 0.48), p(0.32, 0.46)],
                thin,
            ));
        }
        IconKind::StatusError => {
            painter.add(egui::Shape::closed_line(
                vec![p(0.50, 0.14), p(0.88, 0.84), p(0.12, 0.84)],
                stroke,
            ));
            painter.line_segment([p(0.50, 0.39), p(0.50, 0.62)], stroke);
            painter.circle_filled(p(0.50, 0.74), sw * 0.65, color);
        }
        IconKind::StatusNeedsRepick => {
            painter.circle_stroke(sq.center(), s * 0.34, thin);
            painter.line_segment([p(0.50, 0.08), p(0.50, 0.28)], thin);
            painter.line_segment([p(0.50, 0.72), p(0.50, 0.92)], thin);
            painter.line_segment([p(0.08, 0.50), p(0.28, 0.50)], thin);
            painter.line_segment([p(0.72, 0.50), p(0.92, 0.50)], thin);
            painter.circle_filled(sq.center(), sw * 0.75, color);
        }
        IconKind::StatusSuppressed => {
            painter.circle_stroke(sq.center(), s * 0.38, stroke);
            painter.line_segment([p(0.24, 0.76), p(0.76, 0.24)], stroke);
        }
        IconKind::Facing => {
            painter.add(egui::Shape::line(
                vec![
                    p(0.14, 0.24),
                    p(0.86, 0.24),
                    p(0.86, 0.50),
                    p(0.14, 0.50),
                    p(0.14, 0.76),
                    p(0.86, 0.76),
                ],
                stroke,
            ));
        }
        IconKind::Contour2D => {
            let r_out = Rect::from_min_max(p(0.16, 0.18), p(0.84, 0.82));
            painter.rect_stroke(r_out, CornerRadius::same(2), stroke, StrokeKind::Inside);
            painter.line_segment([p(0.50, 0.08), p(0.50, 0.24)], thin);
        }
        IconKind::Pocket2D => {
            let r1 = Rect::from_min_max(p(0.14, 0.16), p(0.86, 0.84));
            let r2 = Rect::from_min_max(p(0.32, 0.34), p(0.68, 0.66));
            painter.rect_stroke(r1, CornerRadius::same(2), stroke, StrokeKind::Inside);
            painter.rect_stroke(r2, CornerRadius::same(1), thin, StrokeKind::Inside);
        }
        IconKind::Adaptive2D => {
            let mut spiral = Vec::with_capacity(25);
            for i in 0..=24 {
                let t = (i as f32) / 24.0;
                let ang = t * 2.35 * TAU;
                let rad = s * (0.08 + 0.34 * t);
                spiral.push(Pos2::new(
                    sq.center().x + rad * ang.cos(),
                    sq.center().y + rad * ang.sin(),
                ));
            }
            painter.add(egui::Shape::line(spiral, thin));
        }
        IconKind::DrillBore => {
            painter.circle_stroke(sq.center(), s * 0.32, stroke);
            painter.line_segment([p(0.50, 0.10), p(0.50, 0.90)], thin);
            painter.line_segment([p(0.10, 0.50), p(0.90, 0.50)], thin);
        }
        IconKind::ChamferTrace => {
            painter.add(egui::Shape::line(
                vec![p(0.16, 0.78), p(0.46, 0.78), p(0.82, 0.42), p(0.82, 0.20)],
                stroke,
            ));
            painter.line_segment([p(0.42, 0.26), p(0.74, 0.58)], thin);
        }
        IconKind::VCarve => {
            painter.line_segment([p(0.12, 0.24), p(0.88, 0.24)], thin);
            painter.add(egui::Shape::line(
                vec![p(0.20, 0.24), p(0.50, 0.82), p(0.80, 0.24)],
                stroke,
            ));
        }
        IconKind::Sketch2D => {
            let r = Rect::from_min_max(p(0.14, 0.28), p(0.66, 0.80));
            painter.rect_stroke(r, CornerRadius::same(2), thin, StrokeKind::Inside);
            painter.circle_stroke(p(0.66, 0.36), s * 0.20, stroke);
        }
        IconKind::StepModel | IconKind::OrthoCube => {
            let top = vec![p(0.50, 0.14), p(0.84, 0.32), p(0.50, 0.50), p(0.16, 0.32)];
            painter.add(egui::Shape::closed_line(top, thin));
            painter.add(egui::Shape::line(
                vec![
                    p(0.16, 0.32),
                    p(0.16, 0.68),
                    p(0.50, 0.86),
                    p(0.84, 0.68),
                    p(0.84, 0.32),
                ],
                stroke,
            ));
            painter.line_segment([p(0.50, 0.50), p(0.50, 0.86)], thin);
        }
        IconKind::Tool => {
            let shank = Rect::from_min_max(p(0.38, 0.12), p(0.62, 0.48));
            let flutes = Rect::from_min_max(p(0.32, 0.48), p(0.68, 0.86));
            painter.rect_stroke(shank, CornerRadius::ZERO, thin, StrokeKind::Inside);
            painter.rect_stroke(flutes, CornerRadius::ZERO, stroke, StrokeKind::Inside);
            painter.line_segment([p(0.34, 0.62), p(0.66, 0.74)], thin);
        }
        IconKind::FolderOpen => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.12, 0.26),
                    p(0.40, 0.26),
                    p(0.48, 0.36),
                    p(0.86, 0.36),
                    p(0.86, 0.80),
                    p(0.12, 0.80),
                ],
                stroke,
            ));
        }
        IconKind::Save => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.16, 0.16),
                    p(0.72, 0.16),
                    p(0.84, 0.28),
                    p(0.84, 0.84),
                    p(0.16, 0.84),
                ],
                stroke,
            ));
            let hub = Rect::from_min_max(p(0.32, 0.54), p(0.68, 0.84));
            painter.rect_stroke(hub, CornerRadius::same(1), thin, StrokeKind::Inside);
        }
        IconKind::BundleZip => {
            let box_r = Rect::from_min_max(p(0.16, 0.22), p(0.84, 0.84));
            painter.rect_stroke(box_r, CornerRadius::same(2), stroke, StrokeKind::Inside);
            painter.line_segment([p(0.16, 0.42), p(0.84, 0.42)], thin);
            painter.line_segment([p(0.50, 0.22), p(0.50, 0.62)], thin);
        }
        IconKind::VectorImport => {
            draw_arc(
                painter,
                p(0.52, 0.58),
                s * 0.32,
                -0.92 * PI,
                -0.08 * PI,
                10,
                stroke,
            );
            painter.rect_filled(
                Rect::from_center_size(p(0.22, 0.54), Vec2::splat(sw * 1.9)),
                0.0,
                color,
            );
            painter.rect_filled(
                Rect::from_center_size(p(0.82, 0.54), Vec2::splat(sw * 1.9)),
                0.0,
                color,
            );
            painter.circle_filled(p(0.52, 0.26), sw * 1.1, color);
        }
        IconKind::Reload => {
            draw_arc(
                painter,
                sq.center(),
                s * 0.34,
                -0.75 * PI,
                0.85 * PI,
                14,
                stroke,
            );
            painter.add(egui::Shape::line(
                vec![p(0.66, 0.16), p(0.84, 0.28), p(0.64, 0.36)],
                stroke,
            ));
        }
        IconKind::Undo => {
            draw_arc(
                painter,
                p(0.54, 0.58),
                s * 0.28,
                -0.85 * PI,
                0.35 * PI,
                12,
                stroke,
            );
            painter.add(egui::Shape::line(
                vec![p(0.36, 0.20), p(0.18, 0.36), p(0.38, 0.46)],
                stroke,
            ));
        }
        IconKind::Redo => {
            draw_arc(
                painter,
                p(0.46, 0.58),
                s * 0.28,
                0.65 * PI,
                1.85 * PI,
                12,
                stroke,
            );
            painter.add(egui::Shape::line(
                vec![p(0.64, 0.20), p(0.82, 0.36), p(0.62, 0.46)],
                stroke,
            ));
        }
        IconKind::Plus => {
            painter.line_segment([p(0.50, 0.20), p(0.50, 0.80)], stroke);
            painter.line_segment([p(0.20, 0.50), p(0.80, 0.50)], stroke);
        }
        IconKind::Generate => {
            painter.add(egui::Shape::convex_polygon(
                vec![
                    p(0.56, 0.10),
                    p(0.24, 0.54),
                    p(0.48, 0.54),
                    p(0.42, 0.90),
                    p(0.76, 0.44),
                    p(0.52, 0.44),
                ],
                color,
                Stroke::NONE,
            ));
        }
        IconKind::Simulate | IconKind::Play => {
            painter.add(egui::Shape::convex_polygon(
                vec![p(0.28, 0.20), p(0.80, 0.50), p(0.28, 0.80)],
                color,
                Stroke::NONE,
            ));
        }
        IconKind::Pause => {
            let l = Rect::from_min_max(p(0.26, 0.20), p(0.42, 0.80));
            let r = Rect::from_min_max(p(0.58, 0.20), p(0.74, 0.80));
            painter.rect_filled(l, CornerRadius::same(1), color);
            painter.rect_filled(r, CornerRadius::same(1), color);
        }
        IconKind::CancelStop => {
            let b = Rect::from_min_max(p(0.24, 0.24), p(0.76, 0.76));
            painter.rect_filled(b, CornerRadius::same(2), color);
        }
        IconKind::PostGCode => {
            painter.add(egui::Shape::line(
                vec![p(0.36, 0.24), p(0.14, 0.50), p(0.36, 0.76)],
                stroke,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.64, 0.24), p(0.86, 0.50), p(0.64, 0.76)],
                stroke,
            ));
            painter.line_segment([p(0.55, 0.20), p(0.45, 0.80)], thin);
        }
        IconKind::SkipStart => {
            painter.line_segment([p(0.22, 0.22), p(0.22, 0.78)], stroke);
            painter.add(egui::Shape::convex_polygon(
                vec![p(0.78, 0.22), p(0.32, 0.50), p(0.78, 0.78)],
                color,
                Stroke::NONE,
            ));
        }
        IconKind::StepBack => {
            painter.add(egui::Shape::line(
                vec![p(0.64, 0.20), p(0.30, 0.50), p(0.64, 0.80)],
                stroke,
            ));
        }
        IconKind::StepForward => {
            painter.add(egui::Shape::line(
                vec![p(0.36, 0.20), p(0.70, 0.50), p(0.36, 0.80)],
                stroke,
            ));
        }
        IconKind::SkipNextOp => {
            painter.add(egui::Shape::line(
                vec![p(0.22, 0.22), p(0.52, 0.50), p(0.22, 0.78)],
                stroke,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.52, 0.22), p(0.82, 0.50), p(0.52, 0.78)],
                stroke,
            ));
        }
        IconKind::SkipEnd => {
            painter.add(egui::Shape::convex_polygon(
                vec![p(0.22, 0.22), p(0.68, 0.50), p(0.22, 0.78)],
                color,
                Stroke::NONE,
            ));
            painter.line_segment([p(0.78, 0.22), p(0.78, 0.78)], stroke);
        }
        IconKind::Sun => {
            painter.circle_stroke(sq.center(), s * 0.22, stroke);
            for k in 0..8 {
                let a = (k as f32) * FRAC_PI_4;
                let p0 = Pos2::new(
                    sq.center().x + s * 0.31 * a.cos(),
                    sq.center().y + s * 0.31 * a.sin(),
                );
                let p1 = Pos2::new(
                    sq.center().x + s * 0.44 * a.cos(),
                    sq.center().y + s * 0.44 * a.sin(),
                );
                painter.line_segment([p0, p1], thin);
            }
        }
        IconKind::Moon => {
            draw_arc(
                painter,
                p(0.52, 0.50),
                s * 0.34,
                0.32 * PI,
                1.68 * PI,
                14,
                stroke,
            );
            draw_arc(
                painter,
                p(0.68, 0.42),
                s * 0.28,
                0.55 * PI,
                1.45 * PI,
                12,
                thin,
            );
        }
        IconKind::Eye => {
            draw_arc(
                painter,
                p(0.50, 0.74),
                s * 0.44,
                -0.78 * PI,
                -0.22 * PI,
                10,
                stroke,
            );
            draw_arc(
                painter,
                p(0.50, 0.26),
                s * 0.44,
                0.22 * PI,
                0.78 * PI,
                10,
                stroke,
            );
            painter.circle_filled(sq.center(), s * 0.13, color);
        }
        IconKind::EyeOff => {
            draw_arc(
                painter,
                p(0.50, 0.74),
                s * 0.44,
                -0.78 * PI,
                -0.22 * PI,
                10,
                thin,
            );
            painter.line_segment([p(0.18, 0.82), p(0.82, 0.18)], stroke);
        }
        IconKind::FitView => {
            for (u0, v0, u1, v1, u2, v2) in [
                (0.16, 0.36, 0.16, 0.16, 0.36, 0.16),
                (0.64, 0.16, 0.84, 0.16, 0.84, 0.36),
                (0.84, 0.64, 0.84, 0.84, 0.64, 0.84),
                (0.36, 0.84, 0.16, 0.84, 0.16, 0.64),
            ] {
                painter.add(egui::Shape::line(
                    vec![p(u0, v0), p(u1, v1), p(u2, v2)],
                    stroke,
                ));
            }
        }
        IconKind::TopView => {
            let r = Rect::from_min_max(p(0.20, 0.20), p(0.80, 0.80));
            painter.rect_stroke(r, CornerRadius::same(2), stroke, StrokeKind::Inside);
            painter.circle_filled(sq.center(), sw * 0.9, color);
        }
        IconKind::Trash => {
            painter.line_segment([p(0.20, 0.26), p(0.80, 0.26)], stroke);
            painter.add(egui::Shape::line(
                vec![p(0.28, 0.26), p(0.32, 0.84), p(0.68, 0.84), p(0.72, 0.26)],
                thin,
            ));
            painter.line_segment([p(0.40, 0.16), p(0.60, 0.16)], thin);
        }
        IconKind::Duplicate => {
            let front = Rect::from_min_max(p(0.32, 0.32), p(0.82, 0.82));
            painter.rect_stroke(front, CornerRadius::same(2), stroke, StrokeKind::Inside);
            painter.add(egui::Shape::line(
                vec![
                    p(0.66, 0.32),
                    p(0.66, 0.18),
                    p(0.18, 0.18),
                    p(0.18, 0.66),
                    p(0.32, 0.66),
                ],
                thin,
            ));
        }
        IconKind::InfoCircle => {
            painter.circle_stroke(sq.center(), s * 0.42, thin);
            painter.circle_filled(p(0.50, 0.31), sw * 0.65, color);
            painter.line_segment([p(0.50, 0.45), p(0.50, 0.73)], stroke);
        }
        IconKind::Search => {
            painter.circle_stroke(p(0.43, 0.43), s * 0.25, stroke);
            painter.line_segment([p(0.61, 0.61), p(0.84, 0.84)], stroke);
        }
        IconKind::Sparkles => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.50, 0.14),
                    p(0.60, 0.40),
                    p(0.86, 0.50),
                    p(0.60, 0.60),
                    p(0.50, 0.86),
                    p(0.40, 0.60),
                    p(0.14, 0.50),
                    p(0.40, 0.40),
                ],
                thin,
            ));
        }
        IconKind::Keyboard => {
            let r = Rect::from_min_max(p(0.14, 0.26), p(0.86, 0.76));
            painter.rect_stroke(r, CornerRadius::same(2), thin, StrokeKind::Inside);
            for u in [0.30, 0.50, 0.70] {
                painter.circle_filled(p(u, 0.43), sw * 0.48, color);
            }
            painter.line_segment([p(0.32, 0.62), p(0.68, 0.62)], thin);
        }
        IconKind::Crosshair => {
            painter.circle_stroke(sq.center(), s * 0.28, thin);
            painter.line_segment([p(0.50, 0.12), p(0.50, 0.32)], thin);
            painter.line_segment([p(0.50, 0.68), p(0.50, 0.88)], thin);
            painter.line_segment([p(0.12, 0.50), p(0.32, 0.50)], thin);
            painter.line_segment([p(0.68, 0.50), p(0.88, 0.50)], thin);
        }
        IconKind::ArrowUp => {
            painter.add(egui::Shape::line(
                vec![p(0.22, 0.64), p(0.50, 0.32), p(0.78, 0.64)],
                stroke,
            ));
        }
        IconKind::ArrowDown => {
            painter.add(egui::Shape::line(
                vec![p(0.22, 0.36), p(0.50, 0.68), p(0.78, 0.36)],
                stroke,
            ));
        }
    }
}

/// Renders an inline `14×14` vector icon in the current horizontal `Ui` layout.
pub fn inline_icon(ui: &mut Ui, icon: IconKind, color: Color32, size_px: f32) -> Response {
    let (rect, resp) = ui.allocate_exact_size(Vec2::splat(size_px), Sense::hover());
    if ui.is_rect_visible(rect) {
        paint_icon(ui.painter(), rect, icon, color);
    }
    resp
}

/// Renders a semantic status vector icon (`Clean`, `Dirty`, `Error`, `NeedsRepick`, `Suppressed`).
pub fn status_badge_icon(ui: &mut Ui, status: StatusBadgeKind, tooltip: Option<&str>) -> Response {
    let tok = ThemeTokens::from_ui(ui);
    let col = match status {
        StatusBadgeKind::Clean => tok.clean_fg,
        StatusBadgeKind::Dirty => tok.dirty_fg,
        StatusBadgeKind::Error => tok.error_fg,
        StatusBadgeKind::NeedsRepick => tok.warn_fg,
        StatusBadgeKind::Suppressed => tok.suppressed_fg,
    };
    let resp = inline_icon(ui, IconKind::for_status(status), col, 14.0);
    if let Some(tip) = tooltip {
        resp.on_hover_text(tip)
    } else {
        resp
    }
}

/// Renders a toolbar/panel button combining a procedural vector icon with an optional text label.
pub fn icon_button(ui: &mut Ui, icon: IconKind, label: Option<&str>, tooltip: &str) -> Response {
    icon_button_styled(ui, icon, label, tooltip, false)
}

/// Renders a compact `20×20` px icon-only button used for hover-revealed tree/row quick actions.
pub fn small_icon_button(ui: &mut Ui, icon: IconKind, tooltip: &str) -> Response {
    let tok = ThemeTokens::from_ui(ui);
    let enabled = ui.is_enabled();
    let (rect, resp) = ui.allocate_exact_size(
        Vec2::splat(20.0),
        if enabled {
            Sense::click()
        } else {
            Sense::hover()
        },
    );
    if ui.is_rect_visible(rect) {
        let hovered = enabled && resp.hovered();
        let active = enabled && resp.is_pointer_button_down_on();
        let fg = if !enabled {
            tok.text_muted.gamma_multiply(0.55)
        } else if active {
            tok.accent_on_fg
        } else if hovered {
            tok.text_primary
        } else {
            tok.text_secondary
        };
        if active {
            ui.painter().rect(
                rect,
                CornerRadius::same(4),
                tok.accent,
                Stroke::new(1.0, tok.accent_hover),
                StrokeKind::Inside,
            );
        } else if hovered {
            ui.painter().rect(
                rect,
                CornerRadius::same(4),
                ui.visuals().widgets.hovered.bg_fill,
                Stroke::new(1.0, tok.accent),
                StrokeKind::Inside,
            );
        }
        let icon_rect = Rect::from_center_size(rect.center(), Vec2::splat(12.5));
        paint_icon(ui.painter(), icon_rect, icon, fg);
    }
    if tooltip.is_empty() {
        resp
    } else {
        resp.on_hover_text(tooltip)
    }
}

/// Renders a primary accent-filled button (used for high-priority workflow actions like `Generate`).
pub fn primary_icon_button(ui: &mut Ui, icon: IconKind, label: &str, tooltip: &str) -> Response {
    icon_button_styled(ui, icon, Some(label), tooltip, true)
}

fn icon_button_styled(
    ui: &mut Ui,
    icon: IconKind,
    label: Option<&str>,
    tooltip: &str,
    primary_accent: bool,
) -> Response {
    let tok = ThemeTokens::from_ui(ui);
    let enabled = ui.is_enabled();
    let icon_sz = 14.0_f32;
    let pad_x = 7.0_f32;
    let height = 24.0_f32;

    let galley = label.map(|txt| {
        ui.painter().layout_no_wrap(
            txt.to_string(),
            egui::FontId::proportional(12.5),
            Color32::WHITE,
        )
    });
    let text_w = galley.as_ref().map_or(0.0, |g| g.size().x + 5.0);
    let total_w = pad_x * 2.0 + icon_sz + text_w;

    let sense = if enabled {
        Sense::click()
    } else {
        Sense::hover()
    };
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(total_w, height), sense);

    if ui.is_rect_visible(rect) {
        let hovered = enabled && resp.hovered();
        let active = enabled && resp.is_pointer_button_down_on();
        let (bg, border, fg) = if !enabled {
            (
                tok.card_bg.gamma_multiply(0.55),
                tok.border.gamma_multiply(0.55),
                tok.text_muted.gamma_multiply(0.65),
            )
        } else if primary_accent {
            let fill = if active || hovered {
                tok.accent_hover
            } else {
                tok.accent
            };
            (fill, tok.accent_hover, tok.accent_on_fg)
        } else if active {
            (tok.accent, tok.accent_hover, tok.accent_on_fg)
        } else if hovered {
            (
                ui.visuals().widgets.hovered.bg_fill,
                tok.accent,
                tok.text_primary,
            )
        } else {
            (tok.card_bg, tok.border, tok.text_primary)
        };

        ui.painter().rect(
            rect,
            CornerRadius::same(5),
            bg,
            Stroke::new(1.0, border),
            StrokeKind::Inside,
        );

        let icon_rect = Rect::from_min_size(
            Pos2::new(rect.left() + pad_x, rect.center().y - 0.5 * icon_sz),
            Vec2::splat(icon_sz),
        );
        paint_icon(ui.painter(), icon_rect, icon, fg);

        if let Some(txt) = label {
            ui.painter().text(
                Pos2::new(icon_rect.right() + 5.0, rect.center().y),
                egui::Align2::LEFT_CENTER,
                txt,
                egui::FontId::proportional(12.5),
                fg,
            );
        }
    }

    if tooltip.is_empty() {
        resp
    } else {
        resp.on_hover_text(tooltip)
    }
}

/// Compact toggle pill used inside the Floating Viewport View Bar.
pub fn hud_toggle_pill(
    ui: &mut Ui,
    active: &mut bool,
    icon: Option<IconKind>,
    label: &str,
    tooltip: &str,
) -> bool {
    let tok = ThemeTokens::from_ui(ui);
    let icon_w = if icon.is_some() { 16.0 } else { 0.0 };
    let galley = ui.painter().layout_no_wrap(
        label.to_string(),
        egui::FontId::proportional(11.5),
        Color32::WHITE,
    );
    let w = 12.0 + icon_w + galley.size().x;
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, 22.0), Sense::click());
    let mut changed = false;
    if resp.clicked() {
        *active = !*active;
        changed = true;
    }

    if ui.is_rect_visible(rect) {
        let hovered = resp.hovered();
        let (bg, border, fg) = if *active {
            (tok.accent, tok.accent_hover, tok.accent_on_fg)
        } else if hovered {
            (
                ui.visuals().widgets.hovered.bg_fill,
                tok.accent,
                tok.text_primary,
            )
        } else {
            (tok.card_bg, tok.border, tok.text_secondary)
        };
        ui.painter().rect(
            rect,
            CornerRadius::same(5),
            bg,
            Stroke::new(1.0, border),
            StrokeKind::Inside,
        );
        let mut x = rect.left() + 6.0;
        if let Some(ic) = icon {
            let ir = Rect::from_min_size(Pos2::new(x, rect.center().y - 6.0), Vec2::splat(12.0));
            paint_icon(ui.painter(), ir, ic, fg);
            x += 15.0;
        }
        ui.painter().text(
            Pos2::new(x, rect.center().y),
            egui::Align2::LEFT_CENTER,
            label,
            egui::FontId::proportional(11.5),
            fg,
        );
    }

    let _ = resp.on_hover_text(tooltip);
    changed
}
