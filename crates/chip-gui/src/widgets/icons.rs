use std::f32::consts::{FRAC_PI_4, PI, TAU};
use std::path::Path;

use chip_cad::is_mesh_extension;
use chip_cam::OperationKind;
use chip_project::ModelEntry;
use chip_sketch::SketchEntity;
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IconKind {
    // Status badges
    StatusClean,
    StatusDirty,
    StatusError,
    StatusNeedsRepick,
    StatusSuppressed,

    // Project hierarchy & document nodes
    Project,
    ModelsFolder,
    OperationsFolder,
    MeshModel,
    FixtureModel,
    KeepOutZone,
    Setup,
    Fixture,
    ChevronRight,
    ChevronDown,

    // Operations (2.5D, 3D, Undercut/Deburr, 4th-Axis)
    Facing,
    Contour2D,
    Pocket2D,
    Adaptive2D,
    DrillBore,
    ThreadMill,
    SlotUndercut,
    Dovetail,
    Undercut3D,
    ChamferTrace,
    Deburr3D,
    VCarve,
    Adaptive3D,
    Waterline3D,
    Parallel3D,
    Scallop3D,
    Pencil3D,
    Rotary2Linear,
    Rotary4AxisSim,

    // Sketch & CAD entities
    Sketch2D,
    StepModel,
    Tool,
    EntityRect,
    EntityCircle,
    EntitySlot,
    EntityPolygon,
    EntityBoltCircle,
    EntityPolyline,
    EntityText,

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
    Crosshair,
    Settings,
}

impl IconKind {
    /// Complete list of every [`IconKind`] variant (used for exhaustive rendering verification).
    #[cfg(test)]
    pub const ALL: &'static [Self] = &[
        Self::StatusClean,
        Self::StatusDirty,
        Self::StatusError,
        Self::StatusNeedsRepick,
        Self::StatusSuppressed,
        Self::Project,
        Self::ModelsFolder,
        Self::OperationsFolder,
        Self::MeshModel,
        Self::FixtureModel,
        Self::KeepOutZone,
        Self::Setup,
        Self::Fixture,
        Self::ChevronRight,
        Self::ChevronDown,
        Self::Facing,
        Self::Contour2D,
        Self::Pocket2D,
        Self::Adaptive2D,
        Self::DrillBore,
        Self::ThreadMill,
        Self::SlotUndercut,
        Self::Dovetail,
        Self::Undercut3D,
        Self::ChamferTrace,
        Self::Deburr3D,
        Self::VCarve,
        Self::Adaptive3D,
        Self::Waterline3D,
        Self::Parallel3D,
        Self::Scallop3D,
        Self::Pencil3D,
        Self::Rotary2Linear,
        Self::Rotary4AxisSim,
        Self::Sketch2D,
        Self::StepModel,
        Self::Tool,
        Self::EntityRect,
        Self::EntityCircle,
        Self::EntitySlot,
        Self::EntityPolygon,
        Self::EntityBoltCircle,
        Self::EntityPolyline,
        Self::EntityText,
        Self::FolderOpen,
        Self::Save,
        Self::BundleZip,
        Self::VectorImport,
        Self::Reload,
        Self::Undo,
        Self::Redo,
        Self::Plus,
        Self::Generate,
        Self::Simulate,
        Self::PostGCode,
        Self::CancelStop,
        Self::Play,
        Self::Pause,
        Self::SkipStart,
        Self::StepBack,
        Self::StepForward,
        Self::SkipNextOp,
        Self::SkipEnd,
        Self::Eye,
        Self::EyeOff,
        Self::FitView,
        Self::TopView,
        Self::OrthoCube,
        Self::Trash,
        Self::Duplicate,
        Self::InfoCircle,
        Self::Search,
        Self::Sparkles,
        Self::Crosshair,
        Self::Settings,
    ];

    pub const fn for_operation(kind: OperationKind, is_adaptive: bool) -> Self {
        if is_adaptive && matches!(kind, OperationKind::Pocket2D) {
            return Self::Adaptive2D;
        }
        match kind {
            OperationKind::Facing => Self::Facing,
            OperationKind::Contour2D => Self::Contour2D,
            OperationKind::Pocket2D => Self::Pocket2D,
            OperationKind::DrillBore => Self::DrillBore,
            OperationKind::ThreadMill => Self::ThreadMill,
            OperationKind::SlotUndercut => Self::SlotUndercut,
            OperationKind::Dovetail => Self::Dovetail,
            OperationKind::ChamferTrace => Self::ChamferTrace,
            OperationKind::VCarve => Self::VCarve,
            OperationKind::Adaptive3D => Self::Adaptive3D,
            OperationKind::Waterline3D => Self::Waterline3D,
            OperationKind::Parallel3D => Self::Parallel3D,
            OperationKind::Scallop3D => Self::Scallop3D,
            OperationKind::Pencil3D => Self::Pencil3D,
            OperationKind::Deburr3D => Self::Deburr3D,
            OperationKind::Undercut3D => Self::Undercut3D,
            OperationKind::Rotary2Linear => Self::Rotary2Linear,
            OperationKind::Rotary4AxisSim => Self::Rotary4AxisSim,
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

    pub const fn for_sketch_entity(entity: &SketchEntity) -> Self {
        match entity {
            SketchEntity::Rectangle { .. } => Self::EntityRect,
            SketchEntity::Circle { .. } => Self::EntityCircle,
            SketchEntity::Slot { .. } => Self::EntitySlot,
            SketchEntity::RegularPolygon { .. } => Self::EntityPolygon,
            SketchEntity::PointPattern(_) => Self::EntityBoltCircle,
            SketchEntity::Polyline { .. } => Self::EntityPolyline,
            SketchEntity::Text(_) => Self::EntityText,
        }
    }

    pub fn for_model_entry(entry: &ModelEntry) -> Self {
        if entry.role.is_fixture() {
            return Self::FixtureModel;
        }
        let is_mesh = Path::new(&entry.path)
            .extension()
            .and_then(|s| s.to_str())
            .is_some_and(is_mesh_extension);
        if is_mesh {
            Self::MeshModel
        } else {
            Self::StepModel
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
        IconKind::Project => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.20, 0.12),
                    p(0.62, 0.12),
                    p(0.82, 0.32),
                    p(0.82, 0.88),
                    p(0.20, 0.88),
                ],
                stroke,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.62, 0.12), p(0.62, 0.32), p(0.82, 0.32)],
                thin,
            ));
            painter.line_segment([p(0.32, 0.52), p(0.68, 0.52)], thin);
            painter.line_segment([p(0.32, 0.68), p(0.58, 0.68)], thin);
        }
        IconKind::ModelsFolder => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.10, 0.24),
                    p(0.38, 0.24),
                    p(0.46, 0.34),
                    p(0.90, 0.34),
                    p(0.90, 0.82),
                    p(0.10, 0.82),
                ],
                stroke,
            ));
            painter.add(egui::Shape::closed_line(
                vec![p(0.50, 0.44), p(0.64, 0.56), p(0.50, 0.68), p(0.36, 0.56)],
                thin,
            ));
        }
        IconKind::OperationsFolder => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.10, 0.24),
                    p(0.38, 0.24),
                    p(0.46, 0.34),
                    p(0.90, 0.34),
                    p(0.90, 0.82),
                    p(0.10, 0.82),
                ],
                stroke,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.24, 0.48), p(0.74, 0.48), p(0.24, 0.66), p(0.74, 0.66)],
                thin,
            ));
        }
        IconKind::MeshModel => {
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
            // Triangulation diagonals across all 3 visible faces
            painter.line_segment([p(0.16, 0.32), p(0.84, 0.32)], thin);
            painter.line_segment([p(0.16, 0.32), p(0.50, 0.86)], thin);
            painter.line_segment([p(0.84, 0.32), p(0.50, 0.86)], thin);
        }
        IconKind::FixtureModel => {
            // L-shaped clamp jaw profile with clamping bolt
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.14, 0.82),
                    p(0.86, 0.82),
                    p(0.86, 0.62),
                    p(0.42, 0.62),
                    p(0.42, 0.20),
                    p(0.14, 0.20),
                ],
                stroke,
            ));
            painter.line_segment([p(0.66, 0.22), p(0.66, 0.60)], stroke);
            painter.line_segment([p(0.52, 0.22), p(0.80, 0.22)], stroke);
            painter.line_segment([p(0.56, 0.54), p(0.76, 0.54)], thin);
        }
        IconKind::KeepOutZone => {
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.32, 0.12),
                    p(0.68, 0.12),
                    p(0.88, 0.32),
                    p(0.88, 0.68),
                    p(0.68, 0.88),
                    p(0.32, 0.88),
                    p(0.12, 0.68),
                    p(0.12, 0.32),
                ],
                stroke,
            ));
            painter.line_segment([p(0.26, 0.74), p(0.74, 0.26)], stroke);
        }
        IconKind::Setup => {
            // WCS triad (+Z up, +X right, +Y diagonal) + subtle stock corner bracket
            painter.line_segment([p(0.26, 0.74), p(0.26, 0.16)], stroke);
            painter.line_segment([p(0.26, 0.74), p(0.84, 0.74)], stroke);
            painter.line_segment([p(0.26, 0.74), p(0.58, 0.44)], thin);
            // Small arrowheads on +Z and +X
            painter.add(egui::Shape::line(
                vec![p(0.18, 0.26), p(0.26, 0.16), p(0.34, 0.26)],
                thin,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.74, 0.66), p(0.84, 0.74), p(0.74, 0.82)],
                thin,
            ));
            // Stock frame corner cue
            painter.add(egui::Shape::line(
                vec![p(0.50, 0.22), p(0.84, 0.22), p(0.84, 0.54)],
                thin,
            ));
        }
        IconKind::Fixture => {
            // Mill vise jaws + base rail + lead screw
            painter.line_segment([p(0.10, 0.80), p(0.90, 0.80)], stroke);
            let left_jaw = Rect::from_min_max(p(0.14, 0.28), p(0.34, 0.80));
            let right_jaw = Rect::from_min_max(p(0.66, 0.28), p(0.86, 0.80));
            painter.rect_stroke(left_jaw, CornerRadius::same(1), stroke, StrokeKind::Inside);
            painter.rect_stroke(right_jaw, CornerRadius::same(1), stroke, StrokeKind::Inside);
            painter.line_segment([p(0.34, 0.56), p(0.66, 0.56)], thin);
            painter.line_segment([p(0.14, 0.40), p(0.34, 0.40)], thin);
            painter.line_segment([p(0.66, 0.40), p(0.86, 0.40)], thin);
        }
        IconKind::ChevronRight => {
            painter.add(egui::Shape::line(
                vec![p(0.34, 0.20), p(0.68, 0.50), p(0.34, 0.80)],
                stroke,
            ));
        }
        IconKind::ChevronDown => {
            painter.add(egui::Shape::line(
                vec![p(0.20, 0.34), p(0.50, 0.68), p(0.80, 0.34)],
                stroke,
            ));
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
        IconKind::ThreadMill => {
            // Shank collar
            let shank = Rect::from_min_max(p(0.36, 0.10), p(0.64, 0.25));
            painter.rect_stroke(shank, CornerRadius::ZERO, thin, StrokeKind::Inside);
            // Slim relieved neck
            painter.line_segment([p(0.44, 0.25), p(0.44, 0.46)], thin);
            painter.line_segment([p(0.56, 0.25), p(0.56, 0.46)], thin);
            // V-tooth cutting profile silhouette
            painter.add(egui::Shape::closed_line(
                vec![
                    p(0.44, 0.46),
                    p(0.24, 0.55),
                    p(0.38, 0.64),
                    p(0.24, 0.73),
                    p(0.38, 0.82),
                    p(0.38, 0.88),
                    p(0.62, 0.88),
                    p(0.62, 0.82),
                    p(0.76, 0.73),
                    p(0.62, 0.64),
                    p(0.76, 0.55),
                    p(0.56, 0.46),
                ],
                stroke,
            ));
            // Slanted helix pitch ridges across the cutting band
            painter.line_segment([p(0.29, 0.57), p(0.71, 0.53)], thin);
            painter.line_segment([p(0.29, 0.75), p(0.71, 0.71)], thin);
        }
        IconKind::ChamferTrace => {
            painter.add(egui::Shape::line(
                vec![p(0.16, 0.78), p(0.46, 0.78), p(0.82, 0.42), p(0.82, 0.20)],
                stroke,
            ));
            painter.line_segment([p(0.42, 0.26), p(0.74, 0.58)], thin);
        }
        IconKind::Deburr3D => {
            // 3D curved beveled contour with diagonal edge-break highlight & spherical/conical touch
            painter.add(egui::Shape::line(
                vec![p(0.14, 0.76), p(0.38, 0.72), p(0.60, 0.52), p(0.84, 0.36)],
                stroke,
            ));
            painter.add(egui::Shape::line(
                vec![p(0.14, 0.56), p(0.40, 0.52), p(0.62, 0.32), p(0.84, 0.18)],
                thin,
            ));
            // Bevel tick marks linking upper/lower edges across the 3D chamfer strip
            painter.line_segment([p(0.39, 0.52), p(0.39, 0.72)], thin);
            painter.line_segment([p(0.61, 0.32), p(0.61, 0.52)], thin);
            painter.circle_stroke(p(0.70, 0.68), s * 0.14, stroke);
        }
        IconKind::SlotUndercut => {
            // Inverted-T undercut cavity profile with relieved-neck slot mill disc inside
            painter.add(egui::Shape::line(
                vec![
                    p(0.12, 0.22),
                    p(0.36, 0.22),
                    p(0.36, 0.50),
                    p(0.16, 0.50),
                    p(0.16, 0.80),
                    p(0.84, 0.80),
                    p(0.84, 0.50),
                    p(0.64, 0.50),
                    p(0.64, 0.22),
                    p(0.88, 0.22),
                ],
                stroke,
            ));
            painter.line_segment([p(0.50, 0.14), p(0.50, 0.58)], thin);
            let disc = Rect::from_min_max(p(0.28, 0.58), p(0.72, 0.72));
            painter.rect_stroke(disc, CornerRadius::same(1), thin, StrokeKind::Inside);
        }
        IconKind::Dovetail => {
            // Angled 60° dovetail cavity profile with inverted-cone cutter outline
            painter.add(egui::Shape::line(
                vec![
                    p(0.12, 0.24),
                    p(0.34, 0.24),
                    p(0.18, 0.80),
                    p(0.82, 0.80),
                    p(0.66, 0.24),
                    p(0.88, 0.24),
                ],
                stroke,
            ));
            painter.line_segment([p(0.50, 0.14), p(0.50, 0.44)], thin);
            painter.add(egui::Shape::closed_line(
                vec![p(0.42, 0.44), p(0.58, 0.44), p(0.68, 0.72), p(0.32, 0.72)],
                thin,
            ));
        }
        IconKind::Undercut3D => {
            // Overhanging curved 3D lip with relieved-neck spherical lollipop cutter reaching underneath
            painter.add(egui::Shape::line(
                vec![
                    p(0.14, 0.22),
                    p(0.72, 0.22),
                    p(0.58, 0.42),
                    p(0.36, 0.58),
                    p(0.36, 0.84),
                ],
                stroke,
            ));
            painter.line_segment([p(0.66, 0.14), p(0.66, 0.48)], thin);
            painter.circle_stroke(p(0.66, 0.64), s * 0.16, stroke);
        }
        IconKind::VCarve => {
            painter.line_segment([p(0.12, 0.24), p(0.88, 0.24)], thin);
            painter.add(egui::Shape::line(
                vec![p(0.20, 0.24), p(0.50, 0.82), p(0.80, 0.24)],
                stroke,
            ));
        }
        IconKind::Adaptive3D => {
            // 3D terraced adaptive spiral: stepped base + upper trochoidal loop
            let mut upper = Vec::with_capacity(17);
            for i in 0..=16 {
                let t = (i as f32) / 16.0;
                let ang = t * 1.65 * TAU;
                let rx = s * (0.08 + 0.26 * t);
                let ry = s * (0.05 + 0.16 * t);
                upper.push(Pos2::new(
                    p(0.48, 0.36).x + rx * ang.cos(),
                    p(0.48, 0.36).y + ry * ang.sin(),
                ));
            }
            painter.add(egui::Shape::line(upper, stroke));
            // Stepped Z-level terraces beneath
            painter.add(egui::Shape::line(
                vec![p(0.16, 0.62), p(0.40, 0.62), p(0.48, 0.74), p(0.84, 0.74)],
                thin,
            ));
            painter.line_segment([p(0.22, 0.84), p(0.78, 0.84)], thin);
        }
        IconKind::Waterline3D => {
            // Stacked Z-level topographic elevation contours
            for (cy, rx, ry, st) in [
                (0.74, 0.38, 0.12, stroke),
                (0.50, 0.28, 0.10, stroke),
                (0.26, 0.17, 0.08, thin),
            ] {
                let pts: Vec<Pos2> = (0..=16)
                    .map(|i| {
                        let a = (i as f32 / 16.0) * TAU;
                        Pos2::new(
                            sq.center().x + s * rx * a.cos(),
                            p(0.50, cy).y + s * ry * a.sin(),
                        )
                    })
                    .collect();
                painter.add(egui::Shape::line(pts, st));
            }
        }
        IconKind::Parallel3D => {
            // Parallel 3D surface scanlines curving smoothly over a sculpted ridge
            for y_base in [0.34, 0.56, 0.78] {
                let pts: Vec<Pos2> = (0..=10)
                    .map(|i| {
                        let t = i as f32 / 10.0;
                        let u = 0.12 + 0.76 * t;
                        let v = y_base - 0.15 * (t * PI).sin();
                        p(u, v)
                    })
                    .collect();
                painter.add(egui::Shape::line(pts, stroke));
            }
        }
        IconKind::Scallop3D => {
            // Concentric 3D constant-scallop contour shells over a curved peak
            for (rx, ry, st) in [(0.38, 0.32, stroke), (0.24, 0.20, thin)] {
                let pts: Vec<Pos2> = (0..=18)
                    .map(|i| {
                        let a = (i as f32 / 18.0) * TAU;
                        Pos2::new(
                            sq.center().x + s * rx * a.cos(),
                            sq.center().y + s * ry * a.sin(),
                        )
                    })
                    .collect();
                painter.add(egui::Shape::line(pts, st));
            }
            painter.circle_filled(sq.center(), sw * 0.85, color);
        }
        IconKind::Pencil3D => {
            // Concave V-valley corner walls with ball-end cutter tracing the crease
            painter.add(egui::Shape::line(
                vec![p(0.14, 0.24), p(0.50, 0.82), p(0.86, 0.24)],
                thin,
            ));
            painter.line_segment([p(0.50, 0.12), p(0.50, 0.40)], thin);
            painter.circle_stroke(p(0.50, 0.56), s * 0.16, stroke);
        }
        IconKind::Rotary2Linear => {
            // Horizontal rotary cylinder with axial cutting pass & rotation arrow
            painter.line_segment([p(0.24, 0.30), p(0.82, 0.30)], stroke);
            painter.line_segment([p(0.24, 0.70), p(0.82, 0.70)], stroke);
            draw_arc(
                painter,
                p(0.82, 0.50),
                s * 0.20,
                -0.5 * PI,
                0.5 * PI,
                8,
                stroke,
            );
            draw_arc(
                painter,
                p(0.24, 0.50),
                s * 0.20,
                0.5 * PI,
                1.5 * PI,
                8,
                thin,
            );
            // Linear pass along cylinder axis + rotary arrow
            painter.line_segment([p(0.26, 0.50), p(0.82, 0.50)], thin);
            draw_arc(
                painter,
                p(0.24, 0.50),
                s * 0.30,
                0.65 * PI,
                1.35 * PI,
                8,
                stroke,
            );
            painter.add(egui::Shape::line(
                vec![p(0.10, 0.26), p(0.22, 0.20), p(0.24, 0.34)],
                thin,
            ));
        }
        IconKind::Rotary4AxisSim => {
            // Simultaneous 4-axis cylinder + helical multi-axis wrap curve + A-axis arrow
            painter.line_segment([p(0.16, 0.30), p(0.84, 0.30)], thin);
            painter.line_segment([p(0.16, 0.70), p(0.84, 0.70)], thin);
            let helix: Vec<Pos2> = (0..=16)
                .map(|i| {
                    let t = i as f32 / 16.0;
                    let u = 0.16 + 0.68 * t;
                    let v = 0.50 + 0.20 * (t * 2.5 * PI).sin();
                    p(u, v)
                })
                .collect();
            painter.add(egui::Shape::line(helix, stroke));
            // Curved A-axis rotary arrow above
            draw_arc(
                painter,
                p(0.50, 0.34),
                s * 0.24,
                -0.82 * PI,
                -0.18 * PI,
                8,
                stroke,
            );
            painter.add(egui::Shape::line(
                vec![p(0.64, 0.14), p(0.72, 0.22), p(0.60, 0.26)],
                thin,
            ));
        }
        IconKind::EntityRect => {
            let r = Rect::from_min_max(p(0.16, 0.22), p(0.84, 0.78));
            painter.rect_stroke(r, CornerRadius::same(1), stroke, StrokeKind::Inside);
            for (u, v) in [(0.16, 0.22), (0.84, 0.22), (0.84, 0.78), (0.16, 0.78)] {
                painter.circle_filled(p(u, v), sw * 0.65, color);
            }
        }
        IconKind::EntityCircle => {
            painter.circle_stroke(sq.center(), s * 0.35, stroke);
            painter.circle_filled(sq.center(), sw * 0.75, color);
        }
        IconKind::EntitySlot => {
            painter.line_segment([p(0.32, 0.30), p(0.68, 0.30)], stroke);
            painter.line_segment([p(0.32, 0.70), p(0.68, 0.70)], stroke);
            draw_arc(
                painter,
                p(0.32, 0.50),
                s * 0.20,
                0.5 * PI,
                1.5 * PI,
                8,
                stroke,
            );
            draw_arc(
                painter,
                p(0.68, 0.50),
                s * 0.20,
                -0.5 * PI,
                0.5 * PI,
                8,
                stroke,
            );
        }
        IconKind::EntityPolygon => {
            let pts: Vec<Pos2> = (0..6)
                .map(|k| {
                    let a = (k as f32) * (TAU / 6.0) - 0.5 * PI;
                    Pos2::new(
                        sq.center().x + s * 0.38 * a.cos(),
                        sq.center().y + s * 0.38 * a.sin(),
                    )
                })
                .collect();
            painter.add(egui::Shape::closed_line(pts, stroke));
        }
        IconKind::EntityBoltCircle => {
            painter.circle_stroke(sq.center(), s * 0.32, thin);
            for k in 0..6 {
                let a = (k as f32) * (TAU / 6.0);
                let pt = Pos2::new(
                    sq.center().x + s * 0.32 * a.cos(),
                    sq.center().y + s * 0.32 * a.sin(),
                );
                painter.circle_filled(pt, sw * 0.85, color);
            }
        }
        IconKind::EntityPolyline => {
            let pts = vec![p(0.14, 0.74), p(0.36, 0.28), p(0.64, 0.68), p(0.86, 0.24)];
            painter.add(egui::Shape::line(pts.clone(), stroke));
            for pt in pts {
                painter.circle_filled(pt, sw * 0.75, color);
            }
        }
        IconKind::EntityText => {
            painter.line_segment([p(0.18, 0.20), p(0.82, 0.20)], stroke);
            painter.line_segment([p(0.18, 0.20), p(0.18, 0.30)], thin);
            painter.line_segment([p(0.82, 0.20), p(0.82, 0.30)], thin);
            painter.line_segment([p(0.50, 0.20), p(0.50, 0.82)], stroke);
            painter.line_segment([p(0.34, 0.82), p(0.66, 0.82)], stroke);
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
        IconKind::Crosshair => {
            painter.circle_stroke(sq.center(), s * 0.28, thin);
            painter.line_segment([p(0.50, 0.12), p(0.50, 0.32)], thin);
            painter.line_segment([p(0.50, 0.68), p(0.50, 0.88)], thin);
            painter.line_segment([p(0.12, 0.50), p(0.32, 0.50)], thin);
            painter.line_segment([p(0.68, 0.50), p(0.88, 0.50)], thin);
        }
        IconKind::Settings => {
            let c = sq.center();
            let r_inner = s * 0.14;
            let r_body = s * 0.28;
            let r_tooth = s * 0.40;
            painter.circle_stroke(c, r_body, thin);
            painter.circle_stroke(c, r_inner, thin);
            for k in 0..8 {
                let a = (k as f32) * FRAC_PI_4;
                let dir = Vec2::new(a.cos(), a.sin());
                painter.line_segment([c + dir * r_body, c + dir * r_tooth], stroke);
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_mill_icon_maps_operation_and_renders_without_panic() {
        assert_eq!(
            IconKind::for_operation(OperationKind::ThreadMill, false),
            IconKind::ThreadMill
        );

        let ctx = egui::Context::default();
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(200.0, 200.0))),
            ..Default::default()
        };
        let mut out = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                let (rect, _) = ui.allocate_exact_size(Vec2::splat(24.0), Sense::hover());
                paint_icon(ui.painter(), rect, IconKind::ThreadMill, Color32::WHITE);
                let _ = inline_icon(ui, IconKind::ThreadMill, Color32::LIGHT_GRAY, 14.0);
            });
        });
        out.textures_delta.clear();
        assert!(
            !out.shapes.is_empty(),
            "expected IconKind::ThreadMill to emit vector shapes"
        );
    }

    #[test]
    fn deburr3d_icon_maps_operation_and_renders_without_panic() {
        assert_eq!(
            IconKind::for_operation(OperationKind::Deburr3D, false),
            IconKind::Deburr3D
        );

        let ctx = egui::Context::default();
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(200.0, 200.0))),
            ..Default::default()
        };
        let mut out = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                let (rect, _) = ui.allocate_exact_size(Vec2::splat(24.0), Sense::hover());
                paint_icon(ui.painter(), rect, IconKind::Deburr3D, Color32::WHITE);
                let _ = inline_icon(ui, IconKind::Deburr3D, Color32::LIGHT_GRAY, 14.0);
            });
        });
        out.textures_delta.clear();
        assert!(
            !out.shapes.is_empty(),
            "expected IconKind::Deburr3D to emit vector shapes"
        );
    }

    #[test]
    fn undercut_icons_map_operations_and_render_without_panic() {
        assert_eq!(
            IconKind::for_operation(OperationKind::SlotUndercut, false),
            IconKind::SlotUndercut
        );
        assert_eq!(
            IconKind::for_operation(OperationKind::Dovetail, false),
            IconKind::Dovetail
        );
        assert_eq!(
            IconKind::for_operation(OperationKind::Undercut3D, false),
            IconKind::Undercut3D
        );

        let ctx = egui::Context::default();
        for icon in [
            IconKind::SlotUndercut,
            IconKind::Dovetail,
            IconKind::Undercut3D,
        ] {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(200.0, 200.0))),
                ..Default::default()
            };
            let mut out = ctx.run_ui(input, |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let (rect, _) = ui.allocate_exact_size(Vec2::splat(24.0), Sense::hover());
                    paint_icon(ui.painter(), rect, icon, Color32::WHITE);
                    let _ = inline_icon(ui, icon, Color32::LIGHT_GRAY, 14.0);
                });
            });
            out.textures_delta.clear();
            assert!(
                !out.shapes.is_empty(),
                "expected {icon:?} to emit vector shapes"
            );
        }
    }

    #[test]
    fn all_icon_kinds_render_cleanly_without_panic() {
        let ctx = egui::Context::default();
        for &icon in IconKind::ALL {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(200.0, 200.0))),
                ..Default::default()
            };
            let mut out = ctx.run_ui(input, |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let (rect, _) = ui.allocate_exact_size(Vec2::splat(16.0), Sense::hover());
                    paint_icon(ui.painter(), rect, icon, Color32::WHITE);
                });
            });
            out.textures_delta.clear();
            assert!(
                !out.shapes.is_empty(),
                "expected IconKind::{icon:?} to emit vector shapes"
            );
        }
    }

    #[test]
    fn all_18_operation_kinds_map_to_distinct_icons() {
        use std::collections::HashSet;

        assert_eq!(OperationKind::ALL.len(), 18);
        let mut seen = HashSet::new();
        for kind in OperationKind::ALL {
            let icon = IconKind::for_operation(kind, false);
            assert!(
                seen.insert(icon),
                "duplicate icon {icon:?} mapped for OperationKind::{kind:?}"
            );
        }
        // Adaptive 2D Pocket overrides to Adaptive2D, distinct from Adaptive3D
        let adap_2d = IconKind::for_operation(OperationKind::Pocket2D, true);
        assert_eq!(adap_2d, IconKind::Adaptive2D);
        assert_ne!(
            adap_2d,
            IconKind::for_operation(OperationKind::Adaptive3D, false)
        );
    }

    #[test]
    fn sketch_entity_and_model_entry_helpers_map_distinctly() {
        use chip_math::{ArcPolyline2D, ParamLength};
        use chip_sketch::{
            PointPatternSpec, PolygonSizingMode, RectAnchorMode, SketchTextSpec, SlotLengthMode,
        };
        use std::collections::HashSet;

        let entities = [
            SketchEntity::Rectangle {
                anchor: RectAnchorMode::default(),
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(20.0),
                height: ParamLength::from_mm(10.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
            SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(10.0),
            ),
            SketchEntity::Slot {
                mode: SlotLengthMode::default(),
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(0.0),
                length: ParamLength::from_mm(20.0),
                width: ParamLength::from_mm(6.0),
                angle_deg: 0.0,
            },
            SketchEntity::RegularPolygon {
                center_x: ParamLength::from_mm(0.0),
                center_y: ParamLength::from_mm(0.0),
                sides: 6,
                sizing: PolygonSizingMode::default(),
                diameter: ParamLength::from_mm(12.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
            SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
                center_x: ParamLength::from_mm(0.0),
                center_y: ParamLength::from_mm(0.0),
                bolt_circle_diameter: ParamLength::from_mm(30.0),
                hole_diameter: ParamLength::from_mm(5.0),
                count: 6,
                start_angle_deg: 0.0,
            }),
            SketchEntity::new_polyline(ArcPolyline2D::default()),
            SketchEntity::Text(SketchTextSpec::default()),
        ];

        let mut seen = HashSet::new();
        for ent in &entities {
            let ic = IconKind::for_sketch_entity(ent);
            assert!(
                seen.insert(ic),
                "duplicate icon {ic:?} for sketch entity {}",
                ent.kind_label()
            );
        }
        assert_eq!(seen.len(), 7);

        let step = ModelEntry::new("m1", "parts/bracket.step", "abc");
        let stl = ModelEntry::new("m2", "parts/relief.STL", "def");
        let obj = ModelEntry::new("m3", "parts/bust.obj", "ghi");
        let mf3 = ModelEntry::new("m4", "parts/case.3mf", "jkl");
        let fix = ModelEntry::new_fixture("f1", "fixtures/kurt_vise.step", "mno");
        assert_eq!(IconKind::for_model_entry(&step), IconKind::StepModel);
        assert_eq!(IconKind::for_model_entry(&stl), IconKind::MeshModel);
        assert_eq!(IconKind::for_model_entry(&obj), IconKind::MeshModel);
        assert_eq!(IconKind::for_model_entry(&mf3), IconKind::MeshModel);
        assert_eq!(IconKind::for_model_entry(&fix), IconKind::FixtureModel);
    }
}
