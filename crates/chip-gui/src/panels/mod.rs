pub mod about;
pub mod browser;
pub mod command_palette;
pub mod diagnostics;
pub mod gcode_preview;
pub mod machine_manager;
pub mod machine_tool_swap;
pub mod mesh_import_modal;
pub mod model_inspector;
pub mod operation_inspector;
pub mod post_profile_inspector;
pub mod project_properties;
pub mod settings;
pub mod setup_inspector;
pub mod simulation;
pub mod sketch_inspector;
pub mod tool_library;
pub mod vector_import_modal;
pub mod welcome_gallery;
pub mod workholding_wizards;

use chip_cad::EdgeId;

/// Interactive 2D primitive creation tool active inside [`ViewportPickMode::SketchAuthor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchAuthorTool {
    Rectangle,
    Circle,
    Slot,
    Polygon,
    BoltCircle,
    PlaceText,
}

impl SketchAuthorTool {
    pub const ALL: [Self; 6] = [
        Self::Rectangle,
        Self::Circle,
        Self::Slot,
        Self::Polygon,
        Self::BoltCircle,
        Self::PlaceText,
    ];

    pub const fn short_label(self) -> &'static str {
        match self {
            Self::Rectangle => "Rect",
            Self::Circle => "Circle",
            Self::Slot => "Slot",
            Self::Polygon => "Hex/Poly",
            Self::BoltCircle => "Bolt Circle",
            Self::PlaceText => "Text",
        }
    }

    pub const fn tooltip(self) -> &'static str {
        match self {
            Self::Rectangle => "2-Click Corner-to-Corner Rectangle on Sketch Plane",
            Self::Circle => "2-Click Center + Radius Circle on Sketch Plane",
            Self::Slot => "2-Click Center + End Stadium Slot on Sketch Plane",
            Self::Polygon => "2-Click Center + Radius Regular Hexagon on Sketch Plane",
            Self::BoltCircle => "2-Click Center + Bolt Circle Radius Pattern on Sketch Plane",
            Self::PlaceText => "1-Click Place CAM Text on Sketch Plane",
        }
    }
}

/// Currently inspected node in the right-hand Inspector pane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InspectorFocus {
    CadEntity,
    Model { model_idx: usize },
    Sketch { sketch_idx: usize },
    Setup { setup_idx: usize },
    Operation { setup_idx: usize, op_idx: usize },
}

/// Active sub-tab in the Setup Inspector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SetupTab {
    #[default]
    Wcs,
    Stock,
    Fixtures,
    Rotary,
    Heights,
}

/// Interactive workholding fixture type dropped by [`ViewportPickMode::PlaceFixtureClamp`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FixturePlaceKind {
    #[default]
    ToeClamp,
    CylinderKeepOut,
}

impl FixturePlaceKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::ToeClamp => "Toe Clamp",
            Self::CylinderKeepOut => "Cylinder Keep-Out",
        }
    }
}

/// Active sub-tab in the 5-tab Operation Inspector (`08-ui-viewport.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OperationTab {
    Tool,
    #[default]
    Geometry,
    Heights,
    Passes,
    Linking,
}

/// Destination for interactive 3D/2D geometry picking inside Operation `[Geometry]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GeometryPickTarget {
    /// Route viewport clicks into the operation's primary cutting geometry (`op.selections`).
    #[default]
    CuttingArea,
    /// Route viewport clicks into `op.avoid.selections` (or the actively edited `SharedAvoidRegion`).
    AvoidKeepOut,
}

/// Interactive 3D viewport selection capture mode driven by Setup WCS or Operation Geometry/Passes tabs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ViewportPickMode {
    #[default]
    InspectEntity,
    SetupWcsZ {
        setup_idx: usize,
    },
    SetupWcsX {
        setup_idx: usize,
    },
    SetupWcsXyPlane {
        setup_idx: usize,
    },
    SetupWcsXyPlaneXEdge {
        setup_idx: usize,
    },
    SetupWcsOriginFeature {
        setup_idx: usize,
    },
    OpPlanarFaces {
        setup_idx: usize,
        op_idx: usize,
    },
    OpClosedLoops {
        setup_idx: usize,
        op_idx: usize,
    },
    OpOpenChain {
        setup_idx: usize,
        op_idx: usize,
        start_edge: Option<EdgeId>,
    },
    OpCylindricalHoles {
        setup_idx: usize,
        op_idx: usize,
    },
    PlaceContourTab {
        setup_idx: usize,
        op_idx: usize,
    },
    PickSketchHostFace {
        setup_idx: usize,
    },
    RebindSketchHostFace {
        sketch_idx: usize,
    },
    SketchAuthor {
        sketch_idx: usize,
        tool: SketchAuthorTool,
    },
    PickModelLayFlatFace {
        model_idx: usize,
    },
    PlaceFixtureClamp {
        setup_idx: usize,
        kind: FixturePlaceKind,
    },
}

impl ViewportPickMode {
    pub fn banner_text(self) -> Option<String> {
        match self {
            Self::InspectEntity => None,
            Self::SetupWcsZ { .. } => {
                Some("Picking Setup WCS +Z Axis: Click a planar face, cylinder, or straight edge (Esc to cancel)".to_string())
            }
            Self::SetupWcsX { .. } => {
                Some("Picking Setup WCS +X Axis: Click a perpendicular straight edge or planar face (Esc to cancel)".to_string())
            }
            Self::SetupWcsXyPlane { .. } => {
                Some("Picking Setup WCS XY Plane: Click a planar face (Esc to cancel)".to_string())
            }
            Self::SetupWcsXyPlaneXEdge { .. } => Some(
                "Picking Setup WCS +X Edge: Click a straight edge to align +X in the XY plane (Esc to cancel)"
                    .to_string(),
            ),
            Self::SetupWcsOriginFeature { .. } => {
                Some("Picking WCS Origin Feature: Click a hole wall, circular edge, or edge midpoint (Esc to cancel)".to_string())
            }
            Self::OpPlanarFaces { .. } => {
                Some("Geometry Pick Active: Click horizontal planar faces in 3D viewport to toggle selection".to_string())
            }
            Self::OpClosedLoops { .. } => {
                Some("Geometry Pick Active: Click B-Rep edges to toggle closed boundary loops".to_string())
            }
            Self::OpOpenChain {
                start_edge: None, ..
            } => Some("Open Edge Chain (Step 1/2): Click the START edge in the 3D viewport".to_string()),
            Self::OpOpenChain {
                start_edge: Some(e),
                ..
            } => Some(format!(
                "Open Edge Chain (Step 2/2): Start Edge #{} set — click END edge on same body",
                e.0
            )),
            Self::OpCylindricalHoles { .. } => {
                Some("Geometry Pick Active: Click cylindrical hole walls to toggle selection".to_string())
            }
            Self::PlaceContourTab { .. } => Some(
                "Manual Holding Tab Mode: Click on selected contour edge to add a tab (Right-Click near tab to remove, Esc to exit)"
                    .to_string(),
            ),
            Self::PickSketchHostFace { .. } | Self::RebindSketchHostFace { .. } => Some(
                "Sketch on 3D Face: Click any planar B-Rep face in the 3D viewport to attach an associative 2D Sketch (Esc to cancel)"
                    .to_string(),
            ),
            Self::SketchAuthor { tool, .. } => Some(format!(
                "2D Sketch Draw [{}]: Click on the Sketch plane (2D Snapping active; Right-Click / Esc to exit)",
                tool.short_label()
            )),
            Self::PickModelLayFlatFace { .. } => Some(
                "Lay Flat on Face: Click any planar STEP face or STL triangle to lay it flat (-Z) on Z=0 (Esc to cancel)"
                    .to_string(),
            ),
            Self::PlaceFixtureClamp { kind, .. } => Some(format!(
                "Place {} Mode: Click on a stock edge, corner, or spoilboard in 3D to drop & orient a {} (Esc / Done to exit)",
                kind.label(),
                kind.label()
            )),
        }
    }

    /// Returns `(title, instruction, supports_done_button)` for the Top-Center Floating Guidance HUD.
    pub fn guidance_info(self) -> Option<(&'static str, String, bool)> {
        match self {
            Self::InspectEntity => None,
            Self::SetupWcsZ { .. } => Some((
                "Setup WCS +Z Axis",
                "Click a planar face, cylinder wall, or straight edge in 3D".to_string(),
                false,
            )),
            Self::SetupWcsX { .. } => Some((
                "Setup WCS +X Axis",
                "Click a perpendicular straight edge or planar face in 3D".to_string(),
                false,
            )),
            Self::SetupWcsXyPlane { .. } => Some((
                "Setup WCS XY Plane",
                "Click a planar B-Rep face to align WCS Z & origin".to_string(),
                false,
            )),
            Self::SetupWcsXyPlaneXEdge { .. } => Some((
                "Setup WCS +X Edge",
                "Click a straight B-Rep edge in 3D to align Setup +X in the XY plane".to_string(),
                false,
            )),
            Self::SetupWcsOriginFeature { .. } => Some((
                "Pick WCS Origin Feature",
                "Click a hole wall, circular edge, or straight edge midpoint".to_string(),
                false,
            )),
            Self::OpPlanarFaces { .. } => Some((
                "Pick Planar Faces",
                "Click horizontal planar faces in 3D to add/remove machining regions".to_string(),
                true,
            )),
            Self::OpClosedLoops { .. } => Some((
                "Pick Closed Boundary Loops",
                "Click B-Rep edges in 3D to toggle closed contour/chamfer loops".to_string(),
                true,
            )),
            Self::OpOpenChain {
                start_edge: None, ..
            } => Some((
                "Open Edge Chain · Step 1/2",
                "Click the START edge of the open profile in 3D".to_string(),
                true,
            )),
            Self::OpOpenChain {
                start_edge: Some(e),
                ..
            } => Some((
                "Open Edge Chain · Step 2/2",
                format!(
                    "Start Edge #{} locked — click the END edge on the same body",
                    e.0
                ),
                true,
            )),
            Self::OpCylindricalHoles { .. } => Some((
                "Pick Cylindrical Holes",
                "Click cylindrical hole walls in 3D to toggle drill/bore targets".to_string(),
                true,
            )),
            Self::PlaceContourTab { .. } => Some((
                "Manual Holding Tabs",
                "Click selected contour to place tab · Right-click near tab to delete".to_string(),
                true,
            )),
            Self::PickSketchHostFace { .. } | Self::RebindSketchHostFace { .. } => Some((
                "Attach Sketch to 3D Face",
                "Click any planar B-Rep face in 3D to anchor the associative 2D Sketch".to_string(),
                false,
            )),
            Self::SketchAuthor { tool, .. } => Some((
                "2D Sketch Authoring",
                format!(
                    "[{}] Click on Sketch plane (2D Snap active · Right-Click or Esc to finish)",
                    tool.short_label()
                ),
                true,
            )),
            Self::PickModelLayFlatFace { .. } => Some((
                "Lay Model Flat on Face",
                "Click any planar STEP face or STL triangle in 3D to place it flat (-Z) on Z = 0"
                    .to_string(),
                false,
            )),
            Self::PlaceFixtureClamp { kind, .. } => Some((
                match kind {
                    FixturePlaceKind::ToeClamp => "Place Toe Clamp in 3D",
                    FixturePlaceKind::CylinderKeepOut => "Place Cylinder Keep-Out in 3D",
                },
                "Click on a stock edge, corner, or spoilboard to drop & auto-orient workholding"
                    .to_string(),
                true,
            )),
        }
    }
}
