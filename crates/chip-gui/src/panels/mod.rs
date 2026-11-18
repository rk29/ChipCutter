pub mod browser;
pub mod diagnostics;
pub mod gcode_preview;
pub mod operation_inspector;
pub mod setup_inspector;
pub mod simulation;
pub mod tool_library;

use chip_cad::EdgeId;

/// Currently inspected node in the right-hand Inspector pane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InspectorFocus {
    CadEntity,
    Setup { setup_idx: usize },
    Operation { setup_idx: usize, op_idx: usize },
}

/// Active sub-tab in the Setup Inspector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SetupTab {
    #[default]
    Wcs,
    Stock,
    Heights,
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
        }
    }
}
