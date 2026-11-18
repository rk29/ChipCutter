use egui_tiles::{Linear, LinearDir, Shares, Tile, TileId, Tiles, Tree};

/// Dockable workspace panes in ChipCutter (`egui_tiles`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkspacePane {
    CamBrowser,
    Viewport3D,
    Inspector,
    SimulationTimeline,
    GCodePreview,
    DiagnosticsLog,
}

impl WorkspacePane {
    pub const fn title(self) -> &'static str {
        match self {
            Self::CamBrowser => "CAM Browser",
            Self::Viewport3D => "3D CAD Viewport",
            Self::Inspector => "Inspector",
            Self::SimulationTimeline => "Simulation Timeline",
            Self::GCodePreview => "G-Code Preview",
            Self::DiagnosticsLog => "Diagnostics & Log",
        }
    }
}

/// Builds the default CAM workspace (`08-ui-viewport.md` §1):
/// - Top row (74% height): `CAM Browser` (21%) | `3D CAD Viewport` (55%) | `Inspector` (24%)
/// - Bottom row (26% height): Tabbed `[Simulation Timeline] | [G-Code Preview] | [Diagnostics & Log]`
pub fn create_default_workspace_tree() -> Tree<WorkspacePane> {
    let mut tiles: Tiles<WorkspacePane> = Tiles::default();

    let browser: TileId = tiles.insert_pane(WorkspacePane::CamBrowser);
    let viewport: TileId = tiles.insert_pane(WorkspacePane::Viewport3D);
    let inspector: TileId = tiles.insert_pane(WorkspacePane::Inspector);

    let sim_pane: TileId = tiles.insert_pane(WorkspacePane::SimulationTimeline);
    let gcode_pane: TileId = tiles.insert_pane(WorkspacePane::GCodePreview);
    let diag_pane: TileId = tiles.insert_pane(WorkspacePane::DiagnosticsLog);
    let bottom_tabs: TileId = tiles.insert_tab_tile(vec![sim_pane, gcode_pane, diag_pane]);

    let mut top_shares = Shares::default();
    top_shares.set_share(browser, 0.21);
    top_shares.set_share(viewport, 0.55);
    top_shares.set_share(inspector, 0.24);

    let top_row = tiles.insert_container(Linear {
        children: vec![browser, viewport, inspector],
        dir: LinearDir::Horizontal,
        shares: top_shares,
    });

    let mut root_shares = Shares::default();
    root_shares.set_share(top_row, 0.74);
    root_shares.set_share(bottom_tabs, 0.26);

    let root = tiles.insert_container(Linear {
        children: vec![top_row, bottom_tabs],
        dir: LinearDir::Vertical,
        shares: root_shares,
    });

    Tree::new(egui::Id::unique("chipcutter_workspace"), root, tiles)
}

/// Activates `target` if it sits inside a `Tabs` container.
pub fn focus_workspace_pane(tree: &mut Tree<WorkspacePane>, target: WorkspacePane) {
    let _ = tree.make_active(|_id, tile| matches!(tile, Tile::Pane(p) if *p == target));
}
