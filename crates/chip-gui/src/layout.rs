use egui_tiles::{Linear, LinearDir, Shares, TileId, Tiles, Tree};

/// Dockable workspace panes in ChipCutter (`egui_tiles`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkspacePane {
    CamBrowser,
    Viewport3D,
    Inspector,
    DiagnosticsLog,
}

impl WorkspacePane {
    pub const fn title(self) -> &'static str {
        match self {
            Self::CamBrowser => "CAM Browser",
            Self::Viewport3D => "3D CAD Viewport",
            Self::Inspector => "Inspector",
            Self::DiagnosticsLog => "Diagnostics & Log",
        }
    }
}

/// Builds the default 4-pane CAM workspace:
/// - Top row (78% height): `CAM Browser` (20%) | `3D CAD Viewport` (56%) | `Inspector` (24%)
/// - Bottom row (22% height): `Diagnostics & Log`
pub fn create_default_workspace_tree() -> Tree<WorkspacePane> {
    let mut tiles: Tiles<WorkspacePane> = Tiles::default();

    let browser: TileId = tiles.insert_pane(WorkspacePane::CamBrowser);
    let viewport: TileId = tiles.insert_pane(WorkspacePane::Viewport3D);
    let inspector: TileId = tiles.insert_pane(WorkspacePane::Inspector);
    let bottom_log: TileId = tiles.insert_pane(WorkspacePane::DiagnosticsLog);

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
    root_shares.set_share(top_row, 0.78);
    root_shares.set_share(bottom_log, 0.22);

    let root = tiles.insert_container(Linear {
        children: vec![top_row, bottom_log],
        dir: LinearDir::Vertical,
        shares: root_shares,
    });

    Tree::new(egui::Id::unique("chipcutter_workspace"), root, tiles)
}
