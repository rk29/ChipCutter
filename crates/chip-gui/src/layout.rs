use egui_tiles::{Container, Linear, LinearDir, Shares, Tile, TileId, Tiles, Tree};
use serde::{Deserialize, Serialize};

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

/// Normalized split ratios for the default 4-zone CAM workspace, persisted in `prefs.toml`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WorkspaceLayoutShares {
    pub browser_share: f32,
    pub viewport_share: f32,
    pub inspector_share: f32,
    pub top_row_share: f32,
}

impl Default for WorkspaceLayoutShares {
    fn default() -> Self {
        Self {
            browser_share: 0.21,
            viewport_share: 0.55,
            inspector_share: 0.24,
            top_row_share: 0.74,
        }
    }
}

impl WorkspaceLayoutShares {
    /// Clamps ratios into safe visible ranges and normalizes horizontal top-row shares to sum to `1.0`.
    pub fn clamped(self) -> Self {
        let b = if self.browser_share.is_finite() {
            self.browser_share.clamp(0.12, 0.45)
        } else {
            0.21
        };
        let v = if self.viewport_share.is_finite() {
            self.viewport_share.clamp(0.25, 0.76)
        } else {
            0.55
        };
        let i = if self.inspector_share.is_finite() {
            self.inspector_share.clamp(0.14, 0.48)
        } else {
            0.24
        };
        let sum = (b + v + i).max(0.01);
        let top = if self.top_row_share.is_finite() {
            self.top_row_share.clamp(0.42, 0.88)
        } else {
            0.74
        };
        Self {
            browser_share: b / sum,
            viewport_share: v / sum,
            inspector_share: i / sum,
            top_row_share: top,
        }
    }

    pub fn approx_eq(self, other: Self) -> bool {
        (self.browser_share - other.browser_share).abs() < 1e-3
            && (self.viewport_share - other.viewport_share).abs() < 1e-3
            && (self.inspector_share - other.inspector_share).abs() < 1e-3
            && (self.top_row_share - other.top_row_share).abs() < 1e-3
    }
}

/// Builds the default CAM workspace (`08-ui-viewport.md` §1):
/// - Top row (74% height): `CAM Browser` (21%) | `3D CAD Viewport` (55%) | `Inspector` (24%)
/// - Bottom row (26% height): Tabbed `[Simulation Timeline] | [G-Code Preview] | [Diagnostics & Log]`
pub fn create_default_workspace_tree() -> Tree<WorkspacePane> {
    create_workspace_tree_with_shares(WorkspaceLayoutShares::default())
}

/// Builds the CAM workspace tree using clamped [`WorkspaceLayoutShares`].
pub fn create_workspace_tree_with_shares(shares: WorkspaceLayoutShares) -> Tree<WorkspacePane> {
    let s = shares.clamped();
    let mut tiles: Tiles<WorkspacePane> = Tiles::default();

    let browser: TileId = tiles.insert_pane(WorkspacePane::CamBrowser);
    let viewport: TileId = tiles.insert_pane(WorkspacePane::Viewport3D);
    let inspector: TileId = tiles.insert_pane(WorkspacePane::Inspector);

    let sim_pane: TileId = tiles.insert_pane(WorkspacePane::SimulationTimeline);
    let gcode_pane: TileId = tiles.insert_pane(WorkspacePane::GCodePreview);
    let diag_pane: TileId = tiles.insert_pane(WorkspacePane::DiagnosticsLog);
    let bottom_tabs: TileId = tiles.insert_tab_tile(vec![sim_pane, gcode_pane, diag_pane]);

    let mut top_shares = Shares::default();
    top_shares.set_share(browser, s.browser_share);
    top_shares.set_share(viewport, s.viewport_share);
    top_shares.set_share(inspector, s.inspector_share);

    let top_row = tiles.insert_container(Linear {
        children: vec![browser, viewport, inspector],
        dir: LinearDir::Horizontal,
        shares: top_shares,
    });

    let mut root_shares = Shares::default();
    root_shares.set_share(top_row, s.top_row_share);
    root_shares.set_share(bottom_tabs, (1.0 - s.top_row_share).max(0.12));

    let root = tiles.insert_container(Linear {
        children: vec![top_row, bottom_tabs],
        dir: LinearDir::Vertical,
        shares: root_shares,
    });

    Tree::new(egui::Id::unique("chipcutter_workspace"), root, tiles)
}

/// Extracts current [`WorkspaceLayoutShares`] from `tree` if the canonical root/top linear splitters
/// are intact, falling back cleanly to defaults for any rearranged panes.
pub fn extract_workspace_shares(tree: &Tree<WorkspacePane>) -> WorkspaceLayoutShares {
    let mut out = WorkspaceLayoutShares::default();
    let Some(root_id) = tree.root() else {
        return out;
    };
    if let Some(Tile::Container(Container::Linear(root_lin))) = tree.tiles.get(root_id)
        && root_lin.dir == LinearDir::Vertical
        && root_lin.children.len() == 2
    {
        let c0 = root_lin.children[0];
        let c1 = root_lin.children[1];
        let s0 = root_lin.shares[c0];
        let s1 = root_lin.shares[c1];
        if s0 + s1 > 1e-5 {
            out.top_row_share = s0 / (s0 + s1);
        }
        if let Some(Tile::Container(Container::Linear(top_lin))) = tree.tiles.get(c0)
            && top_lin.dir == LinearDir::Horizontal
        {
            let mut b_val = None;
            let mut v_val = None;
            let mut i_val = None;
            for &child_id in &top_lin.children {
                match tree.tiles.get(child_id) {
                    Some(Tile::Pane(WorkspacePane::CamBrowser)) => {
                        b_val = Some(top_lin.shares[child_id]);
                    }
                    Some(Tile::Pane(WorkspacePane::Viewport3D)) => {
                        v_val = Some(top_lin.shares[child_id]);
                    }
                    Some(Tile::Pane(WorkspacePane::Inspector)) => {
                        i_val = Some(top_lin.shares[child_id]);
                    }
                    _ => {}
                }
            }
            if let (Some(b), Some(v), Some(i)) = (b_val, v_val, i_val) {
                let sum = b + v + i;
                if sum > 1e-5 {
                    out.browser_share = b / sum;
                    out.viewport_share = v / sum;
                    out.inspector_share = i / sum;
                }
            }
        }
    }
    out.clamped()
}

/// Activates `target` if it sits inside a `Tabs` container.
pub fn focus_workspace_pane(tree: &mut Tree<WorkspacePane>, target: WorkspacePane) {
    let _ = tree.make_active(|_id, tile| matches!(tile, Tile::Pane(p) if *p == target));
}
