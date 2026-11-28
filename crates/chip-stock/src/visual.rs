//! Shared stock surface color constants, visual coloring modes, and heatmap/operation color
//! resolvers across Heightfield (`heightfield/mesher.rs`), Tri-Dexel (`tridexel/mesher.rs`), and
//! Cylindrical (`cylindrical/mesher.rs`) stock mesh builders.

use serde::{Deserialize, Serialize};

use crate::types::UNCUT_OP_IDX;

/// Visual coloring mode for simulated 3D stock meshes (`Heightfield`, `TriDexel`, and 4th-axis `CylindricalStock`).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimStockColorMode {
    /// Uniform machined metallic shading (raw uncut stock slightly darker).
    Metallic,
    /// Distinct color per machining operation / tool (`OPERATION_STOCK_PALETTE`).
    #[default]
    ByOperation,
    /// Signed surface-deviation heatmap comparing machined stock against target geometry:
    /// Green = within `±tolerance_mm`, Cyan -> Blue = uncut excess (`+`), Yellow -> Red = gouge (`-`).
    ErrorHeatmap {
        tolerance_mm: f64,
        max_scale_mm: f64,
    },
}

/// Maps a signed surface deviation (`+` = uncut excess, `-` = overcut/gouge) to a perceptual
/// diverging RGBA color for the 3D stock comparison heatmap.
#[inline]
pub fn deviation_to_heatmap_rgba(
    signed_dev_mm: f64,
    tolerance_mm: f64,
    max_scale_mm: f64,
) -> [f32; 4] {
    let tol = tolerance_mm.max(1e-4);
    let span = (max_scale_mm - tol).max(tol);
    let abs_d = signed_dev_mm.abs();
    if abs_d <= tol {
        return [0.18, 0.78, 0.36, 1.0]; // In-tolerance emerald green
    }
    let t = ((abs_d - tol) / span).clamp(0.0, 1.0) as f32;
    if signed_dev_mm > 0.0 {
        [
            0.22 + t * (0.12 - 0.22),
            0.80 + t * (0.28 - 0.80),
            0.94 + t * (0.88 - 0.94),
            1.0,
        ]
    } else {
        [
            0.98 + t * (0.94 - 0.98),
            0.78 + t * (0.16 - 0.78),
            0.16 + t * (0.20 - 0.16),
            1.0,
        ]
    }
}

/// Distinct operation color palette for **Color Stock by Operation / Tool** (`06-simulation.md` §3).
pub const OPERATION_STOCK_PALETTE: [[f32; 4]; 8] = [
    [0.34, 0.66, 0.94, 1.0], // Azure (Op 1 - e.g. Facing)
    [0.24, 0.80, 0.62, 1.0], // Mint Emerald (Op 2 - Pocket)
    [0.92, 0.64, 0.26, 1.0], // Warm Amber (Op 3 - Step)
    [0.72, 0.48, 0.94, 1.0], // Violet (Op 4 - Bore)
    [0.94, 0.44, 0.56, 1.0], // Coral Rose (Op 5 - Drill)
    [0.28, 0.80, 0.86, 1.0], // Cyan Teal (Op 6 - Contour)
    [0.86, 0.80, 0.30, 1.0], // Citrine (Op 7 - Chamfer)
    [0.58, 0.78, 0.40, 1.0], // Lime Sage (Op 8+)
];

/// Uncut raw stock top/outer surface color (sRGB RGBA).
pub const RAW_STOCK_TOP_COLOR: [f32; 4] = [0.58, 0.62, 0.67, 1.0];
/// Uncut raw stock side/bottom/end-cap surface color (sRGB RGBA).
pub const RAW_STOCK_SIDE_COLOR: [f32; 4] = [0.50, 0.54, 0.59, 1.0];
/// Machined metallic surface color when `ByOperation` coloring is off (sRGB RGBA).
pub const MACHINED_METAL_COLOR: [f32; 4] = [0.76, 0.80, 0.85, 1.0];

/// Resolves the vertex sRGB RGBA color for a stock surface cell tagged with `op_idx`.
#[inline]
pub fn op_color(op_idx: u32, color_by_op: bool, is_side: bool) -> [f32; 4] {
    if op_idx == UNCUT_OP_IDX {
        if is_side {
            RAW_STOCK_SIDE_COLOR
        } else {
            RAW_STOCK_TOP_COLOR
        }
    } else if color_by_op {
        OPERATION_STOCK_PALETTE[(op_idx as usize) % OPERATION_STOCK_PALETTE.len()]
    } else {
        MACHINED_METAL_COLOR
    }
}
