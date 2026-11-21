//! Shared stock surface color constants and per-operation color resolver across
//! Heightmap (`heightmap/mesher.rs`), Tri-Dexel (`tridexel/mesher.rs`), and
//! Cylindrical (`cylindrical.rs`) stock mesh builders.

use crate::heightmap::UNCUT_OP_IDX;

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
pub fn op_color(op_idx: u16, color_by_op: bool, is_side: bool) -> [f32; 4] {
    if op_idx == UNCUT_OP_IDX {
        if is_side {
            RAW_STOCK_SIDE_COLOR
        } else {
            RAW_STOCK_TOP_COLOR
        }
    } else if color_by_op {
        OPERATION_STOCK_PALETTE[usize::from(op_idx) % OPERATION_STOCK_PALETTE.len()]
    } else {
        MACHINED_METAL_COLOR
    }
}
