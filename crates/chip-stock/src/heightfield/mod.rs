//! 2.5D Z-Dexel heightfield volumetric stock kernel (`HeightfieldStock`).
//!
//! Stores stock-local `f32` height buffers relative to `origin_wcs` and `u32` operation tags
//! matching WGSL storage/atomic buffer layouts directly (zero `Vec<bool>`).

mod grid;
mod mesher;
pub(crate) mod sweep;

pub use grid::HeightfieldStock;
pub use mesher::{
    tessellate_stock_mesh, tessellate_stock_mesh_colored, tessellate_stock_mesh_with_options,
};
