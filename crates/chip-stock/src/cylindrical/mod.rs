//! Polar / cylindrical heightmap stock kernel (`(u_axial, θ) -> r_max`) for 4th-axis rotary
//! setups (`X (A)`, `Y (B)`, or `Z (C)`), with stock-local `f32` radial fields and `u32` op tags.

mod grid;
mod mesher;
mod sweep;

pub use grid::CylindricalStock;
pub use mesher::{
    tessellate_cylindrical_stock_mesh, tessellate_cylindrical_stock_mesh_colored,
    tessellate_cylindrical_stock_mesh_with_options,
};
