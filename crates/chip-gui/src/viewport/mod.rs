pub mod gizmos;
mod gpu_types;
pub mod highlight;
pub mod overlays;
pub mod pipeline;
pub mod scene_builders;
#[cfg(feature = "gpu-compute")]
mod sim_gpu_render;
pub mod toolpath_instances;

pub use highlight::{
    FixtureVisualMesh, HighlightState, SimStockRenderSource, SketchLayerVisual,
    ToolpathRenderEntry, ZPlaneVisual,
};
pub use overlays::{
    GuidanceHudAction, ViewportHudAction, draw_view_cube_overlay, draw_viewport_bottom_hud,
    draw_viewport_guidance_hud, draw_wcs_origin_27pt_overlay,
};
pub use pipeline::{ViewportRenderer, render_offscreen_png, render_offscreen_rgba_on_device};
