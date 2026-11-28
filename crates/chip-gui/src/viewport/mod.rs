pub mod gizmos;
mod gpu_types;
pub mod highlight;
pub mod overlays;
pub mod pipeline;
pub mod scene_builders;
#[cfg(feature = "gpu-compute")]
pub(crate) mod sim_gpu_render;
pub mod tool_mesh;
pub mod toolpath_instances;

pub use highlight::{
    FixtureVisualMesh, HighlightState, MeasureToolState, SimStockRenderSource, SketchLayerVisual,
    ToolpathRenderEntry, ZPlaneVisual,
};
pub use overlays::{
    GuidanceHudAction, MeasureCardAction, ViewportHudAction, draw_measure_hover_label,
    draw_measure_readout_card, draw_reach_legend_overlay, draw_view_cube_overlay,
    draw_viewport_bottom_hud, draw_viewport_guidance_hud, draw_wcs_origin_27pt_overlay,
};
pub use pipeline::ViewportRenderer;
#[cfg(test)]
pub use pipeline::{render_offscreen_png, render_offscreen_rgba_on_device};
#[cfg(feature = "gpu-compute")]
pub use sim_gpu_render::GpuStockRenderHandle;
pub use tool_mesh::{SimToolMesh, build_cutter_assembly_mesh};
