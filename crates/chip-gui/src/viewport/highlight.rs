use std::{collections::HashSet, sync::Arc};

use chip_cad::{
    CadDocument, EdgeId, FaceId, MeasureSnapTarget, MeasurementResult, PickTarget,
    expand_coaxial_hole_faces, measure_pair, measure_single,
};
use chip_cam::{
    BoundaryPreviewOverlay, ChainDirectionArrow3D, GeneratedToolpath, ReachAnalysisResult,
    ResolvedRotaryConfig, ResolvedStockGeometry, TabMarker3D, TriMesh3D,
};
use chip_math::{ArcPolyline2D, DMat4, DVec2, DVec3, DisplayUnit, MM_PER_INCH, Ray3, RigidFrame3};
#[cfg(feature = "gpu-compute")]
use chip_sim::SimStockColorMode;
use chip_sim::SimStockMesh;
use chip_sketch::{SketchLayer, SketchPickHit};

#[cfg(feature = "gpu-compute")]
use super::sim_gpu_render::GpuStockRenderHandle;
use super::tool_mesh::SimToolMesh;
use crate::{camera::CadCamera, theme::AppTheme};

/// Pass #6 simulation machined-stock render source consumed by [`crate::viewport::ViewportRenderer`].
///
/// Abstracts how simulated stock geometry reaches the GPU:
/// - [`Self::CpuMesh`] uploads a CPU-tessellated [`SimStockMesh`] into reusable GPU buffers only
///   when `revision` advances, applying `stock_model` on the GPU (so 4th-axis rotary playback
///   rotates without CPU re-tessellation).
/// - [`Self::GpuGrid`] (behind `#[cfg(feature = "gpu-compute")]`) renders live GPU-resident
///   Heightfield, 4th-axis Cylindrical, or Tri-Dexel indirect-draw buffers directly on the shared
///   `wgpu::Device` with zero CPU meshing readback and `< 0.1 ms` instant Error Heatmap updates.
#[derive(Debug, Clone)]
pub enum SimStockRenderSource {
    /// CPU-tessellated indexed stock mesh with monotonic upload `revision` and rigid Setup-WCS
    /// model matrix (`stock_model` uniform in `vs_sim_stock`).
    CpuMesh {
        revision: u64,
        mesh: Arc<SimStockMesh>,
        stock_model: DMat4,
    },
    /// Zero-readback direct GPU storage-buffer / indirect-draw render handle on the shared device.
    #[cfg(feature = "gpu-compute")]
    GpuGrid {
        revision: u64,
        handle: Arc<GpuStockRenderHandle>,
        stock_model: DMat4,
        color_mode: SimStockColorMode,
        mesh_skinning: bool,
    },
}

impl SimStockRenderSource {
    /// Monotonic revision identifier used to skip redundant GPU vertex/index re-uploads.
    #[cfg(test)]
    #[inline]
    pub const fn revision(&self) -> u64 {
        match self {
            Self::CpuMesh { revision, .. } => *revision,
            #[cfg(feature = "gpu-compute")]
            Self::GpuGrid { revision, .. } => *revision,
        }
    }

    /// Rigid Setup-WCS transform applied to the rendered stock on the GPU (`stock_model`).
    #[inline]
    pub const fn stock_model(&self) -> DMat4 {
        match self {
            Self::CpuMesh { stock_model, .. } => *stock_model,
            #[cfg(feature = "gpu-compute")]
            Self::GpuGrid { stock_model, .. } => *stock_model,
        }
    }

    /// Returns the underlying CPU mesh when backed by [`Self::CpuMesh`].
    #[cfg(test)]
    #[inline]
    pub fn cpu_mesh(&self) -> Option<&Arc<SimStockMesh>> {
        match self {
            Self::CpuMesh { mesh, .. } => Some(mesh),
            #[cfg(feature = "gpu-compute")]
            Self::GpuGrid { .. } => None,
        }
    }
}

/// Visual Z-height reference plane rendered across the active Setup's stock XY footprint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZPlaneVisual {
    pub z_wcs: f64,
    pub fill_rgba: [f32; 4],
    pub border_rgba: [f32; 4],
}

/// Cached generated toolpath item queued for Pass #5 GPU-instanced rendering (`08-ui-viewport.md`).
#[derive(Debug, Clone)]
pub struct ToolpathRenderEntry {
    pub op_id: String,
    pub cache_key: [u8; 32],
    pub toolpath: Arc<GeneratedToolpath>,
    pub is_focused: bool,
}

/// Visible 2D [`SketchLayer`] paired with its resolved 3D World [`RigidFrame3`] for GPU curve rendering.
#[derive(Debug, Clone)]
pub struct SketchLayerVisual {
    pub sketch: SketchLayer,
    pub world_frame: RigidFrame3,
}

/// Resolved 3D workholding fixture mesh queued for translucent shaded + wireframe viewport rendering.
#[derive(Debug, Clone)]
pub struct FixtureVisualMesh {
    pub revision: u64,
    pub fixture_id: String,
    pub mesh: Arc<TriMesh3D>,
    pub base_rgba: [f32; 4],
    pub colliding: bool,
    pub selected: bool,
}

/// Interactive click-to-measure state machine for [`crate::panels::ViewportPickMode::Measure`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeasureToolState {
    pub hovered_snap: Option<MeasureSnapTarget>,
    pub anchor1: Option<MeasureSnapTarget>,
    pub anchor2: Option<MeasureSnapTarget>,
    pub result: Option<MeasurementResult>,
}

impl MeasureToolState {
    /// Clears picked anchors and measurement result while keeping hover snap live.
    pub fn clear_measurement(&mut self) {
        self.anchor1 = None;
        self.anchor2 = None;
        self.result = None;
    }

    /// Resets all measurement state including hover snap (called on `Esc` / mode exit).
    pub fn reset_all(&mut self) {
        self.hovered_snap = None;
        self.clear_measurement();
    }

    /// Advances the click-to-measure state machine on clicking `target`:
    /// - Click 1 (or Click 3 after a completed pair): sets `anchor1` and computes `measure_single`.
    /// - Click 2: sets `anchor2` and computes `measure_pair(anchor1, anchor2)`.
    pub fn on_click_target(&mut self, target: MeasureSnapTarget) {
        if self.anchor1.is_none() || self.anchor2.is_some() {
            let res = measure_single(&target);
            self.anchor1 = Some(target);
            self.anchor2 = None;
            self.result = Some(res);
        } else if let Some(first) = &self.anchor1 {
            let res = measure_pair(first, &target);
            self.anchor2 = Some(target);
            self.result = Some(res);
        }
    }

    /// Returns a contextual Setup WCS Z height suitable for the **`Set Op Bottom Z`** helper button.
    pub fn suggested_bottom_z_wcs(&self) -> Option<f64> {
        match (&self.anchor1, &self.anchor2) {
            (Some(a), Some(b)) => match (a.suggested_bottom_z_wcs(), b.suggested_bottom_z_wcs()) {
                (Some(za), Some(zb)) => Some(za.min(zb)),
                (Some(z), None) | (None, Some(z)) => Some(z),
                (None, None) => None,
            },
            (Some(a), None) => a.suggested_bottom_z_wcs(),
            _ => None,
        }
    }
}

/// Active hover, multi-selection, WCS, Stock, Z-plane, 2D Sketch, and Toolpath scene state fed into the 3D CAD/CAM viewport.
#[derive(Debug, Clone)]
pub struct HighlightState {
    pub theme: AppTheme,
    pub hovered: Option<PickTarget>,
    pub selected: Option<PickTarget>,
    pub hovered_sketch: Option<SketchPickHit>,
    pub selected_sketch: Option<SketchPickHit>,
    pub extra_selected_faces: HashSet<FaceId>,
    pub extra_selected_edges: HashSet<EdgeId>,
    pub extra_selected_sketch_regions: HashSet<(String, String)>,
    pub extra_selected_sketch_entities: HashSet<(String, String)>,
    pub sketch_visuals: Vec<SketchLayerVisual>,
    pub sketch_preview_polys: Vec<(RigidFrame3, ArcPolyline2D)>,
    pub sketch_snap_marker: Option<(DVec3, &'static str, DVec2)>,
    pub chain_arrows: Vec<ChainDirectionArrow3D>,
    pub wcs_frame: RigidFrame3,
    pub stock: Option<ResolvedStockGeometry>,
    pub rotary: Option<ResolvedRotaryConfig>,
    pub show_stock: bool,
    pub show_fixtures: bool,
    pub fixture_meshes: Vec<FixtureVisualMesh>,
    pub z_planes_wcs: Vec<ZPlaneVisual>,
    /// Live cutting-area / machining-boundary 3D preview shown when `OperationTab::Geometry` is active.
    pub boundary_preview: Option<Arc<BoundaryPreviewOverlay>>,
    pub ghost_part: bool,
    pub show_grid: bool,
    pub toolpaths: Vec<ToolpathRenderEntry>,
    pub show_toolpaths: bool,
    pub show_rapids: bool,
    pub preview_tab_marker_wcs: Option<TabMarker3D>,
    /// Pass #6: Optional 3D machined stock render source (`CpuMesh` now, future `GpuGrid`).
    pub sim_stock_source: Option<SimStockRenderSource>,
    /// Pass #6: Optional 3D revolved Flute+Shank+Holder mesh (`(tool_rev, Arc<SimToolMesh>)`) + WCS tip position.
    pub sim_tool_mesh: Option<(u64, Arc<SimToolMesh>, DVec3)>,
    /// Optional global motion segment limit for progressive toolpath trail during playback.
    pub sim_trail_limit: Option<usize>,
    /// Current 4th-axis rotary angle (`deg`) during simulation playback (rotates wrapped toolpaths,
    /// ghost/CAD solid, and B-Rep edges in sync with cylindrical machined stock).
    pub sim_rotary_deg: f64,
    /// Active click-to-measure hover/anchor/leader visual overlay when Measure mode is active.
    pub measure_state: Option<MeasureToolState>,
    /// Toggle for the Minimum-Radius & Flute/Stickout Reach Heatmap overlay.
    pub show_reach_heatmap: bool,
    /// Cached background-computed Minimum-Radius & Reach analysis result for rendering.
    pub reach_analysis: Option<Arc<ReachAnalysisResult>>,
}

impl Default for HighlightState {
    fn default() -> Self {
        Self {
            theme: AppTheme::Dark,
            hovered: None,
            selected: None,
            hovered_sketch: None,
            selected_sketch: None,
            extra_selected_faces: HashSet::new(),
            extra_selected_edges: HashSet::new(),
            extra_selected_sketch_regions: HashSet::new(),
            extra_selected_sketch_entities: HashSet::new(),
            sketch_visuals: Vec::new(),
            sketch_preview_polys: Vec::new(),
            sketch_snap_marker: None,
            chain_arrows: Vec::new(),
            wcs_frame: RigidFrame3::IDENTITY,
            stock: None,
            rotary: None,
            show_stock: true,
            show_fixtures: true,
            fixture_meshes: Vec::new(),
            z_planes_wcs: Vec::new(),
            boundary_preview: None,
            ghost_part: false,
            show_grid: true,
            toolpaths: Vec::new(),
            show_toolpaths: true,
            show_rapids: true,
            preview_tab_marker_wcs: None,
            sim_stock_source: None,
            sim_tool_mesh: None,
            sim_trail_limit: None,
            sim_rotary_deg: 0.0,
            measure_state: None,
            show_reach_heatmap: false,
            reach_analysis: None,
        }
    }
}

impl HighlightState {
    /// Clears all project-scoped selection, overlay, toolpath, sketch, and simulation visual state
    /// while preserving user viewport display toggles and theme settings.
    pub fn clear_project_scene_state(&mut self) {
        self.hovered = None;
        self.selected = None;
        self.hovered_sketch = None;
        self.selected_sketch = None;
        self.extra_selected_faces.clear();
        self.extra_selected_edges.clear();
        self.extra_selected_sketch_regions.clear();
        self.extra_selected_sketch_entities.clear();
        self.sketch_visuals.clear();
        self.sketch_preview_polys.clear();
        self.sketch_snap_marker = None;
        self.chain_arrows.clear();
        self.wcs_frame = RigidFrame3::IDENTITY;
        self.stock = None;
        self.rotary = None;
        self.fixture_meshes.clear();
        self.z_planes_wcs.clear();
        self.boundary_preview = None;
        self.toolpaths.clear();
        self.preview_tab_marker_wcs = None;
        self.sim_stock_source = None;
        self.sim_tool_mesh = None;
        self.sim_trail_limit = None;
        self.sim_rotary_deg = 0.0;
        self.measure_state = None;
        self.reach_analysis = None;
    }

    /// Returns `true` when 4th-axis simulation playback is currently rotating the workpiece
    /// (and therefore the CAD/ghost model and its B-Rep edges) away from `0°`.
    #[inline]
    pub fn has_active_cad_rotary_transform(&self) -> bool {
        self.rotary.is_some() && self.sim_rotary_deg.abs() > 1e-6
    }

    /// Rigid Setup-WCS transform applied to the Pass #6 machined stock mesh on the GPU
    /// (`stock_model` uniform in `vs_sim_stock`).
    ///
    /// Returns [`DMat4::IDENTITY`] for 3-axis Heightmap and Tri-Dexel setups (where `rotary` is
    /// `None` or `sim_rotary_deg == 0.0`), and the exact rotary-axis rotation around
    /// `rot.center_wcs` matching [`ResolvedRotaryConfig::rotate_point_wcs`] for 4th-axis
    /// Cylindrical stock.
    pub fn stock_model_wcs_transform(&self) -> DMat4 {
        if let Some(rot) = self.rotary
            && self.sim_rotary_deg.abs() > 1e-6
        {
            let angle_rad = (self.sim_rotary_deg * rot.direction.sign()).to_radians();
            DMat4::from_translation(rot.center_wcs)
                * DMat4::from_axis_angle(rot.axis.axis_unit(), angle_rad)
                * DMat4::from_translation(-rot.center_wcs)
        } else {
            DMat4::IDENTITY
        }
    }

    /// Rigid World-space transform applied to the CAD solid / ghost model and its B-Rep edges
    /// so they rotate in exact lockstep with the 4th-axis rotary simulation stock and toolpaths.
    pub fn cad_world_transform(&self) -> DMat4 {
        if self.has_active_cad_rotary_transform() {
            self.wcs_frame.to_dmat4_local_to_world()
                * self.stock_model_wcs_transform()
                * self.wcs_frame.to_dmat4_world_to_local()
        } else {
            DMat4::IDENTITY
        }
    }

    /// Maps a World-space picking ray into unrotated CAD model space by applying the inverse of
    /// [`Self::cad_world_transform`], keeping hover/click raycasts aligned with a rotated ghost model.
    pub fn world_ray_to_cad_model_space(&self, ray: &Ray3) -> Ray3 {
        if let Some(rot) = self.rotary
            && self.sim_rotary_deg.abs() > 1e-6
        {
            let inv_angle_rad = (-self.sim_rotary_deg * rot.direction.sign()).to_radians();
            let inv_rot_wcs = DMat4::from_translation(rot.center_wcs)
                * DMat4::from_axis_angle(rot.axis.axis_unit(), inv_angle_rad)
                * DMat4::from_translation(-rot.center_wcs);
            let inv_world = self.wcs_frame.to_dmat4_local_to_world()
                * inv_rot_wcs
                * self.wcs_frame.to_dmat4_world_to_local();
            Ray3::new(
                inv_world.transform_point3(ray.origin),
                inv_world.transform_vector3(ray.dir),
            )
            .unwrap_or(*ray)
        } else {
            *ray
        }
    }

    pub(crate) fn build_face_states(&self, doc: Option<&CadDocument>) -> (u32, Vec<u32>) {
        let face_count = doc.map_or(64, |d| d.faces.len().max(64));
        let mut states = vec![0_u32; face_count];

        let mut hovered_fid = u32::MAX;
        if let Some(PickTarget::Face(fid)) = &self.hovered {
            hovered_fid = fid.0;
            if let Some(cad) = doc {
                for gfid in expand_coaxial_hole_faces(cad, *fid) {
                    if let Some(slot) = states.get_mut(gfid.0 as usize) {
                        *slot = 1;
                    }
                }
            } else if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 1;
            }
        }

        if let Some(PickTarget::Face(fid)) = &self.selected {
            if let Some(cad) = doc {
                for gfid in expand_coaxial_hole_faces(cad, *fid) {
                    if let Some(slot) = states.get_mut(gfid.0 as usize) {
                        *slot = 2;
                    }
                }
            } else if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 2;
            }
        }

        for &fid in &self.extra_selected_faces {
            if let Some(slot) = states.get_mut(fid.0 as usize) {
                *slot = 2;
            }
        }

        (hovered_fid, states)
    }

    pub(crate) fn collect_edge_sets(&self) -> (HashSet<EdgeId>, HashSet<EdgeId>) {
        let mut hov = HashSet::new();
        let mut sel = self.extra_selected_edges.clone();
        match &self.hovered {
            Some(PickTarget::Edge(eid)) => {
                hov.insert(*eid);
            }
            Some(PickTarget::Loop { edges, .. }) => {
                hov.extend(edges.iter().copied());
            }
            _ => {}
        }
        match &self.selected {
            Some(PickTarget::Edge(eid)) => {
                sel.insert(*eid);
            }
            Some(PickTarget::Loop { edges, .. }) => {
                sel.extend(edges.iter().copied());
            }
            _ => {}
        }
        (hov, sel)
    }
}

pub(crate) fn dmat4_to_f32_2d(m: DMat4) -> [[f32; 4]; 4] {
    let cols = m.to_cols_array_2d();
    let mut out = [[0.0_f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = cols[c][r] as f32;
        }
    }
    out
}

pub(crate) fn dvec3_f32(v: DVec3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

pub(crate) fn adaptive_grid_steps(
    camera: &CadCamera,
    height_px: f32,
    unit: DisplayUnit,
) -> (f32, f32) {
    let mm_per_px = camera.mm_per_pixel(height_px);
    let target_minor_mm = mm_per_px * 18.0;
    match unit {
        DisplayUnit::Millimeter => {
            let candidates = [0.1, 0.5, 1.0, 5.0, 10.0, 25.0, 50.0, 100.0];
            let minor = candidates
                .into_iter()
                .find(|&c| c >= target_minor_mm)
                .unwrap_or(100.0);
            (minor as f32, (minor * 10.0) as f32)
        }
        DisplayUnit::Inch => {
            let candidates_in = [0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0];
            let target_in = target_minor_mm / MM_PER_INCH;
            let minor_in = candidates_in
                .into_iter()
                .find(|&c| c >= target_in)
                .unwrap_or(5.0);
            let minor_mm = minor_in * MM_PER_INCH;
            let major_mm = minor_mm * 10.0;
            (minor_mm as f32, major_mm as f32)
        }
    }
}
