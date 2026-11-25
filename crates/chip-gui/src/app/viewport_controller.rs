use std::borrow::Cow;

use chip_cad::{CurveKind, PickFilter, PickTarget, SurfaceKind};
use chip_cam::{
    ExtractedFeature25D, FixtureBody, FixtureKind, GeometrySelectionItem, ManualTabAnchor,
    Operation, OperationRotaryMode, OperationStatus, ResolvedHeights, ResolvedRotaryConfig,
    ResolvedSetup, StrategyParams, TabMarker3D, TabPlacementMode, TabProfileKind, WcsAxisRef,
    WcsFeaturePointRef, WcsOrientationMode, WcsOriginMode, build_indexed_virtual_setup,
    extract_selection_features_with_sketches, resolve_sketch_world_frame,
    resolve_sketch_world_frame_with_doc,
};
use chip_math::{Aabb2, DMat4, DVec2, DVec3, ParamLength, Ray3, RigidFrame3};
use chip_project::cad_binding_to_sketch_face;
use chip_sketch::{
    FaceSketchOriginMode, PointPatternSpec, PolygonEvalSpec, PolygonSizingMode, RectAnchorMode,
    RectEvalSpec, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPickHit,
    SketchSnapContext, SketchTextSpec, SlotLengthMode, build_circle_polyline,
    build_rectangle_polyline, build_regular_polygon_polyline, build_slot_polyline,
    pick_sketch_layer_2d, snap_sketch_point_2d,
};

use crate::{
    app::ChipCutterApp,
    panels::{
        FixturePlaceKind, GeometryPickTarget, InspectorFocus, OperationTab, SetupTab,
        SketchAuthorTool, ViewportPickMode,
        operation_inspector::avoid_card::{
            mark_ops_dirty_with_rest, mark_shared_avoid_dependents_dirty,
        },
    },
    viewport::{
        BoundaryPreviewInput, FixtureVisualMesh, SharedAvoidPreviewInput, SketchLayerVisual,
        ToolpathRenderEntry, ZPlaneVisual,
    },
};

pub(crate) struct SketchAuthorInteractionInput {
    pub sketch_idx: usize,
    pub tool: SketchAuthorTool,
    pub ray: Ray3,
    pub mm_per_px: f64,
    pub primary_click: bool,
    pub right_click: bool,
}

/// Resolves the active viewport rotary angle (`deg`) and optional Indexed 3+1 HUD badge
/// `(axis_letter, angle_deg)` following strict precedence:
/// 1. Active simulation playback (`sim_active == true`) always owns `sim_rotary_deg`.
/// 2. Otherwise, when `auto_rotate_indexed_op == true` and `focused_op` is `Indexed3Plus1 { angle_deg }`
///    on a rotary-enabled Setup, auto-rotate to `angle_deg` and return the HUD badge when rotated.
/// 3. Otherwise `0.0°` with no indexed badge.
pub(crate) fn resolve_effective_viewport_rotary(
    sim_active: bool,
    sim_rotary_deg: f64,
    auto_rotate_indexed_op: bool,
    focused_op: Option<&Operation>,
    rotary: Option<ResolvedRotaryConfig>,
) -> (f64, Option<(char, f64)>) {
    if sim_active {
        return (sim_rotary_deg, None);
    }
    if auto_rotate_indexed_op
        && let Some(rot) = rotary
        && let Some(op) = focused_op
        && let OperationRotaryMode::Indexed3Plus1 { angle_deg } = op.rotary
    {
        let badge = (angle_deg.abs() > 1e-6).then_some((rot.axis_letter, angle_deg));
        return (angle_deg, badge);
    }
    (0.0, None)
}

/// Returns the effective [`ResolvedSetup`] used for `BoundaryPreviewOverlay`, `ZPlaneVisual`,
/// and `[Geometry]` / `[Heights]` inspection: when auto-rotate is active for an `Indexed3Plus1`
/// operation outside simulation playback, returns [`build_indexed_virtual_setup`] so previews and
/// heights match the exact upright `+Z` rotated orientation machined by `generate_operation_toolpath`.
pub(crate) fn effective_setup_for_indexed_preview<'a>(
    rs: &'a ResolvedSetup,
    auto_rotate_indexed_op: bool,
    sim_active: bool,
    op: &Operation,
) -> Cow<'a, ResolvedSetup> {
    if !sim_active
        && auto_rotate_indexed_op
        && let Some(rot) = rs.rotary
        && let OperationRotaryMode::Indexed3Plus1 { angle_deg } = op.rotary
    {
        return Cow::Owned(build_indexed_virtual_setup(rs, &rot, angle_deg));
    }
    Cow::Borrowed(rs)
}

fn transform_rigid_frame_by_dmat4(f: RigidFrame3, m: DMat4) -> RigidFrame3 {
    RigidFrame3 {
        origin_mm: m.transform_point3(f.origin_mm),
        x_axis: m.transform_vector3(f.x_axis).normalize_or_zero(),
        y_axis: m.transform_vector3(f.y_axis).normalize_or_zero(),
        z_axis: m.transform_vector3(f.z_axis).normalize_or_zero(),
    }
}

impl ChipCutterApp {
    pub(crate) fn sync_viewport_scene_state(&mut self) {
        self.highlight.theme = self.theme;
        self.highlight.extra_selected_faces.clear();
        self.highlight.extra_selected_edges.clear();
        self.highlight.extra_selected_sketch_regions.clear();
        self.highlight.extra_selected_sketch_entities.clear();
        self.highlight.sketch_visuals.clear();
        self.highlight.fixture_meshes.clear();
        self.highlight.chain_arrows.clear();
        self.highlight.z_planes_wcs.clear();
        self.highlight.boundary_preview = None;

        // 0. Resolve simulation visuals first so `sim_active` precedence is known before frame-geo
        //    caching, indexed auto-rotate, BoundaryPreviewOverlay, and ZPlaneVisual resolution.
        let mut sim_vis_rotary_deg = 0.0_f64;
        if self.effective_cad_arc().is_some()
            && self.project.setups.get(self.active_setup_idx).is_some()
        {
            let vis = if self.self_test.is_some() {
                self.sim_panel
                    .resolve_viewport_visuals_sync(&self.project, self.active_setup_idx)
            } else {
                self.sim_panel
                    .resolve_viewport_visuals(&self.project, self.active_setup_idx)
            };
            self.highlight.sim_stock_source = vis.stock;
            self.highlight.sim_tool_mesh = vis.cutter;
            self.highlight.sim_trail_limit = vis.trail_limit;
            sim_vis_rotary_deg = vis.rotary_deg;
        } else {
            self.highlight.sim_stock_source = None;
            self.highlight.sim_tool_mesh = None;
            self.highlight.sim_trail_limit = None;
        }
        let sim_active =
            self.highlight.sim_stock_source.is_some() || self.highlight.sim_tool_mesh.is_some();

        let cached_geo = self.ensure_frame_geo_cached().clone();
        let resolved_opt = cached_geo.resolved_setup.and_then(Result::ok);

        let focused_active_op: Option<&Operation> = match self.inspector_focus {
            InspectorFocus::Operation { setup_idx, op_idx }
                if setup_idx == self.active_setup_idx =>
            {
                self.project
                    .setups
                    .get(setup_idx)
                    .and_then(|s| s.operations.get(op_idx))
            }
            _ => None,
        };

        if let Some(rs) = &resolved_opt {
            self.highlight.wcs_frame = rs.frame;
            self.camera.turntable_frame = rs.frame;
            self.highlight.stock = Some(rs.stock);
            self.highlight.rotary = rs.rotary;

            let (eff_rotary_deg, _) = resolve_effective_viewport_rotary(
                sim_active,
                sim_vis_rotary_deg,
                self.prefs.auto_rotate_indexed_op,
                focused_active_op,
                rs.rotary,
            );
            self.highlight.sim_rotary_deg = eff_rotary_deg;

            let selected_fix_id = if matches!(
                self.inspector_focus,
                InspectorFocus::Setup { setup_idx } if setup_idx == self.active_setup_idx
            ) && self.setup_tab == SetupTab::Fixtures
            {
                self.selected_fixture_idx
                    .and_then(|fi| {
                        self.project
                            .setups
                            .get(self.active_setup_idx)
                            .and_then(|s| s.fixtures.get(fi))
                    })
                    .map(|f| f.id.as_str())
            } else {
                None
            };
            let colliding_fixture_ids: std::collections::HashSet<&str> = self
                .sim_panel
                .active_result
                .as_ref()
                .filter(|_| self.sim_panel.active_setup_idx == self.active_setup_idx)
                .map(|res| {
                    res.diagnostics
                        .iter()
                        .filter(|d| d.kind == chip_sim::SimIssueKind::FixtureCollision)
                        .filter_map(|d| d.fixture_id.as_deref())
                        .collect()
                })
                .unwrap_or_default();
            let keepout_fixture_ids: &[String] = match self.inspector_focus {
                InspectorFocus::KeepOutRegion { region_idx } => self
                    .project
                    .avoid_regions
                    .get(region_idx)
                    .map_or(&[], |sr| sr.fixture_ids.as_slice()),
                _ => &[],
            };
            for (sol, mesh_arc) in cached_geo
                .resolved_fixtures
                .iter()
                .zip(cached_geo.fixture_mesh_arcs.iter())
            {
                let is_selected_fixture = selected_fix_id == Some(sol.fixture_id.as_str())
                    || keepout_fixture_ids.iter().any(|id| id == &sol.fixture_id);
                self.highlight.fixture_meshes.push(FixtureVisualMesh {
                    revision: cached_geo.sig,
                    fixture_id: sol.fixture_id.clone(),
                    mesh: std::sync::Arc::clone(mesh_arc),
                    base_rgba: sol.color_rgba,
                    colliding: colliding_fixture_ids.contains(sol.fixture_id.as_str()),
                    selected: is_selected_fixture,
                });
            }
            let has_cad_rot = self.highlight.has_active_cad_rotary_transform();
            let cad_xf = if has_cad_rot {
                self.highlight.cad_world_transform()
            } else {
                DMat4::IDENTITY
            };
            // For `SetupXy` sketches when auto-rotated to an Indexed 3+1 angle, keep the sketch
            // flat & upright in `rs.frame` at the upright indexed stock/model Z height.
            let upright_rs_for_xy: Option<ResolvedSetup> = focused_active_op.and_then(|op| {
                match effective_setup_for_indexed_preview(
                    rs,
                    self.prefs.auto_rotate_indexed_op,
                    sim_active,
                    op,
                ) {
                    Cow::Owned(mut v) => {
                        v.frame = rs.frame;
                        Some(v)
                    }
                    Cow::Borrowed(_) => None,
                }
            });
            for (si, sk) in self.project.sketches.iter().enumerate() {
                if sk.visible {
                    let rs_for_sk = match (&sk.anchor, &upright_rs_for_xy) {
                        (SketchAnchor::SetupXy { .. }, Some(u_rs)) => u_rs,
                        _ => rs,
                    };
                    let mut wf = if let Some(m) = &self.model {
                        resolve_sketch_world_frame_with_doc(&m.doc, sk, rs_for_sk)
                    } else {
                        resolve_sketch_world_frame(false, sk, rs_for_sk)
                    };
                    if has_cad_rot && matches!(sk.anchor, SketchAnchor::PlanarFace { .. }) {
                        wf = transform_rigid_frame_by_dmat4(wf, cad_xf);
                    }
                    if matches!(self.inspector_focus, InspectorFocus::Sketch { sketch_idx } if sketch_idx == si)
                        && matches!(sk.anchor, SketchAnchor::PlanarFace { .. })
                    {
                        self.camera.turntable_frame = wf;
                    }
                    self.highlight.sketch_visuals.push(SketchLayerVisual {
                        sketch: sk.clone(),
                        world_frame: wf,
                    });
                }
            }
        } else {
            self.highlight.wcs_frame = RigidFrame3::IDENTITY;
            self.highlight.stock = None;
            self.highlight.rotary = None;
            self.highlight.sim_rotary_deg = 0.0;
        }

        match self.inspector_focus {
            InspectorFocus::Setup { setup_idx } => {
                if self.show_z_planes
                    && self.setup_tab == SetupTab::Heights
                    && setup_idx == self.active_setup_idx
                    && let Some(rs) = &resolved_opt
                {
                    self.highlight.z_planes_wcs.push(ZPlaneVisual {
                        z_wcs: rs.clearance_z_wcs,
                        fill_rgba: [0.24, 0.72, 1.0, 0.11],
                        border_rgba: [0.34, 0.80, 1.0, 0.85],
                    });
                    self.highlight.z_planes_wcs.push(ZPlaneVisual {
                        z_wcs: rs.retract_z_wcs,
                        fill_rgba: [1.0, 0.72, 0.22, 0.10],
                        border_rgba: [1.0, 0.78, 0.28, 0.85],
                    });
                }
            }
            InspectorFocus::Operation { setup_idx, op_idx } => {
                if let Some(rs) = &resolved_opt
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && let Some(op) = setup.operations.get(op_idx)
                {
                    let extracted_opt = cached_geo
                        .focused_extracted
                        .as_ref()
                        .and_then(|r| r.as_ref().ok());
                    if let Some(sum) = extracted_opt {
                        for &fid in &sum.highlight_faces {
                            self.highlight.extra_selected_faces.insert(fid);
                        }
                        for &eid in &sum.highlight_edges {
                            self.highlight.extra_selected_edges.insert(eid);
                        }
                        for rkey in &sum.highlight_sketch_regions {
                            self.highlight
                                .extra_selected_sketch_regions
                                .insert(rkey.clone());
                        }
                        for ekey in &sum.highlight_sketch_entities {
                            self.highlight
                                .extra_selected_sketch_entities
                                .insert(ekey.clone());
                        }
                        self.highlight.chain_arrows = sum.direction_arrows.clone();
                    }

                    let eff_rs = if setup_idx == self.active_setup_idx {
                        effective_setup_for_indexed_preview(
                            rs,
                            self.prefs.auto_rotate_indexed_op,
                            sim_active,
                            op,
                        )
                    } else {
                        Cow::Borrowed(rs)
                    };

                    if self.op_tab == OperationTab::Geometry && setup_idx == self.active_setup_idx {
                        let active_tool = op.tool_id.as_ref().and_then(|tid| {
                            self.project
                                .tool(tid)
                                .or_else(|| self.global_tools.tool(tid))
                        });
                        let (doc_opt, cad_rev) = match &self.model {
                            Some(m) => (Some(m.doc.as_ref()), m.revision),
                            None => (None, 0),
                        };
                        self.highlight.boundary_preview = self
                            .boundary_preview_cache
                            .get_or_compute(&BoundaryPreviewInput {
                                active: true,
                                setup_idx,
                                op_idx,
                                cad_rev,
                                frame_geo_sig: cached_geo.sig,
                                display_unit: self.project.display_units,
                                doc: doc_opt,
                                sketches: &self.project.sketches,
                                resolved_setup: eff_rs.as_ref(),
                                fixtures_frame: &rs.frame,
                                resolved_fixtures: &cached_geo.resolved_fixtures,
                                shared_avoid_regions: &self.project.avoid_regions,
                                op,
                                active_tool,
                                extracted: extracted_opt,
                            });
                    }

                    if self.show_z_planes
                        && self.op_tab == OperationTab::Heights
                        && let Some(sum) = extracted_opt
                    {
                        let mut ctx = eff_rs.eval_context(self.project.display_units);
                        if let Some(tid) = &op.tool_id
                            && let Some(t) = self.project.tool(tid)
                        {
                            t.populate_eval_context(&mut ctx);
                        }
                        ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
                        ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
                        let mut h_eval = op.heights.clone();
                        h_eval.reevaluate(&ctx);
                        if let Ok(rh) = ResolvedHeights::resolve(
                            &h_eval,
                            &eff_rs.model_aabb_wcs,
                            &eff_rs.stock.aabb_wcs(),
                            eff_rs.clearance_z_wcs,
                            eff_rs.retract_z_wcs,
                            sum.inferred_top_z_wcs,
                            sum.inferred_bottom_z_wcs,
                        ) {
                            self.highlight.z_planes_wcs = vec![
                                ZPlaneVisual {
                                    z_wcs: rh.clearance_z_wcs,
                                    fill_rgba: [0.24, 0.72, 1.0, 0.10],
                                    border_rgba: [0.34, 0.80, 1.0, 0.85],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.retract_z_wcs,
                                    fill_rgba: [1.0, 0.72, 0.22, 0.09],
                                    border_rgba: [1.0, 0.78, 0.28, 0.85],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.feed_z_wcs,
                                    fill_rgba: [0.26, 0.90, 0.46, 0.09],
                                    border_rgba: [0.30, 0.94, 0.50, 0.85],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.top_z_wcs,
                                    fill_rgba: [0.78, 0.52, 1.0, 0.10],
                                    border_rgba: [0.84, 0.60, 1.0, 0.88],
                                },
                                ZPlaneVisual {
                                    z_wcs: rh.bottom_z_wcs,
                                    fill_rgba: [1.0, 0.36, 0.52, 0.12],
                                    border_rgba: [1.0, 0.44, 0.58, 0.92],
                                },
                            ];
                        }
                    }
                }
            }
            InspectorFocus::KeepOutRegion { region_idx } => {
                if let Some(rs) = &resolved_opt
                    && let Some(sr) = self.project.avoid_regions.get(region_idx)
                {
                    let (doc_opt, cad_rev) = match &self.model {
                        Some(m) => (Some(m.doc.as_ref()), m.revision),
                        None => (None, 0),
                    };
                    if let Some(doc) = doc_opt
                        && let Ok(sum) = extract_selection_features_with_sketches(
                            doc,
                            &self.project.sketches,
                            rs,
                            &sr.selections,
                        )
                    {
                        for &fid in &sum.highlight_faces {
                            self.highlight.extra_selected_faces.insert(fid);
                        }
                        for &eid in &sum.highlight_edges {
                            self.highlight.extra_selected_edges.insert(eid);
                        }
                        for rkey in &sum.highlight_sketch_regions {
                            self.highlight
                                .extra_selected_sketch_regions
                                .insert(rkey.clone());
                        }
                        for ekey in &sum.highlight_sketch_entities {
                            self.highlight
                                .extra_selected_sketch_entities
                                .insert(ekey.clone());
                        }
                    } else {
                        for sel in &sr.selections {
                            match sel {
                                GeometrySelectionItem::SketchRegion {
                                    sketch_id,
                                    region_id,
                                } => {
                                    self.highlight
                                        .extra_selected_sketch_regions
                                        .insert((sketch_id.clone(), region_id.clone()));
                                }
                                GeometrySelectionItem::SketchEntity {
                                    sketch_id,
                                    entity_id,
                                    ..
                                } => {
                                    self.highlight
                                        .extra_selected_sketch_entities
                                        .insert((sketch_id.clone(), entity_id.clone()));
                                }
                                _ => {}
                            }
                        }
                    }
                    self.highlight.boundary_preview = self
                        .boundary_preview_cache
                        .get_or_compute_shared_avoid(&SharedAvoidPreviewInput {
                            region_idx,
                            cad_rev,
                            frame_geo_sig: cached_geo.sig,
                            display_unit: self.project.display_units,
                            doc: doc_opt,
                            sketches: &self.project.sketches,
                            resolved_setup: rs,
                            resolved_fixtures: &cached_geo.resolved_fixtures,
                            shared_region: sr,
                        });
                }
            }
            InspectorFocus::Project
            | InspectorFocus::ProjectTool { .. }
            | InspectorFocus::CadEntity
            | InspectorFocus::Model { .. }
            | InspectorFocus::Sketch { .. } => {}
        }

        if let ViewportPickMode::OpOpenChain {
            start_edge: Some(se),
            ..
        } = self.pick_mode
        {
            self.highlight.extra_selected_edges.insert(se);
        }

        self.highlight.toolpaths.clear();
        if let Some((_, cad_rev)) = self.effective_cad_arc()
            && let Some(setup) = self.project.setups.get(self.active_setup_idx)
        {
            let focused_op_idx = match self.inspector_focus {
                InspectorFocus::Operation { setup_idx, op_idx }
                    if setup_idx == self.active_setup_idx =>
                {
                    Some(op_idx)
                }
                _ => None,
            };

            for (o_idx, op) in setup.operations.iter().enumerate() {
                if op.suppressed || self.hidden_operation_ids.contains(&op.id) {
                    continue;
                }
                let is_focused = focused_op_idx.is_none_or(|f_idx| f_idx == o_idx);
                if !self.show_all_setup_toolpaths && !is_focused {
                    continue;
                }
                if let Some((hash, tp)) = self.toolpath_cache.get_for_operation(
                    &self.project,
                    self.active_setup_idx,
                    o_idx,
                    cad_rev,
                ) {
                    if let Some(stage) = &tp.clearing_stage {
                        let mut stage_key = hash.0;
                        stage_key[0] ^= 0x53;
                        stage_key[1] ^= 0x31;
                        self.highlight.toolpaths.push(ToolpathRenderEntry {
                            op_id: format!("{}_clear", op.id),
                            cache_key: stage_key,
                            toolpath: std::sync::Arc::new(stage.toolpath.clone()),
                            is_focused,
                        });
                    }
                    self.highlight.toolpaths.push(ToolpathRenderEntry {
                        op_id: op.id.clone(),
                        cache_key: hash.0,
                        toolpath: tp,
                        is_focused,
                    });
                }
            }
        }
    }

    pub(crate) fn effective_pick_filter(&self) -> PickFilter {
        match self.pick_mode {
            ViewportPickMode::InspectEntity => self.pick_filter,
            ViewportPickMode::SetupWcsZ { .. } | ViewportPickMode::SetupWcsX { .. } => {
                PickFilter::Any
            }
            ViewportPickMode::SetupWcsXyPlane { .. }
            | ViewportPickMode::OpPlanarFaces { .. }
            | ViewportPickMode::PickSketchHostFace { .. }
            | ViewportPickMode::RebindSketchHostFace { .. }
            | ViewportPickMode::PickModelLayFlatFace { .. } => PickFilter::FacesOnly,
            ViewportPickMode::SetupWcsOriginFeature { .. } => PickFilter::Any,
            ViewportPickMode::OpClosedLoops { .. } => PickFilter::EdgeLoops,
            ViewportPickMode::OpOpenChain { .. }
            | ViewportPickMode::PlaceContourTab { .. }
            | ViewportPickMode::SetupWcsXyPlaneXEdge { .. } => PickFilter::EdgesOnly,
            ViewportPickMode::OpCylindricalHoles { .. } => PickFilter::CylindricalHolesOnly,
            ViewportPickMode::SketchAuthor { .. }
            | ViewportPickMode::PlaceFixtureClamp { .. }
            | ViewportPickMode::PickKeepOutGeometry { .. } => PickFilter::Any,
        }
    }

    /// Ray-plane hit-testing against visible 2D [`chip_sketch::SketchLayer`]s in the active Setup.
    pub(crate) fn pick_sketch_in_viewport(
        &mut self,
        ray: &Ray3,
        mm_per_px: f64,
    ) -> Option<SketchPickHit> {
        if self.project.sketches.is_empty() {
            return None;
        }
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return None;
        };
        let doc_has_bodies = self.model.is_some();
        let edge_tol_mm = (10.0 * mm_per_px).clamp(0.4, 8.0);
        let prefer_regions = matches!(self.pick_mode, ViewportPickMode::OpPlanarFaces { .. });

        let mut best_t = f64::INFINITY;
        let mut best_hit: Option<SketchPickHit> = None;

        let has_cad_rot = self.highlight.has_active_cad_rotary_transform();
        let cad_xf = if has_cad_rot {
            self.highlight.cad_world_transform()
        } else {
            DMat4::IDENTITY
        };
        for sk in &self.project.sketches {
            if !sk.visible {
                continue;
            }
            let mut wf = if let Some(m) = &self.model {
                resolve_sketch_world_frame_with_doc(&m.doc, sk, &rs)
            } else {
                resolve_sketch_world_frame(doc_has_bodies, sk, &rs)
            };
            if has_cad_rot && matches!(sk.anchor, SketchAnchor::PlanarFace { .. }) {
                wf = transform_rigid_frame_by_dmat4(wf, cad_xf);
            }
            let denom = ray.dir.dot(wf.z_axis);
            if denom.abs() <= 1e-6 {
                continue;
            }
            let t = (wf.origin_mm - ray.origin).dot(wf.z_axis) / denom;
            if t <= 0.0 || t >= best_t {
                continue;
            }
            let hit_world = ray.origin + ray.dir * t;
            let hit_local = wf.world_to_local_point(hit_world);
            let query_xy = DVec2::new(hit_local.x, hit_local.y);
            if let Some(hit) = pick_sketch_layer_2d(sk, query_xy, edge_tol_mm, prefer_regions) {
                best_t = t;
                best_hit = Some(hit);
            }
        }
        best_hit
    }

    /// Toggles (or appends for `OpenChain`) a picked `GeometrySelectionItem` into either:
    /// 1. the actively edited [`chip_cam::SharedAvoidRegion`] (`editing_shared_avoid_id`),
    /// 2. the operation's local `op.avoid.selections` (`GeometryPickTarget::AvoidKeepOut`), or
    /// 3. the operation's primary `op.selections` (`GeometryPickTarget::CuttingArea`).
    ///
    /// Records an atomic Undo step and marks affected + downstream Rest operations `Dirty`.
    fn apply_geometry_pick_toggle<F>(
        &mut self,
        setup_idx: usize,
        op_idx: usize,
        undo_label: &str,
        new_item: GeometrySelectionItem,
        toggle_match: Option<F>,
    ) where
        F: Fn(&GeometrySelectionItem) -> bool,
    {
        if let Some(eid) = self.editing_shared_avoid_id.clone()
            && self.project.avoid_region(&eid).is_some()
        {
            self.history.record_before_edit(
                format!("{undo_label} (Shared Avoid)"),
                None,
                &self.project,
            );
            if let Some(sr) = self.project.avoid_region_mut(&eid) {
                if let Some(pred) = toggle_match
                    && let Some(pos) = sr.selections.iter().position(pred)
                {
                    sr.selections.remove(pos);
                } else {
                    sr.selections.push(new_item);
                }
            }
            mark_shared_avoid_dependents_dirty(&mut self.project, &eid);
            return;
        }

        if setup_idx >= self.project.setups.len()
            || op_idx >= self.project.setups[setup_idx].operations.len()
        {
            return;
        }

        if self.geometry_pick_target == GeometryPickTarget::AvoidKeepOut {
            self.history
                .record_before_edit(format!("{undo_label} (Avoid)"), None, &self.project);
            let sels = &mut self.project.setups[setup_idx].operations[op_idx]
                .avoid
                .selections;
            if let Some(pred) = toggle_match
                && let Some(pos) = sels.iter().position(pred)
            {
                sels.remove(pos);
            } else {
                sels.push(new_item);
            }
            mark_ops_dirty_with_rest(&mut self.project.setups[setup_idx], &[op_idx]);
            return;
        }

        self.history
            .record_before_edit(undo_label, None, &self.project);
        let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
        if matches!(op_mut.runtime_status, OperationStatus::NeedsRepick { .. }) {
            op_mut.selections.clear();
        }
        let sels = &mut op_mut.selections;
        if let Some(pred) = toggle_match
            && let Some(pos) = sels.iter().position(pred)
        {
            sels.remove(pos);
        } else {
            sels.push(new_item);
        }
        mark_ops_dirty_with_rest(&mut self.project.setups[setup_idx], &[op_idx]);
    }

    /// Toggles `new_item` directly inside `project.avoid_regions[region_idx]` with atomic Undo
    /// and dependent operation dirty propagation (used by `ViewportPickMode::PickKeepOutGeometry`).
    fn apply_keepout_region_pick_toggle<F>(
        &mut self,
        region_idx: usize,
        undo_label: &str,
        new_item: GeometrySelectionItem,
        toggle_match: F,
    ) where
        F: Fn(&GeometrySelectionItem) -> bool,
    {
        let Some(rid) = self
            .project
            .avoid_regions
            .get(region_idx)
            .map(|r| r.id.clone())
        else {
            return;
        };
        self.history.record_before_edit(
            format!("{undo_label} (Shared Avoid)"),
            None,
            &self.project,
        );
        if let Some(sr) = self.project.avoid_regions.get_mut(region_idx) {
            if let Some(pos) = sr.selections.iter().position(toggle_match) {
                sr.selections.remove(pos);
            } else {
                sr.selections.push(new_item);
            }
        }
        mark_shared_avoid_dependents_dirty(&mut self.project, &rid);
        self.cached_frame_geo = None;
    }

    pub(crate) fn handle_viewport_sketch_click(&mut self, hit: SketchPickHit) {
        match self.pick_mode {
            ViewportPickMode::InspectEntity => {
                let sk_id = match &hit {
                    SketchPickHit::Entity { sketch_id, .. }
                    | SketchPickHit::Region { sketch_id, .. } => sketch_id.clone(),
                };
                if let Some(idx) = self.project.sketches.iter().position(|s| s.id == sk_id) {
                    self.inspector_focus = InspectorFocus::Sketch { sketch_idx: idx };
                }
                self.highlight.selected_sketch = Some(hit);
            }
            ViewportPickMode::PickKeepOutGeometry { region_idx } => {
                let new_item = match hit {
                    SketchPickHit::Region {
                        sketch_id,
                        region_id,
                        ..
                    } => GeometrySelectionItem::SketchRegion {
                        sketch_id,
                        region_id,
                    },
                    SketchPickHit::Entity {
                        sketch_id,
                        entity_id,
                        ..
                    } => GeometrySelectionItem::SketchEntity {
                        sketch_id,
                        entity_id,
                        reverse: false,
                        side_flip: false,
                    },
                };
                let match_item = new_item.clone();
                self.apply_keepout_region_pick_toggle(
                    region_idx,
                    "Toggle 2D Sketch Selection",
                    new_item,
                    move |existing| match (existing, &match_item) {
                        (
                            GeometrySelectionItem::SketchRegion {
                                sketch_id: a,
                                region_id: ra,
                            },
                            GeometrySelectionItem::SketchRegion {
                                sketch_id: b,
                                region_id: rb,
                            },
                        ) => a == b && ra == rb,
                        (
                            GeometrySelectionItem::SketchEntity {
                                sketch_id: a,
                                entity_id: ea,
                                ..
                            },
                            GeometrySelectionItem::SketchEntity {
                                sketch_id: b,
                                entity_id: eb,
                                ..
                            },
                        ) => a == b && ea == eb,
                        _ => false,
                    },
                );
            }
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx }
            | ViewportPickMode::OpClosedLoops { setup_idx, op_idx }
            | ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx }
            | ViewportPickMode::OpOpenChain {
                setup_idx, op_idx, ..
            } => {
                let new_item = match hit {
                    SketchPickHit::Region {
                        sketch_id,
                        region_id,
                        ..
                    } => GeometrySelectionItem::SketchRegion {
                        sketch_id,
                        region_id,
                    },
                    SketchPickHit::Entity {
                        sketch_id,
                        entity_id,
                        ..
                    } => GeometrySelectionItem::SketchEntity {
                        sketch_id,
                        entity_id,
                        reverse: false,
                        side_flip: false,
                    },
                };
                let match_item = new_item.clone();
                self.apply_geometry_pick_toggle(
                    setup_idx,
                    op_idx,
                    "Toggle 2D Sketch Selection",
                    new_item,
                    Some(
                        move |existing: &GeometrySelectionItem| match (existing, &match_item) {
                            (
                                GeometrySelectionItem::SketchRegion {
                                    sketch_id: a,
                                    region_id: ra,
                                },
                                GeometrySelectionItem::SketchRegion {
                                    sketch_id: b,
                                    region_id: rb,
                                },
                            ) => a == b && ra == rb,
                            (
                                GeometrySelectionItem::SketchEntity {
                                    sketch_id: a,
                                    entity_id: ea,
                                    ..
                                },
                                GeometrySelectionItem::SketchEntity {
                                    sketch_id: b,
                                    entity_id: eb,
                                    ..
                                },
                            ) => a == b && ea == eb,
                            _ => false,
                        },
                    ),
                );
            }
            _ => {}
        }
    }

    pub(crate) fn handle_viewport_pick_click(&mut self, picked: Option<PickTarget>) {
        let Some(target) = picked else {
            if self.pick_mode == ViewportPickMode::InspectEntity {
                self.highlight.selected = None;
            }
            return;
        };

        let Some(m) = &self.model else {
            return;
        };
        let doc = &m.doc;

        match self.pick_mode {
            ViewportPickMode::InspectEntity => {
                self.highlight.selected = Some(target);
                self.inspector_focus = InspectorFocus::CadEntity;
            }
            ViewportPickMode::SetupWcsZ { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let axis_opt = match target {
                        PickTarget::Face(fid) => doc.bind_face(fid).map(WcsAxisRef::FaceAxis),
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.bind_edge(eid).map(WcsAxisRef::EdgeDirection)
                        }
                    };
                    if let Some(z_axis_ref) = axis_opt {
                        self.history.record_before_edit(
                            "Pick Setup WCS Z Axis",
                            None,
                            &self.project,
                        );
                        if let WcsOrientationMode::SelectZAndX { z_ref, .. } =
                            &mut self.project.setups[setup_idx].wcs.orientation
                        {
                            *z_ref = z_axis_ref;
                        }
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::SetupWcsX { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let axis_opt = match target {
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.bind_edge(eid).map(WcsAxisRef::EdgeDirection)
                        }
                        PickTarget::Face(fid) => doc.bind_face(fid).map(WcsAxisRef::FaceAxis),
                    };
                    if let Some(x_axis_ref) = axis_opt {
                        self.history.record_before_edit(
                            "Pick Setup WCS X Axis",
                            None,
                            &self.project,
                        );
                        if let WcsOrientationMode::SelectZAndX { x_ref, .. } =
                            &mut self.project.setups[setup_idx].wcs.orientation
                        {
                            *x_ref = Some(x_axis_ref);
                        }
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::SetupWcsXyPlane { setup_idx } => {
                if let PickTarget::Face(fid) = target
                    && setup_idx < self.project.setups.len()
                    && let Some(bind) = doc.bind_face(fid)
                {
                    self.history
                        .record_before_edit("Pick Setup WCS XY Plane", None, &self.project);
                    if let WcsOrientationMode::SelectXYPlane { face_ref, .. } =
                        &mut self.project.setups[setup_idx].wcs.orientation
                    {
                        *face_ref = bind;
                    }
                    self.pick_mode = ViewportPickMode::InspectEntity;
                }
            }
            ViewportPickMode::SetupWcsXyPlaneXEdge { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let edge_opt = match target {
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.bind_edge(eid)
                        }
                        PickTarget::Face(_) => None,
                    };
                    if let Some(bind) = edge_opt {
                        self.history.record_before_edit(
                            "Pick Setup WCS XY Plane +X Edge",
                            None,
                            &self.project,
                        );
                        if let WcsOrientationMode::SelectXYPlane { x_edge_ref, .. } =
                            &mut self.project.setups[setup_idx].wcs.orientation
                        {
                            *x_edge_ref = Some(bind);
                        }
                        for op in &mut self.project.setups[setup_idx].operations {
                            op.mark_dirty();
                        }
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::SetupWcsOriginFeature { setup_idx } => {
                if setup_idx < self.project.setups.len() {
                    let feat_opt = match target {
                        PickTarget::Face(fid) => {
                            if let Some(f) = doc.face(fid)
                                && matches!(
                                    f.surface,
                                    SurfaceKind::Cylinder {
                                        is_internal: true,
                                        ..
                                    }
                                )
                            {
                                doc.bind_face(fid)
                                    .map(|b| WcsFeaturePointRef::HoleAxisCenter {
                                        hole_face: b,
                                        top_rim: true,
                                    })
                            } else {
                                None
                            }
                        }
                        PickTarget::Edge(eid) | PickTarget::Loop { seed_edge: eid, .. } => {
                            doc.edge(eid).and_then(|e| {
                                let b = doc.bind_edge(eid)?;
                                if matches!(e.curve, CurveKind::CircleArc { .. }) {
                                    Some(WcsFeaturePointRef::CircleCenter(b))
                                } else {
                                    Some(WcsFeaturePointRef::EdgeMidpoint(b))
                                }
                            })
                        }
                    };
                    if let Some(feature) = feat_opt {
                        self.history.record_before_edit(
                            "Pick WCS Origin Feature",
                            None,
                            &self.project,
                        );
                        self.project.setups[setup_idx].wcs.origin =
                            WcsOriginMode::SelectedCadFeature { feature };
                        self.pick_mode = ViewportPickMode::InspectEntity;
                    }
                }
            }
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx } => {
                if let PickTarget::Face(fid) = target
                    && let Some(bind) = doc.bind_face(fid)
                {
                    let fid_idx = fid.0;
                    self.apply_geometry_pick_toggle(
                        setup_idx,
                        op_idx,
                        "Toggle Planar Face Selection",
                        GeometrySelectionItem::PlanarFace { binding: bind },
                        Some(move |s: &GeometrySelectionItem| {
                            matches!(s, GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid_idx)
                        }),
                    );
                }
            }
            ViewportPickMode::OpClosedLoops { setup_idx, op_idx } => {
                let lid_opt = match target {
                    PickTarget::Loop { loop_id, .. } => Some(loop_id),
                    PickTarget::Edge(eid) => doc.edge(eid).and_then(|e| e.loops.first().copied()),
                    _ => None,
                };
                if let Some(lid) = lid_opt
                    && let Some(bind) = doc.bind_loop(lid)
                {
                    let lid_idx = lid.0;
                    self.apply_geometry_pick_toggle(
                        setup_idx,
                        op_idx,
                        "Toggle Closed Loop Selection",
                        GeometrySelectionItem::ClosedLoop {
                            binding: bind,
                            reverse: false,
                            side_flip: false,
                        },
                        Some(move |s: &GeometrySelectionItem| {
                            matches!(s, GeometrySelectionItem::ClosedLoop { binding, .. } if binding.last_index == lid_idx)
                        }),
                    );
                }
            }
            ViewportPickMode::OpOpenChain {
                setup_idx,
                op_idx,
                start_edge,
            } => {
                let eid_opt = match target {
                    PickTarget::Edge(eid) => Some(eid),
                    PickTarget::Loop { seed_edge, .. } => Some(seed_edge),
                    _ => None,
                };
                if let Some(eid) = eid_opt {
                    match start_edge {
                        None => {
                            self.pick_mode = ViewportPickMode::OpOpenChain {
                                setup_idx,
                                op_idx,
                                start_edge: Some(eid),
                            };
                        }
                        Some(se) => {
                            if let Some(chain_uses) = m.topo.find_open_edge_chain(doc, se, eid)
                                && let (Some(sb), Some(eb), Some(s_rec)) =
                                    (doc.bind_edge(se), doc.bind_edge(eid), doc.edge(se))
                            {
                                let body_id = s_rec.body_id;
                                self.apply_geometry_pick_toggle(
                                    setup_idx,
                                    op_idx,
                                    "Add Open Edge Chain",
                                    GeometrySelectionItem::OpenChain {
                                        body_id,
                                        start_edge: sb,
                                        end_edge: eb,
                                        resolved_edges: chain_uses,
                                        reverse: false,
                                        side_flip: false,
                                    },
                                    Option::<fn(&GeometrySelectionItem) -> bool>::None,
                                );
                            } else {
                                self.logs.push(format!(
                                    "Could not find connected open edge path from Edge #{} to Edge #{}.",
                                    se.0, eid.0
                                ));
                            }
                            self.pick_mode = ViewportPickMode::OpOpenChain {
                                setup_idx,
                                op_idx,
                                start_edge: None,
                            };
                        }
                    }
                }
            }
            ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx } => {
                if let PickTarget::Face(fid) = target
                    && let Some(bind) = doc.bind_face(fid)
                {
                    let fid_idx = fid.0;
                    self.apply_geometry_pick_toggle(
                        setup_idx,
                        op_idx,
                        "Toggle Cylindrical Hole",
                        GeometrySelectionItem::CylindricalHole { binding: bind },
                        Some(move |s: &GeometrySelectionItem| {
                            matches!(s, GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid_idx)
                        }),
                    );
                }
            }
            ViewportPickMode::PlaceContourTab { .. } | ViewportPickMode::SketchAuthor { .. } => {}
            ViewportPickMode::PickSketchHostFace { setup_idx } => {
                if let PickTarget::Face(fid) = target
                    && let Some(face) = doc.face(fid)
                    && matches!(face.surface, SurfaceKind::Plane { .. })
                    && let Some(bind) = doc.bind_face(fid)
                {
                    self.history.record_before_edit(
                        "Create Sketch on 3D Face",
                        None,
                        &self.project,
                    );
                    let setup_id = self
                        .project
                        .setups
                        .get(setup_idx)
                        .map_or_else(|| "setup_1".to_string(), |s| s.id.clone());
                    let n = self.project.sketches.len() + 1;
                    let mut sk = SketchLayer::new(
                        format!("sketch_face_{n}"),
                        format!("Sketch {n} (Face #{})", fid.0),
                        setup_id.clone(),
                    );
                    sk.anchor = SketchAnchor::PlanarFace {
                        setup_id,
                        face_binding: cad_binding_to_sketch_face(&bind),
                        origin_mode: FaceSketchOriginMode::FaceCentroid,
                        x_axis_clock_deg: 0.0,
                        z_offset: ParamLength::from_mm(0.0),
                    };
                    self.project.sketches.push(sk);
                    let sk_idx = self.project.sketches.len() - 1;
                    self.inspector_focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
                    self.selected_sketch_entity_idx = None;
                    self.pick_mode = ViewportPickMode::InspectEntity;
                    self.cached_frame_geo = None;
                    self.align_camera_to_sketch(sk_idx);
                    self.logs.push(format!(
                        "Attached associative 2D Sketch '{}' to planar B-Rep Face #{}.",
                        self.project.sketches[sk_idx].name, fid.0
                    ));
                } else {
                    self.logs.push(
                        "Sketch on 3D Face requires clicking a planar B-Rep face.".to_string(),
                    );
                }
            }
            ViewportPickMode::RebindSketchHostFace { sketch_idx } => {
                if let PickTarget::Face(fid) = target
                    && sketch_idx < self.project.sketches.len()
                    && let Some(face) = doc.face(fid)
                    && matches!(face.surface, SurfaceKind::Plane { .. })
                    && let Some(bind) = doc.bind_face(fid)
                {
                    self.history.record_before_edit(
                        "Attach Sketch to 3D Face",
                        None,
                        &self.project,
                    );
                    let setup_id = self.project.sketches[sketch_idx]
                        .anchor
                        .setup_id()
                        .to_string();
                    self.project.sketches[sketch_idx].anchor = SketchAnchor::PlanarFace {
                        setup_id,
                        face_binding: cad_binding_to_sketch_face(&bind),
                        origin_mode: FaceSketchOriginMode::FaceCentroid,
                        x_axis_clock_deg: 0.0,
                        z_offset: ParamLength::from_mm(0.0),
                    };
                    self.pick_mode = ViewportPickMode::InspectEntity;
                    self.cached_frame_geo = None;
                    self.align_camera_to_sketch(sketch_idx);
                }
            }
            ViewportPickMode::PickKeepOutGeometry { region_idx } => match target {
                PickTarget::Face(fid) => {
                    let is_cyl_hole = doc.face(fid).is_some_and(|f| {
                        matches!(
                            f.surface,
                            SurfaceKind::Cylinder {
                                is_internal: true,
                                ..
                            }
                        )
                    });
                    if let Some(bind) = doc.bind_face(fid) {
                        let fid_idx = fid.0;
                        if is_cyl_hole {
                            self.apply_keepout_region_pick_toggle(
                                region_idx,
                                "Toggle Cylindrical Hole",
                                GeometrySelectionItem::CylindricalHole { binding: bind },
                                move |s| {
                                    matches!(s, GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid_idx)
                                },
                            );
                        } else {
                            self.apply_keepout_region_pick_toggle(
                                region_idx,
                                "Toggle Planar Face",
                                GeometrySelectionItem::PlanarFace { binding: bind },
                                move |s| {
                                    matches!(s, GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid_idx)
                                },
                            );
                        }
                    }
                }
                PickTarget::Loop { loop_id, .. } => {
                    if let Some(bind) = doc.bind_loop(loop_id) {
                        let lid_idx = loop_id.0;
                        self.apply_keepout_region_pick_toggle(
                            region_idx,
                            "Toggle Closed Loop",
                            GeometrySelectionItem::ClosedLoop {
                                binding: bind,
                                reverse: false,
                                side_flip: false,
                            },
                            move |s| {
                                matches!(s, GeometrySelectionItem::ClosedLoop { binding, .. } if binding.last_index == lid_idx)
                            },
                        );
                    }
                }
                PickTarget::Edge(eid) => {
                    if let Some(lid) = doc.edge(eid).and_then(|e| e.loops.first().copied())
                        && let Some(bind) = doc.bind_loop(lid)
                    {
                        let lid_idx = lid.0;
                        self.apply_keepout_region_pick_toggle(
                            region_idx,
                            "Toggle Closed Loop",
                            GeometrySelectionItem::ClosedLoop {
                                binding: bind,
                                reverse: false,
                                side_flip: false,
                            },
                            move |s| {
                                matches!(s, GeometrySelectionItem::ClosedLoop { binding, .. } if binding.last_index == lid_idx)
                            },
                        );
                    }
                }
            },
            ViewportPickMode::PickModelLayFlatFace { .. }
            | ViewportPickMode::PlaceFixtureClamp { .. } => {}
        }
    }

    pub(crate) fn apply_lay_flat_face_normal(&mut self, model_idx: usize, normal: DVec3) {
        if model_idx >= self.project.models.len() {
            return;
        }
        let Some(m) = &self.model else {
            return;
        };
        let pivot = if m.doc.aabb_mm.is_valid() {
            m.doc.aabb_mm.center()
        } else {
            DVec3::ZERO
        };
        let old_xf = self.project.models[model_idx].transform;
        let laid_xf = old_xf.laid_flat_normal_to_neg_z(normal, pivot);
        let laid_doc = m.raw_doc.transformed(&laid_xf);
        let final_xf = if laid_doc.aabb_mm.is_valid() {
            laid_xf.dropped_bottom_to_z0_for_aabb(laid_doc.aabb_mm)
        } else {
            laid_xf
        };
        if final_xf.validate().is_err() {
            return;
        }
        self.history
            .record_before_edit("Lay Model Flat on Face", None, &self.project);
        self.project.models[model_idx].transform = final_xf;
        for setup in &mut self.project.setups {
            for op in &mut setup.operations {
                op.mark_dirty();
            }
        }
        self.pick_mode = ViewportPickMode::InspectEntity;
        self.sync_loaded_model_transform();
        self.fit_scene_to_viewport();
        self.logs.push(format!(
            "Laid model flat on clicked face (normal {:+.3}, {:+.3}, {:+.3}) and dropped bottom to Z = 0.",
            normal.x, normal.y, normal.z
        ));
    }

    pub(crate) fn handle_sketch_author_viewport_interaction(
        &mut self,
        input: SketchAuthorInteractionInput,
    ) {
        let SketchAuthorInteractionInput {
            sketch_idx,
            tool,
            ray,
            mm_per_px,
            primary_click,
            right_click,
        } = input;

        if right_click {
            if self.sketch_author_start_xy.take().is_none() {
                self.pick_mode = ViewportPickMode::InspectEntity;
            }
            return;
        }
        if sketch_idx >= self.project.sketches.len() {
            self.pick_mode = ViewportPickMode::InspectEntity;
            return;
        }
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };
        let wf = if let Some(m) = &self.model {
            resolve_sketch_world_frame_with_doc(&m.doc, &self.project.sketches[sketch_idx], &rs)
        } else {
            resolve_sketch_world_frame(false, &self.project.sketches[sketch_idx], &rs)
        };

        let denom = ray.dir.dot(wf.z_axis);
        if denom.abs() <= 1e-6 {
            return;
        }
        let t = (wf.origin_mm - ray.origin).dot(wf.z_axis) / denom;
        if t <= 0.0 {
            return;
        }
        let hit_world = ray.origin + ray.dir * t;
        let hit_local = wf.world_to_local_point(hit_world);
        let raw_xy = DVec2::new(hit_local.x, hit_local.y);

        // Project stock corners & 3D STEP vertices/circle centers into Sketch XY for 2D Snapping!
        let st_wcs = rs.stock.aabb_wcs();
        let stock_xy = if st_wcs.is_valid() {
            let mut b = Aabb2::EMPTY;
            for p_wcs in [
                DVec3::new(st_wcs.min.x, st_wcs.min.y, st_wcs.max.z),
                DVec3::new(st_wcs.max.x, st_wcs.min.y, st_wcs.max.z),
                DVec3::new(st_wcs.max.x, st_wcs.max.y, st_wcs.max.z),
                DVec3::new(st_wcs.min.x, st_wcs.max.y, st_wcs.max.z),
            ] {
                let pw = rs.frame.local_to_world_point(p_wcs);
                let pl = wf.world_to_local_point(pw);
                b.extend_point(DVec2::new(pl.x, pl.y));
            }
            b.is_valid().then_some(b)
        } else {
            None
        };

        let mut cad_pts_2d: Vec<DVec2> = Vec::new();
        if let Some(m) = &self.model {
            for edge in &m.doc.edges {
                match edge.curve {
                    CurveKind::Line { start_mm, end_mm } => {
                        for pw in [start_mm, end_mm] {
                            let pl = wf.world_to_local_point(pw);
                            if pl.z.abs() <= 25.0 {
                                cad_pts_2d.push(DVec2::new(pl.x, pl.y));
                            }
                        }
                    }
                    CurveKind::CircleArc {
                        center_mm,
                        start_mm,
                        end_mm,
                        ..
                    } => {
                        for pw in [center_mm, start_mm, end_mm] {
                            let pl = wf.world_to_local_point(pw);
                            if pl.z.abs() <= 25.0 {
                                cad_pts_2d.push(DVec2::new(pl.x, pl.y));
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        let snap_res = snap_sketch_point_2d(
            raw_xy,
            &SketchSnapContext {
                sketch: &self.project.sketches[sketch_idx],
                stock_xy_bounds: stock_xy,
                projected_cad_points: &cad_pts_2d,
                snap_radius_mm: (12.0 * mm_per_px).clamp(0.5, 10.0),
                grid_step_mm: self.sketch_snap_grid_mm,
            },
        );
        let pt_xy = snap_res.point_xy;
        let snap_world = wf.local_to_world_point(DVec3::new(pt_xy.x, pt_xy.y, 0.06));
        let badge = snap_res.kind.map_or("XY", |k| k.badge());
        self.highlight.sketch_snap_marker = Some((snap_world, badge, pt_xy));

        // Build live rubber-band preview when first point is active
        if let Some(p0) = self.sketch_author_start_xy {
            let preview_polys = build_rubber_band_polys(tool, p0, pt_xy);
            for poly in preview_polys {
                if !poly.segments.is_empty() {
                    self.highlight.sketch_preview_polys.push((wf, poly));
                }
            }
        }

        if !primary_click {
            return;
        }

        if tool == SketchAuthorTool::PlaceText {
            self.history
                .record_before_edit("Place CAM Text", None, &self.project);
            let sk = &mut self.project.sketches[sketch_idx];
            let n = sk.entities.len() + 1;
            let spec = SketchTextSpec {
                x: ParamLength::from_mm(pt_xy.x),
                y: ParamLength::from_mm(pt_xy.y),
                ..SketchTextSpec::default()
            };
            sk.entities.push(SketchEntityEntry {
                id: format!("text_{n}"),
                name: format!("Text {n}"),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::Text(spec),
            });
            sk.rebuild_regions(None);
            self.selected_sketch_entity_idx = Some(sk.entities.len() - 1);
            self.cached_frame_geo = None;
            return;
        }

        let Some(p0) = self.sketch_author_start_xy else {
            self.sketch_author_start_xy = Some(pt_xy);
            return;
        };

        let delta = pt_xy - p0;
        let dist = delta.length();
        if dist <= 0.25 {
            return;
        }
        self.sketch_author_start_xy = None;
        self.history.record_before_edit(
            format!("Draw Sketch {}", tool.short_label()),
            None,
            &self.project,
        );
        let sk = &mut self.project.sketches[sketch_idx];
        let n = sk.entities.len() + 1;
        let new_entry = match tool {
            SketchAuthorTool::Rectangle => {
                let c = 0.5 * (p0 + pt_xy);
                let w = delta.x.abs().max(1.0);
                let h = delta.y.abs().max(1.0);
                SketchEntityEntry {
                    id: format!("rect_{n}"),
                    name: format!("Rectangle {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Rectangle {
                        anchor: RectAnchorMode::Center,
                        x: ParamLength::from_mm(c.x),
                        y: ParamLength::from_mm(c.y),
                        width: ParamLength::from_mm(w),
                        height: ParamLength::from_mm(h),
                        corner_radius: ParamLength::from_mm(0.0),
                        rotation_deg: 0.0,
                    },
                }
            }
            SketchAuthorTool::Circle => SketchEntityEntry {
                id: format!("circle_{n}"),
                name: format!("Circle {n}"),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::new_circle(
                    ParamLength::from_mm(p0.x),
                    ParamLength::from_mm(p0.y),
                    ParamLength::from_mm(2.0 * dist),
                ),
            },
            SketchAuthorTool::Slot => {
                let c = 0.5 * (p0 + pt_xy);
                let ang = delta.y.atan2(delta.x).to_degrees();
                SketchEntityEntry {
                    id: format!("slot_{n}"),
                    name: format!("Slot {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Slot {
                        mode: SlotLengthMode::CenterToCenter,
                        x: ParamLength::from_mm(c.x),
                        y: ParamLength::from_mm(c.y),
                        length: ParamLength::from_mm(dist),
                        width: ParamLength::from_mm(8.0),
                        angle_deg: ang,
                    },
                }
            }
            SketchAuthorTool::Polygon => {
                let ang = delta.y.atan2(delta.x).to_degrees();
                SketchEntityEntry {
                    id: format!("poly_{n}"),
                    name: format!("Hexagon {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::RegularPolygon {
                        center_x: ParamLength::from_mm(p0.x),
                        center_y: ParamLength::from_mm(p0.y),
                        sides: 6,
                        sizing: PolygonSizingMode::CircumscribedDiameter,
                        diameter: ParamLength::from_mm(2.0 * dist),
                        corner_radius: ParamLength::from_mm(0.0),
                        rotation_deg: ang,
                    },
                }
            }
            SketchAuthorTool::BoltCircle | SketchAuthorTool::PlaceText => {
                let ang = delta.y.atan2(delta.x).to_degrees();
                SketchEntityEntry {
                    id: format!("bolts_{n}"),
                    name: format!("Bolt Circle {n}"),
                    sub_layer: Some("HOLES".to_string()),
                    is_construction: false,
                    entity: SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
                        center_x: ParamLength::from_mm(p0.x),
                        center_y: ParamLength::from_mm(p0.y),
                        bolt_circle_diameter: ParamLength::from_mm(2.0 * dist),
                        hole_diameter: ParamLength::from_mm(6.0),
                        count: 6,
                        start_angle_deg: ang,
                    }),
                }
            }
        };
        sk.entities.push(new_entry);
        sk.rebuild_regions(None);
        self.selected_sketch_entity_idx = Some(sk.entities.len() - 1);
        self.cached_frame_geo = None;
    }

    pub(crate) fn handle_manual_tab_viewport_interaction(
        &mut self,
        setup_idx: usize,
        op_idx: usize,
        ray: &Ray3,
        mm_per_px: f64,
        primary_click: bool,
        right_click: bool,
    ) {
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };
        let Some((cad_doc, _)) = self.effective_cad_arc() else {
            return;
        };
        let Some(setup) = self.project.setups.get(setup_idx) else {
            return;
        };
        let Some(op) = setup.operations.get(op_idx) else {
            return;
        };
        let Ok(sum) = extract_selection_features_with_sketches(
            &cad_doc,
            &self.project.sketches,
            &rs,
            &op.selections,
        ) else {
            return;
        };

        let ray_orig_wcs = rs.frame.world_to_local_point(ray.origin);
        let ray_dir_wcs = rs.frame.world_to_local_dir(ray.dir);
        if ray_dir_wcs.z.abs() <= 1e-6 {
            return;
        }

        let mut closed_contours = Vec::new();
        for feat in &sum.features {
            match feat {
                ExtractedFeature25D::Contour {
                    polyline_wcs,
                    min_z_wcs,
                    ..
                } if polyline_wcs.closed => {
                    closed_contours.push((polyline_wcs.clone(), *min_z_wcs));
                }
                ExtractedFeature25D::PlanarFace {
                    outer_boundary_wcs,
                    face_z_wcs,
                    ..
                } if outer_boundary_wcs.closed => {
                    closed_contours
                        .push((outer_boundary_wcs.ensure_orientation(false), *face_z_wcs));
                }
                _ => {}
            }
        }

        let snap_radius_mm = (14.0 * mm_per_px).clamp(1.0, 12.0);
        let mut best_snap: Option<(usize, f64, DVec2, DVec2, f64, f64)> = None;
        for (c_idx, (poly, z_plane)) in closed_contours.iter().enumerate() {
            let t = (z_plane - ray_orig_wcs.z) / ray_dir_wcs.z;
            if t <= 0.0 {
                continue;
            }
            let hit_wcs = ray_orig_wcs + ray_dir_wcs * t;
            let hit_xy = DVec2::new(hit_wcs.x, hit_wcs.y);
            if let Some(proj) = poly.project_point(hit_xy)
                && proj.distance_mm <= snap_radius_mm
                && best_snap.is_none_or(|(_, _, _, _, _, prev_d)| proj.distance_mm < prev_d)
            {
                best_snap = Some((
                    c_idx,
                    proj.s_norm,
                    proj.point,
                    proj.tangent,
                    *z_plane,
                    proj.distance_mm,
                ));
            }
        }

        let Some((contour_index, s_norm, pt_xy, tan_xy, z_wcs, _)) = best_snap else {
            return;
        };

        let (width_mm, height_mm, is_tri) = match op.resolved_strategy() {
            StrategyParams::Contour2D(p) => (
                p.tabs.width.value_mm,
                p.tabs.height.value_mm,
                matches!(p.tabs.profile, TabProfileKind::Triangular { .. }),
            ),
            _ => return,
        };

        self.highlight.preview_tab_marker_wcs = Some(TabMarker3D {
            center_wcs: DVec3::new(pt_xy.x, pt_xy.y, z_wcs),
            tangent_xy: tan_xy,
            width_mm,
            height_mm,
            is_triangular: is_tri,
            contour_index,
            s_norm,
        });

        if primary_click {
            let world_pt = rs
                .frame
                .local_to_world_point(DVec3::new(pt_xy.x, pt_xy.y, z_wcs));
            self.history
                .record_before_edit("Add Manual Holding Tab", None, &self.project);
            let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
            if let StrategyParams::Contour2D(p) = op_mut.strategy_mut() {
                p.tabs.enabled = true;
                p.tabs.placement = TabPlacementMode::Manual;
                p.tabs.manual_anchors.push(ManualTabAnchor {
                    contour_index,
                    s_norm,
                    world_point_mm: world_pt,
                });
            }
            op_mut.mark_dirty();
        } else if right_click {
            let pre_op = self.project.setups[setup_idx].operations[op_idx].clone();
            let mut removed = false;
            {
                let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
                if let StrategyParams::Contour2D(p) = op_mut.strategy_mut()
                    && let Some(pos) = p.tabs.manual_anchors.iter().position(|a| {
                        a.contour_index == contour_index && (a.s_norm - s_norm).abs() <= 0.06
                    })
                {
                    p.tabs.manual_anchors.remove(pos);
                    removed = true;
                }
            }
            if removed {
                let mut pre_doc = self.project.clone();
                pre_doc.setups[setup_idx].operations[op_idx] = pre_op;
                self.history
                    .record_before_edit("Remove Manual Holding Tab", None, &pre_doc);
                self.project.setups[setup_idx].operations[op_idx].mark_dirty();
            }
        }
    }

    /// Interactive 3D click-to-place handler for [`ViewportPickMode::PlaceFixtureClamp`].
    ///
    /// Intersects the viewport cursor ray with the active Setup's Stock Top plane, auto-orients
    /// `ToeClamp` outward away from stock center (snapping to nearest stock edge normal or 45°
    /// corner diagonal), and drops a `ToeClamp` or `CylinderKeepOut` on primary click.
    pub(crate) fn handle_fixture_place_viewport_interaction(
        &mut self,
        setup_idx: usize,
        kind: FixturePlaceKind,
        ray: &Ray3,
        primary_click: bool,
        right_click: bool,
    ) {
        if right_click {
            self.pick_mode = ViewportPickMode::InspectEntity;
            return;
        }
        if setup_idx >= self.project.setups.len() {
            self.pick_mode = ViewportPickMode::InspectEntity;
            return;
        }
        let Some(Ok(rs)) = self.active_resolved_setup() else {
            return;
        };

        let stock_aabb = rs.stock.aabb_wcs();
        let z_plane = if stock_aabb.is_valid() {
            stock_aabb.max.z
        } else {
            0.0
        };
        let ray_orig_wcs = rs.frame.world_to_local_point(ray.origin);
        let ray_dir_wcs = rs.frame.world_to_local_dir(ray.dir);
        if ray_dir_wcs.z.abs() <= 1e-6 {
            return;
        }
        let t = (z_plane - ray_orig_wcs.z) / ray_dir_wcs.z;
        if t <= 0.0 {
            return;
        }

        let hit_wcs = ray_orig_wcs + ray_dir_wcs * t;
        let click_xy = DVec2::new(hit_wcs.x, hit_wcs.y);
        let world_pt =
            rs.frame
                .local_to_world_point(DVec3::new(click_xy.x, click_xy.y, z_plane + 0.2));
        let badge = match kind {
            FixturePlaceKind::ToeClamp => "CLAMP",
            FixturePlaceKind::CylinderKeepOut => "PIN",
        };
        self.highlight.sketch_snap_marker = Some((world_pt, badge, click_xy));

        if !primary_click {
            return;
        }

        // Compute outward approach yaw away from stock XY center
        let yaw_deg = if stock_aabb.is_valid() {
            let c = DVec2::new(stock_aabb.center().x, stock_aabb.center().y);
            let half = DVec2::new(
                (0.5 * stock_aabb.size().x).max(1.0),
                (0.5 * stock_aabb.size().y).max(1.0),
            );
            let rel = click_xy - c;
            let nx = rel.x / half.x;
            let ny = rel.y / half.y;
            if nx.abs() >= 0.75 && ny.abs() >= 0.75 {
                // Corner diagonal (45°, 135°, 225°, 315°)
                let raw = ny.atan2(nx).to_degrees();
                (raw / 45.0).round() * 45.0
            } else if nx.abs() >= ny.abs() {
                if nx >= 0.0 { 0.0 } else { 180.0 }
            } else if ny >= 0.0 {
                90.0
            } else {
                270.0
            }
        } else {
            0.0
        };

        self.history.record_before_edit(
            format!("Place {} in Viewport", kind.label()),
            None,
            &self.project,
        );
        let setup = &mut self.project.setups[setup_idx];
        let mut num = setup.fixtures.len() + 1;
        let prefix = match kind {
            FixturePlaceKind::ToeClamp => "clamp",
            FixturePlaceKind::CylinderKeepOut => "pin",
        };
        let fix_id = loop {
            let cand = format!("{prefix}_{num}");
            if !setup.fixtures.iter().any(|f| f.id == cand) {
                break cand;
            }
            num += 1;
        };

        let fixture = match kind {
            FixturePlaceKind::ToeClamp => {
                let n = setup
                    .fixtures
                    .iter()
                    .filter(|f| matches!(f.kind, FixtureKind::ToeClamp { .. }))
                    .count()
                    + 1;
                FixtureBody::default_toe_clamp(fix_id, format!("Toe Clamp {n}"), click_xy, yaw_deg)
            }
            FixturePlaceKind::CylinderKeepOut => {
                let n = setup
                    .fixtures
                    .iter()
                    .filter(|f| matches!(f.kind, FixtureKind::CylinderKeepOut { .. }))
                    .count()
                    + 1;
                let z_bot = if stock_aabb.is_valid() {
                    stock_aabb.min.z
                } else {
                    -15.0
                };
                FixtureBody::default_cylinder_keepout(
                    fix_id,
                    format!("Keep-Out Pin {n}"),
                    click_xy,
                    z_bot,
                    z_plane + 12.0,
                )
            }
        };

        setup.fixtures.push(fixture);
        for op in &mut setup.operations {
            op.mark_dirty();
        }
        self.selected_fixture_idx = Some(setup.fixtures.len() - 1);
        self.highlight.show_fixtures = true;
        self.cached_frame_geo = None;
    }
}

fn build_rubber_band_polys(
    tool: SketchAuthorTool,
    p0: DVec2,
    p1: DVec2,
) -> Vec<chip_math::ArcPolyline2D> {
    let delta = p1 - p0;
    let dist = delta.length();
    if dist <= 0.1 {
        return Vec::new();
    }
    match tool {
        SketchAuthorTool::Rectangle => {
            let c = 0.5 * (p0 + p1);
            vec![build_rectangle_polyline(RectEvalSpec {
                anchor: RectAnchorMode::Center,
                anchor_xy: c,
                width_mm: delta.x.abs(),
                height_mm: delta.y.abs(),
                corner_radius_mm: 0.0,
                rotation_deg: 0.0,
            })]
        }
        SketchAuthorTool::Circle => vec![build_circle_polyline(p0, 2.0 * dist)],
        SketchAuthorTool::Slot => {
            let c = 0.5 * (p0 + p1);
            let ang = delta.y.atan2(delta.x).to_degrees();
            vec![build_slot_polyline(
                SlotLengthMode::CenterToCenter,
                c,
                dist,
                8.0,
                ang,
            )]
        }
        SketchAuthorTool::Polygon => {
            let ang = delta.y.atan2(delta.x).to_degrees();
            vec![build_regular_polygon_polyline(PolygonEvalSpec {
                center_xy: p0,
                sides: 6,
                sizing: PolygonSizingMode::CircumscribedDiameter,
                diameter_mm: 2.0 * dist,
                corner_radius_mm: 0.0,
                rotation_deg: ang,
            })]
        }
        SketchAuthorTool::BoltCircle => {
            let mut out = vec![build_circle_polyline(p0, 2.0 * dist)];
            let pat = PointPatternSpec::BoltCircle {
                center_x: ParamLength::from_mm(p0.x),
                center_y: ParamLength::from_mm(p0.y),
                bolt_circle_diameter: ParamLength::from_mm(2.0 * dist),
                hole_diameter: ParamLength::from_mm(6.0),
                count: 6,
                start_angle_deg: delta.y.atan2(delta.x).to_degrees(),
            };
            for h in pat.evaluate_holes() {
                out.push(build_circle_polyline(h.center_xy, h.diameter_mm));
            }
            out
        }
        SketchAuthorTool::PlaceText => Vec::new(),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        OperationKind, ResolvedRotaryConfig, ResolvedStockGeometry, RotaryAxisKind,
        RotaryDirection, RotaryDriveMode, RotaryUnwindMode,
    };
    use chip_math::{Aabb3, DVec3, Ray3, RigidFrame3};

    use super::*;
    use crate::viewport::{HighlightState, gizmos::emit_stock_visuals};

    fn sample_x_rotary() -> ResolvedRotaryConfig {
        ResolvedRotaryConfig {
            mode: RotaryDriveMode::Indexed3Plus1,
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::RightHandCcw,
            axis_letter: 'A',
            center_wcs: DVec3::new(0.0, 2.0, -3.0),
            stock_u_min: -45.0,
            stock_u_max: 45.0,
            stock_outer_radius_mm: 30.0,
            clearance_radius_mm: 40.0,
            chuck_end_positive: false,
            chuck_u_min: -65.0,
            chuck_u_max: -45.0,
            chuck_radius_mm: 35.0,
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        }
    }

    #[test]
    fn m84_indexed_auto_rotate_precedence_stock_wireframe_and_picking_ray_alignment() {
        let rot = sample_x_rotary();
        let mut op_idx90 = Operation::new("op90", "Index 90", OperationKind::Adaptive3D);
        op_idx90.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 90.0 };

        // 1. Auto-rotate enabled + no active sim -> rotates to 90° with HUD badge
        let (deg, badge) =
            resolve_effective_viewport_rotary(false, 0.0, true, Some(&op_idx90), Some(rot));
        assert!((deg - 90.0).abs() < 1e-9);
        assert_eq!(badge, Some(('A', 90.0)));

        // 2. Auto-rotate disabled -> stays at 0° with no badge
        let (deg_off, badge_off) =
            resolve_effective_viewport_rotary(false, 0.0, false, Some(&op_idx90), Some(rot));
        assert!((deg_off - 0.0).abs() < 1e-9);
        assert_eq!(badge_off, None);

        // 3. Simulation playback active -> strict precedence over auto-rotate!
        let (deg_sim, badge_sim) =
            resolve_effective_viewport_rotary(true, 137.5, true, Some(&op_idx90), Some(rot));
        assert!((deg_sim - 137.5).abs() < 1e-9);
        assert_eq!(badge_sim, None);

        // 4. `emit_stock_visuals` rotates asymmetric raw Box stock in lockstep with `sim_rotary_deg = 90°`
        let stock = ResolvedStockGeometry::Box {
            aabb_wcs: Aabb3::new(
                DVec3::new(-45.0, -10.0, -25.0),
                DVec3::new(45.0, 10.0, 25.0),
            ),
        };
        let mut rot_origin = rot;
        rot_origin.center_wcs = DVec3::ZERO;
        let hl90 = HighlightState {
            wcs_frame: RigidFrame3::IDENTITY,
            rotary: Some(rot_origin),
            sim_rotary_deg: 90.0,
            ..HighlightState::default()
        };
        let mut tris = Vec::new();
        let mut edges = Vec::new();
        emit_stock_visuals(
            &stock,
            &hl90.wcs_frame,
            hl90.stock_model_wcs_transform(),
            false,
            &mut tris,
            &mut edges,
        );
        assert_eq!(edges.len(), 12);
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;
        for e in &edges {
            for p in [e.p0_mm, e.p1_mm] {
                min_y = min_y.min(p[1]);
                max_y = max_y.max(p[1]);
                min_z = min_z.min(p[2]);
                max_z = max_z.max(p[2]);
            }
        }
        assert!(
            (max_y - min_y - 50.0).abs() < 1e-3 && (max_z - min_z - 20.0).abs() < 1e-3,
            "Box stock rotated by 90° around X must swap Y (50 mm) and Z (20 mm) extents; got Y=[{min_y}, {max_y}], Z=[{min_z}, {max_z}]"
        );

        // 5. `world_ray_to_cad_model_space` keeps picking hits 100% aligned at 90°, 180°, 270°, and 45°
        let wcs_frame = RigidFrame3 {
            origin_mm: DVec3::new(12.0, -5.0, 7.0),
            x_axis: DVec3::X,
            y_axis: DVec3::Y,
            z_axis: DVec3::Z,
        };
        let sample_cad_pt = DVec3::new(18.0, 14.0, -9.0);
        for angle_deg in [45.0_f64, 90.0, 180.0, 270.0] {
            let hl = HighlightState {
                wcs_frame,
                rotary: Some(rot),
                sim_rotary_deg: angle_deg,
                ..HighlightState::default()
            };
            let displayed_world = hl.cad_world_transform().transform_point3(sample_cad_pt);
            // Shoot a ray directly from +Z above the displayed rotated point downward through it
            let ray_world =
                Ray3::new(displayed_world + DVec3::new(0.0, 0.0, 80.0), DVec3::NEG_Z).unwrap();
            let ray_cad = hl.world_ray_to_cad_model_space(&ray_world);
            let hit_on_ray = ray_cad.origin + ray_cad.dir * 80.0;
            assert!(
                hit_on_ray.distance(sample_cad_pt) < 1e-8,
                "picking ray at {angle_deg}° must map displayed point back onto exact unrotated CAD point (err = {:.3e} mm)",
                hit_on_ray.distance(sample_cad_pt)
            );
        }
    }

    #[test]
    fn m88_select_xy_plane_x_edge_pick_mode_metadata() {
        let mode = ViewportPickMode::SetupWcsXyPlaneXEdge { setup_idx: 0 };
        let banner = mode
            .banner_text()
            .expect("SetupWcsXyPlaneXEdge must provide a viewport pick banner");
        assert!(banner.contains("+X Edge"));
        let (title, instr, done) = mode
            .guidance_info()
            .expect("SetupWcsXyPlaneXEdge must provide guidance HUD info");
        assert!(title.contains("+X Edge"));
        assert!(instr.contains("edge"));
        assert!(!done);
    }
}
