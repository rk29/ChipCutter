use chip_cad::{CurveKind, PickFilter, PickTarget, SurfaceKind};
use chip_cam::{
    ExtractedFeature25D, GeometrySelectionItem, ManualTabAnchor, OperationStatus, ResolvedHeights,
    StrategyParams, TabMarker3D, TabPlacementMode, TabProfileKind, WcsAxisRef, WcsFeaturePointRef,
    WcsOrientationMode, WcsOriginMode, extract_selection_features_with_sketches,
    resolve_sketch_world_frame, resolve_sketch_world_frame_with_doc,
};
use chip_math::{Aabb2, DVec2, DVec3, ParamLength, Ray3, RigidFrame3};
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
    panels::{InspectorFocus, OperationTab, SetupTab, SketchAuthorTool, ViewportPickMode},
    viewport::{SketchLayerVisual, ToolpathRenderEntry, ZPlaneVisual},
};

pub(crate) struct SketchAuthorInteractionInput {
    pub sketch_idx: usize,
    pub tool: SketchAuthorTool,
    pub ray: Ray3,
    pub mm_per_px: f64,
    pub primary_click: bool,
    pub right_click: bool,
}

impl ChipCutterApp {
    pub(crate) fn sync_viewport_scene_state(&mut self) {
        self.highlight.theme = self.theme;
        self.highlight.extra_selected_faces.clear();
        self.highlight.extra_selected_edges.clear();
        self.highlight.extra_selected_sketch_regions.clear();
        self.highlight.extra_selected_sketch_entities.clear();
        self.highlight.sketch_visuals.clear();
        self.highlight.chain_arrows.clear();
        self.highlight.z_planes_wcs.clear();

        let cached_geo = self.ensure_frame_geo_cached().clone();
        let resolved_opt = cached_geo.resolved_setup.and_then(Result::ok);

        if let Some(rs) = &resolved_opt {
            self.highlight.wcs_frame = rs.frame;
            self.camera.turntable_frame = rs.frame;
            self.highlight.stock = Some(rs.stock);
            for (si, sk) in self.project.sketches.iter().enumerate() {
                if sk.visible {
                    let wf = if let Some(m) = &self.model {
                        resolve_sketch_world_frame_with_doc(&m.doc, sk, rs)
                    } else {
                        resolve_sketch_world_frame(false, sk, rs)
                    };
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
                if let (Some(rs), Some(Ok(sum))) = (&resolved_opt, cached_geo.focused_extracted)
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && let Some(op) = setup.operations.get(op_idx)
                {
                    for fid in sum.highlight_faces {
                        self.highlight.extra_selected_faces.insert(fid);
                    }
                    for eid in sum.highlight_edges {
                        self.highlight.extra_selected_edges.insert(eid);
                    }
                    for rkey in sum.highlight_sketch_regions {
                        self.highlight.extra_selected_sketch_regions.insert(rkey);
                    }
                    for ekey in sum.highlight_sketch_entities {
                        self.highlight.extra_selected_sketch_entities.insert(ekey);
                    }
                    self.highlight.chain_arrows = sum.direction_arrows;

                    if self.show_z_planes && self.op_tab == OperationTab::Heights {
                        let mut ctx = rs.eval_context(self.project.display_units);
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
                            &rs.model_aabb_wcs,
                            &rs.stock.aabb_wcs(),
                            rs.clearance_z_wcs,
                            rs.retract_z_wcs,
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
            InspectorFocus::CadEntity | InspectorFocus::Sketch { .. } => {}
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
                if op.suppressed {
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

            let vis = self.sim_panel.resolve_viewport_visuals(
                &self.project,
                self.active_setup_idx,
                &self.toolpath_cache,
                cad_rev,
            );
            self.highlight.sim_stock_mesh = vis.stock;
            self.highlight.sim_tool_mesh = vis.cutter;
            self.highlight.sim_trail_limit = vis.trail_limit;
        } else {
            self.highlight.sim_stock_mesh = None;
            self.highlight.sim_tool_mesh = None;
            self.highlight.sim_trail_limit = None;
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
            | ViewportPickMode::RebindSketchHostFace { .. } => PickFilter::FacesOnly,
            ViewportPickMode::SetupWcsOriginFeature { .. } => PickFilter::Any,
            ViewportPickMode::OpClosedLoops { .. } => PickFilter::EdgeLoops,
            ViewportPickMode::OpOpenChain { .. } | ViewportPickMode::PlaceContourTab { .. } => {
                PickFilter::EdgesOnly
            }
            ViewportPickMode::OpCylindricalHoles { .. } => PickFilter::CylindricalHolesOnly,
            ViewportPickMode::SketchAuthor { .. } => PickFilter::Any,
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

        for sk in &self.project.sketches {
            if !sk.visible {
                continue;
            }
            let wf = if let Some(m) = &self.model {
                resolve_sketch_world_frame_with_doc(&m.doc, sk, &rs)
            } else {
                resolve_sketch_world_frame(doc_has_bodies, sk, &rs)
            };
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
            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx }
            | ViewportPickMode::OpClosedLoops { setup_idx, op_idx }
            | ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx }
            | ViewportPickMode::OpOpenChain {
                setup_idx, op_idx, ..
            } => {
                let Some(setup) = self.project.setups.get(setup_idx) else {
                    return;
                };
                if op_idx >= setup.operations.len() {
                    return;
                }
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

                self.history
                    .record_before_edit("Toggle 2D Sketch Selection", None, &self.project);
                let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
                if matches!(op_mut.runtime_status, OperationStatus::NeedsRepick { .. }) {
                    op_mut.selections.clear();
                }
                if let Some(pos) =
                    op_mut
                        .selections
                        .iter()
                        .position(|existing| match (existing, &new_item) {
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
                        })
                {
                    op_mut.selections.remove(pos);
                } else {
                    op_mut.selections.push(new_item);
                }
                op_mut.mark_dirty();
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
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && op_idx < setup.operations.len()
                {
                    self.history.record_before_edit(
                        "Toggle Planar Face Selection",
                        None,
                        &self.project,
                    );
                    let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
                    if matches!(op_mut.runtime_status, OperationStatus::NeedsRepick { .. }) {
                        op_mut.selections.clear();
                    }
                    let sels = &mut op_mut.selections;
                    if let Some(pos) = sels.iter().position(|s| {
                        matches!(s, GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid.0)
                    }) {
                        sels.remove(pos);
                    } else {
                        sels.push(GeometrySelectionItem::PlanarFace { binding: bind });
                    }
                    op_mut.mark_dirty();
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
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && op_idx < setup.operations.len()
                {
                    self.history.record_before_edit(
                        "Toggle Closed Loop Selection",
                        None,
                        &self.project,
                    );
                    let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
                    if matches!(op_mut.runtime_status, OperationStatus::NeedsRepick { .. }) {
                        op_mut.selections.clear();
                    }
                    let sels = &mut op_mut.selections;
                    if let Some(pos) = sels.iter().position(|s| {
                        matches!(s, GeometrySelectionItem::ClosedLoop { binding, .. } if binding.last_index == lid.0)
                    }) {
                        sels.remove(pos);
                    } else {
                        sels.push(GeometrySelectionItem::ClosedLoop {
                            binding: bind,
                            reverse: false,
                            side_flip: false,
                        });
                    }
                    op_mut.mark_dirty();
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
                                && let Some(setup) = self.project.setups.get(setup_idx)
                                && op_idx < setup.operations.len()
                            {
                                self.history.record_before_edit(
                                    "Add Open Edge Chain",
                                    None,
                                    &self.project,
                                );
                                let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
                                if matches!(
                                    op_mut.runtime_status,
                                    OperationStatus::NeedsRepick { .. }
                                ) {
                                    op_mut.selections.clear();
                                }
                                op_mut.selections.push(GeometrySelectionItem::OpenChain {
                                    body_id: s_rec.body_id,
                                    start_edge: sb,
                                    end_edge: eb,
                                    resolved_edges: chain_uses,
                                    reverse: false,
                                    side_flip: false,
                                });
                                op_mut.mark_dirty();
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
                    && let Some(setup) = self.project.setups.get(setup_idx)
                    && op_idx < setup.operations.len()
                {
                    self.history
                        .record_before_edit("Toggle Cylindrical Hole", None, &self.project);
                    let op_mut = &mut self.project.setups[setup_idx].operations[op_idx];
                    if matches!(op_mut.runtime_status, OperationStatus::NeedsRepick { .. }) {
                        op_mut.selections.clear();
                    }
                    let sels = &mut op_mut.selections;
                    if let Some(pos) = sels.iter().position(|s| {
                        matches!(s, GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid.0)
                    }) {
                        sels.remove(pos);
                    } else {
                        sels.push(GeometrySelectionItem::CylindricalHole { binding: bind });
                    }
                    op_mut.mark_dirty();
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
        }
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
