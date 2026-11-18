use chip_cad::{CurveKind, PickFilter, PickTarget, SurfaceKind};
use chip_cam::{
    ExtractedFeature25D, GeometrySelectionItem, ManualTabAnchor, OperationStatus, ResolvedHeights,
    StrategyParams, TabMarker3D, TabPlacementMode, TabProfileKind, WcsAxisRef, WcsFeaturePointRef,
    WcsOrientationMode, WcsOriginMode, extract_selection_features_with_sketches,
    resolve_sketch_world_frame,
};
use chip_math::{DVec2, DVec3, Ray3, RigidFrame3};
use chip_sketch::{SketchPickHit, pick_sketch_layer_2d};

use crate::{
    app::ChipCutterApp,
    panels::{InspectorFocus, OperationTab, SetupTab, ViewportPickMode},
    viewport::{SketchLayerVisual, ToolpathRenderEntry, ZPlaneVisual},
};

impl ChipCutterApp {
    pub(crate) fn sync_viewport_scene_state(&mut self) {
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
            let doc_has_bodies = self.model.is_some();
            for sk in &self.project.sketches {
                if sk.visible {
                    let wf = resolve_sketch_world_frame(doc_has_bodies, sk, rs);
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
            ViewportPickMode::SetupWcsXyPlane { .. } | ViewportPickMode::OpPlanarFaces { .. } => {
                PickFilter::FacesOnly
            }
            ViewportPickMode::SetupWcsOriginFeature { .. } => PickFilter::Any,
            ViewportPickMode::OpClosedLoops { .. } => PickFilter::EdgeLoops,
            ViewportPickMode::OpOpenChain { .. } | ViewportPickMode::PlaceContourTab { .. } => {
                PickFilter::EdgesOnly
            }
            ViewportPickMode::OpCylindricalHoles { .. } => PickFilter::CylindricalHolesOnly,
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
            let wf = resolve_sketch_world_frame(doc_has_bodies, sk, &rs);
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
            ViewportPickMode::PlaceContourTab { .. } => {}
        }
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
