use chip_cad::{CurveKind, PickFilter, PickTarget, SurfaceKind};
use chip_cam::{
    GeometrySelectionItem, OperationStatus, WcsAxisRef, WcsFeaturePointRef, WcsOrientationMode,
    WcsOriginMode, cad_binding_to_sketch_face, resolve_sketch_world_frame,
    resolve_sketch_world_frame_with_doc,
};
use chip_math::{DMat4, DVec2, DVec3, ParamLength, Ray3};
use chip_sketch::{
    FaceSketchOriginMode, SketchAnchor, SketchLayer, SketchPickHit, pick_sketch_layer_2d,
};

use crate::{
    app::ChipCutterApp,
    panels::{
        GeometryPickTarget, InspectorFocus, ViewportPickMode,
        operation_inspector::avoid_card::{
            mark_ops_dirty_with_rest, mark_shared_avoid_dependents_dirty,
        },
    },
};

impl ChipCutterApp {
    pub(crate) fn effective_pick_filter(&self) -> PickFilter {
        match self.selection.pick_mode {
            ViewportPickMode::InspectEntity => self.selection.pick_filter,
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
            | ViewportPickMode::PickKeepOutGeometry { .. }
            | ViewportPickMode::Measure => PickFilter::Any,
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
        let doc_has_bodies = self.assets.model.is_some();
        let edge_tol_mm = (10.0 * mm_per_px).clamp(0.4, 8.0);
        let prefer_regions = matches!(
            self.selection.pick_mode,
            ViewportPickMode::OpPlanarFaces { .. }
        );

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
            let mut wf = if let Some(m) = &self.assets.model {
                resolve_sketch_world_frame_with_doc(&m.doc, sk, &rs)
            } else {
                resolve_sketch_world_frame(doc_has_bodies, sk, &rs)
            };
            if has_cad_rot && matches!(sk.anchor, SketchAnchor::PlanarFace { .. }) {
                wf = wf.transformed_by(&cad_xf);
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
        if let Some(eid) = self.selection.editing_shared_avoid_id.clone()
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

        if self.selection.geometry_pick_target == GeometryPickTarget::AvoidKeepOut {
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
        self.workers.cached_frame_geo = None;
    }

    pub(crate) fn handle_viewport_sketch_click(&mut self, hit: SketchPickHit) {
        match self.selection.pick_mode {
            ViewportPickMode::InspectEntity => {
                let sk_id = match &hit {
                    SketchPickHit::Entity { sketch_id, .. }
                    | SketchPickHit::Region { sketch_id, .. } => sketch_id.clone(),
                };
                if let Some(idx) = self.project.sketches.iter().position(|s| s.id == sk_id) {
                    self.selection.inspector_focus = InspectorFocus::Sketch { sketch_idx: idx };
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
            if self.selection.pick_mode == ViewportPickMode::InspectEntity {
                self.highlight.selected = None;
            }
            return;
        };

        let Some(m) = &self.assets.model else {
            return;
        };
        let doc = &m.doc;

        match self.selection.pick_mode {
            ViewportPickMode::InspectEntity => {
                self.highlight.selected = Some(target);
                self.selection.inspector_focus = InspectorFocus::CadEntity;
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
                        self.selection.pick_mode = ViewportPickMode::InspectEntity;
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
                        self.selection.pick_mode = ViewportPickMode::InspectEntity;
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
                    self.selection.pick_mode = ViewportPickMode::InspectEntity;
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
                        self.selection.pick_mode = ViewportPickMode::InspectEntity;
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
                        self.selection.pick_mode = ViewportPickMode::InspectEntity;
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
                            self.selection.pick_mode = ViewportPickMode::OpOpenChain {
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
                            self.selection.pick_mode = ViewportPickMode::OpOpenChain {
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
                    self.selection.inspector_focus = InspectorFocus::Sketch { sketch_idx: sk_idx };
                    self.selection.selected_sketch_entity_idx = None;
                    self.selection.pick_mode = ViewportPickMode::InspectEntity;
                    self.workers.cached_frame_geo = None;
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
                    self.selection.pick_mode = ViewportPickMode::InspectEntity;
                    self.workers.cached_frame_geo = None;
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
            | ViewportPickMode::PlaceFixtureClamp { .. }
            | ViewportPickMode::Measure => {}
        }
    }

    pub(crate) fn apply_lay_flat_face_normal(&mut self, model_idx: usize, normal: DVec3) {
        if model_idx >= self.project.models.len() {
            return;
        }
        let Some(m) = &self.assets.model else {
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
        self.selection.pick_mode = ViewportPickMode::InspectEntity;
        self.sync_loaded_model_transform();
        self.fit_scene_to_viewport();
        self.logs.push(format!(
            "Laid model flat on clicked face (normal {:+.3}, {:+.3}, {:+.3}) and dropped bottom to Z = 0.",
            normal.x, normal.y, normal.z
        ));
    }
}
