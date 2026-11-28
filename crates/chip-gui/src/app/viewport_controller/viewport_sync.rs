use std::borrow::Cow;

use chip_cam::{
    BoundaryPreviewInput, GeometrySelectionItem, Operation, ResolvedHeights, ResolvedSetup,
    SharedAvoidPreviewInput, extract_selection_features_with_sketches, resolve_sketch_world_frame,
    resolve_sketch_world_frame_with_doc,
};
use chip_math::{DMat4, RigidFrame3};
use chip_sketch::SketchAnchor;

use super::{effective_setup_for_indexed_preview, resolve_effective_viewport_rotary};
use crate::{
    app::ChipCutterApp,
    panels::{InspectorFocus, OperationTab, SetupTab, ViewportPickMode},
    viewport::{FixtureVisualMesh, SketchLayerVisual, ToolpathRenderEntry, ZPlaneVisual},
};

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
            && self
                .project
                .setups
                .get(self.selection.active_setup_idx)
                .is_some()
        {
            let vis = if self.self_test.is_some() {
                self.sim_panel
                    .resolve_viewport_visuals_sync(&self.project, self.selection.active_setup_idx)
            } else {
                self.sim_panel
                    .resolve_viewport_visuals(&self.project, self.selection.active_setup_idx)
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

        let focused_active_op: Option<&Operation> = match self.selection.inspector_focus {
            InspectorFocus::Operation { setup_idx, op_idx }
                if setup_idx == self.selection.active_setup_idx =>
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
                self.session.prefs.auto_rotate_indexed_op,
                focused_active_op,
                rs.rotary,
            );
            self.highlight.sim_rotary_deg = eff_rotary_deg;

            let selected_fix_id = if matches!(
                self.selection.inspector_focus,
                InspectorFocus::Setup { setup_idx } if setup_idx == self.selection.active_setup_idx
            ) && self.selection.setup_tab == SetupTab::Fixtures
            {
                self.selection
                    .selected_fixture_idx
                    .and_then(|fi| {
                        self.project
                            .setups
                            .get(self.selection.active_setup_idx)
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
                .filter(|_| self.sim_panel.active_setup_idx == self.selection.active_setup_idx)
                .map(|res| {
                    res.diagnostics
                        .iter()
                        .filter(|d| d.kind == chip_sim::SimIssueKind::FixtureCollision)
                        .filter_map(|d| d.fixture_id.as_deref())
                        .collect()
                })
                .unwrap_or_default();
            let keepout_fixture_ids: &[String] = match self.selection.inspector_focus {
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
                    self.session.prefs.auto_rotate_indexed_op,
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
                    let mut wf = if let Some(m) = &self.assets.model {
                        resolve_sketch_world_frame_with_doc(&m.doc, sk, rs_for_sk)
                    } else {
                        resolve_sketch_world_frame(false, sk, rs_for_sk)
                    };
                    if has_cad_rot && matches!(sk.anchor, SketchAnchor::PlanarFace { .. }) {
                        wf = wf.transformed_by(&cad_xf);
                    }
                    if matches!(self.selection.inspector_focus, InspectorFocus::Sketch { sketch_idx } if sketch_idx == si)
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

        match self.selection.inspector_focus {
            InspectorFocus::Setup { setup_idx } => {
                if self.overlay_prefs.show_z_planes
                    && self.selection.setup_tab == SetupTab::Heights
                    && setup_idx == self.selection.active_setup_idx
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

                    let eff_rs = if setup_idx == self.selection.active_setup_idx {
                        effective_setup_for_indexed_preview(
                            rs,
                            self.session.prefs.auto_rotate_indexed_op,
                            sim_active,
                            op,
                        )
                    } else {
                        Cow::Borrowed(rs)
                    };

                    if self.selection.op_tab == OperationTab::Geometry
                        && setup_idx == self.selection.active_setup_idx
                    {
                        let active_tool = op.tool_id.as_ref().and_then(|tid| {
                            self.project
                                .tool(tid)
                                .or_else(|| self.global_tools.tool(tid))
                        });
                        let (doc_opt, cad_rev) = match &self.assets.model {
                            Some(m) => (Some(m.doc.as_ref()), m.revision),
                            None => (None, 0),
                        };
                        self.highlight.boundary_preview = self
                            .workers
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

                    if self.overlay_prefs.show_z_planes
                        && self.selection.op_tab == OperationTab::Heights
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
                    let (doc_opt, cad_rev) = match &self.assets.model {
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
                        .workers
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
        } = self.selection.pick_mode
        {
            self.highlight.extra_selected_edges.insert(se);
        }

        self.highlight.toolpaths.clear();
        if let Some((_, cad_rev)) = self.effective_cad_arc()
            && let Some(setup) = self.project.setups.get(self.selection.active_setup_idx)
        {
            let focused_op_idx = match self.selection.inspector_focus {
                InspectorFocus::Operation { setup_idx, op_idx }
                    if setup_idx == self.selection.active_setup_idx =>
                {
                    Some(op_idx)
                }
                _ => None,
            };

            for (o_idx, op) in setup.operations.iter().enumerate() {
                if op.suppressed || self.overlay_prefs.hidden_operation_ids.contains(&op.id) {
                    continue;
                }
                let is_focused = focused_op_idx.is_none_or(|f_idx| f_idx == o_idx);
                if !self.overlay_prefs.show_all_setup_toolpaths && !is_focused {
                    continue;
                }
                if let Some((hash, tp)) = self.toolpath_cache.get_for_operation(
                    &self.project,
                    self.selection.active_setup_idx,
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

        self.highlight.reach_analysis = if self.highlight.show_reach_heatmap {
            self.workers.reach_result.clone()
        } else {
            None
        };
        self.highlight.measure_state = if self.selection.pick_mode == ViewportPickMode::Measure {
            Some(self.selection.measure_state.clone())
        } else {
            None
        };
    }
}
