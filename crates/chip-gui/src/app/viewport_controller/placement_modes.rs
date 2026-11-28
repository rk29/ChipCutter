use chip_cam::{
    ExtractedFeature25D, FixtureBody, FixtureKind, ManualTabAnchor, StrategyParams, TabMarker3D,
    TabPlacementMode, TabProfileKind, extract_selection_features_with_sketches,
};
use chip_math::{DVec2, DVec3, Ray3};

use crate::{
    app::ChipCutterApp,
    panels::{FixturePlaceKind, ViewportPickMode},
};

impl ChipCutterApp {
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
            self.selection.pick_mode = ViewportPickMode::InspectEntity;
            return;
        }
        if setup_idx >= self.project.setups.len() {
            self.selection.pick_mode = ViewportPickMode::InspectEntity;
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
        self.selection.selected_fixture_idx = Some(setup.fixtures.len() - 1);
        self.highlight.show_fixtures = true;
        self.workers.cached_frame_geo = None;
    }
}
