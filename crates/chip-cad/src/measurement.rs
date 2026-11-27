//! Exact `f64` B-Rep and mesh geometry measurement kernel (`chip-cad`).
//!
//! Derives measurements directly from exact `f64` [`CadDocument`] B-Rep curves/surfaces,
//! falling back to `f64`-promoted mesh vertices/planes only when no B-Rep target is hit
//! (per `AGENTS.md` §2).

use chip_math::{COINCIDENCE_TOL_MM, DVec3, DisplayUnit, RigidFrame3};

use crate::{
    CadDocument, CurveKind, EdgeId, FaceId, PickHit, PickTarget, SurfaceKind, TopologyGraph,
    expand_coaxial_hole_faces,
};

const PARALLEL_COS_TOL: f64 = 1.0 - 1e-7;
const PARALLEL_AXIS_COS_TOL: f64 = 0.9993908270190958; // cos(2 deg)
const HORIZONTAL_NORMAL_Z_TOL: f64 = 1.0 - 1e-4;

/// Exact `f64` geometric anchor snapped in active **Setup WCS** coordinates.
#[derive(Debug, Clone, PartialEq)]
pub enum MeasureSnapTarget {
    BRepVertex {
        pos_wcs: DVec3,
    },
    BRepEdge {
        edge_id: EdgeId,
        midpoint_wcs: DVec3,
        length_mm: f64,
        arc_radius_mm: Option<f64>,
        arc_center_wcs: Option<DVec3>,
    },
    BRepPlanarFace {
        face_id: FaceId,
        point_wcs: DVec3,
        normal_wcs: DVec3,
        z_wcs: f64,
        area_mm2: f64,
    },
    BRepCylindricalFace {
        face_id: FaceId,
        axis_origin_wcs: DVec3,
        axis_dir_wcs: DVec3,
        diameter_mm: f64,
        depth_mm: f64,
        is_internal: bool,
        top_z_wcs: f64,
        bottom_z_wcs: f64,
    },
    SketchPoint {
        sketch_id: String,
        pos_wcs: DVec3,
    },
    SketchCircle {
        sketch_id: String,
        center_wcs: DVec3,
        diameter_mm: f64,
    },
    MeshPoint {
        pos_wcs: DVec3,
        normal_wcs: DVec3,
    },
}

impl MeasureSnapTarget {
    /// Canonical 3D anchor position in active Setup WCS millimeters.
    pub fn anchor_wcs(&self) -> DVec3 {
        match self {
            Self::BRepVertex { pos_wcs }
            | Self::SketchPoint { pos_wcs, .. }
            | Self::MeshPoint { pos_wcs, .. } => *pos_wcs,
            Self::BRepEdge {
                midpoint_wcs,
                arc_center_wcs,
                ..
            } => arc_center_wcs.unwrap_or(*midpoint_wcs),
            Self::BRepPlanarFace { point_wcs, .. } => *point_wcs,
            Self::BRepCylindricalFace {
                axis_origin_wcs, ..
            } => *axis_origin_wcs,
            Self::SketchCircle { center_wcs, .. } => *center_wcs,
        }
    }

    /// Plain-ASCII concise label suitable for `egui` badges and measurement cards.
    pub fn short_label(&self) -> String {
        match self {
            Self::BRepVertex { .. } => "Vertex".to_string(),
            Self::BRepEdge {
                edge_id,
                arc_radius_mm: Some(_),
                ..
            } => format!("Arc Edge #{}", edge_id.0),
            Self::BRepEdge { edge_id, .. } => format!("Edge #{}", edge_id.0),
            Self::BRepPlanarFace { face_id, .. } => format!("Planar Face #{}", face_id.0),
            Self::BRepCylindricalFace {
                face_id,
                is_internal: true,
                ..
            } => format!("Hole Face #{}", face_id.0),
            Self::BRepCylindricalFace {
                face_id,
                is_internal: false,
                ..
            } => format!("Cylinder Face #{}", face_id.0),
            Self::SketchPoint { .. } => "Sketch Point".to_string(),
            Self::SketchCircle { .. } => "Sketch Circle".to_string(),
            Self::MeshPoint { .. } => "Mesh Point".to_string(),
        }
    }

    /// Suggested WCS Z height if the user applies this anchor to an Operation's Bottom Z.
    pub fn suggested_bottom_z_wcs(&self) -> Option<f64> {
        let z = match self {
            Self::BRepPlanarFace { z_wcs, .. } => *z_wcs,
            Self::BRepCylindricalFace { bottom_z_wcs, .. } => *bottom_z_wcs,
            Self::BRepVertex { pos_wcs }
            | Self::SketchPoint { pos_wcs, .. }
            | Self::MeshPoint { pos_wcs, .. } => pos_wcs.z,
            Self::BRepEdge { midpoint_wcs, .. } => midpoint_wcs.z,
            Self::SketchCircle { center_wcs, .. } => center_wcs.z,
        };
        z.is_finite().then_some(z)
    }
}

/// Exact `f64` inspection or two-entity measurement result in active Setup WCS.
#[derive(Debug, Clone, PartialEq)]
pub enum MeasurementResult {
    SingleEntity {
        target: MeasureSnapTarget,
        anchor_wcs: DVec3,
        primary_value_mm: Option<f64>,
    },
    PointToPoint {
        p1_wcs: DVec3,
        p2_wcs: DVec3,
        distance_3d_mm: f64,
        delta_wcs: DVec3,
        distance_xy_mm: f64,
    },
    ParallelPlanes {
        p1_wcs: DVec3,
        p2_wcs: DVec3,
        leader_p1_wcs: DVec3,
        leader_p2_wcs: DVec3,
        normal_wcs: DVec3,
        perpendicular_gap_mm: f64,
        distance_3d_mm: f64,
        delta_z_wcs: f64,
        is_horizontal: bool,
    },
    AngledPlanes {
        p1_wcs: DVec3,
        p2_wcs: DVec3,
        normal1_wcs: DVec3,
        normal2_wcs: DVec3,
        acute_angle_deg: f64,
        supplementary_angle_deg: f64,
        intersection_dir_wcs: DVec3,
    },
    CylinderToCylinder {
        c1_wcs: DVec3,
        c2_wcs: DVec3,
        diameter1_mm: f64,
        diameter2_mm: f64,
        center_distance_3d_mm: f64,
        center_distance_xy_mm: f64,
        perpendicular_pitch_mm: Option<f64>,
        delta_wcs: DVec3,
        /// Minimum wall thickness between parallel bores/bosses (`perp_pitch - r1 - r2`),
        /// or radial wall `|r1 - r2|` when coaxial. `None` if axes are skewed.
        min_wall_thickness_mm: Option<f64>,
        parallel_axes: bool,
    },
    PointToPlane {
        point_wcs: DVec3,
        plane_point_wcs: DVec3,
        plane_normal_wcs: DVec3,
        projected_on_plane_wcs: DVec3,
        signed_distance_mm: f64,
        abs_distance_mm: f64,
        delta_z_wcs: f64,
    },
}

impl MeasurementResult {
    /// Returns the pair of WCS endpoints for rendering the 3D dimension leader line.
    pub fn leader_endpoints_wcs(&self) -> Option<(DVec3, DVec3)> {
        match self {
            Self::SingleEntity { .. } => None,
            Self::PointToPoint { p1_wcs, p2_wcs, .. }
            | Self::AngledPlanes { p1_wcs, p2_wcs, .. } => Some((*p1_wcs, *p2_wcs)),
            Self::ParallelPlanes {
                leader_p1_wcs,
                leader_p2_wcs,
                ..
            } => Some((*leader_p1_wcs, *leader_p2_wcs)),
            Self::CylinderToCylinder { c1_wcs, c2_wcs, .. } => Some((*c1_wcs, *c2_wcs)),
            Self::PointToPlane {
                point_wcs,
                projected_on_plane_wcs,
                ..
            } => Some((*point_wcs, *projected_on_plane_wcs)),
        }
    }

    /// Plain-ASCII single-line summary formatted in `unit` for clipboard Copy.
    pub fn format_copy_text(&self, unit: DisplayUnit) -> String {
        match self {
            Self::SingleEntity { target, .. } => match target {
                MeasureSnapTarget::BRepVertex { pos_wcs }
                | MeasureSnapTarget::SketchPoint { pos_wcs, .. }
                | MeasureSnapTarget::MeshPoint { pos_wcs, .. } => format!(
                    "{}: X={} Y={} Z={}",
                    target.short_label(),
                    unit.format_length(pos_wcs.x),
                    unit.format_length(pos_wcs.y),
                    unit.format_length(pos_wcs.z),
                ),
                MeasureSnapTarget::BRepEdge {
                    length_mm,
                    arc_radius_mm: Some(r),
                    ..
                } => format!(
                    "{}: R={} Dia={} Len={}",
                    target.short_label(),
                    unit.format_length(*r),
                    unit.format_length(2.0 * *r),
                    unit.format_length(*length_mm),
                ),
                MeasureSnapTarget::BRepEdge { length_mm, .. } => {
                    format!(
                        "{}: Len={}",
                        target.short_label(),
                        unit.format_length(*length_mm)
                    )
                }
                MeasureSnapTarget::BRepPlanarFace {
                    z_wcs, area_mm2, ..
                } => format!(
                    "{}: Z={} Area={:.3} mm^2",
                    target.short_label(),
                    unit.format_length(*z_wcs),
                    area_mm2,
                ),
                MeasureSnapTarget::BRepCylindricalFace {
                    diameter_mm,
                    depth_mm,
                    top_z_wcs,
                    bottom_z_wcs,
                    ..
                } => format!(
                    "{}: Dia={} R={} Depth={} Z=[{}, {}]",
                    target.short_label(),
                    unit.format_length(*diameter_mm),
                    unit.format_length(0.5 * *diameter_mm),
                    unit.format_length(*depth_mm),
                    unit.format_length(*bottom_z_wcs),
                    unit.format_length(*top_z_wcs),
                ),
                MeasureSnapTarget::SketchCircle {
                    center_wcs,
                    diameter_mm,
                    ..
                } => format!(
                    "Sketch Circle: Dia={} R={} Center=({}, {}, {})",
                    unit.format_length(*diameter_mm),
                    unit.format_length(0.5 * *diameter_mm),
                    unit.format_length(center_wcs.x),
                    unit.format_length(center_wcs.y),
                    unit.format_length(center_wcs.z),
                ),
            },
            Self::PointToPoint {
                distance_3d_mm,
                delta_wcs,
                distance_xy_mm,
                ..
            } => format!(
                "Dist={} XY={} dX={} dY={} dZ={}",
                unit.format_length(*distance_3d_mm),
                unit.format_length(*distance_xy_mm),
                unit.format_length(delta_wcs.x),
                unit.format_length(delta_wcs.y),
                unit.format_length(delta_wcs.z),
            ),
            Self::ParallelPlanes {
                perpendicular_gap_mm,
                delta_z_wcs,
                distance_3d_mm,
                ..
            } => format!(
                "Gap={} dZ={} Dist3D={}",
                unit.format_length(*perpendicular_gap_mm),
                unit.format_length(*delta_z_wcs),
                unit.format_length(*distance_3d_mm),
            ),
            Self::AngledPlanes {
                acute_angle_deg,
                supplementary_angle_deg,
                ..
            } => format!(
                "Angle={:.3} deg (supp {:.3} deg)",
                acute_angle_deg, supplementary_angle_deg,
            ),
            Self::CylinderToCylinder {
                diameter1_mm,
                diameter2_mm,
                center_distance_3d_mm,
                center_distance_xy_mm,
                min_wall_thickness_mm,
                ..
            } => {
                let wall = min_wall_thickness_mm
                    .map(|w| format!(" Wall={}", unit.format_length(w)))
                    .unwrap_or_default();
                format!(
                    "PitchXY={} Pitch3D={} Dia1={} Dia2={}{}",
                    unit.format_length(*center_distance_xy_mm),
                    unit.format_length(*center_distance_3d_mm),
                    unit.format_length(*diameter1_mm),
                    unit.format_length(*diameter2_mm),
                    wall,
                )
            }
            Self::PointToPlane {
                abs_distance_mm,
                signed_distance_mm,
                delta_z_wcs,
                ..
            } => format!(
                "PerpDist={} Signed={} dZ={}",
                unit.format_length(*abs_distance_mm),
                unit.format_length(*signed_distance_mm),
                unit.format_length(*delta_z_wcs),
            ),
        }
    }
}

/// Inspects a single snapped geometric entity in exact `f64` Setup WCS.
pub fn measure_single(target: &MeasureSnapTarget) -> MeasurementResult {
    let primary_value_mm = match target {
        MeasureSnapTarget::BRepVertex { .. }
        | MeasureSnapTarget::SketchPoint { .. }
        | MeasureSnapTarget::MeshPoint { .. } => None,
        MeasureSnapTarget::BRepEdge {
            length_mm,
            arc_radius_mm,
            ..
        } => Some(arc_radius_mm.unwrap_or(*length_mm)),
        MeasureSnapTarget::BRepPlanarFace { z_wcs, .. } => Some(*z_wcs),
        MeasureSnapTarget::BRepCylindricalFace { diameter_mm, .. }
        | MeasureSnapTarget::SketchCircle { diameter_mm, .. } => Some(*diameter_mm),
    };
    MeasurementResult::SingleEntity {
        target: target.clone(),
        anchor_wcs: target.anchor_wcs(),
        primary_value_mm,
    }
}

/// Computes the exact `f64` geometric measurement between two snapped entities in active Setup WCS.
pub fn measure_pair(first: &MeasureSnapTarget, second: &MeasureSnapTarget) -> MeasurementResult {
    // 1. Plane-to-Plane (Parallel gap vs Angled dihedral)
    if let (
        MeasureSnapTarget::BRepPlanarFace {
            point_wcs: p1,
            normal_wcs: n1_raw,
            ..
        },
        MeasureSnapTarget::BRepPlanarFace {
            point_wcs: p2,
            normal_wcs: n2_raw,
            ..
        },
    ) = (first, second)
    {
        let n1 = safe_unit(*n1_raw, DVec3::Z);
        let n2 = safe_unit(*n2_raw, DVec3::Z);
        let cos_raw = n1.dot(n2).clamp(-1.0, 1.0);
        let abs_cos = cos_raw.abs();
        if abs_cos >= PARALLEL_COS_TOL {
            let signed_gap = (*p2 - *p1).dot(n1);
            let perpendicular_gap_mm = signed_gap.abs();
            let leader_p1_wcs = *p1;
            let leader_p2_wcs = *p1 + n1 * signed_gap;
            return MeasurementResult::ParallelPlanes {
                p1_wcs: *p1,
                p2_wcs: *p2,
                leader_p1_wcs,
                leader_p2_wcs,
                normal_wcs: n1,
                perpendicular_gap_mm,
                distance_3d_mm: (*p2 - *p1).length(),
                delta_z_wcs: p2.z - p1.z,
                is_horizontal: n1.z.abs() >= HORIZONTAL_NORMAL_Z_TOL,
            };
        }
        let acute_angle_deg = abs_cos.acos().to_degrees();
        let supplementary_angle_deg = 180.0 - acute_angle_deg;
        let intersection_dir_wcs = n1.cross(n2).normalize_or_zero();
        return MeasurementResult::AngledPlanes {
            p1_wcs: *p1,
            p2_wcs: *p2,
            normal1_wcs: n1,
            normal2_wcs: n2,
            acute_angle_deg,
            supplementary_angle_deg,
            intersection_dir_wcs,
        };
    }

    // 2. Cylinder/Circle-to-Cylinder/Circle (center pitch + min wall thickness)
    if let (Some((c1, a1, d1)), Some((c2, a2, d2))) =
        (extract_cylinder_like(first), extract_cylinder_like(second))
    {
        let u1 = safe_unit(a1, DVec3::Z);
        let u2 = safe_unit(a2, DVec3::Z);
        let delta = c2 - c1;
        let center_distance_3d_mm = delta.length();
        let center_distance_xy_mm = (delta.x * delta.x + delta.y * delta.y).sqrt();
        let parallel_axes = u1.dot(u2).abs() >= PARALLEL_AXIS_COS_TOL;
        let (perpendicular_pitch_mm, min_wall_thickness_mm) = if parallel_axes {
            let perp = delta - u1 * delta.dot(u1);
            let perp_dist = perp.length();
            let r1 = 0.5 * d1;
            let r2 = 0.5 * d2;
            let wall = if perp_dist <= COINCIDENCE_TOL_MM {
                (r1 - r2).abs()
            } else {
                perp_dist - r1 - r2
            };
            (Some(perp_dist), Some(wall))
        } else {
            (None, None)
        };
        return MeasurementResult::CylinderToCylinder {
            c1_wcs: c1,
            c2_wcs: c2,
            diameter1_mm: d1,
            diameter2_mm: d2,
            center_distance_3d_mm,
            center_distance_xy_mm,
            perpendicular_pitch_mm,
            delta_wcs: delta,
            min_wall_thickness_mm,
            parallel_axes,
        };
    }

    // 3. Point-to-Plane (either ordering)
    if let MeasureSnapTarget::BRepPlanarFace {
        point_wcs: plane_pt,
        normal_wcs: plane_n_raw,
        ..
    } = first
    {
        return measure_point_to_plane(second.anchor_wcs(), *plane_pt, *plane_n_raw);
    }
    if let MeasureSnapTarget::BRepPlanarFace {
        point_wcs: plane_pt,
        normal_wcs: plane_n_raw,
        ..
    } = second
    {
        return measure_point_to_plane(first.anchor_wcs(), *plane_pt, *plane_n_raw);
    }

    // 4. General Point-to-Point
    let p1_wcs = first.anchor_wcs();
    let p2_wcs = second.anchor_wcs();
    let delta_wcs = p2_wcs - p1_wcs;
    let distance_3d_mm = delta_wcs.length();
    let distance_xy_mm = (delta_wcs.x * delta_wcs.x + delta_wcs.y * delta_wcs.y).sqrt();
    MeasurementResult::PointToPoint {
        p1_wcs,
        p2_wcs,
        distance_3d_mm,
        delta_wcs,
        distance_xy_mm,
    }
}

fn measure_point_to_plane(
    point_wcs: DVec3,
    plane_point_wcs: DVec3,
    plane_normal_raw: DVec3,
) -> MeasurementResult {
    let n = safe_unit(plane_normal_raw, DVec3::Z);
    let signed_distance_mm = (point_wcs - plane_point_wcs).dot(n);
    let projected_on_plane_wcs = point_wcs - n * signed_distance_mm;
    MeasurementResult::PointToPlane {
        point_wcs,
        plane_point_wcs,
        plane_normal_wcs: n,
        projected_on_plane_wcs,
        signed_distance_mm,
        abs_distance_mm: signed_distance_mm.abs(),
        delta_z_wcs: point_wcs.z - plane_point_wcs.z,
    }
}

fn extract_cylinder_like(target: &MeasureSnapTarget) -> Option<(DVec3, DVec3, f64)> {
    match target {
        MeasureSnapTarget::BRepCylindricalFace {
            axis_origin_wcs,
            axis_dir_wcs,
            diameter_mm,
            ..
        } => Some((*axis_origin_wcs, *axis_dir_wcs, *diameter_mm)),
        MeasureSnapTarget::SketchCircle {
            center_wcs,
            diameter_mm,
            ..
        } => Some((*center_wcs, DVec3::Z, *diameter_mm)),
        MeasureSnapTarget::BRepEdge {
            arc_radius_mm: Some(r),
            arc_center_wcs: Some(c),
            ..
        } => Some((*c, DVec3::Z, 2.0 * *r)),
        _ => None,
    }
}

/// Converts a B-Rep viewport [`PickHit`] into an exact `f64` [`MeasureSnapTarget`] in active
/// Setup WCS coordinates, snapping to exact B-Rep vertices within `snap_radius_mm` when close.
pub fn snap_brep_pick_to_measure_target(
    doc: &CadDocument,
    topo: &TopologyGraph,
    setup_frame: &RigidFrame3,
    hit: &PickHit,
    snap_radius_mm: f64,
) -> Option<MeasureSnapTarget> {
    let snap_r = snap_radius_mm.max(0.0);
    match &hit.target {
        PickTarget::Edge(edge_id)
        | PickTarget::Loop {
            seed_edge: edge_id, ..
        } => snap_brep_edge_to_measure_target(doc, setup_frame, *edge_id, hit.point_mm, snap_r),
        PickTarget::Face(face_id) => {
            snap_brep_face_to_measure_target(doc, topo, setup_frame, *face_id, hit.point_mm, snap_r)
        }
    }
}

/// Builds an exact [`MeasureSnapTarget`] for a B-Rep edge (snapping to endpoints if within `snap_radius_mm`).
pub fn snap_brep_edge_to_measure_target(
    doc: &CadDocument,
    setup_frame: &RigidFrame3,
    edge_id: EdgeId,
    hit_point_world: DVec3,
    snap_radius_mm: f64,
) -> Option<MeasureSnapTarget> {
    let edge = doc.edge(edge_id)?;
    match edge.curve {
        CurveKind::Line { start_mm, end_mm } => {
            if snap_radius_mm > 0.0 {
                let ds = hit_point_world.distance(start_mm);
                let de = hit_point_world.distance(end_mm);
                if ds <= snap_radius_mm && ds <= de {
                    return Some(MeasureSnapTarget::BRepVertex {
                        pos_wcs: setup_frame.world_to_local_point(start_mm),
                    });
                }
                if de <= snap_radius_mm {
                    return Some(MeasureSnapTarget::BRepVertex {
                        pos_wcs: setup_frame.world_to_local_point(end_mm),
                    });
                }
            }
            let mid_world = 0.5 * (start_mm + end_mm);
            Some(MeasureSnapTarget::BRepEdge {
                edge_id,
                midpoint_wcs: setup_frame.world_to_local_point(mid_world),
                length_mm: start_mm.distance(end_mm),
                arc_radius_mm: None,
                arc_center_wcs: None,
            })
        }
        CurveKind::CircleArc {
            center_mm,
            radius_mm,
            start_mm,
            end_mm,
            is_full_circle,
            ..
        } => {
            if !is_full_circle && snap_radius_mm > 0.0 {
                let ds = hit_point_world.distance(start_mm);
                let de = hit_point_world.distance(end_mm);
                if ds <= snap_radius_mm && ds <= de {
                    return Some(MeasureSnapTarget::BRepVertex {
                        pos_wcs: setup_frame.world_to_local_point(start_mm),
                    });
                }
                if de <= snap_radius_mm {
                    return Some(MeasureSnapTarget::BRepVertex {
                        pos_wcs: setup_frame.world_to_local_point(end_mm),
                    });
                }
            }
            let mid_world = edge_polyline_midpoint(doc, edge_id).unwrap_or(center_mm);
            Some(MeasureSnapTarget::BRepEdge {
                edge_id,
                midpoint_wcs: setup_frame.world_to_local_point(mid_world),
                length_mm: edge.length_mm,
                arc_radius_mm: Some(radius_mm),
                arc_center_wcs: Some(setup_frame.world_to_local_point(center_mm)),
            })
        }
        CurveKind::EllipseArc | CurveKind::OtherNurbs => {
            let mid_world =
                edge_polyline_midpoint(doc, edge_id).unwrap_or(edge.signature.centroid_mm);
            Some(MeasureSnapTarget::BRepEdge {
                edge_id,
                midpoint_wcs: setup_frame.world_to_local_point(mid_world),
                length_mm: edge.length_mm,
                arc_radius_mm: None,
                arc_center_wcs: None,
            })
        }
    }
}

/// Builds an exact [`MeasureSnapTarget`] for a B-Rep face (planar or cylindrical), optionally
/// snapping to a nearby topological vertex within `snap_radius_mm`.
pub fn snap_brep_face_to_measure_target(
    doc: &CadDocument,
    topo: &TopologyGraph,
    setup_frame: &RigidFrame3,
    face_id: FaceId,
    hit_point_world: DVec3,
    snap_radius_mm: f64,
) -> Option<MeasureSnapTarget> {
    let face = doc.face(face_id)?;

    // Vertex snap check on the picked face's boundary loops
    if snap_radius_mm > 0.0
        && let Some(v_world) =
            nearest_face_vertex(doc, topo, face_id, hit_point_world, snap_radius_mm)
    {
        return Some(MeasureSnapTarget::BRepVertex {
            pos_wcs: setup_frame.world_to_local_point(v_world),
        });
    }

    match face.surface {
        SurfaceKind::Plane { origin_mm, normal } => {
            let n_world = safe_unit(normal, setup_frame.z_axis);
            // Project pick point onto the exact analytic plane (never rely on tessellation Z!)
            let exact_pt_world =
                hit_point_world - n_world * (hit_point_world - origin_mm).dot(n_world);
            let point_wcs = setup_frame.world_to_local_point(exact_pt_world);
            let normal_wcs = safe_unit(setup_frame.world_to_local_dir(n_world), DVec3::Z);
            Some(MeasureSnapTarget::BRepPlanarFace {
                face_id,
                point_wcs,
                normal_wcs,
                z_wcs: point_wcs.z,
                area_mm2: face.area_mm2,
            })
        }
        SurfaceKind::Cylinder {
            axis_origin_mm,
            axis_dir,
            radius_mm,
            is_internal,
        } => {
            let mut axis_world = safe_unit(axis_dir, setup_frame.z_axis);
            if axis_world.dot(setup_frame.z_axis) < 0.0 {
                axis_world = -axis_world;
            }
            let group = if is_internal {
                expand_coaxial_hole_faces(doc, face_id)
            } else {
                vec![face_id]
            };

            let (min_ax, max_ax, min_z_wcs, max_z_wcs) =
                cylinder_group_axial_extents(doc, &group, axis_world, setup_frame);
            let depth_mm = if max_ax.is_finite() && min_ax.is_finite() {
                (max_ax - min_ax).max(0.0)
            } else {
                0.0
            };
            // Place canonical axis anchor at the upper rim along `axis_world` so coaxial/same-plane
            // holes measure exact center-to-center pitch in both 2D XY and 3D!
            let anchor_world = if max_ax.is_finite() {
                axis_origin_mm + axis_world * (max_ax - axis_origin_mm.dot(axis_world))
            } else {
                axis_origin_mm
            };
            let axis_origin_wcs = setup_frame.world_to_local_point(anchor_world);
            let axis_dir_wcs = safe_unit(setup_frame.world_to_local_dir(axis_world), DVec3::Z);
            let top_z_wcs = if max_z_wcs.is_finite() {
                max_z_wcs
            } else {
                axis_origin_wcs.z
            };
            let bottom_z_wcs = if min_z_wcs.is_finite() {
                min_z_wcs
            } else {
                axis_origin_wcs.z - depth_mm
            };

            Some(MeasureSnapTarget::BRepCylindricalFace {
                face_id,
                axis_origin_wcs,
                axis_dir_wcs,
                diameter_mm: 2.0 * radius_mm,
                depth_mm,
                is_internal,
                top_z_wcs,
                bottom_z_wcs,
            })
        }
        SurfaceKind::Cone { .. } | SurfaceKind::Torus { .. } | SurfaceKind::OtherNurbs => {
            let normal_wcs = safe_unit(
                setup_frame.world_to_local_dir(face.signature.normal_or_axis),
                DVec3::Z,
            );
            Some(MeasureSnapTarget::MeshPoint {
                pos_wcs: setup_frame.world_to_local_point(hit_point_world),
                normal_wcs,
            })
        }
    }
}

/// Promotes a triangle-mesh ray hit (`STL`/`OBJ`/`3MF`) to a `MeasureSnapTarget::MeshPoint` in Setup WCS.
pub fn snap_mesh_hit_to_measure_target(
    point_world: DVec3,
    normal_world: DVec3,
    setup_frame: &RigidFrame3,
) -> MeasureSnapTarget {
    MeasureSnapTarget::MeshPoint {
        pos_wcs: setup_frame.world_to_local_point(point_world),
        normal_wcs: safe_unit(setup_frame.world_to_local_dir(normal_world), DVec3::Z),
    }
}

fn edge_polyline_midpoint(doc: &CadDocument, edge_id: EdgeId) -> Option<DVec3> {
    let ep = doc
        .render_mesh
        .edge_polylines
        .iter()
        .find(|p| p.edge_id == edge_id)?;
    let n = ep.points_mm.len();
    if n == 0 {
        return None;
    }
    if n == 1 {
        return Some(ep.points_mm[0]);
    }
    let total_len: f64 = ep.points_mm.windows(2).map(|w| w[0].distance(w[1])).sum();
    if total_len <= 1e-12 {
        return Some(ep.points_mm[n / 2]);
    }
    let half = 0.5 * total_len;
    let mut accum = 0.0;
    for w in ep.points_mm.windows(2) {
        let seg = w[0].distance(w[1]);
        if accum + seg >= half && seg > 1e-12 {
            let t = ((half - accum) / seg).clamp(0.0, 1.0);
            return Some(w[0].lerp(w[1], t));
        }
        accum += seg;
    }
    ep.points_mm.last().copied()
}

fn nearest_face_vertex(
    doc: &CadDocument,
    topo: &TopologyGraph,
    face_id: FaceId,
    hit_world: DVec3,
    snap_radius_mm: f64,
) -> Option<DVec3> {
    let face = doc.face(face_id)?;
    let mut best: Option<(f64, DVec3)> = None;
    let loop_ids = face
        .outer_loop
        .into_iter()
        .chain(face.inner_loops.iter().copied());
    for lid in loop_ids {
        let Some(lp) = doc.brep_loop(lid) else {
            continue;
        };
        for u in &lp.edges {
            if let Some(ep) = topo.endpoint_info(u.edge_id)
                && !ep.is_closed
            {
                for v in [ep.start_pos_mm, ep.end_pos_mm] {
                    let d = hit_world.distance(v);
                    if d <= snap_radius_mm && best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, v));
                    }
                }
            }
        }
    }
    best.map(|(_, v)| v)
}

fn cylinder_group_axial_extents(
    doc: &CadDocument,
    group: &[FaceId],
    axis_world: DVec3,
    setup_frame: &RigidFrame3,
) -> (f64, f64, f64, f64) {
    let mut min_ax = f64::INFINITY;
    let mut max_ax = f64::NEG_INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;

    for &fid in group {
        let Some(face) = doc.face(fid) else {
            continue;
        };
        let loop_ids = face
            .outer_loop
            .into_iter()
            .chain(face.inner_loops.iter().copied());
        for lid in loop_ids {
            let Some(lp) = doc.brep_loop(lid) else {
                continue;
            };
            for u in &lp.edges {
                let Some(edge) = doc.edge(u.edge_id) else {
                    continue;
                };
                match edge.curve {
                    CurveKind::Line { start_mm, end_mm } => {
                        for p in [start_mm, end_mm] {
                            let ax = p.dot(axis_world);
                            min_ax = min_ax.min(ax);
                            max_ax = max_ax.max(ax);
                            let z = setup_frame.world_to_local_point(p).z;
                            min_z = min_z.min(z);
                            max_z = max_z.max(z);
                        }
                    }
                    CurveKind::CircleArc {
                        center_mm,
                        start_mm,
                        end_mm,
                        ..
                    } => {
                        for p in [center_mm, start_mm, end_mm] {
                            let ax = p.dot(axis_world);
                            min_ax = min_ax.min(ax);
                            max_ax = max_ax.max(ax);
                            let z = setup_frame.world_to_local_point(p).z;
                            min_z = min_z.min(z);
                            max_z = max_z.max(z);
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    if !min_ax.is_finite() || !max_ax.is_finite() {
        for tri in &doc.render_mesh.pick_triangles {
            if group.contains(&tri.face_id) {
                for p in [tri.v0_mm, tri.v1_mm, tri.v2_mm] {
                    let ax = p.dot(axis_world);
                    min_ax = min_ax.min(ax);
                    max_ax = max_ax.max(ax);
                    let z = setup_frame.world_to_local_point(p).z;
                    min_z = min_z.min(z);
                    max_z = max_z.max(z);
                }
            }
        }
    }

    (min_ax, max_ax, min_z, max_z)
}

#[inline]
fn safe_unit(v: DVec3, fallback: DVec3) -> DVec3 {
    let n = v.normalize_or_zero();
    if n.length_squared() > 0.5 {
        n
    } else {
        fallback
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, RigidFrame3, approx_eq};

    use super::*;
    use crate::{
        CadKernel, MonstertruckKernel, find_same_diameter_holes, sample_multi_feature_bracket_step,
    };

    #[test]
    fn exact_f64_brep_bracket_measures_hole_diameter_center_pitch_and_parallel_face_depth() {
        let step = sample_multi_feature_bracket_step(false).unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let topo = TopologyGraph::build(&doc);
        let frame = RigidFrame3::from_z_and_x_hint(
            doc.aabb_mm.min,
            DVec3::Z,
            Some(DVec3::X),
            false,
            false,
            0,
        )
        .unwrap();

        // 1. Single-pick Ø6 mm mounting hole -> exact 6.0 mm diameter & 20.0 mm through depth
        let r3_seed = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Cylinder {
                        radius_mm,
                        is_internal: true,
                        ..
                    } if approx_eq(radius_mm, 3.0, 1e-3)
                )
            })
            .unwrap()
            .id;

        let hole_faces = find_same_diameter_holes(&doc, r3_seed, 1e-3, 1e-3);
        let mut unique_holes = Vec::new();
        for fid in hole_faces {
            let t = snap_brep_face_to_measure_target(
                &doc,
                &topo,
                &frame,
                fid,
                doc.face(fid).unwrap().signature.centroid_mm,
                0.0,
            )
            .unwrap();
            let c = t.anchor_wcs();
            if !unique_holes
                .iter()
                .any(|u: &MeasureSnapTarget| u.anchor_wcs().distance(c) < 0.1)
            {
                unique_holes.push(t);
            }
        }
        assert_eq!(unique_holes.len(), 4);

        match measure_single(&unique_holes[0]) {
            MeasurementResult::SingleEntity {
                target:
                    MeasureSnapTarget::BRepCylindricalFace {
                        diameter_mm,
                        depth_mm,
                        ..
                    },
                ..
            } => {
                assert!(
                    (diameter_mm - 6.0).abs() < 1e-6,
                    "expected exact 6.0 mm hole diameter, got {diameter_mm}"
                );
                assert!(
                    (depth_mm - 20.0).abs() < 1e-4,
                    "expected 20.0 mm through-hole depth, got {depth_mm}"
                );
            }
            other => panic!("expected SingleEntity cylindrical hole, got {other:?}"),
        }

        // 2. Two-pick hole-to-hole center pitch & min wall thickness
        unique_holes.sort_by(|a, b| {
            a.anchor_wcs()
                .x
                .total_cmp(&b.anchor_wcs().x)
                .then_with(|| a.anchor_wcs().y.total_cmp(&b.anchor_wcs().y))
        });
        let pair_res = measure_pair(&unique_holes[0], &unique_holes[1]);
        match pair_res {
            MeasurementResult::CylinderToCylinder {
                diameter1_mm,
                diameter2_mm,
                center_distance_xy_mm,
                min_wall_thickness_mm,
                parallel_axes,
                ..
            } => {
                assert!(parallel_axes);
                assert!((diameter1_mm - 6.0).abs() < 1e-6);
                assert!((diameter2_mm - 6.0).abs() < 1e-6);
                let p0 = unique_holes[0].anchor_wcs();
                let p1 = unique_holes[1].anchor_wcs();
                let expected_pitch = (p1.x - p0.x).hypot(p1.y - p0.y);
                assert!((center_distance_xy_mm - expected_pitch).abs() < 1e-6);
                assert!(center_distance_xy_mm > 10.0);
                let wall = min_wall_thickness_mm.expect("parallel holes have min wall");
                assert!((wall - (expected_pitch - 6.0)).abs() < 1e-6);
            }
            other => panic!("expected CylinderToCylinder, got {other:?}"),
        }

        // 3. Parallel face Z depth (top face Z=20.0 mm to Pocket A floor Z=12.0 mm => 8.0 mm gap)
        let top_face = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 20.0, 1e-2)
                )
            })
            .unwrap();
        let pkt_floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
                )
            })
            .unwrap();

        let t_top = snap_brep_face_to_measure_target(
            &doc,
            &topo,
            &frame,
            top_face.id,
            top_face.signature.centroid_mm,
            0.0,
        )
        .unwrap();
        let t_pkt = snap_brep_face_to_measure_target(
            &doc,
            &topo,
            &frame,
            pkt_floor.id,
            pkt_floor.signature.centroid_mm,
            0.0,
        )
        .unwrap();

        match measure_pair(&t_top, &t_pkt) {
            MeasurementResult::ParallelPlanes {
                perpendicular_gap_mm,
                delta_z_wcs,
                is_horizontal,
                ..
            } => {
                assert!(is_horizontal);
                assert!(
                    (perpendicular_gap_mm - 8.0).abs() < 1e-6,
                    "expected 8.0 mm pocket depth, got {perpendicular_gap_mm}"
                );
                assert!((delta_z_wcs + 8.0).abs() < 1e-6);
            }
            other => panic!("expected ParallelPlanes, got {other:?}"),
        }
    }

    #[test]
    fn exact_45_deg_bevel_dihedral_angle_and_point_to_plane_measurements() {
        // 45-degree bevel planar face against horizontal Z+ face
        let horiz = MeasureSnapTarget::BRepPlanarFace {
            face_id: FaceId(1),
            point_wcs: DVec3::new(10.0, 10.0, 20.0),
            normal_wcs: DVec3::Z,
            z_wcs: 20.0,
            area_mm2: 100.0,
        };
        let bevel = MeasureSnapTarget::BRepPlanarFace {
            face_id: FaceId(2),
            point_wcs: DVec3::new(20.0, 10.0, 18.0),
            normal_wcs: DVec3::new(1.0, 0.0, 1.0).normalize(),
            z_wcs: 18.0,
            area_mm2: 28.284,
        };
        match measure_pair(&horiz, &bevel) {
            MeasurementResult::AngledPlanes {
                acute_angle_deg,
                supplementary_angle_deg,
                intersection_dir_wcs,
                ..
            } => {
                assert!((acute_angle_deg - 45.0).abs() < 1e-9);
                assert!((supplementary_angle_deg - 135.0).abs() < 1e-9);
                assert!(intersection_dir_wcs.y.abs() > 0.999999);
            }
            other => panic!("expected AngledPlanes, got {other:?}"),
        }

        // Point to plane signed perpendicular distance
        let vtx = MeasureSnapTarget::BRepVertex {
            pos_wcs: DVec3::new(10.0, 10.0, 12.5),
        };
        match measure_pair(&vtx, &horiz) {
            MeasurementResult::PointToPlane {
                signed_distance_mm,
                abs_distance_mm,
                ..
            } => {
                assert!((signed_distance_mm + 7.5).abs() < 1e-9);
                assert!((abs_distance_mm - 7.5).abs() < 1e-9);
            }
            other => panic!("expected PointToPlane, got {other:?}"),
        }
    }
}
