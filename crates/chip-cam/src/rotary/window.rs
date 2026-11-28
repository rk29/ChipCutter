use chip_cad::{CadDocument, CurveKind, EdgeId, FaceId, LoopId};
use chip_math::{Aabb3, DVec3, RigidFrame3};

use crate::{
    params::{RotaryAxialBoundaryConfig, RotaryAxialBoundaryMode, ToolContainmentMode},
    selection::{ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem},
    surface3d::SurfaceMesh3D,
    toolpath::{GeneratedToolpath, SegmentMotion3D},
    wcs::CamError,
};

use super::{
    ROTARY_CHUCK_GUARD_CLEARANCE_MM, ResolvedRotaryConfig, RotaryAxisKind, rotary_chuck_guard_u,
};

/// Fully resolved 4th-axis rotary axial (`-U`/`+U`), angular (`θ`), and radial (`r_floor..r_ceiling`)
/// machining window shared by native rotary toolpath kernels and live 3D viewport previews.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedRotaryWindow {
    /// Nominal axial bounds before tool-containment & margin shifts (`mm` along rotary axis).
    pub nominal_u_min: f64,
    pub nominal_u_max: f64,
    /// Final chuck-safe tool-center axial span `[u_start, u_end]` (`mm`).
    pub u_start: f64,
    pub u_end: f64,
    /// Effective angular span `[start_angle_deg, end_angle_deg]` (`deg`).
    pub start_angle_deg: f64,
    pub end_angle_deg: f64,
    /// Effective radial envelope `[r_floor, r_ceiling]` (`mm` from rotary centerline).
    pub r_floor: f64,
    pub r_ceiling: f64,
    /// Hard chuck keep-out boundary plane coordinate (`mm`) along the rotary axis.
    pub chuck_guard_u: f64,
    /// Whether the chuck keep-out guard actively clamped `u_start` or `u_end`.
    pub clamped_by_chuck: bool,
    pub warnings: Vec<String>,
}

/// Input bundle for [`resolve_rotary_window`].
pub struct RotaryWindowInput<'a> {
    pub rot: &'a ResolvedRotaryConfig,
    pub frame: &'a RigidFrame3,
    pub doc: Option<&'a CadDocument>,
    pub base_mesh: &'a SurfaceMesh3D,
    pub selections: &'a [GeometrySelectionItem],
    pub extracted_selection: Option<&'a ExtractedSelectionSummary>,
    pub boundary: &'a RotaryAxialBoundaryConfig,
    pub tool_diameter_mm: f64,
    pub top_z_wcs: f64,
    pub bottom_z_wcs: f64,
    pub op_name: &'a str,
}

fn extend_u_from_aabb_world(
    aabb_world: &Aabb3,
    frame: &RigidFrame3,
    u_vec: DVec3,
    u_min: &mut f64,
    u_max: &mut f64,
) {
    if !aabb_world.is_valid() {
        return;
    }
    for x in [aabb_world.min.x, aabb_world.max.x] {
        for y in [aabb_world.min.y, aabb_world.max.y] {
            for z in [aabb_world.min.z, aabb_world.max.z] {
                let p_wcs = frame.world_to_local_point(DVec3::new(x, y, z));
                let u = p_wcs.dot(u_vec);
                *u_min = (*u_min).min(u);
                *u_max = (*u_max).max(u);
            }
        }
    }
}

fn extend_u_from_edge(
    doc: &CadDocument,
    eid: EdgeId,
    frame: &RigidFrame3,
    u_vec: DVec3,
    u_min: &mut f64,
    u_max: &mut f64,
) {
    let mut found_pts = false;
    if let Some(ep) = doc
        .render_mesh
        .edge_polylines
        .iter()
        .find(|p| p.edge_id == eid)
    {
        for &pt_world in &ep.points_mm {
            let u = frame.world_to_local_point(pt_world).dot(u_vec);
            *u_min = (*u_min).min(u);
            *u_max = (*u_max).max(u);
            found_pts = true;
        }
    }
    if let Some(edge) = doc.edge(eid) {
        match edge.curve {
            CurveKind::Line { start_mm, end_mm } => {
                for p in [start_mm, end_mm] {
                    let u = frame.world_to_local_point(p).dot(u_vec);
                    *u_min = (*u_min).min(u);
                    *u_max = (*u_max).max(u);
                    found_pts = true;
                }
            }
            CurveKind::CircleArc {
                center_mm,
                normal,
                radius_mm,
                start_mm,
                end_mm,
                is_full_circle,
            } => {
                for p in [start_mm, end_mm] {
                    let u = frame.world_to_local_point(p).dot(u_vec);
                    *u_min = (*u_min).min(u);
                    *u_max = (*u_max).max(u);
                    found_pts = true;
                }
                if is_full_circle {
                    // Exact axial projection of a 3D circle of radius `r` and unit normal `n`:
                    // `u(θ) = c · u_vec ± r * sqrt(1 - (n · u_vec)^2)`.
                    let c_wcs = frame.world_to_local_point(center_mm);
                    let n_wcs = frame.world_to_local_dir(normal.normalize_or_zero());
                    let cos_nu = n_wcs.dot(u_vec).clamp(-1.0, 1.0);
                    let sin_nu = (1.0 - cos_nu * cos_nu).max(0.0).sqrt();
                    let cu = c_wcs.dot(u_vec);
                    let r_u = radius_mm.abs() * sin_nu;
                    *u_min = (*u_min).min(cu - r_u);
                    *u_max = (*u_max).max(cu + r_u);
                    found_pts = true;
                }
            }
            _ => {}
        }
        if !found_pts {
            extend_u_from_aabb_world(&edge.aabb_mm, frame, u_vec, u_min, u_max);
        }
    }
}

fn extend_u_from_face(
    doc: &CadDocument,
    fid: FaceId,
    frame: &RigidFrame3,
    u_vec: DVec3,
    u_min: &mut f64,
    u_max: &mut f64,
) {
    let mut found_pts = false;
    for tri in &doc.render_mesh.pick_triangles {
        if tri.face_id == fid {
            for p in [tri.v0_mm, tri.v1_mm, tri.v2_mm] {
                let u = frame.world_to_local_point(p).dot(u_vec);
                *u_min = (*u_min).min(u);
                *u_max = (*u_max).max(u);
            }
            found_pts = true;
        }
    }
    if let Some(face) = doc.face(fid) {
        if let Some(lid) = face.outer_loop
            && let Some(lp) = doc.brep_loop(lid)
        {
            for eu in &lp.edges {
                extend_u_from_edge(doc, eu.edge_id, frame, u_vec, u_min, u_max);
                found_pts = true;
            }
        }
        for &ilid in &face.inner_loops {
            if let Some(lp) = doc.brep_loop(ilid) {
                for eu in &lp.edges {
                    extend_u_from_edge(doc, eu.edge_id, frame, u_vec, u_min, u_max);
                    found_pts = true;
                }
            }
        }
        if !found_pts {
            extend_u_from_aabb_world(&face.aabb_mm, frame, u_vec, u_min, u_max);
        }
    }
}

/// Projects all selected 3D B-Rep faces/loops/chains/holes and extracted 2D sketch features
/// onto the rotary axis unit vector `u_vec` to obtain `[sel_u_min, sel_u_max]` (`mm`).
///
/// Returns `None` if no selections are present or none project to finite coordinates.
pub fn project_selections_onto_rotary_axis(
    doc: Option<&CadDocument>,
    frame: &RigidFrame3,
    selections: &[GeometrySelectionItem],
    extracted_selection: Option<&ExtractedSelectionSummary>,
    rot: &ResolvedRotaryConfig,
) -> Option<(f64, f64)> {
    let (_, _, u_vec) = rot.axis.basis();
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;

    let extend_xy_box = |box2: chip_math::Aabb2, u_min: &mut f64, u_max: &mut f64| {
        if box2.is_valid() {
            for x in [box2.min.x, box2.max.x] {
                for y in [box2.min.y, box2.max.y] {
                    let u = DVec3::new(x, y, 0.0).dot(u_vec);
                    *u_min = (*u_min).min(u);
                    *u_max = (*u_max).max(u);
                }
            }
        }
    };

    if let Some(summary) = extracted_selection {
        for feat in &summary.features {
            match feat {
                ExtractedFeature25D::PlanarFace {
                    outer_boundary_wcs,
                    inner_islands_wcs,
                    ..
                } => {
                    extend_xy_box(outer_boundary_wcs.exact_aabb(), &mut u_min, &mut u_max);
                    for isl in inner_islands_wcs {
                        extend_xy_box(isl.exact_aabb(), &mut u_min, &mut u_max);
                    }
                }
                ExtractedFeature25D::Contour { polyline_wcs, .. } => {
                    extend_xy_box(polyline_wcs.exact_aabb(), &mut u_min, &mut u_max);
                }
                ExtractedFeature25D::ModeledChamfer {
                    upper_polyline_wcs,
                    lower_polyline_wcs,
                    ..
                } => {
                    extend_xy_box(upper_polyline_wcs.exact_aabb(), &mut u_min, &mut u_max);
                    extend_xy_box(lower_polyline_wcs.exact_aabb(), &mut u_min, &mut u_max);
                }
                ExtractedFeature25D::CylindricalHole {
                    center_wcs,
                    diameter_mm,
                    ..
                } => {
                    let r = 0.5 * diameter_mm.abs();
                    let cu = DVec3::new(center_wcs.x, center_wcs.y, 0.0).dot(u_vec);
                    u_min = u_min.min(cu - r);
                    u_max = u_max.max(cu + r);
                }
            }
        }
    }

    if let Some(cad) = doc {
        for item in selections {
            match item {
                GeometrySelectionItem::PlanarFace { binding }
                | GeometrySelectionItem::CylindricalHole { binding } => {
                    let fid = FaceId(binding.last_index);
                    if cad.face(fid).is_some() {
                        extend_u_from_face(cad, fid, frame, u_vec, &mut u_min, &mut u_max);
                    } else {
                        extend_u_from_aabb_world(
                            &binding.signature.aabb_mm,
                            frame,
                            u_vec,
                            &mut u_min,
                            &mut u_max,
                        );
                    }
                }
                GeometrySelectionItem::ClosedLoop { binding, .. } => {
                    let lid = LoopId(binding.last_index);
                    if let Some(lp) = cad.brep_loop(lid) {
                        for eu in &lp.edges {
                            extend_u_from_edge(
                                cad, eu.edge_id, frame, u_vec, &mut u_min, &mut u_max,
                            );
                        }
                    } else {
                        extend_u_from_aabb_world(
                            &binding.signature.aabb_mm,
                            frame,
                            u_vec,
                            &mut u_min,
                            &mut u_max,
                        );
                    }
                }
                GeometrySelectionItem::OpenChain {
                    start_edge,
                    resolved_edges,
                    ..
                } => {
                    if resolved_edges.is_empty() {
                        let eid = EdgeId(start_edge.last_index);
                        if cad.edge(eid).is_some() {
                            extend_u_from_edge(cad, eid, frame, u_vec, &mut u_min, &mut u_max);
                        } else {
                            extend_u_from_aabb_world(
                                &start_edge.signature.aabb_mm,
                                frame,
                                u_vec,
                                &mut u_min,
                                &mut u_max,
                            );
                        }
                    } else {
                        for eu in resolved_edges {
                            extend_u_from_edge(
                                cad, eu.edge_id, frame, u_vec, &mut u_min, &mut u_max,
                            );
                        }
                    }
                }
                GeometrySelectionItem::SketchRegion { .. }
                | GeometrySelectionItem::SketchEntity { .. }
                | GeometrySelectionItem::SketchLayerAll { .. } => {
                    // Handled via `extracted_selection` above.
                }
            }
        }
    }

    if u_min.is_finite() && u_max.is_finite() {
        Some((u_min, u_max))
    } else {
        None
    }
}

/// Resolves the complete 4th-axis rotary machining window (`-U`/`+U` axial bounds, tool
/// containment, signed margins, mandatory hard chuck keep-out guard, angular sector, and
/// radial floor/ceiling envelope) for [`crate::strategies::rotary2linear`] and
/// [`crate::strategies::rotary4axis_sim`].
pub fn resolve_rotary_window(
    input: &RotaryWindowInput<'_>,
) -> Result<ResolvedRotaryWindow, CamError> {
    let rot = input.rot;
    let boundary = input.boundary;
    let (_, _, u_vec) = rot.axis.basis();

    let mut mesh_u_min = f64::INFINITY;
    let mut mesh_u_max = f64::NEG_INFINITY;
    for &v in &input.base_mesh.vertices {
        let u = v.dot(u_vec);
        mesh_u_min = mesh_u_min.min(u);
        mesh_u_max = mesh_u_max.max(u);
    }

    let model_nominal = if mesh_u_min.is_finite() && mesh_u_max.is_finite() {
        (
            mesh_u_min.max(rot.stock_u_min),
            mesh_u_max.min(rot.stock_u_max),
        )
    } else {
        (rot.stock_u_min, rot.stock_u_max)
    };

    let mut warnings = Vec::new();
    let (nominal_u_min, nominal_u_max) = match boundary.axial_boundary_mode {
        RotaryAxialBoundaryMode::StockBounds => (rot.stock_u_min, rot.stock_u_max),
        RotaryAxialBoundaryMode::ModelBounds => model_nominal,
        RotaryAxialBoundaryMode::SelectedGeometry => {
            if let Some((sel_min, sel_max)) = project_selections_onto_rotary_axis(
                input.doc,
                input.frame,
                input.selections,
                input.extracted_selection,
                rot,
            ) {
                (sel_min, sel_max)
            } else {
                warnings.push(
                    "Rotary axial boundary mode 'Selected Geometry' has no projectable selections; falling back to Model Bounds."
                        .to_string(),
                );
                model_nominal
            }
        }
        RotaryAxialBoundaryMode::ExplicitRange => {
            let e_start = boundary.explicit_u_start.value_mm;
            let e_end = boundary.explicit_u_end.value_mm;
            if !e_start.is_finite() {
                return Err(CamError::InvalidParameter {
                    param: "explicit_u_start",
                    value: e_start,
                    reason: "must be finite",
                });
            }
            if !e_end.is_finite() {
                return Err(CamError::InvalidParameter {
                    param: "explicit_u_end",
                    value: e_end,
                    reason: "must be finite",
                });
            }
            if e_start > e_end + 1e-9 {
                warnings.push(format!(
                    "Explicit rotary axial range start ({e_start:.2} mm) > end ({e_end:.2} mm); normalized to [{e_end:.2}, {e_start:.2}] mm."
                ));
            }
            (e_start.min(e_end), e_start.max(e_end))
        }
    };

    let margin_start = boundary.axial_margin_start.value_mm;
    let margin_end = boundary.axial_margin_end.value_mm;
    if !margin_start.is_finite() {
        return Err(CamError::InvalidParameter {
            param: "axial_margin_start",
            value: margin_start,
            reason: "must be finite",
        });
    }
    if !margin_end.is_finite() {
        return Err(CamError::InvalidParameter {
            param: "axial_margin_end",
            value: margin_end,
            reason: "must be finite",
        });
    }

    let tool_d = input.tool_diameter_mm.max(0.1);
    let tool_r = 0.5 * tool_d;

    // 1. Apply axial tool containment relative to nominal [-U, +U] ends
    let (mut u_start, mut u_end) = match boundary.axial_containment {
        ToolContainmentMode::ToolInsideBoundary => (nominal_u_min + tool_r, nominal_u_max - tool_r),
        ToolContainmentMode::ToolCenterOnBoundary => (nominal_u_min, nominal_u_max),
        ToolContainmentMode::ToolOutsideBoundary => {
            (nominal_u_min - tool_r, nominal_u_max + tool_r)
        }
    };

    // 2. Apply signed margins (> 0 trims inward, < 0 extends outward past boundary)
    u_start += margin_start;
    u_end -= margin_end;

    // 3. Enforce mandatory hard rotary chuck keep-out guard AFTER containment and margins
    let chuck_guard_u = rotary_chuck_guard_u(rot, tool_d);
    let mut clamped_by_chuck = false;
    if rot.chuck_end_positive {
        if u_end > chuck_guard_u + 1e-9 {
            let requested = u_end;
            u_end = chuck_guard_u;
            clamped_by_chuck = true;
            warnings.push(format!(
                "Rotary axial +U limit ({requested:.2} mm) clamped to {chuck_guard_u:.2} mm by mandatory chuck keep-out guard (tool radius + {ROTARY_CHUCK_GUARD_CLEARANCE_MM:.1} mm)."
            ));
        }
    } else if u_start < chuck_guard_u - 1e-9 {
        let requested = u_start;
        u_start = chuck_guard_u;
        clamped_by_chuck = true;
        warnings.push(format!(
            "Rotary axial -U limit ({requested:.2} mm) clamped to {chuck_guard_u:.2} mm by mandatory chuck keep-out guard (tool radius + {ROTARY_CHUCK_GUARD_CLEARANCE_MM:.1} mm)."
        ));
    }

    if !u_start.is_finite() || !u_end.is_finite() || u_end <= u_start + 1e-3 {
        return Err(CamError::InvalidRotarySetup {
            op_name: input.op_name.to_string(),
            reason: format!(
                "Axial machining window [{u_start:.2}, {u_end:.2}] mm is empty after containment, margins, and chuck clearance"
            ),
        });
    }

    let start_angle_deg = boundary.start_angle_deg;
    let end_angle_deg = boundary.end_angle_deg;
    if !start_angle_deg.is_finite()
        || !end_angle_deg.is_finite()
        || end_angle_deg <= start_angle_deg + 1e-3
    {
        return Err(CamError::InvalidParameter {
            param: "end_angle_deg",
            value: end_angle_deg,
            reason: "must be finite and greater than start_angle_deg",
        });
    }

    let r_floor = (input.bottom_z_wcs - rot.center_wcs.z).max(0.0);
    let stock_top_z = rot.center_wcs.z + rot.stock_outer_radius_mm;
    let r_ceiling = if input.top_z_wcs < stock_top_z - 1e-6 {
        (input.top_z_wcs - rot.center_wcs.z).clamp(r_floor, rot.stock_outer_radius_mm.max(r_floor))
    } else {
        rot.stock_outer_radius_mm.max(r_floor)
    };

    Ok(ResolvedRotaryWindow {
        nominal_u_min,
        nominal_u_max,
        u_start,
        u_end,
        start_angle_deg,
        end_angle_deg,
        r_floor,
        r_ceiling,
        chuck_guard_u,
        clamped_by_chuck,
        warnings,
    })
}

/// Validates that every motion segment of `tp` (and any nested clearing stage) respects the
/// mandatory rotary chuck axial keep-out plane (`rotary_chuck_guard_u`), returning a structured
/// [`CamError::InvalidRotarySetup`] if any move intrudes into the chuck keep-out zone.
pub fn validate_toolpath_rotary_chuck_clearance(
    tp: &GeneratedToolpath,
    rot: &ResolvedRotaryConfig,
    tool_diameter_mm: f64,
    clearing_tool_diameter_mm: Option<f64>,
    op_name: &str,
) -> Result<(), CamError> {
    if rot.axis == RotaryAxisKind::ZAxis || (rot.chuck_u_max - rot.chuck_u_min).abs() <= 1e-6 {
        return Ok(());
    }
    let guard_u = rotary_chuck_guard_u(rot, tool_diameter_mm);
    let (_, _, u_vec) = rot.axis.basis();
    let chuck_top_z = rot.center_wcs.z + rot.chuck_radius_mm + ROTARY_CHUCK_GUARD_CLEARANCE_MM;
    let mut worst_violation: Option<f64> = None;

    for seg in &tp.segments_wcs {
        let is_rapid = matches!(seg.motion, SegmentMotion3D::Rapid);
        let min_z = seg.start_wcs.z.min(seg.end_wcs.z);
        if is_rapid && min_z > chuck_top_z {
            continue;
        }
        let u0 = seg.start_wcs.dot(u_vec);
        let u1 = seg.end_wcs.dot(u_vec);
        if rot.chuck_end_positive {
            let u_hi = u0.max(u1);
            if u_hi > guard_u + 1e-4 {
                worst_violation = Some(worst_violation.map_or(u_hi, |w| w.max(u_hi)));
            }
        } else {
            let u_lo = u0.min(u1);
            if u_lo < guard_u - 1e-4 {
                worst_violation = Some(worst_violation.map_or(u_lo, |w| w.min(u_lo)));
            }
        }
    }

    if let Some(offending_u) = worst_violation {
        let (side_label, cmp_label) = if rot.chuck_end_positive {
            ("+U", "<=")
        } else {
            ("-U", ">=")
        };
        return Err(CamError::InvalidRotarySetup {
            op_name: op_name.to_string(),
            reason: format!(
                "Toolpath reaches U = {offending_u:.2} mm and intrudes into the {side_label} rotary chuck keep-out zone (must stay {cmp_label} {guard_u:.2} mm for Ø{:.2} mm cutter + {ROTARY_CHUCK_GUARD_CLEARANCE_MM:.1} mm clearance)",
                tool_diameter_mm.max(0.1)
            ),
        });
    }

    if let Some(cs) = &tp.clearing_stage {
        let clear_d = clearing_tool_diameter_mm.unwrap_or(tool_diameter_mm);
        validate_toolpath_rotary_chuck_clearance(&cs.toolpath, rot, clear_d, None, op_name)?;
    }

    Ok(())
}
