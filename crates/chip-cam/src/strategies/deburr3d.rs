//! **3D Edge Deburring & Non-Planar Chamfering** (`OperationKind::Deburr3D`).
//!
//! Extracts exact `f64` 3D B-Rep edge chains and adjacent-face outward normals `(n1, n2)` in Setup
//! WCS, solves cutter-specific 3D contact kinematics (`ChamferMill`, `BallEndmill`, and
//! `LollipopEndmill`, including underside back-chamfering on downward-facing overhang rims),
//! trims colliding ends/sub-segments away from neighboring vertical walls, stepped bosses, and
//! inside corners using `chip-bvh` [`FlatBvh`] with binary boundary refinement (`<= 0.02 mm`),
//! and emits smooth tangent arc lead-in/lead-out transitions with verified safe vertical plunges.

use std::collections::{HashMap, HashSet};

use chip_bvh::TriMesh3D;
use chip_cad::{
    BRepEdge, BRepFace, CadDocument, CurveKind, EdgeId, FaceId, LoopId, OrientedEdgeUse,
    SurfaceKind, group_coaxial_holes,
};
use chip_math::{Aabb3, COINCIDENCE_TOL_MM, DVec2, DVec3, RigidFrame3};
use chip_tools::{AnalyticCutterEnvelope, CutterEnvelopeShape, CutterGeometry};

use crate::{
    cl_data::MotionRole,
    params::Deburr3DParams,
    selection::GeometrySelectionItem,
    strategies::StrategyExecContext,
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

/// Single exact `f64` 3D sample along a B-Rep edge in Setup WCS coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeburrSample3D {
    pub pos_wcs: DVec3,
    pub tangent: DVec3,
    pub n1: DVec3,
    pub n2: DVec3,
    pub bisector_out: DVec3,
    pub is_underside: bool,
    pub face1: FaceId,
    pub face2: FaceId,
}

/// Exact 3D cutter contact solution at a [`DeburrSample3D`] station.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeburrContactSolution {
    /// Programmed cutter tip position `(X, Y, Z)` in Setup WCS millimeters.
    pub tip_wcs: DVec3,
    /// Point on the intentional bevel surface in Setup WCS.
    pub bevel_point_wcs: DVec3,
    /// Unit outward normal of the bevel plane cut by the tool at this station.
    pub bevel_normal_wcs: DVec3,
    /// Horizontal unit direction pointing from the edge outward toward the cutter axis.
    pub outward_xy: DVec2,
    /// Maximum local tangent contact chord (`mm`) along the edge for bevel-wedge exemption.
    pub contact_chord_mm: f64,
}

/// Extracted continuous chain of 3D edge samples in Setup WCS.
#[derive(Debug, Clone)]
struct DeburrEdgeChain {
    samples: Vec<DeburrSample3D>,
    closed: bool,
    is_underside: bool,
    user_reversed: bool,
}

/// Pre-indexed target B-Rep triangle lookup by `FaceId` (world coordinates) for exact normal
/// fallback and geometric convexity checks.
struct FaceTriLookup<'a> {
    by_face: HashMap<FaceId, Vec<usize>>,
    doc: &'a CadDocument,
}

impl<'a> FaceTriLookup<'a> {
    fn build(doc: &'a CadDocument) -> Self {
        let mut by_face: HashMap<FaceId, Vec<usize>> = HashMap::new();
        for (idx, tri) in doc.render_mesh.pick_triangles.iter().enumerate() {
            by_face.entry(tri.face_id).or_default().push(idx);
        }
        Self { by_face, doc }
    }

    /// Evaluates the exact outward unit normal of `face_id` at world point `p_world`.
    fn outward_normal_world(&self, face_id: FaceId, p_world: DVec3) -> Option<DVec3> {
        let face = self.doc.face(face_id)?;
        match face.surface {
            SurfaceKind::Plane { normal, .. } => {
                let n = normal.normalize_or_zero();
                (n.length_squared() > 0.5).then_some(n)
            }
            SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                is_internal,
                ..
            } => {
                let a = axis_dir.normalize_or_zero();
                let v = p_world - axis_origin_mm;
                let radial = (v - a * v.dot(a)).normalize_or_zero();
                if radial.length_squared() < 0.5 {
                    return self.nearest_triangle_normal_world(face_id, p_world);
                }
                Some(if is_internal { -radial } else { radial })
            }
            SurfaceKind::Cone {
                apex_mm,
                axis_dir,
                half_angle_rad,
            } => {
                let a = axis_dir.normalize_or_zero();
                let v = p_world - apex_mm;
                let radial = (v - a * v.dot(a)).normalize_or_zero();
                if radial.length_squared() < 0.5 {
                    return self.nearest_triangle_normal_world(face_id, p_world);
                }
                let axial_sign = if v.dot(a) >= 0.0 { 1.0 } else { -1.0 };
                let (sin_h, cos_h) = half_angle_rad.sin_cos();
                let mut n = (radial * cos_h - a * (axial_sign * sin_h)).normalize_or_zero();
                if let Some(tri_n) = self.nearest_triangle_normal_world(face_id, p_world)
                    && n.dot(tri_n) < 0.0
                {
                    n = -n;
                }
                (n.length_squared() > 0.5).then_some(n)
            }
            SurfaceKind::Torus {
                center_mm,
                axis_dir,
                major_radius_mm,
                ..
            } => {
                let a = axis_dir.normalize_or_zero();
                let v = p_world - center_mm;
                let proj = (v - a * v.dot(a)).normalize_or_zero();
                if proj.length_squared() < 0.5 {
                    return self.nearest_triangle_normal_world(face_id, p_world);
                }
                let major_pt = center_mm + proj * major_radius_mm;
                let mut n = (p_world - major_pt).normalize_or_zero();
                if let Some(tri_n) = self.nearest_triangle_normal_world(face_id, p_world)
                    && n.dot(tri_n) < 0.0
                {
                    n = -n;
                }
                (n.length_squared() > 0.5).then_some(n)
            }
            SurfaceKind::OtherNurbs => self.nearest_triangle_normal_world(face_id, p_world),
        }
    }

    fn nearest_triangle_normal_world(&self, face_id: FaceId, p_world: DVec3) -> Option<DVec3> {
        let tris = self.by_face.get(&face_id)?;
        let mut best_d2 = f64::INFINITY;
        let mut best_n = DVec3::ZERO;
        for &idx in tris {
            let t = self.doc.render_mesh.pick_triangles[idx];
            let cp = closest_point_on_triangle_3d(p_world, t.v0_mm, t.v1_mm, t.v2_mm);
            let d2 = cp.distance_squared(p_world);
            if d2 < best_d2 {
                let n = (t.v1_mm - t.v0_mm)
                    .cross(t.v2_mm - t.v0_mm)
                    .normalize_or_zero();
                if n.length_squared() > 0.5 {
                    best_d2 = d2;
                    best_n = n;
                }
            }
        }
        (best_n.length_squared() > 0.5).then_some(best_n)
    }

    /// Returns a world-space point strictly inside `face_id` near `p_world` (displaced away from
    /// the edge tangent `edge_tan_world`) to determine exact edge convexity vs. concavity.
    fn interior_witness_point_world(
        &self,
        face_id: FaceId,
        p_world: DVec3,
        edge_tan_world: DVec3,
    ) -> Option<DVec3> {
        let tris = self.by_face.get(&face_id)?;
        let mut best_score = f64::INFINITY;
        let mut best_q = None;

        for &idx in tris {
            let t = self.doc.render_mesh.pick_triangles[idx];
            let cp = closest_point_on_triangle_3d(p_world, t.v0_mm, t.v1_mm, t.v2_mm);
            let dist_to_edge_pt = cp.distance(p_world);
            let centroid = (t.v0_mm + t.v1_mm + t.v2_mm) / 3.0;
            let delta = centroid - p_world;
            let perp = delta - edge_tan_world * delta.dot(edge_tan_world);
            if perp.length_squared() < 1e-10 {
                continue;
            }
            if dist_to_edge_pt < best_score {
                best_score = dist_to_edge_pt;
                best_q = Some(centroid);
            }
        }
        best_q
    }
}

/// Solves exact 3D cutter tip placement (`tip_wcs`) for a single [`DeburrSample3D`] station,
/// returning `None` if the cutter cannot physically engage the edge cleanly (e.g., excessive slope
/// for a Chamfer Mill or insufficient relieved-neck clearance for a Lollipop underside pass).
pub fn solve_deburr_tool_tip(
    sample: &DeburrSample3D,
    tool_geom: &CutterGeometry,
    chamfer_width_mm: f64,
    tip_offset_mm: f64,
    wall_clearance_mm: f64,
    allow_underside_back_chamfer: bool,
) -> Option<DeburrContactSolution> {
    if !chamfer_width_mm.is_finite() || chamfer_width_mm <= 1e-6 {
        return None;
    }
    let b = sample.bisector_out.normalize_or_zero();
    if b.length_squared() < 0.5 {
        return None;
    }
    let cos_phi = b.dot(sample.n1).clamp(0.05, 0.9999);
    let sin_phi = (1.0 - cos_phi * cos_phi).max(0.0).sqrt();
    if sin_phi < 1e-4 {
        return None;
    }

    let b_xy = DVec2::new(b.x, b.y);
    let b_xy_len = b_xy.length();
    if b_xy_len < 1e-4 {
        return None;
    }
    let out_xy = b_xy / b_xy_len;

    match *tool_geom {
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => {
            if sample.is_underside || b.z <= 0.05 {
                return None;
            }
            let alpha = (0.5 * included_angle_deg).clamp(5.0, 85.0).to_radians();
            let (sin_a, cos_a) = alpha.sin_cos();
            let tan_a = (sin_a / cos_a).max(1e-6);
            let r_upper = 0.5 * diameter_mm;
            let r_tip = 0.5 * tip_diameter_mm;

            // Outward material normal of the conical flank facing `out_xy`:
            // n_cone = cos(alpha) * out_xy + sin(alpha) * +Z
            let n_cone = DVec3::new(cos_a * out_xy.x, cos_a * out_xy.y, sin_a).normalize_or_zero();

            // Unit in-face directions pointing INTO face 1 and face 2 away from the edge
            let u1 = inward_face_dir(sample.n1, sample.n2, sample.tangent)?;
            let u2 = inward_face_dir(sample.n2, sample.n1, sample.tangent)?;

            let proj1 = -(u1.dot(n_cone));
            let proj2 = -(u2.dot(n_cone));
            if proj1 <= 0.08 || proj2 <= 0.08 {
                return None;
            }

            // Cap bevel depth so neither adjacent face leg exceeds `chamfer_width_mm`.
            let d_cone = chamfer_width_mm * proj1.min(proj2);
            let s2 = d_cone / proj2;

            let p_bevel = sample.pos_wcs - n_cone * d_cone;
            // Lower bevel edge along the steeper/side face (the one with smaller Z along u)
            let p_f1 = sample.pos_wcs + u1 * (d_cone / proj1);
            let p_f2 = sample.pos_wcs + u2 * s2;
            let lower_z = p_f1.z.min(p_f2.z);
            let upper_z = p_f1.z.max(p_f2.z);

            let dz_below_bevel = (p_bevel.z - lower_z).max(0.0);
            let dz_contact = tip_offset_mm.max(0.0) + dz_below_bevel;
            let dz_top = tip_offset_mm.max(0.0) + (upper_z - lower_z).max(0.0);
            let r_contact = r_tip + dz_contact * tan_a;
            let r_top = r_tip + dz_top * tan_a;
            if r_top > r_upper + 1e-3 {
                return None;
            }

            let tip_wcs = DVec3::new(
                p_bevel.x + out_xy.x * r_contact,
                p_bevel.y + out_xy.y * r_contact,
                p_bevel.z - dz_contact,
            );
            let contact_chord_mm = (2.0 * r_top * chamfer_width_mm)
                .max(0.0)
                .sqrt()
                .max(chamfer_width_mm * 1.5);

            Some(DeburrContactSolution {
                tip_wcs,
                bevel_point_wcs: p_bevel,
                bevel_normal_wcs: n_cone,
                outward_xy: out_xy,
                contact_chord_mm,
            })
        }
        CutterGeometry::BallEndmill { diameter_mm, .. } => {
            if sample.is_underside || b.z <= 0.05 {
                return None;
            }
            let r = 0.5 * diameter_mm;
            let d_bevel = chamfer_width_mm * sin_phi;
            if d_bevel >= r * 0.85 {
                return None;
            }
            let p_bevel = sample.pos_wcs - b * d_bevel;
            let p_center = p_bevel + b * r;
            let tip_wcs = DVec3::new(p_center.x, p_center.y, p_center.z - r);
            let contact_chord_mm = (2.0 * r * d_bevel).max(0.0).sqrt().max(chamfer_width_mm);

            Some(DeburrContactSolution {
                tip_wcs,
                bevel_point_wcs: p_bevel,
                bevel_normal_wcs: b,
                outward_xy: out_xy,
                contact_chord_mm,
            })
        }
        CutterGeometry::LollipopEndmill {
            diameter_mm,
            neck_diameter_mm,
            ..
        } => {
            if sample.is_underside && !allow_underside_back_chamfer {
                return None;
            }
            if !sample.is_underside && b.z <= 0.05 {
                return None;
            }
            let r = 0.5 * diameter_mm;
            let r_neck = 0.5 * neck_diameter_mm;
            let d_bevel = chamfer_width_mm * sin_phi;
            if d_bevel >= r * 0.85 {
                return None;
            }

            let p_bevel = sample.pos_wcs - b * d_bevel;
            let p_center = p_bevel + b * r;
            let tip_wcs = DVec3::new(p_center.x, p_center.y, p_center.z - r);

            if sample.is_underside {
                // Upper-hemisphere tangent contact verification:
                // 1) Contact point must sit strictly below the sphere-to-neck join height!
                let z_join_above_center = (r * r - r_neck * r_neck).max(0.0).sqrt();
                let dz_contact_above_center = p_bevel.z - p_center.z;
                if dz_contact_above_center >= z_join_above_center - 1e-3 {
                    return None;
                }
                // 2) Relieved neck must clear the overhang outer wall horizontally with `wall_clearance_mm`!
                let horiz_reach =
                    (p_center.x - sample.pos_wcs.x).hypot(p_center.y - sample.pos_wcs.y);
                if horiz_reach < r_neck + wall_clearance_mm.max(0.0) - 1e-6 {
                    return None;
                }
            }

            let contact_chord_mm = (2.0 * r * d_bevel).max(0.0).sqrt().max(chamfer_width_mm);
            Some(DeburrContactSolution {
                tip_wcs,
                bevel_point_wcs: p_bevel,
                bevel_normal_wcs: b,
                outward_xy: out_xy,
                contact_chord_mm,
            })
        }
        _ => None,
    }
}

/// Computes the unit direction lying in `face_a` (normal `n_a`) perpendicular to edge tangent `t`,
/// pointing INTO the solid side of `face_b` (so `u · n_b < 0` on a convex edge).
fn inward_face_dir(n_a: DVec3, n_b: DVec3, t: DVec3) -> Option<DVec3> {
    let mut u = n_a.cross(t).normalize_or_zero();
    if u.length_squared() < 0.25 {
        // Fallback: project `-n_b` onto plane perpendicular to `n_a`
        u = (-n_b + n_a * n_b.dot(n_a)).normalize_or_zero();
    }
    if u.length_squared() < 0.25 {
        return None;
    }
    if u.dot(n_b) > 0.0 {
        u = -u;
    }
    Some(u)
}

/// Generates **3D Edge Deburring & Non-Planar Chamfering** (`OperationKind::Deburr3D`) toolpaths.
pub fn generate_deburr3d(
    ctx: &StrategyExecContext<'_>,
    params: &Deburr3DParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;
    if !ctx.doc.has_3d_geometry() {
        return Err(CamError::EmptyCadDocument);
    }

    // Validate parameters & tool compatibility
    let chamfer_w = params.chamfer_width.value_mm;
    if !chamfer_w.is_finite() || chamfer_w <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.chamfer_width",
            value: chamfer_w,
            reason: "must be finite and > 0.0001 mm",
        });
    }
    let tip_offset = params.tip_offset.value_mm;
    if !tip_offset.is_finite() || tip_offset < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.tip_offset",
            value: tip_offset,
            reason: "must be finite and >= 0.0 mm",
        });
    }
    let wall_clearance = params.wall_clearance.value_mm;
    if !wall_clearance.is_finite() || wall_clearance < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.wall_clearance",
            value: wall_clearance,
            reason: "must be finite and >= 0.0 mm",
        });
    }
    let edge_extension = params.edge_extension.value_mm;
    if !edge_extension.is_finite() || edge_extension < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.edge_extension",
            value: edge_extension,
            reason: "must be finite and >= 0.0 mm",
        });
    }
    let chord_tol = params.chord_tolerance_mm;
    if !chord_tol.is_finite() || chord_tol <= 1e-5 {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.chord_tolerance_mm",
            value: chord_tol,
            reason: "must be finite and > 0.00001 mm",
        });
    }
    if !params.min_dihedral_angle_deg.is_finite()
        || params.min_dihedral_angle_deg <= 0.0
        || params.min_dihedral_angle_deg >= 179.0
    {
        return Err(CamError::InvalidParameter {
            param: "deburr3d.min_dihedral_angle_deg",
            value: params.min_dihedral_angle_deg,
            reason: "must be within (0°, 179°)",
        });
    }

    match ctx.tool.geometry {
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => {
            if params.allow_underside_back_chamfer {
                return Err(CamError::InvalidChamferGeometry {
                    reason: "Underside back-chamfering requires a Lollipop Endmill cutter."
                        .to_string(),
                });
            }
            let half_a = (0.5 * included_angle_deg).clamp(5.0, 85.0).to_radians();
            let max_flute_radial = 0.5 * (diameter_mm - tip_diameter_mm).max(0.0);
            if tip_offset * half_a.tan() + chamfer_w > max_flute_radial + 1e-3 {
                return Err(CamError::InvalidChamferGeometry {
                    reason: format!(
                        "chamfer width ({chamfer_w:.2} mm) + tip_offset radial shift ({:.2} mm) exceeds cutter cone radial span ({max_flute_radial:.2} mm)",
                        tip_offset * half_a.tan()
                    ),
                });
            }
        }
        CutterGeometry::BallEndmill { .. } => {
            if params.allow_underside_back_chamfer {
                return Err(CamError::InvalidChamferGeometry {
                    reason: "Underside back-chamfering requires a Lollipop Endmill cutter."
                        .to_string(),
                });
            }
        }
        CutterGeometry::LollipopEndmill { .. } => {}
        _ => {
            return Err(CamError::IncompatibleTool {
                op_name: ctx.op.name.clone(),
                op_kind: ctx.op.kind.label(),
                tool_name: ctx.tool.name.clone(),
                tool_kind: ctx.tool.geometry.kind_label(),
            });
        }
    }

    let lookup = FaceTriLookup::build(ctx.doc);
    let poly_by_edge: HashMap<EdgeId, &[DVec3]> = ctx
        .doc
        .render_mesh
        .edge_polylines
        .iter()
        .map(|ep| (ep.edge_id, ep.points_mm.as_slice()))
        .collect();

    // 1. Extract explicit or auto-detected 3D B-Rep edge chains in Setup WCS
    let use_auto_detect = params.auto_detect_edges || ctx.op.selections.is_empty();
    let mut chains = if use_auto_detect {
        auto_detect_sharp_edge_chains(ctx, params, &lookup, &poly_by_edge, chord_tol)?
    } else {
        extract_selected_edge_chains(ctx, params, &lookup, &poly_by_edge, chord_tol)?
    };

    if chains.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    // 2. Orient chains for Climb vs Conventional milling & apply optional open-edge extensions
    for chain in &mut chains {
        orient_chain_for_cut_direction(chain, params.climb_direction);
        if !chain.closed && edge_extension > 1e-4 {
            apply_open_chain_extension(chain, edge_extension);
        }
    }

    // 3. Build target B-Rep TriMesh3D + FlatBvh in Setup WCS for full-assembly collision trimming
    let target_mesh = ctx.doc.build_wcs_tri_mesh(&[], &ctx.setup.frame);
    let envelope = AnalyticCutterEnvelope::from_geometry(&ctx.tool.geometry);
    let stickout_mm = ctx
        .tool
        .stickout_length_mm
        .max(envelope.effective_reach_mm());

    // 4. Progressive width passes (+ optional spring pass)
    let n_passes = params.multiple_passes.clamp(1, 20);
    let mut pass_schedule: Vec<(f64, MotionRole)> = Vec::with_capacity(n_passes as usize + 1);
    for k in 1..=n_passes {
        let frac = (k as f64) / (n_passes as f64);
        pass_schedule.push((chamfer_w * frac, MotionRole::Cut));
    }
    if params.spring_pass {
        pass_schedule.push((chamfer_w, MotionRole::FinishCut));
    }

    let retract_z = ctx.safe_retract_z();
    let mut emitted_subchains = 0usize;
    let mut cand_buf = Vec::<usize>::with_capacity(64);

    for (w_pass, cut_role) in pass_schedule {
        ctx.check_cancel()?;
        for chain in &chains {
            ctx.check_cancel()?;

            let subchains = trim_chain_against_target(
                chain,
                ctx.tool,
                &envelope,
                stickout_mm,
                &target_mesh,
                w_pass,
                tip_offset,
                wall_clearance,
                params.allow_underside_back_chamfer,
                &mut cand_buf,
            );

            for sub in subchains {
                ctx.check_cancel()?;
                if emit_deburr_subchain(
                    ctx,
                    &sub,
                    &envelope,
                    stickout_mm,
                    &target_mesh,
                    wall_clearance,
                    retract_z,
                    cut_role,
                    &mut cand_buf,
                    builder,
                ) {
                    emitted_subchains += 1;
                }
            }
        }
    }

    if emitted_subchains == 0 {
        return Err(CamError::EmptyToolpath {
            op_name: ctx.op.name.clone(),
        });
    }

    Ok(())
}

/// Automatically detects all sharp convex 3D B-Rep edges accessible to the current tool and chains
/// connected edges into maximal continuous 3D paths.
fn auto_detect_sharp_edge_chains(
    ctx: &StrategyExecContext<'_>,
    params: &Deburr3DParams,
    lookup: &FaceTriLookup<'_>,
    poly_by_edge: &HashMap<EdgeId, &[DVec3]>,
    chord_tol_mm: f64,
) -> Result<Vec<DeburrEdgeChain>, CamError> {
    let cos_min_dihedral = params.min_dihedral_angle_deg.to_radians().cos();
    let allow_underside = params.allow_underside_back_chamfer
        && matches!(ctx.tool.geometry, CutterGeometry::LollipopEndmill { .. });
    let model_min_z = ctx.setup.model_aabb_wcs.min.z;

    let mut candidate_segments: Vec<Vec<DeburrSample3D>> = Vec::new();
    let mut closed_chains: Vec<DeburrEdgeChain> = Vec::new();

    for edge in &ctx.doc.edges {
        ctx.check_cancel()?;
        let Some((f1, f2)) = distinct_adjacent_face_pair(edge) else {
            continue;
        };
        let Some(samples) = sample_brep_edge_3d(
            ctx.doc,
            edge,
            true,
            f1,
            f2,
            &ctx.setup.frame,
            lookup,
            poly_by_edge.get(&edge.id).copied(),
            chord_tol_mm,
        ) else {
            continue;
        };
        if samples.len() < 2 {
            continue;
        }

        // Check convexity, dihedral angle, vertical filter, and top/underside accessibility
        let mid = &samples[samples.len() / 2];
        let mid_world = ctx.setup.frame.local_to_world_point(mid.pos_wcs);
        let tan_world = ctx.setup.frame.local_to_world_dir(mid.tangent);
        let n1_world = ctx.setup.frame.local_to_world_dir(mid.n1);
        let n2_world = ctx.setup.frame.local_to_world_dir(mid.n2);

        if !is_edge_convex_world(lookup, f1, f2, mid_world, tan_world, n1_world, n2_world) {
            continue;
        }

        // Dihedral angle check across samples (skip smooth tangent seams)
        let dot_n = mid.n1.dot(mid.n2).clamp(-1.0, 1.0);
        if dot_n >= cos_min_dihedral {
            continue;
        }

        // Exclude pure vertical edges when requested
        if params.exclude_vertical_edges && samples.iter().all(|s| s.tangent.z.abs() > 0.95) {
            continue;
        }

        // Accessibility classification (top vs underside)
        let is_underside = mid.is_underside;
        if is_underside {
            if !allow_underside {
                continue;
            }
            // Skip bottom resting edges flush with the part bottom (table/vise seating face)
            if samples.iter().all(|s| s.pos_wcs.z <= model_min_z + 0.05) {
                continue;
            }
            if DVec2::new(mid.bisector_out.x, mid.bisector_out.y).length() < 0.15 {
                continue;
            }
        } else if mid.bisector_out.z < 0.05 {
            continue;
        }

        // Respect Operation Z height bounds (`[bottom_z_wcs, top_z_wcs]`)
        let z_ok = samples.iter().any(|s| {
            s.pos_wcs.z >= ctx.heights.bottom_z_wcs - 0.05
                && s.pos_wcs.z <= ctx.heights.top_z_wcs + 0.05
        });
        if !z_ok {
            continue;
        }

        let is_closed = is_sample_list_closed(&samples);
        if is_closed {
            closed_chains.push(DeburrEdgeChain {
                samples,
                closed: true,
                is_underside,
                user_reversed: false,
            });
        } else {
            candidate_segments.push(samples);
        }
    }

    let mut out = closed_chains;
    out.extend(stitch_open_edge_segments(candidate_segments));
    Ok(out)
}

/// Extracts explicitly selected B-Rep loops, open chains, planar face rims, or hole rims into
/// 3D edge chains in Setup WCS.
fn extract_selected_edge_chains(
    ctx: &StrategyExecContext<'_>,
    params: &Deburr3DParams,
    lookup: &FaceTriLookup<'_>,
    poly_by_edge: &HashMap<EdgeId, &[DVec3]>,
    chord_tol_mm: f64,
) -> Result<Vec<DeburrEdgeChain>, CamError> {
    let mut chains = Vec::new();
    let allow_underside = params.allow_underside_back_chamfer
        && matches!(ctx.tool.geometry, CutterGeometry::LollipopEndmill { .. });

    for item in &ctx.op.selections {
        ctx.check_cancel()?;
        match item {
            GeometrySelectionItem::ClosedLoop {
                binding, reverse, ..
            } => {
                let lid = LoopId(binding.last_index);
                let lp = ctx
                    .doc
                    .brep_loop(lid)
                    .ok_or(CamError::MissingSelectedEntity {
                        index: binding.last_index,
                    })?;
                if let Some(ch) = build_chain_from_edge_uses(
                    ctx.doc,
                    &lp.edges,
                    true,
                    *reverse,
                    allow_underside,
                    &ctx.setup.frame,
                    lookup,
                    poly_by_edge,
                    chord_tol_mm,
                ) {
                    chains.push(ch);
                }
            }
            GeometrySelectionItem::OpenChain {
                resolved_edges,
                reverse,
                start_edge,
                ..
            } => {
                if resolved_edges.is_empty() {
                    return Err(CamError::MissingSelectedEntity {
                        index: start_edge.last_index,
                    });
                }
                if let Some(ch) = build_chain_from_edge_uses(
                    ctx.doc,
                    resolved_edges,
                    false,
                    *reverse,
                    allow_underside,
                    &ctx.setup.frame,
                    lookup,
                    poly_by_edge,
                    chord_tol_mm,
                ) {
                    chains.push(ch);
                }
            }
            GeometrySelectionItem::PlanarFace { binding } => {
                let fid = FaceId(binding.last_index);
                let face = ctx.doc.face(fid).ok_or(CamError::MissingSelectedEntity {
                    index: binding.last_index,
                })?;
                for lid in face.outer_loop.iter().chain(face.inner_loops.iter()) {
                    if let Some(lp) = ctx.doc.brep_loop(*lid)
                        && let Some(ch) = build_chain_from_edge_uses(
                            ctx.doc,
                            &lp.edges,
                            true,
                            false,
                            allow_underside,
                            &ctx.setup.frame,
                            lookup,
                            poly_by_edge,
                            chord_tol_mm,
                        )
                    {
                        chains.push(ch);
                    }
                }
            }
            GeometrySelectionItem::CylindricalHole { binding } => {
                let fid = FaceId(binding.last_index);
                let grouped = group_coaxial_holes(ctx.doc, &[fid], ctx.setup.frame.z_axis);
                let mut seen_edges = HashSet::new();
                let mut hole_segs = Vec::new();
                for h in grouped {
                    for gfid in h.faces {
                        let Some(face) = ctx.doc.face(gfid) else {
                            continue;
                        };
                        for lid in face.outer_loop.iter().chain(face.inner_loops.iter()) {
                            let Some(lp) = ctx.doc.brep_loop(*lid) else {
                                continue;
                            };
                            for u in &lp.edges {
                                if !seen_edges.insert(u.edge_id) {
                                    continue;
                                }
                                let Some(edge) = ctx.doc.edge(u.edge_id) else {
                                    continue;
                                };
                                let Some((f1, f2)) = distinct_adjacent_face_pair(edge) else {
                                    continue;
                                };
                                let Some(samples) = sample_brep_edge_3d(
                                    ctx.doc,
                                    edge,
                                    u.forward,
                                    f1,
                                    f2,
                                    &ctx.setup.frame,
                                    lookup,
                                    poly_by_edge.get(&edge.id).copied(),
                                    chord_tol_mm,
                                ) else {
                                    continue;
                                };
                                if samples.len() < 2 {
                                    continue;
                                }
                                let mid = &samples[samples.len() / 2];
                                if mid.is_underside && !allow_underside {
                                    continue;
                                }
                                if !mid.is_underside && mid.bisector_out.z < 0.05 {
                                    continue;
                                }
                                if is_sample_list_closed(&samples) {
                                    chains.push(DeburrEdgeChain {
                                        is_underside: mid.is_underside,
                                        samples,
                                        closed: true,
                                        user_reversed: false,
                                    });
                                } else {
                                    hole_segs.push(samples);
                                }
                            }
                        }
                    }
                }
                chains.extend(stitch_open_edge_segments(hole_segs));
            }
            GeometrySelectionItem::SketchRegion { .. }
            | GeometrySelectionItem::SketchEntity { .. }
            | GeometrySelectionItem::SketchLayerAll { .. } => {}
        }
    }

    Ok(chains)
}

#[allow(clippy::too_many_arguments)]
fn build_chain_from_edge_uses(
    doc: &CadDocument,
    uses: &[OrientedEdgeUse],
    hint_closed: bool,
    reverse: bool,
    allow_underside: bool,
    frame: &RigidFrame3,
    lookup: &FaceTriLookup<'_>,
    poly_by_edge: &HashMap<EdgeId, &[DVec3]>,
    chord_tol_mm: f64,
) -> Option<DeburrEdgeChain> {
    let mut combined: Vec<DeburrSample3D> = Vec::new();
    let mut underside_votes = 0usize;
    let mut total_edges = 0usize;

    for u in uses {
        let edge = doc.edge(u.edge_id)?;
        let (f1, f2) = distinct_adjacent_face_pair(edge)?;
        let samples = sample_brep_edge_3d(
            doc,
            edge,
            u.forward,
            f1,
            f2,
            frame,
            lookup,
            poly_by_edge.get(&edge.id).copied(),
            chord_tol_mm,
        )?;
        if samples.len() < 2 {
            continue;
        }
        let mid = &samples[samples.len() / 2];
        let mid_w = frame.local_to_world_point(mid.pos_wcs);
        let tan_w = frame.local_to_world_dir(mid.tangent);
        let n1_w = frame.local_to_world_dir(mid.n1);
        let n2_w = frame.local_to_world_dir(mid.n2);
        if !is_edge_convex_world(lookup, f1, f2, mid_w, tan_w, n1_w, n2_w) {
            continue;
        }
        if mid.is_underside {
            if !allow_underside {
                continue;
            }
            underside_votes += 1;
        }
        total_edges += 1;
        append_connected_samples(&mut combined, &samples);
    }

    if combined.len() < 2 || total_edges == 0 {
        return None;
    }

    let closed = hint_closed || is_sample_list_closed(&combined);
    let is_underside = underside_votes * 2 > total_edges;

    if reverse {
        reverse_sample_chain(&mut combined);
    }

    Some(DeburrEdgeChain {
        samples: combined,
        closed,
        is_underside,
        user_reversed: reverse,
    })
}

fn distinct_adjacent_face_pair(edge: &BRepEdge) -> Option<(FaceId, FaceId)> {
    let f1 = *edge.adjacent_faces.first()?;
    let f2 = edge.adjacent_faces.iter().copied().find(|&f| f != f1)?;
    Some((f1, f2))
}

fn is_edge_convex_world(
    lookup: &FaceTriLookup<'_>,
    f1: FaceId,
    f2: FaceId,
    p_world: DVec3,
    edge_tan_world: DVec3,
    n1_world: DVec3,
    n2_world: DVec3,
) -> bool {
    let Some(q1) = lookup.interior_witness_point_world(f1, p_world, edge_tan_world) else {
        return false;
    };
    let Some(q2) = lookup.interior_witness_point_world(f2, p_world, edge_tan_world) else {
        return false;
    };
    let v1 = (q1 - p_world).normalize_or_zero();
    let v2 = (q2 - p_world).normalize_or_zero();
    let s1 = v1.dot(n2_world);
    let s2 = v2.dot(n1_world);
    s1 < -1e-3 && s2 < -1e-3
}

/// Samples a single B-Rep edge in exact `f64` Setup WCS coordinates with adaptive chord-sagitta
/// subdivision (`<= chord_tol_mm`), evaluating exact adjacent-face normals `(n1, n2)` at every
/// sample point.
#[allow(clippy::too_many_arguments)]
fn sample_brep_edge_3d(
    doc: &CadDocument,
    edge: &BRepEdge,
    forward: bool,
    f1: FaceId,
    f2: FaceId,
    frame: &RigidFrame3,
    lookup: &FaceTriLookup<'_>,
    poly_pts: Option<&[DVec3]>,
    chord_tol_mm: f64,
) -> Option<Vec<DeburrSample3D>> {
    let tol = chord_tol_mm.clamp(0.0005, 0.25);
    let max_step_mm = 0.5_f64;

    let mut world_pts = match edge.curve {
        CurveKind::Line { start_mm, end_mm } => {
            let mut p0 = start_mm;
            let mut p1 = end_mm;
            if let Some(pts) = poly_pts
                && let Some(&first) = pts.first()
                && first.distance(end_mm) < first.distance(start_mm)
            {
                std::mem::swap(&mut p0, &mut p1);
            }
            let len = p0.distance(p1);
            if len < 1e-6 {
                return None;
            }
            let n = (len / max_step_mm).ceil().max(2.0) as usize;
            (0..=n)
                .map(|i| p0.lerp(p1, (i as f64) / (n as f64)))
                .collect::<Vec<_>>()
        }
        CurveKind::CircleArc {
            center_mm,
            normal,
            radius_mm,
            start_mm,
            end_mm,
            is_full_circle,
        } => sample_circle_arc_3d(
            center_mm,
            normal,
            radius_mm,
            start_mm,
            end_mm,
            is_full_circle,
            poly_pts,
            tol,
            max_step_mm,
        )?,
        CurveKind::EllipseArc | CurveKind::OtherNurbs => {
            let raw = poly_pts?;
            if raw.len() < 2 {
                return None;
            }
            refine_freeform_edge_3d(doc, f1, f2, raw, tol, max_step_mm)
        }
    };

    if !forward {
        world_pts.reverse();
    }
    if world_pts.len() < 2 {
        return None;
    }

    let n_pts = world_pts.len();
    let mut samples = Vec::with_capacity(n_pts);
    for i in 0..n_pts {
        let p_w = world_pts[i];
        let t_w = if i == 0 {
            (world_pts[1] - world_pts[0]).normalize_or_zero()
        } else if i + 1 == n_pts {
            (world_pts[n_pts - 1] - world_pts[n_pts - 2]).normalize_or_zero()
        } else {
            let t_prev = (world_pts[i] - world_pts[i - 1]).normalize_or_zero();
            let t_next = (world_pts[i + 1] - world_pts[i]).normalize_or_zero();
            (t_prev + t_next).normalize_or_zero()
        };
        if t_w.length_squared() < 0.25 {
            continue;
        }
        let n1_w = lookup.outward_normal_world(f1, p_w)?;
        let n2_w = lookup.outward_normal_world(f2, p_w)?;
        let pos_wcs = frame.world_to_local_point(p_w);
        let tangent = frame.world_to_local_dir(t_w).normalize_or_zero();
        let n1 = frame.world_to_local_dir(n1_w).normalize_or_zero();
        let n2 = frame.world_to_local_dir(n2_w).normalize_or_zero();
        let bisector_out = (n1 + n2).normalize_or_zero();
        if bisector_out.length_squared() < 0.25 {
            continue;
        }
        let is_underside = bisector_out.z < -0.15;
        samples.push(DeburrSample3D {
            pos_wcs,
            tangent,
            n1,
            n2,
            bisector_out,
            is_underside,
            face1: f1,
            face2: f2,
        });
    }

    (samples.len() >= 2).then_some(samples)
}

#[allow(clippy::too_many_arguments)]
fn sample_circle_arc_3d(
    center_mm: DVec3,
    normal: DVec3,
    radius_mm: f64,
    start_mm: DVec3,
    end_mm: DVec3,
    is_full_circle: bool,
    poly_pts: Option<&[DVec3]>,
    chord_tol_mm: f64,
    max_step_mm: f64,
) -> Option<Vec<DVec3>> {
    if !radius_mm.is_finite() || radius_mm <= 1e-5 {
        return None;
    }
    let n = normal.normalize_or_zero();
    if n.length_squared() < 0.5 {
        return None;
    }

    // Align start/end endpoints and sweep sense with the trimmed polyline when available
    let (p_start, p_end) = if is_full_circle {
        (start_mm, start_mm)
    } else if let Some(pts) = poly_pts
        && pts.len() >= 2
    {
        (pts[0], pts[pts.len() - 1])
    } else {
        (start_mm, end_mm)
    };

    let u = (p_start - center_mm - n * (p_start - center_mm).dot(n)).normalize_or_zero();
    if u.length_squared() < 0.5 {
        return None;
    }
    let mut v = n.cross(u).normalize_or_zero();

    // Determine exact sweep angle in `[0, 2π]` matching `poly_pts` midpoint if partial arc
    let sweep = if is_full_circle {
        if let Some(pts) = poly_pts
            && pts.len() >= 3
        {
            let d0 = (pts[1] - pts[0]).normalize_or_zero();
            if d0.dot(v) < 0.0 {
                v = -v;
            }
        }
        std::f64::consts::TAU
    } else {
        let r_end = (p_end - center_mm - n * (p_end - center_mm).dot(n)).normalize_or_zero();
        let mut ang_pos = r_end.dot(v).atan2(r_end.dot(u));
        if ang_pos <= 1e-9 {
            ang_pos += std::f64::consts::TAU;
        }
        if let Some(pts) = poly_pts
            && pts.len() >= 3
        {
            let mid_poly = pts[pts.len() / 2];
            let mid_pos =
                center_mm + (u * (0.5 * ang_pos).cos() + v * (0.5 * ang_pos).sin()) * radius_mm;
            let ang_neg = std::f64::consts::TAU - ang_pos;
            let mid_neg =
                center_mm + (u * (0.5 * ang_neg).cos() - v * (0.5 * ang_neg).sin()) * radius_mm;
            if mid_poly.distance_squared(mid_neg) < mid_poly.distance_squared(mid_pos) {
                v = -v;
                ang_neg
            } else {
                ang_pos
            }
        } else {
            ang_pos
        }
    };

    let d_theta_sag = if chord_tol_mm < radius_mm {
        2.0 * (1.0 - chord_tol_mm / radius_mm).clamp(-1.0, 1.0).acos()
    } else {
        0.25
    };
    let d_theta_len = (max_step_mm / radius_mm).clamp(0.01, 0.5);
    let d_theta = d_theta_sag.min(d_theta_len).max(1e-3);
    let n_steps = (sweep / d_theta).ceil().clamp(4.0, 2048.0) as usize;

    let mut pts = Vec::with_capacity(n_steps + 1);
    for i in 0..=n_steps {
        let ang = sweep * (i as f64) / (n_steps as f64);
        let (s, c) = ang.sin_cos();
        pts.push(center_mm + (u * c + v * s) * radius_mm);
    }
    if is_full_circle
        && let Some(&first) = pts.first()
        && let Some(last) = pts.last_mut()
    {
        *last = first;
    }
    Some(pts)
}

/// Refines a freeform/ellipse edge polyline to satisfy `chord_tol_mm` and `max_step_mm`.
/// When the two adjacent B-Rep faces are an analytic `Plane` and `Cylinder`, midpoints are
/// projected onto the exact `Plane ∩ Cylinder` intersection ellipse in `f64`!
fn refine_freeform_edge_3d(
    doc: &CadDocument,
    f1: FaceId,
    f2: FaceId,
    raw: &[DVec3],
    chord_tol_mm: f64,
    max_step_mm: f64,
) -> Vec<DVec3> {
    let plane_cyl = extract_plane_cylinder_pair(doc.face(f1), doc.face(f2));

    let mut work = raw.to_vec();
    if let Some((p_orig, p_norm, c_orig, c_axis, c_rad)) = plane_cyl {
        // Recursive midpoint subdivision projected onto exact `Plane ∩ Cylinder`!
        let mut out = Vec::with_capacity(work.len() * 4);
        if let Some(&first) = work.first() {
            out.push(project_onto_plane_cylinder(
                first, p_orig, p_norm, c_orig, c_axis, c_rad,
            ));
        }
        for seg in work.windows(2) {
            subdivide_plane_cylinder(
                seg[0],
                seg[1],
                p_orig,
                p_norm,
                c_orig,
                c_axis,
                c_rad,
                chord_tol_mm,
                max_step_mm,
                0,
                &mut out,
            );
        }
        return out;
    }

    // General NURBS / ellipse polyline: densify long chords to `<= max_step_mm`
    let mut out = Vec::with_capacity(work.len() * 2);
    if let Some(&first) = work.first() {
        out.push(first);
    }
    for seg in work.windows(2) {
        let len = seg[0].distance(seg[1]);
        if len < 1e-7 {
            continue;
        }
        let n = (len / max_step_mm).ceil().max(1.0) as usize;
        for k in 1..=n {
            out.push(seg[0].lerp(seg[1], (k as f64) / (n as f64)));
        }
    }
    work = out;
    work
}

fn extract_plane_cylinder_pair(
    fa: Option<&BRepFace>,
    fb: Option<&BRepFace>,
) -> Option<(DVec3, DVec3, DVec3, DVec3, f64)> {
    let (fa, fb) = (fa?, fb?);
    match (fa.surface, fb.surface) {
        (
            SurfaceKind::Plane { origin_mm, normal },
            SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                radius_mm,
                ..
            },
        )
        | (
            SurfaceKind::Cylinder {
                axis_origin_mm,
                axis_dir,
                radius_mm,
                ..
            },
            SurfaceKind::Plane { origin_mm, normal },
        ) => {
            let n = normal.normalize_or_zero();
            let a = axis_dir.normalize_or_zero();
            if n.dot(a).abs() > 0.05 && radius_mm > 1e-4 {
                Some((origin_mm, n, axis_origin_mm, a, radius_mm))
            } else {
                None
            }
        }
        _ => None,
    }
}

fn project_onto_plane_cylinder(
    pt: DVec3,
    plane_origin: DVec3,
    plane_normal: DVec3,
    cyl_origin: DVec3,
    cyl_axis: DVec3,
    cyl_radius: f64,
) -> DVec3 {
    // 1. Project radially onto exact cylinder surface at radius `cyl_radius`
    let v = pt - cyl_origin;
    let radial = v - cyl_axis * v.dot(cyl_axis);
    let r_dir = radial.normalize_or_zero();
    if r_dir.length_squared() < 0.5 {
        return pt;
    }
    let q_cyl = cyl_origin + r_dir * cyl_radius;
    // 2. Intersect cylinder generator line `q_cyl + t * cyl_axis` with `plane`
    let denom = cyl_axis.dot(plane_normal);
    if denom.abs() < 1e-8 {
        return pt;
    }
    let t = (plane_origin - q_cyl).dot(plane_normal) / denom;
    q_cyl + cyl_axis * t
}

#[allow(clippy::too_many_arguments)]
fn subdivide_plane_cylinder(
    a: DVec3,
    b: DVec3,
    plane_origin: DVec3,
    plane_normal: DVec3,
    cyl_origin: DVec3,
    cyl_axis: DVec3,
    cyl_radius: f64,
    chord_tol_mm: f64,
    max_step_mm: f64,
    depth: u32,
    out: &mut Vec<DVec3>,
) {
    let mid_lin = 0.5 * (a + b);
    let mid_exact = project_onto_plane_cylinder(
        mid_lin,
        plane_origin,
        plane_normal,
        cyl_origin,
        cyl_axis,
        cyl_radius,
    );
    let sag = mid_exact.distance(mid_lin);
    let len = a.distance(b);
    if depth < 10 && (sag > chord_tol_mm || len > max_step_mm) {
        subdivide_plane_cylinder(
            a,
            mid_exact,
            plane_origin,
            plane_normal,
            cyl_origin,
            cyl_axis,
            cyl_radius,
            chord_tol_mm,
            max_step_mm,
            depth + 1,
            out,
        );
        subdivide_plane_cylinder(
            mid_exact,
            b,
            plane_origin,
            plane_normal,
            cyl_origin,
            cyl_axis,
            cyl_radius,
            chord_tol_mm,
            max_step_mm,
            depth + 1,
            out,
        );
    } else {
        let b_exact = project_onto_plane_cylinder(
            b,
            plane_origin,
            plane_normal,
            cyl_origin,
            cyl_axis,
            cyl_radius,
        );
        out.push(b_exact);
    }
}

fn is_sample_list_closed(samples: &[DeburrSample3D]) -> bool {
    match (samples.first(), samples.last()) {
        (Some(a), Some(b)) => a.pos_wcs.distance(b.pos_wcs) <= COINCIDENCE_TOL_MM * 3.0,
        _ => false,
    }
}

fn append_connected_samples(dst: &mut Vec<DeburrSample3D>, src: &[DeburrSample3D]) {
    if src.is_empty() {
        return;
    }
    if let Some(last) = dst.last()
        && let Some(first) = src.first()
        && last.pos_wcs.distance(first.pos_wcs) <= COINCIDENCE_TOL_MM * 3.0
        && last.bisector_out.dot(first.bisector_out) >= 0.995
    {
        dst.extend_from_slice(&src[1..]);
    } else {
        dst.extend_from_slice(src);
    }
}

fn reverse_sample_chain(samples: &mut [DeburrSample3D]) {
    samples.reverse();
    for s in samples.iter_mut() {
        s.tangent = -s.tangent;
    }
}

/// Stitches open auto-detected 3D edge segments sharing endpoints (within coincidence tolerance)
/// into continuous maximal chains, preferring smooth tangent continuity and matching top/underside
/// classification.
fn stitch_open_edge_segments(mut segs: Vec<Vec<DeburrSample3D>>) -> Vec<DeburrEdgeChain> {
    let mut used = vec![false; segs.len()];
    let mut chains = Vec::new();
    let weld_tol = COINCIDENCE_TOL_MM * 4.0;

    for i in 0..segs.len() {
        if used[i] || segs[i].len() < 2 {
            continue;
        }
        used[i] = true;
        let mut cur = std::mem::take(&mut segs[i]);
        let is_underside = cur[cur.len() / 2].is_underside;

        // Extend forward
        while let Some(&tail) = cur.last() {
            if cur.len() > 2 && tail.pos_wcs.distance(cur[0].pos_wcs) <= weld_tol {
                break;
            }
            let mut best: Option<(usize, bool, f64)> = None;
            for (j, cand) in segs.iter().enumerate() {
                if used[j] || cand.len() < 2 || cand[cand.len() / 2].is_underside != is_underside {
                    continue;
                }
                if let Some(&c_first) = cand.first()
                    && c_first.pos_wcs.distance(tail.pos_wcs) <= weld_tol
                {
                    let score = tail.tangent.dot(c_first.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, false, score));
                    }
                }
                if let Some(&c_last) = cand.last()
                    && c_last.pos_wcs.distance(tail.pos_wcs) <= weld_tol
                {
                    let score = tail.tangent.dot(-c_last.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, true, score));
                    }
                }
            }
            let Some((best_j, flip, _)) = best else {
                break;
            };
            used[best_j] = true;
            let mut next_seg = std::mem::take(&mut segs[best_j]);
            if flip {
                reverse_sample_chain(&mut next_seg);
            }
            append_connected_samples(&mut cur, &next_seg);
        }

        // Extend backward (prepend)
        while let Some(&head) = cur.first() {
            if cur.len() > 2
                && let Some(&tail) = cur.last()
                && tail.pos_wcs.distance(head.pos_wcs) <= weld_tol
            {
                break;
            }
            let mut best: Option<(usize, bool, f64)> = None;
            for (j, cand) in segs.iter().enumerate() {
                if used[j] || cand.len() < 2 || cand[cand.len() / 2].is_underside != is_underside {
                    continue;
                }
                if let Some(&c_last) = cand.last()
                    && c_last.pos_wcs.distance(head.pos_wcs) <= weld_tol
                {
                    let score = c_last.tangent.dot(head.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, false, score));
                    }
                }
                if let Some(&c_first) = cand.first()
                    && c_first.pos_wcs.distance(head.pos_wcs) <= weld_tol
                {
                    let score = (-c_first.tangent).dot(head.tangent);
                    if best.is_none_or(|b| score > b.2) {
                        best = Some((j, true, score));
                    }
                }
            }
            let Some((best_j, flip, _)) = best else {
                break;
            };
            used[best_j] = true;
            let mut prev_seg = std::mem::take(&mut segs[best_j]);
            if flip {
                reverse_sample_chain(&mut prev_seg);
            }
            append_connected_samples(&mut prev_seg, &cur);
            cur = prev_seg;
        }

        let closed = is_sample_list_closed(&cur);
        chains.push(DeburrEdgeChain {
            samples: cur,
            closed,
            is_underside,
            user_reversed: false,
        });
    }

    chains
}

/// Ensures chain travel direction matches `climb_direction` (unless explicitly reversed by user).
fn orient_chain_for_cut_direction(chain: &mut DeburrEdgeChain, climb_direction: bool) {
    if chain.user_reversed || chain.samples.len() < 2 {
        return;
    }
    // For CW spindle (`M3`), Climb milling puts the outward air normal to the LEFT of travel:
    // `(tangent_xy × bisector_xy).z > 0`.
    let mut score = 0.0_f64;
    for s in &chain.samples {
        score += s.tangent.x * s.bisector_out.y - s.tangent.y * s.bisector_out.x;
    }
    let want_reverse = if climb_direction {
        score < -1e-6
    } else {
        score > 1e-6
    };
    if want_reverse {
        reverse_sample_chain(&mut chain.samples);
    }
}

fn apply_open_chain_extension(chain: &mut DeburrEdgeChain, ext_mm: f64) {
    if chain.samples.len() < 2 || ext_mm <= 1e-4 {
        return;
    }
    let steps = (ext_mm / 0.25).ceil().clamp(1.0, 40.0) as usize;
    let first = chain.samples[0];
    let last = chain.samples[chain.samples.len() - 1];

    let mut prefix = Vec::with_capacity(steps);
    for k in (1..=steps).rev() {
        let d = ext_mm * (k as f64) / (steps as f64);
        let mut s = first;
        s.pos_wcs = first.pos_wcs - first.tangent * d;
        prefix.push(s);
    }
    let mut suffix = Vec::with_capacity(steps);
    for k in 1..=steps {
        let d = ext_mm * (k as f64) / (steps as f64);
        let mut s = last;
        s.pos_wcs = last.pos_wcs + last.tangent * d;
        suffix.push(s);
    }

    let mut extended = Vec::with_capacity(prefix.len() + chain.samples.len() + suffix.len());
    extended.extend(prefix);
    extended.extend_from_slice(&chain.samples);
    extended.extend(suffix);
    chain.samples = extended;
}

/// Solved collision-free station ready for toolpath emission.
#[derive(Debug, Clone, Copy)]
struct SolvedStation {
    sample: DeburrSample3D,
    sol: DeburrContactSolution,
}

/// Collision-free sub-chain produced by trimming a [`DeburrEdgeChain`] against neighbor walls.
struct TrimmedSubChain {
    stations: Vec<SolvedStation>,
    closed: bool,
    is_underside: bool,
}

#[allow(clippy::too_many_arguments)]
fn trim_chain_against_target(
    chain: &DeburrEdgeChain,
    tool: &chip_tools::Tool,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    chamfer_w: f64,
    tip_offset: f64,
    wall_clearance: f64,
    allow_underside: bool,
    cand_buf: &mut Vec<usize>,
) -> Vec<TrimmedSubChain> {
    let n = chain.samples.len();
    if n < 2 {
        return Vec::new();
    }

    let mut solved: Vec<Option<DeburrContactSolution>> = Vec::with_capacity(n);
    let mut free: Vec<bool> = Vec::with_capacity(n);

    for s in &chain.samples {
        let sol = solve_deburr_tool_tip(
            s,
            &tool.geometry,
            chamfer_w,
            tip_offset,
            wall_clearance,
            allow_underside,
        );
        let ok = if let Some(cs) = sol {
            is_station_collision_free(
                s,
                &cs,
                envelope,
                stickout_mm,
                target_mesh,
                chamfer_w,
                wall_clearance,
                cand_buf,
            )
        } else {
            false
        };
        solved.push(sol);
        free.push(ok);
    }

    let all_free = free.iter().all(|&ok| ok);
    if all_free {
        let stations: Vec<SolvedStation> = chain
            .samples
            .iter()
            .zip(solved.iter())
            .filter_map(|(&sample, sol)| sol.map(|sol| SolvedStation { sample, sol }))
            .collect();
        if chain_length(&stations) >= 0.05 {
            return vec![TrimmedSubChain {
                stations,
                closed: chain.closed,
                is_underside: chain.is_underside,
            }];
        }
        return Vec::new();
    }

    // Split into contiguous collision-free runs with binary boundary refinement (`<= 0.02 mm`)!
    let mut subchains = Vec::new();
    let mut i = 0usize;
    while i < n {
        if !free[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i + 1 < n && free[i + 1] {
            i += 1;
        }
        let end = i;
        i += 1;

        let mut stations = Vec::with_capacity(end - start + 3);

        // Refine entering boundary (`start - 1` colliding -> `start` collision-free)
        if start > 0
            && let Some(refined) = binary_refine_boundary(
                &chain.samples[start],
                &chain.samples[start - 1],
                tool,
                envelope,
                stickout_mm,
                target_mesh,
                chamfer_w,
                tip_offset,
                wall_clearance,
                allow_underside,
                cand_buf,
            )
            && refined.sol.tip_wcs.distance(
                solved[start]
                    .map(|s| s.tip_wcs)
                    .unwrap_or(refined.sol.tip_wcs),
            ) > 1e-4
        {
            stations.push(refined);
        }

        for k in start..=end {
            if let Some(sol) = solved[k] {
                stations.push(SolvedStation {
                    sample: chain.samples[k],
                    sol,
                });
            }
        }

        // Refine exiting boundary (`end` collision-free -> `end + 1` colliding)
        if end + 1 < n
            && let Some(refined) = binary_refine_boundary(
                &chain.samples[end],
                &chain.samples[end + 1],
                tool,
                envelope,
                stickout_mm,
                target_mesh,
                chamfer_w,
                tip_offset,
                wall_clearance,
                allow_underside,
                cand_buf,
            )
            && let Some(last) = stations.last()
            && refined.sol.tip_wcs.distance(last.sol.tip_wcs) > 1e-4
        {
            stations.push(refined);
        }

        if stations.len() >= 2 && chain_length(&stations) >= 0.05 {
            subchains.push(TrimmedSubChain {
                stations,
                closed: false,
                is_underside: chain.is_underside,
            });
        }
    }

    // If the original chain was closed and both the first and last subchains touch the seam,
    // merge them into one seamless open subchain!
    if chain.closed
        && subchains.len() >= 2
        && let (Some(first_pt), Some(last_pt)) = (
            subchains.first().and_then(|s| s.stations.first()),
            subchains.last().and_then(|s| s.stations.last()),
        )
        && first_pt.sample.pos_wcs.distance(last_pt.sample.pos_wcs) <= COINCIDENCE_TOL_MM * 4.0
    {
        let mut tail = subchains.pop().unwrap_or(TrimmedSubChain {
            stations: Vec::new(),
            closed: false,
            is_underside: chain.is_underside,
        });
        let head = subchains.remove(0);
        if let (Some(tail_last), Some(head_first)) = (tail.stations.last(), head.stations.first()) {
            let same_normal = tail_last.sol.outward_xy.dot(head_first.sol.outward_xy) >= 0.995;
            if same_normal {
                tail.stations.extend_from_slice(&head.stations[1..]);
            } else {
                // Seam sits on a sharp convex corner: keep `head.stations[0]` so `emit_deburr_subchain`
                // sees `corner_dist <= 0.25` and emits a smooth `emit_corner_roll` around the miter!
                tail.stations.extend_from_slice(&head.stations);
            }
        }
        subchains.insert(0, tail);
    }

    subchains
}

/// Binary-refines between a collision-free sample `good` (`u = 0`) and a colliding sample `bad`
/// (`u = 1`) down to `< 0.005 mm` (8 iterations on `<= 0.5 mm` step).
#[allow(clippy::too_many_arguments)]
fn binary_refine_boundary(
    good: &DeburrSample3D,
    bad: &DeburrSample3D,
    tool: &chip_tools::Tool,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    chamfer_w: f64,
    tip_offset: f64,
    wall_clearance: f64,
    allow_underside: bool,
    cand_buf: &mut Vec<usize>,
) -> Option<SolvedStation> {
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut best: Option<SolvedStation> = None;

    for _ in 0..8 {
        let mid_u = 0.5 * (lo + hi);
        let sample = lerp_sample(good, bad, mid_u);
        let ok = if let Some(sol) = solve_deburr_tool_tip(
            &sample,
            &tool.geometry,
            chamfer_w,
            tip_offset,
            wall_clearance,
            allow_underside,
        ) {
            if is_station_collision_free(
                &sample,
                &sol,
                envelope,
                stickout_mm,
                target_mesh,
                chamfer_w,
                wall_clearance,
                cand_buf,
            ) {
                best = Some(SolvedStation { sample, sol });
                true
            } else {
                false
            }
        } else {
            false
        };
        if ok {
            lo = mid_u;
        } else {
            hi = mid_u;
        }
    }

    best
}

fn lerp_sample(a: &DeburrSample3D, b: &DeburrSample3D, u: f64) -> DeburrSample3D {
    let pos_wcs = a.pos_wcs.lerp(b.pos_wcs, u);
    let tangent = a.tangent.lerp(b.tangent, u).normalize_or_zero();
    let n1 = a.n1.lerp(b.n1, u).normalize_or_zero();
    let n2 = a.n2.lerp(b.n2, u).normalize_or_zero();
    let bisector_out = (n1 + n2).normalize_or_zero();
    DeburrSample3D {
        pos_wcs,
        tangent: if tangent.length_squared() > 0.25 {
            tangent
        } else {
            a.tangent
        },
        n1: if n1.length_squared() > 0.25 { n1 } else { a.n1 },
        n2: if n2.length_squared() > 0.25 { n2 } else { a.n2 },
        bisector_out: if bisector_out.length_squared() > 0.25 {
            bisector_out
        } else {
            a.bisector_out
        },
        is_underside: a.is_underside,
        face1: a.face1,
        face2: a.face2,
    }
}

fn chain_length(stations: &[SolvedStation]) -> f64 {
    stations
        .windows(2)
        .map(|w| w[0].sol.tip_wcs.distance(w[1].sol.tip_wcs))
        .sum()
}

/// Checks whether the full revolved cutter assembly (flutes + relieved neck + cylindrical shank)
/// at `sol.tip_wcs` clears all non-bevel geometry of `target_mesh` by at least `wall_clearance`.
#[allow(clippy::too_many_arguments)]
fn is_station_collision_free(
    sample: &DeburrSample3D,
    sol: &DeburrContactSolution,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    chamfer_w: f64,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> bool {
    if target_mesh.triangles.is_empty() {
        return true;
    }

    let tip = sol.tip_wcs;
    let r_max =
        envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance + 0.05;
    let query_aabb = Aabb3::new(
        DVec3::new(tip.x - r_max, tip.y - r_max, tip.z - wall_clearance - 0.05),
        DVec3::new(tip.x + r_max, tip.y + r_max, tip.z + stickout_mm),
    );
    target_mesh.bvh.query_aabb_into(&query_aabb, cand_buf);

    let f1_id = sample.face1.0;
    let f2_id = sample.face2.0;
    let flute_len = envelope.flute_length_mm();
    let neck_reach = envelope.effective_reach_mm();
    let tip_profile_h = envelope.tip_profile_height_mm();
    let max_chord_excl = sol.contact_chord_mm + wall_clearance + 0.25;

    for &tri_idx in cand_buf.iter() {
        let tri = target_mesh.triangles[tri_idx];
        let v0 = target_mesh.vertices[tri.indices[0] as usize];
        let v1 = target_mesh.vertices[tri.indices[1] as usize];
        let v2 = target_mesh.vertices[tri.indices[2] as usize];

        let tri_z_min = v0.z.min(v1.z).min(v2.z);
        let tri_z_max = v0.z.max(v1.z).max(v2.z);
        let z_lo = tri_z_min.max(tip.z - wall_clearance);
        let z_hi = tri_z_max.min(tip.z + stickout_mm);
        if z_hi < z_lo - 1e-6 {
            continue;
        }

        let is_adj_face = tri.face_id == f1_id || tri.face_id == f2_id;
        let same_adj_side =
            is_adj_face && (tri.normal.dot(sample.n1) > 0.35 || tri.normal.dot(sample.n2) > 0.35);

        // Collect key Z test planes across `[z_lo, z_hi]`
        let mut z_planes = [0.0_f64; 8];
        let mut n_key = 0usize;
        z_planes[n_key] = z_lo;
        n_key += 1;
        if (z_hi - z_lo).abs() > 1e-6 {
            z_planes[n_key] = z_hi;
            n_key += 1;
        }
        for dz_crit in [0.0, tip_profile_h, flute_len, neck_reach] {
            let zc = tip.z + dz_crit;
            if zc > z_lo + 1e-5 && zc < z_hi - 1e-5 && n_key < z_planes.len() {
                z_planes[n_key] = zc;
                n_key += 1;
            }
        }

        // Uniform Z sub-samples every <= 0.5 mm across the triangle's active overlap
        let z_span = (z_hi - z_lo).max(0.0);
        let n_sub = ((z_span / 0.5).ceil() as usize).clamp(1, 64);

        let check_z = |zk: f64| -> bool {
            let Some(q) = closest_point_on_triangle_at_z(v0, v1, v2, zk, DVec2::new(tip.x, tip.y))
            else {
                return true;
            };
            let dz = zk - tip.z;
            let r_axis = (q.x - tip.x).hypot(q.y - tip.y);

            let Some(r_lim) =
                cutter_inflated_radius_at_dz(envelope, stickout_mm, dz, wall_clearance)
            else {
                return true;
            };
            if r_axis >= r_lim - 1e-5 {
                return true;
            }

            // Non-cutting relieved neck or cylindrical shank is NEVER exempted from collision!
            if dz > flute_len + 1e-4 {
                return false;
            }

            // Below cutter tip (vertical floor clearance check)
            if dz < -1e-4 {
                return false;
            }

            // Inside the active flute zone: check whether `q` lies strictly inside the intentional
            // bevel wedge of the active edge!
            if is_point_in_intentional_bevel_wedge(
                q,
                sample,
                sol,
                same_adj_side,
                chamfer_w,
                max_chord_excl,
            ) {
                return true;
            }

            false
        };

        for &zk in &z_planes[..n_key] {
            if !check_z(zk) {
                return false;
            }
        }
        if n_sub > 1 {
            for s in 1..n_sub {
                let zk = z_lo + z_span * (s as f64) / (n_sub as f64);
                if !check_z(zk) {
                    return false;
                }
            }
        }
    }

    true
}

/// Checks whether a purely vertical plunge/retract column `(xy, z_start ..= z_top)` is completely
/// collision-free against `target_mesh` (with zero bevel exemption!).
fn is_vertical_column_clear(
    tip_bottom: DVec3,
    z_top: f64,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> bool {
    if target_mesh.triangles.is_empty() {
        return true;
    }
    let r_max =
        envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance + 0.05;
    let z_lo_all = tip_bottom.z - wall_clearance;
    let z_hi_all = z_top.max(tip_bottom.z) + stickout_mm;
    let query_aabb = Aabb3::new(
        DVec3::new(tip_bottom.x - r_max, tip_bottom.y - r_max, z_lo_all),
        DVec3::new(tip_bottom.x + r_max, tip_bottom.y + r_max, z_hi_all),
    );
    target_mesh.bvh.query_aabb_into(&query_aabb, cand_buf);
    if cand_buf.is_empty() {
        return true;
    }

    // Check swept vertical column: at any Z >= tip_bottom.z, the maximum radius of any part of
    // the cutter passing through Z during the vertical stroke `[tip_bottom.z, z_top]` is
    // `max(cutting_radius, shank_radius)`!
    let r_swept = envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance;
    let xy = DVec2::new(tip_bottom.x, tip_bottom.y);

    for &tri_idx in cand_buf.iter() {
        let tri = target_mesh.triangles[tri_idx];
        let v0 = target_mesh.vertices[tri.indices[0] as usize];
        let v1 = target_mesh.vertices[tri.indices[1] as usize];
        let v2 = target_mesh.vertices[tri.indices[2] as usize];
        let tri_z_max = v0.z.max(v1.z).max(v2.z);
        if tri_z_max < z_lo_all {
            continue;
        }
        // Project triangle to XY: if its XY distance to `xy` is < r_swept while above `z_lo_all`,
        // test closest 3D point on the triangle above `z_lo_all`!
        let tri_z_min = v0.z.min(v1.z).min(v2.z);
        let z_a = tri_z_min.max(z_lo_all);
        let z_b = tri_z_max.min(z_hi_all);
        if z_b < z_a - 1e-6 {
            continue;
        }
        for zk in [z_a, 0.5 * (z_a + z_b), z_b] {
            if let Some(q) = closest_point_on_triangle_at_z(v0, v1, v2, zk, xy) {
                let r_axis = (q.x - xy.x).hypot(q.y - xy.y);
                let r_lim = if zk < tip_bottom.z {
                    cutter_inflated_radius_at_dz(
                        envelope,
                        stickout_mm,
                        zk - tip_bottom.z,
                        wall_clearance,
                    )
                    .unwrap_or(0.0)
                } else {
                    r_swept
                };
                if r_axis < r_lim - 1e-5 {
                    return false;
                }
            }
        }
    }

    true
}

/// Checks whether a horizontal transit point `tip` (outside the cut, e.g. on a lead-in/lead-out
/// arc or approach bridge) clears the target mesh completely with zero bevel exemption.
fn is_air_position_clear(
    tip: DVec3,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> bool {
    if target_mesh.triangles.is_empty() {
        return true;
    }
    let r_max =
        envelope.cutting_radius_mm().max(envelope.shank_radius_mm()) + wall_clearance + 0.05;
    let query_aabb = Aabb3::new(
        DVec3::new(tip.x - r_max, tip.y - r_max, tip.z - wall_clearance - 0.05),
        DVec3::new(tip.x + r_max, tip.y + r_max, tip.z + stickout_mm),
    );
    target_mesh.bvh.query_aabb_into(&query_aabb, cand_buf);
    let xy = DVec2::new(tip.x, tip.y);

    for &tri_idx in cand_buf.iter() {
        let tri = target_mesh.triangles[tri_idx];
        let v0 = target_mesh.vertices[tri.indices[0] as usize];
        let v1 = target_mesh.vertices[tri.indices[1] as usize];
        let v2 = target_mesh.vertices[tri.indices[2] as usize];
        let z_lo = v0.z.min(v1.z).min(v2.z).max(tip.z - wall_clearance);
        let z_hi = v0.z.max(v1.z).max(v2.z).min(tip.z + stickout_mm);
        if z_hi < z_lo - 1e-6 {
            continue;
        }
        let z_span = (z_hi - z_lo).max(0.0);
        let n_sub = ((z_span / 0.5).ceil() as usize).clamp(1, 32);
        for s in 0..=n_sub {
            let zk = z_lo + z_span * (s as f64) / (n_sub as f64);
            if let Some(q) = closest_point_on_triangle_at_z(v0, v1, v2, zk, xy) {
                let dz = zk - tip.z;
                if let Some(r_lim) =
                    cutter_inflated_radius_at_dz(envelope, stickout_mm, dz, wall_clearance)
                    && (q.x - xy.x).hypot(q.y - xy.y) < r_lim - 1e-5
                {
                    return false;
                }
            }
        }
    }
    true
}

/// Evaluates the cutter's revolved radius at height `dz` above `tip_wcs`, inflated outward in 3D by
/// `wall_clearance` (handling Ball, Lollipop sphere + relieved neck + shank, and Conical flanks).
fn cutter_inflated_radius_at_dz(
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    dz: f64,
    wall_clearance: f64,
) -> Option<f64> {
    let wc = wall_clearance.max(0.0);
    if !dz.is_finite() || dz < -wc - 1e-6 || dz > stickout_mm + 1e-6 {
        return None;
    }

    match envelope.shape() {
        CutterEnvelopeShape::Ball { radius_mm } => {
            if dz < radius_mm {
                let r_inf = radius_mm + wc;
                let dy = dz - radius_mm;
                let disc = r_inf * r_inf - dy * dy;
                if disc <= 0.0 { None } else { Some(disc.sqrt()) }
            } else if dz <= envelope.flute_length_mm() + 1e-6 {
                Some(radius_mm + wc)
            } else {
                Some(envelope.shank_radius_mm() + wc)
            }
        }
        CutterEnvelopeShape::Lollipop {
            sphere_radius_mm,
            neck_radius_mm,
            neck_length_mm,
            z_neck_join_mm,
            ..
        } => {
            let r_inf = sphere_radius_mm + wc;
            let dy = dz - sphere_radius_mm;
            let sphere_r = if dy.abs() < r_inf {
                Some((r_inf * r_inf - dy * dy).max(0.0).sqrt())
            } else {
                None
            };
            if dz <= z_neck_join_mm + wc {
                // Take the union of the inflated sphere and the relieved neck above the join
                let neck_r = if dz >= z_neck_join_mm - 1e-6 {
                    Some(neck_radius_mm + wc)
                } else {
                    None
                };
                match (sphere_r, neck_r) {
                    (Some(a), Some(b)) => Some(a.max(b)),
                    (Some(a), None) => Some(a),
                    (None, Some(b)) => Some(b),
                    (None, None) => None,
                }
            } else if dz <= neck_length_mm + 1e-6 {
                Some(neck_radius_mm + wc)
            } else {
                Some(envelope.shank_radius_mm() + wc)
            }
        }
        CutterEnvelopeShape::Conical {
            tip_radius_mm,
            upper_radius_mm,
            half_angle_rad,
            tan_half_angle,
        } => {
            if dz < 0.0 {
                // Within `wall_clearance` directly below the tip
                Some(tip_radius_mm + wc)
            } else if dz <= envelope.flute_length_mm() + 1e-6 {
                let radial_infl = wc / half_angle_rad.cos().max(0.1);
                Some((tip_radius_mm + dz * tan_half_angle + radial_infl).min(upper_radius_mm + wc))
            } else {
                Some(envelope.shank_radius_mm() + wc)
            }
        }
        _ => {
            if dz < 0.0 {
                Some(envelope.cutting_radius_mm() + wc)
            } else {
                envelope
                    .cutter_radius_at_z_within(dz, stickout_mm)
                    .map(|r| r + wc)
            }
        }
    }
}

/// Convex-Corner Bevel-Wedge Lemma: returns `true` iff `q` lies strictly inside the intentional
/// chamfered corner material around `sample.pos_wcs` (never exempting protruding neighbor walls,
/// inside pocket corners, raised stepped bosses, lower floors, or opposite bore walls!).
fn is_point_in_intentional_bevel_wedge(
    q: DVec3,
    sample: &DeburrSample3D,
    sol: &DeburrContactSolution,
    same_adj_side: bool,
    chamfer_w: f64,
    max_chord_excl: f64,
) -> bool {
    let dist_to_edge = q.distance(sample.pos_wcs);
    if dist_to_edge > max_chord_excl {
        return false;
    }

    let delta = q - sample.pos_wcs;
    let d1 = delta.dot(sample.n1);
    let d2 = delta.dot(sample.n2);
    let bevel_signed = (q - sol.bevel_point_wcs).dot(sol.bevel_normal_wcs);

    if same_adj_side {
        // Active adjacent face (`f1` or `f2`, same orientation side): allow slight positive sag
        // along curved internal hole rims while still verifying `q` is inside the bevel cut depth.
        let curv_allow = 0.08 + 0.15 * chamfer_w;
        d1 <= curv_allow && d2 <= curv_allow && bevel_signed >= -(0.12 + 0.25 * chamfer_w)
    } else {
        // Non-adjacent face (e.g., perpendicular side face at an outer convex box corner):
        // ONLY exempt when `q` lies strictly on the SOLID side of BOTH `n1` and `n2` AND inside
        // the bevel wedge! Protruding inside pocket walls and raised bosses fail `d1 <= 0.03` or
        // `d2 <= 0.03` immediately!
        d1 <= 0.03 && d2 <= 0.03 && bevel_signed >= -0.06
    }
}

/// Emits a single collision-free 3D deburr sub-chain with smooth tangent arc lead-in/lead-out and
/// verified collision-free vertical approach/retract columns.
#[allow(clippy::too_many_arguments)]
fn emit_deburr_subchain(
    ctx: &StrategyExecContext<'_>,
    sub: &TrimmedSubChain,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    retract_z: f64,
    cut_role: MotionRole,
    cand_buf: &mut Vec<usize>,
    builder: &mut ToolpathBuilder,
) -> bool {
    let Some(first) = sub.stations.first() else {
        return false;
    };
    let Some(last) = sub.stations.last() else {
        return false;
    };

    let p0 = first.sol.tip_wcs;
    let out0 = first.sol.outward_xy;
    let t0_xy = DVec2::new(first.sample.tangent.x, first.sample.tangent.y).normalize_or_zero();

    let p1 = last.sol.tip_wcs;
    let out1 = last.sol.outward_xy;
    let t1_xy = DVec2::new(last.sample.tangent.x, last.sample.tangent.y).normalize_or_zero();

    let pref_lead_r = if ctx.linking.lead_in.enabled {
        ctx.linking
            .lead_in
            .horizontal_radius
            .value_mm
            .clamp(0.4, 4.0)
    } else {
        0.0
    };

    // Resolve safe tangent lead-in arc + safe vertical plunge column!
    let (plunge_pt, lead_in_arc, linear_approach_pt) = plan_safe_lead(
        p0,
        t0_xy,
        out0,
        true,
        sub.is_underside,
        pref_lead_r,
        retract_z,
        envelope,
        stickout_mm,
        target_mesh,
        wall_clearance,
        cand_buf,
    );

    builder.pass_count += 1;

    // 1. Safe rapid to approach XY at `retract_z`
    if let Some(cur) = builder.current_pos()
        && cur.z < retract_z - 1e-6
    {
        builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
    }
    builder.rapid_to(DVec3::new(plunge_pt.x, plunge_pt.y, retract_z));
    let feed_z = ctx.heights.feed_z_wcs.max(plunge_pt.z);
    if feed_z < retract_z - 1e-4 {
        builder.rapid_to(DVec3::new(plunge_pt.x, plunge_pt.y, feed_z));
    }

    // 2. Plunge vertically in open air to approach Z
    builder.linear_to(
        plunge_pt,
        ctx.cutting.plunge_feed_mm_min,
        MotionRole::Plunge,
    );

    // 3. Horizontal linear approach (if pushed out past an overhang) + smooth tangent lead-in arc
    if let Some(mid_pt) = linear_approach_pt
        && mid_pt.distance(plunge_pt) > 1e-4
    {
        builder.linear_to(mid_pt, ctx.cutting.lead_feed_mm_min, MotionRole::LeadIn);
    }
    if let Some((center_xy, radius_mm, ccw)) = lead_in_arc {
        builder.arc_xy_to(XyArcMove {
            end_wcs: p0,
            center_xy,
            radius_mm,
            ccw,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadIn,
        });
    } else if builder.current_pos().is_some_and(|c| c.distance(p0) > 1e-4) {
        builder.linear_to(p0, ctx.cutting.lead_feed_mm_min, MotionRole::LeadIn);
    }

    // 4. Cut exact 3D non-planar tool-tip trajectory with smooth circular corner-rolls around sharp
    //    outer convex corners (preserving full flute & relieved-neck clearance around 90° miters!).
    let mut prev_st = first;
    for st in sub.stations.iter().skip(1) {
        let out_dot = prev_st
            .sol
            .outward_xy
            .dot(st.sol.outward_xy)
            .clamp(-1.0, 1.0);
        let corner_dist = prev_st.sample.pos_wcs.distance(st.sample.pos_wcs);
        if out_dot < 0.995 && corner_dist <= 0.25 {
            emit_corner_roll(
                prev_st,
                st,
                ctx.cutting.cutting_feed_mm_min,
                cut_role,
                builder,
            );
        } else if builder
            .current_pos()
            .is_none_or(|c| c.distance(st.sol.tip_wcs) > 1e-6)
        {
            builder.linear_to(st.sol.tip_wcs, ctx.cutting.cutting_feed_mm_min, cut_role);
        }
        prev_st = st;
    }

    // Close the corner smoothly on closed loops whose first and last edges meet at a sharp angle!
    let mut exit_tip = p1;
    let mut exit_tan_xy = t1_xy;
    let mut exit_out_xy = out1;
    if sub.closed
        && sub.stations.len() >= 3
        && last.sol.outward_xy.dot(first.sol.outward_xy) < 0.995
        && last.sample.pos_wcs.distance(first.sample.pos_wcs) <= 0.25
    {
        emit_corner_roll(
            last,
            first,
            ctx.cutting.cutting_feed_mm_min,
            cut_role,
            builder,
        );
        exit_tip = first.sol.tip_wcs;
        exit_tan_xy = t0_xy;
        exit_out_xy = out0;
    }

    // Optional closed-loop seam overlap to avoid dwell witness marks (tracking exact final exit state!)
    if sub.closed && sub.stations.len() >= 3 {
        let ov_target = ctx.linking.seam_overlap.value_mm.clamp(0.0, 2.0);
        if ov_target > 1e-3 {
            let mut acc = 0.0_f64;
            for w in sub.stations.windows(2) {
                let seg_len = w[0].sol.tip_wcs.distance(w[1].sol.tip_wcs);
                if seg_len < 1e-6 {
                    continue;
                }
                if w[0].sol.outward_xy.dot(w[1].sol.outward_xy) < 0.995 {
                    break;
                }
                if acc + seg_len >= ov_target {
                    let frac = ((ov_target - acc) / seg_len).clamp(0.0, 1.0);
                    let pt = w[0].sol.tip_wcs.lerp(w[1].sol.tip_wcs, frac);
                    builder.linear_to(pt, ctx.cutting.cutting_feed_mm_min, cut_role);
                    exit_tip = pt;
                    let t_interp = w[0].sample.tangent.lerp(w[1].sample.tangent, frac);
                    exit_tan_xy = DVec2::new(t_interp.x, t_interp.y).normalize_or_zero();
                    exit_out_xy = w[0]
                        .sol
                        .outward_xy
                        .lerp(w[1].sol.outward_xy, frac)
                        .normalize_or_zero();
                    break;
                }
                builder.linear_to(w[1].sol.tip_wcs, ctx.cutting.cutting_feed_mm_min, cut_role);
                exit_tip = w[1].sol.tip_wcs;
                exit_tan_xy =
                    DVec2::new(w[1].sample.tangent.x, w[1].sample.tangent.y).normalize_or_zero();
                exit_out_xy = w[1].sol.outward_xy;
                acc += seg_len;
            }
        }
    }

    // Resolve safe tangent lead-out arc + safe vertical retract column at exact `exit_tip`!
    let (retract_pt, lead_out_arc, linear_depart_pt) = plan_safe_lead(
        exit_tip,
        exit_tan_xy,
        exit_out_xy,
        false,
        sub.is_underside,
        pref_lead_r,
        retract_z,
        envelope,
        stickout_mm,
        target_mesh,
        wall_clearance,
        cand_buf,
    );

    // 5. Smooth tangent lead-out arc + optional horizontal clearance departure (for overhangs)
    if let Some((center_xy, radius_mm, ccw)) = lead_out_arc
        && let Some(end_arc) = linear_depart_pt.or(Some(retract_pt))
    {
        builder.arc_xy_to(XyArcMove {
            end_wcs: end_arc,
            center_xy,
            radius_mm,
            ccw,
            turns: 0,
            feed_mm_min: ctx.cutting.lead_feed_mm_min,
            role: MotionRole::LeadOut,
        });
    }
    if builder
        .current_pos()
        .is_some_and(|c| c.distance(retract_pt) > 1e-4)
    {
        builder.linear_to(
            retract_pt,
            ctx.cutting.lead_feed_mm_min,
            MotionRole::LeadOut,
        );
    }

    // 6. Safe vertical rapid retract up to `retract_z`
    builder.rapid_to(DVec3::new(retract_pt.x, retract_pt.y, retract_z));
    true
}

/// Rolls smoothly around a sharp outer convex corner vertex shared by `prev_st` and `next_st`
/// (interpolating the radial offset vector around the corner in `<= 5°` angular increments so
/// straight chords never clip the solid corner or rub a relieved lollipop neck!).
fn emit_corner_roll(
    prev_st: &SolvedStation,
    next_st: &SolvedStation,
    feed_mm_min: f64,
    role: MotionRole,
    builder: &mut ToolpathBuilder,
) {
    let corner_xy = 0.5
        * (DVec2::new(prev_st.sample.pos_wcs.x, prev_st.sample.pos_wcs.y)
            + DVec2::new(next_st.sample.pos_wcs.x, next_st.sample.pos_wcs.y));
    let v0 = DVec2::new(prev_st.sol.tip_wcs.x, prev_st.sol.tip_wcs.y) - corner_xy;
    let v1 = DVec2::new(next_st.sol.tip_wcs.x, next_st.sol.tip_wcs.y) - corner_xy;
    let r0 = v0.length();
    let r1 = v1.length();
    if r0 < 1e-4 || r1 < 1e-4 {
        builder.linear_to(next_st.sol.tip_wcs, feed_mm_min, role);
        return;
    }
    let a0 = v0.y.atan2(v0.x);
    let u0 = v0 / r0;
    let u1 = v1 / r1;
    let d_ang = (u0.x * u1.y - u0.y * u1.x).atan2(u0.dot(u1));
    let steps = (d_ang.abs() / 5.0_f64.to_radians()).ceil().clamp(2.0, 36.0) as usize;
    let z0 = prev_st.sol.tip_wcs.z;
    let z1 = next_st.sol.tip_wcs.z;

    for k in 1..=steps {
        let frac = (k as f64) / (steps as f64);
        let pt = if k == steps {
            next_st.sol.tip_wcs
        } else {
            let ang = a0 + d_ang * frac;
            let r = r0 + (r1 - r0) * frac;
            let (s, c) = ang.sin_cos();
            DVec3::new(
                corner_xy.x + r * c,
                corner_xy.y + r * s,
                z0 + (z1 - z0) * frac,
            )
        };
        if builder
            .current_pos()
            .is_none_or(|cur| cur.distance(pt) > 1e-6)
        {
            builder.linear_to(pt, feed_mm_min, role);
        }
    }
}

/// Plans a collision-free tangent arc lead-in (`is_lead_in = true`) or lead-out
/// (`is_lead_in = false`) plus a verified collision-free vertical plunge/retract column.
///
/// Returns `(vertical_column_pt, Option<(arc_center_xy, radius, ccw)>, Option<arc_outer_pt>)`.
#[allow(clippy::too_many_arguments)]
fn plan_safe_lead(
    edge_tip: DVec3,
    tan_xy: DVec2,
    out_xy: DVec2,
    is_lead_in: bool,
    is_underside: bool,
    pref_radius: f64,
    retract_z: f64,
    envelope: &AnalyticCutterEnvelope,
    stickout_mm: f64,
    target_mesh: &TriMesh3D,
    wall_clearance: f64,
    cand_buf: &mut Vec<usize>,
) -> (DVec3, Option<(DVec2, f64, bool)>, Option<DVec3>) {
    let tip_xy = DVec2::new(edge_tip.x, edge_tip.y);
    let radii = if pref_radius > 0.1 {
        [pref_radius, pref_radius * 0.5, pref_radius * 0.25]
    } else {
        [0.0, 0.0, 0.0]
    };

    // Helper: push a point horizontally along `out_xy` until the vertical plunge/retract column
    // clears any overhang roof / neighbor wall completely!
    let find_clear_vertical = |start_pt: DVec3, cand_buf: &mut Vec<usize>| -> Option<DVec3> {
        let extra_offsets: &[f64] = if is_underside {
            &[0.0, 1.0, 2.0, 3.5, 5.0, 8.0, 12.0]
        } else {
            &[0.0, 0.5, 1.5, 3.0, 6.0]
        };
        for &off in extra_offsets {
            let p = DVec3::new(
                start_pt.x + out_xy.x * off,
                start_pt.y + out_xy.y * off,
                start_pt.z,
            );
            if is_vertical_column_clear(
                p,
                retract_z,
                envelope,
                stickout_mm,
                target_mesh,
                wall_clearance,
                cand_buf,
            ) {
                return Some(p);
            }
        }
        None
    };

    if tan_xy.length_squared() > 0.5 {
        for &r in &radii {
            if r < 0.15 {
                continue;
            }
            let center_xy = tip_xy + out_xy * r;
            // Quarter-circle (90°) tangent arc on the `out_xy` side of the edge
            let arc_outer_xy = if is_lead_in {
                center_xy - tan_xy * r
            } else {
                center_xy + tan_xy * r
            };
            let arc_outer_pt = DVec3::new(arc_outer_xy.x, arc_outer_xy.y, edge_tip.z);

            // Arc midpoint (45° along the quarter-circle)
            let mid_dir = if is_lead_in {
                (-out_xy - tan_xy).normalize_or_zero()
            } else {
                (-out_xy + tan_xy).normalize_or_zero()
            };
            let arc_mid_xy = center_xy + mid_dir * r;
            let arc_mid_pt = DVec3::new(arc_mid_xy.x, arc_mid_xy.y, edge_tip.z);

            let arc_ok = is_air_position_clear(
                arc_outer_pt,
                envelope,
                stickout_mm,
                target_mesh,
                wall_clearance,
                cand_buf,
            ) && is_air_position_clear(
                arc_mid_pt,
                envelope,
                stickout_mm,
                target_mesh,
                wall_clearance,
                cand_buf,
            );
            if !arc_ok {
                continue;
            }

            if let Some(col_pt) = find_clear_vertical(arc_outer_pt, cand_buf) {
                // Determine exact XY arc orientation (`ccw`):
                // For lead-in: vector from center to `edge_tip` is `-out_xy`, arriving tangent is `tan_xy`
                // Cross `(-out_xy) × tan_xy > 0` => CCW!
                // For lead-out: leaving `edge_tip` with tangent `tan_xy`, radius `-out_xy` => same cross!
                let cross = (-out_xy.x) * tan_xy.y - (-out_xy.y) * tan_xy.x;
                let ccw = cross > 0.0;
                return (col_pt, Some((center_xy, r, ccw)), Some(arc_outer_pt));
            }
        }
    }

    // Fallback: straight outward clearance approach along `out_xy`
    let base_out = if is_underside {
        envelope.cutting_radius_mm() + wall_clearance + 1.0
    } else {
        wall_clearance + 0.5
    };
    let start_out = DVec3::new(
        edge_tip.x + out_xy.x * base_out,
        edge_tip.y + out_xy.y * base_out,
        edge_tip.z,
    );
    if let Some(col_pt) = find_clear_vertical(start_out, cand_buf) {
        return (col_pt, None, None);
    }
    if let Some(col_pt) = find_clear_vertical(edge_tip, cand_buf) {
        return (col_pt, None, None);
    }

    (edge_tip, None, None)
}

/// Intersects triangle `(v0, v1, v2)` with horizontal plane `Z = zk` and returns the closest point
/// on that intersection to `(xy.x, xy.y)`.
fn closest_point_on_triangle_at_z(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    zk: f64,
    xy: DVec2,
) -> Option<DVec3> {
    let z_min = v0.z.min(v1.z).min(v2.z);
    let z_max = v0.z.max(v1.z).max(v2.z);
    if zk < z_min - 1e-6 || zk > z_max + 1e-6 {
        return None;
    }

    // Nearly horizontal triangle: use exact 3D triangle closest point directly!
    if z_max - z_min < 1e-5 {
        let p = closest_point_on_triangle_3d(DVec3::new(xy.x, xy.y, zk), v0, v1, v2);
        return Some(DVec3::new(p.x, p.y, zk));
    }

    let verts = [v0, v1, v2];
    let mut hits = [DVec2::ZERO; 4];
    let mut n_hits = 0usize;

    for e in 0..3 {
        let a = verts[e];
        let b = verts[(e + 1) % 3];
        let da = a.z - zk;
        let db = b.z - zk;
        if da.abs() <= 1e-9 {
            if n_hits < hits.len() {
                hits[n_hits] = DVec2::new(a.x, a.y);
                n_hits += 1;
            }
        } else if (da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0) {
            let t = da / (da - db);
            let p = a.lerp(b, t);
            if n_hits < hits.len() {
                hits[n_hits] = DVec2::new(p.x, p.y);
                n_hits += 1;
            }
        }
    }

    if n_hits == 0 {
        let p = closest_point_on_triangle_3d(DVec3::new(xy.x, xy.y, zk), v0, v1, v2);
        return Some(DVec3::new(p.x, p.y, zk));
    }
    if n_hits == 1 {
        return Some(DVec3::new(hits[0].x, hits[0].y, zk));
    }

    // Closest point on the 2D intersection segment (or polygon if > 2 hits due to coplanar edge)
    let mut best_pt = hits[0];
    let mut best_d2 = hits[0].distance_squared(xy);
    for i in 0..n_hits {
        let a = hits[i];
        let b = hits[(i + 1) % n_hits];
        let cp = closest_point_on_segment_2d(xy, a, b);
        let d2 = cp.distance_squared(xy);
        if d2 < best_d2 {
            best_d2 = d2;
            best_pt = cp;
        }
    }
    Some(DVec3::new(best_pt.x, best_pt.y, zk))
}

#[inline]
fn closest_point_on_segment_2d(p: DVec2, a: DVec2, b: DVec2) -> DVec2 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq <= 1e-18 {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Exact Ericson closest point on a 3D triangle `(a, b, c)` to point `p`.
fn closest_point_on_triangle_3d(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> DVec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v;
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w;
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    let denom = 1.0 / (va + vb + vc).max(1e-18);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;

    fn sample_90deg_top_edge() -> DeburrSample3D {
        DeburrSample3D {
            pos_wcs: DVec3::new(0.0, 0.0, 10.0),
            tangent: DVec3::Y,
            n1: DVec3::Z,
            n2: DVec3::X,
            bisector_out: DVec3::new(1.0, 0.0, 1.0).normalize(),
            is_underside: false,
            face1: FaceId(0),
            face2: FaceId(1),
        }
    }

    #[test]
    fn solve_deburr_tool_tip_exact_chamfer_ball_and_lollipop_kinematics() {
        let s = sample_90deg_top_edge();

        // 1. 90° Chamfer Mill (`D=6`, `tip=0`, `w=0.5`, `tip_offset=0.75`):
        //    Matches classic 2D chamfer kinematics: tip at `(x = +0.75, z = 10 - 0.5 - 0.75 = 8.75)`.
        let chamf = CutterGeometry::ChamferMill {
            diameter_mm: 6.0,
            tip_diameter_mm: 0.0,
            included_angle_deg: 90.0,
            flute_length_mm: 3.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        let sol_c = solve_deburr_tool_tip(&s, &chamf, 0.5, 0.75, 0.15, false).unwrap();
        assert!(
            approx_eq(sol_c.tip_wcs.x, 0.75, 1e-6),
            "got {:?}",
            sol_c.tip_wcs
        );
        assert!(
            approx_eq(sol_c.tip_wcs.z, 8.75, 1e-6),
            "got {:?}",
            sol_c.tip_wcs
        );

        // 2. Ø6 mm Ball Endmill (`R=3`, `w=0.5`, `d_bevel = 0.5 / sqrt(2)`):
        //    Sphere center at `p + (R - d_bevel) * b`, tip `R` below center.
        let ball = CutterGeometry::BallEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        };
        let sol_b = solve_deburr_tool_tip(&s, &ball, 0.5, 0.75, 0.15, false).unwrap();
        let d_bevel = 0.5 * std::f64::consts::FRAC_1_SQRT_2;
        let expected_c = s.pos_wcs + s.bisector_out * (3.0 - d_bevel);
        assert!(approx_eq(sol_b.tip_wcs.x, expected_c.x, 1e-6));
        assert!(approx_eq(sol_b.tip_wcs.z, expected_c.z - 3.0, 1e-6));

        // 3. Ø6 mm Lollipop Endmill (`R=3`, `r_neck=1.5`) on underside overhang rim (`b = (+1, 0, -1)/sqrt(2)`):
        let s_under = DeburrSample3D {
            pos_wcs: DVec3::new(20.0, 0.0, 10.0),
            tangent: DVec3::Y,
            n1: -DVec3::Z,
            n2: DVec3::X,
            bisector_out: DVec3::new(1.0, 0.0, -1.0).normalize(),
            is_underside: true,
            face1: FaceId(2),
            face2: FaceId(3),
        };
        let lolli = CutterGeometry::LollipopEndmill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.0,
            neck_length_mm: 18.0,
            flute_length_mm: 5.5,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        // Rejected when `allow_underside_back_chamfer == false`
        assert!(solve_deburr_tool_tip(&s_under, &lolli, 0.5, 0.75, 0.15, false).is_none());
        // Accepted when `allow_underside_back_chamfer == true` (horiz_reach ≈ 1.871 mm > 1.5 + 0.15)
        let sol_l = solve_deburr_tool_tip(&s_under, &lolli, 0.5, 0.75, 0.15, true).unwrap();
        let exp_l_center = s_under.pos_wcs + s_under.bisector_out * (3.0 - d_bevel);
        assert!(approx_eq(sol_l.tip_wcs.x, exp_l_center.x, 1e-6));
        assert!(approx_eq(sol_l.tip_wcs.z, exp_l_center.z - 3.0, 1e-6));
        // Rejected when neck is too thick (`r_neck = 2.0 > horiz_reach = 1.871`)
        let thick_lolli = CutterGeometry::LollipopEndmill {
            diameter_mm: 6.0,
            neck_diameter_mm: 4.0,
            neck_length_mm: 18.0,
            flute_length_mm: 5.2,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        assert!(solve_deburr_tool_tip(&s_under, &thick_lolli, 0.5, 0.75, 0.15, true).is_none());
    }
}
