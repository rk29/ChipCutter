use chip_math::{DVec2, DVec3};
use chip_tools::CutterGeometry;

use super::{DeburrContactSolution, DeburrSample3D};

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
