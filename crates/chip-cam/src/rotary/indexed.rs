use chip_math::{Aabb3, DVec3, RigidFrame3};

use crate::{setup::ResolvedSetup, stock::ResolvedStockGeometry};

use super::{ResolvedRotaryConfig, RotaryAxisKind, rotate_point_about_axis};

/// Computes the exact rotary index angle (`deg ∈ [0, 360)`) that rotates `normal_wcs` directly
/// upward to `+Z_wcs` (spindle axis) around `rotary.axis` (`XAxis` or `YAxis`).
///
/// Returns `None` if `rotary.axis == ZAxis` or `normal_wcs` is nearly parallel to the rotary axis.
pub fn compute_index_angle_for_normal(
    normal_wcs: DVec3,
    rotary: &ResolvedRotaryConfig,
) -> Option<f64> {
    if rotary.axis == RotaryAxisKind::ZAxis {
        return None;
    }
    let (e1, e2, _) = rotary.axis.basis();
    let c1 = normal_wcs.dot(e1);
    let c2 = normal_wcs.dot(e2) * rotary.direction.sign();
    if c1.hypot(c2) <= 1e-4 {
        return None;
    }
    let phi_deg = c2.atan2(c1).to_degrees();
    let idx = (-phi_deg).rem_euclid(360.0);
    Some(if idx.abs() < 1e-7 || (idx - 360.0).abs() < 1e-7 {
        0.0
    } else {
        idx
    })
}

/// Constructs a virtual [`ResolvedSetup`] whose WCS frame is rotated by `-angle_deg` around
/// `rotary` so that running any standard 2.5D or 3-axis strategy in that virtual Setup produces
/// coordinates directly in machine WCS for `Indexed3Plus1` machining at `angle_deg`.
pub fn build_indexed_virtual_setup(
    setup: &ResolvedSetup,
    rotary: &ResolvedRotaryConfig,
    angle_deg: f64,
) -> ResolvedSetup {
    let inv_rot_vec = |v: DVec3| {
        rotate_point_about_axis(v, DVec3::ZERO, rotary.axis, -angle_deg, rotary.direction)
            .normalize_or_zero()
    };
    let inv_rot_pt = |p: DVec3| {
        rotate_point_about_axis(
            p,
            rotary.center_wcs,
            rotary.axis,
            -angle_deg,
            rotary.direction,
        )
    };
    let fwd_rot_pt = |p: DVec3| {
        rotate_point_about_axis(
            p,
            rotary.center_wcs,
            rotary.axis,
            angle_deg,
            rotary.direction,
        )
    };

    let vx = setup.frame.local_to_world_dir(inv_rot_vec(DVec3::X));
    let vy = setup.frame.local_to_world_dir(inv_rot_vec(DVec3::Y));
    let vz = setup.frame.local_to_world_dir(inv_rot_vec(DVec3::Z));
    let v_origin_world = setup.frame.local_to_world_point(inv_rot_pt(DVec3::ZERO));

    let virtual_frame = RigidFrame3 {
        origin_mm: v_origin_world,
        x_axis: vx,
        y_axis: vy,
        z_axis: vz,
    };

    // Transform model AABB corners into rotated virtual WCS
    let mut rot_model_aabb = Aabb3::EMPTY;
    if setup.model_aabb_wcs.is_valid() {
        for x in [setup.model_aabb_wcs.min.x, setup.model_aabb_wcs.max.x] {
            for y in [setup.model_aabb_wcs.min.y, setup.model_aabb_wcs.max.y] {
                for z in [setup.model_aabb_wcs.min.z, setup.model_aabb_wcs.max.z] {
                    rot_model_aabb.extend_point(fwd_rot_pt(DVec3::new(x, y, z)));
                }
            }
        }
    }

    // Transform stock geometry into rotated virtual WCS
    let rot_stock = match setup.stock {
        ResolvedStockGeometry::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            aabb_wcs,
        } if axis == rotary.axis.to_stock_axis() => {
            let new_c = fwd_rot_pt(center_wcs);
            let mut new_aabb = Aabb3::EMPTY;
            for x in [aabb_wcs.min.x, aabb_wcs.max.x] {
                for y in [aabb_wcs.min.y, aabb_wcs.max.y] {
                    for z in [aabb_wcs.min.z, aabb_wcs.max.z] {
                        new_aabb.extend_point(fwd_rot_pt(DVec3::new(x, y, z)));
                    }
                }
            }
            ResolvedStockGeometry::Cylinder {
                axis,
                center_wcs: new_c,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                aabb_wcs: new_aabb,
            }
        }
        other => {
            let aabb = other.aabb_wcs();
            let mut new_aabb = Aabb3::EMPTY;
            for x in [aabb.min.x, aabb.max.x] {
                for y in [aabb.min.y, aabb.max.y] {
                    for z in [aabb.min.z, aabb.max.z] {
                        new_aabb.extend_point(fwd_rot_pt(DVec3::new(x, y, z)));
                    }
                }
            }
            ResolvedStockGeometry::Box { aabb_wcs: new_aabb }
        }
    };

    let safe_r_z = rotary.center_wcs.z + rotary.clearance_radius_mm;
    ResolvedSetup {
        frame: virtual_frame,
        model_aabb_wcs: rot_model_aabb,
        stock: rot_stock,
        rotary: Some(*rotary),
        clearance_z_wcs: setup.clearance_z_wcs.max(safe_r_z + 2.0),
        retract_z_wcs: setup.retract_z_wcs.max(safe_r_z),
    }
}
