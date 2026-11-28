use std::f64::consts::TAU;

use chip_bvh::TriMesh3D;
use chip_cad::CadDocument;
use chip_geom2d::{PocketRegion2D, RegionSet2D, region_from_convex_hull_2d};
use chip_math::{
    Aabb3, ArcPolyline2D, ArcSegment2D, DMat3, DVec2, DVec3, DisplayUnit, EvalContext,
};

use crate::{
    setup::{ResolvedSetup, Setup},
    wcs::CamError,
};

use super::model::{FixtureBody, FixtureKind, ResolvedFixtureSolid, ViseClampAxis, ViseFixedSide};

/// Resolves all enabled fixtures on `setup` into exact `f64` Setup-WCS [`ResolvedFixtureSolid`]s.
///
/// `lookup_model` resolves `FixtureKind::ImportedModel { model_id, .. }` references to their
/// loaded [`CadDocument`]s (ignoring disabled fixtures).
pub fn resolve_setup_fixtures<'a, F>(
    setup: &Setup,
    resolved: &ResolvedSetup,
    display_unit: DisplayUnit,
    mut lookup_model: F,
) -> Result<Vec<ResolvedFixtureSolid>, CamError>
where
    F: FnMut(&str) -> Option<&'a CadDocument>,
{
    if setup.fixtures.is_empty() {
        return Ok(Vec::new());
    }
    let eval_ctx = resolved.eval_context(display_unit);
    let stock_aabb = resolved.stock.aabb_wcs();
    let mut out = Vec::with_capacity(setup.fixtures.len());
    for fixture in &setup.fixtures {
        if !fixture.enabled {
            continue;
        }
        out.push(resolve_fixture_body(
            fixture,
            &stock_aabb,
            &eval_ctx,
            &mut lookup_model,
        )?);
    }
    Ok(out)
}

/// Convenience wrapper for setups containing only parametric fixtures (no `ImportedModel` lookup).
pub fn resolve_setup_fixtures_parametric(
    setup: &Setup,
    resolved: &ResolvedSetup,
    display_unit: DisplayUnit,
) -> Result<Vec<ResolvedFixtureSolid>, CamError> {
    resolve_setup_fixtures(setup, resolved, display_unit, |_| None)
}

/// Resolves a single [`FixtureBody`] into an exact `f64` Setup-WCS [`ResolvedFixtureSolid`].
pub fn resolve_fixture_body<'a, F>(
    fixture: &FixtureBody,
    stock_aabb: &Aabb3,
    eval_ctx: &EvalContext,
    lookup_model: &mut F,
) -> Result<ResolvedFixtureSolid, CamError>
where
    F: FnMut(&str) -> Option<&'a CadDocument>,
{
    let mut eval_fix = fixture.clone();
    eval_fix.reevaluate(eval_ctx);

    let fname = eval_fix.name.as_str();
    let clearance_margin_mm = validate_non_negative(
        fname,
        "clearance_margin",
        eval_fix.clearance_margin.value_mm,
    )?;
    let allowed_overcut_mm =
        validate_non_negative(fname, "allowed_overcut", eval_fix.allowed_overcut.value_mm)?;

    let (mesh_wcs, footprint_2d) = match &eval_fix.kind {
        FixtureKind::ViseJaws {
            clamp_axis,
            jaw_width,
            jaw_thickness,
            jaw_height,
            grip_depth,
            step_shelf_depth,
            step_shelf_height,
            include_parallels,
            parallel_thickness,
            center_offset_along_jaw,
            fixed_jaw_side,
        } => build_vise_jaws_geometry(
            fname,
            stock_aabb,
            ViseBuildParams {
                clamp_axis: *clamp_axis,
                jaw_width: validate_positive(fname, "jaw_width", jaw_width.value_mm)?,
                jaw_thickness: validate_positive(fname, "jaw_thickness", jaw_thickness.value_mm)?,
                jaw_height: validate_positive(fname, "jaw_height", jaw_height.value_mm)?,
                grip_depth: validate_positive(fname, "grip_depth", grip_depth.value_mm)?,
                step_shelf_depth: validate_non_negative(
                    fname,
                    "step_shelf_depth",
                    step_shelf_depth.value_mm,
                )?,
                step_shelf_height: validate_non_negative(
                    fname,
                    "step_shelf_height",
                    step_shelf_height.value_mm,
                )?,
                include_parallels: *include_parallels,
                parallel_thickness: validate_non_negative(
                    fname,
                    "parallel_thickness",
                    parallel_thickness.value_mm,
                )?,
                center_offset_along_jaw: validate_finite(
                    fname,
                    "center_offset_along_jaw",
                    center_offset_along_jaw.value_mm,
                )?,
                fixed_jaw_side: *fixed_jaw_side,
            },
        )?,
        FixtureKind::ToeClamp {
            anchor_xy,
            z_reference,
            z_offset,
            yaw_deg,
            strap_length,
            strap_width,
            strap_height,
            nose_overhang,
            stud_diameter,
            stud_height_above_strap,
        } => build_toe_clamp_geometry(
            fname,
            stock_aabb,
            ToeClampBuildParams {
                anchor_xy: DVec2::new(
                    validate_finite(fname, "anchor_xy[0]", anchor_xy[0].value_mm)?,
                    validate_finite(fname, "anchor_xy[1]", anchor_xy[1].value_mm)?,
                ),
                z_contact: z_reference.resolve_z_wcs(stock_aabb)
                    + validate_finite(fname, "z_offset", z_offset.value_mm)?,
                yaw_rad: validate_finite(fname, "yaw_deg", *yaw_deg)?.to_radians(),
                strap_length: validate_positive(fname, "strap_length", strap_length.value_mm)?,
                strap_width: validate_positive(fname, "strap_width", strap_width.value_mm)?,
                strap_height: validate_positive(fname, "strap_height", strap_height.value_mm)?,
                nose_overhang: validate_non_negative(
                    fname,
                    "nose_overhang",
                    nose_overhang.value_mm,
                )?,
                stud_diameter: validate_non_negative(
                    fname,
                    "stud_diameter",
                    stud_diameter.value_mm,
                )?,
                stud_height_above_strap: validate_non_negative(
                    fname,
                    "stud_height_above_strap",
                    stud_height_above_strap.value_mm,
                )?,
            },
        )?,
        FixtureKind::Spoilboard {
            z_top_ref,
            z_offset,
            thickness,
            margin_xy,
        } => build_spoilboard_geometry(
            fname,
            stock_aabb,
            z_top_ref.resolve_z_wcs(stock_aabb)
                + validate_finite(fname, "z_offset", z_offset.value_mm)?,
            validate_positive(fname, "thickness", thickness.value_mm)?,
            validate_non_negative(fname, "margin_xy", margin_xy.value_mm)?,
        )?,
        FixtureKind::BoxKeepOut {
            center_wcs,
            size_wcs,
            yaw_deg,
        } => build_box_keepout_geometry(
            DVec3::new(
                validate_finite(fname, "center_wcs[0]", center_wcs[0].value_mm)?,
                validate_finite(fname, "center_wcs[1]", center_wcs[1].value_mm)?,
                validate_finite(fname, "center_wcs[2]", center_wcs[2].value_mm)?,
            ),
            DVec3::new(
                validate_positive(fname, "size_wcs[0]", size_wcs[0].value_mm)?,
                validate_positive(fname, "size_wcs[1]", size_wcs[1].value_mm)?,
                validate_positive(fname, "size_wcs[2]", size_wcs[2].value_mm)?,
            ),
            validate_finite(fname, "yaw_deg", *yaw_deg)?.to_radians(),
        ),
        FixtureKind::CylinderKeepOut {
            center_xy_wcs,
            z_bottom,
            z_top,
            diameter,
        } => {
            let cx = validate_finite(fname, "center_xy_wcs[0]", center_xy_wcs[0].value_mm)?;
            let cy = validate_finite(fname, "center_xy_wcs[1]", center_xy_wcs[1].value_mm)?;
            let zb = validate_finite(fname, "z_bottom", z_bottom.value_mm)?;
            let zt = validate_finite(fname, "z_top", z_top.value_mm)?;
            if zt <= zb + 1e-6 {
                return Err(CamError::InvalidFixture {
                    fixture_name: fname.to_string(),
                    reason: format!(
                        "cylinder z_top ({zt:.3} mm) must exceed z_bottom ({zb:.3} mm)"
                    ),
                });
            }
            let dia = validate_positive(fname, "diameter", diameter.value_mm)?;
            build_cylinder_keepout_geometry(DVec2::new(cx, cy), zb, zt, dia)
        }
        FixtureKind::ImportedModel {
            model_id,
            offset_wcs,
            rotate_deg,
        } => {
            let off = DVec3::new(
                validate_finite(fname, "offset_wcs[0]", offset_wcs[0].value_mm)?,
                validate_finite(fname, "offset_wcs[1]", offset_wcs[1].value_mm)?,
                validate_finite(fname, "offset_wcs[2]", offset_wcs[2].value_mm)?,
            );
            let rot = [
                validate_finite(fname, "rotate_deg[0]", rotate_deg[0])?,
                validate_finite(fname, "rotate_deg[1]", rotate_deg[1])?,
                validate_finite(fname, "rotate_deg[2]", rotate_deg[2])?,
            ];
            let Some(cad_doc) = lookup_model(model_id.as_str()) else {
                return Err(CamError::InvalidFixture {
                    fixture_name: fname.to_string(),
                    reason: format!("referenced fixture model_id '{model_id}' is not loaded"),
                });
            };
            build_imported_model_geometry(fname, cad_doc, off, rot)?
        }
    };

    if mesh_wcs.is_empty() || !mesh_wcs.bounds().is_valid() {
        return Err(CamError::InvalidFixture {
            fixture_name: fname.to_string(),
            reason: "resolved fixture solid contains no valid triangles".to_string(),
        });
    }

    let color_rgba = eval_fix.effective_color_rgba();
    let is_spoilboard = matches!(eval_fix.kind, FixtureKind::Spoilboard { .. });
    let aabb_wcs = mesh_wcs.bounds();
    Ok(ResolvedFixtureSolid {
        fixture_id: eval_fix.id,
        name: eval_fix.name,
        mesh_wcs,
        aabb_wcs,
        footprint_2d,
        clearance_margin_mm,
        allowed_overcut_mm,
        is_spoilboard,
        color_rgba,
    })
}

fn validate_finite(fixture_name: &str, field: &'static str, v: f64) -> Result<f64, CamError> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!("field '{field}' must be finite (got {v})"),
        })
    }
}

fn validate_positive(fixture_name: &str, field: &'static str, v: f64) -> Result<f64, CamError> {
    if v.is_finite() && v > 1e-6 {
        Ok(v)
    } else {
        Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!("field '{field}' must be finite and > 0 (got {v:.4} mm)"),
        })
    }
}

fn validate_non_negative(fixture_name: &str, field: &'static str, v: f64) -> Result<f64, CamError> {
    if v.is_finite() && v >= -1e-9 {
        Ok(v.max(0.0))
    } else {
        Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!("field '{field}' must be finite and >= 0 (got {v:.4} mm)"),
        })
    }
}

struct ViseBuildParams {
    clamp_axis: ViseClampAxis,
    jaw_width: f64,
    jaw_thickness: f64,
    jaw_height: f64,
    grip_depth: f64,
    step_shelf_depth: f64,
    step_shelf_height: f64,
    include_parallels: bool,
    parallel_thickness: f64,
    center_offset_along_jaw: f64,
    fixed_jaw_side: ViseFixedSide,
}

fn build_vise_jaws_geometry(
    fixture_name: &str,
    stock_aabb: &Aabb3,
    p: ViseBuildParams,
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    if !stock_aabb.is_valid() {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: "cannot resolve ViseJaws without valid stock bounds".to_string(),
        });
    }
    if p.jaw_height <= p.grip_depth + 0.05 {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!(
                "jaw_height ({:.2} mm) must exceed grip_depth ({:.2} mm)",
                p.jaw_height, p.grip_depth
            ),
        });
    }

    let stock_c = stock_aabb.center();
    let (clamp_min, clamp_max, along_center) = match p.clamp_axis {
        ViseClampAxis::Y => (
            stock_aabb.min.y,
            stock_aabb.max.y,
            stock_c.x + p.center_offset_along_jaw,
        ),
        ViseClampAxis::X => (
            stock_aabb.min.x,
            stock_aabb.max.x,
            stock_c.y + p.center_offset_along_jaw,
        ),
    };
    let clamp_span = (clamp_max - clamp_min).max(0.0);
    let along_min = along_center - 0.5 * p.jaw_width;
    let along_max = along_center + 0.5 * p.jaw_width;

    let z_stock_bot = stock_aabb.min.z;
    let z_jaw_top = z_stock_bot + p.grip_depth;
    let z_jaw_bot = z_jaw_top - p.jaw_height;

    let has_shelf = p.step_shelf_depth > 1e-4 && p.step_shelf_height > 1e-4;
    let shelf_d = if has_shelf {
        p.step_shelf_depth.min(0.45 * clamp_span)
    } else {
        0.0
    };
    let shelf_bot_z = if has_shelf {
        (z_stock_bot - p.step_shelf_height).max(z_jaw_bot)
    } else {
        z_stock_bot
    };

    let mut mb = MeshBuilder::new();
    let mut fp_sets: Vec<RegionSet2D> = Vec::new();

    // Helper: map `(clamp_coord, along_coord, z)` -> Setup WCS `DVec3`
    let to_wcs = |u_clamp: f64, v_along: f64, z: f64| -> DVec3 {
        match p.clamp_axis {
            ViseClampAxis::Y => DVec3::new(v_along, u_clamp, z),
            ViseClampAxis::X => DVec3::new(u_clamp, v_along, z),
        }
    };
    let rect_fp = |u0: f64, u1: f64, v0: f64, v1: f64| -> RegionSet2D {
        let (x0, x1, y0, y1) = match p.clamp_axis {
            ViseClampAxis::Y => (v0.min(v1), v0.max(v1), u0.min(u1), u0.max(u1)),
            ViseClampAxis::X => (u0.min(u1), u0.max(u1), v0.min(v1), v0.max(v1)),
        };
        region_from_rect(x0, y0, x1, y1)
    };

    // Build Positive (+side) and Negative (-side) jaws
    for (is_pos, face_edge, outward_sign) in
        [(true, clamp_max, 1.0_f64), (false, clamp_min, -1.0_f64)]
    {
        let is_fixed = (is_pos && p.fixed_jaw_side == ViseFixedSide::Positive)
            || (!is_pos && p.fixed_jaw_side == ViseFixedSide::Negative);
        let face_id = if is_fixed { 1 } else { 2 };

        let u_inner = face_edge;
        let u_outer = face_edge + outward_sign * p.jaw_thickness;

        if has_shelf && shelf_d > 1e-4 {
            // Watertight single-solid stepped jaw extruded along `along_min..along_max`.
            // 2D cross-section in `(r, z)` where `r = 0` at `u_inner`, `+` outward, `-` inward:
            let profile_rz: Vec<(f64, f64)> = if shelf_bot_z > z_jaw_bot + 1e-5 {
                vec![
                    (0.0, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_top),
                    (0.0, z_jaw_top),
                    (0.0, z_stock_bot),
                    (-shelf_d, z_stock_bot),
                    (-shelf_d, shelf_bot_z),
                    (0.0, shelf_bot_z),
                ]
            } else {
                vec![
                    (-shelf_d, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_bot),
                    (p.jaw_thickness, z_jaw_top),
                    (0.0, z_jaw_top),
                    (0.0, z_stock_bot),
                    (-shelf_d, z_stock_bot),
                ]
            };
            mb.push_extruded_jaw_profile(
                &profile_rz,
                face_edge,
                outward_sign,
                along_min,
                along_max,
                p.clamp_axis,
                face_id,
            );
            let u_shelf = face_edge - outward_sign * shelf_d;
            fp_sets.push(rect_fp(u_shelf, u_outer, along_min, along_max));
        } else {
            let p_min = to_wcs(u_inner.min(u_outer), along_min, z_jaw_bot);
            let p_max = to_wcs(u_inner.max(u_outer), along_max, z_jaw_top);
            mb.push_aabb_box(p_min, p_max, face_id);
            fp_sets.push(rect_fp(u_inner, u_outer, along_min, along_max));
        }

        // Optional machinist parallels tucked under the stock bottom against each jaw face
        if p.include_parallels && p.parallel_thickness > 1e-4 && z_stock_bot > z_jaw_bot + 0.5 {
            let par_top_z = if has_shelf && shelf_d > 1e-4 {
                shelf_bot_z
            } else {
                z_stock_bot
            };
            if par_top_z > z_jaw_bot + 0.5 {
                let par_t = p.parallel_thickness.min(0.45 * clamp_span);
                let par_u0 = face_edge;
                let par_u1 = face_edge - outward_sign * par_t;
                let p_min = to_wcs(par_u0.min(par_u1), along_min, z_jaw_bot);
                let p_max = to_wcs(par_u0.max(par_u1), along_max, par_top_z);
                mb.push_aabb_box(p_min, p_max, 3);
                fp_sets.push(rect_fp(par_u0, par_u1, along_min, along_max));
            }
        }
    }

    Ok((mb.finish(), RegionSet2D::union_many(&mut fp_sets)))
}

struct ToeClampBuildParams {
    anchor_xy: DVec2,
    z_contact: f64,
    yaw_rad: f64,
    strap_length: f64,
    strap_width: f64,
    strap_height: f64,
    nose_overhang: f64,
    stud_diameter: f64,
    stud_height_above_strap: f64,
}

fn build_toe_clamp_geometry(
    fixture_name: &str,
    stock_aabb: &Aabb3,
    p: ToeClampBuildParams,
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    if p.nose_overhang >= p.strap_length - 1e-4 {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!(
                "nose_overhang ({:.2} mm) must be smaller than strap_length ({:.2} mm)",
                p.nose_overhang, p.strap_length
            ),
        });
    }

    let (sin_y, cos_y) = p.yaw_rad.sin_cos();
    let u_dir = DVec2::new(cos_y, sin_y); // Outward along clamp strap away from stock
    let v_dir = DVec2::new(-sin_y, cos_y);

    let u_front = -p.nose_overhang;
    let u_back = p.strap_length - p.nose_overhang;
    let half_w = 0.5 * p.strap_width;

    let mut mb = MeshBuilder::new();
    let mut fp_sets = Vec::new();

    // 1. Strap bar body (u in [u_front, u_back], v in [-half_w, half_w], z in [z_contact, z_contact + strap_height])
    let strap_center_u = 0.5 * (u_front + u_back);
    let strap_center_xy = p.anchor_xy + u_dir * strap_center_u;
    mb.push_oriented_box(
        DVec3::new(
            strap_center_xy.x,
            strap_center_xy.y,
            p.z_contact + 0.5 * p.strap_height,
        ),
        DVec3::new(p.strap_length, p.strap_width, p.strap_height),
        p.yaw_rad,
        1,
    );
    fp_sets.push(region_from_oriented_rect(
        strap_center_xy,
        p.strap_length,
        p.strap_width,
        p.yaw_rad,
    ));

    // 2. Heel / step-block riser supporting rear end of strap down to stock bottom
    let riser_bot_z = if stock_aabb.is_valid() {
        stock_aabb.min.z
    } else {
        p.z_contact - 15.0
    };
    if p.z_contact > riser_bot_z + 0.5 {
        let heel_len = (p.strap_length * 0.30).clamp(4.0, 18.0).min(u_back);
        if heel_len > 0.5 {
            let heel_u_center = u_back - 0.5 * heel_len;
            let heel_xy = p.anchor_xy + u_dir * heel_u_center;
            let heel_h = p.z_contact - riser_bot_z;
            mb.push_oriented_box(
                DVec3::new(heel_xy.x, heel_xy.y, riser_bot_z + 0.5 * heel_h),
                DVec3::new(heel_len, p.strap_width, heel_h),
                p.yaw_rad,
                2,
            );
        }
    }

    // 3. Clamping T-slot stud + nut tower protruding above strap top
    if p.stud_diameter > 1e-4 && p.stud_height_above_strap > 1e-4 {
        let stud_r = 0.5 * p.stud_diameter;
        let stud_u = (0.45 * u_back).clamp(stud_r.min(0.5 * u_back), (u_back - stud_r).max(0.0));
        let stud_xy = p.anchor_xy + u_dir * stud_u;
        let z0 = p.z_contact + p.strap_height;
        let z1 = z0 + p.stud_height_above_strap;
        mb.push_cylinder_z(stud_xy, z0, z1, stud_r, 24, 3);
        fp_sets.push(RegionSet2D::from_circle(stud_xy, stud_r));
    }

    let _ = (v_dir, half_w);
    Ok((mb.finish(), RegionSet2D::union_many(&mut fp_sets)))
}

fn build_spoilboard_geometry(
    fixture_name: &str,
    stock_aabb: &Aabb3,
    z_top: f64,
    thickness: f64,
    margin_xy: f64,
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    if !stock_aabb.is_valid() {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: "cannot resolve Spoilboard without valid stock bounds".to_string(),
        });
    }
    let min = DVec3::new(
        stock_aabb.min.x - margin_xy,
        stock_aabb.min.y - margin_xy,
        z_top - thickness,
    );
    let max = DVec3::new(
        stock_aabb.max.x + margin_xy,
        stock_aabb.max.y + margin_xy,
        z_top,
    );
    let mut mb = MeshBuilder::new();
    mb.push_aabb_box(min, max, 1);
    let fp = region_from_rect(min.x, min.y, max.x, max.y);
    Ok((mb.finish(), fp))
}

fn build_box_keepout_geometry(
    center_wcs: DVec3,
    size_wcs: DVec3,
    yaw_rad: f64,
) -> (TriMesh3D, RegionSet2D) {
    let mut mb = MeshBuilder::new();
    mb.push_oriented_box(center_wcs, size_wcs, yaw_rad, 1);
    let fp = region_from_oriented_rect(
        DVec2::new(center_wcs.x, center_wcs.y),
        size_wcs.x,
        size_wcs.y,
        yaw_rad,
    );
    (mb.finish(), fp)
}

fn build_cylinder_keepout_geometry(
    center_xy: DVec2,
    z_bottom: f64,
    z_top: f64,
    diameter: f64,
) -> (TriMesh3D, RegionSet2D) {
    let radius = 0.5 * diameter;
    let mut mb = MeshBuilder::new();
    mb.push_cylinder_z(center_xy, z_bottom, z_top, radius, 32, 1);
    let fp = RegionSet2D::from_circle(center_xy, radius);
    (mb.finish(), fp)
}

fn build_imported_model_geometry(
    fixture_name: &str,
    cad_doc: &CadDocument,
    offset_wcs: DVec3,
    rotate_deg: [f64; 3],
) -> Result<(TriMesh3D, RegionSet2D), CamError> {
    let rot_mat = DMat3::from_rotation_z(rotate_deg[2].to_radians())
        * DMat3::from_rotation_y(rotate_deg[1].to_radians())
        * DMat3::from_rotation_x(rotate_deg[0].to_radians());
    let xform = |p: DVec3| rot_mat * p + offset_wcs;

    let mut vertices: Vec<DVec3> = Vec::new();
    let mut indexed_faces: Vec<([u32; 3], u32)> = Vec::new();

    if !cad_doc.meshes.is_empty() {
        for (m_idx, mesh_body) in cad_doc.meshes.iter().enumerate() {
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            vertices.reserve(mesh_body.vertices_mm.len());
            for &v in &mesh_body.vertices_mm {
                vertices.push(xform(v));
            }
            indexed_faces.reserve(mesh_body.triangles.len());
            for tri in &mesh_body.triangles {
                indexed_faces.push((
                    [
                        base + tri.indices[0],
                        base + tri.indices[1],
                        base + tri.indices[2],
                    ],
                    (m_idx + 1) as u32,
                ));
            }
        }
    } else if !cad_doc.render_mesh.indices.is_empty() {
        let rm = &cad_doc.render_mesh;
        vertices.reserve(rm.vertices.len());
        for v in &rm.vertices {
            let p = DVec3::new(
                f64::from(v.position_mm[0]),
                f64::from(v.position_mm[1]),
                f64::from(v.position_mm[2]),
            );
            vertices.push(xform(p));
        }
        indexed_faces.reserve(rm.indices.len() / 3);
        for &[i0, i1, i2] in rm.indices.as_chunks::<3>().0 {
            let fid = rm.vertices.get(i0 as usize).map_or(0, |v| v.face_id);
            indexed_faces.push(([i0, i1, i2], fid));
        }
    }

    if indexed_faces.is_empty() {
        return Err(CamError::InvalidFixture {
            fixture_name: fixture_name.to_string(),
            reason: format!(
                "imported fixture model '{}' contains no triangles",
                cad_doc.source_name
            ),
        });
    }

    let pts_2d: Vec<DVec2> = vertices
        .iter()
        .filter(|v| v.is_finite())
        .map(|v| DVec2::new(v.x, v.y))
        .collect();
    let mesh = TriMesh3D::from_indexed_with_faces(vertices, &indexed_faces);
    let footprint = region_from_convex_hull_2d(&pts_2d).unwrap_or_else(|| {
        let b = mesh.bounds();
        region_from_rect(b.min.x, b.min.y, b.max.x, b.max.y)
    });
    Ok((mesh, footprint))
}

// ============================================================================
// Exact Watertight Parametric Mesh Builder
// ============================================================================

struct MeshBuilder {
    vertices: Vec<DVec3>,
    indexed_faces: Vec<([u32; 3], u32)>,
}

impl MeshBuilder {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indexed_faces: Vec::new(),
        }
    }

    fn push_aabb_box(&mut self, min: DVec3, max: DVec3, face_id: u32) {
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let corners = [
            DVec3::new(min.x, min.y, min.z), // 0
            DVec3::new(max.x, min.y, min.z), // 1
            DVec3::new(max.x, max.y, min.z), // 2
            DVec3::new(min.x, max.y, min.z), // 3
            DVec3::new(min.x, min.y, max.z), // 4
            DVec3::new(max.x, min.y, max.z), // 5
            DVec3::new(max.x, max.y, max.z), // 6
            DVec3::new(min.x, max.y, max.z), // 7
        ];
        self.vertices.extend_from_slice(&corners);
        // Strictly outward-facing CCW winding
        let tris: [[u32; 3]; 12] = [
            // Bottom (-Z)
            [0, 2, 1],
            [0, 3, 2],
            // Top (+Z)
            [4, 5, 6],
            [4, 6, 7],
            // Front (-Y)
            [0, 1, 5],
            [0, 5, 4],
            // Back (+Y)
            [3, 7, 6],
            [3, 6, 2],
            // Left (-X)
            [0, 4, 7],
            [0, 7, 3],
            // Right (+X)
            [1, 2, 6],
            [1, 6, 5],
        ];
        for [a, b, c] in tris {
            self.indexed_faces
                .push(([base + a, base + b, base + c], face_id));
        }
    }

    fn push_oriented_box(&mut self, center: DVec3, size: DVec3, yaw_rad: f64, face_id: u32) {
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let (s, c) = yaw_rad.sin_cos();
        let hx = 0.5 * size.x;
        let hy = 0.5 * size.y;
        let hz = 0.5 * size.z;
        let local = [
            DVec3::new(-hx, -hy, -hz),
            DVec3::new(hx, -hy, -hz),
            DVec3::new(hx, hy, -hz),
            DVec3::new(-hx, hy, -hz),
            DVec3::new(-hx, -hy, hz),
            DVec3::new(hx, -hy, hz),
            DVec3::new(hx, hy, hz),
            DVec3::new(-hx, hy, hz),
        ];
        for lp in local {
            let wp = DVec3::new(
                center.x + lp.x * c - lp.y * s,
                center.y + lp.x * s + lp.y * c,
                center.z + lp.z,
            );
            self.vertices.push(wp);
        }
        let tris: [[u32; 3]; 12] = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        for [a, b, c] in tris {
            self.indexed_faces
                .push(([base + a, base + b, base + c], face_id));
        }
    }

    /// Extrudes a simple CCW 2D cross-section polygon `profile_rz` (in `(r, z)` where
    /// `u_clamp = face_edge + outward_sign * r`) along `along_min..along_max` into a 100%
    /// watertight manifold mesh with strictly outward normals.
    #[allow(clippy::too_many_arguments)]
    fn push_extruded_jaw_profile(
        &mut self,
        profile_rz: &[(f64, f64)],
        face_edge: f64,
        outward_sign: f64,
        along_min: f64,
        along_max: f64,
        clamp_axis: ViseClampAxis,
        face_id: u32,
    ) {
        let n = profile_rz.len();
        if n < 3 {
            return;
        }
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let to_wcs = |r: f64, z: f64, v_along: f64| -> DVec3 {
            let u = face_edge + outward_sign * r;
            match clamp_axis {
                ViseClampAxis::Y => DVec3::new(v_along, u, z),
                ViseClampAxis::X => DVec3::new(u, v_along, z),
            }
        };

        // Determine orientation parity of the mapping `(r, z, v_along) -> (x, y, z)`:
        // - For `ViseClampAxis::Y`: e_r = (0, s, 0), e_z = (0, 0, 1), e_v = (1, 0, 0) -> det = +s.
        // - For `ViseClampAxis::X`: e_r = (s, 0, 0), e_z = (0, 0, 1), e_v = (0, 1, 0) -> det = -s.
        let right_handed = match clamp_axis {
            ViseClampAxis::Y => outward_sign > 0.0,
            ViseClampAxis::X => outward_sign < 0.0,
        };

        // Front ring (`v = along_min`, indices `base..base+n`) and Back ring (`v = along_max`, indices `base+n..base+2n`)
        for &(r, z) in profile_rz {
            self.vertices.push(to_wcs(r, z, along_min));
        }
        for &(r, z) in profile_rz {
            self.vertices.push(to_wcs(r, z, along_max));
        }

        let n_u32 = n as u32;
        // Side quads around extruded perimeter
        for i in 0..n_u32 {
            let j = (i + 1) % n_u32;
            let f0 = base + i;
            let f1 = base + j;
            let b0 = base + n_u32 + i;
            let b1 = base + n_u32 + j;
            if right_handed {
                // Outward normal for CCW `(r, z)` extruded along `+e_v` points along `(e_r_edge) x (+e_v)` -> (f0, f1, b1), (f0, b1, b0)
                self.indexed_faces.push(([f0, f1, b1], face_id));
                self.indexed_faces.push(([f0, b1, b0], face_id));
            } else {
                self.indexed_faces.push(([f0, b1, f1], face_id));
                self.indexed_faces.push(([f0, b0, b1], face_id));
            }
        }

        // End caps via exact ear-clipping triangulation of simple CCW 2D polygon `profile_rz`
        let profile_pts: Vec<DVec2> = profile_rz.iter().map(|&(x, y)| DVec2::new(x, y)).collect();
        let cap_tris = chip_geom2d::ear_clip_ccw_2d(&profile_pts);
        for [i0, i1, i2] in cap_tris {
            let a = i0 as u32;
            let b = i1 as u32;
            let c = i2 as u32;
            if right_handed {
                // Front cap (`along_min`, outward normal `-e_v`) -> CW in `(r, z)`
                self.indexed_faces
                    .push(([base + a, base + c, base + b], face_id));
                // Back cap (`along_max`, outward normal `+e_v`) -> CCW in `(r, z)`
                self.indexed_faces.push((
                    [base + n_u32 + a, base + n_u32 + b, base + n_u32 + c],
                    face_id,
                ));
            } else {
                self.indexed_faces
                    .push(([base + a, base + b, base + c], face_id));
                self.indexed_faces.push((
                    [base + n_u32 + a, base + n_u32 + c, base + n_u32 + b],
                    face_id,
                ));
            }
        }
    }

    fn push_cylinder_z(
        &mut self,
        center_xy: DVec2,
        z_min: f64,
        z_max: f64,
        radius: f64,
        segs: usize,
        face_id: u32,
    ) {
        let segs = segs.max(8);
        let Ok(base) = u32::try_from(self.vertices.len()) else {
            return;
        };
        let n = segs as u32;
        // Bottom ring: base .. base + n
        // Top ring:    base + n .. base + 2*n
        // Bottom ctr:  base + 2*n
        // Top ctr:     base + 2*n + 1
        for k in 0..segs {
            let th = TAU * (k as f64) / (segs as f64);
            let x = center_xy.x + radius * th.cos();
            let y = center_xy.y + radius * th.sin();
            self.vertices.push(DVec3::new(x, y, z_min));
        }
        for k in 0..segs {
            let th = TAU * (k as f64) / (segs as f64);
            let x = center_xy.x + radius * th.cos();
            let y = center_xy.y + radius * th.sin();
            self.vertices.push(DVec3::new(x, y, z_max));
        }
        let c_bot = base + 2 * n;
        let c_top = base + 2 * n + 1;
        self.vertices
            .push(DVec3::new(center_xy.x, center_xy.y, z_min));
        self.vertices
            .push(DVec3::new(center_xy.x, center_xy.y, z_max));

        for k in 0..n {
            let kn = (k + 1) % n;
            let b0 = base + k;
            let b1 = base + kn;
            let t0 = base + n + k;
            let t1 = base + n + kn;
            // Side wall (outward normal)
            self.indexed_faces.push(([b0, b1, t1], face_id));
            self.indexed_faces.push(([b0, t1, t0], face_id));
            // Bottom cap (-Z normal)
            self.indexed_faces.push(([c_bot, b1, b0], face_id));
            // Top cap (+Z normal)
            self.indexed_faces.push(([c_top, t0, t1], face_id));
        }
    }

    fn finish(self) -> TriMesh3D {
        TriMesh3D::from_indexed_with_faces(self.vertices, &self.indexed_faces)
    }
}

// ============================================================================
// Exact 2D Keep-Out Footprint Helpers (`RegionSet2D`)
// ============================================================================

fn region_from_rect(x0: f64, y0: f64, x1: f64, y1: f64) -> RegionSet2D {
    let min_x = x0.min(x1);
    let max_x = x0.max(x1);
    let min_y = y0.min(y1);
    let max_y = y0.max(y1);
    if max_x - min_x <= 1e-5 || max_y - min_y <= 1e-5 {
        return RegionSet2D::empty();
    }
    region_from_ccw_ring(&[
        DVec2::new(min_x, min_y),
        DVec2::new(max_x, min_y),
        DVec2::new(max_x, max_y),
        DVec2::new(min_x, max_y),
    ])
}

fn region_from_oriented_rect(center: DVec2, size_x: f64, size_y: f64, yaw_rad: f64) -> RegionSet2D {
    let (s, c) = yaw_rad.sin_cos();
    let hx = 0.5 * size_x;
    let hy = 0.5 * size_y;
    let rot = |lx: f64, ly: f64| DVec2::new(center.x + lx * c - ly * s, center.y + lx * s + ly * c);
    region_from_ccw_ring(&[rot(-hx, -hy), rot(hx, -hy), rot(hx, hy), rot(-hx, hy)])
}

fn region_from_ccw_ring(pts: &[DVec2]) -> RegionSet2D {
    if pts.len() < 3 {
        return RegionSet2D::empty();
    }
    let mut segs = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        if a.distance_squared(b) > 1e-10 {
            segs.push(ArcSegment2D::Line { start: a, end: b });
        }
    }
    if segs.len() < 3 {
        return RegionSet2D::empty();
    }
    let poly = ArcPolyline2D {
        segments: segs,
        closed: true,
    }
    .ensure_orientation(true);
    RegionSet2D::from_single(PocketRegion2D::new(poly, Vec::new()))
}
