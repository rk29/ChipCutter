use chip_stock::{RadialCutterProfile, SimColoredVertex};
use chip_tools::{CutterEnvelopeShape, Tool};

/// Revolved 3D triangle mesh for a [`Tool`] (`tip = (0, 0, 0)`, axis = `+Z` in Setup WCS).
#[derive(Debug, Clone, Default)]
pub struct SimToolMesh {
    pub vertices: Vec<SimColoredVertex>,
    pub indices: Vec<u32>,
}

/// Builds a 3D surface-of-revolution mesh for `tool` (`tip = (0, 0, 0)`, spindle up = `+Z`).
///
/// Distinct colors distinguish:
/// 1. **Cutting Flutes / V-Tooth Band** (`0 .. flute_length`)
/// 2. **Relieved Neck & Non-Cutting Shank** (`flute_length .. stickout_length`)
/// 3. **Collet Nut & Holder Cone** (`> stickout_length`)
pub fn build_cutter_assembly_mesh(
    tool: &Tool,
    radial_slices: u32,
    crash_highlight: bool,
) -> SimToolMesh {
    let prof = RadialCutterProfile::from_tool(tool);
    let slices = radial_slices.clamp(12, 96) as usize;

    let (flute_color, shank_color, holder_color) = if crash_highlight {
        (
            [0.98, 0.20, 0.22, 1.0],
            [0.92, 0.34, 0.24, 1.0],
            [0.78, 0.16, 0.18, 1.0],
        )
    } else {
        (
            [0.94, 0.75, 0.24, 1.0], // TiAlN / ZrN gold cutting flutes
            [0.68, 0.73, 0.79, 1.0], // Ground carbide shank / relieved neck
            [0.24, 0.27, 0.32, 1.0], // Anodized collet nut / holder
        )
    };

    let push_station = |pts: &mut Vec<(f64, f64)>, r: f64, z: f64| {
        if pts
            .last()
            .is_none_or(|&(pr, pz)| (r - pr).hypot(z - pz) > 1e-5)
        {
            pts.push((r, z));
        }
    };
    let build_relieved_shank = |r_neck: f64, head_top_z: f64| -> (Vec<(f64, f64)>, f64) {
        let neck_top_z = prof.neck_length_mm.max(head_top_z + 0.2);
        let stickout_z = prof.stickout_length_mm.max(neck_top_z + 0.5);
        let taper_len = ((prof.shank_radius_mm - r_neck).abs() * 1.2).clamp(0.4, 2.5);
        let taper_end_z = (neck_top_z + taper_len).min(stickout_z);
        let shank = vec![
            (r_neck, head_top_z),
            (r_neck, neck_top_z),
            (prof.shank_radius_mm, taper_end_z),
            (prof.shank_radius_mm, stickout_z),
        ];
        (shank, stickout_z)
    };

    let mut flute_pts: Vec<(f64, f64)> = Vec::new();
    let (shank_pts, stickout_z) = match prof.envelope().shape() {
        CutterEnvelopeShape::ThreadMill { .. } => {
            let r_cut = prof.cutting_radius_mm;
            let r_neck = prof.neck_radius_mm;
            let flute_top_z = prof.flute_length_mm;
            let (z_start, tooth_step, n_teeth, tan_half) = prof.thread_mill_tooth_band_params();
            let flank_dz = ((r_cut - r_neck).max(0.0) * tan_half).min(0.5 * tooth_step);

            // Flat cutter tip face out to the lower root radius
            push_station(&mut flute_pts, 0.0, 0.0);
            let r_base = prof.thread_mill_tooth_radius_at_z(0.0);
            push_station(&mut flute_pts, r_base, 0.0);
            if z_start > 1e-5 {
                push_station(&mut flute_pts, r_neck, z_start);
            }

            // Crisp periodic 60° (or `thread_angle_deg`) V-tooth rings across `[0, flute_length_mm]`
            for k in 0..n_teeth {
                let z_lo = z_start + (k as f64) * tooth_step;
                let z_crest = z_lo + 0.5 * tooth_step;
                let z_hi = z_lo + tooth_step;
                let r_lo = (r_cut - (z_crest - z_lo) / tan_half).clamp(r_neck, r_cut);
                let r_hi = (r_cut - (z_hi - z_crest) / tan_half).clamp(r_neck, r_cut);
                let z_flank_lo = z_crest - flank_dz;
                let z_flank_hi = z_crest + flank_dz;

                push_station(&mut flute_pts, r_lo, z_lo);
                if z_flank_lo > z_lo + 1e-5 {
                    push_station(&mut flute_pts, r_neck, z_flank_lo);
                }
                push_station(&mut flute_pts, r_cut, z_crest);
                if z_flank_hi < z_hi - 1e-5 {
                    push_station(&mut flute_pts, r_neck, z_flank_hi);
                }
                push_station(&mut flute_pts, r_hi, z_hi);
            }
            // Step cleanly inward to the relieved neck top shoulder at `z = flute_length_mm`
            push_station(&mut flute_pts, r_neck, flute_top_z);
            build_relieved_shank(r_neck, flute_top_z)
        }
        CutterEnvelopeShape::Lollipop {
            sphere_radius_mm,
            neck_radius_mm,
            z_neck_join_mm,
            ..
        } => {
            // Smooth 16-station spherical head from south pole `(0, 0)` up to neck join `(r_neck, z_join)`
            let r_sphere = sphere_radius_mm.max(1e-4);
            let sin_join = ((z_neck_join_mm - r_sphere) / r_sphere).clamp(-1.0, 1.0);
            let theta_lo = -std::f64::consts::FRAC_PI_2;
            let theta_hi = sin_join.asin();
            let steps = 16usize;
            for i in 0..=steps {
                let u = i as f64 / steps as f64;
                let th = theta_lo + (theta_hi - theta_lo) * u;
                let (s, c) = th.sin_cos();
                let r = if i == 0 {
                    0.0
                } else if i == steps {
                    neck_radius_mm
                } else {
                    (r_sphere * c).max(0.0)
                };
                let z = if i == 0 {
                    0.0
                } else if i == steps {
                    z_neck_join_mm
                } else {
                    (r_sphere + r_sphere * s).clamp(0.0, z_neck_join_mm)
                };
                push_station(&mut flute_pts, r, z);
            }
            build_relieved_shank(neck_radius_mm, z_neck_join_mm)
        }
        CutterEnvelopeShape::SlotMill {
            flat_radius_mm,
            cutting_radius_mm,
            cutting_thickness_mm,
            corner_radius_mm,
            neck_radius_mm,
            ..
        } => {
            let h = cutting_thickness_mm;
            let rc = corner_radius_mm;
            push_station(&mut flute_pts, 0.0, 0.0);
            if rc > 1e-4 {
                push_station(&mut flute_pts, flat_radius_mm, 0.0);
                let arc_steps = 6usize;
                for i in 1..=arc_steps {
                    let th = -std::f64::consts::FRAC_PI_2
                        + std::f64::consts::FRAC_PI_2 * (i as f64 / arc_steps as f64);
                    let (s, c) = th.sin_cos();
                    push_station(&mut flute_pts, flat_radius_mm + rc * c, rc + rc * s);
                }
                if h > 2.0 * rc + 1e-4 {
                    push_station(&mut flute_pts, cutting_radius_mm, h - rc);
                }
                for i in 1..=arc_steps {
                    let th = std::f64::consts::FRAC_PI_2 * (i as f64 / arc_steps as f64);
                    let (s, c) = th.sin_cos();
                    push_station(&mut flute_pts, flat_radius_mm + rc * c, (h - rc) + rc * s);
                }
            } else {
                push_station(&mut flute_pts, cutting_radius_mm, 0.0);
                push_station(&mut flute_pts, cutting_radius_mm, h);
            }
            // Top disc shoulder stepping cleanly inward to the relieved neck
            push_station(&mut flute_pts, neck_radius_mm, h);
            build_relieved_shank(neck_radius_mm, h)
        }
        CutterEnvelopeShape::Dovetail {
            bottom_radius_mm,
            top_radius_mm,
            cutting_height_mm,
            side_angle_rad,
            corner_radius_mm,
            neck_radius_mm,
            ..
        } => {
            let h = cutting_height_mm;
            let rc = corner_radius_mm;
            push_station(&mut flute_pts, 0.0, 0.0);
            if rc > 1e-4 {
                let r_c = (bottom_radius_mm - rc).max(0.0);
                push_station(&mut flute_pts, r_c, 0.0);
                let theta_lo = -std::f64::consts::FRAC_PI_2;
                let theta_hi = (std::f64::consts::FRAC_PI_2 - side_angle_rad).max(0.0);
                let arc_steps = 6usize;
                for i in 1..=arc_steps {
                    let th = theta_lo + (theta_hi - theta_lo) * (i as f64 / arc_steps as f64);
                    let (s, c) = th.sin_cos();
                    push_station(&mut flute_pts, r_c + rc * c, (rc + rc * s).min(h));
                }
            } else {
                push_station(&mut flute_pts, bottom_radius_mm, 0.0);
            }
            // Inverted conical dovetail flank up to top diameter, then step-in to relieved neck
            push_station(&mut flute_pts, top_radius_mm.max(neck_radius_mm), h);
            push_station(&mut flute_pts, neck_radius_mm, h);
            build_relieved_shank(neck_radius_mm, h)
        }
        _ => {
            // Build profile stations `(r_mm, z_mm)` for each zone and revolve cleanly with sharp zone normals
            flute_pts.push((0.0, 0.0));
            let tip_steps = 8usize;
            for i in 1..=tip_steps {
                let r = prof.cutting_radius_mm * (i as f64 / tip_steps as f64);
                let dz = prof
                    .tip_to_envelope_dz(r)
                    .unwrap_or(0.0)
                    .min(prof.flute_length_mm);
                if flute_pts
                    .last()
                    .is_none_or(|&(pr, pz)| (r - pr).hypot(dz - pz) > 1e-4)
                {
                    flute_pts.push((r, dz));
                }
            }
            let tip_top_z = flute_pts.last().map_or(0.0, |p| p.1);
            let flute_top_z = prof.flute_length_mm.max(tip_top_z + 0.2);
            flute_pts.push((prof.cutting_radius_mm, flute_top_z));

            let stickout_z = prof.stickout_length_mm.max(flute_top_z + 0.5);
            let shank = vec![
                (prof.cutting_radius_mm, flute_top_z),
                (prof.shank_radius_mm, (flute_top_z + 0.8).min(stickout_z)),
                (prof.shank_radius_mm, stickout_z),
            ];
            (shank, stickout_z)
        }
    };

    let asm = prof.assembly();
    let nut_r = asm.holder_nut_radius_mm;
    let body_r = asm.holder_body_radius_mm;
    let cone_z0 = stickout_z + (asm.holder_cone_z0 - asm.stickout_length_mm);
    let cone_z1 = stickout_z + (asm.holder_cone_z1 - asm.stickout_length_mm);
    let holder_pts = vec![
        (prof.shank_radius_mm, stickout_z),
        (nut_r, stickout_z + 6.0),
        (nut_r, cone_z0),
        (body_r, cone_z1),
        (body_r, cone_z1 + 14.0),
        (0.0, cone_z1 + 14.0),
    ];

    let mut mesh = SimToolMesh::default();
    append_revolved_strip(&mut mesh, &flute_pts, slices, flute_color);
    append_revolved_strip(&mut mesh, &shank_pts, slices, shank_color);
    append_revolved_strip(&mut mesh, &holder_pts, slices, holder_color);
    mesh
}

fn append_revolved_strip(
    mesh: &mut SimToolMesh,
    pts: &[(f64, f64)],
    slices: usize,
    color_rgba: [f32; 4],
) {
    for seg in pts.windows(2) {
        let (r0, z0) = seg[0];
        let (r1, z1) = seg[1];
        let dr = r1 - r0;
        let dz = z1 - z0;
        let seg_len = dr.hypot(dz);
        if seg_len <= 1e-6 {
            continue;
        }
        // Outward normal in `(r, z)` meridional plane for curve walking bottom -> top: `(dz, -dr) / len`
        let nr = (dz / seg_len) as f32;
        let nz = (-dr / seg_len) as f32;

        let base_idx = mesh.vertices.len() as u32;
        for s in 0..=slices {
            let theta = std::f64::consts::TAU * (s as f64 / slices as f64);
            let (sin_t, cos_t) = theta.sin_cos();
            let c = cos_t as f32;
            let si = sin_t as f32;
            let n = [nr * c, nr * si, nz];

            mesh.vertices.push(SimColoredVertex {
                pos_mm: [(r0 * cos_t) as f32, (r0 * sin_t) as f32, z0 as f32],
                normal: n,
                color_rgba,
            });
            mesh.vertices.push(SimColoredVertex {
                pos_mm: [(r1 * cos_t) as f32, (r1 * sin_t) as f32, z1 as f32],
                normal: n,
                color_rgba,
            });
        }

        for s in 0..(slices as u32) {
            let i0 = base_idx + 2 * s;
            let i1 = i0 + 1;
            let i2 = i0 + 2;
            let i3 = i0 + 3;
            if r0 > 1e-5 {
                mesh.indices.extend_from_slice(&[i0, i2, i1]);
            }
            if r1 > 1e-5 {
                mesh.indices.extend_from_slice(&[i1, i2, i3]);
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DisplayUnit, approx_eq};
    use chip_tools::{
        CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId, default_starter_tool_library,
        starter_undercut_tools,
    };

    use super::*;

    #[test]
    fn starter_library_revolved_cutter_meshes_are_non_empty_and_indexed() {
        let lib = default_starter_tool_library();
        assert_eq!(lib.tools.len(), 10);
        for t in &lib.tools {
            let mesh = build_cutter_assembly_mesh(t, 24, false);
            assert!(!mesh.vertices.is_empty());
            assert_eq!(mesh.indices.len() % 3, 0);
        }
    }

    #[test]
    fn thread_mill_single_point_and_multi_form_revolved_meshes_match_profile() {
        let lib = default_starter_tool_library();
        let sp_tool = &lib.tools[8]; // T9: Ø4.75mm single-point 60° thread mill
        let sp = RadialCutterProfile::from_tool(sp_tool);

        let mf_tool = Tool {
            id: ToolId("tm_multiform".to_string()),
            tool_number: 11,
            length_offset_number: 11,
            diameter_offset_number: 11,
            name: "6mm Multi-Form M8x1.25 Thread Mill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Flood,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::ThreadMill {
                diameter_mm: 6.0,
                pitch_mm: 1.25,
                teeth_count: 4,
                thread_angle_deg: 60.0,
                flute_length_mm: 5.0,
                neck_diameter_mm: 3.6,
                neck_length_mm: 18.0,
                overall_length_mm: 58.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        };
        let mf = RadialCutterProfile::from_tool(&mf_tool);

        for (tool, prof, expected_crests) in [(sp_tool, sp, 1usize), (&mf_tool, mf, 4usize)] {
            for crash_hl in [false, true] {
                let mesh = build_cutter_assembly_mesh(tool, 32, crash_hl);
                assert!(!mesh.vertices.is_empty());
                assert_eq!(mesh.indices.len() % 3, 0);
                for &idx in &mesh.indices {
                    assert!((idx as usize) < mesh.vertices.len());
                }

                let mut max_r_flute = 0.0_f64;
                let mut max_r_neck = 0.0_f64;
                let mut max_r_shank = 0.0_f64;
                let mut crest_z_levels: Vec<i32> = Vec::new();

                for v in &mesh.vertices {
                    let n_len = (f64::from(v.normal[0]).powi(2)
                        + f64::from(v.normal[1]).powi(2)
                        + f64::from(v.normal[2]).powi(2))
                    .sqrt();
                    assert!(
                        (n_len - 1.0).abs() < 1e-3,
                        "vertex normal must be unit length, got {n_len}"
                    );
                    let r = f64::from(v.pos_mm[0]).hypot(f64::from(v.pos_mm[1]));
                    let z = f64::from(v.pos_mm[2]);
                    if (0.0..=prof.flute_length_mm + 1e-4).contains(&z) {
                        max_r_flute = max_r_flute.max(r);
                        if (r - prof.cutting_radius_mm).abs() < 1e-3 {
                            let z_bucket = (z * 100.0).round() as i32;
                            if !crest_z_levels.contains(&z_bucket) {
                                crest_z_levels.push(z_bucket);
                            }
                        }
                    } else if (z - prof.neck_length_mm).abs() < 1e-3 {
                        max_r_neck = max_r_neck.max(r);
                    } else if (z - prof.stickout_length_mm).abs() < 1e-3
                        && r <= prof.shank_radius_mm + 1e-3
                    {
                        max_r_shank = max_r_shank.max(r);
                    }
                }

                assert!(
                    approx_eq(max_r_flute, prof.cutting_radius_mm, 1e-3),
                    "flute band max radius {max_r_flute} != {}",
                    prof.cutting_radius_mm
                );
                assert!(
                    approx_eq(max_r_neck, prof.neck_radius_mm, 1e-3),
                    "relieved neck max radius {max_r_neck} != {}",
                    prof.neck_radius_mm
                );
                assert!(
                    max_r_neck < max_r_flute - 0.5,
                    "relieved neck ({max_r_neck}) must be visibly narrower than tooth crest ({max_r_flute})"
                );
                assert!(
                    approx_eq(max_r_shank, prof.shank_radius_mm, 1e-3),
                    "shank max radius {max_r_shank} != {}",
                    prof.shank_radius_mm
                );
                assert_eq!(
                    crest_z_levels.len(),
                    expected_crests,
                    "expected {expected_crests} distinct V-tooth crest rings, got {crest_z_levels:?}"
                );
            }
        }
    }

    #[test]
    fn all_ten_cutter_geometry_variants_produce_valid_outward_meshes() {
        let mut all_ten = default_starter_tool_library().tools;
        all_ten.extend(starter_undercut_tools());
        assert!(all_ten.len() >= 13);

        let mut seen_kinds = std::collections::BTreeSet::new();
        for tool in &all_ten {
            seen_kinds.insert(tool.geometry.kind_label());
            let mesh = build_cutter_assembly_mesh(tool, 24, false);
            assert!(
                !mesh.vertices.is_empty() && !mesh.indices.is_empty(),
                "mesh for '{}' must be non-empty",
                tool.name
            );
            assert_eq!(mesh.indices.len() % 3, 0);
            for &idx in &mesh.indices {
                assert!((idx as usize) < mesh.vertices.len());
            }
            let mut max_r = 0.0_f32;
            let mut min_z = f32::INFINITY;
            let mut max_z = f32::NEG_INFINITY;
            for v in &mesh.vertices {
                assert!(v.pos_mm.iter().all(|c| c.is_finite()));
                let nlen = (v.normal[0] * v.normal[0]
                    + v.normal[1] * v.normal[1]
                    + v.normal[2] * v.normal[2])
                    .sqrt();
                assert!(
                    (nlen - 1.0).abs() < 1e-3,
                    "non-unit normal on '{}': len={nlen}",
                    tool.name
                );
                let r = v.pos_mm[0].hypot(v.pos_mm[1]);
                if r > 0.25 {
                    let radial_dot = (v.pos_mm[0] * v.normal[0] + v.pos_mm[1] * v.normal[1]) / r;
                    assert!(
                        radial_dot >= -0.05,
                        "inward normal on '{}' at pos={:?}, n={:?}",
                        tool.name,
                        v.pos_mm,
                        v.normal
                    );
                }
                max_r = max_r.max(r);
                min_z = min_z.min(v.pos_mm[2]);
                max_z = max_z.max(v.pos_mm[2]);
            }
            assert!(min_z.abs() < 1e-3, "tip of '{}' must sit at Z=0", tool.name);
            assert!(max_z > tool.stickout_length_mm as f32 + 20.0);
            assert!(max_r >= (0.5 * tool.geometry.cutting_diameter_mm()) as f32 - 0.1);
        }
        assert_eq!(
            seen_kinds.len(),
            10,
            "expected all 10 CutterGeometry variants covered, got {seen_kinds:?}"
        );
    }
}
