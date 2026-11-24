use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use chip_tools::CutterGeometry;

use crate::{
    cl_data::MotionRole,
    params::{ThreadCutDirection, ThreadFormKind, ThreadHandedness, ThreadMillParams},
    selection::ExtractedFeature25D,
    strategies::StrategyExecContext,
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

/// Resolved 3D helical direction and axial traversal sense for a thread milling cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThreadHelixKinematics {
    /// `true` = CCW (`G3`), `false` = CW (`G2`) in the XY plane looking from `+Z`.
    pub ccw: bool,
    /// `true` = Bottom-Up (`+Z` per revolution), `false` = Top-Down (`-Z` per revolution).
    pub bottom_up: bool,
}

impl ThreadHelixKinematics {
    /// Concise human-readable badge (e.g. `"Bottom-Up G3 (Climb)"`).
    pub const fn summary_label(self) -> &'static str {
        match (self.bottom_up, self.ccw) {
            (true, true) => "Bottom-Up G3 (CCW)",
            (false, false) => "Top-Down G2 (CW)",
            (true, false) => "Bottom-Up G2 (CW)",
            (false, true) => "Top-Down G3 (CCW)",
        }
    }
}

/// Resolves exact helical rotation (`G2`/`G3`) and axial sense (`+Z`/`-Z`) for all 8 combinations
/// of `(Internal|External) × (RightHand|LeftHand) × (Climb|Conventional)` with a CW (`M3`) spindle.
#[inline]
pub const fn resolve_thread_helix_kinematics(
    form: ThreadFormKind,
    handedness: ThreadHandedness,
    cut_direction: ThreadCutDirection,
) -> ThreadHelixKinematics {
    // With a CW (M3) spindle:
    // - Internal Climb & External Conventional travel CCW (`G3`)
    // - Internal Conventional & External Climb travel CW (`G2`)
    let ccw = matches!(
        (form, cut_direction),
        (ThreadFormKind::InternalHole, ThreadCutDirection::Climb)
            | (
                ThreadFormKind::ExternalBoss,
                ThreadCutDirection::Conventional
            )
    );
    // Screw-hand geometry:
    // - Right-Hand helix rises (+Z) when rotating CCW and descends (-Z) when rotating CW
    // - Left-Hand helix rises (+Z) when rotating CW and descends (-Z) when rotating CCW
    let bottom_up = match handedness {
        ThreadHandedness::RightHand => ccw,
        ThreadHandedness::LeftHand => !ccw,
    };
    ThreadHelixKinematics { ccw, bottom_up }
}

/// Computes the nominal V-thread minor diameter (`mm`) from major diameter, pitch, and included
/// thread flank angle (`60°` for ISO Metric & Unified UNC/UNF).
#[inline]
pub fn compute_thread_minor_diameter_mm(
    form: ThreadFormKind,
    major_diameter_mm: f64,
    pitch_mm: f64,
    thread_angle_deg: f64,
) -> f64 {
    let half_rad = (0.5 * thread_angle_deg.clamp(29.0, 90.0)).to_radians();
    let fundamental_h = pitch_mm / (2.0 * half_rad.tan().max(1e-6));
    match form {
        // ISO 68-1 / ASME B1.1 internal minor diameter D1 = D - (5/4) * H
        ThreadFormKind::InternalHole => major_diameter_mm - 1.25 * fundamental_h,
        // ISO 68-1 / ASME B1.1 external minor diameter d3 = D - (17/12) * H
        ThreadFormKind::ExternalBoss => major_diameter_mm - (17.0 / 12.0) * fundamental_h,
    }
}

/// Computes centerline-compensated programmed feedrate (`mm/min`) so the cutting edge maintains
/// the target chipload on internal (`F * (D_major - D_tool) / D_major`) and external
/// (`F * (D_minor + D_tool) / D_minor`) circular arcs.
#[inline]
pub fn compute_thread_mill_centerline_feed(
    form: ThreadFormKind,
    edge_feed_mm_min: f64,
    major_diameter_eff_mm: f64,
    minor_diameter_eff_mm: f64,
    tool_diameter_mm: f64,
    compensate: bool,
) -> f64 {
    if !compensate || !edge_feed_mm_min.is_finite() || edge_feed_mm_min <= 0.0 {
        return edge_feed_mm_min.max(1.0);
    }
    let raw_scale = match form {
        ThreadFormKind::InternalHole => {
            if major_diameter_eff_mm > 1e-6 {
                (major_diameter_eff_mm - tool_diameter_mm) / major_diameter_eff_mm
            } else {
                1.0
            }
        }
        ThreadFormKind::ExternalBoss => {
            if minor_diameter_eff_mm > 1e-6 {
                (minor_diameter_eff_mm + tool_diameter_mm) / minor_diameter_eff_mm
            } else {
                1.0
            }
        }
    };
    let clamped_scale = raw_scale.clamp(0.05, 4.0);
    (edge_feed_mm_min * clamped_scale).max(1.0)
}

#[derive(Debug, Clone, Copy)]
struct ResolvedThreadTarget {
    center_wcs: DVec2,
    feature_diameter_mm: f64,
    top_z_wcs: f64,
    bottom_z_wcs: f64,
}

#[derive(Debug, Clone, Copy)]
struct AxialHelixBand {
    z_entry: f64,
    z_helix_start: f64,
    n_turns: u32,
    z_helix_end: f64,
    z_exit: f64,
}

#[derive(Debug, Clone, Copy)]
struct ThreadPassContext {
    center: DVec2,
    r_path: f64,
    r_clear_external: f64,
    pitch: f64,
    kin: ThreadHelixKinematics,
    form: ThreadFormKind,
    cut_feed: f64,
    lead_feed: f64,
    plunge_feed: f64,
    role: MotionRole,
}

/// Generates a **Helical Thread Milling** toolpath (`OperationKind::ThreadMill`).
pub fn generate_thread_mill(
    ctx: &StrategyExecContext<'_>,
    params: &ThreadMillParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    // 1. Validate cutter type is `CutterGeometry::ThreadMill`
    let CutterGeometry::ThreadMill {
        diameter_mm: d_tool,
        neck_length_mm,
        pitch_mm: tool_pitch_mm,
        teeth_count,
        thread_angle_deg,
        ..
    } = ctx.tool.geometry
    else {
        return Err(CamError::IncompatibleTool {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
            tool_name: ctx.tool.name.clone(),
            tool_kind: ctx.tool.geometry.kind_label(),
        });
    };

    // 2. Validate thread parameters
    let major_d = params.major_diameter.value_mm;
    if !major_d.is_finite() || major_d <= 0.1 {
        return Err(CamError::InvalidParameter {
            param: "major_diameter",
            value: major_d,
            reason: "must be finite and > 0.1 mm",
        });
    }
    let pitch = params.pitch.value_mm;
    if !pitch.is_finite() || pitch <= 0.05 {
        return Err(CamError::InvalidParameter {
            param: "pitch",
            value: pitch,
            reason: "must be finite and > 0.05 mm",
        });
    }
    let pd_offset = params.pitch_diameter_offset.value_mm;
    if !pd_offset.is_finite() {
        return Err(CamError::InvalidParameter {
            param: "pitch_diameter_offset",
            value: pd_offset,
            reason: "must be finite",
        });
    }
    let lead_in_clearance = params.lead_in_clearance.value_mm;
    if !lead_in_clearance.is_finite() || lead_in_clearance < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "lead_in_clearance",
            value: lead_in_clearance,
            reason: "must be finite and >= 0 mm",
        });
    }
    if !(1..=8).contains(&params.radial_passes) {
        return Err(CamError::InvalidParameter {
            param: "radial_passes",
            value: f64::from(params.radial_passes),
            reason: "must be between 1 and 8",
        });
    }
    if params.spring_passes > 3 {
        return Err(CamError::InvalidParameter {
            param: "spring_passes",
            value: f64::from(params.spring_passes),
            reason: "must be between 0 and 3",
        });
    }

    // Multi-form pitch match safety invariant (`|tool.pitch_mm - op.pitch| <= 0.02 mm`)
    if teeth_count > 1 && (tool_pitch_mm - pitch).abs() > 0.02 {
        return Err(CamError::InvalidThreadMillGeometry {
            reason: format!(
                "multi-form thread mill pitch ({tool_pitch_mm:.3} mm, {teeth_count} teeth) does not match target thread pitch ({pitch:.3} mm); mismatch exceeds 0.02 mm tolerance"
            ),
        });
    }

    let d_minor_nom =
        compute_thread_minor_diameter_mm(params.form, major_d, pitch, thread_angle_deg);
    let d_major_eff = major_d + pd_offset;
    let d_minor_eff = d_minor_nom + pd_offset;
    if d_minor_eff <= 0.1 || d_major_eff <= d_minor_eff + 1e-4 {
        return Err(CamError::InvalidThreadMillGeometry {
            reason: format!(
                "effective thread diameters are degenerate (major = {d_major_eff:.3} mm, minor = {d_minor_eff:.3} mm for pitch {pitch:.3} mm)"
            ),
        });
    }

    // Internal cutter clearance safety invariant: `D_tool < D_minor` and `D_tool < D_major_eff`
    if params.form == ThreadFormKind::InternalHole
        && (d_tool >= d_minor_nom - 1e-4
            || d_tool >= d_minor_eff - 1e-4
            || d_tool >= d_major_eff - 1e-4)
    {
        return Err(CamError::InvalidThreadMillGeometry {
            reason: format!(
                "thread mill Ø{d_tool:.3} mm is too large to enter internal thread minor Ø{d_minor_nom:.3} mm (effective major Ø{d_major_eff:.3} mm)"
            ),
        });
    }

    // 3. Extract circular hole/boss targets from selection features (deduplicated by center XY)
    let mut targets = extract_thread_targets(&ctx.selection.features);
    if targets.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    if params.optimize_hole_order && targets.len() > 2 {
        sort_targets_nearest_2opt(&mut targets);
    }

    // 4. Compute kinematics, radial pass schedule, and centerline-compensated feedrates
    let kin = resolve_thread_helix_kinematics(params.form, params.handedness, params.cut_direction);
    let cut_feed = compute_thread_mill_centerline_feed(
        params.form,
        ctx.cutting.cutting_feed_mm_min,
        d_major_eff,
        d_minor_eff,
        d_tool,
        params.compensate_centerline_feed,
    );
    let lead_scale = cut_feed / ctx.cutting.cutting_feed_mm_min.max(1e-6);
    let lead_feed = if params.compensate_centerline_feed {
        (ctx.cutting.lead_feed_mm_min * lead_scale)
            .min(cut_feed)
            .max(1.0)
    } else {
        ctx.cutting.lead_feed_mm_min.max(1.0)
    };
    let plunge_feed = ctx.cutting.plunge_feed_mm_min.max(1.0);

    let radial_schedule = build_radial_pass_schedule(
        params.form,
        d_major_eff,
        d_minor_eff,
        d_tool,
        params.radial_passes,
        params.spring_passes,
    )?;
    let r_clear_external = 0.5 * (d_major_eff + d_tool) + lead_in_clearance;
    let dz_lead = match params.form {
        ThreadFormKind::InternalHole => 0.5 * pitch,
        ThreadFormKind::ExternalBoss => 0.25 * pitch,
    };

    let retract_z = ctx.safe_retract_z();
    let z_top_shift = ctx.heights.top_z_wcs - ctx.selection.inferred_top_z_wcs;
    let z_bot_shift = ctx.heights.bottom_z_wcs - ctx.selection.inferred_bottom_z_wcs;

    // 5. Emit verified helical thread paths for every selected hole / boss
    for (idx, target) in targets.iter().enumerate() {
        ctx.check_cancel()?;

        if params.form == ThreadFormKind::InternalHole
            && target.feature_diameter_mm > 1e-4
            && d_tool >= target.feature_diameter_mm - 1e-4
        {
            return Err(CamError::InvalidThreadMillGeometry {
                reason: format!(
                    "thread mill Ø{d_tool:.3} mm cannot enter selected pilot hole Ø{:.3} mm",
                    target.feature_diameter_mm
                ),
            });
        }

        let thread_top_z = target.top_z_wcs + z_top_shift;
        let thread_bottom_z =
            (target.bottom_z_wcs + z_bot_shift) + ctx.passes.stock_to_leave_axial.value_mm;
        let thread_depth = thread_top_z - thread_bottom_z;
        if !thread_depth.is_finite() || thread_depth <= 1e-4 {
            return Err(CamError::InvalidThreadMillGeometry {
                reason: format!(
                    "thread depth ({thread_depth:.3} mm) must be > 0 (top Z = {thread_top_z:.3} mm, bottom Z = {thread_bottom_z:.3} mm)"
                ),
            });
        }
        if thread_depth > neck_length_mm + 1e-4 {
            return Err(CamError::InvalidThreadMillGeometry {
                reason: format!(
                    "thread depth ({thread_depth:.3} mm) exceeds thread mill usable neck length ({neck_length_mm:.3} mm)"
                ),
            });
        }

        let axial_bands = build_axial_bands(
            thread_top_z,
            thread_bottom_z,
            pitch,
            teeth_count,
            dz_lead,
            kin.bottom_up,
        );

        let first_r = radial_schedule[0].0;
        let approach_xy = match params.form {
            ThreadFormKind::InternalHole => target.center_wcs,
            ThreadFormKind::ExternalBoss => {
                external_lead_start_xy(target.center_wcs, first_r, r_clear_external, kin.ccw)
            }
        };

        let first_z_entry = axial_bands[0].z_entry;
        let safe_z = retract_z.max(first_z_entry);
        let init_z = if idx == 0 {
            ctx.heights.clearance_z_wcs.max(safe_z)
        } else {
            safe_z
        };

        if let Some(cur) = builder.current_pos()
            && cur.z < safe_z - 1e-6
        {
            builder.rapid_to(DVec3::new(cur.x, cur.y, safe_z));
        }
        builder.rapid_to(DVec3::new(approach_xy.x, approach_xy.y, init_z));
        let feed_approach_z = ctx.heights.feed_z_wcs.max(first_z_entry).min(init_z);
        if feed_approach_z < init_z - 1e-6 {
            builder.rapid_to(DVec3::new(approach_xy.x, approach_xy.y, feed_approach_z));
        }

        for band in &axial_bands {
            for &(r_path, role) in &radial_schedule {
                ctx.check_cancel()?;
                builder.pass_count += 1;
                let pass_ctx = ThreadPassContext {
                    center: target.center_wcs,
                    r_path,
                    r_clear_external,
                    pitch,
                    kin,
                    form: params.form,
                    cut_feed,
                    lead_feed,
                    plunge_feed,
                    role,
                };
                emit_single_thread_pass(builder, pass_ctx, *band);
            }
        }

        // Retract straight up along safe centerline (Internal) or clearance radius (External)
        if let Some(cur) = builder.current_pos() {
            let exit_retract_z = retract_z.max(cur.z);
            if exit_retract_z > cur.z + 1e-6 {
                builder.rapid_to(DVec3::new(cur.x, cur.y, exit_retract_z));
            }
        }
    }

    Ok(())
}

fn build_radial_pass_schedule(
    form: ThreadFormKind,
    d_major_eff: f64,
    d_minor_eff: f64,
    d_tool: f64,
    radial_passes: u32,
    spring_passes: u32,
) -> Result<Vec<(f64, MotionRole)>, CamError> {
    let (r_start, r_final) = match form {
        ThreadFormKind::InternalHole => {
            let r_s = 0.5 * (d_minor_eff - d_tool);
            let r_f = 0.5 * (d_major_eff - d_tool);
            if r_f <= 0.01 {
                return Err(CamError::InvalidThreadMillGeometry {
                    reason: format!(
                        "internal thread path radius ({r_f:.4} mm) is too small for cutter Ø{d_tool:.3} mm"
                    ),
                });
            }
            (r_s.max(0.01), r_f)
        }
        ThreadFormKind::ExternalBoss => {
            let r_s = 0.5 * (d_major_eff + d_tool);
            let r_f = 0.5 * (d_minor_eff + d_tool);
            (r_s, r_f)
        }
    };

    let n = radial_passes.max(1);
    let mut passes = Vec::with_capacity((n + spring_passes) as usize);
    for k in 1..=n {
        // Equal-area progression `sqrt(k / N)` keeps chip cross-section uniform across V-groove passes
        let frac = (f64::from(k) / f64::from(n)).sqrt();
        let r_k = if k == n {
            r_final
        } else {
            r_start + (r_final - r_start) * frac
        };
        let role = if k == n {
            MotionRole::FinishCut
        } else {
            MotionRole::Cut
        };
        passes.push((r_k, role));
    }
    for _ in 0..spring_passes {
        passes.push((r_final, MotionRole::FinishCut));
    }
    Ok(passes)
}

fn build_axial_bands(
    thread_top_z: f64,
    thread_bottom_z: f64,
    pitch: f64,
    teeth_count: u32,
    dz_lead: f64,
    bottom_up: bool,
) -> Vec<AxialHelixBand> {
    let thread_depth = (thread_top_z - thread_bottom_z).max(pitch);
    if teeth_count <= 1 {
        // Single-point thread mill: one continuous multi-turn helix covering full thread depth
        let band = if bottom_up {
            let z_entry = thread_bottom_z;
            let z_helix_start = z_entry + dz_lead;
            let n_turns = ((thread_top_z - z_helix_start) / pitch).ceil().max(1.0) as u32;
            let z_helix_end = z_helix_start + f64::from(n_turns) * pitch;
            let z_exit = z_helix_end + dz_lead;
            AxialHelixBand {
                z_entry,
                z_helix_start,
                n_turns,
                z_helix_end,
                z_exit,
            }
        } else {
            let z_exit = thread_bottom_z;
            let z_helix_end = z_exit + dz_lead;
            let n_turns = ((thread_top_z - z_helix_end) / pitch).ceil().max(1.0) as u32;
            let z_helix_start = z_helix_end + f64::from(n_turns) * pitch;
            let z_entry = z_helix_start + dz_lead;
            AxialHelixBand {
                z_entry,
                z_helix_start,
                n_turns,
                z_helix_end,
                z_exit,
            }
        };
        return vec![band];
    }

    // Multi-form thread mill (`teeth_count > 1`):
    // - When `teeth_count * pitch >= thread_depth`, 1 full revolution finishes the entire thread!
    // - Otherwise, step axially in exact `teeth_count * pitch` increments so threads remain 100% in phase.
    let active_span = f64::from(teeth_count) * pitch;
    let n_bands = if active_span >= thread_depth - 1e-4 {
        1
    } else {
        (thread_depth / active_span).ceil().max(1.0) as u32
    };

    let mut bands = Vec::with_capacity(n_bands as usize);
    for b in 0..n_bands {
        let band_idx = if bottom_up { b } else { n_bands - 1 - b };
        let base_z = thread_bottom_z + f64::from(band_idx * teeth_count) * pitch;
        if bottom_up {
            let z_entry = base_z;
            let z_helix_start = z_entry + dz_lead;
            let z_helix_end = z_helix_start + pitch;
            let z_exit = z_helix_end + dz_lead;
            bands.push(AxialHelixBand {
                z_entry,
                z_helix_start,
                n_turns: 1,
                z_helix_end,
                z_exit,
            });
        } else {
            let z_exit = base_z;
            let z_helix_end = z_exit + dz_lead;
            let z_helix_start = z_helix_end + pitch;
            let z_entry = z_helix_start + dz_lead;
            bands.push(AxialHelixBand {
                z_entry,
                z_helix_start,
                n_turns: 1,
                z_helix_end,
                z_exit,
            });
        }
    }
    bands
}

#[inline]
fn external_lead_start_xy(center: DVec2, r_path: f64, r_clear: f64, ccw: bool) -> DVec2 {
    let r_lead = (r_clear - r_path).max(0.25);
    let y_sign = if ccw { -1.0 } else { 1.0 };
    DVec2::new(center.x + r_path + r_lead, center.y + y_sign * r_lead)
}

#[inline]
fn external_lead_exit_xy(center: DVec2, r_path: f64, r_clear: f64, ccw: bool) -> DVec2 {
    let r_lead = (r_clear - r_path).max(0.25);
    let y_sign = if ccw { 1.0 } else { -1.0 };
    DVec2::new(center.x + r_path + r_lead, center.y + y_sign * r_lead)
}

fn emit_full_helical_turns(
    builder: &mut ToolpathBuilder,
    p: ThreadPassContext,
    band: AxialHelixBand,
) {
    let z_dir = if p.kin.bottom_up { 1.0 } else { -1.0 };
    let y_dir = if p.kin.ccw { 1.0 } else { -1.0 };
    let q1_xy = DVec2::new(p.center.x, p.center.y + y_dir * p.r_path);
    let q2_xy = DVec2::new(p.center.x - p.r_path, p.center.y);
    let q3_xy = DVec2::new(p.center.x, p.center.y - y_dir * p.r_path);
    let q4_xy = DVec2::new(p.center.x + p.r_path, p.center.y);

    let mut cur_z = band.z_helix_start;
    for turn in 0..band.n_turns {
        let z1 = cur_z + z_dir * 0.25 * p.pitch;
        let z2 = cur_z + z_dir * 0.50 * p.pitch;
        let z3 = cur_z + z_dir * 0.75 * p.pitch;
        let z4 = if turn + 1 == band.n_turns {
            band.z_helix_end
        } else {
            cur_z + z_dir * p.pitch
        };
        for (end_xy, z_end) in [(q1_xy, z1), (q2_xy, z2), (q3_xy, z3), (q4_xy, z4)] {
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(end_xy.x, end_xy.y, z_end),
                center_xy: p.center,
                radius_mm: p.r_path,
                ccw: p.kin.ccw,
                turns: 0,
                feed_mm_min: p.cut_feed,
                role: p.role,
            });
        }
        cur_z = z4;
    }
}

fn emit_single_thread_pass(
    builder: &mut ToolpathBuilder,
    p: ThreadPassContext,
    band: AxialHelixBand,
) {
    let tan_xy = DVec2::new(p.center.x + p.r_path, p.center.y);

    match p.form {
        ThreadFormKind::InternalHole => {
            // Position safely on hole centerline at `z_entry`
            let entry_pt = DVec3::new(p.center.x, p.center.y, band.z_entry);
            if builder
                .current_pos()
                .is_none_or(|cur| cur.distance(entry_pt) > 1e-6)
            {
                builder.linear_to(entry_pt, p.plunge_feed, MotionRole::Plunge);
            }

            // Smooth 180° radial tangent helical arc-in from centerline `(cx, cy)` to `(cx + r_path, cy)`
            let r_lead = 0.5 * p.r_path;
            let c_lead = DVec2::new(p.center.x + r_lead, p.center.y);
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(tan_xy.x, tan_xy.y, band.z_helix_start),
                center_xy: c_lead,
                radius_mm: r_lead,
                ccw: p.kin.ccw,
                turns: 0,
                feed_mm_min: p.lead_feed,
                role: MotionRole::LeadIn,
            });

            // Full 360° helical revolutions (4 exact 90° quadrant arcs per revolution)
            emit_full_helical_turns(builder, p, band);

            // Symmetric 180° radial tangent helical arc-out back to centerline `(cx, cy)`
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(p.center.x, p.center.y, band.z_exit),
                center_xy: c_lead,
                radius_mm: r_lead,
                ccw: p.kin.ccw,
                turns: 0,
                feed_mm_min: p.lead_feed,
                role: MotionRole::LeadOut,
            });
        }
        ThreadFormKind::ExternalBoss => {
            let r_lead = (p.r_clear_external - p.r_path).max(0.25);
            let c_lead = DVec2::new(p.center.x + p.r_path + r_lead, p.center.y);
            let start_xy =
                external_lead_start_xy(p.center, p.r_path, p.r_clear_external, p.kin.ccw);
            let exit_xy = external_lead_exit_xy(p.center, p.r_path, p.r_clear_external, p.kin.ccw);

            // Move outside `r_clear_external` to `start_xy` at `z_entry`
            if let Some(cur) = builder.current_pos()
                && (DVec2::new(cur.x, cur.y).distance(start_xy) > 1e-6)
            {
                builder.linear_to(
                    DVec3::new(start_xy.x, start_xy.y, cur.z),
                    p.lead_feed,
                    MotionRole::LinkTransition,
                );
            }
            let entry_pt = DVec3::new(start_xy.x, start_xy.y, band.z_entry);
            if builder
                .current_pos()
                .is_none_or(|cur| cur.distance(entry_pt) > 1e-6)
            {
                builder.linear_to(entry_pt, p.plunge_feed, MotionRole::Plunge);
            }

            // Smooth 90° external tangent helical arc-in onto `(cx + r_path, cy)`
            // External tangent arc bends opposite to main boss circle to stay strictly outside `r_path`
            let lead_ccw = !p.kin.ccw;
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(tan_xy.x, tan_xy.y, band.z_helix_start),
                center_xy: c_lead,
                radius_mm: r_lead,
                ccw: lead_ccw,
                turns: 0,
                feed_mm_min: p.lead_feed,
                role: MotionRole::LeadIn,
            });

            // Full 360° helical revolutions around external boss
            emit_full_helical_turns(builder, p, band);

            // Symmetric 90° external tangent helical arc-out back outside `r_clear_external`
            builder.arc_xy_to(XyArcMove {
                end_wcs: DVec3::new(exit_xy.x, exit_xy.y, band.z_exit),
                center_xy: c_lead,
                radius_mm: r_lead,
                ccw: lead_ccw,
                turns: 0,
                feed_mm_min: p.lead_feed,
                role: MotionRole::LeadOut,
            });
        }
    }
}

fn extract_circle_from_polyline(poly: &ArcPolyline2D) -> Option<(DVec2, f64)> {
    if !poly.closed || poly.segments.is_empty() {
        return None;
    }
    let mut center_acc = DVec2::ZERO;
    let mut radius_acc = 0.0;
    let mut first_center: Option<DVec2> = None;
    let mut first_radius = 0.0;
    let mut count = 0usize;

    for seg in &poly.segments {
        let ArcSegment2D::Arc { center, radius, .. } = *seg else {
            return None;
        };
        if !center.is_finite() || !radius.is_finite() || radius <= 1e-4 {
            return None;
        }
        match first_center {
            None => {
                first_center = Some(center);
                first_radius = radius;
            }
            Some(c0) => {
                if c0.distance(center) > 2e-2 || (first_radius - radius).abs() > 2e-2 {
                    return None;
                }
            }
        }
        center_acc += center;
        radius_acc += radius;
        count += 1;
    }

    if count == 0 {
        return None;
    }
    let inv = 1.0 / (count as f64);
    Some((center_acc * inv, 2.0 * radius_acc * inv))
}

fn extract_thread_targets(features: &[ExtractedFeature25D]) -> Vec<ResolvedThreadTarget> {
    let mut targets: Vec<ResolvedThreadTarget> = Vec::new();
    let push_dedup = |list: &mut Vec<ResolvedThreadTarget>, cand: ResolvedThreadTarget| {
        if !list
            .iter()
            .any(|existing| existing.center_wcs.distance(cand.center_wcs) <= 1e-3)
        {
            list.push(cand);
        }
    };

    // Priority 1: explicit cylindrical holes (3D B-Rep holes or sketch circles)
    for feat in features {
        if let ExtractedFeature25D::CylindricalHole {
            center_wcs,
            diameter_mm,
            top_z_wcs,
            bottom_z_wcs,
            ..
        } = *feat
        {
            push_dedup(
                &mut targets,
                ResolvedThreadTarget {
                    center_wcs,
                    feature_diameter_mm: diameter_mm,
                    top_z_wcs,
                    bottom_z_wcs,
                },
            );
        }
    }

    // Priority 2: closed circular contours (B-Rep circular loops / external boss edges / sketch circles)
    for feat in features {
        if let ExtractedFeature25D::Contour {
            polyline_wcs,
            min_z_wcs,
            max_z_wcs,
            ..
        } = feat
            && let Some((center_wcs, diameter_mm)) = extract_circle_from_polyline(polyline_wcs)
        {
            push_dedup(
                &mut targets,
                ResolvedThreadTarget {
                    center_wcs,
                    feature_diameter_mm: diameter_mm,
                    top_z_wcs: *max_z_wcs,
                    bottom_z_wcs: *min_z_wcs,
                },
            );
        }
    }

    // Priority 3: circular planar faces (e.g., top face of an external cylindrical boss)
    for feat in features {
        if let ExtractedFeature25D::PlanarFace {
            outer_boundary_wcs,
            face_z_wcs,
            inferred_top_z_wcs,
            adjacent_lower_floor_z_wcs,
            ..
        } = feat
            && let Some((center_wcs, diameter_mm)) =
                extract_circle_from_polyline(outer_boundary_wcs)
        {
            push_dedup(
                &mut targets,
                ResolvedThreadTarget {
                    center_wcs,
                    feature_diameter_mm: diameter_mm,
                    top_z_wcs: face_z_wcs.max(*inferred_top_z_wcs),
                    bottom_z_wcs: adjacent_lower_floor_z_wcs.unwrap_or(*face_z_wcs),
                },
            );
        }
    }

    targets
}

/// Multi-start Nearest-Neighbor + Open-Path 2-Opt TSP hole ordering to minimize rapid travel.
fn sort_targets_nearest_2opt(targets: &mut [ResolvedThreadTarget]) {
    let n = targets.len();
    if n <= 1 {
        return;
    }

    let path_len = |seq: &[ResolvedThreadTarget]| -> f64 {
        seq.windows(2)
            .map(|w| w[0].center_wcs.distance(w[1].center_wcs))
            .sum()
    };

    let starts: Vec<usize> = if n <= 48 {
        (0..n).collect()
    } else {
        let mut min_idx = 0;
        for (idx, h) in targets.iter().enumerate() {
            let cur = targets[min_idx].center_wcs;
            if (h.center_wcs.x + h.center_wcs.y) < (cur.x + cur.y) {
                min_idx = idx;
            }
        }
        vec![0, min_idx]
    };

    let mut best_tour = targets.to_vec();
    let mut best_cost = path_len(&best_tour);

    for start_idx in starts {
        let mut cand = targets.to_vec();
        cand.swap(0, start_idx);
        for i in 0..n - 1 {
            let cur = cand[i].center_wcs;
            let mut best_j = i + 1;
            let mut best_d = cur.distance_squared(cand[i + 1].center_wcs);
            for (j, cand_j) in cand.iter().enumerate().take(n).skip(i + 2) {
                let d = cur.distance_squared(cand_j.center_wcs);
                if d < best_d {
                    best_d = d;
                    best_j = j;
                }
            }
            cand.swap(i + 1, best_j);
        }
        let cost = path_len(&cand);
        if cost + 1e-9 < best_cost {
            best_cost = cost;
            best_tour = cand;
        }
    }
    targets.copy_from_slice(&best_tour);

    for _pass in 0..8 {
        let mut improved = false;
        for k in 1..(n - 1) {
            let old_edge = targets[k].center_wcs.distance(targets[k + 1].center_wcs);
            let new_edge = targets[0].center_wcs.distance(targets[k + 1].center_wcs);
            if old_edge - new_edge > 1e-6 {
                targets[0..=k].reverse();
                improved = true;
            }
        }
        for i in 0..n.saturating_sub(2) {
            for k in (i + 2)..n {
                let a = targets[i].center_wcs;
                let b = targets[i + 1].center_wcs;
                let c = targets[k].center_wcs;
                let gain = if k + 1 < n {
                    let d = targets[k + 1].center_wcs;
                    (a.distance(b) + c.distance(d)) - (a.distance(c) + b.distance(d))
                } else {
                    a.distance(b) - a.distance(c)
                };
                if gain > 1e-6 {
                    targets[i + 1..=k].reverse();
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use chip_cad::CadDocument;
    use chip_math::{Aabb3, DisplayUnit, ParamLength, RigidFrame3, approx_eq};
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{
        CoolantMode, CuttingPreset, SpindleDirection, SurfaceSpeedPresetSpec, Tool, ToolId,
        default_starter_tool_library,
    };

    use super::*;
    use crate::{
        cl_data::ArcDir,
        heights::{HeightReference, HeightSpec},
        operation::{Operation, OperationKind},
        params::StrategyParams,
        selection::GeometrySelectionItem,
        setup::ResolvedSetup,
        stock::ResolvedStockGeometry,
        strategies::{OperationGenRequest, generate_operation_toolpath_with_rest},
        toolpath::SegmentMotion3D,
    };

    fn make_pure_2d_setup() -> ResolvedSetup {
        let aabb = Aabb3::new(DVec3::new(0.0, 0.0, -20.0), DVec3::new(100.0, 100.0, 0.0));
        ResolvedSetup {
            frame: RigidFrame3::IDENTITY,
            model_aabb_wcs: aabb,
            stock: ResolvedStockGeometry::Box { aabb_wcs: aabb },
            rotary: None,
            clearance_z_wcs: 15.0,
            retract_z_wcs: 5.0,
        }
    }

    fn make_circle_sketch(
        centers: &[(f64, f64, f64)],
    ) -> (SketchLayer, Vec<GeometrySelectionItem>) {
        let mut sk = SketchLayer::new("sk_threads", "Thread Centers", "setup_1");
        let mut sels = Vec::new();
        for (idx, &(x, y, d)) in centers.iter().enumerate() {
            let eid = format!("hole_{idx}");
            sk.entities.push(SketchEntityEntry {
                id: eid.clone(),
                name: format!("Hole {idx}"),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::Circle {
                    center_x: ParamLength::from_mm(x),
                    center_y: ParamLength::from_mm(y),
                    center_xy: None,
                    diameter: ParamLength::from_mm(d),
                },
            });
            sels.push(GeometrySelectionItem::SketchEntity {
                sketch_id: sk.id.clone(),
                entity_id: eid,
                reverse: false,
                side_flip: false,
            });
        }
        (sk, sels)
    }

    fn make_multiform_m8_tool(pitch_mm: f64, teeth_count: u32) -> Tool {
        Tool {
            id: ToolId("tm_m8_multi".to_string()),
            tool_number: 11,
            length_offset_number: 11,
            diameter_offset_number: 11,
            name: "5.2mm Multi-Form M8 Thread Mill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Mist,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::ThreadMill {
                diameter_mm: 5.2,
                neck_diameter_mm: 3.2,
                neck_length_mm: 18.0,
                pitch_mm,
                teeth_count,
                thread_angle_deg: 60.0,
                flute_length_mm: pitch_mm * f64::from(teeth_count),
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: vec![CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "al6061",
                name: "Aluminum 6061",
                material: "Aluminum 6061",
                diameter_mm: 5.2,
                flutes: 3,
                surface_speed_m_min: 180.0,
                chipload_mm_tooth: 0.02,
                default_stepdown_mm: 1.25,
                default_stepover_mm: 0.5,
                coolant_override: Some(CoolantMode::Mist),
            })],
        }
    }

    #[test]
    fn resolves_all_eight_helical_kinematics_combinations_with_exact_pitch_and_direction() {
        let lib = default_starter_tool_library();
        let tm_475 = &lib.tools[8]; // T9: Ø4.75mm single-point 60° thread mill
        let doc = CadDocument::empty();
        let setup = make_pure_2d_setup();
        let (sk, sels) = make_circle_sketch(&[(30.0, 40.0, 6.8)]);
        let cancel = AtomicBool::new(false);

        let pitch = 1.25;
        let expected_cases = [
            // (form, handedness, cut_direction, expected_ccw, expected_bottom_up)
            (
                ThreadFormKind::InternalHole,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Climb,
                true,
                true,
            ),
            (
                ThreadFormKind::InternalHole,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Conventional,
                false,
                false,
            ),
            (
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Climb,
                false,
                false,
            ),
            (
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Conventional,
                true,
                true,
            ),
            (
                ThreadFormKind::InternalHole,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Climb,
                true,
                false,
            ),
            (
                ThreadFormKind::InternalHole,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Conventional,
                false,
                true,
            ),
            (
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Climb,
                false,
                true,
            ),
            (
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Conventional,
                true,
                false,
            ),
        ];

        for (form, handedness, cut_direction, exp_ccw, exp_bottom_up) in expected_cases {
            let kin = resolve_thread_helix_kinematics(form, handedness, cut_direction);
            assert_eq!(
                kin.ccw, exp_ccw,
                "{form:?} {handedness:?} {cut_direction:?}"
            );
            assert_eq!(
                kin.bottom_up, exp_bottom_up,
                "{form:?} {handedness:?} {cut_direction:?}"
            );

            let mut op = Operation::new("op_tm", "M8x1.25 Thread", OperationKind::ThreadMill);
            op.tool_id = Some(tm_475.id.clone());
            op.selections = sels.clone();
            op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
            op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -10.0);
            op.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
                form,
                handedness,
                cut_direction,
                standard_preset: Some("M8 x 1.25".to_string()),
                major_diameter: ParamLength::from_mm(8.0),
                pitch: ParamLength::from_mm(pitch),
                pitch_diameter_offset: ParamLength::from_mm(0.0),
                radial_passes: 1,
                spring_passes: 0,
                lead_in_clearance: ParamLength::from_mm(0.5),
                compensate_centerline_feed: true,
                optimize_hole_order: true,
            }));

            let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
                doc: &doc,
                sketches: std::slice::from_ref(&sk),
                setup: &setup,
                op: &op,
                tool: tm_475,
                clearing_tool: None,
                display_unit: DisplayUnit::Millimeter,
                is_first_tool_in_setup: true,
                upstream_rest: &[],
                carryover_stock: None,
                fixtures: &[],
                shared_avoid_regions: &[],
                cancel: &cancel,
            })
            .unwrap();

            let expected_dir = if exp_ccw { ArcDir::CCW } else { ArcDir::CW };
            let expected_signed_pitch_ratio = match handedness {
                ThreadHandedness::RightHand => pitch / std::f64::consts::TAU,
                ThreadHandedness::LeftHand => -pitch / std::f64::consts::TAU,
            };

            let mut main_cut_arcs = 0usize;
            for seg in &tp.segments_wcs {
                // Blind-hole safety: tool tip must never dip below bottom_z (-10.0 mm)
                assert!(
                    seg.start_wcs.z >= -10.0 - 1e-6 && seg.end_wcs.z >= -10.0 - 1e-6,
                    "segment dipped below thread bottom Z: {seg:?}"
                );
                if let SegmentMotion3D::Arc {
                    dir,
                    signed_sweep_rad,
                    role,
                    ..
                } = seg.motion
                {
                    let dz = seg.end_wcs.z - seg.start_wcs.z;
                    if exp_bottom_up {
                        assert!(dz > 0.0, "expected +Z bottom-up helix, got dz={dz}");
                    } else {
                        assert!(dz < 0.0, "expected -Z top-down helix, got dz={dz}");
                    }
                    // Magnitude of pitch ratio matches `pitch / (2*pi)` on EVERY arc (including lead-in/out!)
                    assert!(
                        approx_eq(
                            (dz / signed_sweep_rad).abs(),
                            pitch / std::f64::consts::TAU,
                            1e-6
                        ),
                        "pitch magnitude mismatch: dz={dz}, sweep={signed_sweep_rad}"
                    );
                    if matches!(role, MotionRole::Cut | MotionRole::FinishCut) {
                        main_cut_arcs += 1;
                        assert_eq!(dir, expected_dir);
                        assert!(
                            approx_eq(dz / signed_sweep_rad, expected_signed_pitch_ratio, 1e-6),
                            "signed helix ratio mismatch for {form:?}/{handedness:?}/{cut_direction:?}"
                        );
                    }
                }
            }
            assert!(
                main_cut_arcs >= 16,
                "single-point 10mm deep M8x1.25 should emit >= 8 full turns (16 half-arcs), got {main_cut_arcs}"
            );
        }
    }

    #[test]
    fn radial_multipass_spring_passes_pitch_offset_and_centerline_feed_scaling_are_exact() {
        let lib = default_starter_tool_library();
        let tm_475 = &lib.tools[8]; // Ø4.75mm
        let doc = CadDocument::empty();
        let setup = make_pure_2d_setup();
        let (sk, sels) = make_circle_sketch(&[(20.0, 20.0, 6.8)]);
        let cancel = AtomicBool::new(false);

        let mut op = Operation::new("op_mp", "M8 Multi-Pass", OperationKind::ThreadMill);
        op.tool_id = Some(tm_475.id.clone());
        op.selections = sels;
        op.feed_overrides.cutting_feed_mm_min = Some(1_000.0);
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -8.0);
        op.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
            form: ThreadFormKind::InternalHole,
            handedness: ThreadHandedness::RightHand,
            cut_direction: ThreadCutDirection::Climb,
            standard_preset: Some("M8 x 1.25".to_string()),
            major_diameter: ParamLength::from_mm(8.0),
            pitch: ParamLength::from_mm(1.25),
            pitch_diameter_offset: ParamLength::from_mm(0.06),
            radial_passes: 3,
            spring_passes: 2,
            lead_in_clearance: ParamLength::from_mm(0.5),
            compensate_centerline_feed: true,
            optimize_hole_order: true,
        }));

        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op,
            tool: tm_475,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();

        // 3 radial passes + 2 spring passes = 5 total passes
        assert_eq!(tp.stats.pass_count, 5);

        // Collect distinct cutting radii in order of appearance
        let mut radii = Vec::new();
        let expected_feed = 1_000.0 * (8.06 - 4.75) / 8.06;
        for seg in &tp.segments_wcs {
            if let SegmentMotion3D::Arc {
                radius_mm,
                feed_mm_min,
                role: MotionRole::Cut | MotionRole::FinishCut,
                ..
            } = seg.motion
            {
                assert!(approx_eq(feed_mm_min, expected_feed, 1e-6));
                if radii
                    .last()
                    .is_none_or(|&prev: &f64| (prev - radius_mm).abs() > 1e-6)
                {
                    radii.push(radius_mm);
                }
            }
        }
        // 3 strictly increasing radial steps, then spring passes stay at final radius `(8.06 - 4.75)/2 = 1.655`
        assert_eq!(radii.len(), 3);
        assert!(radii[0] < radii[1] && radii[1] < radii[2]);
        assert!(approx_eq(radii[2], 0.5 * (8.06 - 4.75), 1e-6));

        // Verify external centerline feed compensation scales UP (> edge feed)
        let ext_feed = compute_thread_mill_centerline_feed(
            ThreadFormKind::ExternalBoss,
            1_000.0,
            8.0,
            6.466,
            4.75,
            true,
        );
        assert!(approx_eq(ext_feed, 1_000.0 * (6.466 + 4.75) / 6.466, 1e-6));
        assert!(ext_feed > 1_000.0);
    }

    #[test]
    fn multiform_single_revolution_optimization_and_safety_rejections_work() {
        let doc = CadDocument::empty();
        let setup = make_pure_2d_setup();
        let (sk, sels) = make_circle_sketch(&[(25.0, 25.0, 6.8)]);
        let cancel = AtomicBool::new(false);

        // 1. Multi-form 8-tooth M8x1.25 (active span = 10 mm) on 10 mm deep hole -> 1 single 360° rev (2 half-arcs)!
        let mf_tool = make_multiform_m8_tool(1.25, 8);
        let mut op = Operation::new("op_mf", "M8 Multi-Form", OperationKind::ThreadMill);
        op.tool_id = Some(mf_tool.id.clone());
        op.selections = sels.clone();
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -10.0);
        op.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
            major_diameter: ParamLength::from_mm(8.0),
            pitch: ParamLength::from_mm(1.25),
            ..ThreadMillParams::default()
        }));

        let tp_mf = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op,
            tool: &mf_tool,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();

        let cut_arcs = tp_mf
            .segments_wcs
            .iter()
            .filter(|s| {
                matches!(
                    s.motion,
                    SegmentMotion3D::Arc {
                        role: MotionRole::Cut | MotionRole::FinishCut,
                        ..
                    }
                )
            })
            .count();
        assert_eq!(
            cut_arcs, 4,
            "multi-form covering full depth must complete in exactly 1 revolution (4 quadrant arcs)"
        );

        // 1b. Multi-form with shorter active teeth (`3 teeth * 1.25 mm = 3.75 mm` on `10 mm` deep hole)
        // steps axially in `ceil(10 / 3.75) = 3` bands shifted by exact integer multiples of `pitch`!
        let mf_short = make_multiform_m8_tool(1.25, 3);
        let tp_mf_stepped = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op,
            tool: &mf_short,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();
        assert_eq!(tp_mf_stepped.stats.pass_count, 3);
        let lead_in_zs: Vec<f64> = tp_mf_stepped
            .segments_wcs
            .iter()
            .filter_map(|s| match s.motion {
                SegmentMotion3D::Arc {
                    role: MotionRole::LeadIn,
                    ..
                } => Some(s.end_wcs.z),
                _ => None,
            })
            .collect();
        assert_eq!(lead_in_zs.len(), 3);
        assert!(approx_eq(lead_in_zs[1] - lead_in_zs[0], 3.0 * 1.25, 1e-6));
        assert!(approx_eq(lead_in_zs[2] - lead_in_zs[1], 3.0 * 1.25, 1e-6));

        // 2. Multi-form pitch mismatch (`1.50` vs `1.25`) MUST be rejected!
        let mf_wrong_pitch = make_multiform_m8_tool(1.5, 6);
        let err_pitch = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op,
            tool: &mf_wrong_pitch,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(matches!(
            err_pitch,
            CamError::InvalidThreadMillGeometry { .. }
        ));

        // 3. Non-ThreadMill cutter (FlatEndmill) MUST be rejected!
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];
        let err_tool = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op,
            tool: flat_6mm,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(matches!(err_tool, CamError::IncompatibleTool { .. }));

        // 4. Oversized thread mill (`Ø4.75 mm` in `M5 x 0.8` with minor `Ø4.13 mm`) MUST be rejected!
        let tm_475 = &lib.tools[8];
        let mut op_small = op.clone();
        op_small.tool_id = Some(tm_475.id.clone());
        op_small.strategy = Some(StrategyParams::ThreadMill(ThreadMillParams {
            major_diameter: ParamLength::from_mm(5.0),
            pitch: ParamLength::from_mm(0.8),
            ..ThreadMillParams::default()
        }));
        let err_oversize = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op_small,
            tool: tm_475,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(matches!(
            err_oversize,
            CamError::InvalidThreadMillGeometry { .. }
        ));

        // 5. Thread depth exceeding `neck_length_mm` (`18 mm` vs `16 mm`) MUST be rejected!
        let mut op_deep = op.clone();
        op_deep.tool_id = Some(tm_475.id.clone());
        op_deep.heights.bottom = HeightSpec::new(HeightReference::StockTop, -18.0);
        let err_reach = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op_deep,
            tool: tm_475,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(matches!(
            err_reach,
            CamError::InvalidThreadMillGeometry { .. }
        ));
    }

    #[test]
    fn optimizes_multi_hole_thread_mill_order_and_roundtrips_operation_toml() {
        let lib = default_starter_tool_library();
        let tm_475 = &lib.tools[8];
        let doc = CadDocument::empty();
        let setup = make_pure_2d_setup();
        // Deliberately scrambled 4-hole sequence so 2-opt shortens travel dramatically
        let (sk, sels) = make_circle_sketch(&[
            (10.0, 10.0, 6.8),
            (80.0, 80.0, 6.8),
            (10.0, 80.0, 6.8),
            (80.0, 10.0, 6.8),
        ]);
        let cancel = AtomicBool::new(false);

        let mut op_opt = Operation::new("op_4holes", "4x M8 Thread", OperationKind::ThreadMill);
        op_opt.tool_id = Some(tm_475.id.clone());
        op_opt.selections = sels;
        op_opt.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_opt.heights.bottom = HeightSpec::new(HeightReference::StockTop, -8.0);

        let mut op_raw = op_opt.clone();
        if let Some(StrategyParams::ThreadMill(p)) = &mut op_raw.strategy {
            p.optimize_hole_order = false;
        }

        let tp_opt = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op_opt,
            tool: tm_475,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();
        let tp_raw = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &setup,
            op: &op_raw,
            tool: tm_475,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();

        assert_eq!(tp_opt.stats.pass_count, 4);
        assert!(
            tp_opt.stats.rapid_distance_mm < tp_raw.stats.rapid_distance_mm - 20.0,
            "2-opt ordering should reduce rapid travel: opt={} vs raw={}",
            tp_opt.stats.rapid_distance_mm,
            tp_raw.stats.rapid_distance_mm
        );
    }
}
