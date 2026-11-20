use chip_cam::{
    ArcDir, ArcPlane, DrillCycleKind, DrillCycleSpec, MotionRole, SegmentMotion3D,
    ToolpathSegment3D, expand_drill_cycle_segments,
};
use chip_math::DVec3;

use crate::{
    modal::{LineWriter, ModalState, collapse_spaces, emit_linear_or_rapid, format_decimal},
    profile::{ArcCenterMode, PostError, PostProfile, PostUnits},
};

#[derive(Debug, Clone, Copy)]
pub(crate) struct ToolChangeEmitSpec<'a> {
    pub suppress_m6: bool,
    pub tool_number: u32,
    pub length_offset: u32,
    pub tool_desc: &'a str,
    pub is_first_tool: bool,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct ArcEmitSpec {
    pub start_wcs: DVec3,
    pub plane: ArcPlane,
    pub dir: ArcDir,
    pub end_wcs: DVec3,
    pub center_offset_ijk: DVec3,
    pub radius_mm: f64,
    pub turns: u32,
    pub feed_mm_min: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct DrillEmitSpec {
    pub cur_wcs: DVec3,
    pub kind: DrillCycleKind,
    pub bottom_wcs: DVec3,
    pub r_plane_z_wcs: f64,
    pub clear_z_wcs: f64,
    pub peck_q_mm: f64,
    pub decreasing_factor: Option<f64>,
    pub min_peck_mm: f64,
    pub chip_break_retract_mm: f64,
    pub dwell_sec: f64,
    pub feed_mm_min: f64,
}

pub(crate) fn handle_tool_change(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    active_tool_num: &mut Option<u32>,
    spec: &ToolChangeEmitSpec<'_>,
) -> Result<(), PostError> {
    let _ = spec.is_first_tool;
    if *active_tool_num == Some(spec.tool_number) {
        return Ok(());
    }
    *active_tool_num = Some(spec.tool_number);

    writer.push_comment_text(&format!("Tool T{}: {}", spec.tool_number, spec.tool_desc))?;

    if spec.suppress_m6 {
        return Ok(());
    }

    if profile.tool_change_block.is_empty() {
        writer.push_code_line(&format!("T{} M6", spec.tool_number))?;
        if profile.use_g43_length_offset {
            writer.push_code_line(&format!("G43 H{}", spec.length_offset))?;
        }
    } else {
        for tmpl in &profile.tool_change_block {
            if tmpl.contains("G43") && !profile.use_g43_length_offset {
                continue;
            }
            let line = tmpl
                .replace("{tool_number}", &spec.tool_number.to_string())
                .replace("{length_offset}", &spec.length_offset.to_string());
            let collapsed = collapse_spaces(&line);
            if !collapsed.is_empty() {
                writer.push_code_line(&collapsed)?;
            }
        }
    }
    modal.force_full_next_move();
    Ok(())
}

pub(crate) fn emit_split_arc_move(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    spec: &ArcEmitSpec,
) -> Result<(), PostError> {
    let center_wcs = spec.start_wcs + spec.center_offset_ijk;
    let signed_sweep_rad = compute_arc_signed_sweep(
        spec.start_wcs,
        spec.end_wcs,
        center_wcs,
        spec.plane,
        spec.dir,
        spec.turns,
    );
    if signed_sweep_rad.abs() <= 1e-8 || spec.radius_mm <= 1e-6 {
        return emit_linear_or_rapid(writer, profile, modal, spec.end_wcs, Some(spec.feed_mm_min));
    }

    let full_seg = ToolpathSegment3D {
        start_wcs: spec.start_wcs,
        end_wcs: spec.end_wcs,
        motion: SegmentMotion3D::Arc {
            plane: spec.plane,
            dir: spec.dir,
            center_wcs,
            radius_mm: spec.radius_mm,
            signed_sweep_rad,
            feed_mm_min: spec.feed_mm_min,
            role: MotionRole::Cut,
        },
        start_rotary_deg: 0.0,
        end_rotary_deg: 0.0,
    };

    let axial_delta = match spec.plane {
        ArcPlane::XY => (spec.end_wcs.z - spec.start_wcs.z).abs(),
        ArcPlane::XZ => (spec.end_wcs.y - spec.start_wcs.y).abs(),
        ArcPlane::YZ => (spec.end_wcs.x - spec.start_wcs.x).abs(),
    };
    let is_helical = axial_delta > 1e-5;

    // Linearize 3D helical arcs to `G1` chords if disabled in profile
    if is_helical && !profile.allow_helical_arcs {
        let tol = profile.helical_linearize_tol_mm.clamp(0.002, 0.2);
        let cos_arg = (1.0 - (tol / spec.radius_mm).clamp(0.0, 0.5)).clamp(-1.0, 1.0);
        let step_rad = (2.0 * cos_arg.acos()).max(0.02);
        let n_chords = ((signed_sweep_rad.abs() / step_rad).ceil() as usize).clamp(4, 4096);
        for k in 1..=n_chords {
            let pt = full_seg.sample_point(k as f64 / n_chords as f64);
            emit_linear_or_rapid(writer, profile, modal, pt, Some(spec.feed_mm_min))?;
        }
        return Ok(());
    }

    // Switch active plane (`G17` / `G18` / `G19`) if needed
    if modal.plane != spec.plane {
        let pcode = match spec.plane {
            ArcPlane::XY => "G17",
            ArcPlane::XZ => "G18",
            ArcPlane::YZ => "G19",
        };
        writer.push_code_line(pcode)?;
        modal.plane = spec.plane;
    }

    let max_sweep_deg = if profile.arc_center_mode == ArcCenterMode::Radius {
        profile.max_arc_sweep_deg.clamp(15.0, 179.0)
    } else {
        profile.max_arc_sweep_deg.clamp(15.0, 360.0)
    };
    let sub_count = ((signed_sweep_rad.abs().to_degrees() / max_sweep_deg).ceil() as usize).max(1);

    let g_arc = match spec.dir {
        ArcDir::CW => "G2",
        ArcDir::CCW => "G3",
    };
    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;

    let mut sub_start = spec.start_wcs;
    for k in 1..=sub_count {
        let sub_end = if k == sub_count {
            spec.end_wcs
        } else {
            full_seg.sample_point(k as f64 / sub_count as f64)
        };

        let xs = format_decimal(profile.units.scale_from_mm(sub_end.x), c_dec, strip);
        let ys = format_decimal(profile.units.scale_from_mm(sub_end.y), c_dec, strip);
        let zs = format_decimal(profile.units.scale_from_mm(sub_end.z), c_dec, strip);

        let mut words: Vec<String> = Vec::with_capacity(8);
        if modal.motion_code != Some(g_arc) {
            words.push(g_arc.to_string());
            modal.motion_code = Some(g_arc);
        }

        // Emit changed coordinates (forcing at least one in-plane axis so G2/G3 is valid on all controllers)
        let mut emitted_coord = false;
        if modal.x_word.as_deref() != Some(xs.as_str()) {
            words.push(format!("X{xs}"));
            modal.x_word = Some(xs.clone());
            emitted_coord = true;
        }
        if modal.y_word.as_deref() != Some(ys.as_str()) {
            words.push(format!("Y{ys}"));
            modal.y_word = Some(ys.clone());
            emitted_coord = true;
        }
        if modal.z_word.as_deref() != Some(zs.as_str()) {
            words.push(format!("Z{zs}"));
            modal.z_word = Some(zs.clone());
            emitted_coord = true;
        }
        if !emitted_coord {
            words.push(format!("X{xs}"));
            modal.x_word = Some(xs);
        }

        match profile.arc_center_mode {
            ArcCenterMode::IncrementalIjk => {
                let ijk = center_wcs - sub_start;
                push_plane_ijk(&mut words, spec.plane, ijk, profile.units, c_dec, strip);
            }
            ArcCenterMode::AbsoluteIjk => {
                push_plane_ijk(
                    &mut words,
                    spec.plane,
                    center_wcs,
                    profile.units,
                    c_dec,
                    strip,
                );
            }
            ArcCenterMode::Radius => {
                let rs = format_decimal(profile.units.scale_from_mm(spec.radius_mm), c_dec, strip);
                words.push(format!("R{rs}"));
            }
        }

        let f_scaled = profile.units.scale_from_mm(spec.feed_mm_min).max(0.1);
        let fs = format_decimal(f_scaled, profile.feed_decimals, strip);
        if modal.f_word.as_deref() != Some(fs.as_str()) {
            words.push(format!("F{fs}"));
            modal.f_word = Some(fs);
        }

        writer.push_code_line(&words.join(" "))?;
        sub_start = sub_end;
    }

    Ok(())
}

pub(crate) fn emit_drill_cycle(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    spec: &DrillEmitSpec,
) -> Result<DVec3, PostError> {
    let has_decreasing_peck = matches!(
        spec.kind,
        DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck
    ) && spec.decreasing_factor.is_some_and(|f| f < 0.999);

    if profile.expand_drill_cycles || has_decreasing_peck {
        if modal.canned_active {
            writer.push_code_line("G80")?;
            modal.canned_active = false;
            modal.force_full_next_move();
        }
        let segs: Vec<ToolpathSegment3D> = expand_drill_cycle_segments(
            spec.cur_wcs,
            DrillCycleSpec {
                kind: spec.kind,
                bottom_wcs: spec.bottom_wcs,
                r_plane_z_wcs: spec.r_plane_z_wcs,
                clear_z_wcs: spec.clear_z_wcs,
                peck_q_mm: spec.peck_q_mm,
                decreasing_factor: spec.decreasing_factor,
                min_peck_mm: spec.min_peck_mm,
                chip_break_retract_mm: spec.chip_break_retract_mm,
                dwell_sec: spec.dwell_sec,
                feed_mm_min: spec.feed_mm_min,
            },
        );
        let mut cur_wcs = spec.cur_wcs;
        let mut emitted_dwell = false;
        for seg in &segs {
            match seg.motion {
                SegmentMotion3D::Rapid => {
                    emit_linear_or_rapid(writer, profile, modal, seg.end_wcs, None)?;
                }
                SegmentMotion3D::Linear {
                    feed_mm_min: f_seg, ..
                } => {
                    emit_linear_or_rapid(writer, profile, modal, seg.end_wcs, Some(f_seg))?;
                    if !emitted_dwell
                        && spec.dwell_sec > 1e-4
                        && (seg.end_wcs.z - spec.bottom_wcs.z).abs() <= 1e-4
                    {
                        writer.push_code_line(&format!(
                            "G4 P{}",
                            format_decimal(spec.dwell_sec, 2, true)
                        ))?;
                        emitted_dwell = true;
                    }
                }
                SegmentMotion3D::Arc { .. } => {}
            }
            cur_wcs = seg.end_wcs;
        }
        Ok(cur_wcs)
    } else {
        emit_native_drill_cycle(writer, profile, modal, spec)?;
        let end_z = spec.clear_z_wcs.max(spec.r_plane_z_wcs);
        Ok(DVec3::new(spec.bottom_wcs.x, spec.bottom_wcs.y, end_z))
    }
}

fn emit_native_drill_cycle(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    spec: &DrillEmitSpec,
) -> Result<(), PostError> {
    let retract_mode = if spec.clear_z_wcs > spec.r_plane_z_wcs + 1e-4 {
        "G98"
    } else {
        "G99"
    };
    let cycle_code = match spec.kind {
        DrillCycleKind::Spot if spec.dwell_sec > 1e-4 => "G82",
        DrillCycleKind::Spot | DrillCycleKind::Simple => "G81",
        DrillCycleKind::ChipBreak => "G73",
        DrillCycleKind::DeepPeck => "G83",
    };

    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;
    let xs = format_decimal(profile.units.scale_from_mm(spec.bottom_wcs.x), c_dec, strip);
    let ys = format_decimal(profile.units.scale_from_mm(spec.bottom_wcs.y), c_dec, strip);
    let zs = format_decimal(profile.units.scale_from_mm(spec.bottom_wcs.z), c_dec, strip);
    let rs = format_decimal(
        profile.units.scale_from_mm(spec.r_plane_z_wcs),
        c_dec,
        strip,
    );
    let fs = format_decimal(
        profile.units.scale_from_mm(spec.feed_mm_min).max(0.1),
        profile.feed_decimals,
        strip,
    );

    let mut words = vec![
        retract_mode.to_string(),
        cycle_code.to_string(),
        format!("X{xs}"),
        format!("Y{ys}"),
        format!("Z{zs}"),
        format!("R{rs}"),
    ];
    if matches!(
        spec.kind,
        DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck
    ) {
        let qs = format_decimal(
            profile.units.scale_from_mm(spec.peck_q_mm.max(0.05)),
            c_dec,
            strip,
        );
        words.push(format!("Q{qs}"));
    }
    if cycle_code == "G82" {
        words.push(format!("P{}", format_decimal(spec.dwell_sec, 2, true)));
    }
    words.push(format!("F{fs}"));

    writer.push_code_line(&words.join(" "))?;
    modal.canned_active = true;
    modal.motion_code = Some(cycle_code);
    modal.x_word = Some(xs);
    modal.y_word = Some(ys);
    modal.z_word = Some(format_decimal(
        profile
            .units
            .scale_from_mm(spec.clear_z_wcs.max(spec.r_plane_z_wcs)),
        c_dec,
        strip,
    ));
    modal.f_word = Some(fs);
    Ok(())
}

fn push_plane_ijk(
    words: &mut Vec<String>,
    plane: ArcPlane,
    ijk_mm: DVec3,
    units: PostUnits,
    c_dec: usize,
    strip: bool,
) {
    match plane {
        ArcPlane::XY => {
            words.push(format!(
                "I{}",
                format_decimal(units.scale_from_mm(ijk_mm.x), c_dec, strip)
            ));
            words.push(format!(
                "J{}",
                format_decimal(units.scale_from_mm(ijk_mm.y), c_dec, strip)
            ));
        }
        ArcPlane::XZ => {
            words.push(format!(
                "I{}",
                format_decimal(units.scale_from_mm(ijk_mm.x), c_dec, strip)
            ));
            words.push(format!(
                "K{}",
                format_decimal(units.scale_from_mm(ijk_mm.z), c_dec, strip)
            ));
        }
        ArcPlane::YZ => {
            words.push(format!(
                "J{}",
                format_decimal(units.scale_from_mm(ijk_mm.y), c_dec, strip)
            ));
            words.push(format!(
                "K{}",
                format_decimal(units.scale_from_mm(ijk_mm.z), c_dec, strip)
            ));
        }
    }
}

fn compute_arc_signed_sweep(
    start_wcs: DVec3,
    end_wcs: DVec3,
    center_wcs: DVec3,
    plane: ArcPlane,
    dir: ArcDir,
    turns: u32,
) -> f64 {
    let (su, sv, eu, ev) = match plane {
        ArcPlane::XY => (
            start_wcs.x - center_wcs.x,
            start_wcs.y - center_wcs.y,
            end_wcs.x - center_wcs.x,
            end_wcs.y - center_wcs.y,
        ),
        ArcPlane::XZ => (
            start_wcs.x - center_wcs.x,
            start_wcs.z - center_wcs.z,
            end_wcs.x - center_wcs.x,
            end_wcs.z - center_wcs.z,
        ),
        ArcPlane::YZ => (
            start_wcs.y - center_wcs.y,
            start_wcs.z - center_wcs.z,
            end_wcs.y - center_wcs.y,
            end_wcs.z - center_wcs.z,
        ),
    };
    let a0 = sv.atan2(su);
    let a1 = ev.atan2(eu);
    let ccw = dir == ArcDir::CCW;
    let mut base = if ccw {
        (a1 - a0).rem_euclid(std::f64::consts::TAU)
    } else {
        (a0 - a1).rem_euclid(std::f64::consts::TAU)
    };
    if base <= 1e-8 && (su - eu).hypot(sv - ev) <= 1e-4 {
        base = std::f64::consts::TAU;
    }
    let total = base + f64::from(turns) * std::f64::consts::TAU;
    if ccw { total } else { -total }
}
