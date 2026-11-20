use chip_cam::{
    ClEvent, CutterCompMode, ResolvedRotaryConfig, ResolvedStockGeometry, WorkOffsetSlot,
    compute_inverse_time_feed_g93, compute_rotary_angular_feed_deg_min, resolve_rotary_target_deg,
};
use chip_math::DVec3;
use chip_tools::{CoolantMode, SpindleDirection, Tool};

use crate::{
    cycles_arcs::{
        ArcEmitSpec, DrillEmitSpec, ToolChangeEmitSpec, emit_drill_cycle, emit_split_arc_move,
        handle_tool_change,
    },
    modal::{LineWriter, ModalState, collapse_spaces, emit_linear_or_rapid, format_decimal},
    partition::{PostOutputFile, PostPartition, SetupPostInput, partition_operations},
    profile::{PostError, PostProfile, SwapAxisTarget},
};

/// Converts a Setup's clean operations into one or more dialect-specific G-code files (`07-post-processor.md`).
pub fn post_process_setup(
    input: &SetupPostInput<'_>,
    profile: &PostProfile,
    stem: &str,
) -> Result<Vec<PostOutputFile>, PostError> {
    if input.operations.is_empty() {
        return Err(PostError::NoOperations {
            setup_name: input.setup_name.to_string(),
        });
    }

    let partitions = partition_operations(&input.operations, profile, stem);
    let mut outputs = Vec::with_capacity(partitions.len());

    for part in partitions {
        outputs.push(emit_partition(input, profile, &part)?);
    }

    Ok(outputs)
}

fn emit_partition(
    setup_in: &SetupPostInput<'_>,
    profile: &PostProfile,
    part: &PostPartition<'_>,
) -> Result<PostOutputFile, PostError> {
    let mut writer = LineWriter::new(profile);
    let mut warnings: Vec<String> = Vec::new();

    // Compute partition tool table & estimated cycle time
    let mut tools_used: Vec<&Tool> = Vec::new();
    let mut tool_numbers: Vec<u32> = Vec::new();
    let mut operation_names: Vec<String> = Vec::new();
    let mut est_time_sec = 0.0_f64;
    let mut prev_tnum: Option<u32> = None;

    for item in &part.ops {
        operation_names.push(item.op.name.clone());
        est_time_sec += item.toolpath.stats.total_est_time_sec;
        if let Some(pt) = prev_tnum
            && pt != item.tool.tool_number
        {
            est_time_sec += profile.tool_change_time_sec.max(0.0);
        }
        prev_tnum = Some(item.tool.tool_number);

        if !tool_numbers.contains(&item.tool.tool_number) {
            tool_numbers.push(item.tool.tool_number);
            tools_used.push(item.tool);
        }
    }

    // 1. Emit structured operator Setup-Sheet G-Code Header (`07-post-processor.md` §Structured G-Code Header Summary)
    emit_structured_header(&mut writer, setup_in, profile, &tools_used, est_time_sec)?;

    // Resolve WCS work-offset code (`G54`..`G59.3`)
    let wcs_code = resolve_wcs_code(setup_in.wcs_slot, profile, &mut warnings);
    let blend_code = match profile.path_blend_tol_mm {
        Some(tol_mm) if tol_mm > 0.0 => {
            let scaled = profile.units.scale_from_mm(tol_mm);
            format!(
                "G64 P{}",
                format_decimal(
                    scaled,
                    profile.coord_decimals.max(3),
                    profile.suppress_trailing_zeros
                )
            )
        }
        _ => String::new(),
    };

    // 2. Emit `program_header` template lines
    for tmpl in &profile.program_header {
        let expanded = tmpl
            .replace("{unit_code}", profile.units.g_code())
            .replace("{wcs_code}", &wcs_code)
            .replace("{blend_code}", &blend_code);
        let collapsed = collapse_spaces(&expanded);
        if !collapsed.is_empty() {
            writer.push_code_line(&collapsed)?;
        }
    }

    let mut modal = ModalState::default();
    let mut cur_wcs = DVec3::new(0.0, 0.0, setup_in.clearance_z_wcs);
    let mut cur_rotary_deg = 0.0_f64;
    let mut active_tool_num: Option<u32> = None;

    let effective_rotary_letter = setup_in
        .rotary
        .map(|r| r.axis_letter)
        .unwrap_or(profile.rotary.axis_letter)
        .to_ascii_uppercase();
    let default_wrap_radius_mm = setup_in
        .rotary
        .map_or(20.0, |r| r.stock_outer_radius_mm.max(1.0));
    if let Some(swap) = profile.rotary.swap_linear_axis
        && setup_in.rotary.is_some()
    {
        let target_lbl = match swap {
            SwapAxisTarget::ReplaceY => "Y",
            SwapAxisTarget::ReplaceX => "X",
        };
        warnings.push(format!(
            "Profile '{}' maps rotary angular commands onto linear axis {target_lbl} at wrap radius {:.3} mm.",
            profile.id, default_wrap_radius_mm
        ));
    }

    // 3. Process each operation's typed `ClEvent` stream in order
    for (local_op_idx, item) in part.ops.iter().enumerate() {
        modal.force_full_next_move();
        let mut active_diam_offset: u32 = item.tool.diameter_offset_number;

        // Ensure tool change is emitted if `ClEvent::ToolChange` wasn't explicitly in `events` or handle inside stream
        let has_tc_event = item
            .toolpath
            .events
            .iter()
            .any(|e| matches!(e, ClEvent::ToolChange { .. }));
        if !has_tc_event {
            handle_tool_change(
                &mut writer,
                profile,
                &mut modal,
                &mut active_tool_num,
                &ToolChangeEmitSpec {
                    suppress_m6: part.suppress_m6,
                    tool_number: item.tool.tool_number,
                    length_offset: item.tool.length_offset_number,
                    tool_desc: &item.tool.name,
                    is_first_tool: local_op_idx == 0,
                },
            )?;
        }

        for ev in &item.toolpath.events {
            // Cancel active canned drill cycle (`G80`) before any non-DrillCycle motion/boundary
            if modal.canned_active
                && matches!(
                    ev,
                    ClEvent::RapidMove { .. }
                        | ClEvent::LinearMove { .. }
                        | ClEvent::ArcMove { .. }
                        | ClEvent::ToolChange { .. }
                        | ClEvent::OperationEnd
                        | ClEvent::ProgramEnd
                )
            {
                writer.push_code_line("G80")?;
                modal.canned_active = false;
                modal.force_full_next_move();
            }

            match ev {
                ClEvent::SetupStart { .. } => {}
                ClEvent::OperationStart {
                    op_name,
                    strategy_name,
                    ..
                } => {
                    for tmpl in &profile.operation_header {
                        let line = tmpl
                            .replace("{op_name}", op_name)
                            .replace("{strategy_name}", strategy_name);
                        writer.push_code_line(&line)?;
                    }
                    modal.force_full_next_move();
                }
                ClEvent::ToolChange {
                    tool_number,
                    length_offset,
                    diameter_offset,
                    tool_desc,
                    is_first_tool,
                } => {
                    active_diam_offset = *diameter_offset;
                    handle_tool_change(
                        &mut writer,
                        profile,
                        &mut modal,
                        &mut active_tool_num,
                        &ToolChangeEmitSpec {
                            suppress_m6: part.suppress_m6,
                            tool_number: *tool_number,
                            length_offset: *length_offset,
                            tool_desc,
                            is_first_tool: *is_first_tool && local_op_idx == 0,
                        },
                    )?;
                }
                ClEvent::SpindleStart { rpm, direction } => {
                    let m_dir = match direction {
                        SpindleDirection::Cw => "M3",
                        SpindleDirection::Ccw => "M4",
                    };
                    let rpm_str = format_decimal(
                        rpm.max(0.0),
                        profile.spindle_decimals,
                        profile.suppress_trailing_zeros,
                    );
                    if profile.spindle_start_block.is_empty() {
                        writer.push_code_line(&format!("S{rpm_str} {m_dir}"))?;
                    } else {
                        for tmpl in &profile.spindle_start_block {
                            let line = tmpl
                                .replace("{rpm}", &rpm_str)
                                .replace("{spindle_dir}", m_dir);
                            writer.push_code_line(&collapse_spaces(&line))?;
                        }
                    }
                }
                ClEvent::SpindleStop => {
                    writer.push_code_line("M5")?;
                }
                ClEvent::Coolant { mode } => match mode {
                    CoolantMode::Disabled => {
                        if modal.coolant_on {
                            writer.push_code_line("M9")?;
                            modal.coolant_on = false;
                        }
                    }
                    CoolantMode::Flood => {
                        writer.push_code_line("M8")?;
                        modal.coolant_on = true;
                    }
                    CoolantMode::Mist | CoolantMode::AirBlast => {
                        writer.push_code_line("M7")?;
                        modal.coolant_on = true;
                    }
                },
                ClEvent::CutterComp { mode } => {
                    if profile.use_cutter_comp_g41_g42 && *mode != modal.cutter_comp {
                        let code = match mode {
                            CutterCompMode::Off => "G40".to_string(),
                            CutterCompMode::WearLeft => format!("G41 D{active_diam_offset}"),
                            CutterCompMode::WearRight => format!("G42 D{active_diam_offset}"),
                        };
                        writer.push_code_line(&code)?;
                        modal.cutter_comp = *mode;
                    }
                }
                ClEvent::RapidMove { end_wcs } => {
                    ensure_g94_mode(&mut writer, &mut modal)?;
                    emit_linear_or_rapid(&mut writer, profile, &mut modal, *end_wcs, None)?;
                    cur_wcs = *end_wcs;
                }
                ClEvent::LinearMove {
                    end_wcs,
                    feed_mm_min,
                    ..
                } => {
                    ensure_g94_mode(&mut writer, &mut modal)?;
                    emit_linear_or_rapid(
                        &mut writer,
                        profile,
                        &mut modal,
                        *end_wcs,
                        Some(*feed_mm_min),
                    )?;
                    cur_wcs = *end_wcs;
                }
                ClEvent::ArcMove {
                    plane,
                    dir,
                    end_wcs,
                    center_offset_ijk,
                    radius_mm,
                    turns,
                    feed_mm_min,
                    ..
                } => {
                    ensure_g94_mode(&mut writer, &mut modal)?;
                    emit_split_arc_move(
                        &mut writer,
                        profile,
                        &mut modal,
                        &ArcEmitSpec {
                            start_wcs: cur_wcs,
                            plane: *plane,
                            dir: *dir,
                            end_wcs: *end_wcs,
                            center_offset_ijk: *center_offset_ijk,
                            radius_mm: *radius_mm,
                            turns: *turns,
                            feed_mm_min: *feed_mm_min,
                        },
                    )?;
                    cur_wcs = *end_wcs;
                }
                ClEvent::DrillCycle {
                    kind,
                    bottom_wcs,
                    r_plane_z_wcs,
                    clear_z_wcs,
                    peck_q_mm,
                    decreasing_factor,
                    min_peck_mm,
                    chip_break_retract_mm,
                    dwell_sec,
                    feed_mm_min,
                } => {
                    ensure_g94_mode(&mut writer, &mut modal)?;
                    cur_wcs = emit_drill_cycle(
                        &mut writer,
                        profile,
                        &mut modal,
                        &DrillEmitSpec {
                            cur_wcs,
                            kind: *kind,
                            bottom_wcs: *bottom_wcs,
                            r_plane_z_wcs: *r_plane_z_wcs,
                            clear_z_wcs: *clear_z_wcs,
                            peck_q_mm: *peck_q_mm,
                            decreasing_factor: *decreasing_factor,
                            min_peck_mm: *min_peck_mm,
                            chip_break_retract_mm: *chip_break_retract_mm,
                            dwell_sec: *dwell_sec,
                            feed_mm_min: *feed_mm_min,
                        },
                    )?;
                }
                ClEvent::RotaryIndex {
                    angle_deg,
                    rapid,
                    feed_deg_min,
                } => {
                    let target_deg = resolve_rotary_target_deg(
                        cur_rotary_deg,
                        *angle_deg,
                        effective_unwind_mode(setup_in.rotary.as_ref(), profile),
                    );
                    emit_rotary_index(
                        &mut writer,
                        profile,
                        &mut modal,
                        effective_rotary_letter,
                        cur_wcs,
                        target_deg,
                        *rapid,
                        *feed_deg_min,
                        default_wrap_radius_mm,
                    )?;
                    cur_rotary_deg = target_deg;
                }
                ClEvent::RotaryRapidMove {
                    end_wcs,
                    rotary_deg,
                } => {
                    let target_deg = resolve_rotary_target_deg(
                        cur_rotary_deg,
                        *rotary_deg,
                        effective_unwind_mode(setup_in.rotary.as_ref(), profile),
                    );
                    emit_rotary_motion(
                        &mut writer,
                        profile,
                        &mut modal,
                        effective_rotary_letter,
                        cur_wcs,
                        *end_wcs,
                        cur_rotary_deg,
                        target_deg,
                        None,
                        default_wrap_radius_mm,
                    )?;
                    cur_wcs = *end_wcs;
                    cur_rotary_deg = target_deg;
                }
                ClEvent::RotaryLinearMove {
                    end_wcs,
                    rotary_deg,
                    feed_mm_min,
                    effective_radius_mm,
                    ..
                } => {
                    let target_deg = resolve_rotary_target_deg(
                        cur_rotary_deg,
                        *rotary_deg,
                        effective_unwind_mode(setup_in.rotary.as_ref(), profile),
                    );
                    let eff_r = if *effective_radius_mm > 1e-4 {
                        *effective_radius_mm
                    } else {
                        default_wrap_radius_mm
                    };
                    emit_rotary_motion(
                        &mut writer,
                        profile,
                        &mut modal,
                        effective_rotary_letter,
                        cur_wcs,
                        *end_wcs,
                        cur_rotary_deg,
                        target_deg,
                        Some(*feed_mm_min),
                        eff_r,
                    )?;
                    cur_wcs = *end_wcs;
                    cur_rotary_deg = target_deg;
                }
                ClEvent::Dwell { seconds } => {
                    if *seconds > 1e-4 {
                        writer.push_code_line(&format!(
                            "G4 P{}",
                            format_decimal(*seconds, 2, true)
                        ))?;
                    }
                }
                ClEvent::Comment(text) => {
                    writer.push_comment_text(text)?;
                }
                ClEvent::OperationEnd => {
                    if profile.use_cutter_comp_g41_g42 && modal.cutter_comp != CutterCompMode::Off {
                        writer.push_code_line("G40")?;
                        modal.cutter_comp = CutterCompMode::Off;
                    }
                    for line in &profile.operation_footer {
                        if line.trim() == "G80" && !modal.canned_active {
                            continue;
                        }
                        writer.push_code_line(line)?;
                    }
                }
                ClEvent::ProgramEnd => {}
            }
        }
    }

    if modal.canned_active {
        writer.push_code_line("G80")?;
        modal.canned_active = false;
    }
    if modal.feed_mode_g93
        && !profile
            .program_footer
            .iter()
            .any(|l| l.split_whitespace().any(|w| w == "G94"))
    {
        writer.push_code_line("G94")?;
        modal.feed_mode_g93 = false;
    }
    if modal.coolant_on
        && !profile
            .program_footer
            .iter()
            .any(|l| l.split_whitespace().any(|w| w == "M9"))
    {
        writer.push_code_line("M9")?;
    }

    // 4. Emit `program_footer`
    for line in &profile.program_footer {
        if line.contains("G53") && !profile.safe_home_z_on_end {
            continue;
        }
        writer.push_code_line(line)?;
    }

    let (content, line_count, max_line_chars) = writer.finish();
    Ok(PostOutputFile {
        filename: part.filename.clone(),
        content,
        line_count,
        max_line_chars,
        tool_numbers,
        operation_names,
        est_time_sec,
        warnings,
    })
}

fn emit_structured_header(
    writer: &mut LineWriter<'_>,
    setup_in: &SetupPostInput<'_>,
    profile: &PostProfile,
    tools_used: &[&Tool],
    est_time_sec: f64,
) -> Result<(), PostError> {
    let u_lbl = profile.units.unit_suffix();
    let c_dec = profile.coord_decimals;

    let stock_desc = match setup_in.stock {
        ResolvedStockGeometry::Box { aabb_wcs } => {
            let sz = aabb_wcs.size();
            format!(
                "X={}  Y={}  Z={} {u_lbl}",
                format_decimal(profile.units.scale_from_mm(sz.x), c_dec, false),
                format_decimal(profile.units.scale_from_mm(sz.y), c_dec, false),
                format_decimal(profile.units.scale_from_mm(sz.z), c_dec, false),
            )
        }
        ResolvedStockGeometry::Cylinder {
            axis,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            ..
        } => {
            let dia = 2.0 * radius_mm;
            let len = (max_axial_mm - min_axial_mm).abs();
            format!(
                "Cyl({:?}) D={}  L={} {u_lbl}",
                axis,
                format_decimal(profile.units.scale_from_mm(dia), c_dec, false),
                format_decimal(profile.units.scale_from_mm(len), c_dec, false),
            )
        }
    };

    let safe_z_desc = format!(
        "Clearance={}  Retract={} {u_lbl}",
        format_decimal(
            profile.units.scale_from_mm(setup_in.clearance_z_wcs),
            c_dec,
            false
        ),
        format_decimal(
            profile.units.scale_from_mm(setup_in.retract_z_wcs),
            c_dec,
            false
        ),
    );

    let total_sec = est_time_sec.max(0.0).round() as u64;
    let hms = format!(
        "{:02}:{:02}:{:02}",
        total_sec / 3600,
        (total_sec % 3600) / 60,
        total_sec % 60
    );

    let sep = "==========================================================";
    writer.push_comment_text(sep)?;
    writer.push_comment_text("Generated by ChipCutter v0.1.0")?;
    writer.push_comment_text(&format!("Project:      {}", setup_in.project_name))?;
    writer.push_comment_text(&format!(
        "Setup:        {} [WCS: {}]",
        setup_in.setup_name,
        setup_in.wcs_slot.short_code()
    ))?;
    writer.push_comment_text(&format!(
        "WCS Zero:     {}",
        setup_in.wcs_origin_description
    ))?;
    writer.push_comment_text(&format!("Stock:        {stock_desc}"))?;
    if let Some(rot) = setup_in.rotary {
        let mode_str = rot.mode.label();
        let clr_r = format_decimal(
            profile.units.scale_from_mm(rot.clearance_radius_mm),
            c_dec,
            true,
        );
        writer.push_comment_text(&format!(
            "Rotary 4th:   {}-Axis ({}) | {} | Clearance R={clr_r}{u_lbl}",
            rot.axis_letter,
            rot.axis.label(),
            mode_str
        ))?;
    }
    writer.push_comment_text(&format!("Safe Z:       {safe_z_desc}"))?;
    writer.push_comment_text("Tools Used:")?;
    for tool in tools_used {
        let dia_mm = tool.geometry.cutting_diameter_mm();
        let stick_mm = tool.stickout_length_mm;
        let t_line = format!(
            "  T{}: {}{} {} [Stickout: {}{}]",
            tool.tool_number,
            format_decimal(profile.units.scale_from_mm(dia_mm), c_dec, true),
            u_lbl,
            tool.name,
            format_decimal(profile.units.scale_from_mm(stick_mm), 1, true),
            u_lbl
        );
        writer.push_comment_text(&t_line)?;
    }
    writer.push_comment_text(&format!("Est. Cycle:   {hms}"))?;
    writer.push_comment_text(sep)?;
    Ok(())
}

fn resolve_wcs_code(
    slot: WorkOffsetSlot,
    profile: &PostProfile,
    warnings: &mut Vec<String>,
) -> String {
    if !profile.emit_wcs_code || slot == WorkOffsetSlot::Default {
        return String::new();
    }
    match slot {
        WorkOffsetSlot::Default => String::new(),
        WorkOffsetSlot::G54
        | WorkOffsetSlot::G55
        | WorkOffsetSlot::G56
        | WorkOffsetSlot::G57
        | WorkOffsetSlot::G58
        | WorkOffsetSlot::G59 => slot.short_code().to_string(),
        WorkOffsetSlot::G59_1 | WorkOffsetSlot::G59_2 | WorkOffsetSlot::G59_3 => {
            if profile.allow_extended_wcs {
                slot.short_code().to_string()
            } else {
                warnings.push(format!(
                    "Profile '{}' does not support extended WCS '{}'; falling back to G59.",
                    profile.id,
                    slot.short_code()
                ));
                "G59".to_string()
            }
        }
    }
}

fn effective_unwind_mode(
    setup_rotary: Option<&ResolvedRotaryConfig>,
    profile: &PostProfile,
) -> chip_cam::RotaryUnwindMode {
    setup_rotary
        .map(|r| r.unwind_mode)
        .unwrap_or(profile.rotary.unwind_mode)
}

fn ensure_g94_mode(writer: &mut LineWriter<'_>, modal: &mut ModalState) -> Result<(), PostError> {
    if modal.feed_mode_g93 {
        writer.push_code_line("G94")?;
        modal.feed_mode_g93 = false;
        modal.motion_code = None;
        modal.f_word = None;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn emit_rotary_index(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    axis_letter: char,
    cur_wcs: DVec3,
    target_deg: f64,
    rapid: bool,
    feed_deg_min: Option<f64>,
    wrap_radius_mm: f64,
) -> Result<(), PostError> {
    ensure_g94_mode(writer, modal)?;

    if let Some(swap) = profile.rotary.swap_linear_axis {
        let lin_mm = target_deg.to_radians() * wrap_radius_mm.max(1.0);
        let mapped_wcs = match swap {
            SwapAxisTarget::ReplaceY => DVec3::new(cur_wcs.x, lin_mm, cur_wcs.z),
            SwapAxisTarget::ReplaceX => DVec3::new(lin_mm, cur_wcs.y, cur_wcs.z),
        };
        let feed_mm = if rapid {
            None
        } else {
            let deg_m = feed_deg_min
                .unwrap_or(profile.rotary.max_rotary_feed_deg_min)
                .clamp(1.0, profile.rotary.max_rotary_feed_deg_min.max(1.0));
            Some(deg_m.to_radians() * wrap_radius_mm.max(1.0))
        };
        return emit_linear_or_rapid(writer, profile, modal, mapped_wcs, feed_mm);
    }

    let a_str = format_decimal(
        target_deg,
        profile.rotary.angle_decimals,
        profile.suppress_trailing_zeros,
    );
    if modal.rotary_word.as_deref() == Some(a_str.as_str()) {
        return Ok(());
    }
    let g_code = if rapid { "G0" } else { "G1" };
    let mut words = Vec::with_capacity(4);
    if modal.motion_code != Some(g_code) {
        words.push(g_code.to_string());
        modal.motion_code = Some(g_code);
    }
    words.push(format!("{axis_letter}{a_str}"));
    modal.rotary_word = Some(a_str);
    if !rapid {
        let f_deg = feed_deg_min
            .unwrap_or(profile.rotary.max_rotary_feed_deg_min)
            .clamp(1.0, profile.rotary.max_rotary_feed_deg_min.max(1.0));
        let fs = format_decimal(
            f_deg,
            profile.feed_decimals,
            profile.suppress_trailing_zeros,
        );
        if modal.f_word.as_deref() != Some(fs.as_str()) {
            words.push(format!("F{fs}"));
            modal.f_word = Some(fs);
        }
    }
    writer.push_code_line(&words.join(" "))
}

#[allow(clippy::too_many_arguments)]
fn emit_rotary_motion(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    axis_letter: char,
    start_wcs: DVec3,
    end_wcs: DVec3,
    start_deg: f64,
    end_deg: f64,
    feed_mm_min: Option<f64>,
    effective_radius_mm: f64,
) -> Result<(), PostError> {
    if let Some(swap) = profile.rotary.swap_linear_axis {
        ensure_g94_mode(writer, modal)?;
        let lin_mm = end_deg.to_radians() * effective_radius_mm.max(1.0);
        let mapped_wcs = match swap {
            SwapAxisTarget::ReplaceY => DVec3::new(end_wcs.x, lin_mm, end_wcs.z),
            SwapAxisTarget::ReplaceX => DVec3::new(lin_mm, end_wcs.y, end_wcs.z),
        };
        return emit_linear_or_rapid(writer, profile, modal, mapped_wcs, feed_mm_min);
    }

    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;
    let xs = format_decimal(profile.units.scale_from_mm(end_wcs.x), c_dec, strip);
    let ys = format_decimal(profile.units.scale_from_mm(end_wcs.y), c_dec, strip);
    let zs = format_decimal(profile.units.scale_from_mm(end_wcs.z), c_dec, strip);
    let as_ = format_decimal(end_deg, profile.rotary.angle_decimals, strip);

    let x_changed = modal.x_word.as_deref() != Some(xs.as_str());
    let y_changed = modal.y_word.as_deref() != Some(ys.as_str());
    let z_changed = modal.z_word.as_deref() != Some(zs.as_str());
    let a_changed = modal.rotary_word.as_deref() != Some(as_.as_str());

    if !x_changed && !y_changed && !z_changed && !a_changed {
        return Ok(());
    }

    let has_rot_delta = (end_deg - start_deg).abs() > 1e-6;
    let use_g93 = feed_mm_min.is_some()
        && has_rot_delta
        && profile.rotary.supports_g93_inverse_time
        && profile.rotary.use_g93_for_simultaneous;

    let mut words: Vec<String> = Vec::with_capacity(7);
    if use_g93 {
        if !modal.feed_mode_g93 {
            words.push("G93".to_string());
            modal.feed_mode_g93 = true;
            modal.motion_code = None;
            modal.f_word = None;
        }
    } else {
        ensure_g94_mode(writer, modal)?;
    }

    let g_target = if feed_mm_min.is_some() { "G1" } else { "G0" };
    if modal.motion_code != Some(g_target) {
        words.push(g_target.to_string());
        modal.motion_code = Some(g_target);
    }
    if x_changed {
        words.push(format!("X{xs}"));
        modal.x_word = Some(xs);
    }
    if y_changed {
        words.push(format!("Y{ys}"));
        modal.y_word = Some(ys);
    }
    if z_changed {
        words.push(format!("Z{zs}"));
        modal.z_word = Some(zs);
    }
    if a_changed {
        words.push(format!("{axis_letter}{as_}"));
        modal.rotary_word = Some(as_);
    }

    if let Some(f_mm) = feed_mm_min {
        if use_g93 {
            let f_inv = compute_inverse_time_feed_g93(
                start_wcs,
                end_wcs,
                start_deg,
                end_deg,
                effective_radius_mm,
                f_mm,
                profile.rotary.max_rotary_feed_deg_min,
            )
            .unwrap_or(60.0);
            // RS274NGC mandates an explicit F word on EVERY G93 block
            let fs = format_decimal(f_inv, profile.feed_decimals.max(2), strip);
            words.push(format!("F{fs}"));
            modal.f_word = Some(fs);
        } else if !x_changed && !y_changed && !z_changed && has_rot_delta {
            let f_deg = compute_rotary_angular_feed_deg_min(
                f_mm,
                effective_radius_mm,
                profile.rotary.max_rotary_feed_deg_min,
            );
            let fs = format_decimal(f_deg, profile.feed_decimals, strip);
            if modal.f_word.as_deref() != Some(fs.as_str()) {
                words.push(format!("F{fs}"));
                modal.f_word = Some(fs);
            }
        } else {
            let f_scaled = profile.units.scale_from_mm(f_mm).max(0.1);
            let fs = format_decimal(f_scaled, profile.feed_decimals, strip);
            if modal.f_word.as_deref() != Some(fs.as_str()) {
                words.push(format!("F{fs}"));
                modal.f_word = Some(fs);
            }
        }
    }

    writer.push_code_line(&words.join(" "))
}
