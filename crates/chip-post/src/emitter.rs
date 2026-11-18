use chip_cam::{ClEvent, CutterCompMode, ResolvedStockGeometry, WorkOffsetSlot};
use chip_math::DVec3;
use chip_tools::{CoolantMode, SpindleDirection, Tool};

use crate::{
    cycles_arcs::{
        ArcEmitSpec, DrillEmitSpec, ToolChangeEmitSpec, emit_drill_cycle, emit_split_arc_move,
        handle_tool_change,
    },
    modal::{LineWriter, ModalState, collapse_spaces, emit_linear_or_rapid, format_decimal},
    partition::{PostOutputFile, PostPartition, SetupPostInput, partition_operations},
    profile::{PostError, PostProfile},
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
    let mut active_tool_num: Option<u32> = None;

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
                    emit_linear_or_rapid(&mut writer, profile, &mut modal, *end_wcs, None)?;
                    cur_wcs = *end_wcs;
                }
                ClEvent::LinearMove {
                    end_wcs,
                    feed_mm_min,
                    ..
                } => {
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
