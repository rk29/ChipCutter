use chip_cam::{
    ArcDir, ArcPlane, ClEvent, CutterCompMode, DrillCycleKind, ResolvedStockGeometry,
    SegmentMotion3D, ToolpathSegment3D, WorkOffsetSlot, expand_drill_cycle_segments,
};
use chip_math::DVec3;
use chip_tools::{CoolantMode, SpindleDirection, Tool};

use crate::{
    partition::{PostOutputFile, PostPartition, SetupPostInput, partition_operations},
    profile::{ArcCenterMode, CommentStyle, PostError, PostProfile, PostUnits},
};

#[derive(Debug, Clone)]
struct ModalState {
    motion_code: Option<&'static str>,
    plane: ArcPlane,
    x_word: Option<String>,
    y_word: Option<String>,
    z_word: Option<String>,
    f_word: Option<String>,
    canned_active: bool,
    cutter_comp: CutterCompMode,
    coolant_on: bool,
}

impl Default for ModalState {
    fn default() -> Self {
        Self {
            motion_code: None,
            plane: ArcPlane::XY,
            x_word: None,
            y_word: None,
            z_word: None,
            f_word: None,
            canned_active: false,
            cutter_comp: CutterCompMode::Off,
            coolant_on: false,
        }
    }
}

impl ModalState {
    fn force_full_next_move(&mut self) {
        self.motion_code = None;
        self.x_word = None;
        self.y_word = None;
        self.z_word = None;
        self.f_word = None;
    }
}

struct LineWriter<'a> {
    profile: &'a PostProfile,
    lines: Vec<String>,
    next_line_num: Option<u32>,
    max_observed_len: usize,
}

impl<'a> LineWriter<'a> {
    fn new(profile: &'a PostProfile) -> Self {
        Self {
            profile,
            lines: Vec::with_capacity(512),
            next_line_num: profile.line_numbers.map(|c| c.start),
            max_observed_len: 0,
        }
    }

    fn push_code_line(&mut self, raw_line: &str) -> Result<(), PostError> {
        let trimmed = raw_line.trim();
        if trimmed.is_empty() {
            return Ok(());
        }
        // Tape markers `%` never receive `N##` block numbers
        let formatted = if trimmed == "%" {
            "%".to_string()
        } else if trimmed.starts_with('(') || trimmed.starts_with(';') {
            return self.push_comment_text(
                trimmed
                    .trim_start_matches(['(', ';'])
                    .trim_end_matches(')')
                    .trim(),
            );
        } else if let Some(n) = self.next_line_num {
            let inc = self.profile.line_numbers.map_or(10, |c| c.increment.max(1));
            self.next_line_num = Some(n.saturating_add(inc));
            format!("N{n} {trimmed}")
        } else {
            trimmed.to_string()
        };

        let len = formatted.len();
        if let Some(max_len) = self.profile.max_line_length
            && len > max_len
        {
            return Err(PostError::LineTooLong {
                len,
                max: max_len,
                line: formatted,
            });
        }
        self.max_observed_len = self.max_observed_len.max(len);
        self.lines.push(formatted);
        Ok(())
    }

    fn push_comment_text(&mut self, raw_text: &str) -> Result<(), PostError> {
        let clean = sanitize_comment_ascii(raw_text);
        let max_total = self.profile.max_line_length.unwrap_or(240);
        let wrapper_overhead = match self.profile.comment_style {
            CommentStyle::Parentheses => 4, // "( " + " )"
            CommentStyle::Semicolon => 2,   // "; "
        };
        let max_body = max_total.saturating_sub(wrapper_overhead).max(8);
        let body = if clean.len() > max_body {
            clean[..max_body].trim_end().to_string()
        } else {
            clean
        };

        let line = match self.profile.comment_style {
            CommentStyle::Parentheses => format!("( {body} )"),
            CommentStyle::Semicolon => format!("; {body}"),
        };
        self.max_observed_len = self.max_observed_len.max(line.len());
        self.lines.push(line);
        Ok(())
    }

    fn finish(self) -> (String, usize, usize) {
        let count = self.lines.len();
        let max_chars = self.max_observed_len;
        let mut joined = self.lines.join("\n");
        joined.push('\n');
        (joined, count, max_chars)
    }
}

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
                part.suppress_m6,
                &mut active_tool_num,
                item.tool.tool_number,
                item.tool.length_offset_number,
                &item.tool.name,
                local_op_idx == 0,
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
                        part.suppress_m6,
                        &mut active_tool_num,
                        *tool_number,
                        *length_offset,
                        tool_desc,
                        *is_first_tool && local_op_idx == 0,
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
                        cur_wcs,
                        *plane,
                        *dir,
                        *end_wcs,
                        *center_offset_ijk,
                        *radius_mm,
                        *turns,
                        *feed_mm_min,
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
                    let has_decreasing_peck =
                        matches!(kind, DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck)
                            && decreasing_factor.is_some_and(|f| f < 0.999);

                    if profile.expand_drill_cycles || has_decreasing_peck {
                        if modal.canned_active {
                            writer.push_code_line("G80")?;
                            modal.canned_active = false;
                            modal.force_full_next_move();
                        }
                        let segs: Vec<ToolpathSegment3D> = expand_drill_cycle_segments(
                            cur_wcs,
                            *kind,
                            *bottom_wcs,
                            *r_plane_z_wcs,
                            *clear_z_wcs,
                            *peck_q_mm,
                            *decreasing_factor,
                            *min_peck_mm,
                            *chip_break_retract_mm,
                            *feed_mm_min,
                        );
                        let mut emitted_dwell = false;
                        for seg in &segs {
                            match seg.motion {
                                SegmentMotion3D::Rapid => {
                                    emit_linear_or_rapid(
                                        &mut writer,
                                        profile,
                                        &mut modal,
                                        seg.end_wcs,
                                        None,
                                    )?;
                                }
                                SegmentMotion3D::Linear {
                                    feed_mm_min: f_seg, ..
                                } => {
                                    emit_linear_or_rapid(
                                        &mut writer,
                                        profile,
                                        &mut modal,
                                        seg.end_wcs,
                                        Some(f_seg),
                                    )?;
                                    if !emitted_dwell
                                        && *dwell_sec > 1e-4
                                        && (seg.end_wcs.z - bottom_wcs.z).abs() <= 1e-4
                                    {
                                        writer.push_code_line(&format!(
                                            "G4 P{}",
                                            format_decimal(*dwell_sec, 2, true)
                                        ))?;
                                        emitted_dwell = true;
                                    }
                                }
                                SegmentMotion3D::Arc { .. } => {}
                            }
                            cur_wcs = seg.end_wcs;
                        }
                    } else {
                        emit_native_drill_cycle(
                            &mut writer,
                            profile,
                            &mut modal,
                            *kind,
                            *bottom_wcs,
                            *r_plane_z_wcs,
                            *clear_z_wcs,
                            *peck_q_mm,
                            *dwell_sec,
                            *feed_mm_min,
                        )?;
                        let end_z = clear_z_wcs.max(*r_plane_z_wcs);
                        cur_wcs = DVec3::new(bottom_wcs.x, bottom_wcs.y, end_z);
                    }
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

#[allow(clippy::too_many_arguments)]
fn handle_tool_change(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    suppress_m6: bool,
    active_tool_num: &mut Option<u32>,
    tool_number: u32,
    length_offset: u32,
    tool_desc: &str,
    _is_first_tool: bool,
) -> Result<(), PostError> {
    if *active_tool_num == Some(tool_number) {
        return Ok(());
    }
    *active_tool_num = Some(tool_number);

    writer.push_comment_text(&format!("Tool T{tool_number}: {tool_desc}"))?;

    if suppress_m6 {
        return Ok(());
    }

    if profile.tool_change_block.is_empty() {
        writer.push_code_line(&format!("T{tool_number} M6"))?;
        if profile.use_g43_length_offset {
            writer.push_code_line(&format!("G43 H{length_offset}"))?;
        }
    } else {
        for tmpl in &profile.tool_change_block {
            if tmpl.contains("G43") && !profile.use_g43_length_offset {
                continue;
            }
            let line = tmpl
                .replace("{tool_number}", &tool_number.to_string())
                .replace("{length_offset}", &length_offset.to_string());
            let collapsed = collapse_spaces(&line);
            if !collapsed.is_empty() {
                writer.push_code_line(&collapsed)?;
            }
        }
    }
    modal.force_full_next_move();
    Ok(())
}

fn emit_linear_or_rapid(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    end_wcs_mm: DVec3,
    feed_mm_min: Option<f64>,
) -> Result<(), PostError> {
    let g_target = if feed_mm_min.is_some() { "G1" } else { "G0" };
    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;

    let xs = format_decimal(profile.units.scale_from_mm(end_wcs_mm.x), c_dec, strip);
    let ys = format_decimal(profile.units.scale_from_mm(end_wcs_mm.y), c_dec, strip);
    let zs = format_decimal(profile.units.scale_from_mm(end_wcs_mm.z), c_dec, strip);

    let x_changed = modal.x_word.as_deref() != Some(xs.as_str());
    let y_changed = modal.y_word.as_deref() != Some(ys.as_str());
    let z_changed = modal.z_word.as_deref() != Some(zs.as_str());

    if !x_changed && !y_changed && !z_changed {
        return Ok(());
    }

    let mut words: Vec<String> = Vec::with_capacity(6);
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
    if let Some(f_mm) = feed_mm_min {
        let f_scaled = profile.units.scale_from_mm(f_mm).max(0.1);
        let fs = format_decimal(f_scaled, profile.feed_decimals, strip);
        if modal.f_word.as_deref() != Some(fs.as_str()) {
            words.push(format!("F{fs}"));
            modal.f_word = Some(fs);
        }
    }

    writer.push_code_line(&words.join(" "))
}

#[allow(clippy::too_many_arguments)]
fn emit_split_arc_move(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    start_wcs: DVec3,
    plane: ArcPlane,
    dir: ArcDir,
    end_wcs: DVec3,
    center_offset_ijk: DVec3,
    radius_mm: f64,
    turns: u32,
    feed_mm_min: f64,
) -> Result<(), PostError> {
    let center_wcs = start_wcs + center_offset_ijk;
    let signed_sweep_rad =
        compute_arc_signed_sweep(start_wcs, end_wcs, center_wcs, plane, dir, turns);
    if signed_sweep_rad.abs() <= 1e-8 || radius_mm <= 1e-6 {
        return emit_linear_or_rapid(writer, profile, modal, end_wcs, Some(feed_mm_min));
    }

    let full_seg = ToolpathSegment3D {
        start_wcs,
        end_wcs,
        motion: SegmentMotion3D::Arc {
            plane,
            dir,
            center_wcs,
            radius_mm,
            signed_sweep_rad,
            feed_mm_min,
            role: chip_cam::MotionRole::Cut,
        },
    };

    let axial_delta = match plane {
        ArcPlane::XY => (end_wcs.z - start_wcs.z).abs(),
        ArcPlane::XZ => (end_wcs.y - start_wcs.y).abs(),
        ArcPlane::YZ => (end_wcs.x - start_wcs.x).abs(),
    };
    let is_helical = axial_delta > 1e-5;

    // Linearize 3D helical arcs to `G1` chords if disabled in profile
    if is_helical && !profile.allow_helical_arcs {
        let tol = profile.helical_linearize_tol_mm.clamp(0.002, 0.2);
        let cos_arg = (1.0 - (tol / radius_mm).clamp(0.0, 0.5)).clamp(-1.0, 1.0);
        let step_rad = (2.0 * cos_arg.acos()).max(0.02);
        let n_chords = ((signed_sweep_rad.abs() / step_rad).ceil() as usize).clamp(4, 4096);
        for k in 1..=n_chords {
            let pt = full_seg.sample_point(k as f64 / n_chords as f64);
            emit_linear_or_rapid(writer, profile, modal, pt, Some(feed_mm_min))?;
        }
        return Ok(());
    }

    // Switch active plane (`G17` / `G18` / `G19`) if needed
    if modal.plane != plane {
        let pcode = match plane {
            ArcPlane::XY => "G17",
            ArcPlane::XZ => "G18",
            ArcPlane::YZ => "G19",
        };
        writer.push_code_line(pcode)?;
        modal.plane = plane;
    }

    let max_sweep_deg = if profile.arc_center_mode == ArcCenterMode::Radius {
        profile.max_arc_sweep_deg.clamp(15.0, 179.0)
    } else {
        profile.max_arc_sweep_deg.clamp(15.0, 360.0)
    };
    let sub_count = ((signed_sweep_rad.abs().to_degrees() / max_sweep_deg).ceil() as usize).max(1);

    let g_arc = match dir {
        ArcDir::CW => "G2",
        ArcDir::CCW => "G3",
    };
    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;

    let mut sub_start = start_wcs;
    for k in 1..=sub_count {
        let sub_end = if k == sub_count {
            end_wcs
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
                push_plane_ijk(&mut words, plane, ijk, profile.units, c_dec, strip);
            }
            ArcCenterMode::AbsoluteIjk => {
                push_plane_ijk(&mut words, plane, center_wcs, profile.units, c_dec, strip);
            }
            ArcCenterMode::Radius => {
                let rs = format_decimal(profile.units.scale_from_mm(radius_mm), c_dec, strip);
                words.push(format!("R{rs}"));
            }
        }

        let f_scaled = profile.units.scale_from_mm(feed_mm_min).max(0.1);
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

#[allow(clippy::too_many_arguments)]
fn emit_native_drill_cycle(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    kind: DrillCycleKind,
    bottom_wcs: DVec3,
    r_plane_z_wcs: f64,
    clear_z_wcs: f64,
    peck_q_mm: f64,
    dwell_sec: f64,
    feed_mm_min: f64,
) -> Result<(), PostError> {
    let retract_mode = if clear_z_wcs > r_plane_z_wcs + 1e-4 {
        "G98"
    } else {
        "G99"
    };
    let cycle_code = match kind {
        DrillCycleKind::Spot if dwell_sec > 1e-4 => "G82",
        DrillCycleKind::Spot | DrillCycleKind::Simple => "G81",
        DrillCycleKind::ChipBreak => "G73",
        DrillCycleKind::DeepPeck => "G83",
    };

    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;
    let xs = format_decimal(profile.units.scale_from_mm(bottom_wcs.x), c_dec, strip);
    let ys = format_decimal(profile.units.scale_from_mm(bottom_wcs.y), c_dec, strip);
    let zs = format_decimal(profile.units.scale_from_mm(bottom_wcs.z), c_dec, strip);
    let rs = format_decimal(profile.units.scale_from_mm(r_plane_z_wcs), c_dec, strip);
    let fs = format_decimal(
        profile.units.scale_from_mm(feed_mm_min).max(0.1),
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
    if matches!(kind, DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck) {
        let qs = format_decimal(
            profile.units.scale_from_mm(peck_q_mm.max(0.05)),
            c_dec,
            strip,
        );
        words.push(format!("Q{qs}"));
    }
    if cycle_code == "G82" {
        words.push(format!("P{}", format_decimal(dwell_sec, 2, true)));
    }
    words.push(format!("F{fs}"));

    writer.push_code_line(&words.join(" "))?;
    modal.canned_active = true;
    modal.motion_code = Some(cycle_code);
    modal.x_word = Some(xs);
    modal.y_word = Some(ys);
    modal.z_word = Some(format_decimal(
        profile.units.scale_from_mm(clear_z_wcs.max(r_plane_z_wcs)),
        c_dec,
        strip,
    ));
    modal.f_word = Some(fs);
    Ok(())
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

pub(crate) fn format_decimal(val: f64, decimals: usize, suppress_trailing_zeros: bool) -> String {
    let clean_val = if val.abs() < 0.5 * 10_f64.powi(-(decimals as i32)) {
        0.0
    } else {
        val
    };
    if decimals == 0 {
        return format!("{:.0}", clean_val.round());
    }
    let mut s = format!("{clean_val:.decimals$}");
    if suppress_trailing_zeros && s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" || s.is_empty() {
        "0".to_string()
    } else {
        s
    }
}

fn sanitize_comment_ascii(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        match ch {
            '(' => out.push('['),
            ')' => out.push(']'),
            ';' => out.push(':'),
            '\r' | '\n' | '\t' => out.push(' '),
            'Ø' | 'ø' => out.push('D'),
            '°' => out.push_str("deg"),
            '—' | '–' => out.push('-'),
            '×' => out.push('x'),
            c if c.is_ascii_graphic() || c == ' ' => out.push(c),
            _ => {}
        }
    }
    out.trim().to_string()
}

fn collapse_spaces(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for word in s.split_whitespace() {
        if !out.is_empty() {
            out.push(' ');
        }
        out.push_str(word);
    }
    out
}
