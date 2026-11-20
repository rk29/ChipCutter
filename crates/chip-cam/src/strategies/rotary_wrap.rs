//! Cylindrical wrapping (`OperationRotaryMode::WrapCylinder` — 2-linear + rotary `Z+X+A` / `Z+Y+B`)
//! and positional `3+1` multi-face indexing (`OperationRotaryMode::Indexed3Plus1`) wrappers.

use chip_math::DVec3;

use crate::{
    cl_data::{ClEvent, MotionRole},
    rotary::{ResolvedRotaryConfig, RotaryAxisKind},
    toolpath::{ClearingSubToolpath, GeneratedToolpath, SegmentMotion3D, ToolpathBuilder},
};

/// Wraps a flat 2D planar toolpath `(x_2d, y_2d, z_2d)` onto a cylinder of radius `wrap_radius_mm`
/// around `rotary`, strictly locking the perpendicular cross-axis at `rotary.center_wcs`:
/// - `XAxis` (`Z+X+A`): `u_axial = x_2d + axial_offset`, `s_circ = y_2d`, `y_wcs = center.y`
/// - `YAxis` (`Z+Y+B`): `u_axial = y_2d + axial_offset`, `s_circ = -x_2d`, `x_wcs = center.x`
/// - `z_wcs = center.z + wrap_radius_mm + z_2d`, `rotary_deg = angle_offset_deg + (s_circ / wrap_radius_mm).to_degrees()`
pub fn wrap_planar_toolpath_to_cylinder(
    flat_tp: &GeneratedToolpath,
    rotary: &ResolvedRotaryConfig,
    wrap_radius_mm: f64,
    axial_offset_mm: f64,
    angle_offset_deg: f64,
) -> GeneratedToolpath {
    let r_wrap = wrap_radius_mm.max(1.0);
    let safe_z_wcs = rotary.center_wcs.z + rotary.clearance_radius_mm.max(r_wrap + 5.0);

    let map_pt = |p_flat: DVec3| -> (DVec3, f64, f64) {
        let (u_axial, s_circ) = match rotary.axis {
            RotaryAxisKind::XAxis => (p_flat.x + axial_offset_mm, p_flat.y),
            RotaryAxisKind::YAxis => (p_flat.y + axial_offset_mm, -p_flat.x),
            RotaryAxisKind::ZAxis => (p_flat.x + axial_offset_mm, p_flat.y),
        };
        let r_tip = (r_wrap + p_flat.z).max(0.25);
        let z_wcs = rotary.center_wcs.z + r_tip;
        let wcs = match rotary.axis {
            RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                DVec3::new(u_axial, rotary.center_wcs.y, z_wcs)
            }
            RotaryAxisKind::YAxis => DVec3::new(rotary.center_wcs.x, u_axial, z_wcs),
        };
        let deg = angle_offset_deg + (s_circ / r_wrap).to_degrees();
        (wcs, deg, r_tip)
    };

    let mut builder = ToolpathBuilder::new(flat_tp.op_id.clone());
    builder.pass_count = flat_tp.stats.pass_count;
    builder.warnings = flat_tp.warnings.clone();

    // Preserve non-motion header events (OperationStart, ToolChange, SpindleStart, Coolant, etc.)
    for ev in &flat_tp.events {
        match ev {
            ClEvent::SetupStart { .. }
            | ClEvent::OperationStart { .. }
            | ClEvent::ToolChange { .. }
            | ClEvent::SpindleStart { .. }
            | ClEvent::Coolant { .. }
            | ClEvent::Comment(_) => builder.push_event(ev.clone()),
            _ => break,
        }
    }

    let max_chord_mm = 0.35_f64;
    let max_step_deg = 1.5_f64;

    for (seg_idx, seg) in flat_tp.segments_wcs.iter().enumerate() {
        let (start_wcs, start_deg, start_r) = map_pt(seg.start_wcs);
        if seg_idx == 0 {
            let entry_safe = DVec3::new(start_wcs.x, start_wcs.y, safe_z_wcs.max(start_wcs.z));
            builder.rotary_rapid_to(entry_safe, start_deg);
            if start_wcs.z < entry_safe.z - 1e-4 {
                match seg.motion {
                    SegmentMotion3D::Rapid => builder.rotary_rapid_to(start_wcs, start_deg),
                    SegmentMotion3D::Linear { feed_mm_min, .. }
                    | SegmentMotion3D::Arc { feed_mm_min, .. } => {
                        builder.rotary_linear_to(
                            start_wcs,
                            start_deg,
                            feed_mm_min,
                            start_r,
                            MotionRole::Plunge,
                        );
                    }
                }
            }
        }

        match seg.motion {
            SegmentMotion3D::Rapid => {
                let (mut end_wcs, end_deg, _) = map_pt(seg.end_wcs);
                // Keep rapids at or above rotary clearance radius when traversing laterally/angularly
                let lateral = (end_wcs.x - start_wcs.x).hypot(end_wcs.y - start_wcs.y);
                if lateral > 1e-3 || (end_deg - start_deg).abs() > 1e-3 {
                    end_wcs.z = end_wcs.z.max(safe_z_wcs);
                }
                builder.rotary_rapid_to(end_wcs, end_deg);
            }
            SegmentMotion3D::Linear { feed_mm_min, role } => {
                let (_, end_deg, _) = map_pt(seg.end_wcs);
                let d_deg = (end_deg - start_deg).abs();
                let n_steps = ((d_deg / max_step_deg).ceil() as usize)
                    .max((seg.length_mm() / (max_chord_mm * 2.0)).ceil() as usize)
                    .clamp(1, 512);
                for k in 1..=n_steps {
                    let t = k as f64 / n_steps as f64;
                    let (w, d, r) = map_pt(seg.sample_point(t));
                    builder.rotary_linear_to(w, d, feed_mm_min, r, role);
                }
            }
            SegmentMotion3D::Arc {
                feed_mm_min, role, ..
            } => {
                let len = seg.length_mm();
                let n_steps = ((len / max_chord_mm).ceil() as usize).clamp(4, 512);
                for k in 1..=n_steps {
                    let t = k as f64 / n_steps as f64;
                    let (w, d, r) = map_pt(seg.sample_point(t));
                    builder.rotary_linear_to(w, d, feed_mm_min, r, role);
                }
            }
        }
    }

    if let Some(last) = builder.current_pos()
        && last.z < safe_z_wcs - 1e-4
    {
        builder.rotary_rapid_to(
            DVec3::new(last.x, last.y, safe_z_wcs),
            builder.current_rotary_deg(),
        );
    }

    // Append trailing OperationEnd / ProgramEnd events
    for ev in &flat_tp.events {
        if matches!(ev, ClEvent::OperationEnd | ClEvent::ProgramEnd) {
            builder.push_event(ev.clone());
        }
    }

    if let Some(cs) = &flat_tp.clearing_stage {
        let wrapped_clear = wrap_planar_toolpath_to_cylinder(
            &cs.toolpath,
            rotary,
            r_wrap,
            axial_offset_mm,
            angle_offset_deg,
        );
        builder.clearing_stage = Some(Box::new(ClearingSubToolpath {
            tool_id: cs.tool_id.clone(),
            toolpath: wrapped_clear,
        }));
    }

    builder.finish()
}

/// Wraps a toolpath generated inside an [`crate::rotary::build_indexed_virtual_setup`] rotated
/// frame with safe clearance-radius retracts, a leading `ClEvent::RotaryIndex`, and constant
/// `start_rotary_deg = end_rotary_deg = angle_deg` across every motion segment.
pub fn wrap_indexed_3plus1_toolpath(
    local_tp: &GeneratedToolpath,
    rotary: &ResolvedRotaryConfig,
    angle_deg: f64,
) -> GeneratedToolpath {
    let safe_z_wcs = rotary.center_wcs.z + rotary.clearance_radius_mm;
    let mut out = local_tp.clone();

    // Tag every motion segment with the constant indexed rotary angle
    for seg in &mut out.segments_wcs {
        seg.start_rotary_deg = angle_deg;
        seg.end_rotary_deg = angle_deg;
    }

    // Ensure safe entry above rotary clearance radius before executing the index rotation
    let first_xy = out
        .segments_wcs
        .first()
        .map(|s| (s.start_wcs.x, s.start_wcs.y))
        .unwrap_or((0.0, 0.0));
    let safe_entry = DVec3::new(first_xy.0, first_xy.1, safe_z_wcs);

    if let Some(first_seg) = out.segments_wcs.first()
        && first_seg.start_wcs.z < safe_z_wcs - 1e-3
    {
        out.segments_wcs.insert(
            0,
            crate::toolpath::ToolpathSegment3D {
                start_wcs: safe_entry,
                end_wcs: first_seg.start_wcs,
                motion: SegmentMotion3D::Rapid,
                start_rotary_deg: angle_deg,
                end_rotary_deg: angle_deg,
            },
        );
    }

    // Ensure safe final retract to rotary clearance radius at end of indexed operation
    if let Some(last_seg) = out.segments_wcs.last().copied()
        && last_seg.end_wcs.z < safe_z_wcs - 1e-3
    {
        let safe_exit = DVec3::new(last_seg.end_wcs.x, last_seg.end_wcs.y, safe_z_wcs);
        out.segments_wcs.push(crate::toolpath::ToolpathSegment3D {
            start_wcs: last_seg.end_wcs,
            end_wcs: safe_exit,
            motion: SegmentMotion3D::Rapid,
            start_rotary_deg: angle_deg,
            end_rotary_deg: angle_deg,
        });
    }

    // Insert safe retract + `RotaryIndex` right before the first motion event in `ClEvent` stream
    let mut new_events = Vec::with_capacity(out.events.len() + 4);
    let mut indexed_emitted = false;
    for ev in out.events {
        let is_motion = matches!(
            ev,
            ClEvent::RapidMove { .. }
                | ClEvent::LinearMove { .. }
                | ClEvent::ArcMove { .. }
                | ClEvent::DrillCycle { .. }
        );
        if is_motion && !indexed_emitted {
            new_events.push(ClEvent::RapidMove {
                end_wcs: DVec3::new(first_xy.0, first_xy.1, safe_z_wcs),
            });
            new_events.push(ClEvent::RotaryIndex {
                angle_deg,
                rapid: true,
                feed_deg_min: None,
            });
            indexed_emitted = true;
        }
        if matches!(ev, ClEvent::OperationEnd | ClEvent::ProgramEnd)
            && indexed_emitted
            && let Some(last) = out.segments_wcs.last()
        {
            new_events.push(ClEvent::RapidMove {
                end_wcs: DVec3::new(last.end_wcs.x, last.end_wcs.y, safe_z_wcs),
            });
        }
        new_events.push(ev);
    }
    out.events = new_events;

    if let Some(cs) = &mut out.clearing_stage {
        cs.toolpath = wrap_indexed_3plus1_toolpath(&cs.toolpath, rotary, angle_deg);
    }

    out
}
