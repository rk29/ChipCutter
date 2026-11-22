use std::hash::{Hash as _, Hasher as _};

use chip_cam::{MotionRole, SegmentMotion3D};
use chip_math::DVec3;

use super::{
    gpu_types::EdgeInstanceGpu,
    highlight::{HighlightState, dvec3_f32},
};

pub(crate) fn compute_toolpath_gpu_signature(highlight: &HighlightState) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    highlight.theme.hash(&mut h);
    highlight.show_toolpaths.hash(&mut h);
    highlight.show_rapids.hash(&mut h);
    highlight.sim_trail_limit.hash(&mut h);
    highlight.sim_rotary_deg.to_bits().hash(&mut h);
    if let Some(rot) = highlight.rotary {
        true.hash(&mut h);
        rot.axis.hash(&mut h);
        rot.direction.hash(&mut h);
        rot.center_wcs.x.to_bits().hash(&mut h);
        rot.center_wcs.y.to_bits().hash(&mut h);
        rot.center_wcs.z.to_bits().hash(&mut h);
    } else {
        false.hash(&mut h);
    }
    for v in [
        highlight.wcs_frame.origin_mm,
        highlight.wcs_frame.x_axis,
        highlight.wcs_frame.y_axis,
        highlight.wcs_frame.z_axis,
    ] {
        v.x.to_bits().hash(&mut h);
        v.y.to_bits().hash(&mut h);
        v.z.to_bits().hash(&mut h);
    }
    if highlight.show_toolpaths {
        highlight.toolpaths.len().hash(&mut h);
        for item in &highlight.toolpaths {
            item.op_id.hash(&mut h);
            item.cache_key.hash(&mut h);
            item.is_focused.hash(&mut h);
        }
    }
    h.finish()
}

pub(crate) fn build_toolpath_gpu_instances(highlight: &HighlightState) -> Vec<EdgeInstanceGpu> {
    if !highlight.show_toolpaths || highlight.toolpaths.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::new();
    let frame = &highlight.wcs_frame;
    let mut global_seg_counter = 0usize;

    let is_light = highlight.theme.is_light();
    for entry in &highlight.toolpaths {
        let alpha = if entry.is_focused { 0.96_f32 } else { 0.42_f32 };
        let width_scale = if entry.is_focused { 1.0_f32 } else { 0.80_f32 };

        for seg in &entry.toolpath.segments_wcs {
            let cur_global = global_seg_counter;
            global_seg_counter += 1;
            if let Some(limit) = highlight.sim_trail_limit
                && cur_global > limit
            {
                continue;
            }
            let (color, width_px, arc_steps) = match seg.motion {
                SegmentMotion3D::Rapid => {
                    if !highlight.show_rapids {
                        continue;
                    }
                    let r_col = if is_light {
                        [0.82, 0.48, 0.04, 0.86 * alpha]
                    } else {
                        [0.98, 0.75, 0.16, 0.82 * alpha]
                    };
                    (r_col, 1.5 * width_scale, 1usize)
                }
                SegmentMotion3D::Linear { role, .. } => {
                    let (rgb, w) = role_color_and_width(role, is_light);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, 1usize)
                }
                SegmentMotion3D::Arc {
                    radius_mm,
                    signed_sweep_rad,
                    role,
                    ..
                } => {
                    let (rgb, w) = role_color_and_width(role, is_light);
                    // Discretize circular/helical arcs strictly on GPU upload to <= ~0.025 mm chord error
                    let step_angle = if radius_mm > 0.05 {
                        (2.0 * (1.0 - (0.025 / radius_mm).min(0.5)).acos()).clamp(0.06, 0.32)
                    } else {
                        0.25
                    };
                    let n = ((signed_sweep_rad.abs() / step_angle).ceil() as usize).clamp(4, 96);
                    ([rgb[0], rgb[1], rgb[2], alpha], w * width_scale, n)
                }
            };

            let rot_steps = if highlight.rotary.is_some() {
                let d_deg = (seg.end_rotary_deg - seg.start_rotary_deg).abs();
                if d_deg > 2.5 {
                    ((d_deg / 2.5).ceil() as usize).clamp(2, 128)
                } else {
                    1
                }
            } else {
                1
            };
            let steps = arc_steps.max(rot_steps);
            let sim_rot = highlight.sim_rotary_deg;
            let map_wcs = |p_wcs: DVec3, seg_rot_deg: f64| -> DVec3 {
                let pt = if let Some(rot) = highlight.rotary {
                    let net_deg = -seg_rot_deg + sim_rot;
                    if net_deg.abs() > 1e-6 {
                        rot.rotate_point_wcs(p_wcs, net_deg)
                    } else {
                        p_wcs
                    }
                } else {
                    p_wcs
                };
                frame.local_to_world_point(pt)
            };

            if steps <= 1 {
                let p0 = map_wcs(seg.start_wcs, seg.start_rotary_deg);
                let p1 = map_wcs(seg.end_wcs, seg.end_rotary_deg);
                out.push(EdgeInstanceGpu {
                    p0_mm: dvec3_f32(p0),
                    p1_mm: dvec3_f32(p1),
                    color,
                    width_px,
                });
            } else {
                let mut prev = map_wcs(seg.start_wcs, seg.start_rotary_deg);
                for k in 1..=steps {
                    let t = (k as f64) / (steps as f64);
                    let cur = map_wcs(seg.sample_point(t), seg.sample_rotary_deg(t));
                    out.push(EdgeInstanceGpu {
                        p0_mm: dvec3_f32(prev),
                        p1_mm: dvec3_f32(cur),
                        color,
                        width_px,
                    });
                    prev = cur;
                }
            }
        }
    }

    out
}

fn role_color_and_width(role: MotionRole, is_light: bool) -> ([f32; 3], f32) {
    if is_light {
        match role {
            MotionRole::Cut => ([0.04, 0.44, 0.84], 2.3),
            MotionRole::FinishCut => ([0.02, 0.58, 0.66], 2.5),
            MotionRole::LeadIn | MotionRole::LeadOut => ([0.06, 0.56, 0.36], 2.0),
            MotionRole::Plunge | MotionRole::Ramp => ([0.82, 0.18, 0.28], 2.2),
            MotionRole::TabRamp => ([0.82, 0.50, 0.04], 2.4),
            MotionRole::LinkTransition => ([0.18, 0.46, 0.80], 1.8),
        }
    } else {
        match role {
            MotionRole::Cut => ([0.22, 0.74, 0.98], 2.25),
            MotionRole::FinishCut => ([0.14, 0.90, 0.94], 2.45),
            MotionRole::LeadIn | MotionRole::LeadOut => ([0.20, 0.84, 0.60], 1.95),
            MotionRole::Plunge | MotionRole::Ramp => ([0.98, 0.44, 0.52], 2.15),
            MotionRole::TabRamp => ([0.98, 0.80, 0.24], 2.35),
            MotionRole::LinkTransition => ([0.56, 0.78, 0.98], 1.75),
        }
    }
}
