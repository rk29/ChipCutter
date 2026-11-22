struct ViewportUniforms {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    wcs_world_to_local: mat4x4<f32>,
    wcs_local_to_world: mat4x4<f32>,
    cad_model: mat4x4<f32>,
    stock_model: mat4x4<f32>,
    camera_pos: vec4<f32>,
    // x = width_px, y = height_px, z = grid_minor_mm, w = grid_major_mm
    viewport_and_grid: vec4<f32>,
    // x = hovered_face_id (u32::MAX if none), y = ghost_part (1/0), z = show_grid (1/0), w = ortho_flag (1/0)
    pick_flags: vec4<u32>,
    // xyz = active 3D cutter tip position in Setup WCS (mm)
    sim_tool_tip_wcs: vec4<f32>,
    bg_top_rgb: vec4<f32>,
    bg_bottom_rgb: vec4<f32>,
    grid_rgb_and_scale: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> u_vp: ViewportUniforms;

// Per-face highlight state: 0 = Default, 1 = Hovered, 2 = Selected
@group(0) @binding(1)
var<storage, read> face_states: array<u32>;

// ============================================================================
// Pass 1: Studio Background Gradient + Active Setup WCS XY Grid (Z_wcs = 0)
// ============================================================================

struct BgOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

@vertex
fn vs_background(@builtin(vertex_index) vid: u32) -> BgOut {
    var pos = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    var out: BgOut;
    out.ndc = pos[vid];
    out.clip_pos = vec4<f32>(pos[vid], 1.0, 1.0);
    return out;
}

struct BgFragOut {
    @location(0) color: vec4<f32>,
    @builtin(frag_depth) depth: f32,
};

@fragment
fn fs_background(in: BgOut) -> BgFragOut {
    let t = clamp(0.5 * (in.ndc.y + 1.0), 0.0, 1.0);
    let top_col = u_vp.bg_top_rgb.xyz;
    let bot_col = u_vp.bg_bottom_rgb.xyz;
    var col = mix(bot_col, top_col, t);
    var out_depth = 1.0;

    if (u_vp.pick_flags.z != 0u) {
        let near_h = u_vp.inv_view_proj * vec4<f32>(in.ndc, 0.0, 1.0);
        let far_h = u_vp.inv_view_proj * vec4<f32>(in.ndc, 1.0, 1.0);
        let p_near_world = near_h.xyz / near_h.w;
        let p_far_world = far_h.xyz / far_h.w;

        let p_near = (u_vp.wcs_world_to_local * vec4<f32>(p_near_world, 1.0)).xyz;
        let p_far = (u_vp.wcs_world_to_local * vec4<f32>(p_far_world, 1.0)).xyz;
        let ray_dir = p_far - p_near;

        if (abs(ray_dir.z) > 1e-6) {
            let s = -p_near.z / ray_dir.z;
            if (s >= 0.0 && s <= 1.0) {
                let hit_wcs = p_near + ray_dir * s;
                let hit_world = (u_vp.wcs_local_to_world * vec4<f32>(hit_wcs, 1.0)).xyz;
                let clip_hit = u_vp.view_proj * vec4<f32>(hit_world, 1.0);
                let ndc_z = clip_hit.z / clip_hit.w;

                if (ndc_z >= 0.0 && ndc_z <= 1.0) {
                    let minor_step = max(u_vp.viewport_and_grid.z, 0.001);
                    let major_step = max(u_vp.viewport_and_grid.w, 0.01);

                    let coord_minor = hit_wcs.xy / minor_step;
                    let deriv_minor = fwidth(coord_minor);
                    let dist_minor = abs(fract(coord_minor - 0.5) - 0.5) / max(deriv_minor, vec2<f32>(1e-4));
                    let line_minor = 1.0 - clamp(min(dist_minor.x, dist_minor.y), 0.0, 1.0);

                    let coord_major = hit_wcs.xy / major_step;
                    let deriv_major = fwidth(coord_major);
                    let dist_major = abs(fract(coord_major - 0.5) - 0.5) / max(deriv_major, vec2<f32>(1e-4));
                    let line_major = 1.0 - clamp(min(dist_major.x, dist_major.y), 0.0, 1.0);

                    let axis_px = abs(hit_wcs.xy) / max(fwidth(hit_wcs.xy), vec2<f32>(1e-4));
                    let x_axis = 1.0 - clamp(axis_px.y - 0.35, 0.0, 1.0);
                    let y_axis = 1.0 - clamp(axis_px.x - 0.35, 0.0, 1.0);

                    let dist_from_origin = length(hit_wcs.xy);
                    let fade = 1.0 - smoothstep(major_step * 8.0, major_step * 28.0, dist_from_origin);

                    var grid_rgb = u_vp.grid_rgb_and_scale.xyz;
                    var alpha = (line_minor * 0.22 + line_major * 0.42) * u_vp.grid_rgb_and_scale.w;

                    if (x_axis > 0.01 && hit_wcs.x >= -major_step * 2.0) {
                        grid_rgb = mix(grid_rgb, vec3<f32>(0.85, 0.28, 0.28), x_axis);
                        alpha = max(alpha, x_axis * 0.78);
                    }
                    if (y_axis > 0.01 && hit_wcs.y >= -major_step * 2.0) {
                        grid_rgb = mix(grid_rgb, vec3<f32>(0.26, 0.78, 0.38), y_axis);
                        alpha = max(alpha, y_axis * 0.78);
                    }

                    alpha = clamp(alpha * fade, 0.0, 0.85);
                    if (alpha > 0.005) {
                        col = mix(col, grid_rgb, alpha);
                    }
                }
            }
        }
    }

    var out: BgFragOut;
    out.color = vec4<f32>(col, 1.0);
    out.depth = out_depth;
    return out;
}

// ============================================================================
// Pass 2: Shaded B-Rep Solid Faces (Multi-Face Selection + Ghost Translucency)
// ============================================================================

struct SolidVertexIn {
    @location(0) pos_mm: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) face_id: u32,
};

struct SolidVertexOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) view_normal: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) @interpolate(flat) face_id: u32,
};

@vertex
fn vs_solid(in: SolidVertexIn) -> SolidVertexOut {
    var out: SolidVertexOut;
    let world_pos = (u_vp.cad_model * vec4<f32>(in.pos_mm, 1.0)).xyz;
    let world_n = normalize((u_vp.cad_model * vec4<f32>(in.normal, 0.0)).xyz);
    out.clip_pos = u_vp.view_proj * vec4<f32>(world_pos, 1.0);
    out.world_normal = world_n;
    out.view_normal = normalize((u_vp.view * vec4<f32>(world_n, 0.0)).xyz);
    out.face_id = in.face_id;
    return out;
}

@fragment
fn fs_solid(in: SolidVertexOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    var vn = normalize(in.view_normal);
    if (!is_front) {
        vn = -vn;
    }

    var state: u32 = 0u;
    if (in.face_id < arrayLength(&face_states)) {
        state = face_states[in.face_id];
    }

    var albedo = vec3<f32>(0.68, 0.72, 0.77);
    var emissive = vec3<f32>(0.0);
    var out_alpha: f32 = 1.0;

    if (u_vp.pick_flags.y != 0u) {
        out_alpha = 0.36;
    }

    if (state == 2u) {
        // Selected B-Rep face(s): rich warm gold/amber
        albedo = vec3<f32>(0.96, 0.72, 0.24);
        emissive = vec3<f32>(0.08, 0.05, 0.01);
        out_alpha = max(out_alpha, 0.88);
    } else if (state == 1u || in.face_id == u_vp.pick_flags.x) {
        // Hovered B-Rep face: crisp sky cyan
        albedo = vec3<f32>(0.36, 0.78, 0.96);
        emissive = vec3<f32>(0.04, 0.08, 0.11);
        out_alpha = max(out_alpha, 0.82);
    }

    let key_dir = normalize(vec3<f32>(0.42, 0.56, 0.71));
    let fill_dir = normalize(vec3<f32>(-0.62, -0.25, 0.45));
    let rim_dir = normalize(vec3<f32>(0.0, 0.75, -0.66));
    let view_dir = vec3<f32>(0.0, 0.0, 1.0);

    let ndl_key = max(dot(vn, key_dir), 0.0);
    let ndl_fill = max(dot(vn, fill_dir), 0.0);
    let ndl_rim = max(dot(vn, rim_dir), 0.0);

    let half_vec = normalize(key_dir + view_dir);
    let spec = pow(max(dot(vn, half_vec), 0.0), 36.0) * 0.22;

    let hemi = 0.5 + 0.5 * in.world_normal.z;
    let ambient = mix(vec3<f32>(0.20, 0.22, 0.26), vec3<f32>(0.30, 0.32, 0.35), hemi);

    let lit = albedo * (ambient + 0.64 * ndl_key + 0.24 * ndl_fill + 0.14 * ndl_rim)
        + vec3<f32>(spec)
        + emissive;

    return vec4<f32>(clamp(lit, vec3<f32>(0.0), vec3<f32>(1.0)), out_alpha);
}

// ============================================================================
// Pass 3: Translucent Stock & Z-Height Planes Overlay
// ============================================================================

struct OverlayVertexIn {
    @location(0) pos_mm: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct OverlayVertexOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_overlay(in: OverlayVertexIn) -> OverlayVertexOut {
    var out: OverlayVertexOut;
    out.clip_pos = u_vp.view_proj * vec4<f32>(in.pos_mm, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_overlay(in: OverlayVertexOut) -> @location(0) vec4<f32> {
    return in.color;
}

// ============================================================================
// Pass 4: Screen-Space Width-Stabilized Edges, Stock Wireframe, WCS Triad, & Arrows
// ============================================================================

struct EdgeInstanceIn {
    @location(0) p0_mm: vec3<f32>,
    @location(1) p1_mm: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) width_px: f32,
};

struct EdgeVertexOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_edge(
    @builtin(vertex_index) corner_idx: u32,
    inst: EdgeInstanceIn,
) -> EdgeVertexOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(0.0, -0.5),
        vec2<f32>(1.0, -0.5),
        vec2<f32>(1.0,  0.5),
        vec2<f32>(0.0, -0.5),
        vec2<f32>(1.0,  0.5),
        vec2<f32>(0.0,  0.5),
    );
    let uv = corners[corner_idx];

    var c0 = u_vp.view_proj * vec4<f32>(inst.p0_mm, 1.0);
    var c1 = u_vp.view_proj * vec4<f32>(inst.p1_mm, 1.0);

    let screen_size = max(u_vp.viewport_and_grid.xy, vec2<f32>(1.0, 1.0));
    let s0 = (c0.xy / c0.w) * 0.5 * screen_size;
    let s1 = (c1.xy / c1.w) * 0.5 * screen_size;

    let delta = s1 - s0;
    let seg_len = length(delta);
    var dir = vec2<f32>(1.0, 0.0);
    if (seg_len > 1e-4) {
        dir = delta / seg_len;
    }
    let normal_px = vec2<f32>(-dir.y, dir.x);

    var base_clip = mix(c0, c1, uv.x);
    let offset_ndc = (normal_px * (uv.y * inst.width_px) * 2.0) / screen_size;
    base_clip.x = base_clip.x + offset_ndc.x * base_clip.w;
    base_clip.y = base_clip.y + offset_ndc.y * base_clip.w;
    base_clip.z = max(base_clip.z - 0.00035 * base_clip.w, 0.0);

    var out: EdgeVertexOut;
    out.clip_pos = base_clip;
    out.color = inst.color;
    return out;
}

@fragment
fn fs_edge(in: EdgeVertexOut) -> @location(0) vec4<f32> {
    return in.color;
}

// ============================================================================
// Pass 6: Shaded 3D Machined Stock Mesh & 3D Revolved Cutter/Shank/Holder Mesh
// ============================================================================

struct SimVertexIn {
    @location(0) pos_wcs_mm: vec3<f32>,
    @location(1) normal_wcs: vec3<f32>,
    @location(2) color_rgba: vec4<f32>,
};

struct SimVertexOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) view_normal: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) color_rgba: vec4<f32>,
};

@vertex
fn vs_sim_stock(in: SimVertexIn) -> SimVertexOut {
    var out: SimVertexOut;
    let stock_wcs_pos = (u_vp.stock_model * vec4<f32>(in.pos_wcs_mm, 1.0)).xyz;
    let stock_wcs_n = (u_vp.stock_model * vec4<f32>(in.normal_wcs, 0.0)).xyz;
    let world_pos = (u_vp.wcs_local_to_world * vec4<f32>(stock_wcs_pos, 1.0)).xyz;
    let world_n = normalize((u_vp.wcs_local_to_world * vec4<f32>(stock_wcs_n, 0.0)).xyz);
    out.clip_pos = u_vp.view_proj * vec4<f32>(world_pos, 1.0);
    out.world_normal = world_n;
    out.view_normal = normalize((u_vp.view * vec4<f32>(world_n, 0.0)).xyz);
    out.color_rgba = in.color_rgba;
    return out;
}

@vertex
fn vs_sim_tool(in: SimVertexIn) -> SimVertexOut {
    var out: SimVertexOut;
    let local_wcs = in.pos_wcs_mm + u_vp.sim_tool_tip_wcs.xyz;
    let world_pos = (u_vp.wcs_local_to_world * vec4<f32>(local_wcs, 1.0)).xyz;
    let world_n = normalize((u_vp.wcs_local_to_world * vec4<f32>(in.normal_wcs, 0.0)).xyz);
    out.clip_pos = u_vp.view_proj * vec4<f32>(world_pos, 1.0);
    out.world_normal = world_n;
    out.view_normal = normalize((u_vp.view * vec4<f32>(world_n, 0.0)).xyz);
    out.color_rgba = in.color_rgba;
    return out;
}

fn shade_sim_surface(
    view_normal_in: vec3<f32>,
    world_normal: vec3<f32>,
    color_rgba: vec4<f32>,
    is_front: bool,
) -> vec4<f32> {
    var vn = normalize(view_normal_in);
    if (!is_front) {
        vn = -vn;
    }

    let key_dir = normalize(vec3<f32>(0.42, 0.56, 0.71));
    let fill_dir = normalize(vec3<f32>(-0.62, -0.25, 0.45));
    let rim_dir = normalize(vec3<f32>(0.0, 0.75, -0.66));
    let view_dir = vec3<f32>(0.0, 0.0, 1.0);

    let ndl_key = max(dot(vn, key_dir), 0.0);
    let ndl_fill = max(dot(vn, fill_dir), 0.0);
    let ndl_rim = max(dot(vn, rim_dir), 0.0);

    let half_vec = normalize(key_dir + view_dir);
    let spec = pow(max(dot(vn, half_vec), 0.0), 42.0) * 0.25;

    let hemi = 0.5 + 0.5 * world_normal.z;
    let ambient = mix(vec3<f32>(0.21, 0.23, 0.27), vec3<f32>(0.32, 0.34, 0.38), hemi);

    let lit = color_rgba.rgb * (ambient + 0.65 * ndl_key + 0.24 * ndl_fill + 0.14 * ndl_rim)
        + vec3<f32>(spec);

    return vec4<f32>(clamp(lit, vec3<f32>(0.0), vec3<f32>(1.0)), color_rgba.a);
}

@fragment
fn fs_sim_mesh(in: SimVertexOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    return shade_sim_surface(in.view_normal, in.world_normal, in.color_rgba, is_front);
}

// ============================================================================
// Pass 6 (Direct GPU): Zero-Readback Heightfield & 4th-Axis Cylindrical Shaders
// ============================================================================

const SIM_UNCUT_OP_IDX: u32 = 0xFFFFFFFFu;
const SIM_COLOR_MODE_BY_OP: u32 = 1u;
const SIM_COLOR_MODE_HEATMAP: u32 = 2u;

struct SimStockVisualUniforms {
    // [color_mode_id (0=Metallic, 1=ByOperation, 2=ErrorHeatmap), has_target, 0, 0]
    mode_and_flags: vec4<u32>,
    // [tolerance_mm, max_scale_mm, 0, 0]
    heatmap_params: vec4<f32>,
    raw_top_color: vec4<f32>,
    raw_side_color: vec4<f32>,
    machined_metal_color: vec4<f32>,
    op_palette: array<vec4<f32>, 16>,
};

struct GpuHeightfieldRenderUniforms {
    // [nx, ny, vx, vy]
    dims: vec4<u32>,
    // [stride, 0, 0, 0]
    grid_meta: vec4<u32>,
    // [origin_x, origin_y, origin_z, 0.0]
    origin_wcs: vec4<f32>,
    // [vdx, vdy, 0.0, 0.0]
    pitch_mm: vec4<f32>,
};

struct GpuCylindricalRenderUniforms {
    // [e1.x, e1.y, e1.z, u_min]
    e1_and_u_min: vec4<f32>,
    // [e2_signed.x, e2_signed.y, e2_signed.z, du]
    e2_signed_and_du: vec4<f32>,
    // [u_vec.x, u_vec.y, u_vec.z, dtheta]
    u_vec_and_dtheta: vec4<f32>,
    // [center_wcs.x, center_wcs.y, center_wcs.z, u_max]
    center_and_u_max: vec4<f32>,
    // [nu, ntheta, su, st]
    dims: vec4<u32>,
    // [step_u, step_t, shell_verts, 0]
    steps_and_counts: vec4<u32>,
    // [u_center, 0.0, 0.0, 0.0]
    extra: vec4<f32>,
};

@group(1) @binding(0) var<uniform> u_sim_vis: SimStockVisualUniforms;

// Heightfield bindings (1..=5)
@group(1) @binding(1) var<uniform> u_hf_grid: GpuHeightfieldRenderUniforms;
@group(1) @binding(2) var<storage, read> hf_z_stock: array<f32>;
@group(1) @binding(3) var<storage, read> hf_z_bottom: array<f32>;
@group(1) @binding(4) var<storage, read> hf_z_target: array<f32>;
@group(1) @binding(5) var<storage, read> hf_last_op: array<u32>;

// Cylindrical bindings (6..=9)
@group(1) @binding(6) var<uniform> u_cyl_grid: GpuCylindricalRenderUniforms;
@group(1) @binding(7) var<storage, read> cyl_r_stock: array<f32>;
@group(1) @binding(8) var<storage, read> cyl_r_target: array<f32>;
@group(1) @binding(9) var<storage, read> cyl_last_op: array<u32>;

fn sim_is_finite(v: f32) -> bool {
    return (bitcast<u32>(v) & 0x7F800000u) != 0x7F800000u;
}

fn sim_eval_op_color(op_idx: u32, is_side: bool) -> vec4<f32> {
    if (op_idx == SIM_UNCUT_OP_IDX) {
        if (is_side) {
            return u_sim_vis.raw_side_color;
        }
        return u_sim_vis.raw_top_color;
    }
    if (u_sim_vis.mode_and_flags.x == SIM_COLOR_MODE_BY_OP) {
        return u_sim_vis.op_palette[op_idx & 15u];
    }
    return u_sim_vis.machined_metal_color;
}

fn sim_eval_heatmap(signed_dev: f32) -> vec4<f32> {
    let tol = max(u_sim_vis.heatmap_params.x, 1e-4);
    let max_scale = u_sim_vis.heatmap_params.y;
    let span = max(max_scale - tol, tol);
    let abs_d = abs(signed_dev);
    if (abs_d <= tol) {
        return vec4<f32>(0.18, 0.78, 0.36, 1.0);
    }
    let t = clamp((abs_d - tol) / span, 0.0, 1.0);
    if (signed_dev > 0.0) {
        return vec4<f32>(
            0.22 + t * (0.12 - 0.22),
            0.80 + t * (0.28 - 0.80),
            0.94 + t * (0.88 - 0.94),
            1.0,
        );
    }
    return vec4<f32>(
        0.98 + t * (0.94 - 0.98),
        0.78 + t * (0.16 - 0.78),
        0.16 + t * (0.20 - 0.16),
        1.0,
    );
}

// --- Direct GPU Heightfield Renderer ---

struct SimGpuHfOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) view_normal: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) @interpolate(flat) color_sidx: u32,
    @location(3) @interpolate(flat) color_flags: u32,
};

fn hf_src_idx(vi: u32, vj: u32) -> u32 {
    let stride = u_hf_grid.grid_meta.x;
    let si = min(vi * stride + stride / 2u, u_hf_grid.dims.x - 1u);
    let sj = min(vj * stride + stride / 2u, u_hf_grid.dims.y - 1u);
    return sj * u_hf_grid.dims.x + si;
}

fn hf_is_active(sidx: u32) -> bool {
    let zb = hf_z_bottom[sidx];
    let zt = hf_z_stock[sidx];
    return sim_is_finite(zb) && sim_is_finite(zt) && zt > zb + 1e-4;
}

fn hf_degenerate_out() -> SimGpuHfOut {
    var out: SimGpuHfOut;
    out.clip_pos = vec4<f32>(0.0, 0.0, 2.0, 1.0);
    out.view_normal = vec3<f32>(0.0, 0.0, 1.0);
    out.world_normal = vec3<f32>(0.0, 0.0, 1.0);
    out.color_sidx = 0u;
    out.color_flags = 2u;
    return out;
}

@vertex
fn vs_sim_heightfield_gpu(@builtin(vertex_index) vid: u32) -> SimGpuHfOut {
    let vx = u_hf_grid.dims.z;
    let vy = u_hf_grid.dims.w;
    let cell_idx = vid / 36u;
    if (cell_idx >= vx * vy) {
        return hf_degenerate_out();
    }
    let vi = cell_idx % vx;
    let vj = cell_idx / vx;
    let sidx = hf_src_idx(vi, vj);
    if (!hf_is_active(sidx)) {
        return hf_degenerate_out();
    }

    let rem = vid % 36u;
    let face_id = rem / 6u;
    let tri_v = rem % 6u;
    var quad_lut = array<u32, 6>(0u, 1u, 2u, 0u, 2u, 3u);
    let corner = quad_lut[tri_v];

    let ox = u_hf_grid.origin_wcs.x;
    let oy = u_hf_grid.origin_wcs.y;
    let oz = u_hf_grid.origin_wcs.z;
    let vdx = u_hf_grid.pitch_mm.x;
    let vdy = u_hf_grid.pitch_mm.y;

    let x0 = ox + f32(vi) * vdx;
    let x1 = ox + f32(vi + 1u) * vdx;
    let y0 = oy + f32(vj) * vdy;
    let y1 = oy + f32(vj + 1u) * vdy;

    let zt = hf_z_stock[sidx];
    let zb = hf_z_bottom[sidx];
    let zt_w = oz + zt;
    let zb_w = oz + zb;

    var p0 = vec3<f32>(0.0);
    var p1 = vec3<f32>(0.0);
    var p2 = vec3<f32>(0.0);
    var p3 = vec3<f32>(0.0);
    var normal = vec3<f32>(0.0, 0.0, 1.0);
    var color_sidx = sidx;
    var color_flags = 0u;

    if (face_id == 0u) {
        // Top face (+Z) with smooth finite-difference slope normals matching heightfield/mesher.rs
        let slope_max_dz = 1.6 * max(vdx, vdy);
        var zl = zt;
        if (vi > 0u) {
            let n = hf_src_idx(vi - 1u, vj);
            if (hf_is_active(n) && abs(hf_z_stock[n] - zt) <= slope_max_dz) {
                zl = hf_z_stock[n];
            }
        }
        var zr = zt;
        if (vi + 1u < vx) {
            let n = hf_src_idx(vi + 1u, vj);
            if (hf_is_active(n) && abs(hf_z_stock[n] - zt) <= slope_max_dz) {
                zr = hf_z_stock[n];
            }
        }
        var zd = zt;
        if (vj > 0u) {
            let n = hf_src_idx(vi, vj - 1u);
            if (hf_is_active(n) && abs(hf_z_stock[n] - zt) <= slope_max_dz) {
                zd = hf_z_stock[n];
            }
        }
        var zu = zt;
        if (vj + 1u < vy) {
            let n = hf_src_idx(vi, vj + 1u);
            if (hf_is_active(n) && abs(hf_z_stock[n] - zt) <= slope_max_dz) {
                zu = hf_z_stock[n];
            }
        }
        let dz_dx = (zr - zl) / (2.0 * vdx);
        let dz_dy = (zu - zd) / (2.0 * vdy);
        normal = normalize(vec3<f32>(-dz_dx, -dz_dy, 1.0));
        p0 = vec3<f32>(x0, y0, zt_w);
        p1 = vec3<f32>(x1, y0, zt_w);
        p2 = vec3<f32>(x1, y1, zt_w);
        p3 = vec3<f32>(x0, y1, zt_w);
        color_sidx = sidx;
        color_flags = 0u;
    } else if (face_id == 1u) {
        // Bottom face (-Z)
        normal = vec3<f32>(0.0, 0.0, -1.0);
        p0 = vec3<f32>(x0, y1, zb_w);
        p1 = vec3<f32>(x1, y1, zb_w);
        p2 = vec3<f32>(x1, y0, zb_w);
        p3 = vec3<f32>(x0, y0, zb_w);
        color_sidx = sidx;
        color_flags = 2u;
    } else if (face_id == 2u) {
        // +X wall/riser
        var emit_wall = false;
        var z_lo_w = zb_w;
        if (vi + 1u >= vx) {
            emit_wall = true;
            color_sidx = sidx;
            color_flags = 1u;
        } else {
            let n = hf_src_idx(vi + 1u, vj);
            if (!hf_is_active(n)) {
                emit_wall = true;
                color_sidx = sidx;
                color_flags = 1u;
            } else if (zt > hf_z_stock[n] + 2e-4) {
                emit_wall = true;
                z_lo_w = oz + max(hf_z_stock[n], zb);
                color_sidx = n;
                color_flags = 0u;
            }
        }
        if (!emit_wall) {
            return hf_degenerate_out();
        }
        normal = vec3<f32>(1.0, 0.0, 0.0);
        p0 = vec3<f32>(x1, y0, z_lo_w);
        p1 = vec3<f32>(x1, y1, z_lo_w);
        p2 = vec3<f32>(x1, y1, zt_w);
        p3 = vec3<f32>(x1, y0, zt_w);
    } else if (face_id == 3u) {
        // -X wall/riser
        var emit_wall = false;
        var z_lo_w = zb_w;
        if (vi == 0u) {
            emit_wall = true;
            color_sidx = sidx;
            color_flags = 1u;
        } else {
            let n = hf_src_idx(vi - 1u, vj);
            if (!hf_is_active(n)) {
                emit_wall = true;
                color_sidx = sidx;
                color_flags = 1u;
            } else if (zt > hf_z_stock[n] + 2e-4) {
                emit_wall = true;
                z_lo_w = oz + max(hf_z_stock[n], zb);
                color_sidx = n;
                color_flags = 0u;
            }
        }
        if (!emit_wall) {
            return hf_degenerate_out();
        }
        normal = vec3<f32>(-1.0, 0.0, 0.0);
        p0 = vec3<f32>(x0, y1, z_lo_w);
        p1 = vec3<f32>(x0, y0, z_lo_w);
        p2 = vec3<f32>(x0, y0, zt_w);
        p3 = vec3<f32>(x0, y1, zt_w);
    } else if (face_id == 4u) {
        // +Y wall/riser
        var emit_wall = false;
        var z_lo_w = zb_w;
        if (vj + 1u >= vy) {
            emit_wall = true;
            color_sidx = sidx;
            color_flags = 1u;
        } else {
            let n = hf_src_idx(vi, vj + 1u);
            if (!hf_is_active(n)) {
                emit_wall = true;
                color_sidx = sidx;
                color_flags = 1u;
            } else if (zt > hf_z_stock[n] + 2e-4) {
                emit_wall = true;
                z_lo_w = oz + max(hf_z_stock[n], zb);
                color_sidx = n;
                color_flags = 0u;
            }
        }
        if (!emit_wall) {
            return hf_degenerate_out();
        }
        normal = vec3<f32>(0.0, 1.0, 0.0);
        p0 = vec3<f32>(x1, y1, z_lo_w);
        p1 = vec3<f32>(x0, y1, z_lo_w);
        p2 = vec3<f32>(x0, y1, zt_w);
        p3 = vec3<f32>(x1, y1, zt_w);
    } else {
        // -Y wall/riser
        var emit_wall = false;
        var z_lo_w = zb_w;
        if (vj == 0u) {
            emit_wall = true;
            color_sidx = sidx;
            color_flags = 1u;
        } else {
            let n = hf_src_idx(vi, vj - 1u);
            if (!hf_is_active(n)) {
                emit_wall = true;
                color_sidx = sidx;
                color_flags = 1u;
            } else if (zt > hf_z_stock[n] + 2e-4) {
                emit_wall = true;
                z_lo_w = oz + max(hf_z_stock[n], zb);
                color_sidx = n;
                color_flags = 0u;
            }
        }
        if (!emit_wall) {
            return hf_degenerate_out();
        }
        normal = vec3<f32>(0.0, -1.0, 0.0);
        p0 = vec3<f32>(x0, y0, z_lo_w);
        p1 = vec3<f32>(x1, y0, z_lo_w);
        p2 = vec3<f32>(x1, y0, zt_w);
        p3 = vec3<f32>(x0, y0, zt_w);
    }

    var pos_wcs = p0;
    if (corner == 1u) {
        pos_wcs = p1;
    } else if (corner == 2u) {
        pos_wcs = p2;
    } else if (corner == 3u) {
        pos_wcs = p3;
    }

    let stock_wcs_pos = (u_vp.stock_model * vec4<f32>(pos_wcs, 1.0)).xyz;
    let stock_wcs_n = (u_vp.stock_model * vec4<f32>(normal, 0.0)).xyz;
    let world_pos = (u_vp.wcs_local_to_world * vec4<f32>(stock_wcs_pos, 1.0)).xyz;
    let world_n = normalize((u_vp.wcs_local_to_world * vec4<f32>(stock_wcs_n, 0.0)).xyz);

    var out: SimGpuHfOut;
    out.clip_pos = u_vp.view_proj * vec4<f32>(world_pos, 1.0);
    out.world_normal = world_n;
    out.view_normal = normalize((u_vp.view * vec4<f32>(world_n, 0.0)).xyz);
    out.color_sidx = color_sidx;
    out.color_flags = color_flags;
    return out;
}

@fragment
fn fs_sim_gpu_stock(in: SimGpuHfOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    var col = u_sim_vis.raw_side_color;
    if (in.color_flags != 2u) {
        let sidx = in.color_sidx;
        if (u_sim_vis.mode_and_flags.x == SIM_COLOR_MODE_HEATMAP) {
            let zt = hf_z_target[sidx];
            let zb = hf_z_bottom[sidx];
            if (sim_is_finite(zt) && zt > zb + 1e-5) {
                col = sim_eval_heatmap(hf_z_stock[sidx] - zt);
            } else {
                col = sim_eval_heatmap(u_sim_vis.heatmap_params.y);
            }
        } else {
            col = sim_eval_op_color(hf_last_op[sidx], in.color_flags == 1u);
        }
    }
    return shade_sim_surface(in.view_normal, in.world_normal, col, is_front);
}

// --- Direct GPU 4th-Axis Cylindrical Renderer ---

fn cyl_src_idx(iu: u32, it: u32) -> u32 {
    let nu = u_cyl_grid.dims.x;
    let nt = u_cyl_grid.dims.y;
    let step_u = u_cyl_grid.steps_and_counts.x;
    let step_t = u_cyl_grid.steps_and_counts.y;
    let cu = min(iu * step_u, nu - 1u);
    let ct = (it * step_t) % nt;
    return cu * nt + ct;
}

fn cyl_vertex_pos(iu: u32, it: u32) -> vec3<f32> {
    let su = u_cyl_grid.dims.z;
    let step_u = u_cyl_grid.steps_and_counts.x;
    let step_t = u_cyl_grid.steps_and_counts.y;
    let u_min = u_cyl_grid.e1_and_u_min.w;
    let du = u_cyl_grid.e2_signed_and_du.w;
    let dtheta = u_cyl_grid.u_vec_and_dtheta.w;
    let u_max = u_cyl_grid.center_and_u_max.w;

    var u_val = u_min + (f32(iu * step_u) + 0.5) * du;
    if (iu == 0u) {
        u_val = u_min;
    } else if (iu + 1u == su) {
        u_val = u_max;
    }
    let theta = f32(it * step_t) * dtheta;
    let sidx = cyl_src_idx(iu, it);
    let r = max(cyl_r_stock[sidx], 0.0);

    let e1 = u_cyl_grid.e1_and_u_min.xyz;
    let e2_signed = u_cyl_grid.e2_signed_and_du.xyz;
    let u_vec = u_cyl_grid.u_vec_and_dtheta.xyz;
    let center = u_cyl_grid.center_and_u_max.xyz;
    let u_center = u_cyl_grid.extra.x;

    return center + u_vec * (u_val - u_center) + (e1 * cos(theta) + e2_signed * sin(theta)) * r;
}

fn cyl_cell_color(sidx: u32) -> vec4<f32> {
    if (u_sim_vis.mode_and_flags.x == SIM_COLOR_MODE_HEATMAP) {
        if (u_sim_vis.mode_and_flags.y == 0u) {
            return u_sim_vis.raw_top_color;
        }
        let r_tgt = cyl_r_target[sidx];
        if (r_tgt >= 0.0 && sim_is_finite(r_tgt)) {
            let dev = max(cyl_r_stock[sidx], 0.0) - r_tgt;
            return sim_eval_heatmap(dev);
        }
        return sim_eval_heatmap(u_sim_vis.heatmap_params.y);
    }
    return sim_eval_op_color(cyl_last_op[sidx], false);
}

@vertex
fn vs_sim_cylindrical_gpu(@builtin(vertex_index) vid: u32) -> SimVertexOut {
    let su = u_cyl_grid.dims.z;
    let st = u_cyl_grid.dims.w;
    let shell_verts = u_cyl_grid.steps_and_counts.z;
    let u_vec = u_cyl_grid.u_vec_and_dtheta.xyz;
    let center = u_cyl_grid.center_and_u_max.xyz;
    let u_center = u_cyl_grid.extra.x;

    var pos_wcs = vec3<f32>(0.0);
    var normal_wcs = vec3<f32>(0.0, 0.0, 1.0);
    var col = u_sim_vis.raw_side_color;

    if (vid < shell_verts) {
        let quad_idx = vid / 6u;
        let tri_v = vid % 6u;
        let iu = quad_idx / st;
        let it = quad_idx % st;
        let it_next = (it + 1u) % st;

        var vi_u = iu;
        var vi_t = it;
        if (tri_v == 1u) {
            vi_u = iu + 1u;
            vi_t = it;
        } else if (tri_v == 2u || tri_v == 4u) {
            vi_u = iu + 1u;
            vi_t = it_next;
        } else if (tri_v == 5u) {
            vi_u = iu;
            vi_t = it_next;
        }

        pos_wcs = cyl_vertex_pos(vi_u, vi_t);
        let u_prev = select(vi_u - 1u, 0u, vi_u == 0u);
        let u_next = min(vi_u + 1u, su - 1u);
        let t_prev = (vi_t + st - 1u) % st;
        let t_next = (vi_t + 1u) % st;
        let du_vec = cyl_vertex_pos(u_next, vi_t) - cyl_vertex_pos(u_prev, vi_t);
        let dt_vec = cyl_vertex_pos(vi_u, t_next) - cyl_vertex_pos(vi_u, t_prev);
        let n_raw = cross(du_vec, dt_vec);
        let n_len = length(n_raw);
        if (n_len > 1e-6) {
            normal_wcs = n_raw / n_len;
        } else {
            normal_wcs = u_cyl_grid.e1_and_u_min.xyz;
        }
        col = cyl_cell_color(cyl_src_idx(vi_u, vi_t));
    } else {
        // Axial endcaps (min endcap: -u_vec, max endcap: +u_vec)
        let cap_vid = vid - shell_verts;
        let cap_tri_verts = st * 3u;
        let is_max_end = cap_vid >= cap_tri_verts;
        let local_v = select(cap_vid, cap_vid - cap_tri_verts, is_max_end);
        let it = local_v / 3u;
        let tri_c = local_v % 3u;
        let it_next = (it + 1u) % st;
        let ring_iu = select(0u, su - 1u, is_max_end);
        let cap_u = select(u_cyl_grid.e1_and_u_min.w, u_cyl_grid.center_and_u_max.w, is_max_end);
        normal_wcs = select(-u_vec, u_vec, is_max_end);

        let heatmap_active = u_sim_vis.mode_and_flags.x == SIM_COLOR_MODE_HEATMAP
            && u_sim_vis.mode_and_flags.y != 0u;
        if (tri_c == 0u) {
            pos_wcs = center + u_vec * (cap_u - u_center);
            if (heatmap_active) {
                col = cyl_cell_color(cyl_src_idx(ring_iu, it));
            } else {
                col = u_sim_vis.raw_side_color;
            }
        } else {
            let ring_it = select(
                select(it, it_next, tri_c == 2u),
                select(it_next, it, tri_c == 2u),
                is_max_end,
            );
            pos_wcs = cyl_vertex_pos(ring_iu, ring_it);
            if (heatmap_active) {
                col = cyl_cell_color(cyl_src_idx(ring_iu, ring_it));
            } else {
                col = u_sim_vis.raw_side_color;
            }
        }
    }

    let stock_wcs_pos = (u_vp.stock_model * vec4<f32>(pos_wcs, 1.0)).xyz;
    let stock_wcs_n = (u_vp.stock_model * vec4<f32>(normal_wcs, 0.0)).xyz;
    let world_pos = (u_vp.wcs_local_to_world * vec4<f32>(stock_wcs_pos, 1.0)).xyz;
    let world_n = normalize((u_vp.wcs_local_to_world * vec4<f32>(stock_wcs_n, 0.0)).xyz);

    var out: SimVertexOut;
    out.clip_pos = u_vp.view_proj * vec4<f32>(world_pos, 1.0);
    out.world_normal = world_n;
    out.view_normal = normalize((u_vp.view * vec4<f32>(world_n, 0.0)).xyz);
    out.color_rgba = col;
    return out;
}
