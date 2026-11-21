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

@fragment
fn fs_sim_mesh(in: SimVertexOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    var vn = normalize(in.view_normal);
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

    let hemi = 0.5 + 0.5 * in.world_normal.z;
    let ambient = mix(vec3<f32>(0.21, 0.23, 0.27), vec3<f32>(0.32, 0.34, 0.38), hemi);

    let lit = in.color_rgba.rgb * (ambient + 0.65 * ndl_key + 0.24 * ndl_fill + 0.14 * ndl_rim)
        + vec3<f32>(spec);

    return vec4<f32>(clamp(lit, vec3<f32>(0.0), vec3<f32>(1.0)), in.color_rgba.a);
}
