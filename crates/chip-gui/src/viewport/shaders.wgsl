struct ViewportUniforms {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    camera_pos: vec4<f32>,
    // x = width_px, y = height_px, z = grid_minor_mm, w = grid_major_mm
    viewport_and_grid: vec4<f32>,
    // x = hovered_face_id (u32::MAX if none), y = selected_face_id (u32::MAX if none), z = show_grid (1/0), w = ortho_flag (1/0)
    pick_flags: vec4<u32>,
};

@group(0) @binding(0)
var<uniform> u_vp: ViewportUniforms;

// ============================================================================
// Pass 1: Studio Background Gradient + Anti-Aliased WCS XY Grid (Z = 0)
// ============================================================================

struct BgOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

@vertex
fn vs_background(@builtin(vertex_index) vid: u32) -> BgOut {
    // Fullscreen triangle covering [-1, 1]^2
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
    let top_col = vec3<f32>(0.155, 0.178, 0.212);
    let bot_col = vec3<f32>(0.088, 0.098, 0.116);
    var col = mix(bot_col, top_col, t);
    var out_depth = 1.0;

    if (u_vp.pick_flags.z != 0u) {
        let near_h = u_vp.inv_view_proj * vec4<f32>(in.ndc, 0.0, 1.0);
        let far_h = u_vp.inv_view_proj * vec4<f32>(in.ndc, 1.0, 1.0);
        let p_near = near_h.xyz / near_h.w;
        let p_far = far_h.xyz / far_h.w;
        let ray_dir = p_far - p_near;

        if (abs(ray_dir.z) > 1e-6) {
            let s = -p_near.z / ray_dir.z;
            if (s >= 0.0 && s <= 1.0) {
                let hit = p_near + ray_dir * s;
                let clip_hit = u_vp.view_proj * vec4<f32>(hit, 1.0);
                let ndc_z = clip_hit.z / clip_hit.w;

                if (ndc_z >= 0.0 && ndc_z <= 1.0) {
                    let minor_step = max(u_vp.viewport_and_grid.z, 0.001);
                    let major_step = max(u_vp.viewport_and_grid.w, 0.01);

                    let coord_minor = hit.xy / minor_step;
                    let deriv_minor = fwidth(coord_minor);
                    let dist_minor = abs(fract(coord_minor - 0.5) - 0.5) / max(deriv_minor, vec2<f32>(1e-4));
                    let line_minor = 1.0 - clamp(min(dist_minor.x, dist_minor.y), 0.0, 1.0);

                    let coord_major = hit.xy / major_step;
                    let deriv_major = fwidth(coord_major);
                    let dist_major = abs(fract(coord_major - 0.5) - 0.5) / max(deriv_major, vec2<f32>(1e-4));
                    let line_major = 1.0 - clamp(min(dist_major.x, dist_major.y), 0.0, 1.0);

                    // X & Y WCS zero axes
                    let axis_px = abs(hit.xy) / max(fwidth(hit.xy), vec2<f32>(1e-4));
                    let x_axis = 1.0 - clamp(axis_px.y - 0.35, 0.0, 1.0);
                    let y_axis = 1.0 - clamp(axis_px.x - 0.35, 0.0, 1.0);

                    let dist_from_origin = length(hit.xy);
                    let fade = 1.0 - smoothstep(major_step * 8.0, major_step * 28.0, dist_from_origin);

                    var grid_rgb = vec3<f32>(0.24, 0.27, 0.32);
                    var alpha = line_minor * 0.22 + line_major * 0.42;

                    if (x_axis > 0.01 && hit.x >= -major_step * 2.0) {
                        grid_rgb = mix(grid_rgb, vec3<f32>(0.85, 0.28, 0.28), x_axis);
                        alpha = max(alpha, x_axis * 0.78);
                    }
                    if (y_axis > 0.01 && hit.y >= -major_step * 2.0) {
                        grid_rgb = mix(grid_rgb, vec3<f32>(0.26, 0.78, 0.38), y_axis);
                        alpha = max(alpha, y_axis * 0.78);
                    }

                    alpha = clamp(alpha * fade, 0.0, 0.85);
                    if (alpha > 0.005) {
                        col = mix(col, grid_rgb, alpha);
                        out_depth = clamp(ndc_z + 0.00015, 0.0, 1.0);
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
// Pass 2: Shaded B-Rep Solid Faces (Studio Headlight + FaceId Highlighting)
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
    out.clip_pos = u_vp.view_proj * vec4<f32>(in.pos_mm, 1.0);
    out.world_normal = normalize(in.normal);
    out.view_normal = normalize((u_vp.view * vec4<f32>(in.normal, 0.0)).xyz);
    out.face_id = in.face_id;
    return out;
}

@fragment
fn fs_solid(in: SolidVertexOut, @builtin(front_facing) is_front: bool) -> @location(0) vec4<f32> {
    var vn = normalize(in.view_normal);
    if (!is_front) {
        vn = -vn;
    }

    // Base CAD machined aluminum / steel material color
    var albedo = vec3<f32>(0.68, 0.72, 0.77);
    var emissive = vec3<f32>(0.0);

    if (in.face_id == u_vp.pick_flags.y) {
        // Selected B-Rep face: rich warm gold/amber
        albedo = vec3<f32>(0.96, 0.72, 0.24);
        emissive = vec3<f32>(0.08, 0.05, 0.01);
    } else if (in.face_id == u_vp.pick_flags.x) {
        // Hovered B-Rep face: crisp sky cyan
        albedo = vec3<f32>(0.36, 0.78, 0.96);
        emissive = vec3<f32>(0.04, 0.08, 0.11);
    }

    // Camera-locked 3-point CAD studio lighting (readable from every orbit angle!)
    let key_dir = normalize(vec3<f32>(0.42, 0.56, 0.71));
    let fill_dir = normalize(vec3<f32>(-0.62, -0.25, 0.45));
    let rim_dir = normalize(vec3<f32>(0.0, 0.75, -0.66));
    let view_dir = vec3<f32>(0.0, 0.0, 1.0);

    let ndl_key = max(dot(vn, key_dir), 0.0);
    let ndl_fill = max(dot(vn, fill_dir), 0.0);
    let ndl_rim = max(dot(vn, rim_dir), 0.0);

    let half_vec = normalize(key_dir + view_dir);
    let spec = pow(max(dot(vn, half_vec), 0.0), 36.0) * 0.22;

    // Subtle world +Z hemisphere tint so top vs bottom faces have natural spatial cue
    let hemi = 0.5 + 0.5 * in.world_normal.z;
    let ambient = mix(vec3<f32>(0.20, 0.22, 0.26), vec3<f32>(0.30, 0.32, 0.35), hemi);

    let lit = albedo * (ambient + 0.64 * ndl_key + 0.24 * ndl_fill + 0.14 * ndl_rim)
        + vec3<f32>(spec)
        + emissive;

    return vec4<f32>(clamp(lit, vec3<f32>(0.0), vec3<f32>(1.0)), 1.0);
}

// ============================================================================
// Pass 3: Screen-Space Width-Stabilized Topological Edges & WCS Triad Gizmo
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
    // 2 triangles (6 vertices) forming a screen-space quad around segment p0 -> p1
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
    // Slight depth bias toward camera so topological edges sit cleanly on coplanar faces
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
