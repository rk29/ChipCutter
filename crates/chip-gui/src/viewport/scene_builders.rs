//! Scene mesh and feature-edge builders for 3D workholding fixtures rendered in [`super::ViewportRenderer`].

use std::{
    collections::HashMap,
    hash::{Hash as _, Hasher as _},
};

use chip_math::DVec3;
use chip_sim::SimColoredVertex;

use super::{
    gpu_types::EdgeInstanceGpu,
    highlight::{FixtureVisualMesh, HighlightState, dvec3_f32},
};

/// Computes a fast deterministic signature over `highlight.fixture_meshes`, `show_fixtures`, and
/// `theme` so `ViewportRenderer` skips redundant GPU buffer uploads on idle 60 FPS frames.
pub(crate) fn compute_fixture_gpu_signature(highlight: &HighlightState) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    highlight.show_fixtures.hash(&mut h);
    highlight.theme.is_light().hash(&mut h);
    highlight.fixture_meshes.len().hash(&mut h);
    for f in &highlight.fixture_meshes {
        f.revision.hash(&mut h);
        f.fixture_id.hash(&mut h);
        f.colliding.hash(&mut h);
        f.selected.hash(&mut h);
        for c in f.base_rgba {
            c.to_bits().hash(&mut h);
        }
    }
    h.finish()
}

/// Builds a combined flat-shaded translucent `SimColoredVertex` + index buffer (in Setup WCS mm)
/// for all active [`FixtureVisualMesh`] solids.
pub(crate) fn build_fixture_shaded_mesh(
    highlight: &HighlightState,
) -> (Vec<SimColoredVertex>, Vec<u32>) {
    if !highlight.show_fixtures || highlight.fixture_meshes.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let total_tris: usize = highlight.fixture_meshes.iter().map(|f| f.mesh.len()).sum();
    let mut vertices = Vec::with_capacity(total_tris * 3);
    let mut indices = Vec::with_capacity(total_tris * 3);
    let is_light = highlight.theme.is_light();

    for fix in &highlight.fixture_meshes {
        let color = resolve_fixture_fill_rgba(fix, is_light);
        for tri in &fix.mesh.triangles {
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                fix.mesh.vertices.get(tri.indices[0] as usize),
                fix.mesh.vertices.get(tri.indices[1] as usize),
                fix.mesh.vertices.get(tri.indices[2] as usize),
            ) else {
                continue;
            };
            let n = if tri.normal.length_squared() > 0.25 {
                tri.normal
            } else {
                (v1 - v0).cross(v2 - v0).normalize_or_zero()
            };
            let n_f32 = dvec3_f32(if n.length_squared() > 0.25 {
                n
            } else {
                DVec3::Z
            });
            let Ok(base) = u32::try_from(vertices.len()) else {
                break;
            };
            for v in [v0, v1, v2] {
                vertices.push(SimColoredVertex {
                    pos_mm: dvec3_f32(v),
                    normal: n_f32,
                    color_rgba: color,
                });
            }
            indices.push(base);
            indices.push(base + 1);
            indices.push(base + 2);
        }
    }

    (vertices, indices)
}

/// Emits crisp silhouette / feature-crease wireframe edges in World coordinates for all visible
/// [`FixtureVisualMesh`] bodies, filtering out flat-face triangulation diagonals (`dot > 0.90`).
pub(crate) fn emit_fixture_wireframe_edges(
    highlight: &HighlightState,
    out: &mut Vec<EdgeInstanceGpu>,
) {
    if !highlight.show_fixtures || highlight.fixture_meshes.is_empty() {
        return;
    }
    let is_light = highlight.theme.is_light();
    let frame = &highlight.wcs_frame;

    for fix in &highlight.fixture_meshes {
        let (wire_col, width_px) = resolve_fixture_wire_style(fix, is_light);
        let mesh = &fix.mesh;
        if mesh.is_empty() {
            continue;
        }

        // Quantize vertex positions to 1 µm so seams between adjacent flat quads weld cleanly
        let quant = |p: DVec3| -> [i64; 3] {
            [
                (p.x * 1_000.0).round() as i64,
                (p.y * 1_000.0).round() as i64,
                (p.z * 1_000.0).round() as i64,
            ]
        };

        type EdgeKey = ([i64; 3], [i64; 3]);
        struct EdgeEntry {
            pa: DVec3,
            pb: DVec3,
            first_normal: DVec3,
            keep: bool,
        }

        // Map canonical undirected edge -> edge endpoints, first normal, and crease flag
        let mut edge_map: HashMap<EdgeKey, EdgeEntry> = HashMap::with_capacity(mesh.len() * 2);

        for tri in &mesh.triangles {
            let (Some(&v0), Some(&v1), Some(&v2)) = (
                mesh.vertices.get(tri.indices[0] as usize),
                mesh.vertices.get(tri.indices[1] as usize),
                mesh.vertices.get(tri.indices[2] as usize),
            ) else {
                continue;
            };
            let n = tri.normal;
            for (pa, pb) in [(v0, v1), (v1, v2), (v2, v0)] {
                let qa = quant(pa);
                let qb = quant(pb);
                if qa == qb {
                    continue;
                }
                let key = if qa < qb { (qa, qb) } else { (qb, qa) };
                edge_map
                    .entry(key)
                    .and_modify(|e| {
                        // Mark as feature crease when dihedral angle > ~25° (`dot < 0.90`)
                        e.keep = e.first_normal.dot(n) < 0.90;
                    })
                    .or_insert(EdgeEntry {
                        pa,
                        pb,
                        first_normal: n,
                        keep: true,
                    });
            }
        }

        for entry in edge_map.into_values() {
            if !entry.keep {
                continue;
            }
            let wa = frame.local_to_world_point(entry.pa);
            let wb = frame.local_to_world_point(entry.pb);
            out.push(EdgeInstanceGpu {
                p0_mm: dvec3_f32(wa),
                p1_mm: dvec3_f32(wb),
                color: wire_col,
                width_px,
            });
        }
    }
}

fn resolve_fixture_fill_rgba(fix: &FixtureVisualMesh, is_light: bool) -> [f32; 4] {
    if fix.colliding {
        return [0.96, 0.16, 0.18, 0.56];
    }
    let mut c = fix.base_rgba;
    if fix.selected {
        // Blend softly toward warm selection amber while preserving fixture translucency
        c[0] = (c[0] * 0.55 + 0.96 * 0.45).clamp(0.0, 1.0);
        c[1] = (c[1] * 0.55 + 0.72 * 0.45).clamp(0.0, 1.0);
        c[2] = (c[2] * 0.55 + 0.24 * 0.45).clamp(0.0, 1.0);
        c[3] = (c[3] + 0.08).clamp(0.28, 0.54);
    } else if is_light {
        // Slightly deepen RGB contrast on bright Daylight studio background
        c[0] *= 0.88;
        c[1] *= 0.88;
        c[2] *= 0.88;
        c[3] = c[3].clamp(0.24, 0.48);
    } else {
        c[3] = c[3].clamp(0.24, 0.48);
    }
    c
}

fn resolve_fixture_wire_style(fix: &FixtureVisualMesh, is_light: bool) -> ([f32; 4], f32) {
    if fix.colliding {
        ([1.0, 0.22, 0.24, 0.98], 2.5)
    } else if fix.selected {
        if is_light {
            ([0.86, 0.48, 0.04, 0.96], 2.3)
        } else {
            ([1.0, 0.78, 0.24, 0.96], 2.3)
        }
    } else if is_light {
        ([0.16, 0.20, 0.26, 0.76], 1.45)
    } else {
        ([0.10, 0.13, 0.18, 0.82], 1.45)
    }
}
