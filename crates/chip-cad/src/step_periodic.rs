//! Workarounds for ISO 10303-21 STEP B-spline / NURBS edge trimming and periodic
//! (closed-seam) surface tessellation on top of `monstertruck`.
//!
//! Two gaps in `monstertruck 0.4` affect swept and revolved freeform STEP solids
//! (such as FreeCAD/OpenCASCADE helical sweeps):
//! 1. `EDGE_CURVE` parsing leaves `CurveAny::BoundedCurve` (`B_SPLINE_CURVE` /
//!    `RATIONAL_B_SPLINE_CURVE`) untrimmed when `edge_start` / `edge_end` lie in the
//!    interior of the curve domain (e.g., seam edges shortened by a Boolean union).
//! 2. `BsplineSurface` and `NurbsSurface` report `period_u() == None` and
//!    `period_v() == None` even when geometrically closed (`u_closed = .T.` /
//!    `v_closed = .T.`), causing seam-split boundary wires to collapse onto a single
//!    seam coordinate during UV projection and producing missing or inverted face
//!    tessellations.

use std::collections::HashMap;
use std::ops::Bound;

use monstertruck_io::step::load::step_geometry::{
    BoundedCurve, Curve3D, Cut, InnerSpace, Invertible, MetricSpace, ParameterBoundary2D,
    ParameterDivision1D, ParameterDivision2D, ParameterRange, ParametricCurve, ParametricSurface,
    ParametricSurface3D, Point2, Point3, SearchNearestParameter, SearchParameter,
    SearchParameterHint2D, Surface, SurfaceParameter, Vector3, algo,
};
use monstertruck_meshing::prelude::{
    ExactTrimBoundary2D, Parallelizable, PolygonMesh, PolylineCurve, RobustMeshableShape,
};
use monstertruck_topology::compress::{
    CompressedEdge, CompressedEdgeIndex, CompressedFace, CompressedShell, CompressedTrimmedFace,
    CompressedTrimmedShell,
};

/// Trims open `BsplineCurve` / `NurbsCurve` edges in `trimmed` to their topological
/// `edge_start` / `edge_end` vertices whenever `monstertruck-io` left the underlying
/// STEP `BOUNDED_CURVE` untrimmed past the vertex endpoints.
pub(crate) fn trim_bounded_step_edges<T>(
    trimmed: &mut CompressedTrimmedShell<Point3, Curve3D, Surface, T>,
    unit_scale: f64,
) {
    // Already-matching endpoint tolerance (~0.1 µm in mm) and maximum allowed snap
    // distance (~0.05 mm) to confirm a vertex actually lies on the curve.
    let match_tol = (1e-4 / unit_scale).max(1e-6);
    let max_snap_tol = (0.05 / unit_scale).max(1e-3);

    for edge in &mut trimmed.edges {
        let (v0_idx, v1_idx) = edge.vertices;
        if v0_idx == v1_idx {
            continue;
        }
        let (Some(&v_start), Some(&v_end)) =
            (trimmed.vertices.get(v0_idx), trimmed.vertices.get(v1_idx))
        else {
            continue;
        };
        if v_start.distance(v_end) <= match_tol {
            continue;
        }

        if !matches!(
            edge.curve,
            Curve3D::BsplineCurve(_) | Curve3D::NurbsCurve(_)
        ) {
            continue;
        }

        let front_dist = edge.curve.front().distance(v_start);
        let back_dist = edge.curve.back().distance(v_end);
        if front_dist <= match_tol && back_dist <= match_tol {
            continue;
        }

        let Some((t_start, d_start)) = find_closest_curve_param(&edge.curve, v_start) else {
            continue;
        };
        let Some((t_end, d_end)) = find_closest_curve_param(&edge.curve, v_end) else {
            continue;
        };

        if d_start > max_snap_tol || d_end > max_snap_tol {
            continue;
        }

        let (t0, t1) = edge.curve.range_tuple();
        let span = t1 - t0;
        if !span.is_finite() || span <= 1e-12 || (t_end - t_start).abs() <= span * 1e-7 {
            continue;
        }

        let (lo, hi, reversed) = if t_start <= t_end {
            (t_start, t_end, false)
        } else {
            (t_end, t_start, true)
        };
        let end_eps = span * 1e-9;

        if lo > t0 + end_eps {
            edge.curve = edge.curve.cut(lo);
        }
        if hi < t1 - end_eps {
            let _ = edge.curve.cut(hi);
        }
        if reversed {
            edge.curve.invert();
        }
    }
}

/// Tessellates `trimmed` cleanly, routing any geometrically closed/periodic
/// `BsplineSurface` or `NurbsSurface` faces through [`PeriodicStepSurface`] so
/// seam-split boundary loops unwrap across their periodic domain without being
/// tessellated twice.
pub(crate) fn tessellate_step_shell<T>(
    trimmed: &CompressedTrimmedShell<Point3, Curve3D, Surface, T>,
    tol_file_units: f64,
) -> CompressedShell<Point3, PolylineCurve<Point3>, Option<PolygonMesh>>
where
    T: Clone + ExactTrimBoundary2D + Parallelizable,
{
    let periodic_surfaces: Vec<Option<PeriodicStepSurface>> = trimmed
        .faces
        .iter()
        .map(|f| PeriodicStepSurface::try_from_surface(&f.surface))
        .collect();

    if periodic_surfaces.iter().all(Option::is_none) {
        return trimmed.robust_triangulation(tol_file_units);
    }

    // Separate non-periodic faces (which run through `monstertruck`'s standard trimmed
    // tessellator along with all shell edges) from periodic freeform faces (which run
    // in parallel via `PeriodicStepSurface`).
    let mut non_periodic_indices = Vec::new();
    let mut non_periodic_faces = Vec::new();
    let mut periodic_jobs: Vec<(usize, PeriodicStepSurface)> = Vec::new();

    for (face_idx, (face, periodic)) in trimmed.faces.iter().zip(periodic_surfaces).enumerate() {
        if let Some(ps) = periodic {
            periodic_jobs.push((face_idx, ps));
        } else {
            non_periodic_indices.push(face_idx);
            non_periodic_faces.push(face.clone());
        }
    }

    let non_periodic_shell = CompressedTrimmedShell {
        vertices: trimmed.vertices.clone(),
        edges: trimmed.edges.clone(),
        faces: non_periodic_faces,
    };

    let (meshed_base, periodic_results) = std::thread::scope(|scope| {
        let handles: Vec<_> = periodic_jobs
            .into_iter()
            .map(|(face_idx, ps)| {
                let comp_face = &trimmed.faces[face_idx];
                scope.spawn(move || {
                    let poly = tessellate_periodic_face(trimmed, comp_face, ps, tol_file_units);
                    (face_idx, poly)
                })
            })
            .collect();

        let meshed_base = non_periodic_shell.robust_triangulation(tol_file_units);
        let periodic_results: Vec<(usize, Option<PolygonMesh>)> =
            handles.into_iter().filter_map(|h| h.join().ok()).collect();
        (meshed_base, periodic_results)
    });

    let mut ordered_faces: Vec<Option<CompressedFace<Option<PolygonMesh>>>> =
        vec![None; trimmed.faces.len()];

    for (face_idx, meshed_face) in non_periodic_indices.into_iter().zip(meshed_base.faces) {
        ordered_faces[face_idx] = Some(meshed_face);
    }

    for (face_idx, poly) in periodic_results {
        let src_face = &trimmed.faces[face_idx];
        let boundaries = src_face
            .boundaries
            .iter()
            .map(|wire| {
                wire.iter()
                    .map(|eu| CompressedEdgeIndex {
                        index: eu.index,
                        orientation: eu.orientation,
                    })
                    .collect()
            })
            .collect();
        ordered_faces[face_idx] = Some(CompressedFace {
            boundaries,
            orientation: src_face.orientation,
            surface: poly.filter(|m| !m.faces().is_empty()),
        });
    }

    let faces = ordered_faces
        .into_iter()
        .enumerate()
        .map(|(face_idx, slot)| {
            slot.unwrap_or_else(|| CompressedFace {
                boundaries: Vec::new(),
                orientation: trimmed.faces[face_idx].orientation,
                surface: None,
            })
        })
        .collect();

    CompressedShell {
        vertices: meshed_base.vertices,
        edges: meshed_base.edges,
        faces,
        vertex_stable_ids: meshed_base.vertex_stable_ids,
        edge_stable_ids: meshed_base.edge_stable_ids,
        face_stable_ids: meshed_base.face_stable_ids,
    }
}

/// Finds the parameter `t` on `curve` minimizing `||curve(t) - target||` using
/// dense multi-start sampling followed by damped 1D Gauss-Newton + golden-section
/// fallback so high-degree curves never fail or fall into local minima.
fn find_closest_curve_param(curve: &Curve3D, target: Point3) -> Option<(f64, f64)> {
    let (t0, t1) = curve.range_tuple();
    if !t0.is_finite() || !t1.is_finite() || t1 <= t0 {
        return None;
    }

    const SAMPLES: usize = 128;
    let mut samples = [(0.0_f64, f64::INFINITY); SAMPLES + 1];
    for (i, slot) in samples.iter_mut().enumerate() {
        let t = t0 + (t1 - t0) * (i as f64 / SAMPLES as f64);
        let d2 = curve.subs(t).distance2(target);
        *slot = (t, if d2.is_finite() { d2 } else { f64::INFINITY });
    }

    // Collect local minima indices (including domain endpoints).
    let mut seeds: Vec<(f64, f64)> = Vec::new();
    for i in 0..=SAMPLES {
        let d = samples[i].1;
        let left_ok = i == 0 || d <= samples[i - 1].1;
        let right_ok = i == SAMPLES || d <= samples[i + 1].1;
        if left_ok && right_ok && d.is_finite() {
            seeds.push(samples[i]);
        }
    }
    seeds.sort_by(|a, b| a.1.total_cmp(&b.1));
    seeds.truncate(4);

    let step = (t1 - t0) / SAMPLES as f64;
    let mut best_t = t0;
    let mut best_d2 = f64::INFINITY;

    for (seed_t, _) in seeds {
        let (t_gn, d2_gn) = refine_curve_param(curve, target, seed_t, t0, t1, step);
        if d2_gn < best_d2 {
            best_d2 = d2_gn;
            best_t = t_gn;
        }
    }

    best_d2.is_finite().then_some((best_t, best_d2.sqrt()))
}

fn refine_curve_param(
    curve: &Curve3D,
    target: Point3,
    seed_t: f64,
    t0: f64,
    t1: f64,
    bracket_half: f64,
) -> (f64, f64) {
    let mut t = seed_t.clamp(t0, t1);
    let mut d2 = curve.subs(t).distance2(target);

    for _ in 0..24 {
        let diff = curve.subs(t) - target;
        let d1 = curve.der(t);
        let d2_vec = curve.der2(t);
        let grad = diff.dot(d1);
        let hess = d1.dot(d1) + diff.dot(d2_vec);
        let denom = if hess > 1e-12 {
            hess
        } else {
            d1.dot(d1).max(1e-12)
        };
        let delta = -grad / denom;
        if !delta.is_finite() || delta.abs() < 1e-13 {
            break;
        }

        let mut alpha = 1.0_f64;
        let mut accepted = false;
        while alpha >= 1.0 / 64.0 {
            let cand_t = (t + alpha * delta).clamp(t0, t1);
            let cand_d2 = curve.subs(cand_t).distance2(target);
            if cand_d2.is_finite() && cand_d2 < d2 {
                t = cand_t;
                d2 = cand_d2;
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }
        if !accepted {
            break;
        }
    }

    // Golden-section polish inside a tight bracket around `t` in case derivatives were flat.
    let mut a = (t - bracket_half).max(t0);
    let mut b = (t + bracket_half).min(t1);
    let phi = 0.618_033_988_749_894_9_f64;
    let mut c = b - phi * (b - a);
    let mut d = a + phi * (b - a);
    let mut fc = curve.subs(c).distance2(target);
    let mut fd = curve.subs(d).distance2(target);
    for _ in 0..32 {
        if (b - a).abs() <= 1e-12 * (t1 - t0).max(1.0) {
            break;
        }
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - phi * (b - a);
            fc = curve.subs(c).distance2(target);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + phi * (b - a);
            fd = curve.subs(d).distance2(target);
        }
    }
    let mid = 0.5 * (a + b);
    let mid_d2 = curve.subs(mid).distance2(target);
    if mid_d2.is_finite() && mid_d2 < d2 {
        (mid, mid_d2)
    } else {
        (t, d2)
    }
}

fn tessellate_periodic_face<T>(
    trimmed: &CompressedTrimmedShell<Point3, Curve3D, Surface, T>,
    comp_face: &CompressedTrimmedFace<Surface, T>,
    periodic_surface: PeriodicStepSurface,
    tol_file_units: f64,
) -> Option<PolygonMesh> {
    let mut local_vertices: Vec<Point3> = Vec::new();
    let mut vertex_remap: HashMap<usize, usize> = HashMap::new();
    let mut local_edges: Vec<CompressedEdge<PeriodicStepCurve>> = Vec::new();
    let mut edge_remap: HashMap<usize, usize> = HashMap::new();

    let mut local_boundaries: Vec<Vec<CompressedEdgeIndex>> =
        Vec::with_capacity(comp_face.boundaries.len());

    for wire in &comp_face.boundaries {
        let mut local_wire = Vec::with_capacity(wire.len());
        for edge_use in wire {
            let local_edge_idx = if let Some(&existing) = edge_remap.get(&edge_use.index) {
                existing
            } else {
                let src_edge = trimmed.edges.get(edge_use.index)?;
                let v0 = *vertex_remap.entry(src_edge.vertices.0).or_insert_with(|| {
                    let idx = local_vertices.len();
                    local_vertices.push(trimmed.vertices[src_edge.vertices.0]);
                    idx
                });
                let v1 = *vertex_remap.entry(src_edge.vertices.1).or_insert_with(|| {
                    let idx = local_vertices.len();
                    local_vertices.push(trimmed.vertices[src_edge.vertices.1]);
                    idx
                });
                let idx = local_edges.len();
                local_edges.push(CompressedEdge {
                    vertices: (v0, v1),
                    curve: PeriodicStepCurve(src_edge.curve.clone()),
                });
                edge_remap.insert(edge_use.index, idx);
                idx
            };
            local_wire.push(CompressedEdgeIndex {
                index: local_edge_idx,
                orientation: edge_use.orientation,
            });
        }
        local_boundaries.push(local_wire);
    }

    let single_face_shell = CompressedShell {
        vertices: local_vertices,
        edges: local_edges,
        faces: vec![CompressedFace {
            boundaries: local_boundaries,
            orientation: comp_face.orientation,
            surface: periodic_surface,
        }],
        vertex_stable_ids: None,
        edge_stable_ids: None,
        face_stable_ids: None,
    };

    let meshed_single = single_face_shell.robust_triangulation(tol_file_units);
    meshed_single.faces.into_iter().next()?.surface
}

/// Wrapper around a closed/periodic STEP `BsplineSurface` or `NurbsSurface` that
/// reports its geometric U/V periods to `monstertruck-meshing`, wraps evaluations
/// modulo those periods, and provides reliable multi-start periodic UV projection.
#[derive(Clone, Debug)]
struct PeriodicStepSurface {
    inner: Surface,
    u_range: (f64, f64),
    v_range: (f64, f64),
    u_period: Option<f64>,
    v_period: Option<f64>,
}

impl PeriodicStepSurface {
    /// Returns `Some(PeriodicStepSurface)` only when `surface` is a `BsplineSurface`
    /// or `NurbsSurface` that is geometrically closed in U and/or V.
    fn try_from_surface(surface: &Surface) -> Option<Self> {
        if !matches!(
            surface,
            Surface::BsplineSurface(_) | Surface::NurbsSurface(_)
        ) {
            return None;
        }
        let (Some((u0, u1)), Some((v0, v1))) = surface.try_range_tuple() else {
            return None;
        };
        if !u0.is_finite()
            || !u1.is_finite()
            || !v0.is_finite()
            || !v1.is_finite()
            || u1 <= u0 + 1e-9
            || v1 <= v0 + 1e-9
        {
            return None;
        }

        let u_closed = is_axis_closed(surface, (u0, u1), (v0, v1), true);
        let v_closed = is_axis_closed(surface, (u0, u1), (v0, v1), false);
        if !u_closed && !v_closed {
            return None;
        }

        Some(Self {
            inner: surface.clone(),
            u_range: (u0, u1),
            v_range: (v0, v1),
            u_period: u_closed.then_some(u1 - u0),
            v_period: v_closed.then_some(v1 - v0),
        })
    }

    #[inline]
    fn wrap_axis(val: f64, (lo, hi): (f64, f64), period: Option<f64>) -> f64 {
        if let Some(p) = period {
            if val >= lo && val <= hi {
                val
            } else {
                lo + (val - lo).rem_euclid(p)
            }
        } else {
            val.clamp(lo, hi)
        }
    }

    #[inline]
    fn wrap_uv(&self, u: f64, v: f64) -> (f64, f64) {
        (
            Self::wrap_axis(u, self.u_range, self.u_period),
            Self::wrap_axis(v, self.v_range, self.v_period),
        )
    }

    #[inline]
    fn unwrap_axis_near(val: f64, hint: f64, period: Option<f64>) -> f64 {
        if let Some(p) = period {
            let k = ((hint - val) / p).round();
            val + k * p
        } else {
            val
        }
    }

    fn damped_gauss_newton(
        &self,
        point: Point3,
        start: (f64, f64),
        trials: usize,
    ) -> (f64, f64, f64) {
        let (mut u, mut v) = self.wrap_uv(start.0, start.1);
        let mut best_d2 = self.inner.evaluate(u, v).distance2(point);
        let max_iters = trials.clamp(16, 80);

        for _ in 0..max_iters {
            if best_d2 <= 1e-22 {
                break;
            }
            let r = self.inner.evaluate(u, v) - point;
            let su = self.inner.derivative_u(u, v);
            let sv = self.inner.derivative_v(u, v);

            // Normal equations J^T J delta = -J^T r with Levenberg-Marquardt damping.
            let a00 = su.dot(su);
            let a01 = su.dot(sv);
            let a11 = sv.dot(sv);
            let b0 = -r.dot(su);
            let b1 = -r.dot(sv);

            let scale = (a00 + a11).max(1e-12);
            let damp = 1e-9 * scale;
            let d00 = a00 + damp;
            let d11 = a11 + damp;
            let det = d00 * d11 - a01 * a01;
            if !det.is_finite() || det.abs() <= 1e-24 {
                break;
            }

            let mut du = (d11 * b0 - a01 * b1) / det;
            let mut dv = (-a01 * b0 + d00 * b1) / det;
            if !du.is_finite() || !dv.is_finite() {
                break;
            }

            // Cap single-step jump to 25% of the domain span so Newton stays local.
            let max_du = 0.25 * (self.u_range.1 - self.u_range.0);
            let max_dv = 0.25 * (self.v_range.1 - self.v_range.0);
            let ratio = (du.abs() / max_du).max(dv.abs() / max_dv);
            if ratio > 1.0 {
                du /= ratio;
                dv /= ratio;
            }

            if du.abs() < 1e-13 && dv.abs() < 1e-13 {
                break;
            }

            let mut alpha = 1.0_f64;
            let mut accepted = false;
            while alpha >= 1.0 / 64.0 {
                let (cu, cv) = self.wrap_uv(u + alpha * du, v + alpha * dv);
                let cand_d2 = self.inner.evaluate(cu, cv).distance2(point);
                if cand_d2.is_finite() && cand_d2 < best_d2 {
                    u = cu;
                    v = cv;
                    best_d2 = cand_d2;
                    accepted = true;
                    break;
                }
                alpha *= 0.5;
            }
            if !accepted {
                break;
            }
        }

        (u, v, best_d2)
    }

    fn global_project(&self, point: Point3, trials: usize) -> (f64, f64, f64) {
        const NU: usize = 20;
        const NV: usize = 24;
        let (u0, u1) = self.u_range;
        let (v0, v1) = self.v_range;

        // Avoid duplicating the seam at u1 / v1 when that axis is periodic.
        let u_steps = if self.u_period.is_some() { NU } else { NU + 1 };
        let v_steps = if self.v_period.is_some() { NV } else { NV + 1 };

        let mut top: [(f64, f64, f64); 4] = [(u0, v0, f64::INFINITY); 4];
        for iu in 0..u_steps {
            let u = u0 + (u1 - u0) * (iu as f64 / NU as f64);
            for iv in 0..v_steps {
                let v = v0 + (v1 - v0) * (iv as f64 / NV as f64);
                let d2 = self.inner.evaluate(u, v).distance2(point);
                if d2.is_finite() && d2 < top[3].2 {
                    top[3] = (u, v, d2);
                    top.sort_by(|a, b| a.2.total_cmp(&b.2));
                }
            }
        }

        let mut best = (u0, v0, f64::INFINITY);
        for &(su, sv, _) in &top {
            let cand = self.damped_gauss_newton(point, (su, sv), trials);
            if cand.2 < best.2 {
                best = cand;
            }
        }
        best
    }

    fn project_with_hint(
        &self,
        point: Point3,
        hint: SearchParameterHint2D,
        trials: usize,
        strict_tol: Option<f64>,
    ) -> Option<(f64, f64)> {
        let hint_uv = match hint {
            SearchParameterHint2D::Parameter(hu, hv) if hu.is_finite() && hv.is_finite() => {
                Some((hu, hv))
            }
            SearchParameterHint2D::Range((ua, ub), (va, vb)) => {
                Some((0.5 * (ua + ub), 0.5 * (va + vb)))
            }
            _ => None,
        };

        let accept_hint_d2 = 1e-8_f64;
        let (u, v, d2) = if let Some((hu, hv)) = hint_uv {
            let cand = self.damped_gauss_newton(point, (hu, hv), trials);
            if cand.2 <= accept_hint_d2 {
                cand
            } else {
                let glob = self.global_project(point, trials);
                if glob.2 < cand.2 { glob } else { cand }
            }
        } else {
            self.global_project(point, trials)
        };

        if !u.is_finite() || !v.is_finite() || !d2.is_finite() {
            return None;
        }
        if let Some(tol) = strict_tol
            && d2 > tol * tol
        {
            return None;
        }

        let (out_u, out_v) = if let Some((hu, hv)) = hint_uv {
            (
                Self::unwrap_axis_near(u, hu, self.u_period),
                Self::unwrap_axis_near(v, hv, self.v_period),
            )
        } else {
            (u, v)
        };

        Some((out_u, out_v))
    }
}

fn is_axis_closed(
    surface: &Surface,
    (u0, u1): (f64, f64),
    (v0, v1): (f64, f64),
    check_u: bool,
) -> bool {
    const SAMPLES: usize = 16;
    let mut max_seam_gap = 0.0_f64;
    let mut max_diameter = 0.0_f64;

    for i in 0..=SAMPLES {
        let s = i as f64 / SAMPLES as f64;
        let (p0, p1, p_q1, p_mid, p_q3) = if check_u {
            let v = v0 + (v1 - v0) * s;
            (
                surface.evaluate(u0, v),
                surface.evaluate(u1, v),
                surface.evaluate(u0 + 0.25 * (u1 - u0), v),
                surface.evaluate(u0 + 0.50 * (u1 - u0), v),
                surface.evaluate(u0 + 0.75 * (u1 - u0), v),
            )
        } else {
            let u = u0 + (u1 - u0) * s;
            (
                surface.evaluate(u, v0),
                surface.evaluate(u, v1),
                surface.evaluate(u, v0 + 0.25 * (v1 - v0)),
                surface.evaluate(u, v0 + 0.50 * (v1 - v0)),
                surface.evaluate(u, v0 + 0.75 * (v1 - v0)),
            )
        };
        let gap = p0.distance(p1);
        let diam = p0
            .distance(p_q1)
            .max(p0.distance(p_mid))
            .max(p0.distance(p_q3));
        if !gap.is_finite() || !diam.is_finite() {
            return false;
        }
        max_seam_gap = max_seam_gap.max(gap);
        max_diameter = max_diameter.max(diam);
    }

    max_diameter > 1e-5 && max_seam_gap <= (1e-4 * max_diameter).max(1e-5)
}

impl ParametricSurface for PeriodicStepSurface {
    type Point = Point3;
    type Vector = Vector3;

    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.evaluate(u, v)
    }

    #[inline]
    fn derivative_u(&self, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.derivative_u(u, v)
    }

    #[inline]
    fn derivative_v(&self, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.derivative_v(u, v)
    }

    #[inline]
    fn derivative_uu(&self, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.derivative_uu(u, v)
    }

    #[inline]
    fn derivative_uv(&self, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.derivative_uv(u, v)
    }

    #[inline]
    fn derivative_vv(&self, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.derivative_vv(u, v)
    }

    #[inline]
    fn derivative_mn(&self, m: usize, n: usize, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.derivative_mn(m, n, u, v)
    }

    #[inline]
    fn parameter_range(&self) -> (ParameterRange, ParameterRange) {
        (
            (
                Bound::Included(self.u_range.0),
                Bound::Included(self.u_range.1),
            ),
            (
                Bound::Included(self.v_range.0),
                Bound::Included(self.v_range.1),
            ),
        )
    }

    #[inline]
    fn period_u(&self) -> Option<f64> {
        self.u_period
    }

    #[inline]
    fn period_v(&self) -> Option<f64> {
        self.v_period
    }
}

impl ParametricSurface3D for PeriodicStepSurface {
    #[inline]
    fn normal(&self, u: f64, v: f64) -> Vector3 {
        let (u, v) = self.wrap_uv(u, v);
        self.inner.normal(u, v)
    }
}

impl ParameterDivision2D for PeriodicStepSurface {
    #[inline]
    fn parameter_division(
        &self,
        range: ((f64, f64), (f64, f64)),
        tol: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        algo::surface::parameter_division(self, range, tol)
    }
}

impl SearchParameter<SurfaceParameter> for PeriodicStepSurface {
    type Point = Point3;

    #[inline]
    fn search_parameter<H: Into<SearchParameterHint2D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<(f64, f64)> {
        self.project_with_hint(point, hint.into(), trials, Some(1e-4))
    }
}

impl SearchNearestParameter<SurfaceParameter> for PeriodicStepSurface {
    type Point = Point3;

    #[inline]
    fn search_nearest_parameter<H: Into<SearchParameterHint2D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<(f64, f64)> {
        self.project_with_hint(point, hint.into(), trials, None)
    }
}

/// Thin wrapper around STEP `Curve3D` so `CompressedShell` delegates boundary
/// projection directly to `PolyBoundaryPiece::try_new` with continuous periodic
/// UV unwrapping along each loop.
#[derive(Clone, Debug)]
struct PeriodicStepCurve(Curve3D);

impl ParametricCurve for PeriodicStepCurve {
    type Point = Point3;
    type Vector = Vector3;

    #[inline]
    fn evaluate(&self, t: f64) -> Point3 {
        self.0.evaluate(t)
    }

    #[inline]
    fn derivative(&self, t: f64) -> Vector3 {
        self.0.derivative(t)
    }

    #[inline]
    fn derivative_2(&self, t: f64) -> Vector3 {
        self.0.derivative_2(t)
    }

    #[inline]
    fn derivative_n(&self, n: usize, t: f64) -> Vector3 {
        self.0.derivative_n(n, t)
    }

    #[inline]
    fn parameter_range(&self) -> ParameterRange {
        self.0.parameter_range()
    }

    #[inline]
    fn period(&self) -> Option<f64> {
        self.0.period()
    }
}

impl BoundedCurve for PeriodicStepCurve {}

impl ParameterDivision1D for PeriodicStepCurve {
    type Point = Point3;

    #[inline]
    fn parameter_division(&self, range: (f64, f64), tol: f64) -> (Vec<f64>, Vec<Point3>) {
        self.0.parameter_division(range, tol)
    }
}

impl ParameterBoundary2D<PeriodicStepSurface> for PeriodicStepCurve {
    #[inline]
    fn parameter_boundary_2d(
        &self,
        _surface: &PeriodicStepSurface,
        _tolerance: f64,
    ) -> Option<Vec<Point2>> {
        // Defer to `PolyBoundaryPiece::try_new`, which projects the entire
        // concatenated 3D wire sequentially with continuous periodic hints.
        None
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::PI;

    use chip_fixtures::sample_periodic_helical_sweep_step;
    use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
    use monstertruck_io::step::load::step_geometry::{BsplineSurface, KnotVector};

    use super::*;
    use crate::{CadKernel, MonstertruckKernel, SurfaceKind};

    #[test]
    fn periodic_helical_sweep_step_trims_seam_edge_and_tessellates_full_watertight_solid() {
        let step = sample_periodic_helical_sweep_step();
        let doc = MonstertruckKernel
            .load_step_bytes(
                "helical_sweep.step",
                step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .expect("load periodic helical sweep STEP");

        assert_eq!(doc.bodies.len(), 1);
        assert_eq!(
            doc.faces.len(),
            3,
            "expected 1 helical wall + 2 planar end caps"
        );

        // Seam edge must be trimmed to Z ∈ [0, 40] (not protruding to untrimmed [-5, 45]).
        let seam_edge = doc
            .edges
            .iter()
            .find(|e| matches!(e.curve, crate::CurveKind::OtherNurbs))
            .expect("helical B-spline seam edge present");
        assert!(
            seam_edge.aabb_mm.min.z >= -0.05 && seam_edge.aabb_mm.max.z <= 40.05,
            "seam edge must be trimmed to Z ∈ [0, 40], got [{:.3}, {:.3}]",
            seam_edge.aabb_mm.min.z,
            seam_edge.aabb_mm.max.z
        );

        // Helical periodic NURBS face must cover the full Z ∈ [0, 40] span and full area
        // (arc length of helix centerline ≈ sqrt((12*π)² + 40²) ≈ 54.9 mm -> area ≈ 2π*4*54.9 ≈ 1380 mm²).
        let helix_face = doc
            .faces
            .iter()
            .find(|f| matches!(f.surface, SurfaceKind::OtherNurbs))
            .expect("helical NURBS face present");
        assert!(
            helix_face.area_mm2 > 1100.0 && helix_face.area_mm2 < 1700.0,
            "expected full helical tube area ~1380 mm², got {:.2} mm²",
            helix_face.area_mm2
        );
        assert!(
            helix_face.aabb_mm.min.z >= -0.1 && helix_face.aabb_mm.min.z <= 0.1,
            "helical face min Z should be ~0, got {:.3}",
            helix_face.aabb_mm.min.z
        );
        assert!(
            helix_face.aabb_mm.max.z >= 39.9 && helix_face.aabb_mm.max.z <= 40.1,
            "helical face max Z should be ~40, got {:.3}",
            helix_face.aabb_mm.max.z
        );

        // Divergence-theorem signed volume must be positive and match Cavalieri volume
        // π * r² * H = π * 16 * 40 ≈ 2010.62 mm³!
        let signed_vol_mm3: f64 = doc
            .render_mesh
            .pick_triangles
            .iter()
            .map(|t| t.v0_mm.dot(t.v1_mm.cross(t.v2_mm)) / 6.0)
            .sum();
        let expected_vol = PI * 16.0 * 40.0;
        let rel_err = (signed_vol_mm3 - expected_vol).abs() / expected_vol;
        assert!(
            rel_err < 0.03,
            "expected watertight signed volume ≈ {expected_vol:.2} mm³, got {signed_vol_mm3:.2} mm³ (rel_err={rel_err:.4})"
        );
    }

    #[test]
    fn open_bspline_surface_is_not_classified_as_periodic() {
        let ku = KnotVector::bezier_knot(1);
        let kv = KnotVector::bezier_knot(1);
        let ctrl = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 10.0, 0.0)],
            vec![Point3::new(10.0, 0.0, 0.0), Point3::new(10.0, 10.0, 2.0)],
        ];
        let bsp = BsplineSurface::new((ku, kv), ctrl);
        let surf = Surface::BsplineSurface(bsp);
        assert!(
            PeriodicStepSurface::try_from_surface(&surf).is_none(),
            "open B-spline patch must not be treated as periodic"
        );
    }
}
