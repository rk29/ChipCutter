use std::f64::consts::TAU;

use chip_cam::{CamError, rect_region_2d};
use chip_geom2d::{PocketRegion2D, RegionSet2D, triangulate_polygon_with_holes_2d};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};

use super::SoftJawBoltPattern;

/// Discretizes an `ArcPolyline2D` into a closed 2D vertex ring (`<= 0.02 mm` arc chord sag),
/// snapping points within `5e-3 mm` of the jaw boundary rectangle `[0..W] × [0..D]` cleanly onto
/// the boundary and quantizing onto a `0.001 mm` (`1 µm`) grid for exact manifold edge sharing.
fn discretize_closed_loop(poly: &ArcPolyline2D, w: f64, d: f64) -> Vec<DVec2> {
    let mut pts = Vec::new();
    const CHORD_TOL: f64 = 0.02;
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, .. } => pts.push(snap_jaw_pt(start, w, d)),
            ArcSegment2D::Arc {
                start,
                center,
                radius,
                ccw,
                ..
            } => {
                let sweep = seg.signed_sweep_rad().abs();
                let step_angle = if radius > CHORD_TOL {
                    2.0 * (1.0 - CHORD_TOL / radius).clamp(-1.0, 1.0).acos()
                } else {
                    0.5
                };
                let steps = ((sweep / step_angle.max(0.05)).ceil() as usize).clamp(2, 64);
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let dir = if ccw { 1.0 } else { -1.0 };
                for k in 0..steps {
                    let ang = a0 + dir * sweep * (k as f64 / steps as f64);
                    let p = center + DVec2::new(ang.cos(), ang.sin()) * radius;
                    pts.push(snap_jaw_pt(p, w, d));
                }
            }
        }
    }
    clean_ring_vertices(&mut pts);
    pts
}

fn snap_jaw_pt(p: DVec2, w: f64, d: f64) -> DVec2 {
    let snap = |val: f64, lo: f64, hi: f64| -> f64 {
        if (val - lo).abs() <= 5e-3 {
            lo
        } else if (val - hi).abs() <= 5e-3 {
            hi
        } else {
            (val * 1000.0).round() * 0.001
        }
    };
    DVec2::new(snap(p.x, 0.0, w), snap(p.y, 0.0, d))
}

fn clean_ring_vertices(pts: &mut Vec<DVec2>) {
    let mut out: Vec<DVec2> = Vec::with_capacity(pts.len());
    for &p in pts.iter() {
        if out.last().is_none_or(|prev| prev.distance(p) > 2e-3) {
            out.push(p);
        }
    }
    while out.len() > 2 && out[0].distance(out[out.len() - 1]) <= 2e-3 {
        out.pop();
    }
    *pts = out;
}

fn ring_to_line_polyline(pts: &[DVec2]) -> ArcPolyline2D {
    let n = pts.len();
    if n < 3 {
        return ArcPolyline2D::default();
    }
    let mut segments = Vec::with_capacity(n);
    for i in 0..n {
        segments.push(ArcSegment2D::Line {
            start: pts[i],
            end: pts[(i + 1) % n],
        });
    }
    ArcPolyline2D {
        segments,
        closed: true,
    }
}

fn signed_area_ring(pts: &[DVec2]) -> f64 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    0.5 * sum
}

fn ensure_ring_orientation(mut pts: Vec<DVec2>, ccw: bool) -> Vec<DVec2> {
    let area = signed_area_ring(&pts);
    if (ccw && area < 0.0) || (!ccw && area > 0.0) {
        pts.reverse();
    }
    pts
}

/// Inserts any point from `candidates` that lies strictly on an edge `(a, b)` of `ring` so
/// adjacent planar faces on `Z = Z_floor` share identical half-edges with zero T-junctions.
fn subdivide_ring_collinear(ring: &mut Vec<DVec2>, candidates: &[DVec2]) {
    let n = ring.len();
    if n < 2 {
        return;
    }
    let mut out = Vec::with_capacity(n + 8);
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        out.push(a);
        let ab = b - a;
        let len2 = ab.length_squared();
        if len2 <= 1e-8 {
            continue;
        }
        let len = len2.sqrt();
        let mut splits: Vec<(f64, DVec2)> = Vec::new();
        for &c in candidates {
            if c.distance(a) <= 2e-3 || c.distance(b) <= 2e-3 {
                continue;
            }
            let t = (c - a).dot(ab) / len2;
            if t <= 1e-5 || t >= 1.0 - 1e-5 {
                continue;
            }
            let proj = a + ab * t;
            if proj.distance(c) <= 2e-3 && (t * len) > 2e-3 && ((1.0 - t) * len) > 2e-3 {
                splits.push((t, c));
            }
        }
        splits.sort_by(|x, y| x.0.total_cmp(&y.0));
        splits.dedup_by(|x, y| x.1.distance(y.1) <= 2e-3);
        for (_, pt) in splits {
            out.push(pt);
        }
    }
    clean_ring_vertices(&mut out);
    *ring = out;
}

/// Generates a guaranteed-watertight binary STL byte buffer for a single soft jaw in jaw-local
/// coordinates `[0..w] × [0..d] × [0..h]` with contoured step pocket (`grip_depth` deep from `z = h`)
/// and optional horizontal counterbored mounting bolt holes piercing `Y = 0..d`.
pub(super) fn build_watertight_soft_jaw_stl_bytes(
    header_label: &str,
    w: f64,
    d: f64,
    h: f64,
    grip_depth: f64,
    pocket_local: &RegionSet2D,
    bolt_pattern: SoftJawBoltPattern,
) -> Result<(Vec<u8>, usize), CamError> {
    let z_floor = ((h - grip_depth) * 1000.0).round() * 0.001;
    let w_q = (w * 1000.0).round() * 0.001;
    let d_q = (d * 1000.0).round() * 0.001;
    let h_q = (h * 1000.0).round() * 0.001;

    // 1. Discretize pocket polygons into straight-segment regions so `jaw_rect \ pocket` shares
    //    exact vertices along every pocket wall and jaw perimeter boundary!
    let mut disc_pocket_regs: Vec<PocketRegion2D> = Vec::new();
    for reg in &pocket_local.regions {
        let outer_pts =
            ensure_ring_orientation(discretize_closed_loop(&reg.outer_ccw, w_q, d_q), true);
        if outer_pts.len() < 3 || signed_area_ring(&outer_pts) <= 1e-4 {
            continue;
        }
        let mut islands = Vec::new();
        for isl in &reg.islands_cw {
            let isl_pts = ensure_ring_orientation(discretize_closed_loop(isl, w_q, d_q), false);
            if isl_pts.len() >= 3 && signed_area_ring(&isl_pts).abs() > 1e-4 {
                islands.push(ring_to_line_polyline(&isl_pts));
            }
        }
        disc_pocket_regs.push(PocketRegion2D::new(
            ring_to_line_polyline(&outer_pts),
            islands,
        ));
    }

    let jaw_rect = rect_region_2d(0.0, 0.0, w_q, d_q);
    let pocket_poly_set = RegionSet2D::from_regions(disc_pocket_regs).intersect(&jaw_rect);
    let top_land_set = jaw_rect.difference(&pocket_poly_set);

    let extract_rings = |set: &RegionSet2D| -> Vec<(Vec<DVec2>, Vec<Vec<DVec2>>)> {
        let mut out = Vec::new();
        for reg in &set.regions {
            let outer =
                ensure_ring_orientation(discretize_closed_loop(&reg.outer_ccw, w_q, d_q), true);
            if outer.len() < 3 || signed_area_ring(&outer) <= 1e-4 {
                continue;
            }
            let mut holes = Vec::new();
            for isl in &reg.islands_cw {
                let h_ring = ensure_ring_orientation(discretize_closed_loop(isl, w_q, d_q), false);
                if h_ring.len() >= 3 && signed_area_ring(&h_ring).abs() > 1e-4 {
                    holes.push(h_ring);
                }
            }
            out.push((outer, holes));
        }
        out
    };

    let mut pocket_rings = extract_rings(&pocket_poly_set);
    let mut land_rings = extract_rings(&top_land_set);

    // 2. Snap all vertices across pocket_rings, land_rings, and jaw corners onto one canonical 2D
    //    vertex table (`5e-3 mm` weld radius) so boolean-output vertices coincide bit-for-bit,
    //    then subdivide collinear segments so zero T-junctions can ever exist on `Z = Z_floor`.
    let mut all_z_floor_pts: Vec<DVec2> = vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(w_q, 0.0),
        DVec2::new(w_q, d_q),
        DVec2::new(0.0, d_q),
    ];
    let mut intern_2d = |p: DVec2| -> DVec2 {
        for &existing in &all_z_floor_pts {
            if existing.distance(p) <= 5e-3 {
                return existing;
            }
        }
        all_z_floor_pts.push(p);
        p
    };
    for (o, hs) in pocket_rings.iter_mut().chain(land_rings.iter_mut()) {
        for pt in o.iter_mut() {
            *pt = intern_2d(*pt);
        }
        clean_ring_vertices(o);
        for hr in hs.iter_mut() {
            for pt in hr.iter_mut() {
                *pt = intern_2d(*pt);
            }
            clean_ring_vertices(hr);
        }
    }

    for (o, hs) in &mut pocket_rings {
        subdivide_ring_collinear(o, &all_z_floor_pts);
        for hr in hs {
            subdivide_ring_collinear(hr, &all_z_floor_pts);
        }
    }
    for (o, hs) in &mut land_rings {
        subdivide_ring_collinear(o, &all_z_floor_pts);
        for hr in hs {
            subdivide_ring_collinear(hr, &all_z_floor_pts);
        }
    }

    let mut tris: Vec<[DVec3; 3]> = Vec::with_capacity(512);
    let mut push_tri = |a: DVec3, b: DVec3, c: DVec3| {
        if (b - a).cross(c - a).length_squared() > 1e-12 {
            tris.push([a, b, c]);
        }
    };

    // 3. Upper solid (`Z ∈ [z_floor, h_q]`): top face at `z = h_q` (+Z) and vertical walls
    for (outer, holes) in &land_rings {
        for [a, b, c] in triangulate_polygon_with_holes_2d(outer, holes) {
            push_tri(
                DVec3::new(a.x, a.y, h_q),
                DVec3::new(b.x, b.y, h_q),
                DVec3::new(c.x, c.y, h_q),
            );
        }
        // Vertical walls: for CCW outer ring, outward normal is to the RIGHT of `p0 -> p1`.
        // Quad `(p0_bot, p1_bot, p1_top, p0_top)` has outward normal `(p1 - p0) × +Z` (right!).
        for ring in std::iter::once(outer).chain(holes.iter()) {
            let n = ring.len();
            for i in 0..n {
                let p0 = ring[i];
                let p1 = ring[(i + 1) % n];
                let b0 = DVec3::new(p0.x, p0.y, z_floor);
                let b1 = DVec3::new(p1.x, p1.y, z_floor);
                let t1 = DVec3::new(p1.x, p1.y, h_q);
                let t0 = DVec3::new(p0.x, p0.y, h_q);
                push_tri(b0, b1, t1);
                push_tri(b0, t1, t0);
            }
        }
    }

    // 4. Pocket floor face at `Z = z_floor` (normal `+Z`)
    for (outer, holes) in &pocket_rings {
        for [a, b, c] in triangulate_polygon_with_holes_2d(outer, holes) {
            push_tri(
                DVec3::new(a.x, a.y, z_floor),
                DVec3::new(b.x, b.y, z_floor),
                DVec3::new(c.x, c.y, z_floor),
            );
        }
    }

    // 5. Bottom face at `Z = 0.0` (normal `-Z`)
    push_tri(
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(0.0, d_q, 0.0),
        DVec3::new(w_q, d_q, 0.0),
    );
    push_tri(
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(w_q, d_q, 0.0),
        DVec3::new(w_q, 0.0, 0.0),
    );

    // 6. Lower side walls (`Z ∈ [0, z_floor]`) sharing exact subdivided `Z = z_floor` vertices!
    let mut y0_xs = vec![0.0, w_q];
    let mut yd_xs = vec![0.0, w_q];
    let mut x0_ys = vec![0.0, d_q];
    let mut xw_ys = vec![0.0, d_q];
    for &p in &all_z_floor_pts {
        if p.y.abs() <= 2e-3 {
            y0_xs.push(p.x);
        }
        if (p.y - d_q).abs() <= 2e-3 {
            yd_xs.push(p.x);
        }
        if p.x.abs() <= 2e-3 {
            x0_ys.push(p.y);
        }
        if (p.x - w_q).abs() <= 2e-3 {
            xw_ys.push(p.y);
        }
    }
    let sort_dedup = |v: &mut Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.dedup_by(|a, b| (*a - *b).abs() <= 2e-3);
    };
    sort_dedup(&mut y0_xs);
    sort_dedup(&mut yd_xs);
    sort_dedup(&mut x0_ys);
    sort_dedup(&mut xw_ys);

    // Left lower wall (`X = 0`, normal `-X`) in `(Y, Z)` plane: CCW in `(-Y, +Z)` -> fan/triangulate
    {
        let mut outer_yz = vec![DVec2::new(d_q, 0.0), DVec2::new(0.0, 0.0)];
        for &y in &x0_ys {
            outer_yz.push(DVec2::new(y, z_floor));
        }
        // `(u = y, v = z)`: CCW in `(y, z)` gives normal `+X`, so reverse for `-X`
        let outer_ccw = ensure_ring_orientation(outer_yz, true);
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &[]) {
            push_tri(
                DVec3::new(0.0, a.x, a.y),
                DVec3::new(0.0, c.x, c.y),
                DVec3::new(0.0, b.x, b.y),
            );
        }
    }
    // Right lower wall (`X = w_q`, normal `+X`)
    {
        let mut outer_yz = vec![DVec2::new(0.0, 0.0), DVec2::new(d_q, 0.0)];
        for &y in xw_ys.iter().rev() {
            outer_yz.push(DVec2::new(y, z_floor));
        }
        let outer_ccw = ensure_ring_orientation(outer_yz, true);
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &[]) {
            push_tri(
                DVec3::new(w_q, a.x, a.y),
                DVec3::new(w_q, b.x, b.y),
                DVec3::new(w_q, c.x, c.y),
            );
        }
    }

    // 7. Determine whether horizontal mounting bolt holes + counterbores fit cleanly inside lower body
    let active_bolts: Option<(Vec<f64>, f64, f64, f64, f64)> = bolt_pattern
        .dimensions_mm()
        .and_then(|(spacing, bolt_dia, cbore_dia, cbore_depth, raw_z)| {
            if spacing <= 0.0 || bolt_dia <= 0.5 || cbore_dia < bolt_dia {
                return None;
            }
            let cbore_r = 0.5 * cbore_dia;
            let bolt_z = if raw_z > cbore_r + 1.5 && raw_z + cbore_r + 1.5 <= z_floor {
                raw_z
            } else {
                0.5 * z_floor
            };
            let bolt_z_q = (bolt_z * 1000.0).round() * 0.001;
            let half_sp = 0.5 * spacing;
            let x_l = 0.5 * w_q - half_sp;
            let x_r = 0.5 * w_q + half_sp;
            let fits_x =
                x_l - cbore_r >= 2.0 && x_r + cbore_r <= w_q - 2.0 && spacing > cbore_dia + 2.0;
            let fits_z = bolt_z_q - cbore_r >= 1.5 && bolt_z_q + cbore_r <= z_floor - 1.5;
            let eff_cbore_depth = cbore_depth.min(d_q - 3.0).max(0.0);
            if fits_x && fits_z && d_q > 4.0 {
                Some((
                    vec![
                        (x_l * 1000.0).round() * 0.001,
                        (x_r * 1000.0).round() * 0.001,
                    ],
                    bolt_z_q,
                    0.5 * bolt_dia,
                    cbore_r,
                    (eff_cbore_depth * 1000.0).round() * 0.001,
                ))
            } else {
                None
            }
        });

    const BOLT_SEGS: usize = 24;
    let circle_xz = |cx: f64, cz: f64, r: f64| -> Vec<DVec2> {
        (0..BOLT_SEGS)
            .map(|k| {
                let ang = TAU * (k as f64) / (BOLT_SEGS as f64);
                DVec2::new(
                    ((cx + r * ang.cos()) * 1000.0).round() * 0.001,
                    ((cz + r * ang.sin()) * 1000.0).round() * 0.001,
                )
            })
            .collect()
    };

    // Front lower wall (`Y = 0`, normal `-Y`), with counterbore holes when active
    {
        let mut outer_xz = vec![DVec2::new(0.0, 0.0), DVec2::new(w_q, 0.0)];
        for &x in y0_xs.iter().rev() {
            outer_xz.push(DVec2::new(x, z_floor));
        }
        let outer_ccw = ensure_ring_orientation(outer_xz, true);
        let front_holes: Vec<Vec<DVec2>> = match &active_bolts {
            Some((xs, bz, bolt_r, cbore_r, cbore_d)) => {
                let r_front = if *cbore_d > 0.25 { *cbore_r } else { *bolt_r };
                xs.iter()
                    .map(|&bx| ensure_ring_orientation(circle_xz(bx, *bz, r_front), false))
                    .collect()
            }
            None => Vec::new(),
        };
        // In `(X, Z)` plane, CCW `(a, b, c)` has `(b-a)×(c-a)` along `-Y` (`X × Z = -Y`)!
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &front_holes) {
            push_tri(
                DVec3::new(a.x, 0.0, a.y),
                DVec3::new(b.x, 0.0, b.y),
                DVec3::new(c.x, 0.0, c.y),
            );
        }
    }

    // Back lower wall (`Y = d_q`, normal `+Y`), with through-bolt holes when active
    {
        let mut outer_xz = vec![DVec2::new(0.0, 0.0), DVec2::new(w_q, 0.0)];
        for &x in yd_xs.iter().rev() {
            outer_xz.push(DVec2::new(x, z_floor));
        }
        let outer_ccw = ensure_ring_orientation(outer_xz, true);
        let back_holes: Vec<Vec<DVec2>> = match &active_bolts {
            Some((xs, bz, bolt_r, _, _)) => xs
                .iter()
                .map(|&bx| ensure_ring_orientation(circle_xz(bx, *bz, *bolt_r), false))
                .collect(),
            None => Vec::new(),
        };
        // Reverse `(a, c, b)` so normal points `+Y`!
        for [a, b, c] in triangulate_polygon_with_holes_2d(&outer_ccw, &back_holes) {
            push_tri(
                DVec3::new(a.x, d_q, a.y),
                DVec3::new(c.x, d_q, c.y),
                DVec3::new(b.x, d_q, b.y),
            );
        }
    }

    // Horizontal bolt sleeves & counterbore shoulder rings (inward normals into the bore cavity)
    if let Some((xs, bz, bolt_r, cbore_r, cbore_d)) = active_bolts {
        for bx in xs {
            let ring_bolt = circle_xz(bx, bz, bolt_r); // CCW in (X, Z)
            if cbore_d > 0.25 && cbore_r > bolt_r + 0.05 {
                let ring_cbore = circle_xz(bx, bz, cbore_r);
                // 1. Counterbore sleeve `Y = 0 -> cbore_d`:
                //    Front hole on `Y = 0` traverses CW (`a1 -> a0`), so sleeve traverses CCW (`a0 -> a1`)
                //    with inward normal `(a1 - a0) × (b1 - a0)` pointing toward the bore centerline!
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let p0 = ring_cbore[k];
                    let p1 = ring_cbore[k_next];
                    let a0 = DVec3::new(p0.x, 0.0, p0.y);
                    let a1 = DVec3::new(p1.x, 0.0, p1.y);
                    let b1 = DVec3::new(p1.x, cbore_d, p1.y);
                    let b0 = DVec3::new(p0.x, cbore_d, p0.y);
                    push_tri(a0, a1, b1);
                    push_tri(a0, b1, b0);
                }
                // 2. Counterbore shoulder annulus at `Y = cbore_d` (normal `-Y` facing front mouth):
                //    Outer edge `c0 -> c1` pairs with counterbore sleeve back edge `b1 -> b0` (`c1 -> c0`);
                //    Inner edge `i1 -> i0` pairs with through-bolt sleeve front edge `a0 -> a1` (`i0 -> i1`).
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let c0 = DVec3::new(ring_cbore[k].x, cbore_d, ring_cbore[k].y);
                    let c1 = DVec3::new(ring_cbore[k_next].x, cbore_d, ring_cbore[k_next].y);
                    let i0 = DVec3::new(ring_bolt[k].x, cbore_d, ring_bolt[k].y);
                    let i1 = DVec3::new(ring_bolt[k_next].x, cbore_d, ring_bolt[k_next].y);
                    push_tri(c0, c1, i1);
                    push_tri(c0, i1, i0);
                }
                // 3. Through-bolt sleeve `Y = cbore_d -> d_q`:
                //    Back hole on `Y = d_q` traverses CCW (`b0 -> b1`), pairing with sleeve back edge `b1 -> b0`.
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let p0 = ring_bolt[k];
                    let p1 = ring_bolt[k_next];
                    let a0 = DVec3::new(p0.x, cbore_d, p0.y);
                    let a1 = DVec3::new(p1.x, cbore_d, p1.y);
                    let b1 = DVec3::new(p1.x, d_q, p1.y);
                    let b0 = DVec3::new(p0.x, d_q, p0.y);
                    push_tri(a0, a1, b1);
                    push_tri(a0, b1, b0);
                }
            } else {
                // Straight through-bolt sleeve `Y = 0 -> d_q`
                for k in 0..BOLT_SEGS {
                    let k_next = (k + 1) % BOLT_SEGS;
                    let p0 = ring_bolt[k];
                    let p1 = ring_bolt[k_next];
                    let a0 = DVec3::new(p0.x, 0.0, p0.y);
                    let a1 = DVec3::new(p1.x, 0.0, p1.y);
                    let b1 = DVec3::new(p1.x, d_q, p1.y);
                    let b0 = DVec3::new(p0.x, d_q, p0.y);
                    push_tri(a0, a1, b1);
                    push_tri(a0, b1, b0);
                }
            }
        }
    }

    if tris.is_empty() {
        return Err(CamError::InvalidFixture {
            fixture_name: header_label.to_string(),
            reason: "soft jaw STL generator produced zero triangles".to_string(),
        });
    }

    let bytes = chip_cad::encode_binary_stl(header_label, &tris);
    Ok((bytes, tris.len()))
}
