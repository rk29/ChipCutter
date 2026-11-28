//! Deterministic DXF & SVG vector drawing fixtures.

use dxf::{
    Drawing,
    entities::{Entity, EntityType},
    enums::{DrawingUnits, Units},
};

/// Generates a deterministic, clean `96 DPI` `.svg` badge artwork (`120.0 × 75.0 mm` at `96 DPI`,
/// unitless `453.543307 × 283.464567 px`) featuring explicit cubic Bézier (`C`) curves for true
/// `G2`/`G3` biarc fitting and three CAM-ready `<g id="...">` layers:
/// - `OUTLINE`: `110 × 65 mm` badge perimeter with `R = 10 mm` cubic Bézier rounded corners,
/// - `EMBLEM_POCKET`: Centered cubic Bézier circular medallion (`R = 22 mm` outer, `R = 10 mm` nested
///   counter-island) for annular pocket clearing,
/// - `VCARVE_ACCENT`: Flanking symmetric cubic Bézier teardrop/leaf emblems for medial V-carving.
pub fn sample_svg_badge_bytes() -> Vec<u8> {
    // 96 DPI scale: 1 mm = 96 / 25.4 px
    const PX: f64 = 96.0 / 25.4;
    const KAPPA: f64 = 0.552_284_749_830_793_6;

    let w_px = 120.0 * PX;
    let h_px = 75.0 * PX;

    // Helper: cubic Bézier rounded rectangle path in SVG pixels (x0..x1, y0..y1, radius r in mm)
    let rounded_rect_cubic = |x0_mm: f64, y0_mm: f64, x1_mm: f64, y1_mm: f64, r_mm: f64| {
        let (x0, y0, x1, y1, r) = (x0_mm * PX, y0_mm * PX, x1_mm * PX, y1_mm * PX, r_mm * PX);
        let k = r * KAPPA;
        format!(
            "M {a:.4} {y0:.4} L {b:.4} {y0:.4} C {c:.4} {y0:.4} {x1:.4} {d:.4} {x1:.4} {e:.4} \
             L {x1:.4} {f:.4} C {x1:.4} {g:.4} {c:.4} {y1:.4} {b:.4} {y1:.4} \
             L {a:.4} {y1:.4} C {h:.4} {y1:.4} {x0:.4} {g:.4} {x0:.4} {f:.4} \
             L {x0:.4} {e:.4} C {x0:.4} {d:.4} {h:.4} {y0:.4} {a:.4} {y0:.4} Z",
            a = x0 + r,
            b = x1 - r,
            c = x1 - r + k,
            d = y0 + r - k,
            e = y0 + r,
            f = y1 - r,
            g = y1 - r + k,
            h = x0 + r - k,
        )
    };

    // Helper: 4-segment cubic Bézier circle centered at (cx_mm, cy_mm) with radius r_mm
    let circle_cubic = |cx_mm: f64, cy_mm: f64, r_mm: f64, cw: bool| {
        let (cx, cy, r) = (cx_mm * PX, cy_mm * PX, r_mm * PX);
        let k = r * KAPPA;
        if cw {
            format!(
                "M {rx:.4} {cy:.4} \
                 C {rx:.4} {ypk:.4} {xpk:.4} {by:.4} {cx:.4} {by:.4} \
                 C {xmk:.4} {by:.4} {lx:.4} {ypk:.4} {lx:.4} {cy:.4} \
                 C {lx:.4} {ymk:.4} {xmk:.4} {ty:.4} {cx:.4} {ty:.4} \
                 C {xpk:.4} {ty:.4} {rx:.4} {ymk:.4} {rx:.4} {cy:.4} Z",
                rx = cx + r,
                lx = cx - r,
                by = cy + r,
                ty = cy - r,
                xpk = cx + k,
                xmk = cx - k,
                ypk = cy + k,
                ymk = cy - k,
            )
        } else {
            format!(
                "M {rx:.4} {cy:.4} \
                 C {rx:.4} {ymk:.4} {xpk:.4} {ty:.4} {cx:.4} {ty:.4} \
                 C {xmk:.4} {ty:.4} {lx:.4} {ymk:.4} {lx:.4} {cy:.4} \
                 C {lx:.4} {ypk:.4} {xmk:.4} {by:.4} {cx:.4} {by:.4} \
                 C {xpk:.4} {by:.4} {rx:.4} {ypk:.4} {rx:.4} {cy:.4} Z",
                rx = cx + r,
                lx = cx - r,
                by = cy + r,
                ty = cy - r,
                xpk = cx + k,
                xmk = cx - k,
                ypk = cy + k,
                ymk = cy - k,
            )
        }
    };

    // Helper: symmetric curved cubic Bézier leaf/diamond accent centered at (cx_mm, cy_mm)
    let leaf_cubic = |cx_mm: f64, cy_mm: f64, half_w_mm: f64, half_h_mm: f64| {
        let (cx, cy, hw, hh) = (cx_mm * PX, cy_mm * PX, half_w_mm * PX, half_h_mm * PX);
        let top_y = cy - hh;
        let bot_y = cy + hh;
        format!(
            "M {cx:.4} {top_y:.4} \
             C {r1:.4} {y1:.4} {r1:.4} {y2:.4} {cx:.4} {bot_y:.4} \
             C {l1:.4} {y2:.4} {l1:.4} {y1:.4} {cx:.4} {top_y:.4} Z",
            r1 = cx + hw * 1.35,
            l1 = cx - hw * 1.35,
            y1 = cy - hh * 0.45,
            y2 = cy + hh * 0.45,
        )
    };

    let outline_d = rounded_rect_cubic(5.0, 5.0, 115.0, 70.0, 10.0);
    let emblem_outer_d = circle_cubic(60.0, 37.5, 22.0, true);
    let emblem_island_d = circle_cubic(60.0, 37.5, 10.0, false);
    let accent_left_d = leaf_cubic(22.0, 37.5, 6.0, 13.0);
    let accent_right_d = leaf_cubic(98.0, 37.5, 6.0, 13.0);

    format!(
        r##"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{w_px:.6}" height="{h_px:.6}" viewBox="0 0 {w_px:.6} {h_px:.6}">
  <g id="OUTLINE">
    <path d="{outline_d}" fill="none" stroke="#111111" stroke-width="1"/>
  </g>
  <g id="EMBLEM_POCKET">
    <path d="{emblem_outer_d} {emblem_island_d}" fill="#222222" fill-rule="evenodd"/>
  </g>
  <g id="VCARVE_ACCENT">
    <path d="{accent_left_d}" fill="#333333"/>
    <path d="{accent_right_d}" fill="#333333"/>
  </g>
</svg>
"##
    )
    .into_bytes()
}

/// - Outer `160 × 100 mm` rounded rectangle (`R = 8 mm` exact `LWPOLYLINE` bulge arcs) on layer `OUTLINE`,
/// - `4x Ø6.0 mm` `CIRCLE` mounting holes at corners `(12, 12)`, `(148, 12)`, `(148, 88)`, `(12, 88)` on layer `HOLES`,
/// - Central `60 × 34 mm` display cutout (`[50..110] × [33..67]`) on layer `POCKETS` with a deliberate
///   `0.03 mm` endpoint gap at `(50.03, 33.00)` (verifying R-tree gap healing!) and an inner `20 × 12 mm` boss island,
/// - An `INSERT`ed `VENT_SLOT` block instance at `(26, 50)`!
pub fn sample_router_panel_dxf() -> Vec<u8> {
    use dxf::{
        Block, LwPolylineVertex, Point as DxfPt,
        entities::{Circle, Insert, Line, LwPolyline},
    };

    let mut drawing = Drawing::new();
    drawing.header.version = dxf::enums::AcadVersion::R2000;
    drawing.header.default_drawing_units = Units::Millimeters;
    drawing.header.drawing_units = DrawingUnits::Metric;

    // 1. Define a reusable `VENT_SLOT` block (`12 × 36 mm` stadium centered at `(0, 0)`)
    let mut slot_block = Block {
        name: "VENT_SLOT".to_string(),
        ..Default::default()
    };
    let mut slot_lwp = LwPolyline::default();
    slot_lwp.set_is_closed(true);
    let r_slot = 6.0;
    let h_half = 12.0;
    for (x, y, bulge) in [
        (r_slot, -h_half, 0.0),
        (r_slot, h_half, 1.0), // 180° CCW semicircle (`tan(π/4) = 1.0`)
        (-r_slot, h_half, 0.0),
        (-r_slot, -h_half, 1.0),
    ] {
        slot_lwp.vertices.push(LwPolylineVertex {
            x,
            y,
            bulge,
            ..Default::default()
        });
    }
    let mut slot_ent = Entity::new(EntityType::LwPolyline(slot_lwp));
    slot_ent.common.layer = "CUTOUTS".to_string();
    slot_block.entities.push(slot_ent);
    drawing.add_block(slot_block);

    // 2. Outer `160 × 100 mm` panel outline with `R = 8 mm` rounded corners (`bulge = tan(90°/4) = sqrt(2) - 1`)
    let b90 = (std::f64::consts::FRAC_PI_8).tan();
    let mut outer = LwPolyline::default();
    outer.set_is_closed(true);
    for (x, y, bulge) in [
        (8.0, 0.0, 0.0),
        (152.0, 0.0, b90),
        (160.0, 8.0, 0.0),
        (160.0, 92.0, b90),
        (152.0, 100.0, 0.0),
        (8.0, 100.0, b90),
        (0.0, 92.0, 0.0),
        (0.0, 8.0, b90),
    ] {
        outer.vertices.push(LwPolylineVertex {
            x,
            y,
            bulge,
            ..Default::default()
        });
    }
    let mut outer_ent = Entity::new(EntityType::LwPolyline(outer));
    outer_ent.common.layer = "OUTLINE".to_string();
    drawing.add_entity(outer_ent);

    // 3. 4x Ø6.0 mm (`R = 3.0 mm`) mounting holes on `HOLES`
    for (hx, hy) in [(12.0, 12.0), (148.0, 12.0), (148.0, 88.0), (12.0, 88.0)] {
        let mut c_ent = Entity::new(EntityType::Circle(Circle::new(
            DxfPt::new(hx, hy, 0.0),
            3.0,
        )));
        c_ent.common.layer = "HOLES".to_string();
        drawing.add_entity(c_ent);
    }

    // 4. Central `60 × 34 mm` pocket `[50..110] × [33..67]` made of 4 individual `LINE`s with a
    //    deliberate `0.03 mm` gap at `(50.03, 33.00)` + 1 duplicate overlapping line + inner `20 × 12 mm` island!
    let pocket_lines = [
        ((50.03, 33.0), (110.0, 33.0)), // 0.03 mm gap from (50.0, 33.0)!
        ((110.0, 33.0), (110.0, 67.0)),
        ((110.0, 67.0), (110.0, 33.0)), // Intentional reversed duplicate segment!
        ((110.0, 67.0), (50.0, 67.0)),
        ((50.0, 67.0), (50.0, 33.0)),
    ];
    for ((x0, y0), (x1, y1)) in pocket_lines {
        let mut l_ent = Entity::new(EntityType::Line(Line::new(
            DxfPt::new(x0, y0, 0.0),
            DxfPt::new(x1, y1, 0.0),
        )));
        l_ent.common.layer = "POCKETS".to_string();
        drawing.add_entity(l_ent);
    }

    // Inner `20 × 12 mm` raised island `[70..90] × [44..56]` inside the `[50..110] × [33..67]` pocket!
    let mut isl_lwp = LwPolyline::default();
    isl_lwp.set_is_closed(true);
    for (x, y) in [(70.0, 44.0), (90.0, 44.0), (90.0, 56.0), (70.0, 56.0)] {
        isl_lwp.vertices.push(LwPolylineVertex {
            x,
            y,
            ..Default::default()
        });
    }
    let mut isl_ent = Entity::new(EntityType::LwPolyline(isl_lwp));
    isl_ent.common.layer = "POCKETS".to_string();
    drawing.add_entity(isl_ent);

    // 5. `INSERT` block instance of `VENT_SLOT` at `(28.0, 50.0)`
    let ins = Insert {
        name: "VENT_SLOT".to_string(),
        location: DxfPt::new(28.0, 50.0, 0.0),
        x_scale_factor: 1.0,
        y_scale_factor: 1.0,
        z_scale_factor: 1.0,
        ..Default::default()
    };
    let mut ins_ent = Entity::new(EntityType::Insert(ins));
    ins_ent.common.layer = "CUTOUTS".to_string();
    drawing.add_entity(ins_ent);

    let mut buf = Vec::new();
    let _ = drawing.save(&mut buf);
    buf
}

/// Returns a deterministic W3C SVG badge/emblem fixture (`100mm × 60mm` physical units)
/// featuring an outer plaque, a transformed `<g id="emblem" transform="translate(50, 30)">`
/// concentric circular ring (`R = 18 mm` outer, `R = 10 mm` inner island!), and two side slots.
pub fn sample_badge_logo_svg() -> String {
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="60mm" viewBox="0 0 100 60">
  <g id="plaque">
    <rect x="4" y="4" width="92" height="52" rx="6" ry="6" fill="none" stroke="black"/>
  </g>
  <g id="emblem" transform="translate(50, 30)">
    <path d="M -18 0 A 18 18 0 1 0 18 0 A 18 18 0 1 0 -18 0 Z M -10 0 A 10 10 0 1 1 10 0 A 10 10 0 1 1 -10 0 Z" fill="black"/>
  </g>
</svg>"#
        .to_string()
}
