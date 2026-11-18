//! Built-in public-domain single-line Hershey engraving vector fonts (`SimplexSans`,
//! `DuplexRoman`, `Mono`, `Script`) for 1-pass `2D Curve Trace` / CNC part-number scribing
//! (`11-sketches-vectors.md` §CAM Typography).

use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2};
use serde::{Deserialize, Serialize};

use crate::{
    primitives::transform_arc_polyline_2d,
    typography::{TextHAlign, TextVAlign},
};

/// Built-in single-line engraving font style (`11-sketches-vectors.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HersheyFontKind {
    #[default]
    SimplexSans,
    DuplexRoman,
    Mono,
    Script,
}

impl HersheyFontKind {
    pub const ALL: [Self; 4] = [
        Self::SimplexSans,
        Self::DuplexRoman,
        Self::Mono,
        Self::Script,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::SimplexSans => "Hershey Simplex Sans (1-Line)",
            Self::DuplexRoman => "Hershey Duplex Roman (1-Line)",
            Self::Mono => "Hershey Monospace (1-Line)",
            Self::Script => "Hershey Italic Script (1-Line)",
        }
    }
}

/// Parameters controlling Hershey single-line stroke layout.
#[derive(Debug, Clone, Copy)]
pub struct HersheyLayoutParams<'a> {
    pub content: &'a str,
    pub font: HersheyFontKind,
    pub cap_height_mm: f64,
    pub origin_xy: DVec2,
    pub h_align: TextHAlign,
    pub v_align: TextVAlign,
    pub letter_spacing_mm: f64,
    pub line_spacing_factor: f64,
    pub rotation_deg: f64,
    pub mirror_x: bool,
}

/// Evaluates `params` into open/closed single-pass centerline [`ArcPolyline2D`] strokes.
///
/// Design grid convention: capital letters span `Y ∈ [0.0, 21.0]` (`CAP_UNITS = 21.0`),
/// lowercase x-height is `14.0`, and descenders reach `-6.0`.
pub fn evaluate_hershey_strokes(params: &HersheyLayoutParams<'_>) -> Vec<ArcPolyline2D> {
    const CAP_UNITS: f64 = 21.0;
    let cap_mm = params.cap_height_mm.max(0.2);
    if !cap_mm.is_finite() || params.content.is_empty() {
        return Vec::new();
    }
    let unit_scale = cap_mm / CAP_UNITS;
    let line_step_mm = cap_mm * params.line_spacing_factor.clamp(0.8, 4.0);
    let lines: Vec<&str> = params.content.split('\n').collect();
    let n_lines = lines.len();

    // Pre-measure each line's width in mm
    let mut line_widths_mm = Vec::with_capacity(n_lines);
    for line in &lines {
        let mut w_mm = 0.0;
        let chars: Vec<char> = line.chars().collect();
        for (ci, &ch) in chars.iter().enumerate() {
            let g = glyph_strokes(ch, params.font);
            w_mm += g.advance_units * unit_scale;
            if ci + 1 < chars.len() {
                w_mm += params.letter_spacing_mm;
            }
        }
        line_widths_mm.push(w_mm.max(0.0));
    }

    let total_block_h = cap_mm + ((n_lines.saturating_sub(1)) as f64) * line_step_mm;
    let first_baseline_y = match params.v_align {
        TextVAlign::Baseline => 0.0,
        TextVAlign::Top => -cap_mm,
        TextVAlign::Middle => 0.5 * total_block_h - cap_mm,
    };

    let slant = match params.font {
        HersheyFontKind::Script => 0.22,
        _ => 0.0,
    };

    let mut local_strokes: Vec<ArcPolyline2D> = Vec::new();

    for (li, line) in lines.iter().enumerate() {
        let lw = line_widths_mm[li];
        let mut pen_x = match params.h_align {
            TextHAlign::Left => 0.0,
            TextHAlign::Center => -0.5 * lw,
            TextHAlign::Right => -lw,
        };
        let baseline_y = first_baseline_y - (li as f64) * line_step_mm;

        for ch in line.chars() {
            let g = glyph_strokes(ch, params.font);
            for chain in &g.chains {
                if chain.len() < 2 {
                    continue;
                }
                let mut segs = Vec::with_capacity(chain.len() - 1);
                for w in chain.windows(2) {
                    let p0 =
                        map_glyph_pt(w[0], pen_x, baseline_y, unit_scale, slant, params.mirror_x);
                    let p1 =
                        map_glyph_pt(w[1], pen_x, baseline_y, unit_scale, slant, params.mirror_x);
                    if p0.distance(p1) > 1e-5 {
                        segs.push(ArcSegment2D::Line { start: p0, end: p1 });
                    }
                }
                if !segs.is_empty() {
                    let closed = segs.len() >= 3
                        && segs
                            .first()
                            .zip(segs.last())
                            .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-4);
                    local_strokes.push(ArcPolyline2D {
                        segments: segs,
                        closed,
                    });
                }
            }
            pen_x += g.advance_units * unit_scale + params.letter_spacing_mm;
        }
    }

    local_strokes
        .into_iter()
        .map(|p| transform_arc_polyline_2d(&p, params.origin_xy, 1.0, params.rotation_deg))
        .collect()
}

fn map_glyph_pt(
    raw_u: [f64; 2],
    pen_x_mm: f64,
    baseline_y_mm: f64,
    unit_scale: f64,
    slant: f64,
    mirror_x: bool,
) -> DVec2 {
    let x_mm = pen_x_mm + (raw_u[0] + slant * raw_u[1]) * unit_scale;
    let y_mm = baseline_y_mm + raw_u[1] * unit_scale;
    DVec2::new(if mirror_x { -x_mm } else { x_mm }, y_mm)
}

struct HersheyGlyph {
    advance_units: f64,
    chains: Vec<Vec<[f64; 2]>>,
}

/// Public-domain Hershey single-stroke glyph table (`21`-unit cap height).
fn glyph_strokes(ch: char, font: HersheyFontKind) -> HersheyGlyph {
    let fixed_adv = matches!(font, HersheyFontKind::Mono).then_some(16.0);
    let add_serifs = matches!(font, HersheyFontKind::DuplexRoman);

    let (base_adv, mut chains): (f64, Vec<Vec<[f64; 2]>>) = match ch {
        ' ' => (10.0, vec![]),
        '-' => (13.0, vec![vec![[2.0, 9.5], [11.0, 9.5]]]),
        '_' => (14.0, vec![vec![[1.0, -2.0], [13.0, -2.0]]]),
        '.' => (
            8.0,
            vec![vec![
                [3.5, 0.0],
                [4.5, 1.0],
                [3.5, 2.0],
                [2.5, 1.0],
                [3.5, 0.0],
            ]],
        ),
        ',' => (8.0, vec![vec![[4.0, 2.0], [3.5, 0.0], [2.0, -3.0]]]),
        ':' => (
            8.0,
            vec![
                vec![
                    [3.5, 12.0],
                    [4.5, 13.0],
                    [3.5, 14.0],
                    [2.5, 13.0],
                    [3.5, 12.0],
                ],
                vec![[3.5, 1.0], [4.5, 2.0], [3.5, 3.0], [2.5, 2.0], [3.5, 1.0]],
            ],
        ),
        '/' => (13.0, vec![vec![[1.5, 0.0], [11.5, 21.0]]]),
        '+' => (
            15.0,
            vec![
                vec![[7.5, 4.0], [7.5, 16.0]],
                vec![[2.0, 10.0], [13.0, 10.0]],
            ],
        ),
        '=' => (
            15.0,
            vec![
                vec![[2.0, 7.0], [13.0, 7.0]],
                vec![[2.0, 13.0], [13.0, 13.0]],
            ],
        ),
        '#' => (
            16.0,
            vec![
                vec![[5.0, 0.0], [7.0, 21.0]],
                vec![[10.0, 0.0], [12.0, 21.0]],
                vec![[2.0, 7.0], [14.0, 7.0]],
                vec![[2.0, 14.0], [14.0, 14.0]],
            ],
        ),
        '(' => (
            9.0,
            vec![vec![
                [6.5, -3.0],
                [3.5, 3.0],
                [2.5, 10.0],
                [3.5, 17.0],
                [6.5, 23.0],
            ]],
        ),
        ')' => (
            9.0,
            vec![vec![
                [2.5, -3.0],
                [5.5, 3.0],
                [6.5, 10.0],
                [5.5, 17.0],
                [2.5, 23.0],
            ]],
        ),
        '!' => (
            8.0,
            vec![
                vec![[4.0, 21.0], [4.0, 6.0]],
                vec![[4.0, 0.0], [4.5, 1.0], [4.0, 2.0], [3.5, 1.0], [4.0, 0.0]],
            ],
        ),
        '°' => (9.0, vec![oval_pts(4.5, 17.5, 2.6, 2.6)]),
        'Ø' | 'ø' => (
            17.0,
            vec![
                oval_pts(8.5, 10.5, 6.2, 8.5),
                vec![[2.0, 1.0], [15.0, 20.0]],
            ],
        ),

        // Digits 0..9
        '0' => {
            let mut v = vec![oval_pts(8.0, 10.5, 5.8, 10.5)];
            if matches!(font, HersheyFontKind::Mono) {
                v.push(vec![[4.2, 4.0], [11.8, 17.0]]);
            }
            (16.0, v)
        }
        '1' => (
            14.0,
            vec![
                vec![[4.0, 17.0], [8.0, 21.0], [8.0, 0.0]],
                vec![[4.0, 0.0], [12.0, 0.0]],
            ],
        ),
        '2' => (
            15.0,
            vec![vec![
                [2.5, 16.0],
                [4.0, 19.5],
                [7.5, 21.0],
                [11.0, 19.5],
                [12.5, 16.5],
                [11.5, 13.0],
                [2.0, 0.0],
                [13.0, 0.0],
            ]],
        ),
        '3' => (
            15.0,
            vec![vec![
                [3.0, 21.0],
                [12.5, 21.0],
                [7.5, 13.0],
                [10.5, 13.0],
                [13.0, 10.5],
                [13.0, 5.0],
                [11.0, 1.5],
                [7.5, 0.0],
                [3.5, 1.5],
                [2.0, 4.0],
            ]],
        ),
        '4' => (
            16.0,
            vec![vec![[10.5, 0.0], [10.5, 21.0], [1.8, 7.0], [14.0, 7.0]]],
        ),
        '5' => (
            15.0,
            vec![vec![
                [12.0, 21.0],
                [3.5, 21.0],
                [2.8, 12.0],
                [7.5, 13.5],
                [11.5, 12.0],
                [13.0, 8.0],
                [12.0, 3.0],
                [8.5, 0.0],
                [4.0, 0.5],
                [2.0, 3.5],
            ]],
        ),
        '6' => (
            15.0,
            vec![vec![
                [11.5, 18.5],
                [9.0, 21.0],
                [5.5, 20.5],
                [3.0, 16.5],
                [2.2, 9.0],
                [3.5, 2.5],
                [7.5, 0.0],
                [11.5, 2.0],
                [13.0, 6.5],
                [11.5, 11.0],
                [7.5, 12.5],
                [3.5, 10.5],
                [2.2, 8.0],
            ]],
        ),
        '7' => (15.0, vec![vec![[2.0, 21.0], [13.0, 21.0], [5.5, 0.0]]]),
        '8' => (
            16.0,
            vec![oval_pts(8.0, 16.2, 4.8, 4.8), oval_pts(8.0, 5.7, 5.7, 5.7)],
        ),
        '9' => (
            15.0,
            vec![vec![
                [12.5, 13.5],
                [11.0, 10.0],
                [7.5, 8.5],
                [3.5, 10.0],
                [2.0, 14.5],
                [3.5, 19.0],
                [7.5, 21.0],
                [11.5, 19.0],
                [12.8, 13.5],
                [12.0, 5.5],
                [9.5, 1.0],
                [6.0, 0.0],
                [3.0, 1.5],
            ]],
        ),

        // Uppercase A..Z
        'A' => (
            16.0,
            vec![
                vec![[1.5, 0.0], [8.0, 21.0], [14.5, 0.0]],
                vec![[4.0, 7.5], [12.0, 7.5]],
            ],
        ),
        'B' => (
            16.0,
            vec![
                vec![
                    [2.5, 0.0],
                    [2.5, 21.0],
                    [10.0, 21.0],
                    [13.0, 19.0],
                    [13.5, 15.5],
                    [12.5, 12.0],
                    [9.5, 10.8],
                    [2.5, 10.8],
                ],
                vec![
                    [9.5, 10.8],
                    [13.0, 9.5],
                    [14.0, 6.0],
                    [13.0, 2.0],
                    [10.0, 0.0],
                    [2.5, 0.0],
                ],
            ],
        ),
        'C' => (
            16.0,
            vec![vec![
                [13.8, 16.5],
                [11.5, 20.0],
                [8.0, 21.0],
                [4.5, 19.5],
                [2.2, 15.0],
                [2.2, 6.0],
                [4.5, 1.5],
                [8.0, 0.0],
                [11.5, 1.0],
                [13.8, 4.5],
            ]],
        ),
        'D' => (
            16.0,
            vec![vec![
                [2.5, 0.0],
                [2.5, 21.0],
                [9.0, 21.0],
                [12.5, 18.5],
                [14.0, 14.0],
                [14.0, 7.0],
                [12.5, 2.5],
                [9.0, 0.0],
                [2.5, 0.0],
            ]],
        ),
        'E' => (
            15.0,
            vec![
                vec![[12.5, 21.0], [2.5, 21.0], [2.5, 0.0], [12.5, 0.0]],
                vec![[2.5, 10.8], [10.0, 10.8]],
            ],
        ),
        'F' => (
            14.5,
            vec![
                vec![[12.5, 21.0], [2.5, 21.0], [2.5, 0.0]],
                vec![[2.5, 10.8], [10.0, 10.8]],
            ],
        ),
        'G' => (
            16.5,
            vec![vec![
                [14.0, 16.5],
                [11.8, 20.0],
                [8.2, 21.0],
                [4.5, 19.5],
                [2.2, 15.0],
                [2.2, 6.0],
                [4.5, 1.5],
                [8.5, 0.0],
                [12.5, 1.5],
                [14.0, 5.5],
                [14.0, 9.5],
                [9.0, 9.5],
            ]],
        ),
        'H' => (
            16.5,
            vec![
                vec![[2.5, 21.0], [2.5, 0.0]],
                vec![[14.0, 21.0], [14.0, 0.0]],
                vec![[2.5, 10.8], [14.0, 10.8]],
            ],
        ),
        'I' => (
            9.0,
            vec![
                vec![[4.5, 21.0], [4.5, 0.0]],
                vec![[2.0, 21.0], [7.0, 21.0]],
                vec![[2.0, 0.0], [7.0, 0.0]],
            ],
        ),
        'J' => (
            13.5,
            vec![vec![
                [10.5, 21.0],
                [10.5, 5.0],
                [9.0, 1.2],
                [6.0, 0.0],
                [3.0, 1.2],
                [1.8, 4.5],
            ]],
        ),
        'K' => (
            16.0,
            vec![
                vec![[2.5, 21.0], [2.5, 0.0]],
                vec![[13.5, 21.0], [2.5, 9.5]],
                vec![[6.2, 13.2], [13.8, 0.0]],
            ],
        ),
        'L' => (14.0, vec![vec![[2.5, 21.0], [2.5, 0.0], [12.5, 0.0]]]),
        'M' => (
            19.0,
            vec![vec![
                [2.2, 0.0],
                [2.2, 21.0],
                [9.5, 3.0],
                [16.8, 21.0],
                [16.8, 0.0],
            ]],
        ),
        'N' => (
            16.5,
            vec![vec![[2.5, 0.0], [2.5, 21.0], [14.0, 0.0], [14.0, 21.0]]],
        ),
        'O' => (17.0, vec![oval_pts(8.5, 10.5, 6.2, 10.5)]),
        'P' => (
            15.5,
            vec![vec![
                [2.5, 0.0],
                [2.5, 21.0],
                [10.0, 21.0],
                [13.0, 19.0],
                [13.8, 15.5],
                [13.0, 12.0],
                [10.0, 10.2],
                [2.5, 10.2],
            ]],
        ),
        'Q' => (
            17.0,
            vec![
                oval_pts(8.5, 10.5, 6.2, 10.5),
                vec![[9.5, 5.0], [15.0, -1.5]],
            ],
        ),
        'R' => (
            16.0,
            vec![
                vec![
                    [2.5, 0.0],
                    [2.5, 21.0],
                    [10.0, 21.0],
                    [13.0, 19.0],
                    [13.8, 15.5],
                    [13.0, 12.0],
                    [10.0, 10.5],
                    [2.5, 10.5],
                ],
                vec![[8.8, 10.5], [14.0, 0.0]],
            ],
        ),
        'S' => (
            15.5,
            vec![vec![
                [13.2, 18.0],
                [10.8, 20.8],
                [6.5, 21.0],
                [3.0, 19.0],
                [2.2, 15.8],
                [3.8, 13.0],
                [11.8, 9.5],
                [13.8, 6.5],
                [13.2, 2.5],
                [9.8, 0.0],
                [5.2, 0.2],
                [2.0, 3.0],
            ]],
        ),
        'T' => (
            15.0,
            vec![
                vec![[1.5, 21.0], [13.5, 21.0]],
                vec![[7.5, 21.0], [7.5, 0.0]],
            ],
        ),
        'U' => (
            16.5,
            vec![vec![
                [2.5, 21.0],
                [2.5, 5.5],
                [4.2, 1.5],
                [8.2, 0.0],
                [12.2, 1.5],
                [14.0, 5.5],
                [14.0, 21.0],
            ]],
        ),
        'V' => (16.0, vec![vec![[1.5, 21.0], [8.0, 0.0], [14.5, 21.0]]]),
        'W' => (
            20.0,
            vec![vec![
                [1.5, 21.0],
                [5.5, 0.0],
                [10.0, 16.0],
                [14.5, 0.0],
                [18.5, 21.0],
            ]],
        ),
        'X' => (
            16.0,
            vec![
                vec![[2.0, 21.0], [14.0, 0.0]],
                vec![[14.0, 21.0], [2.0, 0.0]],
            ],
        ),
        'Y' => (
            15.5,
            vec![
                vec![[1.5, 21.0], [7.8, 10.5], [14.0, 21.0]],
                vec![[7.8, 10.5], [7.8, 0.0]],
            ],
        ),
        'Z' => (
            15.5,
            vec![vec![[2.0, 21.0], [13.5, 21.0], [2.0, 0.0], [13.5, 0.0]]],
        ),

        // Lowercase a..z (clean technical x-height = 14.0 strokes)
        c if c.is_ascii_lowercase() => lowercase_strokes(c),
        other => {
            // Fallback to uppercase ASCII if available, otherwise small placeholder box
            let up = other.to_ascii_uppercase();
            if up != other && up.is_ascii_uppercase() {
                return glyph_strokes(up, font);
            }
            (
                14.0,
                vec![vec![
                    [2.0, 0.0],
                    [12.0, 0.0],
                    [12.0, 18.0],
                    [2.0, 18.0],
                    [2.0, 0.0],
                ]],
            )
        }
    };

    if add_serifs && ch.is_ascii_uppercase() && matches!(ch, 'H' | 'K' | 'L' | 'M' | 'N') {
        chains.push(vec![[1.0, 0.0], [4.0, 0.0]]);
        chains.push(vec![[1.0, 21.0], [4.0, 21.0]]);
    }

    if let Some(target_adv) = fixed_adv {
        let shift_x = 0.5 * (target_adv - base_adv);
        if shift_x.abs() > 1e-4 {
            for chain in &mut chains {
                for pt in chain {
                    pt[0] += shift_x;
                }
            }
        }
        HersheyGlyph {
            advance_units: target_adv,
            chains,
        }
    } else {
        HersheyGlyph {
            advance_units: base_adv,
            chains,
        }
    }
}

fn lowercase_strokes(ch: char) -> (f64, Vec<Vec<[f64; 2]>>) {
    match ch {
        'a' => (
            14.0,
            vec![
                vec![[11.5, 14.0], [11.5, 0.0]],
                oval_pts(6.8, 7.0, 4.7, 7.0),
            ],
        ),
        'b' => (
            14.0,
            vec![vec![[2.2, 21.0], [2.2, 0.0]], oval_pts(7.0, 7.0, 4.8, 7.0)],
        ),
        'c' => (
            13.0,
            vec![vec![
                [11.0, 11.5],
                [8.5, 14.0],
                [5.0, 13.5],
                [2.2, 10.0],
                [2.2, 4.0],
                [5.0, 0.5],
                [8.5, 0.0],
                [11.0, 2.5],
            ]],
        ),
        'd' => (
            14.0,
            vec![
                vec![[11.8, 21.0], [11.8, 0.0]],
                oval_pts(7.0, 7.0, 4.8, 7.0),
            ],
        ),
        'e' => (
            13.5,
            vec![vec![
                [2.0, 7.5],
                [11.5, 7.5],
                [11.5, 10.5],
                [9.5, 13.5],
                [6.5, 14.0],
                [3.5, 12.5],
                [2.0, 9.0],
                [2.5, 3.5],
                [5.5, 0.0],
                [9.5, 0.5],
                [11.2, 2.5],
            ]],
        ),
        'f' => (
            10.5,
            vec![
                vec![[8.5, 20.5], [6.5, 21.0], [4.5, 19.5], [4.5, 0.0]],
                vec![[2.0, 14.0], [8.0, 14.0]],
            ],
        ),
        'g' => (
            14.0,
            vec![
                oval_pts(7.0, 7.0, 4.8, 7.0),
                vec![
                    [11.8, 14.0],
                    [11.8, -2.5],
                    [9.8, -5.5],
                    [6.5, -6.0],
                    [3.5, -5.0],
                ],
            ],
        ),
        'h' => (
            14.0,
            vec![
                vec![[2.5, 21.0], [2.5, 0.0]],
                vec![
                    [2.5, 10.5],
                    [5.5, 13.8],
                    [9.2, 14.0],
                    [11.5, 11.2],
                    [11.5, 0.0],
                ],
            ],
        ),
        'i' => (
            7.5,
            vec![
                vec![[3.8, 14.0], [3.8, 0.0]],
                vec![
                    [3.8, 19.0],
                    [4.3, 20.0],
                    [3.8, 21.0],
                    [3.3, 20.0],
                    [3.8, 19.0],
                ],
            ],
        ),
        'j' => (
            8.5,
            vec![
                vec![[5.5, 14.0], [5.5, -3.0], [3.8, -6.0], [1.5, -5.5]],
                vec![
                    [5.5, 19.0],
                    [6.0, 20.0],
                    [5.5, 21.0],
                    [5.0, 20.0],
                    [5.5, 19.0],
                ],
            ],
        ),
        'k' => (
            13.5,
            vec![
                vec![[2.5, 21.0], [2.5, 0.0]],
                vec![[11.2, 14.0], [2.5, 5.5]],
                vec![[5.8, 8.8], [11.8, 0.0]],
            ],
        ),
        'l' => (7.5, vec![vec![[3.8, 21.0], [3.8, 0.0]]]),
        'm' => (
            20.0,
            vec![
                vec![[2.2, 14.0], [2.2, 0.0]],
                vec![
                    [2.2, 10.5],
                    [5.0, 13.8],
                    [8.2, 14.0],
                    [10.0, 11.2],
                    [10.0, 0.0],
                ],
                vec![
                    [10.0, 10.5],
                    [12.8, 13.8],
                    [16.0, 14.0],
                    [17.8, 11.2],
                    [17.8, 0.0],
                ],
            ],
        ),
        'n' => (
            14.0,
            vec![
                vec![[2.5, 14.0], [2.5, 0.0]],
                vec![
                    [2.5, 10.5],
                    [5.5, 13.8],
                    [9.2, 14.0],
                    [11.5, 11.2],
                    [11.5, 0.0],
                ],
            ],
        ),
        'o' => (14.0, vec![oval_pts(7.0, 7.0, 4.8, 7.0)]),
        'p' => (
            14.0,
            vec![vec![[2.2, 14.0], [2.2, -6.0]], oval_pts(7.0, 7.0, 4.8, 7.0)],
        ),
        'q' => (
            14.0,
            vec![
                vec![[11.8, 14.0], [11.8, -6.0]],
                oval_pts(7.0, 7.0, 4.8, 7.0),
            ],
        ),
        'r' => (
            10.5,
            vec![
                vec![[2.5, 14.0], [2.5, 0.0]],
                vec![[2.5, 9.5], [5.0, 13.2], [7.8, 14.0], [9.5, 13.2]],
            ],
        ),
        's' => (
            13.0,
            vec![vec![
                [10.8, 11.8],
                [8.5, 14.0],
                [4.5, 13.8],
                [2.5, 11.5],
                [3.5, 9.0],
                [9.8, 6.5],
                [11.2, 4.0],
                [9.5, 0.5],
                [5.5, 0.0],
                [2.2, 2.0],
            ]],
        ),
        't' => (
            10.5,
            vec![
                vec![[4.8, 19.5], [4.8, 2.5], [6.5, 0.0], [9.0, 0.5]],
                vec![[2.0, 14.0], [8.5, 14.0]],
            ],
        ),
        'u' => (
            14.0,
            vec![
                vec![[2.5, 14.0], [2.5, 3.5], [4.8, 0.2], [8.5, 0.2], [11.5, 3.5]],
                vec![[11.5, 14.0], [11.5, 0.0]],
            ],
        ),
        'v' => (13.0, vec![vec![[1.8, 14.0], [6.5, 0.0], [11.2, 14.0]]]),
        'w' => (
            18.0,
            vec![vec![
                [1.8, 14.0],
                [5.0, 0.0],
                [9.0, 11.5],
                [13.0, 0.0],
                [16.2, 14.0],
            ]],
        ),
        'x' => (
            13.0,
            vec![
                vec![[2.0, 14.0], [11.0, 0.0]],
                vec![[11.0, 14.0], [2.0, 0.0]],
            ],
        ),
        'y' => (
            13.0,
            vec![
                vec![[1.8, 14.0], [6.5, 0.0]],
                vec![[11.2, 14.0], [6.5, 0.0], [4.2, -5.2], [1.8, -6.0]],
            ],
        ),
        'z' => (
            13.0,
            vec![vec![[2.0, 14.0], [11.0, 14.0], [2.0, 0.0], [11.0, 0.0]]],
        ),
        _ => (12.0, vec![]),
    }
}

fn oval_pts(cx: f64, cy: f64, rx: f64, ry: f64) -> Vec<[f64; 2]> {
    const N: usize = 16;
    let mut pts = Vec::with_capacity(N + 1);
    for i in 0..=N {
        let a = (i as f64) * (std::f64::consts::TAU / (N as f64));
        pts.push([cx + rx * a.cos(), cy + ry * a.sin()]);
    }
    pts
}
