use chip_cam::{ArcPlane, CutterCompMode};
use chip_math::DVec3;

use crate::profile::{CommentStyle, PostError, PostProfile};

#[derive(Debug, Clone)]
pub(crate) struct ModalState {
    pub motion_code: Option<&'static str>,
    pub plane: ArcPlane,
    pub x_word: Option<String>,
    pub y_word: Option<String>,
    pub z_word: Option<String>,
    pub f_word: Option<String>,
    pub canned_active: bool,
    pub cutter_comp: CutterCompMode,
    pub coolant_on: bool,
}

impl Default for ModalState {
    fn default() -> Self {
        Self {
            motion_code: None,
            plane: ArcPlane::XY,
            x_word: None,
            y_word: None,
            z_word: None,
            f_word: None,
            canned_active: false,
            cutter_comp: CutterCompMode::Off,
            coolant_on: false,
        }
    }
}

impl ModalState {
    pub(crate) fn force_full_next_move(&mut self) {
        self.motion_code = None;
        self.x_word = None;
        self.y_word = None;
        self.z_word = None;
        self.f_word = None;
    }
}

pub(crate) struct LineWriter<'a> {
    profile: &'a PostProfile,
    lines: Vec<String>,
    next_line_num: Option<u32>,
    max_observed_len: usize,
}

impl<'a> LineWriter<'a> {
    pub(crate) fn new(profile: &'a PostProfile) -> Self {
        Self {
            profile,
            lines: Vec::with_capacity(512),
            next_line_num: profile.line_numbers.map(|c| c.start),
            max_observed_len: 0,
        }
    }

    pub(crate) fn push_code_line(&mut self, raw_line: &str) -> Result<(), PostError> {
        let trimmed = raw_line.trim();
        if trimmed.is_empty() {
            return Ok(());
        }
        // Tape markers `%` never receive `N##` block numbers
        let formatted = if trimmed == "%" {
            "%".to_string()
        } else if trimmed.starts_with('(') || trimmed.starts_with(';') {
            return self.push_comment_text(
                trimmed
                    .trim_start_matches(['(', ';'])
                    .trim_end_matches(')')
                    .trim(),
            );
        } else if let Some(n) = self.next_line_num {
            let inc = self.profile.line_numbers.map_or(10, |c| c.increment.max(1));
            self.next_line_num = Some(n.saturating_add(inc));
            format!("N{n} {trimmed}")
        } else {
            trimmed.to_string()
        };

        let len = formatted.len();
        if let Some(max_len) = self.profile.max_line_length
            && len > max_len
        {
            return Err(PostError::LineTooLong {
                len,
                max: max_len,
                line: formatted,
            });
        }
        self.max_observed_len = self.max_observed_len.max(len);
        self.lines.push(formatted);
        Ok(())
    }

    pub(crate) fn push_comment_text(&mut self, raw_text: &str) -> Result<(), PostError> {
        let clean = sanitize_comment_ascii(raw_text);
        let max_total = self.profile.max_line_length.unwrap_or(240);
        let wrapper_overhead = match self.profile.comment_style {
            CommentStyle::Parentheses => 4, // "( " + " )"
            CommentStyle::Semicolon => 2,   // "; "
        };
        let max_body = max_total.saturating_sub(wrapper_overhead).max(8);
        let body = if clean.len() > max_body {
            clean[..max_body].trim_end().to_string()
        } else {
            clean
        };

        let line = match self.profile.comment_style {
            CommentStyle::Parentheses => format!("( {body} )"),
            CommentStyle::Semicolon => format!("; {body}"),
        };
        self.max_observed_len = self.max_observed_len.max(line.len());
        self.lines.push(line);
        Ok(())
    }

    pub(crate) fn finish(self) -> (String, usize, usize) {
        let count = self.lines.len();
        let max_chars = self.max_observed_len;
        let mut joined = self.lines.join("\n");
        joined.push('\n');
        (joined, count, max_chars)
    }
}

pub(crate) fn emit_linear_or_rapid(
    writer: &mut LineWriter<'_>,
    profile: &PostProfile,
    modal: &mut ModalState,
    end_wcs_mm: DVec3,
    feed_mm_min: Option<f64>,
) -> Result<(), PostError> {
    let g_target = if feed_mm_min.is_some() { "G1" } else { "G0" };
    let c_dec = profile.coord_decimals;
    let strip = profile.suppress_trailing_zeros;

    let xs = format_decimal(profile.units.scale_from_mm(end_wcs_mm.x), c_dec, strip);
    let ys = format_decimal(profile.units.scale_from_mm(end_wcs_mm.y), c_dec, strip);
    let zs = format_decimal(profile.units.scale_from_mm(end_wcs_mm.z), c_dec, strip);

    let x_changed = modal.x_word.as_deref() != Some(xs.as_str());
    let y_changed = modal.y_word.as_deref() != Some(ys.as_str());
    let z_changed = modal.z_word.as_deref() != Some(zs.as_str());

    if !x_changed && !y_changed && !z_changed {
        return Ok(());
    }

    let mut words: Vec<String> = Vec::with_capacity(6);
    if modal.motion_code != Some(g_target) {
        words.push(g_target.to_string());
        modal.motion_code = Some(g_target);
    }
    if x_changed {
        words.push(format!("X{xs}"));
        modal.x_word = Some(xs);
    }
    if y_changed {
        words.push(format!("Y{ys}"));
        modal.y_word = Some(ys);
    }
    if z_changed {
        words.push(format!("Z{zs}"));
        modal.z_word = Some(zs);
    }
    if let Some(f_mm) = feed_mm_min {
        let f_scaled = profile.units.scale_from_mm(f_mm).max(0.1);
        let fs = format_decimal(f_scaled, profile.feed_decimals, strip);
        if modal.f_word.as_deref() != Some(fs.as_str()) {
            words.push(format!("F{fs}"));
            modal.f_word = Some(fs);
        }
    }

    writer.push_code_line(&words.join(" "))
}

pub(crate) fn format_decimal(val: f64, decimals: usize, suppress_trailing_zeros: bool) -> String {
    let clean_val = if val.abs() < 0.5 * 10_f64.powi(-(decimals as i32)) {
        0.0
    } else {
        val
    };
    if decimals == 0 {
        return format!("{:.0}", clean_val.round());
    }
    let mut s = format!("{clean_val:.decimals$}");
    if suppress_trailing_zeros && s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" || s.is_empty() {
        "0".to_string()
    } else {
        s
    }
}

fn sanitize_comment_ascii(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        match ch {
            '(' => out.push('['),
            ')' => out.push(']'),
            ';' => out.push(':'),
            '\r' | '\n' | '\t' => out.push(' '),
            'Ø' | 'ø' => out.push('D'),
            '°' => out.push_str("deg"),
            '—' | '–' => out.push('-'),
            '×' => out.push('x'),
            c if c.is_ascii_graphic() || c == ' ' => out.push(c),
            _ => {}
        }
    }
    out.trim().to_string()
}

pub(crate) fn collapse_spaces(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for word in s.split_whitespace() {
        if !out.is_empty() {
            out.push(' ');
        }
        out.push_str(word);
    }
    out
}
