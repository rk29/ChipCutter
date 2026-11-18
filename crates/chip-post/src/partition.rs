use chip_cam::{
    GeneratedToolpath, Operation, ResolvedStockGeometry, WcsConfig, WcsOriginMode, WorkOffsetSlot,
};
use chip_tools::Tool;
use serde::{Deserialize, Serialize};

use crate::profile::{PostProfile, ToolChangeMode};

/// Single enabled operation + tool + clean toolpath passed to `chip-post`.
#[derive(Debug, Clone, Copy)]
pub struct OperationPostInput<'a> {
    pub op: &'a Operation,
    pub tool: &'a Tool,
    pub toolpath: &'a GeneratedToolpath,
}

/// Complete Setup payload passed to `post_process_setup` (`07-post-processor.md`).
#[derive(Debug, Clone)]
pub struct SetupPostInput<'a> {
    pub project_name: &'a str,
    pub setup_name: &'a str,
    pub wcs_slot: WorkOffsetSlot,
    pub wcs_origin_description: String,
    pub stock: ResolvedStockGeometry,
    pub clearance_z_wcs: f64,
    pub retract_z_wcs: f64,
    pub operations: Vec<OperationPostInput<'a>>,
}

/// Emitted G-code file ready to preview in UI or write to disk.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PostOutputFile {
    pub filename: String,
    pub content: String,
    pub line_count: usize,
    pub max_line_chars: usize,
    pub tool_numbers: Vec<u32>,
    pub operation_names: Vec<String>,
    pub est_time_sec: f64,
    pub warnings: Vec<String>,
}

/// Partitioned slice of operations that emit into a single output file.
pub(crate) struct PostPartition<'a> {
    pub filename: String,
    pub ops: Vec<OperationPostInput<'a>>,
    pub suppress_m6: bool,
}

/// Formats a concise human-readable WCS Zero summary for the operator header (`07-post-processor.md`).
pub fn describe_wcs_origin(wcs: &WcsConfig) -> String {
    let base = match &wcs.origin {
        WcsOriginMode::StockBoxPoint { anchor } => format!("Stock {}", anchor.label()),
        WcsOriginMode::ModelBoxPoint { anchor } => format!("Model {}", anchor.label()),
        WcsOriginMode::SelectedCadFeature { .. } => "Selected CAD Feature".to_string(),
    };
    let dx = wcs.additional_offset_wcs[0].value_mm;
    let dy = wcs.additional_offset_wcs[1].value_mm;
    let dz = wcs.additional_offset_wcs[2].value_mm;
    if dx.abs() > 1e-4 || dy.abs() > 1e-4 || dz.abs() > 1e-4 {
        format!("{base} (+Offset {dx:+.2}, {dy:+.2}, {dz:+.2} mm)")
    } else {
        base
    }
}

pub(crate) fn sanitize_filename_slug(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    let mut prev_underscore = false;
    for ch in raw.chars() {
        if ch.is_ascii_alphanumeric() {
            out.push(ch.to_ascii_lowercase());
            prev_underscore = false;
        } else if !prev_underscore && !out.is_empty() {
            out.push('_');
            prev_underscore = true;
        }
    }
    while out.ends_with('_') {
        out.pop();
    }
    if out.is_empty() {
        "job".to_string()
    } else if out.len() > 24 {
        let mut s = out[..24].to_string();
        while s.ends_with('_') {
            s.pop();
        }
        s
    } else {
        out
    }
}

pub(crate) fn partition_operations<'a>(
    ops: &[OperationPostInput<'a>],
    profile: &PostProfile,
    stem: &str,
) -> Vec<PostPartition<'a>> {
    let clean_stem = sanitize_filename_slug(stem);
    let ext = profile.extension.trim_start_matches('.');

    // Count distinct tool numbers across all operations
    let mut unique_tools: Vec<u32> = Vec::new();
    for item in ops {
        if !unique_tools.contains(&item.tool.tool_number) {
            unique_tools.push(item.tool.tool_number);
        }
    }

    match profile.tool_change_mode {
        ToolChangeMode::M6 => vec![PostPartition {
            filename: format!("{clean_stem}.{ext}"),
            ops: ops.to_vec(),
            suppress_m6: false,
        }],
        ToolChangeMode::IgnoreIfSingleTool => vec![PostPartition {
            filename: format!("{clean_stem}.{ext}"),
            ops: ops.to_vec(),
            suppress_m6: unique_tools.len() <= 1,
        }],
        ToolChangeMode::SplitFilesByTool => {
            // Group consecutive operations sharing the same `tool_number`
            let mut groups: Vec<Vec<OperationPostInput<'a>>> = Vec::new();
            for &item in ops {
                if let Some(last_grp) = groups.last_mut()
                    && last_grp
                        .last()
                        .is_some_and(|prev| prev.tool.tool_number == item.tool.tool_number)
                {
                    last_grp.push(item);
                } else {
                    groups.push(vec![item]);
                }
            }

            groups
                .into_iter()
                .enumerate()
                .map(|(idx, grp)| {
                    let tool = grp[0].tool;
                    let t_slug = sanitize_filename_slug(&tool.name);
                    let filename = format!(
                        "{clean_stem}_{:02}_T{}_{}.{ext}",
                        idx + 1,
                        tool.tool_number,
                        t_slug
                    );
                    PostPartition {
                        filename,
                        ops: grp,
                        suppress_m6: true,
                    }
                })
                .collect()
        }
    }
}
