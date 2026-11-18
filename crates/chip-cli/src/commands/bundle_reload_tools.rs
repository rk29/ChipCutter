use std::{path::PathBuf, process::ExitCode};

use chip_cad::{CadKernel, MonstertruckKernel, TopologyGraph};
use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
use chip_project::{
    OperationReloadOutcome, ProjectDocument, compute_sha256_hex, export_chipz_bundle,
    portable_model_path, rebind_project_to_reloaded_cad, resolve_model_path, unpack_chipz_bundle,
};
use chip_tools::import_fusion_library_path;

pub fn run_reload_step(
    project: PathBuf,
    step: Option<PathBuf>,
    output: Option<PathBuf>,
    allow_needs_repick: bool,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj_doc = ProjectDocument::load_from_path(&project)?;
    let dest_proj = output.unwrap_or_else(|| project.clone());
    let step_path = match step {
        Some(s) => s,
        None => {
            let first = proj_doc
                .models
                .first()
                .ok_or("project has no [[models]] entry")?;
            resolve_model_path(project.parent(), &first.path)
        }
    };
    let step_bytes = std::fs::read(&step_path)?;
    let sha = compute_sha256_hex(&step_bytes);
    let name = step_path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("model.step");
    let new_doc =
        MonstertruckKernel.load_step_bytes(name, &step_bytes, DEFAULT_TESS_CHORD_TOL_MM)?;
    let topo = TopologyGraph::build(&new_doc);
    let new_rel = portable_model_path(Some(&dest_proj), &step_path);

    let report =
        rebind_project_to_reloaded_cad(&mut proj_doc, 0, &new_doc, &topo, &sha, Some(new_rel));
    println!("=== ChipCutter Associative STEP Reload ===");
    println!("{}", report.summary_line());
    for w in &report.setup_wcs_warnings {
        println!("  [WCS WARN] {w}");
    }
    for out in &report.operation_outcomes {
        match out {
            OperationReloadOutcome::ReboundDirty {
                setup_name,
                op_name,
                updated_indices,
                ..
            } => {
                println!(
                    "  [REBOUND ] {setup_name} -> {op_name} ({updated_indices} topology index update(s))"
                );
            }
            OperationReloadOutcome::NeedsRepick {
                setup_name,
                op_name,
                reason,
                ..
            } => {
                println!("  [REPICK! ] {setup_name} -> {op_name}: {reason}");
            }
            OperationReloadOutcome::SuppressedSkipped {
                setup_name,
                op_name,
                ..
            } => {
                println!("  [SKIPPED ] {setup_name} -> {op_name} (Suppressed)");
            }
        }
    }
    proj_doc.save_to_path(&dest_proj)?;
    println!("Saved updated project to {}", dest_proj.display());

    if (report.needs_repick_count > 0 || !report.setup_wcs_warnings.is_empty())
        && !allow_needs_repick
    {
        Ok(ExitCode::FAILURE)
    } else {
        Ok(ExitCode::SUCCESS)
    }
}

pub fn run_bundle_pack(
    project: PathBuf,
    output: PathBuf,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let proj_doc = ProjectDocument::load_from_path(&project)?;
    let sum = export_chipz_bundle(&proj_doc, project.parent(), &[], &[], &output)?;
    println!(
        "Packed portable bundle '{}' ({} bytes, {} model(s), {} custom post(s)).",
        output.display(),
        sum.archive_size_bytes,
        sum.model_files.len(),
        sum.custom_post_files.len()
    );
    Ok(ExitCode::SUCCESS)
}

pub fn run_bundle_unpack(
    archive: PathBuf,
    output_dir: PathBuf,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let unpacked = unpack_chipz_bundle(&archive, &output_dir)?;
    println!(
        "Unpacked '{}' ({} file(s)) -> '{}'.",
        unpacked.project.project_name,
        unpacked.extracted_files.len(),
        unpacked.project_toml_path.display()
    );
    Ok(ExitCode::SUCCESS)
}

pub fn run_import_fusion(
    input: PathBuf,
    output: PathBuf,
    name: Option<String>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let report = import_fusion_library_path(&input, name.as_deref())?;
    report.library.save_to_path(&output)?;
    println!(
        "Imported {} tool(s) and {} cutting preset(s) from '{}' -> '{}'.",
        report.imported_tool_count,
        report.imported_preset_count,
        input.display(),
        output.display()
    );
    for warn in &report.warnings {
        println!("  [WARN] {warn}");
    }
    Ok(ExitCode::SUCCESS)
}
