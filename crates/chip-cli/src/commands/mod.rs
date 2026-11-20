pub mod bundle_reload_tools;
pub mod inspect_sample;
pub mod machines;
pub mod validate_sim_build;

use std::path::{Path, PathBuf};

use chip_cad::CadDocument;
use chip_project::{
    ProjectDocument, create_private_temp_extract_dir, is_chipz_path, load_model_entry_cad_doc,
    resolve_model_path, unpack_chipz_bundle,
};
use chip_sim::{SimDiagnostic, SimResolution};
use serde::Serialize;

pub struct TempExtractGuard(PathBuf);

impl Drop for TempExtractGuard {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

pub struct LoadedCliInput {
    pub proj: ProjectDocument,
    pub dir: Option<PathBuf>,
    pub _guard: Option<TempExtractGuard>,
}

pub fn load_project_or_chipz(path: &Path) -> Result<LoadedCliInput, Box<dyn std::error::Error>> {
    if is_chipz_path(path) {
        let temp_dir = create_private_temp_extract_dir("chipcutter_cli_chipz")?;
        let guard = TempExtractGuard(temp_dir.clone());
        let unpacked = unpack_chipz_bundle(path, &temp_dir)?;
        Ok(LoadedCliInput {
            proj: unpacked.project,
            dir: Some(temp_dir),
            _guard: Some(guard),
        })
    } else {
        let proj = ProjectDocument::load_from_path(path)?;
        let dir = path.parent().map(Path::to_path_buf);
        Ok(LoadedCliInput {
            proj,
            dir,
            _guard: None,
        })
    }
}

pub fn load_primary_cad_for_project(
    proj: &ProjectDocument,
    proj_dir: Option<&Path>,
) -> Result<CadDocument, Box<dyn std::error::Error>> {
    let Some(first) = proj.models.first() else {
        return Ok(CadDocument::empty());
    };
    let model_path = resolve_model_path(proj_dir, &first.path);
    Ok(load_model_entry_cad_doc(&model_path, first.unit_override)
        .map_err(|e| format!("failed to load model '{}': {e}", model_path.display()))?)
}

pub fn parse_sim_resolution(raw: &str) -> SimResolution {
    match raw.trim().to_ascii_lowercase().as_str() {
        "fine" | "1024" => SimResolution::Fine,
        "ultra" | "2048" => SimResolution::Ultra,
        _ => SimResolution::Standard,
    }
}

#[derive(Debug, Serialize)]
pub struct SetupSimReportJson {
    pub setup_id: String,
    pub setup_name: String,
    pub resolution: SimResolution,
    pub total_time_sec: f64,
    pub total_time_hms: String,
    pub cutting_time_sec: f64,
    pub rapid_time_sec: f64,
    pub tool_change_time_sec: f64,
    pub initial_stock_volume_mm3: f64,
    pub remaining_stock_volume_mm3: f64,
    pub removed_volume_mm3: f64,
    pub critical_error_count: usize,
    pub warning_count: usize,
    pub diagnostics: Vec<SimDiagnostic>,
}

#[derive(Debug, Serialize)]
pub struct ProjectSimReportJson {
    pub project_name: String,
    pub passed: bool,
    pub setups: Vec<SetupSimReportJson>,
}

#[derive(Debug, Serialize)]
pub struct BuildFileSummaryJson {
    pub setup_name: String,
    pub post_profile_id: String,
    pub filename: String,
    pub line_count: usize,
    pub max_line_chars: usize,
    pub tool_numbers: Vec<u32>,
    pub operation_names: Vec<String>,
    pub est_time_sec: f64,
}

#[derive(Debug, Serialize)]
pub struct BuildSummaryJson {
    pub project_name: String,
    pub files: Vec<BuildFileSummaryJson>,
    pub simulation: Vec<SetupSimReportJson>,
}
