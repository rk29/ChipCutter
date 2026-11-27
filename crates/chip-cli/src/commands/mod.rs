pub mod bundle_reload_tools;
pub mod generate_sample;
pub mod import_2d;
pub mod import_mesh;
pub mod inspect;
pub mod machines;
pub mod transform_model;
pub mod validate_sim_build;
pub mod workholding;

pub use chip_project::{load_primary_cad_for_project, load_project_or_chipz};
use chip_sim::{
    ActiveSimBackendInfo, SimComputeBackendChoice, SimDiagnostic, SimEngineKind, SimResolution,
    SimTimingBreakdown, StockErrorReport,
};
use serde::Serialize;

pub fn parse_sim_resolution(raw: &str) -> SimResolution {
    SimResolution::from_str_loose(raw).unwrap_or(SimResolution::Standard)
}

pub fn parse_sim_engine(raw: &str) -> Result<SimEngineKind, String> {
    SimEngineKind::from_str_loose(raw)
}

pub fn parse_sim_backend(raw: &str) -> Result<SimComputeBackendChoice, String> {
    SimComputeBackendChoice::from_str_loose(raw)
}

#[derive(Debug, Serialize)]
pub struct SetupSimReportJson {
    pub setup_id: String,
    pub setup_name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub carried_over_from_setup: Option<String>,
    pub engine: SimEngineKind,
    pub stock_kind: String,
    pub resolution: SimResolution,
    pub backend: ActiveSimBackendInfo,
    pub timing: SimTimingBreakdown,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_report: Option<StockErrorReport>,
}

#[derive(Debug, Serialize)]
pub struct ProjectSimReportJson {
    pub project_name: String,
    pub passed: bool,
    pub setups: Vec<SetupSimReportJson>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cumulative_error_report: Option<StockErrorReport>,
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
