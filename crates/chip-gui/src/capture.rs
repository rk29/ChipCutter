//! Deterministic headless screenshot capture options (`--self-test-screenshot` + `--capture-*`).
//!
//! Pure configuration and CLI-string parsing live here so tutorial and CI screenshot
//! automation stays strictly isolated from interactive UI state and core CAM crates.

use crate::{
    camera::StandardView,
    layout::WorkspacePane,
    panels::{OperationTab, SetupTab},
};

/// Camera framing preset applied right before capturing a headless screenshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureView {
    Standard(StandardView),
    SetupTop,
    ActiveSketch,
}

impl CaptureView {
    /// Parse a CLI view token (`iso`, `top`, `front`, `right`, `left`, `back`, `bottom`, `setup-top`, `sketch`).
    pub fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "iso" | "isometric" => Ok(Self::Standard(StandardView::Isometric)),
            "top" => Ok(Self::Standard(StandardView::Top)),
            "bottom" => Ok(Self::Standard(StandardView::Bottom)),
            "front" => Ok(Self::Standard(StandardView::Front)),
            "back" => Ok(Self::Standard(StandardView::Back)),
            "right" => Ok(Self::Standard(StandardView::Right)),
            "left" => Ok(Self::Standard(StandardView::Left)),
            "setup-top" | "setup_top" => Ok(Self::SetupTop),
            "sketch" | "active-sketch" | "active_sketch" => Ok(Self::ActiveSketch),
            other => Err(format!(
                "invalid --capture-view '{other}'; expected one of: iso, top, bottom, front, back, right, left, setup-top, sketch"
            )),
        }
    }
}

/// Inspector focus and active sub-tab applied before capturing a headless screenshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureFocus {
    Project,
    Setup {
        setup_idx: usize,
        tab: SetupTab,
    },
    Operation {
        setup_idx: usize,
        op_idx: usize,
        tab: OperationTab,
    },
    Sketch {
        sketch_idx: usize,
    },
    Model {
        model_idx: usize,
    },
}

impl CaptureFocus {
    /// Parse a CLI focus token:
    /// - `project`
    /// - `setup:<wcs|stock|heights>` or `setup:<setup_idx>:<wcs|stock|heights>`
    /// - `op:<op_idx>:<tool|geometry|heights|passes|linking>` or `op:<setup_idx>:<op_idx>:<tab>`
    /// - `sketch:<sketch_idx>`
    /// - `model` or `model:<model_idx>`
    pub fn parse(raw: &str) -> Result<Self, String> {
        let trimmed = raw.trim();
        let parts: Vec<&str> = trimmed.split(':').collect();
        match parts.as_slice() {
            ["project"] => Ok(Self::Project),
            ["setup", tab_str] => Ok(Self::Setup {
                setup_idx: 0,
                tab: parse_setup_tab(tab_str)?,
            }),
            ["setup", setup_str, tab_str] => Ok(Self::Setup {
                setup_idx: parse_index(setup_str, "setup index")?,
                tab: parse_setup_tab(tab_str)?,
            }),
            ["op", op_str, tab_str] => Ok(Self::Operation {
                setup_idx: 0,
                op_idx: parse_index(op_str, "operation index")?,
                tab: parse_op_tab(tab_str)?,
            }),
            ["op", setup_str, op_str, tab_str] => Ok(Self::Operation {
                setup_idx: parse_index(setup_str, "setup index")?,
                op_idx: parse_index(op_str, "operation index")?,
                tab: parse_op_tab(tab_str)?,
            }),
            ["sketch", sketch_str] => Ok(Self::Sketch {
                sketch_idx: parse_index(sketch_str, "sketch index")?,
            }),
            ["model"] => Ok(Self::Model { model_idx: 0 }),
            ["model", model_str] => Ok(Self::Model {
                model_idx: parse_index(model_str, "model index")?,
            }),
            _ => Err(format!(
                "invalid --capture-focus '{trimmed}'; expected 'project', 'setup:<wcs|stock|heights>', 'op:<idx>:<tool|geometry|heights|passes|linking>', 'sketch:<idx>', or 'model[:idx]'"
            )),
        }
    }
}

fn parse_index(raw: &str, label: &str) -> Result<usize, String> {
    raw.trim()
        .parse::<usize>()
        .map_err(|_| format!("invalid {label} '{raw}'; expected a non-negative integer"))
}

fn parse_setup_tab(raw: &str) -> Result<SetupTab, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "wcs" => Ok(SetupTab::Wcs),
        "stock" => Ok(SetupTab::Stock),
        "fixtures" | "fixture" | "workholding" => Ok(SetupTab::Fixtures),
        "rotary" | "4th" | "4axis" => Ok(SetupTab::Rotary),
        "heights" => Ok(SetupTab::Heights),
        other => Err(format!(
            "invalid Setup tab '{other}'; expected one of: wcs, stock, fixtures, rotary, heights"
        )),
    }
}

fn parse_op_tab(raw: &str) -> Result<OperationTab, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "tool" => Ok(OperationTab::Tool),
        "geometry" | "geo" => Ok(OperationTab::Geometry),
        "heights" => Ok(OperationTab::Heights),
        "passes" => Ok(OperationTab::Passes),
        "linking" => Ok(OperationTab::Linking),
        other => Err(format!(
            "invalid Operation tab '{other}'; expected one of: tool, geometry, heights, passes, linking"
        )),
    }
}

/// Bottom workspace tab to activate before capturing a headless screenshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapturePane {
    Simulation,
    GCode,
    Diagnostics,
}

impl CapturePane {
    pub fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "sim" | "simulation" => Ok(Self::Simulation),
            "gcode" | "g-code" | "post" => Ok(Self::GCode),
            "diagnostics" | "diag" | "log" => Ok(Self::Diagnostics),
            other => Err(format!(
                "invalid --capture-pane '{other}'; expected one of: sim, gcode, diagnostics"
            )),
        }
    }

    pub const fn to_workspace_pane(self) -> WorkspacePane {
        match self {
            Self::Simulation => WorkspacePane::SimulationTimeline,
            Self::GCode => WorkspacePane::GCodePreview,
            Self::Diagnostics => WorkspacePane::DiagnosticsLog,
        }
    }
}

/// Modal overlay to open before capturing a headless screenshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureModal {
    Welcome,
    SampleGallery,
    ToolLibrary,
    Machines,
    FlipJigWizard,
    SoftJawWizard,
    ShortcutCheatsheet,
    About,
}

impl CaptureModal {
    pub fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "welcome" => Ok(Self::Welcome),
            "samples" | "sample-gallery" | "gallery" => Ok(Self::SampleGallery),
            "tools" | "tool-library" => Ok(Self::ToolLibrary),
            "machines" | "machine" | "machine-manager" => Ok(Self::Machines),
            "flip-jig" | "flip-jig-wizard" | "flip_jig" => Ok(Self::FlipJigWizard),
            "soft-jaws" | "soft-jaw" | "soft-jaw-wizard" | "soft_jaws" => Ok(Self::SoftJawWizard),
            "shortcuts" | "cheatsheet" => Ok(Self::ShortcutCheatsheet),
            "about" => Ok(Self::About),
            other => Err(format!(
                "invalid --capture-modal '{other}'; expected one of: welcome, samples, tools, machines, flip-jig, soft-jaws, shortcuts, about"
            )),
        }
    }
}

/// Parse and normalize a built-in post-processor profile identifier for `--capture-post`.
pub fn parse_capture_post_profile(raw: &str) -> Result<String, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "grbl" => Ok("grbl".to_string()),
        "grblhal_atc" | "grblhal-atc" | "grblhal" => Ok("grblhal_atc".to_string()),
        "linuxcnc" => Ok("linuxcnc".to_string()),
        "linuxcnc_4axis" | "linuxcnc-4axis" => Ok("linuxcnc_4axis".to_string()),
        "grblhal_4axis" | "grblhal-4axis" => Ok("grblhal_4axis".to_string()),
        "grbl_rotary_wrap" | "grbl-rotary-wrap" => Ok("grbl_rotary_wrap".to_string()),
        other => Err(format!(
            "invalid --capture-post '{other}'; expected one of: grbl, grblhal_atc, linuxcnc"
        )),
    }
}

/// Parse and normalize a simulation engine identifier for `--capture-sim-engine`.
pub fn parse_capture_sim_engine(raw: &str) -> Result<chip_sim::SimEngineKind, String> {
    chip_sim::SimEngineKind::from_str_loose(raw)
}

/// Parse and normalize a simulation compute backend choice for `--capture-sim-backend`.
pub fn parse_capture_sim_backend(raw: &str) -> Result<chip_sim::SimComputeBackendChoice, String> {
    chip_sim::SimComputeBackendChoice::from_str_loose(raw)
}

/// Parse and normalize a simulation resolution preset for `--capture-sim-resolution`.
pub fn parse_capture_sim_resolution(raw: &str) -> Result<chip_sim::SimResolution, String> {
    chip_sim::SimResolution::from_str_loose(raw)
}

/// Structured options controlling headless UI state before PNG capture.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CaptureSpec {
    pub setup_idx: Option<usize>,
    pub view: Option<CaptureView>,
    pub focus: Option<CaptureFocus>,
    pub pane: Option<CapturePane>,
    pub modal: Option<CaptureModal>,
    pub isolate_op: bool,
    pub show_sim_stock: Option<bool>,
    pub show_sim_cutter: Option<bool>,
    pub sim_progress: Option<f64>,
    pub sim_after_op: Option<usize>,
    pub sim_engine: Option<chip_sim::SimEngineKind>,
    pub sim_backend: Option<chip_sim::SimComputeBackendChoice>,
    pub sim_resolution: Option<chip_sim::SimResolution>,
    pub sim_heatmap: bool,
    pub post_profile: Option<String>,
    pub auto_generate: bool,
    pub show_z_planes: Option<bool>,
    pub show_27pt_dots: Option<bool>,
    pub show_fixtures: Option<bool>,
    pub ghost_part: Option<bool>,
    pub reach_heatmap: bool,
    pub measure_demo: bool,
    pub expand_machinability: bool,
    pub apply_safe_fixes: bool,
    pub thumb_path: Option<std::path::PathBuf>,
}

impl CaptureSpec {
    /// Return normalized simulation scrub progress in `[0.0, 1.0]`, defaulting to `0.82`
    /// for backward compatibility with existing `--self-test-screenshot` snapshots.
    pub fn effective_sim_progress(&self) -> f64 {
        self.sim_progress.map_or(0.82, |v| {
            if v.is_finite() {
                v.clamp(0.0, 1.0)
            } else {
                0.82
            }
        })
    }

    /// Resolve the exact simulation timestamp (in seconds) for this capture spec.
    ///
    /// When `sim_after_op` is set, returns the exact `end_time_sec` of that operation in the
    /// simulated timeline (matching `op_id_for_index` first so suppressed operations do not skew
    /// indexing, then falling back to positional index and finally `total_time_sec`).
    pub fn resolve_sim_timestamp(
        &self,
        timeline: &chip_sim::SimulationTimeline,
        op_id_for_index: Option<&str>,
    ) -> f64 {
        if let Some(op_idx) = self.sim_after_op {
            if let Some(op_id) = op_id_for_index
                && let Some(entry) = timeline.operations.iter().rfind(|o| o.op_id == op_id)
            {
                return entry.end_time_sec.clamp(0.0, timeline.total_time_sec);
            }
            if let Some(entry) = timeline.operations.get(op_idx) {
                return entry.end_time_sec.clamp(0.0, timeline.total_time_sec);
            }
            return timeline.total_time_sec;
        }
        timeline.total_time_sec * self.effective_sim_progress()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_capture_views() {
        assert_eq!(
            CaptureView::parse("iso"),
            Ok(CaptureView::Standard(StandardView::Isometric))
        );
        assert_eq!(
            CaptureView::parse("TOP"),
            Ok(CaptureView::Standard(StandardView::Top))
        );
        assert_eq!(CaptureView::parse("setup-top"), Ok(CaptureView::SetupTop));
        assert_eq!(CaptureView::parse("sketch"), Ok(CaptureView::ActiveSketch));
        assert!(CaptureView::parse("invalid-view").is_err());
    }

    #[test]
    fn parses_capture_focus_variants() {
        assert_eq!(CaptureFocus::parse("project"), Ok(CaptureFocus::Project));
        assert_eq!(
            CaptureFocus::parse("setup:wcs"),
            Ok(CaptureFocus::Setup {
                setup_idx: 0,
                tab: SetupTab::Wcs,
            })
        );
        assert_eq!(
            CaptureFocus::parse("setup:1:stock"),
            Ok(CaptureFocus::Setup {
                setup_idx: 1,
                tab: SetupTab::Stock,
            })
        );
        assert_eq!(
            CaptureFocus::parse("setup:fixtures"),
            Ok(CaptureFocus::Setup {
                setup_idx: 0,
                tab: SetupTab::Fixtures,
            })
        );
        assert_eq!(
            CaptureFocus::parse("op:2:passes"),
            Ok(CaptureFocus::Operation {
                setup_idx: 0,
                op_idx: 2,
                tab: OperationTab::Passes,
            })
        );
        assert_eq!(
            CaptureFocus::parse("op:1:3:linking"),
            Ok(CaptureFocus::Operation {
                setup_idx: 1,
                op_idx: 3,
                tab: OperationTab::Linking,
            })
        );
        assert_eq!(
            CaptureFocus::parse("sketch:0"),
            Ok(CaptureFocus::Sketch { sketch_idx: 0 })
        );
        assert_eq!(
            CaptureFocus::parse("model"),
            Ok(CaptureFocus::Model { model_idx: 0 })
        );
        assert_eq!(
            CaptureFocus::parse("model:0"),
            Ok(CaptureFocus::Model { model_idx: 0 })
        );
        assert!(CaptureFocus::parse("op:abc:passes").is_err());
        assert!(CaptureFocus::parse("setup:unknown").is_err());
        assert!(CaptureFocus::parse("bogus").is_err());
    }

    #[test]
    fn parses_panes_and_modals() {
        assert_eq!(CapturePane::parse("sim"), Ok(CapturePane::Simulation));
        assert_eq!(CapturePane::parse("gcode"), Ok(CapturePane::GCode));
        assert_eq!(
            CapturePane::parse("diagnostics"),
            Ok(CapturePane::Diagnostics)
        );
        assert_eq!(
            CapturePane::GCode.to_workspace_pane(),
            WorkspacePane::GCodePreview
        );
        assert!(CapturePane::parse("nope").is_err());

        assert_eq!(CaptureModal::parse("welcome"), Ok(CaptureModal::Welcome));
        assert_eq!(
            CaptureModal::parse("samples"),
            Ok(CaptureModal::SampleGallery)
        );
        assert_eq!(CaptureModal::parse("tools"), Ok(CaptureModal::ToolLibrary));
        assert_eq!(CaptureModal::parse("machines"), Ok(CaptureModal::Machines));
        assert_eq!(
            CaptureModal::parse("machine-manager"),
            Ok(CaptureModal::Machines)
        );
        assert_eq!(
            CaptureModal::parse("flip-jig"),
            Ok(CaptureModal::FlipJigWizard)
        );
        assert_eq!(
            CaptureModal::parse("soft-jaws"),
            Ok(CaptureModal::SoftJawWizard)
        );
        assert_eq!(
            CaptureModal::parse("shortcuts"),
            Ok(CaptureModal::ShortcutCheatsheet)
        );
        assert_eq!(CaptureModal::parse("about"), Ok(CaptureModal::About));
        assert!(CaptureModal::parse("nope").is_err());
    }

    #[test]
    fn clamps_sim_progress_safely() {
        let def = CaptureSpec::default();
        assert!((def.effective_sim_progress() - 0.82).abs() < 1e-12);

        let high = CaptureSpec {
            sim_progress: Some(1.5),
            ..CaptureSpec::default()
        };
        assert!((high.effective_sim_progress() - 1.0).abs() < 1e-12);

        let low = CaptureSpec {
            sim_progress: Some(-0.2),
            ..CaptureSpec::default()
        };
        assert!((low.effective_sim_progress() - 0.0).abs() < 1e-12);

        let nan = CaptureSpec {
            sim_progress: Some(f64::NAN),
            ..CaptureSpec::default()
        };
        assert!((nan.effective_sim_progress() - 0.82).abs() < 1e-12);
    }

    #[test]
    fn parses_capture_post_profiles() {
        assert_eq!(parse_capture_post_profile("grbl"), Ok("grbl".to_string()));
        assert_eq!(
            parse_capture_post_profile("GRBLHAL-ATC"),
            Ok("grblhal_atc".to_string())
        );
        assert_eq!(
            parse_capture_post_profile("LinuxCNC"),
            Ok("linuxcnc".to_string())
        );
        assert!(parse_capture_post_profile("unknown_post").is_err());
    }

    #[test]
    fn parses_capture_sim_engine() {
        assert_eq!(
            parse_capture_sim_engine("heightmap"),
            Ok(chip_sim::SimEngineKind::Heightmap)
        );
        assert_eq!(
            parse_capture_sim_engine("TRI-DEXEL"),
            Ok(chip_sim::SimEngineKind::TriDexel)
        );
        assert_eq!(
            parse_capture_sim_engine("tridexel"),
            Ok(chip_sim::SimEngineKind::TriDexel)
        );
        assert!(parse_capture_sim_engine("voxel").is_err());
    }

    #[test]
    fn parses_capture_sim_backend_and_resolution() {
        assert_eq!(
            parse_capture_sim_backend("auto"),
            Ok(chip_sim::SimComputeBackendChoice::Auto)
        );
        assert_eq!(
            parse_capture_sim_backend("GPU"),
            Ok(chip_sim::SimComputeBackendChoice::Gpu)
        );
        assert_eq!(
            parse_capture_sim_backend("cpu"),
            Ok(chip_sim::SimComputeBackendChoice::Cpu)
        );
        assert!(parse_capture_sim_backend("cuda").is_err());

        assert_eq!(
            parse_capture_sim_resolution("standard"),
            Ok(chip_sim::SimResolution::Standard)
        );
        assert_eq!(
            parse_capture_sim_resolution("FINE"),
            Ok(chip_sim::SimResolution::Fine)
        );
        assert_eq!(
            parse_capture_sim_resolution("ultra"),
            Ok(chip_sim::SimResolution::Ultra)
        );
        assert!(parse_capture_sim_resolution("extreme").is_err());
    }

    #[test]
    fn resolves_sim_timestamp_after_operation() {
        let mut timeline = chip_sim::SimulationTimeline {
            total_time_sec: 120.0,
            ..Default::default()
        };
        timeline
            .operations
            .push(chip_sim::OperationTimelineSummary {
                op_idx: 0,
                op_id: "op_face".to_string(),
                op_name: "Facing".to_string(),
                tool_id: chip_tools::ToolId("t1".to_string()),
                tool_number: 1,
                tool_name: "T1".to_string(),
                spindle_rpm: 10_000.0,
                first_global_seg: 0,
                end_global_seg: 5,
                start_time_sec: 0.0,
                end_time_sec: 35.5,
                cutting_time_sec: 30.0,
                rapid_time_sec: 5.5,
                dwell_time_sec: 0.0,
                tool_change_time_sec: 0.0,
                cutting_distance_mm: 500.0,
                rapid_distance_mm: 100.0,
            });
        timeline
            .operations
            .push(chip_sim::OperationTimelineSummary {
                op_idx: 1,
                op_id: "op_pocket".to_string(),
                op_name: "Pocket".to_string(),
                tool_id: chip_tools::ToolId("t2".to_string()),
                tool_number: 2,
                tool_name: "T2".to_string(),
                spindle_rpm: 12_000.0,
                first_global_seg: 5,
                end_global_seg: 15,
                start_time_sec: 35.5,
                end_time_sec: 95.0,
                cutting_time_sec: 50.0,
                rapid_time_sec: 9.5,
                dwell_time_sec: 0.0,
                tool_change_time_sec: 0.0,
                cutting_distance_mm: 900.0,
                rapid_distance_mm: 120.0,
            });

        let by_id = CaptureSpec {
            sim_after_op: Some(2),
            ..CaptureSpec::default()
        };
        assert!((by_id.resolve_sim_timestamp(&timeline, Some("op_pocket")) - 95.0).abs() < 1e-12);

        let by_pos = CaptureSpec {
            sim_after_op: Some(0),
            ..CaptureSpec::default()
        };
        assert!((by_pos.resolve_sim_timestamp(&timeline, None) - 35.5).abs() < 1e-12);

        let oob = CaptureSpec {
            sim_after_op: Some(9),
            ..CaptureSpec::default()
        };
        assert!((oob.resolve_sim_timestamp(&timeline, None) - 120.0).abs() < 1e-12);

        let progress = CaptureSpec {
            sim_progress: Some(0.5),
            ..CaptureSpec::default()
        };
        assert!((progress.resolve_sim_timestamp(&timeline, None) - 60.0).abs() < 1e-12);
    }
}
