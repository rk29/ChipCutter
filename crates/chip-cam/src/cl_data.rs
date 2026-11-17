use chip_math::DVec3;
use chip_tools::{CoolantMode, SpindleDirection};
use serde::{Deserialize, Serialize};

use crate::wcs::WorkOffsetSlot;

/// Functional role of a feed motion segment (`07-post-processor.md` & `08-ui-viewport.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionRole {
    Plunge,
    Ramp,
    LeadIn,
    Cut,
    FinishCut,
    LeadOut,
    TabRamp,
    LinkTransition,
}

/// Active circular interpolation plane (`G17` XY, `G18` XZ, `G19` YZ).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArcPlane {
    XY,
    XZ,
    YZ,
}

/// Circular/helical rotation sense viewed down the negative normal of [`ArcPlane`] (`G2` CW, `G3` CCW).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArcDir {
    CW,
    CCW,
}

/// Optional controller radius wear compensation metadata (`G40`/`G41`/`G42` in LinuxCNC).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CutterCompMode {
    #[default]
    Off,
    WearLeft,
    WearRight,
}

/// Canned or expanded drilling cycle variant (`05-operations.md` §4A & `07-post-processor.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DrillCycleKind {
    Spot,
    #[default]
    Simple,
    ChipBreak,
    DeepPeck,
}

impl DrillCycleKind {
    pub const ALL: [Self; 4] = [Self::Spot, Self::Simple, Self::ChipBreak, Self::DeepPeck];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Spot => "Spot / Dwell (G82)",
            Self::Simple => "Simple Feed Out (G81)",
            Self::ChipBreak => "Chip-Breaking Partial Retract (G73)",
            Self::DeepPeck => "Deep-Hole Full Peck (G83)",
        }
    }
}

/// Strongly typed Cutter Location (CL) IR event emitted by `chip-cam` (`07-post-processor.md`).
///
/// Coordinates and center offsets (`IJK`) are stored in canonical `f64` Setup WCS millimeters,
/// feedrates in `mm/min`, and durations in `seconds`. `chip-cam` never emits G-code strings directly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum ClEvent {
    SetupStart {
        setup_name: String,
        wcs_slot: WorkOffsetSlot,
    },
    OperationStart {
        op_id: String,
        op_name: String,
        strategy_name: String,
        est_time_sec: f64,
    },
    ToolChange {
        tool_number: u32,
        length_offset: u32,
        diameter_offset: u32,
        tool_desc: String,
        is_first_tool: bool,
    },
    SpindleStart {
        rpm: f64,
        direction: SpindleDirection,
    },
    SpindleStop,
    Coolant {
        mode: CoolantMode,
    },
    CutterComp {
        mode: CutterCompMode,
    },
    RapidMove {
        end_wcs: DVec3,
    },
    LinearMove {
        end_wcs: DVec3,
        feed_mm_min: f64,
        role: MotionRole,
    },
    ArcMove {
        plane: ArcPlane,
        dir: ArcDir,
        end_wcs: DVec3,
        /// Incremental center offset `(I, J, K)` from the arc start point in WCS millimeters.
        center_offset_ijk: DVec3,
        radius_mm: f64,
        /// Additional full `360°` revolutions (`0` for standard `<= 360°` arc; `>= 1` for multi-turn helix).
        turns: u32,
        feed_mm_min: f64,
        role: MotionRole,
    },
    DrillCycle {
        kind: DrillCycleKind,
        bottom_wcs: DVec3,
        r_plane_z_wcs: f64,
        clear_z_wcs: f64,
        peck_q_mm: f64,
        decreasing_factor: Option<f64>,
        min_peck_mm: f64,
        chip_break_retract_mm: f64,
        dwell_sec: f64,
        feed_mm_min: f64,
    },
    Dwell {
        seconds: f64,
    },
    Comment(String),
    OperationEnd,
    ProgramEnd,
}
