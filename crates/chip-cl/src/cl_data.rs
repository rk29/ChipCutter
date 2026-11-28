use chip_math::DVec3;
use chip_tools::{CoolantMode, SpindleDirection};
use serde::{Deserialize, Serialize};

/// Controller work coordinate offset slot (`G54`..`G59`, `G59.1`..`G59.3`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum WorkOffsetSlot {
    #[serde(rename = "default")]
    Default,
    #[default]
    G54,
    G55,
    G56,
    G57,
    G58,
    G59,
    #[serde(rename = "G59.1")]
    G59_1,
    #[serde(rename = "G59.2")]
    G59_2,
    #[serde(rename = "G59.3")]
    G59_3,
}

impl WorkOffsetSlot {
    pub const ALL: [Self; 10] = [
        Self::Default,
        Self::G54,
        Self::G55,
        Self::G56,
        Self::G57,
        Self::G58,
        Self::G59,
        Self::G59_1,
        Self::G59_2,
        Self::G59_3,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Default => "0 (Controller Default)",
            Self::G54 => "G54 (Offset 1)",
            Self::G55 => "G55 (Offset 2)",
            Self::G56 => "G56 (Offset 3)",
            Self::G57 => "G57 (Offset 4)",
            Self::G58 => "G58 (Offset 5)",
            Self::G59 => "G59 (Offset 6)",
            Self::G59_1 => "G59.1 (Offset 7)",
            Self::G59_2 => "G59.2 (Offset 8)",
            Self::G59_3 => "G59.3 (Offset 9)",
        }
    }

    pub const fn short_code(self) -> &'static str {
        match self {
            Self::Default => "Default",
            Self::G54 => "G54",
            Self::G55 => "G55",
            Self::G56 => "G56",
            Self::G57 => "G57",
            Self::G58 => "G58",
            Self::G59 => "G59",
            Self::G59_1 => "G59.1",
            Self::G59_2 => "G59.2",
            Self::G59_3 => "G59.3",
        }
    }
}

/// Angular unwinding / rollover strategy when commanding rotary target angles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RotaryUnwindMode {
    /// Allow continuous multi-turn coordinates (e.g., `0° -> 720° -> 1440°` for helical spirals).
    #[default]
    ContinuousMultiTurn,
    /// Move via shortest signed angular delta `[-180°, +180°]` from current angle.
    WrapShortestPath,
    /// Fold all target angles into `[0°, 360°)`.
    Modulo0To360,
}

impl RotaryUnwindMode {
    pub const ALL: [Self; 3] = [
        Self::ContinuousMultiTurn,
        Self::WrapShortestPath,
        Self::Modulo0To360,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ContinuousMultiTurn => "Continuous Multi-Turn",
            Self::WrapShortestPath => "Shortest Path (±180°)",
            Self::Modulo0To360 => "Modulo [0°, 360°)",
        }
    }
}

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
    /// Pure angular 4th-axis positioning / indexing move (at safe rotary clearance height).
    RotaryIndex {
        angle_deg: f64,
        rapid: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        feed_deg_min: Option<f64>,
    },
    /// Coordinated 4-axis rapid traverse (`X/Y/Z` + rotary axis).
    RotaryRapidMove {
        end_wcs: DVec3,
        rotary_deg: f64,
    },
    /// Coordinated 4-axis linear + rotary cutting or linking move (post computes `G93` or `G94`).
    RotaryLinearMove {
        end_wcs: DVec3,
        rotary_deg: f64,
        feed_mm_min: f64,
        /// Effective cutting radius (`mm`) from rotary centerline used to compute `G93` inverse-time feed.
        effective_radius_mm: f64,
        role: MotionRole,
    },
    Dwell {
        seconds: f64,
    },
    Comment(String),
    OperationEnd,
    ProgramEnd,
}
