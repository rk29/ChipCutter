//! Typed Cutter Location (CL) IR events, toolpath spans, and work-offset / rotary-unwind
//! primitives shared between `chip-cam`, `chip-sim`, and `chip-post` (`chip-cl`).

pub mod cl_data;
pub mod toolpath;

pub use cl_data::{
    ArcDir, ArcPlane, ClEvent, CutterCompMode, DrillCycleKind, MotionRole, RotaryUnwindMode,
    WorkOffsetSlot,
};
pub use toolpath::{
    ClearingSubToolpath, DEFAULT_RAPID_RATE_MM_MIN, DrillCycleSpec, GeneratedToolpath,
    SegmentMotion3D, TabMarker3D, ToolpathBuilder, ToolpathSegment3D, ToolpathStats,
    VerticalArcMove, XyArcMove, expand_drill_cycle_segments, rebuild_segments_from_events,
};
