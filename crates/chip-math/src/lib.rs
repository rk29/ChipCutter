//! Canonical `f64` millimeter math, units, inline expression evaluation,
//! 2D line/arc curves, and 3D spatial ray primitives for ChipCutter.

pub mod expr;
pub mod geom2d;
pub mod geom3d;
pub mod units;

pub use expr::{EvalContext, ExprError, ParamLength, evaluate_length_mm};
pub use geom2d::{Aabb2, ArcPolyline2D, ArcSegment2D};
pub use geom3d::{Aabb3, Ray3, SegmentHit};
pub use glam::{DAffine3, DMat3, DMat4, DQuat, DVec2, DVec3, DVec4};
pub use units::{
    ANALYTIC_CLASSIFY_TOL_MM, COINCIDENCE_TOL_MM, DEFAULT_ARC_FIT_TOL_MM,
    DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, MM_PER_INCH, approx_eq,
};
