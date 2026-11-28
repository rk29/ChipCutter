//! Canonical `f64` millimeter math, units, inline expression evaluation,
//! orthonormal `SE(3)` coordinate frames, 2D line/arc curves, and 3D spatial ray primitives.

pub mod contour;
pub mod expr;
pub mod frame;
pub mod geom2d;
pub mod geom3d;
pub mod units;

pub use contour::trace_binary_grid_contours;
pub use expr::{EvalContext, ExprError, ParamLength, evaluate_length_mm};
pub use frame::{FrameError, RigidFrame3};
pub use geom2d::{
    Aabb2, ArcPolyline2D, ArcSegment2D, PolylineProjection, circumcircle_through_three_points,
    closest_point_on_segment_2d, point_in_triangle_2d, point_to_segment_dist_2d,
    point_to_segment_dist_sq_2d, segment_to_segment_dist_2d,
};
pub use geom3d::{
    Aabb3, Ray3, SegmentHit, circumcircle_3d, encode_binary_stl, rotation_matrix_to_xyz_euler_deg,
    simplify_3d_polyline_rdp,
};
pub use glam::{DAffine2, DAffine3, DMat2, DMat3, DMat4, DQuat, DVec2, DVec3, DVec4};
pub use units::{
    ANALYTIC_CLASSIFY_TOL_MM, COINCIDENCE_TOL_MM, DEFAULT_ARC_FIT_TOL_MM,
    DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, METERS_PER_FOOT, MM_PER_INCH, approx_eq,
    format_cycle_time_hms,
};
