//! Deterministic synthetic STEP B-Rep, STL/OBJ/3MF mesh, and DXF/SVG vector fixtures
//! shared across workspace tests, tutorials, and demo catalogs (`chip-fixtures`).
//!
//! Intentionally depends only on Layer 0/1 primitives (`chip-math`, `monstertruck-modeling`,
//! `monstertruck-io`, `dxf`, `zip`, `thiserror`) with zero dependency on `chip-cad` or
//! `chip-sketch`, so every core crate can list `chip-fixtures` in `[dev-dependencies]`
//! without dependency cycles.

pub mod mesh;
pub mod step;
pub mod vector;

pub use mesh::{
    sample_box_3mf_bytes, sample_helical_flute_shaft_stl_bytes, sample_relief_wave_obj_text,
    sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes, sample_sculpted_mount_3mf_bytes,
    sample_triple_helix_loft_stl_bytes, sample_two_sided_vise_part_stl_bytes,
    tutorial_yup_bracket_stl_bytes,
};
pub use step::{
    SampleStepError, sample_ambiguous_twin_hole_step, sample_cam_torture_block_step,
    sample_curved_deburr_overhang_step, sample_curved_deburr_overhang_step_bytes,
    sample_dovetail_rail_keyway_step, sample_dovetail_rail_keyway_step_bytes,
    sample_indexed_4sided_bar_step, sample_indexed_4sided_bar_step_bytes,
    sample_keepout_clamp_plate_step, sample_keepout_clamp_plate_step_bytes,
    sample_machined_plate_step, sample_machined_plate_step_inches,
    sample_multi_feature_bracket_step, sample_overhang_flange_step,
    sample_periodic_helical_sweep_step, sample_revised_part_pair_step,
    sample_sloped_boss_stepped_pocket_step, sample_threaded_manifold_step,
    sample_undercut_deburr_block_step, sample_undercut_deburr_block_step_bytes,
    tutorial_adaptive_pocket_step, tutorial_chamfer_engrave_block_step,
    tutorial_contour_plate_step, tutorial_drill_bore_plate_step, tutorial_facing_billet_step,
    tutorial_indexed_4side_block_step, tutorial_pocket_block_step,
};
pub use vector::{sample_badge_logo_svg, sample_router_panel_dxf, sample_svg_badge_bytes};
