//! Declarative sample & tutorial registry unifying raw asset bytes, metadata, and
//! [`ProjectDocument`] builders across `chip-cli` and `chip-gui`.

use chip_cad::{CadDocument, CadKernel, ModelTransform, MonstertruckKernel, TopologyGraph};
use chip_cam::OperationStatus;
use chip_fixtures::{
    SampleStepError, sample_cam_torture_block_step, sample_curved_deburr_overhang_step_bytes,
    sample_dovetail_rail_keyway_step_bytes, sample_helical_flute_shaft_stl_bytes,
    sample_indexed_4sided_bar_step_bytes, sample_keepout_clamp_plate_step_bytes,
    sample_machined_plate_step, sample_multi_feature_bracket_step, sample_overhang_flange_step,
    sample_relief_wave_obj_text, sample_revised_part_pair_step, sample_rotary_shaft_stl_bytes,
    sample_router_panel_dxf, sample_sculpted_dome_stl_bytes, sample_sculpted_mount_3mf_bytes,
    sample_sloped_boss_stepped_pocket_step, sample_svg_badge_bytes, sample_threaded_manifold_step,
    sample_two_sided_vise_part_stl_bytes, sample_undercut_deburr_block_step_bytes,
    tutorial_adaptive_pocket_step, tutorial_chamfer_engrave_block_step,
    tutorial_contour_plate_step, tutorial_drill_bore_plate_step, tutorial_facing_billet_step,
    tutorial_indexed_4side_block_step, tutorial_pocket_block_step, tutorial_yup_bracket_stl_bytes,
};
use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
use chip_project::{
    ModelEntry, ProjectDocument, compute_sha256_hex, rebind_project_to_reloaded_cad,
};
use chip_tools::Tool;

use crate::{
    build_adaptive_rest_demo_project, build_cam_torture_project, build_curved_deburr_3d_project,
    build_dovetail_rail_keyway_project, build_engraved_nameplate_demo_project,
    build_feed_opt_pocket_project, build_flip_jig_softjaws_project,
    build_indexed_model_bounds_project, build_keepout_clamps_project,
    build_mesh_3mf_bracket_project, build_multi_feature_bracket_project,
    build_pure2d_router_panel_demo_project, build_relief_wave_mesh_project,
    build_revision_v1_demo_project, build_rotary_helical_flute_project,
    build_rotary_shaft_preview_project, build_rotary_wrapped_engraving_project,
    build_sculpted_mesh_dome_project, build_svg_badge_sign_project,
    build_thread_mill_manifold_project, build_tutorial_3d_adaptive_waterline_project,
    build_tutorial_3d_surface_finishing_project, build_tutorial_16_tilted_step_project,
    build_tutorial_16_yup_stl_project, build_tutorial_adaptive_rest_project,
    build_tutorial_chamfer_trace_project, build_tutorial_contour_project,
    build_tutorial_drill_bore_project, build_tutorial_facing_project,
    build_tutorial_indexed_step_multiface_project, build_tutorial_pocket_project,
    build_tutorial_rotary_4axis_flowline_project, build_tutorial_rotary_axial_parallel_project,
    build_tutorial_vcarve_inlay_project, build_two_sided_vise_workholding_project,
    build_undercut_deburr_block_project, build_vcarve_sign_and_inlay_demo_project,
};

/// Asset format produced by a [`SampleId`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SampleAssetKind {
    /// No external geometry file; pure-2D / procedural sketch project only.
    None,
    /// ISO 10303-21 STEP B-Rep model.
    Step,
    /// Binary STL triangle mesh.
    Stl,
    /// Wavefront OBJ triangle/quad mesh.
    Obj,
    /// 3MF OPC ZIP archive mesh.
    ThreeMf,
    /// 2D DXF vector drawing.
    Dxf,
    /// 2D SVG vector artwork.
    Svg,
}

impl SampleAssetKind {
    pub const fn is_3d_mesh(self) -> bool {
        matches!(self, Self::Stl | Self::Obj | Self::ThreeMf)
    }

    pub const fn format_label(self) -> &'static str {
        match self {
            Self::None => "Pure-2D",
            Self::Step => "STEP",
            Self::Stl => "STL",
            Self::Obj => "OBJ",
            Self::ThreeMf => "3MF",
            Self::Dxf => "DXF",
            Self::Svg => "SVG",
        }
    }
}

/// Optional GUI focus / viewport defaults applied after seeding a sample project.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFocusHint {
    Default,
    Operation {
        op_idx: usize,
        geometry_tab: bool,
    },
    LastOperationPasses,
    SetupFixtures,
    SketchEntity {
        sketch_idx: usize,
        entity_idx: usize,
    },
}

/// Canonical identifier for every built-in ChipCutter sample & tutorial.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SampleId {
    // Core GUI & CLI 3D STEP/Mesh Demos
    MachinedPlate,
    MultiFeatureBracket,
    MultiFeatureBracketTilted,
    CamTortureBlock,
    AdaptiveRestDemo,
    RevisionDemoV1,
    SculptedMeshDome,
    ReliefWaveMesh,
    RotaryIndexedShaft,
    RotaryHelicalFlute,
    ThreadMillManifold,
    TwoSidedViseWorkholding,
    UndercutAndDeburrBlock,
    FeedOptPocket,
    FlipJigSoftJaws,
    CurvedDeburr3D,
    DovetailRailKeyway,
    Mesh3mfBracket,
    KeepOutClamps,
    IndexedModelBounds,

    // Pure-2D / Vector / Procedural Demos
    SvgBadgeSign,
    RotaryWrappedEngraving,
    Pure2dRouterPanel,
    EngravedNameplate,
    VCarveSignAndInlay,

    // CLI Tutorial Starters
    TutorialFacing,
    TutorialContour,
    TutorialPocket,
    TutorialDrillBore,
    TutorialAdaptiveRest,
    TutorialChamferTrace,
    TutorialVCarveMedial,
    TutorialVCarveInlay,
    Tutorial3dAdaptiveWaterline,
    Tutorial3dSurfaceFinishing,
    TutorialRotaryAxialParallel,
    TutorialIndexedStep,
    TutorialRotary4AxisFlowline,
    TutorialReloadV2,
    TutorialOrientStl,
    TutorialOrientStep,
    TutorialDeburrBoss,
    TutorialDeburrFlange,
}

/// Declarative metadata record for one entry in [`SAMPLE_CATALOG`].
#[derive(Debug, Clone, Copy)]
pub struct SampleSpec {
    pub id: SampleId,
    pub canonical_slug: &'static str,
    pub aliases: &'static [&'static str],
    pub asset_kind: SampleAssetKind,
    pub default_filename: &'static str,
    pub load_status_message: &'static str,
    pub focus_hint: SampleFocusHint,
    pub prefer_tridexel_sim: bool,
    pub show_fixtures: bool,
    pub hide_27pt_dots: bool,
    pub auto_generate_and_sim: bool,
}

/// Complete declarative table of all built-in ChipCutter samples & tutorials.
pub const SAMPLE_CATALOG: &[SampleSpec] = &[
    SampleSpec {
        id: SampleId::MachinedPlate,
        canonical_slug: "machined-plate",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_machined_plate.step",
        load_status_message: "Generating & tessellating sample CNC mounting plate STEP model…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::MultiFeatureBracket,
        canonical_slug: "multi-feature-bracket",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_multi_feature_bracket.step",
        load_status_message: "Generating Milestone 2 multi-feature bracket STEP fixture (tilted=false)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: true,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::MultiFeatureBracketTilted,
        canonical_slug: "multi-feature-bracket-tilted",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_multi_feature_tilted.step",
        load_status_message: "Generating Milestone 2 multi-feature bracket STEP fixture (tilted=true)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: true,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::CamTortureBlock,
        canonical_slug: "cam-torture",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_cam_torture_block.step",
        load_status_message: "Generating Milestone 3 CAM Torture Block STEP fixture (island pocket, open step, Ø18 bore, 45° bevel)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 1,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::AdaptiveRestDemo,
        canonical_slug: "adaptive-rest-demo",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_cam_torture_block.step",
        load_status_message: "Generating Milestone 6 2D Adaptive + Rest Machining STEP job…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 3,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::RevisionDemoV1,
        canonical_slug: "14-reload-v1",
        aliases: &["14-associative-step-reload", "reload-v1"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_revision_part_v1.step",
        load_status_message: "Generating Milestone 5 CAD Revision v1 STEP fixture & seeding CAM job…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 1,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::SculptedMeshDome,
        canonical_slug: "08-mesh-surfacing",
        aliases: &["08-sculpted-dome"],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "sculpted_dome.stl",
        load_status_message: "Generating watertight Sculpted Dome (STL Mesh) sample…",
        focus_hint: SampleFocusHint::LastOperationPasses,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::ReliefWaveMesh,
        canonical_slug: "relief-wave-mesh",
        aliases: &[],
        asset_kind: SampleAssetKind::Obj,
        default_filename: "relief_wave.obj",
        load_status_message: "Generating Relief Wave (OBJ Mesh) 3D finishing sample…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 1,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::RotaryIndexedShaft,
        canonical_slug: "09-rotary-shaft",
        aliases: &["11-indexed-shaft"],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "rotary_shaft.stl",
        load_status_message: "Generating Rotary Indexed Shaft (4th-Axis A) sample…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::RotaryHelicalFlute,
        canonical_slug: "10-simultaneous-4axis",
        aliases: &["10-helical-flute"],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "helical_flute_shaft.stl",
        load_status_message: "Generating Simultaneous 4-Axis Helical Flute (X+Y+Z+A) sample…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 1,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::ThreadMillManifold,
        canonical_slug: "18-thread-milling",
        aliases: &["18-threaded-manifold", "thread-mill", "thread-milling"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "18-threaded-manifold.step",
        load_status_message: "Generating Threaded Manifold Block STEP fixture (M8×1.25, 1/4-20 UNC, External M12×1.75)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 3,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::TwoSidedViseWorkholding,
        canonical_slug: "two-sided-vise",
        aliases: &[],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "two_sided_vise_bracket.stl",
        load_status_message: "Generating Two-Sided Vise & Soft-Jaw Workholding sample (Setup 1 G54 Vise + Toe Clamps -> Setup 2 G55 Flip Carryover)…",
        focus_hint: SampleFocusHint::SetupFixtures,
        prefer_tridexel_sim: false,
        show_fixtures: true,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::UndercutAndDeburrBlock,
        canonical_slug: "22-undercut-block",
        aliases: &["22-undercut-deburr-block"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_undercut_deburr_block.step",
        load_status_message: "Generating Undercut & 3D Deburr Block STEP fixture (M8 T-Slot, 60° Dovetail, Lollipop Overhang, & 3D Deburr)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 3,
            geometry_tab: false,
        },
        prefer_tridexel_sim: true,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::FeedOptPocket,
        canonical_slug: "20-feed-opt",
        aliases: &["20-feed-opt-pocket", "feed-opt"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "20-feed-opt-pocket.step",
        load_status_message: "Generating Feed-Optimized Adaptive Pocket STEP fixture (sharp corners + Ø16 mm boss)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::FlipJigSoftJaws,
        canonical_slug: "flip-jig-softjaws",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "02-contour-plate.step",
        load_status_message: "Generating Flip-Jig Dowel Pins & Auto Soft-Jaw Setup sample (Spoilboard + G54/G55/G56)…",
        focus_hint: SampleFocusHint::SetupFixtures,
        prefer_tridexel_sim: true,
        show_fixtures: true,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::CurvedDeburr3D,
        canonical_slug: "curved-deburr-3d",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_curved_deburr_overhang.step",
        load_status_message: "Generating 3D Curved Arch & Underside Deburring STEP fixture (ChamferMill, BallEndmill, Lollipop)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 3,
            geometry_tab: false,
        },
        prefer_tridexel_sim: true,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::DovetailRailKeyway,
        canonical_slug: "dovetail-rail-keyway",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_dovetail_rail_keyway.step",
        load_status_message: "Generating External 60° Dovetail Rail & Blind Entry-Bore Keyway STEP fixture…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 3,
            geometry_tab: false,
        },
        prefer_tridexel_sim: true,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::Mesh3mfBracket,
        canonical_slug: "08-bracket-3mf",
        aliases: &[],
        asset_kind: SampleAssetKind::ThreeMf,
        default_filename: "08-bracket.3mf",
        load_status_message: "Generating Sculpted Mount Bracket (.3mf unit-aware mesh) sample…",
        focus_hint: SampleFocusHint::LastOperationPasses,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::KeepOutClamps,
        canonical_slug: "keepout-clamps",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_keepout_clamp_plate.step",
        load_status_message: "Generating Clamp & Boss Keep-Out Plate STEP fixture (Shared Avoid Region + Toe Clamps + Boss Keep-Out)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: true,
        },
        prefer_tridexel_sim: false,
        show_fixtures: true,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::IndexedModelBounds,
        canonical_slug: "indexed-model-bounds",
        aliases: &[],
        asset_kind: SampleAssetKind::Step,
        default_filename: "sample_indexed_4sided_bar.step",
        load_status_message: "Generating Indexed 4-Sided Bar STEP fixture (Fixed-Box Center-Z + Model Bounds & Auto-Rotate 3+1)…",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: true,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::SvgBadgeSign,
        canonical_slug: "13-pure2d-svg",
        aliases: &["13-badge-logo", "pure2d-svg"],
        asset_kind: SampleAssetKind::Svg,
        default_filename: "13-badge-logo.svg",
        load_status_message: "Loaded Pure-2D SVG Badge Sign (96 DPI cubic Bézier biarcs — Emblem Pocket + Island, V-Carve Accents, Outer Contour + Tabs).",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: true,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::RotaryWrappedEngraving,
        canonical_slug: "09-rotary-wrap",
        aliases: &["10-rotary-wrap"],
        asset_kind: SampleAssetKind::None,
        default_filename: "rotary_wrapped_engraving.chip.toml",
        load_status_message: "Loaded Wrapped Rotary Engraving (Z+X+A — Ø40×80mm cylinder, wrapped 'CHIP' V-Carve + circumferential flute pocket).",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::Pure2dRouterPanel,
        canonical_slug: "13-pure2d-dxf",
        aliases: &["13-pure2d-dxf-svg-workflow", "pure2d-dxf"],
        asset_kind: SampleAssetKind::Dxf,
        default_filename: "13-router-panel.dxf",
        load_status_message: "Loaded Pure-2D CNC Router Front Panel (.dxf, zero STEP files — 4 operations: Holes, Island Pocket, Vent Slot, Outer Profile + Tabs).",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 1,
            geometry_tab: true,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::EngravedNameplate,
        canonical_slug: "engraved-nameplate",
        aliases: &[],
        asset_kind: SampleAssetKind::None,
        default_filename: "engraved_nameplate.chip.toml",
        load_status_message: "Loaded Engraved Nameplate + Dual-Mode Typography Demo (Bolt Circle Drill, Ø22 Bore, DejaVuSans-Bold 'CHIP' Pocket, Hershey 1-Pass 'S/N 2026-08' Trace, R10 Plaque + Tabs).",
        focus_hint: SampleFocusHint::SketchEntity {
            sketch_idx: 0,
            entity_idx: 3,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::VCarveSignAndInlay,
        canonical_slug: "vcarve-sign-and-inlay",
        aliases: &[],
        asset_kind: SampleAssetKind::None,
        default_filename: "vcarve_sign_and_inlay.chip.toml",
        load_status_message: "Loaded V-Carve Sign & Mated V-Inlay Plug Demo (Setup 1: Two-Stage Ø6mm + 60° V-Bit Crest & 'CHIP' + Medial 'CAM v2' + Plaque Tabs; Setup 2: Mirrored Male V-Inlay Plug).",
        focus_hint: SampleFocusHint::Operation {
            op_idx: 0,
            geometry_tab: false,
        },
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: true,
    },
    SampleSpec {
        id: SampleId::TutorialFacing,
        canonical_slug: "01-facing",
        aliases: &["facing"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "01-facing-billet.step",
        load_status_message: "Generating Tutorial 01 Facing Billet STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialContour,
        canonical_slug: "02-contour",
        aliases: &["02-2d-contour-tabs", "contour"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "02-contour-plate.step",
        load_status_message: "Generating Tutorial 02 Contour Plate STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialPocket,
        canonical_slug: "03-pocket",
        aliases: &["03-2d-pocket", "pocket"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "03-pocket-block.step",
        load_status_message: "Generating Tutorial 03 Pocket Block STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialDrillBore,
        canonical_slug: "04-drill-bore",
        aliases: &["04-drilling-helical-bore", "drill-bore"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "04-drill-bore-plate.step",
        load_status_message: "Generating Tutorial 04 Drill & Bore Plate STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialAdaptiveRest,
        canonical_slug: "05-adaptive-rest",
        aliases: &["05-adaptive-rest-machining", "adaptive-rest"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "05-adaptive-pocket.step",
        load_status_message: "Generating Tutorial 05 Adaptive & Rest Pocket STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialChamferTrace,
        canonical_slug: "06-chamfer-trace",
        aliases: &["06-chamfer-curve-trace", "chamfer-trace"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "06-chamfer-engrave-block.step",
        load_status_message: "Generating Tutorial 06 Chamfer & Trace Block STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialVCarveMedial,
        canonical_slug: "07-vcarve-medial",
        aliases: &["vcarve-medial"],
        asset_kind: SampleAssetKind::None,
        default_filename: "07-vcarve-medial.chip.toml",
        load_status_message: "Generating Tutorial 07 Medial V-Carve project…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialVCarveInlay,
        canonical_slug: "07-vcarve-inlay",
        aliases: &["vcarve-inlay"],
        asset_kind: SampleAssetKind::None,
        default_filename: "07-vcarve-inlay.chip.toml",
        load_status_message: "Generating Tutorial 07 V-Carve Inlay project…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::Tutorial3dAdaptiveWaterline,
        canonical_slug: "08-3d-adaptive-waterline",
        aliases: &[],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "08-sculpted-dome.stl",
        load_status_message: "Generating Tutorial 08 3D Adaptive & Waterline Dome STL…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::Tutorial3dSurfaceFinishing,
        canonical_slug: "09-3d-surface-finishing",
        aliases: &["09-relief-wave"],
        asset_kind: SampleAssetKind::Obj,
        default_filename: "09-relief-wave.obj",
        load_status_message: "Generating Tutorial 09 3D Surface Finishing Relief Wave OBJ…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialRotaryAxialParallel,
        canonical_slug: "10-rotary-axial",
        aliases: &[],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "10-rotary-shaft.stl",
        load_status_message: "Generating Tutorial 10 Rotary Axial Parallel Shaft STL…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialIndexedStep,
        canonical_slug: "11-indexed-step",
        aliases: &["indexed-step"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "11-indexed-4side-block.step",
        load_status_message: "Generating Tutorial 11 Indexed 4-Side Block STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialRotary4AxisFlowline,
        canonical_slug: "12-simultaneous-4axis",
        aliases: &[],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "12-helical-flute-shaft.stl",
        load_status_message: "Generating Tutorial 12 Simultaneous 4-Axis Flowline STL…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: true,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialReloadV2,
        canonical_slug: "14-reload-v2",
        aliases: &["reload-v2"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "14-bracket-v2.step",
        load_status_message: "Generating Tutorial 14 Reload v2 STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialOrientStl,
        canonical_slug: "16-orient-stl",
        aliases: &["16-yup-stl", "orient-stl"],
        asset_kind: SampleAssetKind::Stl,
        default_filename: "16-yup-bracket.stl",
        load_status_message: "Generating Tutorial 16 Y-Up Bracket STL…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialOrientStep,
        canonical_slug: "16-orient-step",
        aliases: &["16-tilted-step", "orient-step"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "16-tilted-bracket.step",
        load_status_message: "Generating Tutorial 16 Tilted Bracket STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialDeburrBoss,
        canonical_slug: "21-deburr-boss",
        aliases: &["21-sloped-boss-deburr"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "21-sloped-boss.step",
        load_status_message: "Generating Tutorial 21 Sloped Boss Deburr STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
    SampleSpec {
        id: SampleId::TutorialDeburrFlange,
        canonical_slug: "21-deburr-flange",
        aliases: &["21-overhang-flange"],
        asset_kind: SampleAssetKind::Step,
        default_filename: "21-overhang-flange.step",
        load_status_message: "Generating Tutorial 21 Overhang Flange STEP…",
        focus_hint: SampleFocusHint::Default,
        prefer_tridexel_sim: false,
        show_fixtures: false,
        hide_27pt_dots: false,
        auto_generate_and_sim: false,
    },
];

/// Context passed to [`SampleId::build_project`] to construct a seeded [`ProjectDocument`].
pub struct SampleBuildContext<'a> {
    pub doc: Option<&'a CadDocument>,
    pub topo: Option<&'a TopologyGraph>,
    pub asset_path: Option<String>,
    pub sha256_hex: &'a str,
    pub available_tools: &'a [Tool],
    pub empty_project: bool,
}

/// Namespace facade over [`SAMPLE_CATALOG`] for lookup and dispatch.
pub struct SampleCatalog;

impl SampleCatalog {
    pub fn all() -> &'static [SampleSpec] {
        SAMPLE_CATALOG
    }

    pub fn spec(id: SampleId) -> &'static SampleSpec {
        SAMPLE_CATALOG
            .iter()
            .find(|s| s.id == id)
            .unwrap_or(&SAMPLE_CATALOG[0])
    }

    /// Resolves a CLI `--tutorial` or sample slug (case-insensitive) into a [`SampleId`].
    pub fn find_by_slug(raw: &str) -> Option<SampleId> {
        let key = raw.trim().to_ascii_lowercase();
        SAMPLE_CATALOG
            .iter()
            .find(|s| {
                s.canonical_slug.eq_ignore_ascii_case(&key)
                    || s.aliases.iter().any(|a| a.eq_ignore_ascii_case(&key))
            })
            .map(|s| s.id)
    }
}

impl SampleId {
    pub fn spec(self) -> &'static SampleSpec {
        SampleCatalog::spec(self)
    }

    /// Generates the raw geometry asset bytes for this sample, or `Ok(None)` for pure-2D
    /// procedural samples with no external asset file.
    pub fn generate_asset_bytes(self) -> Result<Option<Vec<u8>>, SampleStepError> {
        match self {
            Self::MachinedPlate => Ok(Some(sample_machined_plate_step()?.into_bytes())),
            Self::MultiFeatureBracket => {
                Ok(Some(sample_multi_feature_bracket_step(false)?.into_bytes()))
            }
            Self::MultiFeatureBracketTilted | Self::TutorialOrientStep => {
                Ok(Some(sample_multi_feature_bracket_step(true)?.into_bytes()))
            }
            Self::CamTortureBlock | Self::AdaptiveRestDemo => {
                Ok(Some(sample_cam_torture_block_step()?.into_bytes()))
            }
            Self::RevisionDemoV1 => Ok(Some(sample_revised_part_pair_step()?.0.into_bytes())),
            Self::TutorialReloadV2 => Ok(Some(sample_revised_part_pair_step()?.1.into_bytes())),
            Self::SculptedMeshDome | Self::Tutorial3dAdaptiveWaterline => {
                Ok(Some(sample_sculpted_dome_stl_bytes()))
            }
            Self::ReliefWaveMesh | Self::Tutorial3dSurfaceFinishing => {
                Ok(Some(sample_relief_wave_obj_text().into_bytes()))
            }
            Self::RotaryIndexedShaft | Self::TutorialRotaryAxialParallel => {
                Ok(Some(sample_rotary_shaft_stl_bytes()))
            }
            Self::RotaryHelicalFlute | Self::TutorialRotary4AxisFlowline => {
                Ok(Some(sample_helical_flute_shaft_stl_bytes()))
            }
            Self::ThreadMillManifold => Ok(Some(sample_threaded_manifold_step()?.into_bytes())),
            Self::TwoSidedViseWorkholding => Ok(Some(sample_two_sided_vise_part_stl_bytes())),
            Self::UndercutAndDeburrBlock => Ok(Some(sample_undercut_deburr_block_step_bytes()?)),
            Self::FeedOptPocket | Self::TutorialAdaptiveRest => {
                Ok(Some(tutorial_adaptive_pocket_step()?.into_bytes()))
            }
            Self::FlipJigSoftJaws | Self::TutorialContour => {
                Ok(Some(tutorial_contour_plate_step()?.into_bytes()))
            }
            Self::CurvedDeburr3D => Ok(Some(sample_curved_deburr_overhang_step_bytes()?)),
            Self::DovetailRailKeyway => Ok(Some(sample_dovetail_rail_keyway_step_bytes()?)),
            Self::Mesh3mfBracket => Ok(Some(sample_sculpted_mount_3mf_bytes())),
            Self::KeepOutClamps => Ok(Some(sample_keepout_clamp_plate_step_bytes()?)),
            Self::IndexedModelBounds => Ok(Some(sample_indexed_4sided_bar_step_bytes()?)),
            Self::SvgBadgeSign => Ok(Some(sample_svg_badge_bytes())),
            Self::Pure2dRouterPanel => Ok(Some(sample_router_panel_dxf())),
            Self::RotaryWrappedEngraving
            | Self::EngravedNameplate
            | Self::VCarveSignAndInlay
            | Self::TutorialVCarveMedial
            | Self::TutorialVCarveInlay => Ok(None),
            Self::TutorialFacing => Ok(Some(tutorial_facing_billet_step()?.into_bytes())),
            Self::TutorialPocket => Ok(Some(tutorial_pocket_block_step()?.into_bytes())),
            Self::TutorialDrillBore => Ok(Some(tutorial_drill_bore_plate_step()?.into_bytes())),
            Self::TutorialChamferTrace => {
                Ok(Some(tutorial_chamfer_engrave_block_step()?.into_bytes()))
            }
            Self::TutorialIndexedStep => {
                Ok(Some(tutorial_indexed_4side_block_step()?.into_bytes()))
            }
            Self::TutorialOrientStl => Ok(Some(tutorial_yup_bracket_stl_bytes())),
            Self::TutorialDeburrBoss => {
                Ok(Some(sample_sloped_boss_stepped_pocket_step()?.into_bytes()))
            }
            Self::TutorialDeburrFlange => Ok(Some(sample_overhang_flange_step()?.into_bytes())),
        }
    }

    /// Builds the seeded [`ProjectDocument`] for this sample using the supplied [`SampleBuildContext`].
    pub fn build_project(self, ctx: &SampleBuildContext<'_>) -> Result<ProjectDocument, String> {
        let model_path = || {
            ctx.asset_path
                .clone()
                .unwrap_or_else(|| self.spec().default_filename.to_string())
        };
        let need_doc = || {
            ctx.doc.ok_or_else(|| {
                format!(
                    "sample '{}' requires a loaded CadDocument",
                    self.spec().canonical_slug
                )
            })
        };

        let mut proj = match self {
            Self::MachinedPlate => build_cam_torture_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::MultiFeatureBracket | Self::MultiFeatureBracketTilted => {
                build_multi_feature_bracket_project(
                    need_doc()?,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                )
            }
            Self::CamTortureBlock => build_cam_torture_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::AdaptiveRestDemo => build_adaptive_rest_demo_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::RevisionDemoV1 => {
                let doc = need_doc()?;
                let owned_topo;
                let topo = match ctx.topo {
                    Some(t) => t,
                    None => {
                        owned_topo = TopologyGraph::build(doc);
                        &owned_topo
                    }
                };
                let mut p = build_revision_v1_demo_project(
                    doc,
                    topo,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project
                    && let Some(setup) = p.setups.first_mut()
                {
                    setup.operations.clear();
                }
                p
            }
            Self::SculptedMeshDome => {
                let mut p = build_sculpted_mesh_dome_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::ReliefWaveMesh => {
                let mut p = build_relief_wave_mesh_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::RotaryIndexedShaft => {
                let mut p = build_rotary_shaft_preview_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::RotaryHelicalFlute => {
                let mut p = build_rotary_helical_flute_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::ThreadMillManifold => {
                let mut p = build_thread_mill_manifold_project(
                    need_doc()?,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project
                    && let Some(setup) = p.setups.first_mut()
                {
                    setup.operations.clear();
                }
                p
            }
            Self::TwoSidedViseWorkholding => build_two_sided_vise_workholding_project(
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::UndercutAndDeburrBlock => {
                let mut p = build_undercut_deburr_block_project(
                    need_doc()?,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project
                    && let Some(setup) = p.setups.first_mut()
                {
                    setup.operations.clear();
                }
                p
            }
            Self::FeedOptPocket => {
                let mut p = build_feed_opt_pocket_project(
                    need_doc()?,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project
                    && let Some(setup) = p.setups.first_mut()
                {
                    setup.operations.clear();
                }
                p
            }
            Self::FlipJigSoftJaws => build_flip_jig_softjaws_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::CurvedDeburr3D => build_curved_deburr_3d_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::DovetailRailKeyway => build_dovetail_rail_keyway_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::Mesh3mfBracket => {
                let mut p = build_mesh_3mf_bracket_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::KeepOutClamps => build_keepout_clamps_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::IndexedModelBounds => build_indexed_model_bounds_project(
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
            ),
            Self::SvgBadgeSign => {
                let mut p =
                    build_svg_badge_sign_project(ctx.asset_path.clone(), ctx.available_tools);
                if !ctx.sha256_hex.is_empty()
                    && let Some(sk) = p.sketches.first_mut()
                    && let Some(asset) = sk.source_asset.as_mut()
                {
                    asset.sha256 = ctx.sha256_hex.to_string();
                }
                if ctx.empty_project
                    && let Some(setup) = p.setups.first_mut()
                {
                    setup.operations.clear();
                }
                p
            }
            Self::RotaryWrappedEngraving => {
                let mut p = build_rotary_wrapped_engraving_project(ctx.available_tools);
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                }
                p
            }
            Self::Pure2dRouterPanel => {
                let mut p = build_pure2d_router_panel_demo_project(
                    ctx.asset_path.clone(),
                    ctx.available_tools,
                );
                if !ctx.sha256_hex.is_empty()
                    && let Some(sk) = p.sketches.first_mut()
                    && let Some(asset) = sk.source_asset.as_mut()
                {
                    asset.sha256 = ctx.sha256_hex.to_string();
                }
                if ctx.empty_project
                    && let Some(setup) = p.setups.first_mut()
                {
                    setup.operations.clear();
                }
                p
            }
            Self::EngravedNameplate => build_engraved_nameplate_demo_project(ctx.available_tools),
            Self::VCarveSignAndInlay => {
                build_vcarve_sign_and_inlay_demo_project(ctx.available_tools)
            }
            Self::TutorialFacing => build_tutorial_facing_project(
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
                ctx.empty_project,
            ),
            Self::TutorialContour => build_tutorial_contour_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
                ctx.empty_project,
            ),
            Self::TutorialPocket => build_tutorial_pocket_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
                ctx.empty_project,
            ),
            Self::TutorialDrillBore => build_tutorial_drill_bore_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
                ctx.empty_project,
            ),
            Self::TutorialAdaptiveRest => build_tutorial_adaptive_rest_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.empty_project,
            ),
            Self::TutorialChamferTrace => build_tutorial_chamfer_trace_project(
                need_doc()?,
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
                ctx.empty_project,
            ),
            Self::TutorialVCarveMedial => {
                build_tutorial_vcarve_inlay_project(ctx.available_tools, true)
            }
            Self::TutorialVCarveInlay => {
                build_tutorial_vcarve_inlay_project(ctx.available_tools, false)
            }
            Self::Tutorial3dAdaptiveWaterline => {
                let mut p = build_tutorial_3d_adaptive_waterline_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::Tutorial3dSurfaceFinishing => {
                let mut p = build_tutorial_3d_surface_finishing_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::TutorialRotaryAxialParallel => {
                let mut p = build_tutorial_rotary_axial_parallel_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::TutorialIndexedStep => {
                let mut p = build_tutorial_indexed_step_multiface_project(
                    need_doc()?,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                }
                p
            }
            Self::TutorialRotary4AxisFlowline => {
                let mut p = build_tutorial_rotary_4axis_flowline_project(
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    p.setups[0].operations.clear();
                    p.sketches.clear();
                }
                p
            }
            Self::TutorialReloadV2 => {
                let doc = need_doc()?;
                let v1_step = sample_revised_part_pair_step()
                    .map_err(|e| e.to_string())?
                    .0;
                let v1_doc = MonstertruckKernel
                    .load_step_bytes(
                        "14-bracket-v1.step",
                        v1_step.as_bytes(),
                        DEFAULT_TESS_CHORD_TOL_MM,
                    )
                    .map_err(|e| e.to_string())?;
                let v1_topo = TopologyGraph::build(&v1_doc);
                let v1_sha = compute_sha256_hex(v1_step.as_bytes());
                let mut p = build_revision_v1_demo_project(
                    &v1_doc,
                    &v1_topo,
                    "14-bracket-v1.step".to_string(),
                    &v1_sha,
                    ctx.available_tools,
                );
                let owned_topo;
                let v2_topo = match ctx.topo {
                    Some(t) => t,
                    None => {
                        owned_topo = TopologyGraph::build(doc);
                        &owned_topo
                    }
                };
                let _ = rebind_project_to_reloaded_cad(
                    &mut p,
                    0,
                    doc,
                    v2_topo,
                    ctx.sha256_hex,
                    Some(model_path()),
                );
                if ctx.empty_project {
                    if let Some(setup) = p.setups.first_mut() {
                        setup.operations.clear();
                    }
                } else {
                    for setup in &mut p.setups {
                        for op in &mut setup.operations {
                            if matches!(op.runtime_status, OperationStatus::NeedsRepick { .. }) {
                                op.suppressed = true;
                            }
                        }
                    }
                }
                p
            }
            Self::TutorialOrientStl => build_tutorial_16_yup_stl_project(
                model_path(),
                ctx.sha256_hex,
                ctx.available_tools,
                !ctx.empty_project,
            ),
            Self::TutorialOrientStep => {
                let mut p = build_tutorial_16_tilted_step_project(
                    need_doc()?,
                    model_path(),
                    ctx.sha256_hex,
                    ctx.available_tools,
                );
                if ctx.empty_project {
                    for s in &mut p.setups {
                        s.operations.clear();
                    }
                }
                p
            }
            Self::TutorialDeburrBoss | Self::TutorialDeburrFlange => {
                let mut p = ProjectDocument::new("Tutorial 21 — 3D Edge Deburring");
                p.models = vec![ModelEntry {
                    id: "model_1".to_string(),
                    path: model_path(),
                    sha256: ctx.sha256_hex.to_string(),
                    role: Default::default(),
                    unit_override: None,
                    transform: ModelTransform::IDENTITY,
                }];
                for t in ctx.available_tools {
                    p.ensure_tool_embedded(t);
                }
                for t in chip_tools::starter_undercut_tools() {
                    p.ensure_tool_embedded(&t);
                }
                p
            }
        };

        let _ = &mut proj;
        Ok(proj)
    }
}
