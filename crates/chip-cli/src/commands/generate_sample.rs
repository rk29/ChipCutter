use std::{path::PathBuf, process::ExitCode};

use chip_cad::{
    CadKernel, MonstertruckKernel, TopologyGraph, sample_cam_torture_block_step,
    sample_helical_flute_shaft_stl_bytes, sample_machined_plate_step,
    sample_multi_feature_bracket_step, sample_overhang_flange_step, sample_relief_wave_obj_text,
    sample_revised_part_pair_step, sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes,
    sample_sculpted_mount_3mf_bytes, sample_sloped_boss_stepped_pocket_step,
    sample_svg_badge_bytes, sample_threaded_manifold_step, sample_undercut_deburr_block_step,
    tutorial_adaptive_pocket_step, tutorial_chamfer_engrave_block_step,
    tutorial_contour_plate_step, tutorial_drill_bore_plate_step, tutorial_facing_billet_step,
    tutorial_indexed_4side_block_step, tutorial_pocket_block_step, tutorial_yup_bracket_stl_bytes,
};
use chip_cam::OperationStatus;
use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
use chip_project::{
    build_cam_torture_project, build_feed_opt_pocket_project, build_mesh_3mf_bracket_project,
    build_pure2d_router_panel_demo_project, build_revision_v1_demo_project,
    build_rotary_helical_flute_project, build_rotary_shaft_preview_project,
    build_rotary_wrapped_engraving_project, build_sculpted_mesh_dome_project,
    build_svg_badge_sign_project, build_thread_mill_manifold_project,
    build_tutorial_3d_adaptive_waterline_project, build_tutorial_3d_surface_finishing_project,
    build_tutorial_16_tilted_step_project, build_tutorial_16_yup_stl_project,
    build_tutorial_adaptive_rest_project, build_tutorial_chamfer_trace_project,
    build_tutorial_contour_project, build_tutorial_drill_bore_project,
    build_tutorial_facing_project, build_tutorial_indexed_step_multiface_project,
    build_tutorial_pocket_project, build_tutorial_rotary_4axis_flowline_project,
    build_tutorial_rotary_axial_parallel_project, build_tutorial_vcarve_inlay_project,
    build_undercut_deburr_block_project, compute_sha256_hex, portable_model_path,
    rebind_project_to_reloaded_cad,
};
use chip_sketch::sample_router_panel_dxf;
use chip_tools::{default_starter_tool_library, resolve_machine_definition_cli_override};
use clap::Args;

#[derive(Debug, Args)]
pub struct GenerateSampleArgs {
    /// Output `.step` file path.
    pub output: PathBuf,
    /// Generate the Milestone 2 multi-feature bracket (with blind pocket/hole + 4 mounting holes).
    #[arg(long)]
    pub multi_feature: bool,
    /// Tilt the generated multi-feature bracket by 30° X / 25° Z to test WCS alignment.
    #[arg(long)]
    pub tilted: bool,
    /// Generate the Milestone 3 CAM torture block (island pocket, open step, wide bore, 45° bevel).
    #[arg(long)]
    pub cam_torture: bool,
    /// Generate the Phase 18 Undercut & 3D Deburr block (M8 T-Slot, 60° Dovetail, Lollipop overhang groove, & 3D deburr rims).
    #[arg(long)]
    pub undercut_block: bool,
    /// Generate v1 of the Milestone 5 associative reload revision pair.
    #[arg(long)]
    pub revision_v1: bool,
    /// Generate v2 of the Milestone 5 associative reload revision pair (resized pocket, moved/deleted holes).
    #[arg(long)]
    pub revision_v2: bool,
    /// Generate a tutorial starter (`01-facing`, `02-contour`, `03-pocket`, `04-drill-bore`, `05-adaptive-rest`, `06-chamfer-trace`, `07-vcarve-medial`, `07-vcarve-inlay`, `13-pure2d-dxf`, `14-reload-v1`, `14-reload-v2`, `16-orient-stl`, `16-orient-step`, `18-thread-milling`, etc.).
    #[arg(long)]
    pub tutorial: Option<String>,
    /// Optional path to also write a seeded `.chip.toml` CAM project referencing `output`.
    #[arg(long)]
    pub project_out: Option<PathBuf>,
    /// When combined with `--tutorial` and `--project-out`, write an empty 0-operation starter project (Stock & WCS only).
    #[arg(long)]
    pub empty_project: bool,
    /// Optional target CNC machine definition ID (e.g. `generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`) to record on the generated project.
    #[arg(long)]
    pub machine: Option<String>,
}

pub fn run_generate_sample_step(
    args: GenerateSampleArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let explicit_machine = match args.machine.as_deref() {
        Some(mid) => Some(resolve_machine_definition_cli_override(mid, None)?),
        None => None,
    };
    let tutorial_slug = args
        .tutorial
        .as_deref()
        .map(|s| s.trim().to_ascii_lowercase());

    if matches!(
        tutorial_slug.as_deref(),
        Some("07-vcarve-medial" | "07-vcarve-inlay" | "vcarve-medial" | "vcarve-inlay")
    ) {
        let medial_only = matches!(
            tutorial_slug.as_deref(),
            Some("07-vcarve-medial" | "vcarve-medial")
        );
        let starter = default_starter_tool_library();
        let mut proj = build_tutorial_vcarve_inlay_project(&starter.tools, medial_only);
        if let Some(m) = explicit_machine.as_ref() {
            proj.apply_machine_defaults(m, false);
        }
        let dest = args.project_out.unwrap_or(args.output);
        if let Some(parent) = dest.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        proj.save_to_path(&dest)?;
        println!(
            "Wrote Pure-2D V-Carve tutorial project ({} setup(s)) to {}",
            proj.setups.len(),
            dest.display()
        );
        return Ok(ExitCode::SUCCESS);
    }

    // Phase 5 Mesh & 4th-Axis Rotary tutorial assets (`08`–`12`)
    if let Some(slug) = tutorial_slug.as_deref()
        && matches!(
            slug,
            "08-mesh-surfacing"
                | "08-sculpted-dome"
                | "08-3d-adaptive-waterline"
                | "08-bracket-3mf"
                | "09-3d-surface-finishing"
                | "09-relief-wave"
                | "09-rotary-shaft"
                | "09-rotary-wrap"
                | "10-rotary-wrap"
                | "10-rotary-axial"
                | "10-simultaneous-4axis"
                | "10-helical-flute"
                | "11-indexed-shaft"
                | "12-simultaneous-4axis"
        )
    {
        let starter = default_starter_tool_library();
        if matches!(slug, "09-rotary-wrap" | "10-rotary-wrap") {
            let mut proj = build_rotary_wrapped_engraving_project(&starter.tools);
            if args.empty_project {
                proj.setups[0].operations.clear();
            }
            if let Some(m) = explicit_machine.as_ref() {
                proj.apply_machine_defaults(m, false);
            }
            let dest = args.project_out.unwrap_or(args.output);
            if let Some(parent) = dest.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            proj.save_to_path(&dest)?;
            println!(
                "Wrote Wrapped Rotary Engraving tutorial project ({} ops) to {}",
                proj.setups[0].operations.len(),
                dest.display()
            );
            return Ok(ExitCode::SUCCESS);
        }

        let (mesh_bytes, fmt_label) = match slug {
            "08-mesh-surfacing" | "08-sculpted-dome" | "08-3d-adaptive-waterline" => {
                (sample_sculpted_dome_stl_bytes(), "STL")
            }
            "08-bracket-3mf" => (sample_sculpted_mount_3mf_bytes(), "3MF"),
            "09-3d-surface-finishing" | "09-relief-wave" => {
                (sample_relief_wave_obj_text().into_bytes(), "OBJ")
            }
            "09-rotary-shaft" | "10-rotary-axial" | "11-indexed-shaft" => {
                (sample_rotary_shaft_stl_bytes(), "STL")
            }
            _ => (sample_helical_flute_shaft_stl_bytes(), "STL"),
        };
        if let Some(parent) = args.output.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&args.output, &mesh_bytes)?;
        println!(
            "Wrote Phase 5 tutorial {fmt_label} mesh ({} bytes) to {}",
            mesh_bytes.len(),
            args.output.display()
        );
        if let Some(proj_path) = args.project_out {
            let sha = compute_sha256_hex(&mesh_bytes);
            let rel = portable_model_path(Some(&proj_path), &args.output);
            let mut proj = match slug {
                "08-3d-adaptive-waterline" => {
                    build_tutorial_3d_adaptive_waterline_project(rel, &sha, &starter.tools)
                }
                "09-3d-surface-finishing" | "09-relief-wave" => {
                    build_tutorial_3d_surface_finishing_project(rel, &sha, &starter.tools)
                }
                "08-mesh-surfacing" | "08-sculpted-dome" => {
                    build_sculpted_mesh_dome_project(rel, &sha, &starter.tools)
                }
                "08-bracket-3mf" => build_mesh_3mf_bracket_project(rel, &sha, &starter.tools),
                "10-rotary-axial" => {
                    build_tutorial_rotary_axial_parallel_project(rel, &sha, &starter.tools)
                }
                "09-rotary-shaft" | "11-indexed-shaft" => {
                    build_rotary_shaft_preview_project(rel, &sha, &starter.tools)
                }
                "12-simultaneous-4axis" => {
                    build_tutorial_rotary_4axis_flowline_project(rel, &sha, &starter.tools)
                }
                _ => build_rotary_helical_flute_project(rel, &sha, &starter.tools),
            };
            if args.empty_project {
                proj.setups[0].operations.clear();
                proj.sketches.clear();
            }
            if let Some(m) = explicit_machine.as_ref() {
                proj.apply_machine_defaults(m, false);
            }
            if let Some(parent) = proj_path.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            proj.save_to_path(&proj_path)?;
            println!(
                "Wrote Phase 5 tutorial project ({} ops) to {}",
                proj.setups[0].operations.len(),
                proj_path.display()
            );
        }
        return Ok(ExitCode::SUCCESS);
    }

    // Phase 6 Production Workflow tutorial assets (`13`–`14`)
    if let Some(slug) = tutorial_slug.as_deref()
        && matches!(
            slug,
            "13-pure2d-dxf" | "13-pure2d-dxf-svg-workflow" | "pure2d-dxf"
        )
    {
        let dxf_bytes = sample_router_panel_dxf();
        if let Some(parent) = args.output.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&args.output, &dxf_bytes)?;
        println!(
            "Wrote Pure-2D tutorial DXF drawing ({} bytes) to {}",
            dxf_bytes.len(),
            args.output.display()
        );
        if let Some(proj_path) = args.project_out {
            let starter = default_starter_tool_library();
            let rel = portable_model_path(Some(&proj_path), &args.output);
            let mut proj = build_pure2d_router_panel_demo_project(Some(rel), &starter.tools);
            if let Some(sk) = proj.sketches.first_mut()
                && let Some(asset) = sk.source_asset.as_mut()
            {
                asset.sha256 = compute_sha256_hex(&dxf_bytes);
            }
            if args.empty_project
                && let Some(setup) = proj.setups.first_mut()
            {
                setup.operations.clear();
            }
            if let Some(m) = explicit_machine.as_ref() {
                proj.apply_machine_defaults(m, false);
            }
            if let Some(parent) = proj_path.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            proj.save_to_path(&proj_path)?;
            println!(
                "Wrote Pure-2D tutorial project ({} ops) to {}",
                proj.setups.first().map_or(0, |s| s.operations.len()),
                proj_path.display()
            );
        }
        return Ok(ExitCode::SUCCESS);
    }

    // Pure-2D SVG tutorial starter asset (`13-pure2d-svg` -> `13-badge-logo.svg`)
    if let Some(slug) = tutorial_slug.as_deref()
        && matches!(slug, "13-pure2d-svg" | "13-badge-logo" | "pure2d-svg")
    {
        let svg_bytes = sample_svg_badge_bytes();
        if let Some(parent) = args.output.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&args.output, &svg_bytes)?;
        println!(
            "Wrote Pure-2D tutorial SVG artwork ({} bytes) to {}",
            svg_bytes.len(),
            args.output.display()
        );
        if let Some(proj_path) = args.project_out {
            let starter = default_starter_tool_library();
            let rel = portable_model_path(Some(&proj_path), &args.output);
            let mut proj = build_svg_badge_sign_project(Some(rel), &starter.tools);
            if let Some(sk) = proj.sketches.first_mut()
                && let Some(asset) = sk.source_asset.as_mut()
            {
                asset.sha256 = compute_sha256_hex(&svg_bytes);
            }
            if args.empty_project
                && let Some(setup) = proj.setups.first_mut()
            {
                setup.operations.clear();
            }
            if let Some(m) = explicit_machine.as_ref() {
                proj.apply_machine_defaults(m, false);
            }
            if let Some(parent) = proj_path.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            proj.save_to_path(&proj_path)?;
            println!(
                "Wrote Pure-2D SVG tutorial project ({} ops) to {}",
                proj.setups.first().map_or(0, |s| s.operations.len()),
                proj_path.display()
            );
        }
        return Ok(ExitCode::SUCCESS);
    }

    // Phase 10 Tutorial 16 Y-Up STL starter asset (`16-orient-stl`)
    if let Some(slug) = tutorial_slug.as_deref()
        && matches!(slug, "16-orient-stl" | "16-yup-stl" | "orient-stl")
    {
        let stl_bytes = tutorial_yup_bracket_stl_bytes();
        if let Some(parent) = args.output.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&args.output, &stl_bytes)?;
        println!(
            "Wrote Tutorial 16 Y-up STL mesh ({} bytes) to {}",
            stl_bytes.len(),
            args.output.display()
        );
        if let Some(proj_path) = args.project_out {
            let starter = default_starter_tool_library();
            let sha = compute_sha256_hex(&stl_bytes);
            let rel = portable_model_path(Some(&proj_path), &args.output);
            let mut proj =
                build_tutorial_16_yup_stl_project(rel, &sha, &starter.tools, !args.empty_project);
            if let Some(m) = explicit_machine.as_ref() {
                proj.apply_machine_defaults(m, false);
            }
            if let Some(parent) = proj_path.parent()
                && !parent.as_os_str().is_empty()
            {
                std::fs::create_dir_all(parent)?;
            }
            proj.save_to_path(&proj_path)?;
            println!(
                "Wrote Tutorial 16 STL project ({} ops) to {}",
                proj.setups[0].operations.len(),
                proj_path.display()
            );
        }
        return Ok(ExitCode::SUCCESS);
    }

    let step = if let Some(slug) = tutorial_slug.as_deref() {
        match slug {
            "01-facing" | "facing" => tutorial_facing_billet_step()?,
            "02-contour" | "02-2d-contour-tabs" | "contour" => tutorial_contour_plate_step()?,
            "03-pocket" | "03-2d-pocket" | "pocket" => tutorial_pocket_block_step()?,
            "04-drill-bore" | "04-drilling-helical-bore" | "drill-bore" => {
                tutorial_drill_bore_plate_step()?
            }
            "05-adaptive-rest" | "05-adaptive-rest-machining" | "adaptive-rest" => {
                tutorial_adaptive_pocket_step()?
            }
            "06-chamfer-trace" | "06-chamfer-curve-trace" | "chamfer-trace" => {
                tutorial_chamfer_engrave_block_step()?
            }
            "11-indexed-step" | "indexed-step" => tutorial_indexed_4side_block_step()?,
            "14-reload-v1" | "14-associative-step-reload" | "reload-v1" => {
                sample_revised_part_pair_step()?.0
            }
            "14-reload-v2" | "reload-v2" => sample_revised_part_pair_step()?.1,
            "16-orient-step" | "16-tilted-step" | "orient-step" => {
                sample_multi_feature_bracket_step(true)?
            }
            "18-thread-milling" | "18-threaded-manifold" | "thread-mill" | "thread-milling" => {
                sample_threaded_manifold_step()?
            }
            "20-feed-opt" | "20-feed-opt-pocket" | "feed-opt" => tutorial_adaptive_pocket_step()?,
            "21-deburr-boss" | "21-sloped-boss-deburr" => sample_sloped_boss_stepped_pocket_step()?,
            "21-deburr-flange" | "21-overhang-flange" => sample_overhang_flange_step()?,
            "22-undercut-block" | "22-undercut-deburr-block" => {
                sample_undercut_deburr_block_step()?
            }
            other => {
                return Err(format!(
                    "unknown --tutorial '{other}'; expected one of: 01-facing, 02-contour, 03-pocket, 04-drill-bore, 05-adaptive-rest, 06-chamfer-trace, 07-vcarve-medial, 07-vcarve-inlay, 08-3d-adaptive-waterline, 08-mesh-surfacing, 08-bracket-3mf, 09-3d-surface-finishing, 09-rotary-shaft, 09-rotary-wrap, 10-rotary-wrap, 10-rotary-axial, 10-simultaneous-4axis, 11-indexed-step, 11-indexed-shaft, 12-simultaneous-4axis, 13-pure2d-dxf, 13-pure2d-svg, 14-reload-v1, 14-reload-v2, 16-orient-stl, 16-orient-step, 18-thread-milling, 20-feed-opt, 21-deburr-boss, 21-deburr-flange, 22-undercut-block"
                )
                .into());
            }
        }
    } else if args.revision_v1 {
        sample_revised_part_pair_step()?.0
    } else if args.revision_v2 {
        sample_revised_part_pair_step()?.1
    } else if args.undercut_block {
        sample_undercut_deburr_block_step()?
    } else if args.cam_torture {
        sample_cam_torture_block_step()?
    } else if args.multi_feature || args.tilted {
        sample_multi_feature_bracket_step(args.tilted)?
    } else {
        sample_machined_plate_step()?
    };

    if let Some(parent) = args.output.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&args.output, &step)?;
    println!(
        "Wrote sample machined STEP part to {}",
        args.output.display()
    );

    if let Some(proj_path) = args.project_out {
        let doc = MonstertruckKernel.load_step_bytes(
            "sample.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )?;
        let sha = compute_sha256_hex(step.as_bytes());
        let starter = default_starter_tool_library();
        let rel_path = portable_model_path(Some(&proj_path), &args.output);
        let mut proj = match tutorial_slug.as_deref() {
            Some("01-facing" | "facing") => {
                build_tutorial_facing_project(rel_path, &sha, &starter.tools, args.empty_project)
            }
            Some("02-contour" | "02-2d-contour-tabs" | "contour") => {
                build_tutorial_contour_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                    args.empty_project,
                )
            }
            Some("03-pocket" | "03-2d-pocket" | "pocket") => build_tutorial_pocket_project(
                &doc,
                rel_path,
                &sha,
                &starter.tools,
                args.empty_project,
            ),
            Some("04-drill-bore" | "04-drilling-helical-bore" | "drill-bore") => {
                build_tutorial_drill_bore_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                    args.empty_project,
                )
            }
            Some("05-adaptive-rest" | "05-adaptive-rest-machining" | "adaptive-rest") => {
                build_tutorial_adaptive_rest_project(&doc, rel_path, &sha, args.empty_project)
            }
            Some("06-chamfer-trace" | "06-chamfer-curve-trace" | "chamfer-trace") => {
                build_tutorial_chamfer_trace_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                    args.empty_project,
                )
            }
            Some("11-indexed-step" | "indexed-step") => {
                let mut p = build_tutorial_indexed_step_multiface_project(
                    &doc,
                    rel_path,
                    &sha,
                    &starter.tools,
                );
                if args.empty_project {
                    p.setups[0].operations.clear();
                }
                p
            }
            Some("14-reload-v1" | "14-associative-step-reload" | "reload-v1") => {
                let topo = TopologyGraph::build(&doc);
                let mut proj =
                    build_revision_v1_demo_project(&doc, &topo, rel_path, &sha, &starter.tools);
                if args.empty_project
                    && let Some(setup) = proj.setups.first_mut()
                {
                    setup.operations.clear();
                }
                proj
            }
            Some("16-orient-step" | "16-tilted-step" | "orient-step") => {
                let mut proj =
                    build_tutorial_16_tilted_step_project(&doc, rel_path, &sha, &starter.tools);
                if args.empty_project {
                    for s in &mut proj.setups {
                        s.operations.clear();
                    }
                }
                proj
            }
            Some("14-reload-v2" | "reload-v2") => {
                let v1_step = sample_revised_part_pair_step()?.0;
                let v1_doc = MonstertruckKernel.load_step_bytes(
                    "14-bracket-v1.step",
                    v1_step.as_bytes(),
                    DEFAULT_TESS_CHORD_TOL_MM,
                )?;
                let v1_topo = TopologyGraph::build(&v1_doc);
                let v1_sha = compute_sha256_hex(v1_step.as_bytes());
                let mut proj = build_revision_v1_demo_project(
                    &v1_doc,
                    &v1_topo,
                    "14-bracket-v1.step".to_string(),
                    &v1_sha,
                    &starter.tools,
                );
                let v2_topo = TopologyGraph::build(&doc);
                let _ = rebind_project_to_reloaded_cad(
                    &mut proj,
                    0,
                    &doc,
                    &v2_topo,
                    &sha,
                    Some(rel_path),
                );
                if args.empty_project {
                    if let Some(setup) = proj.setups.first_mut() {
                        setup.operations.clear();
                    }
                } else {
                    for setup in &mut proj.setups {
                        for op in &mut setup.operations {
                            if matches!(op.runtime_status, OperationStatus::NeedsRepick { .. }) {
                                op.suppressed = true;
                            }
                        }
                    }
                }
                proj
            }
            Some(
                "18-thread-milling" | "18-threaded-manifold" | "thread-mill" | "thread-milling",
            ) => {
                let mut proj =
                    build_thread_mill_manifold_project(&doc, rel_path, &sha, &starter.tools);
                if args.empty_project
                    && let Some(setup) = proj.setups.first_mut()
                {
                    setup.operations.clear();
                }
                proj
            }
            Some("20-feed-opt" | "20-feed-opt-pocket" | "feed-opt") => {
                let mut proj = build_feed_opt_pocket_project(&doc, rel_path, &sha, &starter.tools);
                if args.empty_project
                    && let Some(setup) = proj.setups.first_mut()
                {
                    setup.operations.clear();
                }
                proj
            }
            Some("22-undercut-block" | "22-undercut-deburr-block") => {
                let mut proj =
                    build_undercut_deburr_block_project(&doc, rel_path, &sha, &starter.tools);
                if args.empty_project
                    && let Some(setup) = proj.setups.first_mut()
                {
                    setup.operations.clear();
                }
                proj
            }
            Some(
                "21-deburr-boss"
                | "21-sloped-boss-deburr"
                | "21-deburr-flange"
                | "21-overhang-flange",
            ) => {
                let mut proj =
                    chip_project::ProjectDocument::new("Tutorial 21 — 3D Edge Deburring");
                proj.models = vec![chip_project::ModelEntry {
                    id: "model_1".to_string(),
                    path: rel_path,
                    sha256: sha,
                    role: Default::default(),
                    unit_override: None,
                    transform: chip_cad::ModelTransform::IDENTITY,
                }];
                for t in &starter.tools {
                    proj.ensure_tool_embedded(t);
                }
                for t in chip_tools::starter_undercut_tools() {
                    proj.ensure_tool_embedded(&t);
                }
                proj
            }
            _ if args.undercut_block => {
                let mut proj =
                    build_undercut_deburr_block_project(&doc, rel_path, &sha, &starter.tools);
                if args.empty_project
                    && let Some(setup) = proj.setups.first_mut()
                {
                    setup.operations.clear();
                }
                proj
            }
            _ => build_cam_torture_project(&doc, rel_path, &sha, &starter.tools),
        };
        if let Some(m) = explicit_machine.as_ref() {
            proj.apply_machine_defaults(m, false);
        }
        if let Some(parent) = proj_path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        proj.save_to_path(&proj_path)?;
        println!(
            "Wrote seeded CAM project ({} operations) to {}",
            proj.setups[0].operations.len(),
            proj_path.display()
        );
    }

    Ok(ExitCode::SUCCESS)
}
