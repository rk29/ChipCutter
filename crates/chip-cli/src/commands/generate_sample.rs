use std::{path::PathBuf, process::ExitCode};

use chip_cad::{CadKernel, MonstertruckKernel};
use chip_demos::{SampleAssetKind, SampleBuildContext, SampleCatalog, SampleId};
use chip_machines::resolve_machine_definition_cli_override;
use chip_math::DEFAULT_TESS_CHORD_TOL_MM;
use chip_project::{compute_sha256_hex, portable_model_path};
use chip_tools::default_starter_tool_library;
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

fn ensure_parent_dir(path: &std::path::Path) -> std::io::Result<()> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent)?;
    }
    Ok(())
}

pub fn run_generate_sample_step(
    args: GenerateSampleArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let explicit_machine = match args.machine.as_deref() {
        Some(mid) => Some(resolve_machine_definition_cli_override(mid, None)?),
        None => None,
    };

    let sample_id = if let Some(slug) = args.tutorial.as_deref() {
        SampleCatalog::find_by_slug(slug).ok_or_else(|| {
            format!(
                "unknown --tutorial '{slug}'; expected one of: 01-facing, 02-contour, 03-pocket, 04-drill-bore, 05-adaptive-rest, 06-chamfer-trace, 07-vcarve-medial, 07-vcarve-inlay, 08-3d-adaptive-waterline, 08-mesh-surfacing, 08-bracket-3mf, 09-3d-surface-finishing, 09-rotary-shaft, 09-rotary-wrap, 10-rotary-wrap, 10-rotary-axial, 10-simultaneous-4axis, 11-indexed-step, 11-indexed-shaft, 12-simultaneous-4axis, 13-pure2d-dxf, 13-pure2d-svg, 14-reload-v1, 14-reload-v2, 16-orient-stl, 16-orient-step, 18-thread-milling, 20-feed-opt, 21-deburr-boss, 21-deburr-flange, 22-undercut-block"
            )
        })?
    } else if args.revision_v1 {
        SampleId::RevisionDemoV1
    } else if args.revision_v2 {
        SampleId::TutorialReloadV2
    } else if args.undercut_block {
        SampleId::UndercutAndDeburrBlock
    } else if args.cam_torture {
        SampleId::CamTortureBlock
    } else if args.tilted {
        SampleId::MultiFeatureBracketTilted
    } else if args.multi_feature {
        SampleId::MultiFeatureBracket
    } else {
        SampleId::MachinedPlate
    };

    let spec = sample_id.spec();
    let starter = default_starter_tool_library();

    // Pure-2D / procedural samples with no external geometry file write the project directly.
    let Some(asset_bytes) = sample_id.generate_asset_bytes()? else {
        let mut proj = sample_id.build_project(&SampleBuildContext {
            doc: None,
            topo: None,
            asset_path: None,
            sha256_hex: "",
            available_tools: &starter.tools,
            empty_project: args.empty_project,
        })?;
        if let Some(m) = explicit_machine.as_ref() {
            proj.apply_machine_defaults(m, false);
        }
        let dest = args.project_out.unwrap_or(args.output);
        ensure_parent_dir(&dest)?;
        proj.save_to_path(&dest)?;
        println!(
            "Wrote {} project ({} setup(s)) to {}",
            spec.canonical_slug,
            proj.setups.len(),
            dest.display()
        );
        return Ok(ExitCode::SUCCESS);
    };

    ensure_parent_dir(&args.output)?;
    std::fs::write(&args.output, &asset_bytes)?;
    println!(
        "Wrote sample {} asset ({} bytes) to {}",
        spec.asset_kind.format_label(),
        asset_bytes.len(),
        args.output.display()
    );

    if let Some(proj_path) = args.project_out {
        let sha = compute_sha256_hex(&asset_bytes);
        let rel = portable_model_path(Some(&proj_path), &args.output);
        let loaded_doc = if spec.asset_kind == SampleAssetKind::Step {
            Some(MonstertruckKernel.load_step_bytes(
                "sample.step",
                &asset_bytes,
                DEFAULT_TESS_CHORD_TOL_MM,
            )?)
        } else {
            None
        };

        // Preserve historical CLI behavior: non-tutorial `--multi-feature` / `--revision-v1` /
        // `--revision-v2` / default plate with `--project-out` uses CamTortureBlock unless
        // `--undercut-block` or `--tutorial` was specified.
        let project_sample_id = if args.tutorial.is_some() || args.undercut_block {
            sample_id
        } else {
            SampleId::CamTortureBlock
        };

        let mut proj = project_sample_id.build_project(&SampleBuildContext {
            doc: loaded_doc.as_ref(),
            topo: None,
            asset_path: Some(rel),
            sha256_hex: &sha,
            available_tools: &starter.tools,
            empty_project: args.empty_project,
        })?;
        if let Some(m) = explicit_machine.as_ref() {
            proj.apply_machine_defaults(m, false);
        }
        ensure_parent_dir(&proj_path)?;
        proj.save_to_path(&proj_path)?;
        println!(
            "Wrote seeded CAM project ({} operations) to {}",
            proj.setups.first().map_or(0, |s| s.operations.len()),
            proj_path.display()
        );
    }

    Ok(ExitCode::SUCCESS)
}
