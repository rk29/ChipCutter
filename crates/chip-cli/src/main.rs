mod commands;

use std::{path::PathBuf, process::ExitCode};

use clap::{Parser, Subcommand};

use crate::commands::{
    bundle_reload_tools::{run_bundle_pack, run_bundle_unpack, run_import_fusion, run_reload_step},
    inspect_sample::{GenerateSampleArgs, run_generate_sample_step, run_inspect},
    validate_sim_build::{BuildArgs, run_build, run_simulate, run_validate},
};

#[derive(Debug, Parser)]
#[command(
    name = "chipcutter",
    version,
    about = "ChipCutter headless CAM CLI (STEP inspection, validation, simulation, G-code build, & tool import)"
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Inspect an ISO 10303-21 STEP file, reporting normalized dimensions and B-Rep features.
    Inspect {
        /// Path to `.step` or `.stp` CAD model.
        path: PathBuf,
        /// Display unit for dimensions (`mm` or `in`).
        #[arg(long, default_value = "mm")]
        units: String,
    },
    /// Write ChipCutter's programmatic machined sample STEP file (and optional `.chip.toml` job) to disk.
    GenerateSampleStep {
        /// Output `.step` file path.
        output: PathBuf,
        /// Generate the Milestone 2 multi-feature bracket (with blind pocket/hole + 4 mounting holes).
        #[arg(long)]
        multi_feature: bool,
        /// Tilt the generated multi-feature bracket by 30° X / 25° Z to test WCS alignment.
        #[arg(long)]
        tilted: bool,
        /// Generate the Milestone 3 CAM torture block (island pocket, open step, wide bore, 45° bevel).
        #[arg(long)]
        cam_torture: bool,
        /// Generate v1 of the Milestone 5 associative reload revision pair.
        #[arg(long)]
        revision_v1: bool,
        /// Generate v2 of the Milestone 5 associative reload revision pair (resized pocket, moved/deleted holes).
        #[arg(long)]
        revision_v2: bool,
        /// Optional path to also write a seeded `.chip.toml` CAM project referencing `output`.
        #[arg(long)]
        project_out: Option<PathBuf>,
    },
    /// Validate a `.chip.toml` project's STEP hashes, tools, Setups, WCS, selections, and Z heights.
    Validate {
        /// Path to `.chip.toml` project file (or `.chipz` bundle).
        project: PathBuf,
        /// Also generate all enabled 2.5D toolpaths in parallel and verify non-empty CL-data output.
        #[arg(long)]
        generate: bool,
    },
    /// Run headless 2.5D Z-dexel heightmap stock removal & 5-rule crash/gouge verification (`06-simulation.md`).
    Simulate {
        /// Path to `.chip.toml` project file (or `.chipz` bundle).
        project: PathBuf,
        /// Heightmap resolution (`standard` = 512, `fine` = 1024, `ultra` = 2048).
        #[arg(long, default_value = "standard")]
        resolution: String,
        /// Optional path to write structured JSON simulation report.
        #[arg(long)]
        report: Option<PathBuf>,
    },
    /// Generate toolpaths in parallel, verify zero critical crashes/gouges, and post-process G-code (`07-post-processor.md`).
    Build {
        /// Path to `.chip.toml` project file (or `.chipz` bundle).
        project: PathBuf,
        /// Output directory for emitted `.nc`/`.ngc` G-code files and `build_summary.json`.
        #[arg(short, long, default_value = "out")]
        output_dir: PathBuf,
        /// Optional filter to build only a specific Setup (by Setup name or ID).
        #[arg(long)]
        setup: Option<String>,
        /// Optional post-processor profile ID (`grbl`, `grblhal_atc`, `linuxcnc`) or `.post.toml` file path override.
        #[arg(long)]
        post: Option<String>,
        /// Allow non-critical simulation warnings (critical crashes/gouges still block build!).
        #[arg(long)]
        allow_warnings: bool,
    },
    /// Reload an updated STEP model into a `.chip.toml` project and re-bind selections via `GeometricSignature`.
    ReloadStep {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Path to revised `.step` / `.stp` model (defaults to re-reading the project's existing model path if omitted).
        #[arg(long)]
        step: Option<PathBuf>,
        /// Optional output `.chip.toml` path (updates `project` in-place if omitted).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Exit 0 even if one or more deleted/ambiguous features require manual re-pick.
        #[arg(long)]
        allow_needs_repick: bool,
    },
    /// Portable `.chipz` archive utilities (`pack` / `unpack`, `09-persistence-cli.md` §2).
    Bundle {
        #[command(subcommand)]
        action: BundleCommands,
    },
    /// Tool library utilities (`import-fusion`).
    Tools {
        #[command(subcommand)]
        action: ToolCommands,
    },
}

#[derive(Debug, Subcommand)]
enum BundleCommands {
    /// Pack a `.chip.toml` project, its STEP models, and custom `.post.toml` files into a portable `.chipz` archive.
    Pack {
        /// Input `.chip.toml` project file.
        project: PathBuf,
        /// Destination `.chipz` bundle file.
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Unpack a `.chipz` bundle archive into a directory on disk.
    Unpack {
        /// Input `.chipz` bundle file.
        archive: PathBuf,
        /// Destination directory to extract `project.chip.toml`, `models/`, and `posts/`.
        #[arg(short, long)]
        output_dir: PathBuf,
    },
}

#[derive(Debug, Subcommand)]
enum ToolCommands {
    /// Convert a Fusion 360 `.json` or `.tools` (ZIP) library into a native `.tools.toml` file.
    ImportFusion {
        /// Input Fusion 360 `.json` or `.tools` file.
        input: PathBuf,
        /// Output `.tools.toml` destination path.
        #[arg(short, long)]
        output: PathBuf,
        /// Optional custom library name.
        #[arg(long)]
        name: Option<String>,
    },
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(err) => {
            eprintln!("error: {err}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    match cli.command {
        Commands::Inspect { path, units } => run_inspect(path, units),
        Commands::GenerateSampleStep {
            output,
            multi_feature,
            tilted,
            cam_torture,
            revision_v1,
            revision_v2,
            project_out,
        } => run_generate_sample_step(GenerateSampleArgs {
            output,
            multi_feature,
            tilted,
            cam_torture,
            revision_v1,
            revision_v2,
            project_out,
        }),
        Commands::Validate { project, generate } => run_validate(project, generate),
        Commands::Simulate {
            project,
            resolution,
            report,
        } => run_simulate(project, resolution, report),
        Commands::Build {
            project,
            output_dir,
            setup,
            post,
            allow_warnings,
        } => run_build(BuildArgs {
            project,
            output_dir,
            setup_filter: setup,
            post_override: post,
            allow_warnings,
        }),
        Commands::ReloadStep {
            project,
            step,
            output,
            allow_needs_repick,
        } => run_reload_step(project, step, output, allow_needs_repick),
        Commands::Bundle { action } => match action {
            BundleCommands::Pack { project, output } => run_bundle_pack(project, output),
            BundleCommands::Unpack {
                archive,
                output_dir,
            } => run_bundle_unpack(archive, output_dir),
        },
        Commands::Tools { action } => match action {
            ToolCommands::ImportFusion {
                input,
                output,
                name,
            } => run_import_fusion(input, output, name),
        },
    }
}
