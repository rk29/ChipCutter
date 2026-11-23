mod commands;

use std::{path::PathBuf, process::ExitCode};

use clap::{Args, Parser, Subcommand};

use crate::commands::{
    bundle_reload_tools::{run_bundle_pack, run_bundle_unpack, run_import_fusion, run_reload_step},
    generate_sample::{GenerateSampleArgs, run_generate_sample_step},
    import_2d::{Import2dArgs, run_import_2d},
    import_mesh::{ImportMeshArgs, run_import_mesh},
    inspect::{InspectArgs, run_inspect},
    machines::{
        run_machines_clone, run_machines_export, run_machines_import, run_machines_inspect,
        run_machines_list,
    },
    transform_model::{TransformModelArgs, run_transform_model},
    validate_sim_build::{BuildArgs, SimulateArgs, run_build, run_simulate, run_validate},
    workholding::{
        AddClampArgs, AddSpoilboardArgs, AddViseArgs, FlipJigArgs, SoftJawsArgs,
        run_workholding_add_clamp, run_workholding_add_spoilboard, run_workholding_add_vise,
        run_workholding_flip_jig, run_workholding_list, run_workholding_soft_jaws,
    },
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

#[derive(Debug, Args)]
struct ValidateArgs {
    /// Path to `.chip.toml` project file (or `.chipz` bundle).
    project: PathBuf,
    /// Also generate all enabled 2.5D toolpaths in parallel and verify non-empty CL-data output.
    #[arg(long)]
    generate: bool,
    /// Optional target machine definition ID or `.machine.toml` path override for validation.
    #[arg(long)]
    machine: Option<String>,
}

#[derive(Debug, Args)]
struct ReloadStepArgs {
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
}

#[derive(Debug, Args)]
struct WorkholdingListArgs {
    /// Path to `.chip.toml` project file or `.chipz` bundle.
    project: PathBuf,
    /// Optional Setup filter (0-based index, Setup ID, or Setup name).
    #[arg(long)]
    setup: Option<String>,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Inspect a 3D STEP (`.step`/`.stp`), Triangle Mesh (`.stl`/`.obj`/`.3mf`), or 2D Vector (`.dxf`/`.svg`) file.
    Inspect(Box<InspectArgs>),
    /// Import a 3D triangle mesh (`.stl`, `.obj`, or `.3mf`) into a new `schema_version = 3` `.chip.toml` project.
    #[command(name = "import-mesh")]
    ImportMesh(Box<ImportMeshArgs>),
    /// Non-destructively rotate, scale, or translate the primary 3D model (STEP or STL/OBJ/3MF) in a `.chip.toml` project.
    #[command(name = "transform-model")]
    TransformModel(Box<TransformModelArgs>),
    /// Import a 2D `.dxf` or `.svg` vector file into a new Pure-2D `.chip.toml` job (or append to `--project`).
    #[command(name = "import-2d")]
    Import2d(Box<Import2dArgs>),
    /// Write ChipCutter's programmatic machined sample STEP file (and optional `.chip.toml` job) to disk.
    GenerateSampleStep(Box<GenerateSampleArgs>),
    /// Validate a `.chip.toml` project's STEP hashes, tools, Setups, WCS, selections, Z heights, and machine limits.
    Validate(Box<ValidateArgs>),
    /// Run headless CAM stock removal (Heightmap or Tri-Dexel) & 5-rule crash/gouge + error-volume verification.
    Simulate(Box<SimulateArgs>),
    /// Generate toolpaths in parallel, verify zero critical crashes/gouges, and post-process G-code (`07-post-processor.md`).
    Build(Box<BuildArgs>),
    /// Reload an updated STEP model into a `.chip.toml` project and re-bind selections via `GeometricSignature`.
    ReloadStep(Box<ReloadStepArgs>),
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
    /// Machine definition utilities (`list`, `inspect`, `clone`, `export`, `import`).
    Machines {
        #[command(subcommand)]
        action: MachineCommands,
    },
    /// 3D workholding fixture & wizard utilities (`list`, `add-vise`, `add-clamp`, `add-spoilboard`, `flip-jig`, `soft-jaws`).
    Workholding {
        #[command(subcommand)]
        action: Box<WorkholdingCommands>,
    },
    /// List all built-in and discovered custom post-processor profiles.
    ListPosts,
}

#[derive(Debug, Subcommand)]
enum WorkholdingCommands {
    /// List all configured Setup fixtures, resolved WCS bounding boxes, margins, and triangle counts.
    List(WorkholdingListArgs),
    /// Add a parametric two-jaw machinist vise (with optional step shelves & parallels) to a Setup.
    #[command(name = "add-vise")]
    AddVise(Box<AddViseArgs>),
    /// Add a parametric toe / strap clamp (with heel riser and T-slot stud tower) to a Setup.
    #[command(name = "add-clamp")]
    AddClamp(Box<AddClampArgs>),
    /// Add a sacrificial MDF / tooling-plate spoilboard beneath a Setup's stock.
    #[command(name = "add-spoilboard")]
    AddSpoilboard(Box<AddSpoilboardArgs>),
    /// Generate symmetric two-sided dowel-pin registration holes, Op-1 pin drilling, spoilboard setup, and paired 180° Op-2 flip setup.
    #[command(name = "flip-jig")]
    FlipJig(Box<FlipJigArgs>),
    /// Generate contoured CNC-machinable soft-jaw Setup and/or watertight 3D-printable `.stl` soft jaws.
    #[command(name = "soft-jaws")]
    SoftJaws(Box<SoftJawsArgs>),
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
        /// Explicitly allow packing asset files located outside the project directory or with non-standard extensions.
        #[arg(long)]
        allow_external_assets: bool,
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

#[derive(Debug, Subcommand)]
enum MachineCommands {
    /// List all built-in (read-only) and custom user machine definitions.
    List,
    /// Print full specifications and tool library for a machine ID or `.machine.toml` path.
    Inspect {
        /// Machine ID (e.g. `generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`) or `.machine.toml` path.
        id: String,
    },
    /// Clone any built-in or custom machine (plus its tool set) into user configuration space.
    Clone {
        /// Source machine ID or `.machine.toml` path to clone.
        source: String,
        /// New custom machine slug ID (e.g. `my-milo`).
        new_id: String,
        /// Optional display name for the cloned machine.
        #[arg(long)]
        name: Option<String>,
    },
    /// Export any machine definition and its full tool set as a shareable `.machine.toml`.
    Export {
        /// Machine ID or `.machine.toml` path to export.
        id: String,
        /// Output `.machine.toml` destination path.
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Validate and import a shared `.machine.toml` into user configuration space.
    Import {
        /// Input `.machine.toml` file path.
        input: PathBuf,
        /// Optional explicit custom machine ID override (auto-disambiguated if omitted).
        #[arg(long)]
        id: Option<String>,
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
        Commands::Inspect(args) => run_inspect(*args),
        Commands::ImportMesh(args) => run_import_mesh(*args),
        Commands::TransformModel(args) => run_transform_model(*args),
        Commands::Import2d(args) => run_import_2d(*args),
        Commands::GenerateSampleStep(args) => run_generate_sample_step(*args),
        Commands::Validate(args) => run_validate(args.project, args.generate, args.machine),
        Commands::Simulate(args) => run_simulate(*args),
        Commands::Build(args) => run_build(*args),
        Commands::ReloadStep(args) => run_reload_step(
            args.project,
            args.step,
            args.output,
            args.allow_needs_repick,
        ),
        Commands::Bundle { action } => match action {
            BundleCommands::Pack {
                project,
                output,
                allow_external_assets,
            } => run_bundle_pack(project, output, allow_external_assets),
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
        Commands::Machines { action } => match action {
            MachineCommands::List => run_machines_list(),
            MachineCommands::Inspect { id } => run_machines_inspect(id),
            MachineCommands::Clone {
                source,
                new_id,
                name,
            } => run_machines_clone(source, new_id, name),
            MachineCommands::Export { id, output } => run_machines_export(id, output),
            MachineCommands::Import { input, id } => run_machines_import(input, id),
        },
        Commands::Workholding { action } => match *action {
            WorkholdingCommands::List(args) => run_workholding_list(args.project, args.setup),
            WorkholdingCommands::AddVise(args) => run_workholding_add_vise(*args),
            WorkholdingCommands::AddClamp(args) => run_workholding_add_clamp(*args),
            WorkholdingCommands::AddSpoilboard(args) => run_workholding_add_spoilboard(*args),
            WorkholdingCommands::FlipJig(args) => run_workholding_flip_jig(*args),
            WorkholdingCommands::SoftJaws(args) => run_workholding_soft_jaws(*args),
        },
        Commands::ListPosts => {
            for profile in chip_post::load_available_post_profiles(None) {
                println!("{}\t{} (.{})", profile.id, profile.name, profile.extension);
            }
            Ok(ExitCode::SUCCESS)
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use clap::CommandFactory;

    use super::*;

    /// Windows binaries default to a 1 MiB main-thread stack. Building clap's
    /// subcommand graph in an unoptimized debug build must stay well below that
    /// threshold (checked here with a 768 KiB stack) so CLI invocations never
    /// overflow the stack on Windows.
    #[test]
    fn cli_parser_fits_inside_small_windows_stack() {
        let handle = std::thread::Builder::new()
            .name("cli-small-stack-check".to_string())
            .stack_size(768 * 1024)
            .spawn(|| {
                Cli::command().debug_assert();

                let parsed_import_2d = Cli::try_parse_from([
                    "chipcutter",
                    "import-2d",
                    "panel.dxf",
                    "-o",
                    "out.chip.toml",
                    "--project",
                    "base.chip.toml",
                    "--thickness",
                    "12.5",
                ])
                .expect("import-2d flags must parse");
                match parsed_import_2d.command {
                    Commands::Import2d(args) => {
                        assert_eq!(
                            args.existing_project.as_deref(),
                            Some(std::path::Path::new("base.chip.toml"))
                        );
                        assert!((args.sheet_thickness_mm - 12.5).abs() < 1e-9);
                    }
                    other => panic!("expected Import2d, got {other:?}"),
                }

                let parsed_build = Cli::try_parse_from([
                    "chipcutter",
                    "build",
                    "job.chip.toml",
                    "-o",
                    "gcode_out",
                    "--setup",
                    "Setup 1",
                    "--post",
                    "linuxcnc",
                    "--allow-warnings",
                ])
                .expect("build flags must parse");
                match parsed_build.command {
                    Commands::Build(args) => {
                        assert_eq!(args.setup_filter.as_deref(), Some("Setup 1"));
                        assert_eq!(args.post_override.as_deref(), Some("linuxcnc"));
                        assert!(args.allow_warnings);
                    }
                    other => panic!("expected Build, got {other:?}"),
                }

                for argv in [
                    vec![
                        "chipcutter",
                        "transform-model",
                        "job.chip.toml",
                        "--rot-x",
                        "-90",
                        "--center-xy",
                    ],
                    vec![
                        "chipcutter",
                        "workholding",
                        "add-vise",
                        "job.chip.toml",
                        "--parallels",
                    ],
                    vec![
                        "chipcutter",
                        "workholding",
                        "add-clamp",
                        "job.chip.toml",
                        "--x",
                        "-10",
                        "--y",
                        "20",
                    ],
                    vec![
                        "chipcutter",
                        "workholding",
                        "add-spoilboard",
                        "job.chip.toml",
                    ],
                    vec![
                        "chipcutter",
                        "workholding",
                        "flip-jig",
                        "job.chip.toml",
                        "--count",
                        "4",
                    ],
                    vec![
                        "chipcutter",
                        "workholding",
                        "soft-jaws",
                        "job.chip.toml",
                        "--add-setup",
                    ],
                    vec!["chipcutter", "list-posts"],
                ] {
                    Cli::try_parse_from(&argv)
                        .unwrap_or_else(|e| panic!("failed to parse {argv:?}: {e}"));
                }
            })
            .expect("spawn stack-check thread");
        handle.join().expect("stack-check thread panicked");
    }
}
