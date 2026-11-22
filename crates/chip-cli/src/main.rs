mod commands;

use std::{path::PathBuf, process::ExitCode};

use clap::{Parser, Subcommand};

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
        AddClampArgs, AddSpoilboardArgs, AddViseArgs, run_workholding_add_clamp,
        run_workholding_add_spoilboard, run_workholding_add_vise, run_workholding_list,
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

#[derive(Debug, Subcommand)]
enum Commands {
    /// Inspect a 3D STEP (`.step`/`.stp`), Triangle Mesh (`.stl`/`.obj`/`.3mf`), or 2D Vector (`.dxf`/`.svg`) file.
    Inspect {
        /// Path to `.step`, `.stp`, `.stl`, `.obj`, `.3mf`, `.dxf`, or `.svg` file.
        path: PathBuf,
        /// Display unit for dimensions (`mm` or `in`).
        #[arg(long, default_value = "mm")]
        units: String,
        /// Optional mesh unit override (`auto`, `mm`, `in`, `cm`, `m`, `ft`, `um`) for `.stl`/`.obj`/`.3mf`.
        #[arg(long, default_value = "auto")]
        mesh_units: String,
        /// SVG unitless pixel DPI (`96` CSS/Inkscape, `72` Illustrator, `90` legacy Inkscape).
        #[arg(long, default_value_t = 96.0)]
        svg_dpi: f64,
        /// Endpoint gap-healing join tolerance in millimeters (`0.0` disables gap healing).
        #[arg(long, default_value_t = 0.05)]
        join_tol: f64,
    },
    /// Import a 3D triangle mesh (`.stl`, `.obj`, or `.3mf`) into a new `schema_version = 3` `.chip.toml` project.
    #[command(name = "import-mesh")]
    ImportMesh {
        /// Input `.stl`, `.obj`, or `.3mf` mesh file.
        input: PathBuf,
        /// Destination `.chip.toml` project file.
        #[arg(short, long)]
        output: PathBuf,
        /// Mesh unit override (`auto`, `mm`, `in`, `cm`, `m`, `ft`, `um`).
        #[arg(long, default_value = "auto")]
        mesh_units: String,
        /// Rotate imported mesh around X axis (degrees).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        rot_x: f64,
        /// Rotate imported mesh around Y axis (degrees).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        rot_y: f64,
        /// Rotate imported mesh around Z axis (degrees).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        rot_z: f64,
        /// Uniform scale factor applied to imported mesh (`1.0` default).
        #[arg(long, default_value_t = 1.0)]
        scale: f64,
        /// Convert Y-up mesh to Z-up (`+90°` around X).
        #[arg(long)]
        y_up_to_z_up: bool,
        /// Center transformed mesh XY bounds on `(0, 0)`.
        #[arg(long)]
        center_xy: bool,
        /// Drop bottom of transformed mesh to `Z = 0`.
        #[arg(long)]
        drop_z0: bool,
    },
    /// Non-destructively rotate, scale, or translate the primary 3D model (STEP or STL/OBJ/3MF) in a `.chip.toml` project.
    #[command(name = "transform-model")]
    TransformModel {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Set X rotation angle (degrees).
        #[arg(long, allow_hyphen_values = true)]
        rot_x: Option<f64>,
        /// Set Y rotation angle (degrees).
        #[arg(long, allow_hyphen_values = true)]
        rot_y: Option<f64>,
        /// Set Z rotation angle (degrees).
        #[arg(long, allow_hyphen_values = true)]
        rot_z: Option<f64>,
        /// Set uniform scale factor across X, Y, and Z.
        #[arg(long)]
        scale: Option<f64>,
        /// Set X scale factor.
        #[arg(long)]
        scale_x: Option<f64>,
        /// Set Y scale factor.
        #[arg(long)]
        scale_y: Option<f64>,
        /// Set Z scale factor.
        #[arg(long)]
        scale_z: Option<f64>,
        /// Uniformly scale so transformed X span equals `<mm>`.
        #[arg(long)]
        fit_size_x: Option<f64>,
        /// Uniformly scale so transformed Y span equals `<mm>`.
        #[arg(long)]
        fit_size_y: Option<f64>,
        /// Uniformly scale so transformed Z span equals `<mm>`.
        #[arg(long)]
        fit_size_z: Option<f64>,
        /// Set X translation (`mm`).
        #[arg(long, allow_hyphen_values = true)]
        move_x: Option<f64>,
        /// Set Y translation (`mm`).
        #[arg(long, allow_hyphen_values = true)]
        move_y: Option<f64>,
        /// Set Z translation (`mm`).
        #[arg(long, allow_hyphen_values = true)]
        move_z: Option<f64>,
        /// Apply `+90° X` (`Y-Up -> Z-Up`) world rotation around model center.
        #[arg(long)]
        y_up_to_z_up: bool,
        /// Center transformed XY bounds on `(0, 0)`.
        #[arg(long)]
        center_xy: bool,
        /// Drop bottom (`min Z`) to `Z = 0`.
        #[arg(long)]
        drop_z0: bool,
        /// Align top (`max Z`) to `Z = 0`.
        #[arg(long)]
        align_top_z0: bool,
        /// Reset transform to identity before applying any other flags.
        #[arg(long)]
        reset: bool,
    },
    /// Import a 2D `.dxf` or `.svg` vector file into a new Pure-2D `.chip.toml` job (or append to `--project`).
    #[command(name = "import-2d")]
    Import2d {
        /// Input `.dxf` or `.svg` vector file.
        input: PathBuf,
        /// Destination `.chip.toml` project file.
        #[arg(short, long)]
        output: PathBuf,
        /// Optional existing `.chip.toml` project to append the `SketchLayer` onto.
        #[arg(long)]
        project: Option<PathBuf>,
        /// Vector unit override (`auto`, `auto_in`, `mm`, `in`, `cm`).
        #[arg(long, default_value = "auto")]
        units: String,
        /// SVG unitless pixel DPI (`96` CSS/Inkscape, `72` Illustrator, `90` legacy Inkscape).
        #[arg(long, default_value_t = 96.0)]
        svg_dpi: f64,
        /// Endpoint gap-healing join tolerance in millimeters (`0.05` default).
        #[arg(long, default_value_t = 0.05)]
        join_tol: f64,
        /// Sheet Z thickness (`mm`) when initializing a new Pure-2D project.
        #[arg(long, default_value_t = 6.0)]
        thickness: f64,
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
        /// Generate a tutorial starter (`01-facing`, `02-contour`, `03-pocket`, `04-drill-bore`, `05-adaptive-rest`, `06-chamfer-trace`, `07-vcarve-medial`, `07-vcarve-inlay`, `13-pure2d-dxf`, `14-reload-v1`, `14-reload-v2`, `16-orient-stl`, `16-orient-step`, `18-thread-milling`, etc.).
        #[arg(long)]
        tutorial: Option<String>,
        /// Optional path to also write a seeded `.chip.toml` CAM project referencing `output`.
        #[arg(long)]
        project_out: Option<PathBuf>,
        /// When combined with `--tutorial` and `--project-out`, write an empty 0-operation starter project (Stock & WCS only).
        #[arg(long)]
        empty_project: bool,
        /// Optional target CNC machine definition ID (e.g. `generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`) to record on the generated project.
        #[arg(long)]
        machine: Option<String>,
    },
    /// Validate a `.chip.toml` project's STEP hashes, tools, Setups, WCS, selections, Z heights, and machine limits.
    Validate {
        /// Path to `.chip.toml` project file (or `.chipz` bundle).
        project: PathBuf,
        /// Also generate all enabled 2.5D toolpaths in parallel and verify non-empty CL-data output.
        #[arg(long)]
        generate: bool,
        /// Optional target machine definition ID or `.machine.toml` path override for validation.
        #[arg(long)]
        machine: Option<String>,
    },
    /// Run headless CAM stock removal (Heightmap or Tri-Dexel) & 5-rule crash/gouge + error-volume verification.
    Simulate {
        /// Path to `.chip.toml` project file (or `.chipz` bundle).
        project: PathBuf,
        /// Simulation engine (`heightmap` fast 2.5D Z-dexel, or `tri-dexel` true 3D volumetric).
        #[arg(long, default_value = "heightmap")]
        engine: String,
        /// Simulation compute backend (`auto` GPU with CPU fallback, `gpu`, or `cpu`).
        #[arg(long, default_value = "auto")]
        backend: String,
        /// Simulation resolution (`standard`, `fine`, `ultra`).
        #[arg(long, default_value = "standard")]
        resolution: String,
        /// Surface deviation pass tolerance band in millimeters for error metrics (`±tolerance`).
        #[arg(long, default_value_t = 0.05)]
        tolerance: f64,
        /// Print structured per-stage timing & throughput benchmark summary.
        #[arg(long)]
        benchmark: bool,
        /// Simulate all Setups sequentially with volumetric stock carryover and emit cumulative end-of-job metrology.
        #[arg(long)]
        all_setups: bool,
        /// Emit structured JSON simulation, backend, timing, and error-volume report to stdout.
        #[arg(long)]
        json: bool,
        /// Optional path to write structured JSON simulation & error-volume report.
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
    /// Machine definition utilities (`list`, `inspect`, `clone`, `export`, `import`).
    Machines {
        #[command(subcommand)]
        action: MachineCommands,
    },
    /// 3D workholding fixture utilities (`list`, `add-vise`, `add-clamp`, `add-spoilboard`).
    Workholding {
        #[command(subcommand)]
        action: WorkholdingCommands,
    },
    /// List all built-in and discovered custom post-processor profiles.
    ListPosts,
}

#[derive(Debug, Subcommand)]
enum WorkholdingCommands {
    /// List all configured Setup fixtures, resolved WCS bounding boxes, margins, and triangle counts.
    List {
        /// Path to `.chip.toml` project file or `.chipz` bundle.
        project: PathBuf,
        /// Optional Setup filter (0-based index, Setup ID, or Setup name).
        #[arg(long)]
        setup: Option<String>,
    },
    /// Add a parametric two-jaw machinist vise (with optional step shelves & parallels) to a Setup.
    #[command(name = "add-vise")]
    AddVise {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Target Setup (0-based index, ID, or name; defaults to Setup 0).
        #[arg(long)]
        setup: Option<String>,
        /// Optional display name for the vise fixture.
        #[arg(long)]
        name: Option<String>,
        /// Clamp axis (`y` = front/rear jaws default, `x` = left/right jaws).
        #[arg(long, default_value = "y")]
        axis: String,
        /// Jaw width along the non-clamping axis (`mm`).
        #[arg(long, default_value_t = 100.0)]
        jaw_width: f64,
        /// Thickness of each jaw plate (`mm`).
        #[arg(long, default_value_t = 18.0)]
        jaw_thickness: f64,
        /// Total jaw height (`mm`).
        #[arg(long, default_value_t = 38.0)]
        jaw_height: f64,
        /// Height of stock bottom gripped upward inside jaws (`mm`).
        #[arg(long, default_value_t = 5.0)]
        grip_depth: f64,
        /// Optional inward step-shelf depth under stock bottom (`mm`, `0` = flat jaws).
        #[arg(long, default_value_t = 0.0)]
        step_depth: f64,
        /// Optional step-shelf vertical thickness below stock bottom (`mm`, `0` = flat jaws).
        #[arg(long, default_value_t = 0.0)]
        step_height: f64,
        /// Include a pair of machinist parallels beneath the workpiece.
        #[arg(long)]
        parallels: bool,
        /// Thickness of each parallel bar (`mm`).
        #[arg(long, default_value_t = 3.175)]
        parallel_thickness: f64,
        /// Stationary fixed jaw side (`positive` = rear/+Y default, `negative` = front/-Y).
        #[arg(long, default_value = "positive")]
        fixed_side: String,
        /// Safety clearance margin halo (`mm`).
        #[arg(long, default_value_t = 1.0)]
        clearance: f64,
    },
    /// Add a parametric toe / strap clamp (with heel riser and T-slot stud tower) to a Setup.
    #[command(name = "add-clamp")]
    AddClamp {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Target Setup (0-based index, ID, or name; defaults to Setup 0).
        #[arg(long)]
        setup: Option<String>,
        /// Optional display name for the clamp fixture.
        #[arg(long)]
        name: Option<String>,
        /// Clamp nose contact X coordinate in Setup WCS (`mm`).
        #[arg(long, allow_hyphen_values = true)]
        x: f64,
        /// Clamp nose contact Y coordinate in Setup WCS (`mm`).
        #[arg(long, allow_hyphen_values = true)]
        y: f64,
        /// Outward approach yaw angle (`deg` in Setup XY; `0` = extends toward `+X`).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        yaw: f64,
        /// Vertical Z datum reference (`stock_top`, `stock_bottom`, `wcs_origin`).
        #[arg(long, default_value = "stock_top")]
        z_ref: String,
        /// Signed Z offset from `z_ref` (`mm`).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        z_offset: f64,
        /// Strap length (`mm`).
        #[arg(long, default_value_t = 45.0)]
        strap_length: f64,
        /// Strap width (`mm`).
        #[arg(long, default_value_t = 16.0)]
        strap_width: f64,
        /// Strap thickness/height (`mm`).
        #[arg(long, default_value_t = 10.0)]
        strap_height: f64,
        /// Nose overlap onto stock edge (`mm`).
        #[arg(long, default_value_t = 6.0)]
        nose_overhang: f64,
        /// T-slot stud/bolt diameter (`mm`).
        #[arg(long, default_value_t = 8.0)]
        stud_diameter: f64,
        /// Stud/nut height protruding above top of strap (`mm`).
        #[arg(long, default_value_t = 12.0)]
        stud_height: f64,
        /// Safety clearance margin halo (`mm`).
        #[arg(long, default_value_t = 1.0)]
        clearance: f64,
    },
    /// Add a sacrificial MDF / tooling-plate spoilboard beneath a Setup's stock.
    #[command(name = "add-spoilboard")]
    AddSpoilboard {
        /// Path to `.chip.toml` project file.
        project: PathBuf,
        /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Target Setup (0-based index, ID, or name; defaults to Setup 0).
        #[arg(long)]
        setup: Option<String>,
        /// Optional display name for the spoilboard.
        #[arg(long)]
        name: Option<String>,
        /// Top surface Z datum reference (`stock_bottom` default, `stock_top`, `wcs_origin`).
        #[arg(long, default_value = "stock_bottom")]
        z_ref: String,
        /// Signed Z offset from `z_ref` (`mm`).
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        z_offset: f64,
        /// Spoilboard thickness (`mm`).
        #[arg(long, default_value_t = 18.0)]
        thickness: f64,
        /// XY overhang beyond stock footprint (`mm`).
        #[arg(long, default_value_t = 25.0)]
        margin_xy: f64,
        /// Permitted sacrificial through-cut depth into spoilboard (`mm`).
        #[arg(long, default_value_t = 0.5)]
        allowed_overcut: f64,
        /// Safety clearance margin halo (`mm`).
        #[arg(long, default_value_t = 0.5)]
        clearance: f64,
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
        Commands::Inspect {
            path,
            units,
            mesh_units,
            svg_dpi,
            join_tol,
        } => run_inspect(InspectArgs {
            path,
            units,
            mesh_units,
            svg_dpi,
            join_tol,
        }),
        Commands::ImportMesh {
            input,
            output,
            mesh_units,
            rot_x,
            rot_y,
            rot_z,
            scale,
            y_up_to_z_up,
            center_xy,
            drop_z0,
        } => run_import_mesh(ImportMeshArgs {
            input,
            output,
            mesh_units,
            rot_x,
            rot_y,
            rot_z,
            scale,
            y_up_to_z_up,
            center_xy,
            drop_z0,
        }),
        Commands::TransformModel {
            project,
            output,
            rot_x,
            rot_y,
            rot_z,
            scale,
            scale_x,
            scale_y,
            scale_z,
            fit_size_x,
            fit_size_y,
            fit_size_z,
            move_x,
            move_y,
            move_z,
            y_up_to_z_up,
            center_xy,
            drop_z0,
            align_top_z0,
            reset,
        } => run_transform_model(TransformModelArgs {
            project,
            output,
            rot_x,
            rot_y,
            rot_z,
            scale,
            scale_x,
            scale_y,
            scale_z,
            fit_size_x,
            fit_size_y,
            fit_size_z,
            move_x,
            move_y,
            move_z,
            y_up_to_z_up,
            center_xy,
            drop_z0,
            align_top_z0,
            reset,
        }),
        Commands::Import2d {
            input,
            output,
            project,
            units,
            svg_dpi,
            join_tol,
            thickness,
        } => run_import_2d(Import2dArgs {
            input,
            output,
            existing_project: project,
            units,
            svg_dpi,
            join_tol,
            sheet_thickness_mm: thickness,
        }),
        Commands::GenerateSampleStep {
            output,
            multi_feature,
            tilted,
            cam_torture,
            revision_v1,
            revision_v2,
            tutorial,
            project_out,
            empty_project,
            machine,
        } => run_generate_sample_step(GenerateSampleArgs {
            output,
            multi_feature,
            tilted,
            cam_torture,
            revision_v1,
            revision_v2,
            tutorial,
            project_out,
            empty_project,
            machine,
        }),
        Commands::Validate {
            project,
            generate,
            machine,
        } => run_validate(project, generate, machine),
        Commands::Simulate {
            project,
            engine,
            backend,
            resolution,
            tolerance,
            benchmark,
            all_setups,
            json,
            report,
        } => run_simulate(SimulateArgs {
            project,
            engine,
            backend,
            resolution,
            tolerance,
            benchmark,
            all_setups,
            json,
            report,
        }),
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
        Commands::Workholding { action } => match action {
            WorkholdingCommands::List { project, setup } => run_workholding_list(project, setup),
            WorkholdingCommands::AddVise {
                project,
                output,
                setup,
                name,
                axis,
                jaw_width,
                jaw_thickness,
                jaw_height,
                grip_depth,
                step_depth,
                step_height,
                parallels,
                parallel_thickness,
                fixed_side,
                clearance,
            } => run_workholding_add_vise(AddViseArgs {
                project,
                output,
                setup,
                name,
                axis,
                jaw_width,
                jaw_thickness,
                jaw_height,
                grip_depth,
                step_depth,
                step_height,
                parallels,
                parallel_thickness,
                fixed_side,
                clearance,
            }),
            WorkholdingCommands::AddClamp {
                project,
                output,
                setup,
                name,
                x,
                y,
                yaw,
                z_ref,
                z_offset,
                strap_length,
                strap_width,
                strap_height,
                nose_overhang,
                stud_diameter,
                stud_height,
                clearance,
            } => run_workholding_add_clamp(AddClampArgs {
                project,
                output,
                setup,
                name,
                x,
                y,
                yaw,
                z_ref,
                z_offset,
                strap_length,
                strap_width,
                strap_height,
                nose_overhang,
                stud_diameter,
                stud_height,
                clearance,
            }),
            WorkholdingCommands::AddSpoilboard {
                project,
                output,
                setup,
                name,
                z_ref,
                z_offset,
                thickness,
                margin_xy,
                allowed_overcut,
                clearance,
            } => run_workholding_add_spoilboard(AddSpoilboardArgs {
                project,
                output,
                setup,
                name,
                z_ref,
                z_offset,
                thickness,
                margin_xy,
                allowed_overcut,
                clearance,
            }),
        },
        Commands::ListPosts => {
            for profile in chip_post::load_available_post_profiles(None) {
                println!("{}\t{} (.{})", profile.id, profile.name, profile.extension);
            }
            Ok(ExitCode::SUCCESS)
        }
    }
}
