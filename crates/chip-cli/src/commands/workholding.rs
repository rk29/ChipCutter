use std::{
    path::{Path, PathBuf},
    process::ExitCode,
};

use chip_cam::{
    FixtureBody, FixtureKind, FixtureZRef, Setup, ViseClampAxis, ViseFixedSide, WorkOffsetSlot,
    resolve_fixture_body, resolve_setup_fixtures,
};
use chip_math::{DVec2, ParamLength};
use chip_project::{
    FlipAxis, FlipJigConfig, FlipJigPinCount, ProjectDocument, SoftJawBoltPattern, SoftJawConfig,
    SoftJawOutputMode, apply_flip_jig_plan, apply_soft_jaw_plan, build_soft_jaw_plan,
    build_two_sided_flip_jig, next_unique_setup_id_with_base, next_work_offset_slot,
    pick_flat_endmill_id, pick_pin_tool,
};
use clap::Args;

use super::{load_primary_cad_for_project, load_project_or_chipz};

#[derive(Debug, Args)]
pub struct AddViseArgs {
    /// Path to `.chip.toml` project file.
    pub project: PathBuf,
    /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    /// Target Setup (0-based index, ID, or name; defaults to Setup 0).
    #[arg(long)]
    pub setup: Option<String>,
    /// Optional display name for the vise fixture.
    #[arg(long)]
    pub name: Option<String>,
    /// Clamp axis (`y` = front/rear jaws default, `x` = left/right jaws).
    #[arg(long, default_value = "y")]
    pub axis: String,
    /// Jaw width along the non-clamping axis (`mm`).
    #[arg(long, default_value_t = 100.0)]
    pub jaw_width: f64,
    /// Thickness of each jaw plate (`mm`).
    #[arg(long, default_value_t = 18.0)]
    pub jaw_thickness: f64,
    /// Total jaw height (`mm`).
    #[arg(long, default_value_t = 38.0)]
    pub jaw_height: f64,
    /// Height of stock bottom gripped upward inside jaws (`mm`).
    #[arg(long, default_value_t = 5.0)]
    pub grip_depth: f64,
    /// Optional inward step-shelf depth under stock bottom (`mm`, `0` = flat jaws).
    #[arg(long, default_value_t = 0.0)]
    pub step_depth: f64,
    /// Optional step-shelf vertical thickness below stock bottom (`mm`, `0` = flat jaws).
    #[arg(long, default_value_t = 0.0)]
    pub step_height: f64,
    /// Include a pair of machinist parallels beneath the workpiece.
    #[arg(long)]
    pub parallels: bool,
    /// Thickness of each parallel bar (`mm`).
    #[arg(long, default_value_t = 3.175)]
    pub parallel_thickness: f64,
    /// Stationary fixed jaw side (`positive` = rear/+Y default, `negative` = front/-Y).
    #[arg(long, default_value = "positive")]
    pub fixed_side: String,
    /// Safety clearance margin halo (`mm`).
    #[arg(long, default_value_t = 1.0)]
    pub clearance: f64,
}

#[derive(Debug, Args)]
pub struct AddClampArgs {
    /// Path to `.chip.toml` project file.
    pub project: PathBuf,
    /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    /// Target Setup (0-based index, ID, or name; defaults to Setup 0).
    #[arg(long)]
    pub setup: Option<String>,
    /// Optional display name for the clamp fixture.
    #[arg(long)]
    pub name: Option<String>,
    /// Clamp nose contact X coordinate in Setup WCS (`mm`).
    #[arg(long, allow_hyphen_values = true)]
    pub x: f64,
    /// Clamp nose contact Y coordinate in Setup WCS (`mm`).
    #[arg(long, allow_hyphen_values = true)]
    pub y: f64,
    /// Outward approach yaw angle (`deg` in Setup XY; `0` = extends toward `+X`).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub yaw: f64,
    /// Vertical Z datum reference (`stock_top`, `stock_bottom`, `wcs_origin`).
    #[arg(long, default_value = "stock_top")]
    pub z_ref: String,
    /// Signed Z offset from `z_ref` (`mm`).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub z_offset: f64,
    /// Strap length (`mm`).
    #[arg(long, default_value_t = 45.0)]
    pub strap_length: f64,
    /// Strap width (`mm`).
    #[arg(long, default_value_t = 16.0)]
    pub strap_width: f64,
    /// Strap thickness/height (`mm`).
    #[arg(long, default_value_t = 10.0)]
    pub strap_height: f64,
    /// Nose overlap onto stock edge (`mm`).
    #[arg(long, default_value_t = 6.0)]
    pub nose_overhang: f64,
    /// T-slot stud/bolt diameter (`mm`).
    #[arg(long, default_value_t = 8.0)]
    pub stud_diameter: f64,
    /// Stud/nut height protruding above top of strap (`mm`).
    #[arg(long, default_value_t = 12.0)]
    pub stud_height: f64,
    /// Safety clearance margin halo (`mm`).
    #[arg(long, default_value_t = 1.0)]
    pub clearance: f64,
}

#[derive(Debug, Args)]
pub struct AddSpoilboardArgs {
    /// Path to `.chip.toml` project file.
    pub project: PathBuf,
    /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    /// Target Setup (0-based index, ID, or name; defaults to Setup 0).
    #[arg(long)]
    pub setup: Option<String>,
    /// Optional display name for the spoilboard.
    #[arg(long)]
    pub name: Option<String>,
    /// Top surface Z datum reference (`stock_bottom` default, `stock_top`, `wcs_origin`).
    #[arg(long, default_value = "stock_bottom")]
    pub z_ref: String,
    /// Signed Z offset from `z_ref` (`mm`).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub z_offset: f64,
    /// Spoilboard thickness (`mm`).
    #[arg(long, default_value_t = 18.0)]
    pub thickness: f64,
    /// XY overhang beyond stock footprint (`mm`).
    #[arg(long, default_value_t = 25.0)]
    pub margin_xy: f64,
    /// Permitted sacrificial through-cut depth into spoilboard (`mm`).
    #[arg(long, default_value_t = 0.5)]
    pub allowed_overcut: f64,
    /// Safety clearance margin halo (`mm`).
    #[arg(long, default_value_t = 0.5)]
    pub clearance: f64,
}

fn find_setup_index(proj: &ProjectDocument, filter: Option<&str>) -> Result<usize, String> {
    if proj.setups.is_empty() {
        return Err("project contains no setups".to_string());
    }
    let Some(raw) = filter.map(str::trim).filter(|s| !s.is_empty()) else {
        return Ok(0);
    };
    if let Ok(idx) = raw.parse::<usize>()
        && idx < proj.setups.len()
    {
        return Ok(idx);
    }
    proj.setups
        .iter()
        .position(|s| s.id.eq_ignore_ascii_case(raw) || s.name.eq_ignore_ascii_case(raw))
        .ok_or_else(|| format!("setup '{raw}' not found in project"))
}

fn next_fixture_id(setup: &Setup, prefix: &str) -> String {
    let mut n = setup.fixtures.len() + 1;
    loop {
        let cand = format!("{prefix}_{n}");
        if !setup.fixtures.iter().any(|f| f.id == cand) {
            return cand;
        }
        n += 1;
    }
}

fn parse_vise_axis(raw: &str) -> Result<ViseClampAxis, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "y" | "y-axis" => Ok(ViseClampAxis::Y),
        "x" | "x-axis" => Ok(ViseClampAxis::X),
        other => Err(format!("invalid --axis '{other}'; expected 'y' or 'x'")),
    }
}

fn parse_fixed_side(raw: &str) -> Result<ViseFixedSide, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "positive" | "pos" | "+" | "rear" | "right" => Ok(ViseFixedSide::Positive),
        "negative" | "neg" | "-" | "front" | "left" => Ok(ViseFixedSide::Negative),
        other => Err(format!(
            "invalid --fixed-side '{other}'; expected 'positive' or 'negative'"
        )),
    }
}

fn parse_z_ref(raw: &str) -> Result<FixtureZRef, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "stock_top" | "stock-top" | "top" => Ok(FixtureZRef::StockTop),
        "stock_bottom" | "stock-bottom" | "bottom" => Ok(FixtureZRef::StockBottom),
        "wcs_origin" | "wcs-origin" | "wcs" | "origin" => Ok(FixtureZRef::WcsOrigin),
        other => Err(format!(
            "invalid --z-ref '{other}'; expected 'stock_top', 'stock_bottom', or 'wcs_origin'"
        )),
    }
}

pub fn run_workholding_list(
    project: PathBuf,
    setup_filter: Option<String>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let loaded = load_project_or_chipz(&project)?;
    let proj = &loaded.proj;
    let cad_doc = load_primary_cad_for_project(proj, loaded.dir.as_deref())?;
    let fixture_docs = proj.load_fixture_cad_documents(loaded.dir.as_deref());

    println!("=== ChipCutter Workholding: {} ===", proj.project_name);
    let mut matched_any = false;

    for (s_idx, setup) in proj.setups.iter().enumerate() {
        if let Some(ref f) = setup_filter {
            let matches_idx = f.parse::<usize>().ok() == Some(s_idx);
            let matches_name =
                setup.id.eq_ignore_ascii_case(f) || setup.name.eq_ignore_ascii_case(f);
            if !matches_idx && !matches_name {
                continue;
            }
        }
        matched_any = true;

        let resolved = setup.resolve_with_sketches(&cad_doc, &proj.sketches, proj.display_units)?;
        let stock_aabb = resolved.stock.aabb_wcs();
        let eval_ctx = resolved.eval_context(proj.display_units);

        println!(
            "\nSetup #{s_idx}: {} [{}] (auto_avoid_rapids = {}, {} fixture(s))",
            setup.name,
            setup.wcs_slot.short_code(),
            setup.auto_avoid_fixtures_on_rapids,
            setup.fixtures.len()
        );
        if setup.fixtures.is_empty() {
            println!("  (no workholding fixtures configured)");
            continue;
        }

        for fix in &setup.fixtures {
            let status = if fix.enabled { "ENABLED " } else { "DISABLED" };
            match resolve_fixture_body(fix, &stock_aabb, &eval_ctx, &mut |id| fixture_docs.get(id))
            {
                Ok(sol) => {
                    let b = sol.aabb_wcs;
                    println!(
                        "  [{status}] {} ({}) — {} | margin={:.2} mm, overcut={:.2} mm | {} tris | WCS AABB X:[{:.2}..{:.2}] Y:[{:.2}..{:.2}] Z:[{:.2}..{:.2}] mm",
                        fix.id,
                        fix.kind.short_badge(),
                        fix.name,
                        sol.clearance_margin_mm,
                        sol.allowed_overcut_mm,
                        sol.mesh_wcs.len(),
                        b.min.x,
                        b.max.x,
                        b.min.y,
                        b.max.y,
                        b.min.z,
                        b.max.z,
                    );
                }
                Err(err) => {
                    println!(
                        "  [{status}] {} ({}) — {} | ERROR: {err}",
                        fix.id,
                        fix.kind.short_badge(),
                        fix.name
                    );
                }
            }
        }
    }

    if !matched_any {
        return Err(format!(
            "no setup matched filter '{}'",
            setup_filter.unwrap_or_default()
        )
        .into());
    }
    Ok(ExitCode::SUCCESS)
}

pub fn run_workholding_add_vise(args: AddViseArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj = ProjectDocument::load_from_path(&args.project)?;
    let s_idx = find_setup_index(&proj, args.setup.as_deref())?;
    let clamp_axis = parse_vise_axis(&args.axis)?;
    let fixed_jaw_side = parse_fixed_side(&args.fixed_side)?;

    let id = next_fixture_id(&proj.setups[s_idx], "vise");
    let name = args.name.unwrap_or_else(|| "Machinist Vise".to_string());
    let fixture = FixtureBody {
        id: id.clone(),
        name: name.clone(),
        enabled: true,
        clearance_margin: ParamLength::from_mm(args.clearance),
        allowed_overcut: ParamLength::from_mm(0.0),
        kind: FixtureKind::ViseJaws {
            clamp_axis,
            jaw_width: ParamLength::from_mm(args.jaw_width),
            jaw_thickness: ParamLength::from_mm(args.jaw_thickness),
            jaw_height: ParamLength::from_mm(args.jaw_height),
            grip_depth: ParamLength::from_mm(args.grip_depth),
            step_shelf_depth: ParamLength::from_mm(args.step_depth),
            step_shelf_height: ParamLength::from_mm(args.step_height),
            include_parallels: args.parallels,
            parallel_thickness: ParamLength::from_mm(args.parallel_thickness),
            center_offset_along_jaw: ParamLength::from_mm(0.0),
            fixed_jaw_side,
        },
        color_rgba: None,
    };

    validate_and_save_fixture(&mut proj, &args.project, args.output, s_idx, fixture)?;
    println!("Added ViseJaws fixture '{id}' ({name}) to setup #{s_idx}.");
    Ok(ExitCode::SUCCESS)
}

pub fn run_workholding_add_clamp(
    args: AddClampArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj = ProjectDocument::load_from_path(&args.project)?;
    let s_idx = find_setup_index(&proj, args.setup.as_deref())?;
    let z_reference = parse_z_ref(&args.z_ref)?;

    let id = next_fixture_id(&proj.setups[s_idx], "clamp");
    let name = args.name.unwrap_or_else(|| {
        format!(
            "Toe Clamp {}",
            proj.setups[s_idx]
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::ToeClamp { .. }))
                .count()
                + 1
        )
    });
    let mut fixture = FixtureBody::default_toe_clamp(
        id.clone(),
        name.clone(),
        DVec2::new(args.x, args.y),
        args.yaw,
    );
    fixture.clearance_margin = ParamLength::from_mm(args.clearance);
    if let FixtureKind::ToeClamp {
        z_reference: zr,
        z_offset,
        strap_length,
        strap_width,
        strap_height,
        nose_overhang,
        stud_diameter,
        stud_height_above_strap,
        ..
    } = &mut fixture.kind
    {
        *zr = z_reference;
        *z_offset = ParamLength::from_mm(args.z_offset);
        *strap_length = ParamLength::from_mm(args.strap_length);
        *strap_width = ParamLength::from_mm(args.strap_width);
        *strap_height = ParamLength::from_mm(args.strap_height);
        *nose_overhang = ParamLength::from_mm(args.nose_overhang);
        *stud_diameter = ParamLength::from_mm(args.stud_diameter);
        *stud_height_above_strap = ParamLength::from_mm(args.stud_height);
    }

    validate_and_save_fixture(&mut proj, &args.project, args.output, s_idx, fixture)?;
    println!("Added ToeClamp fixture '{id}' ({name}) to setup #{s_idx}.");
    Ok(ExitCode::SUCCESS)
}

pub fn run_workholding_add_spoilboard(
    args: AddSpoilboardArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj = ProjectDocument::load_from_path(&args.project)?;
    let s_idx = find_setup_index(&proj, args.setup.as_deref())?;
    let z_top_ref = parse_z_ref(&args.z_ref)?;

    let id = next_fixture_id(&proj.setups[s_idx], "spoilboard");
    let name = args.name.unwrap_or_else(|| "MDF Spoilboard".to_string());
    let fixture = FixtureBody {
        id: id.clone(),
        name: name.clone(),
        enabled: true,
        clearance_margin: ParamLength::from_mm(args.clearance),
        allowed_overcut: ParamLength::from_mm(args.allowed_overcut),
        kind: FixtureKind::Spoilboard {
            z_top_ref,
            z_offset: ParamLength::from_mm(args.z_offset),
            thickness: ParamLength::from_mm(args.thickness),
            margin_xy: ParamLength::from_mm(args.margin_xy),
        },
        color_rgba: None,
    };

    validate_and_save_fixture(&mut proj, &args.project, args.output, s_idx, fixture)?;
    println!("Added Spoilboard fixture '{id}' ({name}) to setup #{s_idx}.");
    Ok(ExitCode::SUCCESS)
}

fn validate_and_save_fixture(
    proj: &mut ProjectDocument,
    source_path: &Path,
    output_path: Option<PathBuf>,
    setup_idx: usize,
    fixture: FixtureBody,
) -> Result<(), Box<dyn std::error::Error>> {
    proj.setups[setup_idx].fixtures.push(fixture);
    for op in &mut proj.setups[setup_idx].operations {
        op.mark_dirty();
    }

    // Verify the added fixture resolves cleanly against the project's CAD/stock geometry
    let cad_doc = load_primary_cad_for_project(proj, source_path.parent())?;
    let fixture_docs = proj.load_fixture_cad_documents(source_path.parent());
    let resolved = proj.setups[setup_idx].resolve_with_sketches(
        &cad_doc,
        &proj.sketches,
        proj.display_units,
    )?;
    let solids = resolve_setup_fixtures(
        &proj.setups[setup_idx],
        &resolved,
        proj.display_units,
        |id| fixture_docs.get(id),
    )?;
    if let Some(last) = solids.last() {
        let b = last.aabb_wcs;
        println!(
            "  Resolved {} tris | WCS AABB X:[{:.2}..{:.2}] Y:[{:.2}..{:.2}] Z:[{:.2}..{:.2}] mm",
            last.mesh_wcs.len(),
            b.min.x,
            b.max.x,
            b.min.y,
            b.max.y,
            b.min.z,
            b.max.z
        );
    }

    let dest = output_path.unwrap_or_else(|| source_path.to_path_buf());
    proj.save_to_path(&dest)?;
    println!("Saved updated project to {}", dest.display());
    Ok(())
}

#[derive(Debug, Args)]
pub struct FlipJigArgs {
    /// Path to `.chip.toml` project file.
    pub project: PathBuf,
    /// Optional output `.chip.toml` path (overwrites `project` in place if omitted).
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    /// Target Setup 1 (0-based index, ID, or name; defaults to Setup 0).
    #[arg(long)]
    pub setup: Option<String>,
    /// Flip axis (`x` = front-to-back around Y center, `y` = left-to-right around X center).
    #[arg(long, default_value = "x")]
    pub axis: String,
    /// Nominal dowel pin diameter (`mm`).
    #[arg(long, default_value_t = 6.0)]
    pub pin_dia: f64,
    /// Pin fit clearance added to hole diameter (`mm`).
    #[arg(long, default_value_t = 0.02)]
    pub pin_clearance: f64,
    /// Number of symmetric registration pins (`2` on flip centerline or `4` mirrored pairs).
    #[arg(long, default_value_t = 2)]
    pub count: usize,
    /// Optional asymmetric poka-yoke shift along the flip axis (`mm`).
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pub poka_yoke: f64,
    /// Minimum clearance outside the target part silhouette (`mm`).
    #[arg(long, default_value_t = 6.0)]
    pub edge_margin: f64,
    /// Stock pin hole depth (`0` = through-stock, `> 0` = blind depth in `mm`).
    #[arg(long, default_value_t = 0.0)]
    pub stock_depth: f64,
    /// Mating spoilboard dowel hole depth (`mm`).
    #[arg(long, default_value_t = 10.0)]
    pub spoilboard_depth: f64,
    /// Omit the companion Spoilboard Dowel Holes setup.
    #[arg(long)]
    pub no_spoilboard: bool,
}

#[derive(Debug, Args)]
pub struct SoftJawsArgs {
    /// Path to `.chip.toml` project file.
    pub project: PathBuf,
    /// Optional output `.chip.toml` path when `--add-setup` is set (overwrites `project` if omitted).
    #[arg(short, long)]
    pub output: Option<PathBuf>,
    /// Source Setup (0-based index, ID, or name; defaults to Setup 0).
    #[arg(long)]
    pub setup: Option<String>,
    /// Vise clamping axis (`y` = front/rear jaws default, `x` = left/right jaws).
    #[arg(long, default_value = "y")]
    pub axis: String,
    /// Soft jaw width (`mm`).
    #[arg(long, default_value_t = 100.0)]
    pub jaw_width: f64,
    /// Soft jaw plate depth/thickness (`mm`).
    #[arg(long, default_value_t = 25.0)]
    pub jaw_depth: f64,
    /// Total vertical soft jaw blank height (`mm`).
    #[arg(long, default_value_t = 38.0)]
    pub jaw_height: f64,
    /// Closed spacer gap between fixed & movable jaws (`mm`).
    #[arg(long, default_value_t = 6.0)]
    pub jaw_gap: f64,
    /// Vertical pocket grip depth cut into top of soft jaws (`mm`).
    #[arg(long, default_value_t = 4.0)]
    pub grip_depth: f64,
    /// Radial slip-fit clearance around target part silhouette (`mm`).
    #[arg(long, default_value_t = 0.05)]
    pub clearance: f64,
    /// Internal sharp-corner dogbone relief radius (`mm`, `0` disables).
    #[arg(long, default_value_t = 1.5)]
    pub corner_relief: f64,
    /// Mounting bolt counterbore pattern (`none`, `kurt6`, `milo4`, `carvera`).
    #[arg(long, default_value = "milo4")]
    pub bolt_pattern: String,
    /// Append a ready-to-generate CNC soft-jaw machining Setup + Sketch to the project.
    #[arg(long)]
    pub add_setup: bool,
    /// Export watertight 3D-printable `fixed_jaw.stl` and `movable_jaw.stl` to this directory.
    #[arg(long)]
    pub export_stl_dir: Option<PathBuf>,
}

fn parse_flip_axis(raw: &str) -> Result<FlipAxis, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "x" | "x-axis" | "x_axis" => Ok(FlipAxis::XAxis),
        "y" | "y-axis" | "y_axis" => Ok(FlipAxis::YAxis),
        other => Err(format!("invalid --axis '{other}'; expected 'x' or 'y'")),
    }
}

fn parse_pin_count(count: usize) -> Result<FlipJigPinCount, String> {
    match count {
        2 => Ok(FlipJigPinCount::TwoOnAxis),
        4 => Ok(FlipJigPinCount::FourSymmetric),
        other => Err(format!("invalid --count '{other}'; expected '2' or '4'")),
    }
}

fn parse_bolt_pattern(raw: &str) -> Result<SoftJawBoltPattern, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "none" | "blank" => Ok(SoftJawBoltPattern::None),
        "kurt6" | "kurt-6" | "kurt" | "6in" => Ok(SoftJawBoltPattern::Kurt6Inch),
        "milo4" | "milo-4" | "milo" | "4in" => Ok(SoftJawBoltPattern::Milo4Inch),
        "carvera" | "carvera-mod" => Ok(SoftJawBoltPattern::CarveraModVise),
        other => Err(format!(
            "invalid --bolt-pattern '{other}'; expected 'none', 'kurt6', 'milo4', or 'carvera'"
        )),
    }
}

pub fn run_workholding_flip_jig(args: FlipJigArgs) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj = ProjectDocument::load_from_path(&args.project)?;
    let s_idx = find_setup_index(&proj, args.setup.as_deref())?;
    let flip_axis = parse_flip_axis(&args.axis)?;
    let pin_count = parse_pin_count(args.count)?;

    let cad_doc = load_primary_cad_for_project(&proj, args.project.parent())?;
    let hole_dia = args.pin_dia + args.pin_clearance;
    let (pin_tool_id, pin_tool_dia) = pick_pin_tool(&proj.tools, hole_dia);

    let cfg = FlipJigConfig {
        flip_axis,
        pin_diameter_mm: args.pin_dia,
        pin_fit_clearance_mm: args.pin_clearance,
        pin_count,
        poka_yoke_offset_mm: args.poka_yoke,
        stock_pin_depth_mm: args.stock_depth,
        spoilboard_pin_depth_mm: args.spoilboard_depth,
        edge_margin_mm: args.edge_margin,
        expand_stock_if_needed: true,
        include_spoilboard_setup: !args.no_spoilboard,
        pin_protrusion_z_mm: 6.0,
        pin_tool_id,
        pin_tool_diameter_mm: pin_tool_dia,
        setup2_id: Some(next_unique_setup_id_with_base(&proj, "setup_2_flip")),
        setup2_wcs_slot: next_work_offset_slot(&proj),
        spoilboard_setup_id: Some(next_unique_setup_id_with_base(
            &proj,
            "setup_spoilboard_pins",
        )),
        spoilboard_wcs_slot: WorkOffsetSlot::G54,
    };

    let plan = build_two_sided_flip_jig(
        &cad_doc,
        &proj.setups[s_idx],
        &proj.sketches,
        proj.display_units,
        &cfg,
    )?;

    let summary_axis = plan.flip_axis.label();
    let pin_centers = plan.pin_centers_setup1_wcs.clone();
    let hole_dia_out = plan.hole_diameter_mm;
    let max_sym_err = plan.max_symmetry_error_mm;
    let min_clear = plan.min_part_clearance_mm;
    let s2_name = plan.setup2_flip.name.clone();
    apply_flip_jig_plan(&mut proj, s_idx, plan);

    let dest = args.output.unwrap_or_else(|| args.project.clone());
    proj.save_to_path(&dest)?;

    println!(
        "=== Flip-Jig Wizard Applied ({} · {} pins · Ø{:.2} mm holes) ===",
        summary_axis,
        pin_centers.len(),
        hole_dia_out
    );
    println!("  Symmetry error: {max_sym_err:.2e} mm | Min part clearance: {min_clear:.2} mm");
    for (i, pt) in pin_centers.iter().enumerate() {
        println!(
            "  Pin #{}: X={:.3} mm, Y={:.3} mm (Setup 1 WCS)",
            i + 1,
            pt.x,
            pt.y
        );
    }
    println!("  Created paired flip setup: '{s2_name}'");
    println!("Saved updated project to {}", dest.display());
    Ok(ExitCode::SUCCESS)
}

pub fn run_workholding_soft_jaws(
    args: SoftJawsArgs,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let mut proj = ProjectDocument::load_from_path(&args.project)?;
    let s_idx = find_setup_index(&proj, args.setup.as_deref())?;
    let clamp_axis = parse_vise_axis(&args.axis)?;
    let mounting_bolt_pattern = parse_bolt_pattern(&args.bolt_pattern)?;

    if !args.add_setup && args.export_stl_dir.is_none() {
        return Err(
            "specify --add-setup (to add a CNC machining setup) and/or --export-stl-dir <dir> (to export 3D-printable STLs)"
                .into(),
        );
    }

    let output_mode = match (args.add_setup, args.export_stl_dir.is_some()) {
        (true, true) => SoftJawOutputMode::Both,
        (true, false) => SoftJawOutputMode::MachiningSetup,
        (false, true) => SoftJawOutputMode::PrintableStl,
        (false, false) => SoftJawOutputMode::Both,
    };

    let cad_doc = load_primary_cad_for_project(&proj, args.project.parent())?;
    let tool_id = pick_flat_endmill_id(&proj.tools);

    let cfg = SoftJawConfig {
        clamp_axis,
        jaw_width_mm: args.jaw_width,
        jaw_depth_mm: args.jaw_depth,
        jaw_height_mm: args.jaw_height,
        jaw_gap_mm: args.jaw_gap,
        grip_depth_mm: args.grip_depth,
        part_clearance_mm: args.clearance,
        corner_relief_radius_mm: args.corner_relief,
        mounting_bolt_pattern,
        output_mode,
        z_band_bottom_offset_mm: 0.0,
        roughing_tool_id: tool_id.clone(),
        finishing_tool_id: tool_id,
        setup_id: Some(next_unique_setup_id_with_base(&proj, "setup_soft_jaws")),
        wcs_slot: next_work_offset_slot(&proj),
    };

    let plan = build_soft_jaw_plan(
        &cad_doc,
        &proj.setups[s_idx],
        &proj.sketches,
        proj.display_units,
        &cfg,
    )?;

    println!(
        "=== Auto Soft-Jaw Wizard ({:.1}×{:.1}×{:.1} mm · grip={:.2} mm · clearance={:.2} mm · {} corner relief(s)) ===",
        plan.jaw_width_mm,
        plan.jaw_depth_mm,
        plan.jaw_height_mm,
        plan.grip_depth_mm,
        plan.part_clearance_mm,
        plan.corner_relief_count
    );

    if let Some(dir) = &args.export_stl_dir {
        std::fs::create_dir_all(dir)?;
        if let Some(fixed_bytes) = &plan.fixed_jaw_stl {
            let p = dir.join("fixed_jaw.stl");
            std::fs::write(&p, fixed_bytes)?;
            println!(
                "  Exported {} ({} triangles, {} bytes)",
                p.display(),
                plan.fixed_jaw_triangle_count,
                fixed_bytes.len()
            );
        }
        if let Some(movable_bytes) = &plan.movable_jaw_stl {
            let p = dir.join("movable_jaw.stl");
            std::fs::write(&p, movable_bytes)?;
            println!(
                "  Exported {} ({} triangles, {} bytes)",
                p.display(),
                plan.movable_jaw_triangle_count,
                movable_bytes.len()
            );
        }
    }

    if args.add_setup {
        if let Some(new_idx) = apply_soft_jaw_plan(&mut proj, plan) {
            println!(
                "  Added CNC machining setup: '{}'",
                proj.setups[new_idx].name
            );
        }
        let dest = args.output.unwrap_or_else(|| args.project.clone());
        proj.save_to_path(&dest)?;
        println!("Saved updated project to {}", dest.display());
    }

    Ok(ExitCode::SUCCESS)
}
