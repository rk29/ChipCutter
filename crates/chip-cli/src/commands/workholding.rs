use std::{
    path::{Path, PathBuf},
    process::ExitCode,
};

use chip_cam::{
    FixtureBody, FixtureKind, FixtureZRef, Setup, ViseClampAxis, ViseFixedSide,
    resolve_fixture_body, resolve_setup_fixtures,
};
use chip_math::{DVec2, ParamLength};
use chip_project::ProjectDocument;

use super::{load_primary_cad_for_project, load_project_or_chipz};

pub struct AddViseArgs {
    pub project: PathBuf,
    pub output: Option<PathBuf>,
    pub setup: Option<String>,
    pub name: Option<String>,
    pub axis: String,
    pub jaw_width: f64,
    pub jaw_thickness: f64,
    pub jaw_height: f64,
    pub grip_depth: f64,
    pub step_depth: f64,
    pub step_height: f64,
    pub parallels: bool,
    pub parallel_thickness: f64,
    pub fixed_side: String,
    pub clearance: f64,
}

pub struct AddClampArgs {
    pub project: PathBuf,
    pub output: Option<PathBuf>,
    pub setup: Option<String>,
    pub name: Option<String>,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub z_ref: String,
    pub z_offset: f64,
    pub strap_length: f64,
    pub strap_width: f64,
    pub strap_height: f64,
    pub nose_overhang: f64,
    pub stud_diameter: f64,
    pub stud_height: f64,
    pub clearance: f64,
}

pub struct AddSpoilboardArgs {
    pub project: PathBuf,
    pub output: Option<PathBuf>,
    pub setup: Option<String>,
    pub name: Option<String>,
    pub z_ref: String,
    pub z_offset: f64,
    pub thickness: f64,
    pub margin_xy: f64,
    pub allowed_overcut: f64,
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
