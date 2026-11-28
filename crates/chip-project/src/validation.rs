use std::{borrow::Cow, path::Path};

use chip_cad::CadDocument;
use chip_cam::{
    CamValidationIssue, CamValidationSeverity, FixtureKind, MachinabilitySeverity,
    OperationMachinabilityContext, OperationMachinabilityReport, OperationRotaryMode,
    ResolvedHeights, ResolvedSetup, RestMachiningMode, Setup, SetupMachinabilityReport,
    analyze_operation_machinability, analyze_setup_machinability, build_indexed_virtual_setup,
    extract_rotary_selection_features, extract_selection_features_with_sketches,
    resolve_fixture_body, validate_operation_extracted_geometry,
    validate_operation_strategy_and_rotary, validate_operation_undercut_depth,
};
use chip_machines::{MachineDefinition, builtin_generic_machine, resolve_machine_definition};
use chip_tools::{Tool, ToolLibrary};
use serde::{Deserialize, Serialize};

use crate::document::{ModelRole, ProjectDocument, compute_sha256_hex, resolve_model_path};

/// Severity of a project validation diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticSeverity {
    Warning,
    Error,
}

/// Structured validation finding on a `.chip.toml` project.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationDiagnostic {
    pub severity: DiagnosticSeverity,
    pub scope: String,
    pub message: String,
}

/// Full report returned by [`validate_project`].
#[derive(Debug, Clone, Default)]
pub struct ProjectValidationReport {
    pub diagnostics: Vec<ValidationDiagnostic>,
}

impl ProjectValidationReport {
    pub fn error_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == DiagnosticSeverity::Error)
            .count()
    }

    pub fn warning_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == DiagnosticSeverity::Warning)
            .count()
    }

    pub fn is_valid(&self) -> bool {
        self.error_count() == 0
    }

    fn push_error(&mut self, scope: impl Into<String>, message: impl Into<String>) {
        self.diagnostics.push(ValidationDiagnostic {
            severity: DiagnosticSeverity::Error,
            scope: scope.into(),
            message: message.into(),
        });
    }

    fn push_warning(&mut self, scope: impl Into<String>, message: impl Into<String>) {
        self.diagnostics.push(ValidationDiagnostic {
            severity: DiagnosticSeverity::Warning,
            scope: scope.into(),
            message: message.into(),
        });
    }
}

/// Validates a [`ProjectDocument`]'s model files/hashes, embedded tools, Setups, WCS frames,
/// Stock bounds, B-Rep selections, Z-height hierarchies, and target machine envelope/spindle
/// limits (`09-persistence-cli.md`).
pub fn validate_project(
    project: &ProjectDocument,
    project_dir: Option<&Path>,
    loaded_cad: Option<&CadDocument>,
) -> ProjectValidationReport {
    validate_project_with_machine(project, project_dir, loaded_cad, None)
}

/// Validates a [`ProjectDocument`] against an explicit [`MachineDefinition`] override (or the
/// project's own `machine_id`, falling back to `"generic"` with a warning if unknown).
pub fn validate_project_with_machine(
    project: &ProjectDocument,
    project_dir: Option<&Path>,
    loaded_cad: Option<&CadDocument>,
    machine_override: Option<&MachineDefinition>,
) -> ProjectValidationReport {
    let mut report = ProjectValidationReport::default();

    let resolved_machine: Cow<'_, MachineDefinition> = if let Some(m) = machine_override {
        Cow::Borrowed(m)
    } else {
        let mid = project.effective_machine_id();
        match resolve_machine_definition(mid, project_dir) {
            Ok(m) => Cow::Owned(m),
            Err(_) => {
                report.push_warning(
                    "Machine",
                    format!(
                        "unknown machine_id '{mid}'; falling back to built-in 'generic' limits."
                    ),
                );
                Cow::Owned(builtin_generic_machine())
            }
        }
    };

    // 1. Validate model transforms & check model files/hashes when project_dir is on disk
    for model in &project.models {
        if let Err(err) = model.transform.validate() {
            report.push_error(format!("Model '{}'", model.id), err.to_string());
        }
    }
    if let Some(dir) = project_dir {
        for model in &project.models {
            let resolved_path = resolve_model_path(Some(dir), &model.path);
            match std::fs::read(&resolved_path) {
                Ok(bytes) => {
                    let actual_sha = compute_sha256_hex(&bytes);
                    if !model.sha256.is_empty() && !actual_sha.eq_ignore_ascii_case(&model.sha256) {
                        report.push_warning(
                            format!("Model '{}'", model.id),
                            format!(
                                "model file '{}' SHA-256 changed on disk (expected {}…, found {}…). Reload model to refresh geometry.",
                                model.path,
                                &model.sha256[..model.sha256.len().min(12)],
                                &actual_sha[..12]
                            ),
                        );
                    }
                }
                Err(e) => {
                    report.push_error(
                        format!("Model '{}'", model.id),
                        format!("cannot read model file '{}': {e}", resolved_path.display()),
                    );
                }
            }
        }
        for sk in &project.sketches {
            if let Some(asset) = &sk.source_asset {
                let resolved_path = resolve_model_path(Some(dir), &asset.path);
                match std::fs::read(&resolved_path) {
                    Ok(bytes) => {
                        let actual_sha = compute_sha256_hex(&bytes);
                        if !asset.sha256.is_empty()
                            && !actual_sha.eq_ignore_ascii_case(&asset.sha256)
                        {
                            report.push_warning(
                                format!("Sketch '{}'", sk.name),
                                format!(
                                    "{} file '{}' SHA-256 changed on disk (expected {}…, found {}…).",
                                    asset.format.label(),
                                    asset.path,
                                    &asset.sha256[..asset.sha256.len().min(12)],
                                    &actual_sha[..12]
                                ),
                            );
                        }
                    }
                    Err(e) => {
                        report.push_warning(
                            format!("Sketch '{}'", sk.name),
                            format!(
                                "source {} file '{}' not found on disk ({e}); using cached sketch geometry.",
                                asset.format.label(),
                                resolved_path.display()
                            ),
                        );
                    }
                }
            }
        }
    }

    // 2. Validate embedded tools
    for tool in &project.tools {
        if let Err(err) = tool.validate() {
            report.push_error(
                format!("Tool T{} ({})", tool.tool_number, tool.id),
                err.to_string(),
            );
        }
    }

    // 2a. Validate project-level reusable Shared Avoid / Keep-Out regions
    for shared in &project.avoid_regions {
        let scope = format!("Shared Avoid Region '{}'", shared.name);
        if let Some(ref rad) = shared.radial_clearance
            && (!rad.value_mm.is_finite() || rad.value_mm < -1e-9)
        {
            report.push_error(
                &scope,
                format!(
                    "shared avoid radial_clearance ({:.3} mm) must be finite and >= 0.",
                    rad.value_mm
                ),
            );
        }
        if let Some(ref ax) = shared.axial_clearance
            && (!ax.value_mm.is_finite() || ax.value_mm < -1e-9)
        {
            report.push_error(
                &scope,
                format!(
                    "shared avoid axial_clearance ({:.3} mm) must be finite and >= 0.",
                    ax.value_mm
                ),
            );
        }
    }

    // 2b. Validate multi-setup volumetric stock carryover references (must precede current setup)
    for (s_idx, setup) in project.setups.iter().enumerate() {
        if let Some(ref src_id) = setup.carryover_from_setup_id {
            let setup_scope = format!("Setup '{}'", setup.name);
            if src_id == &setup.id {
                report.push_error(
                    setup_scope,
                    "Stock carryover cannot reference the same Setup (self-reference).",
                );
            } else {
                match project.setups.iter().position(|s| &s.id == src_id) {
                    None => {
                        report.push_error(
                            setup_scope,
                            format!(
                                "Stock carryover source Setup '{src_id}' does not exist in project."
                            ),
                        );
                    }
                    Some(src_idx) if src_idx >= s_idx => {
                        report.push_error(
                            setup_scope,
                            format!(
                                "Stock carryover source Setup '{src_id}' (index {src_idx}) must precede '{}' (index {s_idx}) in setup order.",
                                setup.name
                            ),
                        );
                    }
                    Some(_) => {}
                }
            }
        }
    }

    // 3. Validate Setups & Operations against loaded CAD geometry (or Pure-2D empty CAD when no Part models exist)
    let empty_cad = CadDocument::empty();
    let has_part_models = project.primary_part_model().is_some();
    let effective_cad: Option<&CadDocument> = if let Some(cad_doc) = loaded_cad {
        Some(cad_doc)
    } else if !has_part_models {
        Some(&empty_cad)
    } else {
        None
    };

    let loaded_fixture_docs =
        if project_dir.is_some() && project.models.iter().any(|m| m.role == ModelRole::Fixture) {
            project.load_fixture_cad_documents(project_dir)
        } else {
            std::collections::BTreeMap::new()
        };

    if let Some(cad_doc) = effective_cad {
        for setup in &project.setups {
            let setup_scope = format!("Setup '{}'", setup.name);
            let resolved_setup = match setup.resolve_with_sketches(
                cad_doc,
                &project.sketches,
                project.display_units,
            ) {
                Ok(rs) => rs,
                Err(err) => {
                    report.push_error(setup_scope, err.to_string());
                    continue;
                }
            };

            // Validate stock bounds against active machine XYZ travel envelope
            let stock_size = resolved_setup.stock.aabb_wcs().size();
            let travel = &resolved_machine.travel_mm;
            if stock_size.x > travel.x_mm + 0.01
                || stock_size.y > travel.y_mm + 0.01
                || stock_size.z > travel.z_mm + 0.01
            {
                report.push_warning(
                    &setup_scope,
                    format!(
                        "stock dimensions ({:.1} × {:.1} × {:.1} mm) exceed machine '{}' travel envelope ({:.1} × {:.1} × {:.1} mm).",
                        stock_size.x,
                        stock_size.y,
                        stock_size.z,
                        resolved_machine.name,
                        travel.x_mm,
                        travel.y_mm,
                        travel.z_mm,
                    ),
                );
            }

            // Validate 4th-axis rotary kinematics, machine rotary capability, & post compatibility
            if let Some(rot) = resolved_setup.rotary {
                if rot.clearance_radius_mm <= rot.stock_outer_radius_mm + 1e-4 {
                    report.push_error(
                        &setup_scope,
                        format!(
                            "rotary clearance radius ({:.2} mm) must exceed stock outer radius ({:.2} mm)",
                            rot.clearance_radius_mm, rot.stock_outer_radius_mm
                        ),
                    );
                }
                match &resolved_machine.rotary {
                    None => {
                        report.push_warning(
                            &setup_scope,
                            format!(
                                "4th-axis rotary is active ({}-Axis), but machine '{}' is configured as 3-axis only.",
                                rot.axis_letter, resolved_machine.name
                            ),
                        );
                    }
                    Some(mrot) => {
                        let stock_dia = rot.stock_outer_radius_mm * 2.0;
                        let stock_len = (rot.stock_u_max - rot.stock_u_min).abs();
                        if stock_dia > mrot.max_stock_diameter_mm + 0.01
                            || stock_len > mrot.max_stock_length_mm + 0.01
                        {
                            report.push_warning(
                                &setup_scope,
                                format!(
                                    "rotary stock (Ø{:.1} × {:.1} mm) exceeds machine '{}' 4th-axis capacity (max Ø{:.1} × {:.1} mm).",
                                    stock_dia,
                                    stock_len,
                                    resolved_machine.name,
                                    mrot.max_stock_diameter_mm,
                                    mrot.max_stock_length_mm,
                                ),
                            );
                        }
                    }
                }
                if let Ok(prof) = chip_post::resolve_post_profile(&setup.post_profile, project_dir)
                    && !prof.rotary.enabled
                {
                    report.push_warning(
                        &setup_scope,
                        format!(
                            "4th-axis rotary is active ({}-Axis), but selected post profile '{}' is 3-axis only (consider 'linuxcnc_4axis', 'grblhal_4axis', or 'grbl_rotary_wrap').",
                            rot.axis_letter, prof.id
                        ),
                    );
                }
            }

            // Validate 3D Workholding fixtures (missing imported models, negative margins, vise grip depth, invalid dims)
            if !setup.fixtures.is_empty() {
                let stock_aabb = resolved_setup.stock.aabb_wcs();
                let fix_ctx = resolved_setup.eval_context(project.display_units);
                for fix in &setup.fixtures {
                    let fix_scope = format!("{setup_scope} -> Fixture '{}'", fix.name);
                    let mut eval_fix = fix.clone();
                    eval_fix.reevaluate(&fix_ctx);

                    if !eval_fix.clearance_margin.value_mm.is_finite()
                        || eval_fix.clearance_margin.value_mm < -1e-9
                    {
                        report.push_error(
                            &fix_scope,
                            format!(
                                "fixture clearance_margin ({:.3} mm) must be finite and >= 0",
                                eval_fix.clearance_margin.value_mm
                            ),
                        );
                    }
                    if !eval_fix.allowed_overcut.value_mm.is_finite()
                        || eval_fix.allowed_overcut.value_mm < -1e-9
                    {
                        report.push_error(
                            &fix_scope,
                            format!(
                                "fixture allowed_overcut ({:.3} mm) must be finite and >= 0",
                                eval_fix.allowed_overcut.value_mm
                            ),
                        );
                    }

                    match &eval_fix.kind {
                        FixtureKind::ViseJaws { grip_depth, .. } => {
                            if grip_depth.value_mm > stock_size.z + 1e-4 {
                                report.push_warning(
                                    &fix_scope,
                                    format!(
                                        "vise grip_depth ({:.2} mm) exceeds stock height ({:.2} mm); jaws rise above stock top.",
                                        grip_depth.value_mm, stock_size.z
                                    ),
                                );
                            }
                        }
                        FixtureKind::ImportedModel { model_id, .. } => {
                            match project.model_by_id(model_id) {
                                None => {
                                    report.push_error(
                                        &fix_scope,
                                        format!(
                                            "referenced fixture model_id '{model_id}' does not exist in project models"
                                        ),
                                    );
                                    continue;
                                }
                                Some(entry) if entry.role != ModelRole::Fixture => {
                                    report.push_warning(
                                        &fix_scope,
                                        format!(
                                            "model '{model_id}' is referenced as an imported fixture but has role 'part' (mark role = \"fixture\" to avoid polluting stock bounds)."
                                        ),
                                    );
                                }
                                Some(_) => {}
                            }
                            if project_dir.is_none() {
                                // Skip disk mesh read when validating purely in-memory project without project_dir
                                continue;
                            }
                        }
                        _ => {}
                    }

                    if eval_fix.enabled {
                        match resolve_fixture_body(&eval_fix, &stock_aabb, &fix_ctx, &mut |id| {
                            loaded_fixture_docs.get(id)
                        }) {
                            Ok(sol) => {
                                if setup.auto_avoid_fixtures_on_rapids && !sol.is_spoilboard {
                                    let req_z = sol.aabb_wcs.max.z + sol.clearance_margin_mm + 2.0;
                                    let z_span = req_z - stock_aabb.min.z;
                                    if z_span > travel.z_mm + 0.01 {
                                        report.push_warning(
                                            &fix_scope,
                                            format!(
                                                "safe-rapid clearance over fixture (Z = {req_z:+.1} mm, span {z_span:.1} mm above stock bottom) exceeds machine '{}' Z travel ({:.1} mm).",
                                                resolved_machine.name, travel.z_mm
                                            ),
                                        );
                                    }
                                }
                            }
                            Err(err) => {
                                report.push_error(&fix_scope, err.to_string());
                            }
                        }
                    }
                }
            }

            for (op_idx, op) in setup.operations.iter().enumerate() {
                if op.suppressed {
                    continue;
                }
                let op_scope = format!("Setup '{}' -> Op '{}'", setup.name, op.name);
                let mut eval_ctx = resolved_setup.eval_context(project.display_units);

                // Validate `FromPrecedingOperations` Rest Machining references (`05-operations.md`)
                if let RestMachiningMode::FromPrecedingOperations { source_op_ids } =
                    &op.passes.rest_machining.mode
                {
                    let any_preceding_enabled = setup.operations[..op_idx]
                        .iter()
                        .any(|prev| !prev.suppressed);
                    if !any_preceding_enabled {
                        report.push_error(
                            &op_scope,
                            "Rest Machining 'From Preceding Operations' requires at least one enabled preceding operation in the Setup.",
                        );
                    }
                    for ref_id in source_op_ids {
                        let found_before = setup.operations[..op_idx]
                            .iter()
                            .any(|prev| prev.id == *ref_id && !prev.suppressed);
                        if !found_before {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "Rest Machining source operation '{ref_id}' is missing, suppressed, or ordered after '{}'.",
                                    op.name
                                ),
                            );
                        }
                    }
                }

                // Validate per-operation & linked shared Avoid / Keep-Out regions (`M85`)
                {
                    let mut eval_avoid = op.avoid.clone();
                    if let Some(tid) = &op.tool_id
                        && let Some(tool) = project.tool(tid)
                    {
                        let mut avoid_ctx = eval_ctx.clone();
                        tool.populate_eval_context(&mut avoid_ctx);
                        eval_avoid.reevaluate(&avoid_ctx);
                    } else {
                        eval_avoid.reevaluate(&eval_ctx);
                    }
                    if !eval_avoid.radial_clearance.value_mm.is_finite()
                        || eval_avoid.radial_clearance.value_mm < -1e-9
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Avoid / Keep-Out radial_clearance ({:.3} mm) must be finite and >= 0.",
                                eval_avoid.radial_clearance.value_mm
                            ),
                        );
                    }
                    if !eval_avoid.axial_clearance.value_mm.is_finite()
                        || eval_avoid.axial_clearance.value_mm < -1e-9
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Avoid / Keep-Out axial_clearance ({:.3} mm) must be finite and >= 0.",
                                eval_avoid.axial_clearance.value_mm
                            ),
                        );
                    }
                    for fid in &op.avoid.fixture_ids {
                        if !setup.fixtures.iter().any(|f| f.id == *fid) {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "Avoid / Keep-Out references missing fixture_id '{fid}' in Setup '{}'.",
                                    setup.name
                                ),
                            );
                        }
                    }
                    for sid in &op.avoid.shared_region_ids {
                        match project.avoid_region(sid) {
                            None => {
                                report.push_error(
                                    &op_scope,
                                    format!(
                                        "Avoid / Keep-Out references missing shared_region_id '{sid}'."
                                    ),
                                );
                            }
                            Some(shared) if !shared.suppressed => {
                                for fid in &shared.fixture_ids {
                                    if !setup.fixtures.iter().any(|f| f.id == *fid) {
                                        report.push_error(
                                            &op_scope,
                                            format!(
                                                "Linked shared Avoid / Keep-Out '{}' references missing fixture_id '{fid}' in Setup '{}'.",
                                                shared.name, setup.name
                                            ),
                                        );
                                    }
                                }
                            }
                            Some(_) => {}
                        }
                    }
                }

                // Validate assigned tool & compatibility
                if let Some(tid) = &op.tool_id {
                    match project.tool(tid) {
                        Some(tool) => {
                            tool.populate_eval_context(&mut eval_ctx);
                            if !op.kind.is_compatible_with_tool(tool) {
                                report.push_error(
                                    &op_scope,
                                    format!(
                                        "cutter '{}' ({}) is incompatible with operation kind '{}'",
                                        tool.name,
                                        tool.geometry.kind_label(),
                                        op.kind.label()
                                    ),
                                );
                            }
                            let eff = op.effective_cutting_params(tool);
                            let sp = &resolved_machine.spindle;
                            if eff.spindle_rpm < sp.min_rpm - 0.5
                                || eff.spindle_rpm > sp.max_rpm + 0.5
                            {
                                report.push_warning(
                                    &op_scope,
                                    format!(
                                        "spindle speed {:.0} RPM is outside machine '{}' spindle range ({:.0}–{:.0} RPM).",
                                        eff.spindle_rpm,
                                        resolved_machine.name,
                                        sp.min_rpm,
                                        sp.max_rpm,
                                    ),
                                );
                            }
                            let kin = &resolved_machine.kinematics;
                            if eff.cutting_feed_mm_min > kin.max_cut_feed_xy_mm_min + 0.5 {
                                report.push_warning(
                                    &op_scope,
                                    format!(
                                        "cutting feedrate {:.0} mm/min exceeds machine '{}' XY limit ({:.0} mm/min).",
                                        eff.cutting_feed_mm_min,
                                        resolved_machine.name,
                                        kin.max_cut_feed_xy_mm_min,
                                    ),
                                );
                            }
                            if eff.plunge_feed_mm_min > kin.max_cut_feed_z_mm_min + 0.5 {
                                report.push_warning(
                                    &op_scope,
                                    format!(
                                        "plunge feedrate {:.0} mm/min exceeds machine '{}' Z limit ({:.0} mm/min).",
                                        eff.plunge_feed_mm_min,
                                        resolved_machine.name,
                                        kin.max_cut_feed_z_mm_min,
                                    ),
                                );
                            }
                        }
                        None => {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "referenced tool_id '{tid}' is not present in project tools"
                                ),
                            );
                        }
                    }
                } else {
                    report.push_warning(&op_scope, "no cutting tool assigned yet");
                }

                if let Some(ctid) = op.clearing_tool_id() {
                    match project.tool(ctid) {
                        Some(ctool) => {
                            if !chip_cam::cutter_supports_operation(
                                &ctool.geometry,
                                chip_cam::ToolOperationClass::VCarveFlatClearance,
                            ) {
                                report.push_error(
                                    &op_scope,
                                    format!(
                                        "secondary flat clearing tool '{}' ({}) must be a Flat Endmill",
                                        ctool.name,
                                        ctool.geometry.kind_label()
                                    ),
                                );
                            }
                        }
                        None => {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "referenced clearing_tool_id '{}' is not present in project tools",
                                    ctid.0
                                ),
                            );
                        }
                    }
                }

                if op.kind.is_3d_surface() && !cad_doc.has_3d_geometry() {
                    report.push_error(
                        &op_scope,
                        "3D surface operations require a loaded 3D STEP solid or STL/OBJ/3MF mesh.",
                    );
                    continue;
                }

                let assigned_tool = op.tool_id.as_ref().and_then(|tid| project.tool(tid));
                let push_cam_issues =
                    |report: &mut ProjectValidationReport, issues: Vec<CamValidationIssue>| {
                        for issue in issues {
                            match issue.severity {
                                CamValidationSeverity::Error => {
                                    report.push_error(&op_scope, issue.message);
                                }
                                CamValidationSeverity::Warning => {
                                    report.push_warning(&op_scope, issue.message);
                                }
                            }
                        }
                    };

                let (strategy_issues, abort_op) = validate_operation_strategy_and_rotary(
                    op,
                    assigned_tool,
                    &resolved_setup,
                    &eval_ctx,
                );
                push_cam_issues(&mut report, strategy_issues);
                if abort_op {
                    continue;
                }

                // Validate selections & Z-height hierarchy (in rotated virtual Setup for Indexed 3+1)
                let effective_setup = match (&op.rotary, resolved_setup.rotary) {
                    (OperationRotaryMode::Indexed3Plus1 { angle_deg }, Some(rot)) => Cow::Owned(
                        build_indexed_virtual_setup(&resolved_setup, &rot, *angle_deg),
                    ),
                    _ => Cow::Borrowed(&resolved_setup),
                };
                let summary_res = if op.kind.is_rotary_native() {
                    extract_rotary_selection_features(
                        cad_doc,
                        &project.sketches,
                        &effective_setup,
                        &op.selections,
                    )
                } else {
                    extract_selection_features_with_sketches(
                        cad_doc,
                        &project.sketches,
                        &effective_setup,
                        &op.selections,
                    )
                };
                let summary = match summary_res {
                    Ok(s) => s,
                    Err(err) => {
                        report.push_error(&op_scope, err.to_string());
                        continue;
                    }
                };

                let geom_issues = validate_operation_extracted_geometry(
                    op,
                    assigned_tool,
                    &resolved_setup,
                    &summary,
                    &eval_ctx,
                );
                push_cam_issues(&mut report, geom_issues);

                let (inf_top_z, inf_bot_z) =
                    if op.kind.is_3d_surface() && summary.features.is_empty() {
                        (
                            effective_setup.stock.aabb_wcs().max.z,
                            effective_setup.model_aabb_wcs.min.z,
                        )
                    } else {
                        (summary.inferred_top_z_wcs, summary.inferred_bottom_z_wcs)
                    };

                eval_ctx.set_var("feature_top_z", inf_top_z);
                eval_ctx.set_var("feature_bottom_z", inf_bot_z);
                if let Some(hd) = summary.primary_hole_diameter_mm {
                    eval_ctx.set_var("hole_d", hd);
                }

                let mut heights = op.heights.clone();
                heights.reevaluate(&eval_ctx);
                match ResolvedHeights::resolve(
                    &heights,
                    &effective_setup.model_aabb_wcs,
                    &effective_setup.stock.aabb_wcs(),
                    effective_setup.clearance_z_wcs,
                    effective_setup.retract_z_wcs,
                    inf_top_z,
                    inf_bot_z,
                ) {
                    Err(err) => {
                        report.push_error(&op_scope, err.to_string());
                    }
                    Ok(rh) => {
                        let depth_issues =
                            validate_operation_undercut_depth(op, assigned_tool, &rh);
                        push_cam_issues(&mut report, depth_issues);
                    }
                }
            }
        }
    } else if has_part_models && project_dir.is_none() {
        report.push_warning(
            "Project",
            "CAD document not loaded in memory; skipped B-Rep WCS and selection checks.",
        );
    }

    report
}

/// Project-wide physics-backed machinability & tool-breakage report across all Setups.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectMachinabilityReport {
    pub project_name: String,
    pub machine_id: String,
    pub machine_name: String,
    pub material_id: String,
    pub material_name: String,
    pub severity: MachinabilitySeverity,
    pub critical_count: usize,
    pub warning_count: usize,
    pub info_count: usize,
    pub setups: Vec<SetupMachinabilityReport>,
}

/// Analyzes cutting physics (spindle load, MRR, chip thinning, stepped cantilever deflection &
/// carbide bending stress, and helical bore hazards) across every Setup and Operation in `project`.
pub fn analyze_project_machinability(
    project: &ProjectDocument,
    machine: &MachineDefinition,
    cad_doc: Option<&CadDocument>,
) -> ProjectMachinabilityReport {
    let material = project.effective_workpiece_material();
    let mut setups = Vec::with_capacity(project.setups.len());
    let mut severity = MachinabilitySeverity::Ok;
    let mut critical_count = 0usize;
    let mut warning_count = 0usize;
    let mut info_count = 0usize;

    for setup in &project.setups {
        let rep = analyze_setup_machinability(
            setup,
            &project.tools,
            &material,
            machine,
            cad_doc,
            &project.sketches,
            project.display_units,
        );
        severity = severity.max(rep.severity);
        critical_count += rep.critical_count;
        warning_count += rep.warning_count;
        info_count += rep.info_count;
        setups.push(rep);
    }

    ProjectMachinabilityReport {
        project_name: project.project_name.clone(),
        machine_id: machine.id.clone(),
        machine_name: machine.name.clone(),
        material_id: material.id,
        material_name: material.name,
        severity,
        critical_count,
        warning_count,
        info_count,
        setups,
    }
}

/// Fast in-memory machine travel envelope, rotary capacity, spindle RPM, and XY/Z feedrate limit
/// check for a single [`Setup`] without touching disk or running full B-Rep extraction.
pub fn validate_setup_machine_limits(
    project: &ProjectDocument,
    setup: &Setup,
    machine: &MachineDefinition,
    loaded_cad: Option<&CadDocument>,
) -> Vec<String> {
    let mut warnings = Vec::new();
    let empty_cad = CadDocument::empty();
    let cad = loaded_cad.unwrap_or(&empty_cad);
    if let Ok(resolved) = setup.resolve_with_sketches(cad, &project.sketches, project.display_units)
    {
        let sb = resolved.stock.aabb_wcs();
        let sx = sb.max.x - sb.min.x;
        let sy = sb.max.y - sb.min.y;
        let sz = sb.max.z - sb.min.z;
        let tr = &machine.travel_mm;
        if sx > tr.x_mm + 0.01 || sy > tr.y_mm + 0.01 || sz > tr.z_mm + 0.01 {
            warnings.push(format!(
                "Stock ({sx:.1} × {sy:.1} × {sz:.1} mm) exceeds '{}' travel envelope ({:.0} × {:.0} × {:.0} mm).",
                machine.name, tr.x_mm, tr.y_mm, tr.z_mm
            ));
        }
        if let Some(rot) = resolved.rotary {
            match &machine.rotary {
                None => warnings.push(format!(
                    "4th-axis rotary is active ({}-Axis), but '{}' is 3-axis only.",
                    rot.axis_letter, machine.name
                )),
                Some(mrot) => {
                    let dia = rot.stock_outer_radius_mm * 2.0;
                    let len = (rot.stock_u_max - rot.stock_u_min).abs();
                    if dia > mrot.max_stock_diameter_mm + 0.01
                        || len > mrot.max_stock_length_mm + 0.01
                    {
                        warnings.push(format!(
                            "Rotary stock (Ø{dia:.1} × {len:.1} mm) exceeds '{}' capacity (Ø{:.1} × {:.1} mm).",
                            machine.name, mrot.max_stock_diameter_mm, mrot.max_stock_length_mm
                        ));
                    }
                }
            }
        }
    }
    let sp = &machine.spindle;
    let kin = &machine.kinematics;
    for op in &setup.operations {
        if op.suppressed {
            continue;
        }
        let Some(tid) = &op.tool_id else {
            continue;
        };
        let Some(tool) = project.tool(tid) else {
            continue;
        };
        let eff = op.effective_cutting_params(tool);
        if eff.spindle_rpm < sp.min_rpm - 0.5 || eff.spindle_rpm > sp.max_rpm + 0.5 {
            warnings.push(format!(
                "{}: {:.0} RPM outside '{}' range ({:.0}–{:.0} RPM).",
                op.name, eff.spindle_rpm, machine.name, sp.min_rpm, sp.max_rpm
            ));
        }
        if eff.cutting_feed_mm_min > kin.max_cut_feed_xy_mm_min + 0.5 {
            warnings.push(format!(
                "{}: feed {:.0} mm/min exceeds '{}' XY limit ({:.0} mm/min).",
                op.name, eff.cutting_feed_mm_min, machine.name, kin.max_cut_feed_xy_mm_min
            ));
        }
        if eff.plunge_feed_mm_min > kin.max_cut_feed_z_mm_min + 0.5 {
            warnings.push(format!(
                "{}: plunge {:.0} mm/min exceeds '{}' Z limit ({:.0} mm/min).",
                op.name, eff.plunge_feed_mm_min, machine.name, kin.max_cut_feed_z_mm_min
            ));
        }
    }
    warnings
}

/// Evaluates cutting-mechanics telemetry and advisory findings for a single operation in `project`.
pub fn evaluate_operation_machinability(
    project: &ProjectDocument,
    setup_idx: usize,
    op_idx: usize,
    global_tools: &ToolLibrary,
    active_machine: Option<&MachineDefinition>,
    resolved_setup: Option<&ResolvedSetup>,
    cad_doc: Option<&CadDocument>,
) -> Option<OperationMachinabilityReport> {
    let setup = project.setups.get(setup_idx)?;
    let op = setup.operations.get(op_idx)?;
    let tool = op
        .tool_id
        .as_ref()
        .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)));
    let material = project.effective_workpiece_material();
    let fallback_machine;
    let machine = match active_machine {
        Some(m) => m,
        None => {
            fallback_machine = builtin_generic_machine();
            &fallback_machine
        }
    };
    let ctx = OperationMachinabilityContext {
        setup_id: &setup.id,
        setup_name: &setup.name,
        resolved_setup,
        cad_doc,
        sketches: &project.sketches,
        material: &material,
        machine,
        display_unit: project.display_units,
    };
    Some(analyze_operation_machinability(op, tool, &ctx))
}

/// Evaluates cutting-mechanics telemetry across all operations in `project.setups[setup_idx]`.
pub fn evaluate_setup_machinability(
    project: &ProjectDocument,
    setup_idx: usize,
    global_tools: &ToolLibrary,
    active_machine: Option<&MachineDefinition>,
    cad_doc: Option<&CadDocument>,
) -> Option<SetupMachinabilityReport> {
    let setup = project.setups.get(setup_idx)?;
    let mut merged_tools: Vec<Tool> = project.tools.clone();
    for gt in &global_tools.tools {
        if !merged_tools.iter().any(|t| t.id == gt.id) {
            merged_tools.push(gt.clone());
        }
    }
    let material = project.effective_workpiece_material();
    let fallback_machine;
    let machine = match active_machine {
        Some(m) => m,
        None => {
            fallback_machine = builtin_generic_machine();
            &fallback_machine
        }
    };
    Some(analyze_setup_machinability(
        setup,
        &merged_tools,
        &material,
        machine,
        cad_doc,
        &project.sketches,
        project.display_units,
    ))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{CadKernel, MonstertruckKernel};
    use chip_cam::{
        Deburr3DParams, Operation, OperationKind, Setup, StrategyParams, WorkOffsetSlot,
    };
    use chip_fixtures::sample_overhang_flange_step;
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, ParamLength};
    use chip_tools::{CutterGeometry, default_starter_tool_library, starter_lollipop_6mm};

    use super::*;

    #[test]
    fn validates_deburr3d_tool_compatibility_underside_lollipop_and_clearances() {
        let step = sample_overhang_flange_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("flange.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let lib = default_starter_tool_library();
        let flat_6 = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
            .unwrap()
            .clone();
        let chamfer_90 = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
            .unwrap()
            .clone();
        let lolli_6 = starter_lollipop_6mm();

        let mut proj = ProjectDocument::new("Deburr3D Validation");
        proj.tools = vec![flat_6.clone(), chamfer_90.clone(), lolli_6.clone()];
        proj.setups = vec![Setup::new("s1", "Setup 1", WorkOffsetSlot::G54)];

        // 1. FlatEndmill is incompatible with Deburr3D
        let mut op = Operation::new("op_d", "3D Deburr", OperationKind::Deburr3D);
        op.tool_id = Some(flat_6.id.clone());
        proj.setups[0].operations = vec![op];
        let rep = validate_project(&proj, None, Some(&doc));
        assert!(rep.error_count() >= 1);

        // 2. ChamferMill with allow_underside_back_chamfer = true must error (requires Lollipop)
        proj.setups[0].operations[0].tool_id = Some(chamfer_90.id.clone());
        proj.setups[0].operations[0].strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            allow_underside_back_chamfer: true,
            ..Deburr3DParams::default()
        }));
        let rep2 = validate_project(&proj, None, Some(&doc));
        assert!(
            rep2.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Error
                    && d.message.contains("Lollipop Endmill")),
            "expected underside Lollipop error, got: {:?}",
            rep2.diagnostics
        );

        // 3. LollipopEndmill with allow_underside_back_chamfer = true and valid parameters passes!
        proj.setups[0].operations[0].tool_id = Some(lolli_6.id.clone());
        let rep3 = validate_project(&proj, None, Some(&doc));
        assert!(
            rep3.is_valid(),
            "valid Lollipop Deburr3D should have 0 errors, got: {:?}",
            rep3.diagnostics
        );

        // 4. Non-positive chamfer_width & negative wall_clearance must fail validation
        proj.setups[0].operations[0].strategy = Some(StrategyParams::Deburr3D(Deburr3DParams {
            chamfer_width: ParamLength::from_mm(0.0),
            wall_clearance: ParamLength::from_mm(-0.1),
            ..Deburr3DParams::default()
        }));
        let rep4 = validate_project(&proj, None, Some(&doc));
        assert!(rep4.error_count() >= 2);
    }

    #[test]
    fn validates_m66_undercut_operations_clearances_angles_and_tool_compatibility() {
        use chip_cam::{DovetailParams, SlotUndercutParams, SlotUndercutPreset, Undercut3DParams};
        use chip_tools::{starter_dovetail_60deg_12mm, starter_tslot_12mm};

        let step = sample_overhang_flange_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("flange.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let slot_12 = starter_tslot_12mm();
        let dove_60 = starter_dovetail_60deg_12mm();
        let lolli_6 = starter_lollipop_6mm();

        let mut proj = ProjectDocument::new("M66 Undercut Validation");
        proj.tools = vec![slot_12.clone(), dove_60.clone(), lolli_6.clone()];
        proj.setups = vec![Setup::new("s1", "Setup 1", WorkOffsetSlot::G54)];

        // 1. SlotUndercut with narrow throat <= neck_diameter fails validation
        let mut slot_p = SlotUndercutParams::default();
        slot_p.apply_preset(SlotUndercutPreset::TSlotM8);
        slot_p.throat_width = ParamLength::from_mm(4.5);
        let mut op_slot = Operation::new("op_slot", "T-Slot", OperationKind::SlotUndercut);
        op_slot.tool_id = Some(slot_12.id.clone());
        op_slot.strategy = Some(StrategyParams::SlotUndercut(slot_p));

        // 2. Dovetail with 45° target angle on 60° cutter fails validation
        let mut op_dove = Operation::new("op_dove", "Dovetail", OperationKind::Dovetail);
        op_dove.tool_id = Some(dove_60.id.clone());
        op_dove.strategy = Some(StrategyParams::Dovetail(DovetailParams {
            side_angle_deg: 45.0,
            ..DovetailParams::default()
        }));

        // 3. Undercut3D with valid Lollipop passes validation!
        let mut op_u3d = Operation::new("op_u3d", "3D Undercut", OperationKind::Undercut3D);
        op_u3d.tool_id = Some(lolli_6.id.clone());
        op_u3d.strategy = Some(StrategyParams::Undercut3D(Undercut3DParams::default()));

        proj.setups[0].operations = vec![op_slot, op_dove, op_u3d];
        let rep = validate_project(&proj, None, Some(&doc));
        assert!(
            rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Error
                    && d.message.contains("neck diameter")),
            "expected slot neck error, got: {:?}",
            rep.diagnostics
        );
        assert!(
            rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Error
                    && d.message.contains("mismatches target dovetail angle")),
            "expected dovetail angle mismatch error, got: {:?}",
            rep.diagnostics
        );
        assert!(
            !rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Error && d.scope.contains("3D Undercut")),
            "valid Undercut3D should have zero errors, got: {:?}",
            rep.diagnostics
        );

        // 4. Verify lossless .chip.toml serialization roundtrip for all 3 M66 operations!
        let toml_txt = proj.to_toml_string().unwrap();
        let parsed = ProjectDocument::from_toml_str(&toml_txt).unwrap();
        assert_eq!(parsed.setups[0].operations.len(), 3);
        assert_eq!(
            parsed.setups[0].operations[0].kind,
            OperationKind::SlotUndercut
        );
        assert_eq!(parsed.setups[0].operations[1].kind, OperationKind::Dovetail);
        assert_eq!(
            parsed.setups[0].operations[2].kind,
            OperationKind::Undercut3D
        );
        assert_eq!(parsed.setups[0].operations, proj.setups[0].operations);
    }

    #[test]
    fn m80_toml_backward_compatibility_and_selected_regions_validation_warning() {
        use chip_cam::{
            GeometrySelectionItem, MachiningBoundaryConfig, MachiningBoundaryMode,
            ToolContainmentMode, Undercut3DParams,
        };
        use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

        // 1. Legacy flat TOML keys deserialize losslessly into `MachiningBoundaryConfig` for all 5 3D strategies
        let legacy_3d_toml = r#"
strategy_kind = "adaptive_3d"
boundary_mode = "model_silhouette"
tool_containment = "tool_inside_boundary"
boundary_offset = { value_mm = -1.5 }
"#;
        let parsed_a3d: StrategyParams = toml::from_str(legacy_3d_toml).unwrap();
        let b = parsed_a3d.machining_boundary().unwrap();
        assert_eq!(b.boundary_mode, MachiningBoundaryMode::ModelSilhouette);
        assert_eq!(b.tool_containment, ToolContainmentMode::ToolInsideBoundary);
        assert!((b.boundary_offset.value_mm - (-1.5)).abs() < 1e-9);
        // Re-serialization preserves flat key names (`boundary_mode`, `tool_containment`, `boundary_offset`)
        let re_toml = toml::to_string(&parsed_a3d).unwrap();
        assert!(re_toml.contains("boundary_mode = \"model_silhouette\""));
        assert!(re_toml.contains("tool_containment = \"tool_inside_boundary\""));
        assert!(re_toml.contains("boundary_offset"));

        // 2. Legacy Undercut3D & Deburr3D TOML without boundary keys load with clean defaults
        let legacy_u3d = r#"
strategy_kind = "undercut_3d"
z_stepdown = { value_mm = 0.3 }
"#;
        let parsed_u3d: StrategyParams = toml::from_str(legacy_u3d).unwrap();
        assert_eq!(
            *parsed_u3d.machining_boundary().unwrap(),
            MachiningBoundaryConfig::new(
                MachiningBoundaryMode::ModelSilhouette,
                ToolContainmentMode::ToolOutsideBoundary,
                0.0
            )
        );

        let legacy_deb = r#"
strategy_kind = "deburr_3d"
chamfer_width = { value_mm = 0.4 }
auto_detect_edges = true
"#;
        let parsed_deb: StrategyParams = toml::from_str(legacy_deb).unwrap();
        assert_eq!(
            *parsed_deb.machining_boundary().unwrap(),
            MachiningBoundaryConfig::new(
                MachiningBoundaryMode::ModelSilhouette,
                ToolContainmentMode::ToolOutsideBoundary,
                0.0
            )
        );

        // Explicit keys on Undercut3D / Deburr3D roundtrip losslessly
        let explicit_u3d = StrategyParams::Undercut3D(Undercut3DParams {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::SelectedRegions,
                ToolContainmentMode::ToolInsideBoundary,
                0.75,
            ),
            ..Undercut3DParams::default()
        });
        let u3d_txt = toml::to_string(&explicit_u3d).unwrap();
        assert!(u3d_txt.contains("boundary_mode = \"selected_regions\""));
        let u3d_rt: StrategyParams = toml::from_str(&u3d_txt).unwrap();
        assert_eq!(u3d_rt, explicit_u3d);

        // 3. Validation emits a non-blocking warning when SelectedRegions has zero 2D/planar selections
        let step = sample_overhang_flange_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("flange.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let lolli_6 = starter_lollipop_6mm();

        let mut proj = ProjectDocument::new("M80 Boundary Validation");
        proj.tools = vec![lolli_6.clone()];
        proj.setups = vec![Setup::new("s1", "Setup 1", WorkOffsetSlot::G54)];

        let mut op = Operation::new("op_u3d", "3D Undercut", OperationKind::Undercut3D);
        op.tool_id = Some(lolli_6.id.clone());
        op.machining_boundary_mut().unwrap().boundary_mode = MachiningBoundaryMode::SelectedRegions;
        proj.setups[0].operations = vec![op];

        let rep_warn = validate_project(&proj, None, Some(&doc));
        assert!(
            rep_warn.is_valid(),
            "missing SelectedRegions must be a non-blocking warning, not error: {:?}",
            rep_warn.diagnostics
        );
        assert!(
            rep_warn.diagnostics.iter().any(|d| {
                d.severity == DiagnosticSeverity::Warning
                    && d.message.contains("Selected 2D Contours")
            }),
            "expected SelectedRegions warning, got: {:?}",
            rep_warn.diagnostics
        );

        // Selecting a closed 2D sketch contour clears the warning!
        let mut sk = SketchLayer::new("sk_1", "Window", "s1");
        sk.entities.push(SketchEntityEntry {
            id: "r1".to_string(),
            name: "Rect".to_string(),
            is_construction: false,
            sub_layer: Some("DEFAULT".to_string()),
            entity: SketchEntity::Rectangle {
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(30.0),
                height: ParamLength::from_mm(30.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
                anchor: RectAnchorMode::Center,
            },
        });
        sk.rebuild_regions(None);
        proj.sketches.push(sk);
        proj.setups[0].operations[0]
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_1".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        let rep_clean = validate_project(&proj, None, Some(&doc));
        assert!(rep_clean.is_valid());
        assert!(
            !rep_clean.diagnostics.iter().any(|d| {
                d.severity == DiagnosticSeverity::Warning
                    && d.message.contains("Selected 2D Contours")
            }),
            "SelectedRegions warning should clear once a closed contour is selected: {:?}",
            rep_clean.diagnostics
        );
    }

    #[test]
    fn m81_validates_rotary_axial_boundary_warnings_and_legacy_toml_compatibility() {
        use chip_cad::{MeshImportOptions, load_mesh_cad_document};
        use chip_cam::{
            BoxAnchor27, GeometrySelectionItem, HeightReference, HeightSpec,
            RotaryAxialBoundaryMode, RotaryAxisKind, RotaryDriveMode, StockAxis, StockConfig,
            ToolContainmentMode, WcsOriginMode,
        };
        use chip_fixtures::sample_rotary_shaft_stl_bytes;
        use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();

        let lib = default_starter_tool_library();
        let ball_6 = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::BallEndmill { .. }))
            .unwrap()
            .clone();
        let flat_6 = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
            .unwrap()
            .clone();

        let mut proj = ProjectDocument::new("M81 Rotary Validation");
        proj.tools = vec![ball_6.clone(), flat_6.clone()];
        let mut setup = Setup::new("s_rot", "Rotary Setup", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        setup.rotary.chuck_length = ParamLength::from_mm(20.0);

        // Sketch intruding into -U chuck AND wide enough to exceed 360°
        let mut sk = SketchLayer::new("sk_bad_wrap", "Bad Wrap", setup.id.clone());
        sk.entities.push(SketchEntityEntry {
            id: "r_bad".to_string(),
            name: "Bad Rect".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Rectangle {
                anchor: RectAnchorMode::Center,
                x: ParamLength::from_mm(-36.0),
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(12.0),
                height: ParamLength::from_mm(150.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
        });
        sk.rebuild_regions(None);
        proj.sketches = vec![sk];

        // 1. Rotary2Linear with SelectedGeometry & empty selections -> warning
        let mut op_sel_empty =
            Operation::new("op_1", "Rot Empty Sel", OperationKind::Rotary2Linear);
        op_sel_empty.tool_id = Some(ball_6.id.clone());
        op_sel_empty
            .rotary_axial_boundary_mut()
            .unwrap()
            .axial_boundary_mode = RotaryAxialBoundaryMode::SelectedGeometry;

        // 2. Rotary4AxisSim with ExplicitRange & u_start >= u_end -> warning
        let mut op_exp_bad = Operation::new("op_2", "Rot Bad Range", OperationKind::Rotary4AxisSim);
        op_exp_bad.tool_id = Some(ball_6.id.clone());
        {
            let ab = op_exp_bad.rotary_axial_boundary_mut().unwrap();
            ab.axial_boundary_mode = RotaryAxialBoundaryMode::ExplicitRange;
            ab.explicit_u_start = ParamLength::from_mm(15.0);
            ab.explicit_u_end = ParamLength::from_mm(5.0);
        }

        // 3. WrapCylinder intruding into chuck keep-out AND > 360° -> warnings
        let mut op_wrap_bad = Operation::new("op_3", "Wrap Bad", OperationKind::Pocket2D);
        op_wrap_bad.tool_id = Some(flat_6.id.clone());
        op_wrap_bad
            .selections
            .push(GeometrySelectionItem::SketchLayerAll {
                sketch_id: "sk_bad_wrap".to_string(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            });
        op_wrap_bad.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_wrap_bad.heights.bottom = HeightSpec::new(HeightReference::StockTop, -2.0);
        op_wrap_bad.rotary = OperationRotaryMode::WrapCylinder {
            wrap_radius: ParamLength::from_mm(20.0),
            axial_offset: ParamLength::from_mm(0.0),
            angle_offset_deg: 0.0,
        };

        setup.operations = vec![op_sel_empty, op_exp_bad, op_wrap_bad];
        proj.setups = vec![setup];

        let rep = validate_project(&proj, None, Some(&doc));
        assert!(
            rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Warning
                    && d.message.contains("Selected Geometry")),
            "expected SelectedGeometry empty warning, got: {:?}",
            rep.diagnostics
        );
        assert!(
            rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Warning
                    && d.message.contains("explicit axial range")),
            "expected ExplicitRange u_start >= u_end warning, got: {:?}",
            rep.diagnostics
        );
        assert!(
            rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Warning
                    && d.message.contains("chuck keep-out")),
            "expected WrapCylinder chuck keep-out warning, got: {:?}",
            rep.diagnostics
        );
        assert!(
            rep.diagnostics
                .iter()
                .any(|d| d.severity == DiagnosticSeverity::Warning && d.message.contains("360°")),
            "expected WrapCylinder > 360° warning, got: {:?}",
            rep.diagnostics
        );

        // 4. Verify lossless TOML roundtrip with flattened `RotaryAxialBoundaryConfig`
        let toml_txt = proj.to_toml_string().unwrap();
        let reparsed = ProjectDocument::from_toml_str(&toml_txt).unwrap();
        assert_eq!(reparsed.setups[0].operations, proj.setups[0].operations);

        // 5. Verify 100% backward-compatible deserialization of legacy TOML lacking new M81 fields
        let legacy_2lin_toml = r#"
strategy_kind = "rotary_2linear"
pattern = "continuous_spiral"
cut_direction = "zig_zag"
use_cusp_height = true
start_angle_deg = -45.0
end_angle_deg = 135.0
angular_sample_deg = 2.0

[stepover]
value_mm = 1.5

[cusp_height]
value_mm = 0.02

[axial_margin_start]
value_mm = 34.0

[axial_margin_end]
value_mm = 8.0

[axial_sample_step]
value_mm = 0.25

[smoothing_tolerance]
value_mm = 0.01
"#;
        let parsed_2lin: StrategyParams = toml::from_str(legacy_2lin_toml).unwrap();
        match parsed_2lin {
            StrategyParams::Rotary2Linear(p) => {
                assert_eq!(
                    p.axial_boundary.axial_boundary_mode,
                    RotaryAxialBoundaryMode::ModelBounds
                );
                assert_eq!(
                    p.axial_boundary.axial_containment,
                    ToolContainmentMode::ToolCenterOnBoundary
                );
                assert!((p.axial_boundary.start_angle_deg - (-45.0)).abs() < 1e-9);
                assert!((p.axial_boundary.end_angle_deg - 135.0).abs() < 1e-9);
                assert!((p.axial_boundary.axial_margin_start.value_mm - 34.0).abs() < 1e-9);
                assert!((p.axial_boundary.axial_margin_end.value_mm - 8.0).abs() < 1e-9);
            }
            other => panic!("expected Rotary2Linear, got {other:?}"),
        }

        let legacy_4ax_toml = r#"
strategy_kind = "rotary_4axis_sim"
pattern = "simultaneous_spiral"
cut_direction = "zig_zag"
tilt_mode = "surface_normal_auto_tilt"
max_tilt_deg = 18.0
tilt_smoothing_window = 3
use_cusp_height = false
start_angle_deg = 0.0
end_angle_deg = 360.0
angular_sample_deg = 1.5

[lateral_offset]
value_mm = 2.5

[stepover]
value_mm = 2.0

[cusp_height]
value_mm = 0.02

[axial_margin_start]
value_mm = 11.0

[axial_margin_end]
value_mm = 11.0

[axial_sample_step]
value_mm = 0.25

[smoothing_tolerance]
value_mm = 0.01
"#;
        let parsed_4ax: StrategyParams = toml::from_str(legacy_4ax_toml).unwrap();
        match parsed_4ax {
            StrategyParams::Rotary4AxisSim(p) => {
                assert_eq!(
                    p.axial_boundary.axial_boundary_mode,
                    RotaryAxialBoundaryMode::ModelBounds
                );
                assert_eq!(
                    p.axial_boundary.axial_containment,
                    ToolContainmentMode::ToolCenterOnBoundary
                );
                assert!((p.axial_boundary.axial_margin_start.value_mm - 11.0).abs() < 1e-9);
                assert!((p.axial_boundary.axial_margin_end.value_mm - 11.0).abs() < 1e-9);
            }
            other => panic!("expected Rotary4AxisSim, got {other:?}"),
        }
    }

    #[test]
    fn m82_25d_cutting_area_params_roundtrip_and_legacy_toml_compat() {
        use chip_cam::{
            Contour2DParams, FacingParams, MachiningBoundaryConfig, MachiningBoundaryMode,
            Pocket2DParams, PocketBoundaryMode, StrategyParams, ToolContainmentMode,
        };
        use chip_math::ParamLength;

        // 1. Roundtrip FacingParams with flattened MachiningBoundaryConfig + finishing_stepdown
        let face = StrategyParams::Facing(FacingParams {
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::SelectedRegions,
                ToolContainmentMode::ToolInsideBoundary,
                -1.5,
            ),
            finishing_stepdown: Some(ParamLength::from_mm(0.35)),
            ..FacingParams::default()
        });
        let face_toml = toml::to_string(&face).unwrap();
        let face_rt: StrategyParams = toml::from_str(&face_toml).unwrap();
        assert_eq!(face_rt, face);

        // 2. Roundtrip Pocket2DParams with boss-clearing & open-outer flags
        let pkt = StrategyParams::Pocket2D(Pocket2DParams {
            boundary_mode: PocketBoundaryMode::ClearOutsideSelectedToStock,
            treat_outer_boundary_as_open: true,
            ..Pocket2DParams::default()
        });
        let pkt_toml = toml::to_string(&pkt).unwrap();
        let pkt_rt: StrategyParams = toml::from_str(&pkt_toml).unwrap();
        assert_eq!(pkt_rt, pkt);

        // 3. Roundtrip Contour2DParams with stock clipping & open chain extension
        let cnt = StrategyParams::Contour2D(Contour2DParams {
            clip_to_stock_bounds: true,
            open_chain_extension: ParamLength::from_mm(4.5),
            ..Contour2DParams::default()
        });
        let cnt_toml = toml::to_string(&cnt).unwrap();
        let cnt_rt: StrategyParams = toml::from_str(&cnt_toml).unwrap();
        assert_eq!(cnt_rt, cnt);

        // 4. Legacy TOML omitting all M82 fields deserializes cleanly
        let legacy_face: StrategyParams = toml::from_str(
            r#"
strategy_kind = "facing"
auto_longest_axis = true
cut_mode = "both_ways_zig_zag"
"#,
        )
        .unwrap();
        match legacy_face {
            StrategyParams::Facing(p) => {
                assert_eq!(p.boundary.boundary_mode, MachiningBoundaryMode::StockBox);
                assert!((p.boundary.boundary_offset.value_mm - 0.0).abs() < 1e-9);
            }
            other => panic!("expected Facing, got {other:?}"),
        }

        let legacy_pkt: StrategyParams = toml::from_str("strategy_kind = \"pocket_2d\"\n").unwrap();
        match legacy_pkt {
            StrategyParams::Pocket2D(p) => {
                assert_eq!(p.boundary_mode, PocketBoundaryMode::SelectedPockets);
                assert!(!p.treat_outer_boundary_as_open);
            }
            other => panic!("expected Pocket2D, got {other:?}"),
        }

        let legacy_cnt: StrategyParams =
            toml::from_str("strategy_kind = \"contour_2d\"\n").unwrap();
        match legacy_cnt {
            StrategyParams::Contour2D(p) => {
                assert!(!p.clip_to_stock_bounds);
                assert!((p.open_chain_extension.value_mm - 0.0).abs() < 1e-9);
            }
            other => panic!("expected Contour2D, got {other:?}"),
        }
    }

    #[test]
    fn m84_model_bounding_box_toml_roundtrip_all_8_strategies_and_cache_hash_invalidation() {
        use chip_cam::{
            Adaptive3DParams, Deburr3DParams, FacingParams, MachiningBoundaryConfig,
            MachiningBoundaryMode, Parallel3DParams, Pencil3DParams, Scallop3DParams,
            StrategyParams, ToolContainmentMode, Undercut3DParams, Waterline3DParams,
        };
        use chip_math::ParamLength;

        use crate::cache::compute_operation_input_hash;

        let mut custom_boundary = MachiningBoundaryConfig::new(
            MachiningBoundaryMode::ModelBoundingBox,
            ToolContainmentMode::ToolOutsideBoundary,
            1.25,
        );
        custom_boundary.model_bounds_offset_x_neg = ParamLength::from_mm(2.5);
        custom_boundary.model_bounds_offset_x_pos = ParamLength::from_mm(-1.75);
        custom_boundary.model_bounds_offset_y_neg = ParamLength::from_mm(0.5);
        custom_boundary.model_bounds_offset_y_pos = ParamLength::from_mm(-3.25);

        let all_eight: Vec<(&str, StrategyParams)> = vec![
            (
                "facing",
                StrategyParams::Facing(FacingParams {
                    boundary: custom_boundary.clone(),
                    ..FacingParams::default()
                }),
            ),
            (
                "adaptive_3d",
                StrategyParams::Adaptive3D(Adaptive3DParams {
                    boundary: custom_boundary.clone(),
                    ..Adaptive3DParams::default()
                }),
            ),
            (
                "waterline_3d",
                StrategyParams::Waterline3D(Waterline3DParams {
                    boundary: custom_boundary.clone(),
                    ..Waterline3DParams::default()
                }),
            ),
            (
                "parallel_3d",
                StrategyParams::Parallel3D(Parallel3DParams {
                    boundary: custom_boundary.clone(),
                    ..Parallel3DParams::default()
                }),
            ),
            (
                "scallop_3d",
                StrategyParams::Scallop3D(Scallop3DParams {
                    boundary: custom_boundary.clone(),
                    ..Scallop3DParams::default()
                }),
            ),
            (
                "pencil_3d",
                StrategyParams::Pencil3D(Pencil3DParams {
                    boundary: custom_boundary.clone(),
                    ..Pencil3DParams::default()
                }),
            ),
            (
                "undercut_3d",
                StrategyParams::Undercut3D(Undercut3DParams {
                    boundary: custom_boundary.clone(),
                    ..Undercut3DParams::default()
                }),
            ),
            (
                "deburr_3d",
                StrategyParams::Deburr3D(Deburr3DParams {
                    boundary: custom_boundary.clone(),
                    ..Deburr3DParams::default()
                }),
            ),
        ];

        for (tag, strat) in all_eight {
            let txt = toml::to_string(&strat).unwrap();
            assert!(
                txt.contains("boundary_mode = \"model_bounding_box\""),
                "{tag} TOML must serialize snake_case model_bounding_box"
            );
            let rt: StrategyParams = toml::from_str(&txt).unwrap();
            assert_eq!(
                rt, strat,
                "{tag} must roundtrip ModelBoundingBox + all 4 per-side offsets losslessly"
            );

            // Legacy TOML without `model_bounds_offset_*` defaults all 4 to 0.0 mm
            let legacy = format!("strategy_kind = \"{tag}\"\n");
            let parsed_legacy: StrategyParams = toml::from_str(&legacy).unwrap();
            let b = parsed_legacy.machining_boundary().unwrap();
            assert!((b.model_bounds_offset_x_neg.value_mm - 0.0).abs() < 1e-9);
            assert!((b.model_bounds_offset_x_pos.value_mm - 0.0).abs() < 1e-9);
            assert!((b.model_bounds_offset_y_neg.value_mm - 0.0).abs() < 1e-9);
            assert!((b.model_bounds_offset_y_pos.value_mm - 0.0).abs() < 1e-9);
        }

        // OperationInputHash invalidates when switching to ModelBoundingBox or editing any of the 4 offsets
        let mut proj = ProjectDocument::new("M84 Cache Hash");
        let op = Operation::new("op1", "3D Adaptive", OperationKind::Adaptive3D);
        proj.setups[0].operations = vec![op];
        let h_base = compute_operation_input_hash(&proj, 0, 0, 1).unwrap();

        proj.setups[0].operations[0]
            .machining_boundary_mut()
            .unwrap()
            .boundary_mode = MachiningBoundaryMode::ModelBoundingBox;
        let h_mb = compute_operation_input_hash(&proj, 0, 0, 1).unwrap();
        assert_ne!(h_base, h_mb);

        let mut prev_h = h_mb;
        for side in 0..4 {
            let b = proj.setups[0].operations[0]
                .machining_boundary_mut()
                .unwrap();
            match side {
                0 => b.model_bounds_offset_x_neg = ParamLength::from_mm(1.0),
                1 => b.model_bounds_offset_x_pos = ParamLength::from_mm(-1.0),
                2 => b.model_bounds_offset_y_neg = ParamLength::from_mm(2.0),
                _ => b.model_bounds_offset_y_pos = ParamLength::from_mm(-2.0),
            }
            let h_next = compute_operation_input_hash(&proj, 0, 0, 1).unwrap();
            assert_ne!(
                prev_h, h_next,
                "changing model_bounds_offset side {side} must invalidate OperationInputHash"
            );
            prev_h = h_next;
        }
    }
}
