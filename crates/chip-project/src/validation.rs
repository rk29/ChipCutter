use std::{borrow::Cow, path::Path};

use chip_cad::CadDocument;
use chip_cam::{
    FixtureKind, OperationRotaryMode, ResolvedHeights, RestMachiningMode,
    build_indexed_virtual_setup, extract_selection_features_with_sketches, resolve_fixture_body,
};
use chip_tools::{MachineDefinition, builtin_generic_machine, resolve_machine_definition};

use crate::document::{ModelRole, ProjectDocument, compute_sha256_hex, resolve_model_path};

/// Severity of a project validation diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticSeverity {
    Warning,
    Error,
}

/// Structured validation finding on a `.chip.toml` project.
#[derive(Debug, Clone, PartialEq, Eq)]
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
                            if !ctool.geometry.supports_operation(
                                chip_tools::ToolOperationClass::VCarveFlatClearance,
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

                // Strategy-specific validation for `OperationKind::Deburr3D`
                if op.kind == chip_cam::OperationKind::Deburr3D
                    && let chip_cam::StrategyParams::Deburr3D(mut p) = op.resolved_strategy()
                {
                    p.reevaluate(&eval_ctx);
                    if !p.chamfer_width.value_mm.is_finite() || p.chamfer_width.value_mm <= 1e-4 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Deburr chamfer width ({:.3} mm) must be finite and > 0.",
                                p.chamfer_width.value_mm
                            ),
                        );
                    }
                    if !p.tip_offset.value_mm.is_finite() || p.tip_offset.value_mm < 0.0 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Deburr tip offset ({:.3} mm) must be finite and >= 0.",
                                p.tip_offset.value_mm
                            ),
                        );
                    }
                    if !p.wall_clearance.value_mm.is_finite() || p.wall_clearance.value_mm < 0.0 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Deburr wall clearance ({:.3} mm) must be finite and >= 0.",
                                p.wall_clearance.value_mm
                            ),
                        );
                    }
                    if !p.edge_extension.value_mm.is_finite() || p.edge_extension.value_mm < 0.0 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Deburr edge extension ({:.3} mm) must be finite and >= 0.",
                                p.edge_extension.value_mm
                            ),
                        );
                    }
                    if !p.chord_tolerance_mm.is_finite() || p.chord_tolerance_mm <= 1e-5 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Deburr chord tolerance ({:.4} mm) must be finite and > 0.",
                                p.chord_tolerance_mm
                            ),
                        );
                    }
                    if !p.min_dihedral_angle_deg.is_finite()
                        || p.min_dihedral_angle_deg <= 0.0
                        || p.min_dihedral_angle_deg >= 179.0
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Deburr min edge angle ({:.1}°) must be within (0°, 179°).",
                                p.min_dihedral_angle_deg
                            ),
                        );
                    }
                    if p.multiple_passes == 0 {
                        report.push_error(&op_scope, "3D Deburr passes count must be >= 1.");
                    }
                    if let Some(tool) = op.tool_id.as_ref().and_then(|tid| project.tool(tid)) {
                        if p.allow_underside_back_chamfer
                            && !matches!(
                                tool.geometry,
                                chip_tools::CutterGeometry::LollipopEndmill { .. }
                            )
                        {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "Underside back-chamfering requires a Lollipop Endmill cutter (assigned: {}).",
                                    tool.geometry.kind_label()
                                ),
                            );
                        }
                        if let chip_tools::CutterGeometry::ChamferMill {
                            diameter_mm,
                            tip_diameter_mm,
                            included_angle_deg,
                            ..
                        } = tool.geometry
                        {
                            let half_a = (0.5 * included_angle_deg).clamp(5.0, 85.0).to_radians();
                            let max_r = 0.5 * (diameter_mm - tip_diameter_mm).max(0.0);
                            let req_r = p.tip_offset.value_mm.max(0.0) * half_a.tan()
                                + p.chamfer_width.value_mm.max(0.0);
                            if req_r > max_r + 1e-3 {
                                report.push_error(
                                    &op_scope,
                                    format!(
                                        "Chamfer width + tip offset radial reach ({req_r:.2} mm) exceeds chamfer mill flute span ({max_r:.2} mm)."
                                    ),
                                );
                            }
                        }
                    }
                }

                // Strategy-specific validation for Milestone 66 Undercut operations
                if op.kind == chip_cam::OperationKind::SlotUndercut
                    && let chip_cam::StrategyParams::SlotUndercut(mut p) = op.resolved_strategy()
                {
                    p.reevaluate(&eval_ctx);
                    if !p.slot_width.value_mm.is_finite() || p.slot_width.value_mm <= 1e-4 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Slot/T-Slot width ({:.3} mm) must be finite and > 0.",
                                p.slot_width.value_mm
                            ),
                        );
                    }
                    if !p.throat_width.value_mm.is_finite() || p.throat_width.value_mm <= 1e-4 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Slot/T-Slot throat width ({:.3} mm) must be finite and > 0.",
                                p.throat_width.value_mm
                            ),
                        );
                    }
                    if !p.slot_height.value_mm.is_finite() || p.slot_height.value_mm <= 1e-4 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Slot/T-Slot height ({:.3} mm) must be finite and > 0.",
                                p.slot_height.value_mm
                            ),
                        );
                    }
                    if !p.radial_stepover.value_mm.is_finite() || p.radial_stepover.value_mm <= 1e-4
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Slot/T-Slot radial stepover ({:.3} mm) must be finite and > 0.",
                                p.radial_stepover.value_mm
                            ),
                        );
                    }
                    if !p.axial_stepdown.value_mm.is_finite() || p.axial_stepdown.value_mm <= 1e-4 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Slot/T-Slot axial stepdown ({:.3} mm) must be finite and > 0.",
                                p.axial_stepdown.value_mm
                            ),
                        );
                    }
                    if let Some(tool) = op.tool_id.as_ref().and_then(|tid| project.tool(tid)) {
                        let needs_undercut = p.mode == chip_cam::SlotUndercutMode::TSlot
                            || p.throat_width.value_mm < p.slot_width.value_mm - 1e-4;
                        match tool.geometry {
                            chip_tools::CutterGeometry::SlotMill {
                                diameter_mm,
                                cutting_thickness_mm,
                                neck_diameter_mm,
                                ..
                            } => {
                                if diameter_mm > p.slot_width.value_mm + 1e-4 {
                                    report.push_error(
                                        &op_scope,
                                        format!(
                                            "Slot mill diameter (Ø{diameter_mm:.3} mm) exceeds slot width ({:.3} mm).",
                                            p.slot_width.value_mm
                                        ),
                                    );
                                }
                                if cutting_thickness_mm > p.slot_height.value_mm + 1e-4 {
                                    report.push_error(
                                        &op_scope,
                                        format!(
                                            "Slot mill cutting thickness ({cutting_thickness_mm:.3} mm) exceeds slot height ({:.3} mm).",
                                            p.slot_height.value_mm
                                        ),
                                    );
                                }
                                if p.mode != chip_cam::SlotUndercutMode::SideGroove
                                    && needs_undercut
                                {
                                    if neck_diameter_mm >= p.throat_width.value_mm - 1e-4 {
                                        report.push_error(
                                            &op_scope,
                                            format!(
                                                "Slot mill neck diameter (Ø{neck_diameter_mm:.3} mm) must be strictly smaller than throat width ({:.3} mm).",
                                                p.throat_width.value_mm
                                            ),
                                        );
                                    } else {
                                        let max_lat = (0.5 * p.slot_width.value_mm
                                            - 0.5 * diameter_mm)
                                            .max(0.0);
                                        if max_lat + 0.5 * neck_diameter_mm
                                            > 0.5 * p.throat_width.value_mm + 1e-4
                                        {
                                            report.push_error(
                                                &op_scope,
                                                format!(
                                                    "Slot mill relieved neck collides with throat wall at full lateral offset ({max_lat:.3} mm + neck radius {:.3} mm > half-throat {:.3} mm).",
                                                    0.5 * neck_diameter_mm,
                                                    0.5 * p.throat_width.value_mm
                                                ),
                                            );
                                        }
                                    }
                                }
                            }
                            chip_tools::CutterGeometry::FlatEndmill { diameter_mm, .. } => {
                                if needs_undercut {
                                    report.push_error(
                                        &op_scope,
                                        "T-Slot undercut requires a relieved-neck Slot / T-Slot Mill cutter (assigned: Flat Endmill).",
                                    );
                                } else if diameter_mm > p.slot_width.value_mm + 1e-4 {
                                    report.push_error(
                                        &op_scope,
                                        format!(
                                            "Flat Endmill diameter (Ø{diameter_mm:.3} mm) exceeds keyway width ({:.3} mm).",
                                            p.slot_width.value_mm
                                        ),
                                    );
                                }
                            }
                            _ => {}
                        }
                    }
                }

                if op.kind == chip_cam::OperationKind::Dovetail
                    && let chip_cam::StrategyParams::Dovetail(mut p) = op.resolved_strategy()
                {
                    p.reevaluate(&eval_ctx);
                    if !p.side_angle_deg.is_finite()
                        || p.side_angle_deg <= 15.0
                        || p.side_angle_deg >= 85.0
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Dovetail side angle ({:.1}°) must be within (15°, 85°).",
                                p.side_angle_deg
                            ),
                        );
                    }
                    if !p.dovetail_depth.value_mm.is_finite() || p.dovetail_depth.value_mm <= 1e-4 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Dovetail depth ({:.3} mm) must be finite and > 0.",
                                p.dovetail_depth.value_mm
                            ),
                        );
                    }
                    if !p.radial_stepover.value_mm.is_finite() || p.radial_stepover.value_mm <= 1e-4
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "Dovetail radial stepover ({:.3} mm) must be finite and > 0.",
                                p.radial_stepover.value_mm
                            ),
                        );
                    }
                    if let Some(tool) = op.tool_id.as_ref().and_then(|tid| project.tool(tid))
                        && let chip_tools::CutterGeometry::DovetailCutter {
                            diameter_mm,
                            side_angle_deg,
                            cutting_height_mm,
                            corner_radius_mm,
                            neck_diameter_mm,
                            ..
                        } = tool.geometry
                    {
                        if (side_angle_deg - p.side_angle_deg).abs() > 0.5 {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "Dovetail cutter angle ({side_angle_deg:.2}°) mismatches target dovetail angle ({:.2}°, max tolerance ±0.5°).",
                                    p.side_angle_deg
                                ),
                            );
                        }
                        if cutting_height_mm + 1e-4 < p.dovetail_depth.value_mm {
                            report.push_error(
                                &op_scope,
                                format!(
                                    "Dovetail cutter cutting height ({cutting_height_mm:.3} mm) is shallower than dovetail depth ({:.3} mm).",
                                    p.dovetail_depth.value_mm
                                ),
                            );
                        }
                        if p.mode == chip_cam::DovetailMode::InternalSlot {
                            let throat = p.top_throat_width.value_mm;
                            if neck_diameter_mm >= throat - 1e-4 {
                                report.push_error(
                                    &op_scope,
                                    format!(
                                        "Dovetail cutter neck diameter (Ø{neck_diameter_mm:.3} mm) must be strictly smaller than top throat width ({throat:.3} mm)."
                                    ),
                                );
                            }
                            let top_d = chip_tools::CutterGeometry::dovetail_top_diameter_mm(
                                diameter_mm,
                                side_angle_deg,
                                p.dovetail_depth.value_mm.max(0.0),
                                corner_radius_mm,
                            );
                            if top_d > throat + 1e-4 {
                                report.push_error(
                                    &op_scope,
                                    format!(
                                        "Dovetail cutter upper flank diameter (Ø{top_d:.3} mm) exceeds top throat width ({throat:.3} mm)."
                                    ),
                                );
                            }
                        }
                    }
                }

                if op.kind == chip_cam::OperationKind::Undercut3D
                    && let chip_cam::StrategyParams::Undercut3D(mut p) = op.resolved_strategy()
                {
                    p.reevaluate(&eval_ctx);
                    if !p.use_cusp_height
                        && (!p.z_stepdown.value_mm.is_finite() || p.z_stepdown.value_mm <= 1e-4)
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Undercut Z stepdown ({:.3} mm) must be finite and > 0.",
                                p.z_stepdown.value_mm
                            ),
                        );
                    }
                    if p.use_cusp_height
                        && (!p.target_cusp.value_mm.is_finite() || p.target_cusp.value_mm <= 1e-5)
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Undercut target cusp ({:.4} mm) must be finite and > 0.",
                                p.target_cusp.value_mm
                            ),
                        );
                    }
                    if !p.min_overhang_angle_deg.is_finite()
                        || p.min_overhang_angle_deg <= 0.0
                        || p.min_overhang_angle_deg >= 89.0
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Undercut min overhang angle ({:.1}°) must be within (0°, 89°).",
                                p.min_overhang_angle_deg
                            ),
                        );
                    }
                    if !p.neck_clearance.value_mm.is_finite() || p.neck_clearance.value_mm < 0.0 {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Undercut neck clearance ({:.3} mm) must be finite and >= 0.",
                                p.neck_clearance.value_mm
                            ),
                        );
                    }
                    if !p.lead_out_distance.value_mm.is_finite()
                        || p.lead_out_distance.value_mm < 0.0
                    {
                        report.push_error(
                            &op_scope,
                            format!(
                                "3D Undercut lead-out distance ({:.3} mm) must be finite and >= 0.",
                                p.lead_out_distance.value_mm
                            ),
                        );
                    }
                }

                if op.kind.is_rotary_native() && resolved_setup.rotary.is_none() {
                    report.push_error(
                        &op_scope,
                        "4th-Axis Rotary operations require 4th-Axis Rotary to be enabled on this Setup.",
                    );
                    continue;
                }
                if op.kind == chip_cam::OperationKind::Rotary4AxisSim
                    && let Some(rot) = resolved_setup.rotary
                    && rot.mode != chip_cam::RotaryDriveMode::Simultaneous4Axis
                {
                    report.push_error(
                        &op_scope,
                        "Simultaneous 4-Axis Milling requires Setup Rotary Mode 'Simultaneous 4-Axis (X+Y+Z+Rotary)'.",
                    );
                    continue;
                }
                if !op.rotary.is_none() && resolved_setup.rotary.is_none() {
                    report.push_error(
                        &op_scope,
                        "Operation 4th-axis indexing/wrapping requires 4th-Axis Rotary to be enabled on this Setup.",
                    );
                    continue;
                }

                // Validate selections & Z-height hierarchy (in rotated virtual Setup for Indexed 3+1)
                let effective_setup = match (&op.rotary, resolved_setup.rotary) {
                    (OperationRotaryMode::Indexed3Plus1 { angle_deg }, Some(rot)) => Cow::Owned(
                        build_indexed_virtual_setup(&resolved_setup, &rot, *angle_deg),
                    ),
                    _ => Cow::Borrowed(&resolved_setup),
                };
                let summary = match extract_selection_features_with_sketches(
                    cad_doc,
                    &project.sketches,
                    &effective_setup,
                    &op.selections,
                ) {
                    Ok(s) => s,
                    Err(err) => {
                        report.push_error(&op_scope, err.to_string());
                        continue;
                    }
                };

                if let Some(boundary) = op.resolved_strategy().machining_boundary()
                    && boundary.boundary_mode == chip_cam::MachiningBoundaryMode::SelectedRegions
                {
                    let has_2d_region = summary.features.iter().any(|feat| {
                        matches!(
                            feat,
                            chip_cam::ExtractedFeature25D::PlanarFace { .. }
                                | chip_cam::ExtractedFeature25D::Contour {
                                    polyline_wcs: chip_math::ArcPolyline2D { closed: true, .. },
                                    ..
                                }
                        )
                    });
                    if !has_2d_region {
                        report.push_warning(
                            &op_scope,
                            "3D Machining Boundary is set to 'Selected 2D Contours' but no closed 2D contours or planar faces are selected; falling back to Model Silhouette.",
                        );
                    }
                }

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
                        let total_depth = (rh.top_z_wcs
                            - (rh.bottom_z_wcs + op.passes.stock_to_leave_axial.value_mm.max(0.0)))
                        .max(0.0);
                        if let Some(tool) = op.tool_id.as_ref().and_then(|tid| project.tool(tid)) {
                            match (op.kind, &tool.geometry) {
                                (
                                    chip_cam::OperationKind::SlotUndercut,
                                    chip_tools::CutterGeometry::SlotMill { neck_length_mm, .. },
                                ) if *neck_length_mm + 1e-4 < total_depth => {
                                    report.push_error(
                                        &op_scope,
                                        format!(
                                            "Slot mill relieved neck length ({neck_length_mm:.3} mm) is shorter than total slot depth ({total_depth:.3} mm)."
                                        ),
                                    );
                                }
                                (
                                    chip_cam::OperationKind::Dovetail,
                                    chip_tools::CutterGeometry::DovetailCutter {
                                        neck_length_mm,
                                        ..
                                    },
                                ) if *neck_length_mm + 1e-4 < total_depth => {
                                    report.push_error(
                                        &op_scope,
                                        format!(
                                            "Dovetail cutter relieved neck length ({neck_length_mm:.3} mm) is shorter than total cut depth ({total_depth:.3} mm)."
                                        ),
                                    );
                                }
                                _ => {}
                            }
                        }
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{CadKernel, MonstertruckKernel, sample_overhang_flange_step};
    use chip_cam::{
        Deburr3DParams, Operation, OperationKind, Setup, StrategyParams, WorkOffsetSlot,
    };
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
}
