use std::{borrow::Cow, path::Path};

use chip_cad::CadDocument;
use chip_cam::{
    OperationRotaryMode, ResolvedHeights, RestMachiningMode, build_indexed_virtual_setup,
    extract_selection_features_with_sketches,
};
use chip_tools::{MachineDefinition, builtin_generic_machine, resolve_machine_definition};

use crate::document::{ProjectDocument, compute_sha256_hex, resolve_model_path};

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

    // 1. Check model files and SHA-256 content hashes when project_dir is on disk
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

    // 3. Validate Setups & Operations against loaded CAD geometry (or Pure-2D empty CAD when `models.is_empty()`)
    let empty_cad = CadDocument::empty();
    let effective_cad: Option<&CadDocument> = if let Some(cad_doc) = loaded_cad {
        Some(cad_doc)
    } else if project.models.is_empty() {
        Some(&empty_cad)
    } else {
        None
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
                if let Err(err) = ResolvedHeights::resolve(
                    &heights,
                    &effective_setup.model_aabb_wcs,
                    &effective_setup.stock.aabb_wcs(),
                    effective_setup.clearance_z_wcs,
                    effective_setup.retract_z_wcs,
                    inf_top_z,
                    inf_bot_z,
                ) {
                    report.push_error(&op_scope, err.to_string());
                }
            }
        }
    } else if !project.models.is_empty() && project_dir.is_none() {
        report.push_warning(
            "Project",
            "CAD document not loaded in memory; skipped B-Rep WCS and selection checks.",
        );
    }

    report
}
