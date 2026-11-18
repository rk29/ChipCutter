use std::path::Path;

use chip_cad::CadDocument;
use chip_cam::{ResolvedHeights, RestMachiningMode, extract_selection_features};

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
/// Stock bounds, B-Rep selections, and Z-height hierarchies (`09-persistence-cli.md`).
pub fn validate_project(
    project: &ProjectDocument,
    project_dir: Option<&Path>,
    loaded_cad: Option<&CadDocument>,
) -> ProjectValidationReport {
    let mut report = ProjectValidationReport::default();

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
                                "STEP file '{}' SHA-256 changed on disk (expected {}…, found {}…). Reload STEP model to re-bind selections.",
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
                        format!("cannot read STEP file '{}': {e}", resolved_path.display()),
                    );
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

    // 3. Validate Setups & Operations against loaded CAD geometry
    if let Some(cad_doc) = loaded_cad {
        for setup in &project.setups {
            let setup_scope = format!("Setup '{}'", setup.name);
            let resolved_setup = match setup.resolve_immutable(cad_doc, project.display_units) {
                Ok(rs) => rs,
                Err(err) => {
                    report.push_error(setup_scope, err.to_string());
                    continue;
                }
            };

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

                // Validate selections & Z-height hierarchy
                let summary =
                    match extract_selection_features(cad_doc, &resolved_setup, &op.selections) {
                        Ok(s) => s,
                        Err(err) => {
                            report.push_error(&op_scope, err.to_string());
                            continue;
                        }
                    };

                eval_ctx.set_var("feature_top_z", summary.inferred_top_z_wcs);
                eval_ctx.set_var("feature_bottom_z", summary.inferred_bottom_z_wcs);
                if let Some(hd) = summary.primary_hole_diameter_mm {
                    eval_ctx.set_var("hole_d", hd);
                }

                let mut heights = op.heights.clone();
                heights.reevaluate(&eval_ctx);
                if let Err(err) = ResolvedHeights::resolve(
                    &heights,
                    &resolved_setup.model_aabb_wcs,
                    &resolved_setup.stock.aabb_wcs(),
                    resolved_setup.clearance_z_wcs,
                    resolved_setup.retract_z_wcs,
                    summary.inferred_top_z_wcs,
                    summary.inferred_bottom_z_wcs,
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
