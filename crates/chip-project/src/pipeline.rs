use std::{
    path::Path,
    sync::{Arc, atomic::AtomicBool},
};

use chip_cad::CadDocument;
use chip_cam::{GeneratedToolpath, Operation};
use chip_post::{
    OperationPostInput, PostOutputFile, PostProfile, SetupPostInput, describe_wcs_origin,
    post_process_setup, resolve_post_profile,
};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SetupSimulationResult, SimOperationInput,
    SimResolution, SimVerificationConfig, run_setup_simulation,
};
use chip_tools::Tool;

use crate::{cache::ToolpathCache, document::ProjectDocument};

/// Enabled operation in a Setup paired with its resolved [`Tool`] and cached [`GeneratedToolpath`].
#[derive(Debug, Clone)]
pub struct ReadySetupOperation<'a> {
    pub op_idx: usize,
    pub op: &'a Operation,
    pub tool: &'a Tool,
    pub toolpath: Arc<GeneratedToolpath>,
}

/// Collects all non-suppressed operations in `project.setups[setup_idx]` that have both a valid
/// embedded [`Tool`] and a resident [`GeneratedToolpath`] in `cache` at `cad_revision`.
pub fn collect_ready_setup_operations<'a>(
    project: &'a ProjectDocument,
    setup_idx: usize,
    cache: &ToolpathCache,
    cad_revision: u64,
) -> Vec<ReadySetupOperation<'a>> {
    let Some(setup) = project.setups.get(setup_idx) else {
        return Vec::new();
    };
    let mut ready = Vec::new();
    for (op_idx, op) in setup.operations.iter().enumerate() {
        if op.suppressed {
            continue;
        }
        if let Some(tid) = &op.tool_id
            && let Some(tool) = project.tools.iter().find(|t| &t.id == tid)
            && let Some((_hash, toolpath)) =
                cache.get_for_operation(project, setup_idx, op_idx, cad_revision)
        {
            // When Two-Stage Flat-Bottom V-Carving produced a secondary Flat Endmill Stage 1
            // clearing toolpath, emit Stage 1 FIRST followed by Stage 2 V-Bit SECOND so both
            // `chip-sim` stock removal and `chip-post` (`M6` ATC or GRBL `split_files_by_tool`)
            // sequence both tools cleanly!
            if let Some(stage) = &toolpath.clearing_stage
                && let Some(clear_tool) = project.tools.iter().find(|t| t.id == stage.tool_id)
            {
                ready.push(ReadySetupOperation {
                    op_idx,
                    op,
                    tool: clear_tool,
                    toolpath: Arc::new(stage.toolpath.clone()),
                });
            }
            ready.push(ReadySetupOperation {
                op_idx,
                op,
                tool,
                toolpath,
            });
        }
    }
    ready
}

/// Maps a [`PostProfile`]'s rapid rate, acceleration, and tool-change duration into
/// [`MachineKinematicsConfig`].
pub fn kinematics_for_post_profile(profile: &PostProfile) -> MachineKinematicsConfig {
    MachineKinematicsConfig {
        rapid_rate_mm_min: profile.rapid_rate_mm_min,
        accel_mm_s2: profile.accel_mm_s2,
        junction_deviation_mm: 0.05,
        tool_change_time_sec: profile.tool_change_time_sec,
    }
}

/// Resolves machine kinematics for `post_profile_key` (falling back to default kinematics if unknown).
pub fn resolve_kinematics_for_key(
    post_profile_key: &str,
    project_dir: Option<&Path>,
) -> MachineKinematicsConfig {
    resolve_post_profile(post_profile_key, project_dir)
        .map(|p| kinematics_for_post_profile(&p))
        .unwrap_or_default()
}

/// Input configuration for [`simulate_project_setup`].
pub struct SetupSimRequest<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: &'a CadDocument,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub kinematics: MachineKinematicsConfig,
    pub resolution: SimResolution,
    pub cancel: Option<&'a AtomicBool>,
}

/// Runs 2.5D Z-dexel heightmap stock removal & 5-rule crash/gouge verification for a single Setup.
pub fn simulate_project_setup(req: &SetupSimRequest<'_>) -> Result<SetupSimulationResult, String> {
    let setup = req
        .project
        .setups
        .get(req.setup_idx)
        .ok_or_else(|| format!("invalid setup index {}", req.setup_idx))?;
    let resolved = setup
        .resolve_with_sketches(
            req.cad_doc,
            &req.project.sketches,
            req.project.display_units,
        )
        .map_err(|e| e.to_string())?;
    let ready =
        collect_ready_setup_operations(req.project, req.setup_idx, req.cache, req.cad_revision);
    let sim_inputs: Vec<SimOperationInput<'_>> = ready
        .iter()
        .map(|item| SimOperationInput::from_operation(item.op, item.tool, item.toolpath.as_ref()))
        .collect();

    run_setup_simulation(&SetupSimulationInput {
        doc: req.cad_doc,
        setup: &resolved,
        target_bodies: &setup.target_bodies,
        operations: &sim_inputs,
        resolution: req.resolution,
        kinematics: req.kinematics,
        verify_config: SimVerificationConfig::default(),
        cancel: req.cancel,
    })
    .map_err(|e| e.to_string())
}

/// Input configuration for [`post_process_project_setup`].
pub struct SetupPostRequest<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: &'a CadDocument,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub post_profile: &'a PostProfile,
    pub project_label: &'a str,
    pub program_stem: &'a str,
}

/// Emits G-code (`PostOutputFile`s) for all cached non-suppressed operations in `setup_idx`.
pub fn post_process_project_setup(
    req: &SetupPostRequest<'_>,
) -> Result<Vec<PostOutputFile>, String> {
    let setup = req
        .project
        .setups
        .get(req.setup_idx)
        .ok_or_else(|| format!("invalid setup index {}", req.setup_idx))?;
    let resolved = setup
        .resolve_with_sketches(
            req.cad_doc,
            &req.project.sketches,
            req.project.display_units,
        )
        .map_err(|e| format!("Setup WCS error: {e}"))?;
    let ready =
        collect_ready_setup_operations(req.project, req.setup_idx, req.cache, req.cad_revision);
    if ready.is_empty() {
        return Err("Generate clean toolpaths (Ctrl+G) to preview or export G-code.".to_string());
    }

    let post_ops: Vec<OperationPostInput<'_>> = ready
        .iter()
        .map(|item| OperationPostInput {
            op: item.op,
            tool: item.tool,
            toolpath: item.toolpath.as_ref(),
        })
        .collect();

    let post_input = SetupPostInput {
        project_name: req.project_label,
        setup_name: &setup.name,
        wcs_slot: setup.wcs_slot,
        wcs_origin_description: describe_wcs_origin(&setup.wcs),
        stock: resolved.stock,
        clearance_z_wcs: resolved.clearance_z_wcs,
        retract_z_wcs: resolved.retract_z_wcs,
        operations: post_ops,
    };

    post_process_setup(&post_input, req.post_profile, req.program_stem).map_err(|e| e.to_string())
}
