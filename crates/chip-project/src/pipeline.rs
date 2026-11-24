use std::{
    collections::{BTreeMap, HashMap},
    path::Path,
    sync::{Arc, atomic::AtomicBool},
};

use chip_cad::CadDocument;
use chip_cam::{FixtureKind, GeneratedToolpath, Operation, resolve_setup_fixtures};
use chip_math::RigidFrame3;
use chip_post::{
    OperationPostInput, PostOutputFile, PostProfile, SetupPostInput, describe_wcs_origin,
    post_process_setup, resolve_post_profile,
};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SetupSimulationResult, SimComputeBackendChoice,
    SimEngineKind, SimOperationInput, SimProgressScope, SimResolution, SimVerificationConfig,
    TriDexelStock, run_setup_simulation_with_choice_and_progress,
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
        max_rotary_rate_deg_min: profile.rotary.max_rotary_feed_deg_min.max(60.0),
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

/// Runs 2.5D Z-dexel heightmap stock removal & 6-rule crash/gouge/fixture verification for a single Setup.
pub fn simulate_project_setup(req: &SetupSimRequest<'_>) -> Result<SetupSimulationResult, String> {
    simulate_project_setup_with_engine(req, SimEngineKind::Heightmap)
}

/// Runs 2.5D Z-dexel heightmap stock removal & 6-rule crash/gouge/fixture verification for a single Setup,
/// resolving any `ImportedModel` workholding fixtures relative to `project_dir`.
pub fn simulate_project_setup_in_dir(
    req: &SetupSimRequest<'_>,
    project_dir: Option<&Path>,
) -> Result<SetupSimulationResult, String> {
    simulate_project_setup_with_choice_in_dir(
        req,
        project_dir,
        SimEngineKind::Heightmap,
        SimComputeBackendChoice::Cpu,
    )
}

/// Runs stock removal & 6-rule crash/gouge/fixture verification for a single Setup using the chosen
/// [`SimEngineKind`] (`Heightmap` or volumetric `TriDexel`) on the exact CPU backend.
pub fn simulate_project_setup_with_engine(
    req: &SetupSimRequest<'_>,
    engine: SimEngineKind,
) -> Result<SetupSimulationResult, String> {
    simulate_project_setup_with_choice(req, engine, SimComputeBackendChoice::Cpu)
}

/// Runs stock removal & 6-rule crash/gouge/fixture verification for a single Setup using the chosen
/// [`SimEngineKind`] and [`SimComputeBackendChoice`] (`Auto` / `Gpu` / `Cpu`).
pub fn simulate_project_setup_with_choice(
    req: &SetupSimRequest<'_>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
) -> Result<SetupSimulationResult, String> {
    simulate_project_setup_with_choice_in_dir(req, None, engine, choice)
}

/// Carried-over volumetric stock transferred from a preceding Setup (`M62`).
#[derive(Debug, Clone)]
pub struct CarryoverStockResult {
    pub stock: TriDexelStock,
    pub initial_raw_volume_mm3: f64,
    pub source_setup_id: String,
    pub source_setup_name: String,
}

/// Resolves and computes the carried-over [`TriDexelStock`] for `req.setup_idx` (recursively
/// resolving multi-setup chains `Setup 1 -> Setup 2 -> Setup N` in `TriDexel` mode and rigidly
/// transforming into `req.setup_idx`'s WCS frame).
///
/// Returns `Ok(None)` when `setup.carryover_from_setup_id` is `None`.
pub fn compute_setup_carryover_stock(
    req: &SetupSimRequest<'_>,
    project_dir: Option<&Path>,
    choice: SimComputeBackendChoice,
) -> Result<Option<CarryoverStockResult>, String> {
    let mut chain_cache: HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)> = HashMap::new();
    compute_setup_carryover_stock_with_cache(req, project_dir, choice, &mut chain_cache, None, None)
}

fn compute_carryover_progress_ranges(
    req: &SetupSimRequest<'_>,
    chain_cache: &HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)>,
) -> HashMap<usize, (f64, f64)> {
    let mut chain_rev = Vec::new();
    let mut cur_idx = req.setup_idx;
    while let Some(setup) = req.project.setups.get(cur_idx) {
        if chain_rev.contains(&cur_idx) {
            break;
        }
        chain_rev.push(cur_idx);
        let Some(ref src_id) = setup.carryover_from_setup_id else {
            break;
        };
        let Some(src_idx) = req.project.setups.iter().position(|s| &s.id == src_id) else {
            break;
        };
        if src_idx >= cur_idx || chain_cache.contains_key(&src_idx) {
            break;
        }
        cur_idx = src_idx;
    }
    chain_rev.reverse();

    let weights: Vec<(usize, f64)> = chain_rev
        .into_iter()
        .map(|idx| {
            let ready =
                collect_ready_setup_operations(req.project, idx, req.cache, req.cad_revision);
            let segs: usize = ready.iter().map(|r| r.toolpath.segments_wcs.len()).sum();
            (idx, (segs.max(1) as f64) + 20.0)
        })
        .collect();
    let total_w: f64 = weights.iter().map(|(_, w)| *w).sum::<f64>().max(1.0);

    let mut ranges = HashMap::with_capacity(weights.len());
    let mut acc = 0.0;
    for (idx, w) in weights {
        let start = acc / total_w;
        acc += w;
        let end = (acc / total_w).min(1.0);
        ranges.insert(idx, (start, end));
    }
    ranges
}

fn scope_for_setup<'a>(
    progress: Option<SimProgressScope<'a>>,
    ranges: Option<&HashMap<usize, (f64, f64)>>,
    setup_idx: usize,
) -> Option<SimProgressScope<'a>> {
    progress.map(|p| {
        ranges
            .and_then(|m| m.get(&setup_idx))
            .map_or(p, |&(start, end)| p.sub_scope(start, end))
    })
}

fn compute_setup_carryover_stock_with_cache(
    req: &SetupSimRequest<'_>,
    project_dir: Option<&Path>,
    choice: SimComputeBackendChoice,
    chain_cache: &mut HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)>,
    progress: Option<SimProgressScope<'_>>,
    progress_ranges: Option<&HashMap<usize, (f64, f64)>>,
) -> Result<Option<CarryoverStockResult>, String> {
    let setup = req
        .project
        .setups
        .get(req.setup_idx)
        .ok_or_else(|| format!("invalid setup index {}", req.setup_idx))?;
    let Some(ref src_id) = setup.carryover_from_setup_id else {
        return Ok(None);
    };
    let src_idx = req
        .project
        .setups
        .iter()
        .position(|s| &s.id == src_id)
        .ok_or_else(|| format!("carryover source setup '{src_id}' not found"))?;
    if src_idx >= req.setup_idx {
        return Err(format!(
            "carryover source setup '{src_id}' (index {src_idx}) must precede '{}' (index {})",
            setup.name, req.setup_idx
        ));
    }

    let dst_resolved = setup
        .resolve_with_sketches(
            req.cad_doc,
            &req.project.sketches,
            req.project.display_units,
        )
        .map_err(|e| e.to_string())?;

    if !chain_cache.contains_key(&src_idx) {
        let src_setup = &req.project.setups[src_idx];
        let enabled_with_tool = src_setup
            .operations
            .iter()
            .filter(|o| !o.suppressed && o.tool_id.is_some())
            .count();
        let src_ready =
            collect_ready_setup_operations(req.project, src_idx, req.cache, req.cad_revision);
        // Note: two-stage V-carve can emit 2 ready ops per operation, so check `< enabled_with_tool`.
        if enabled_with_tool > 0 && src_ready.len() < enabled_with_tool {
            return Err(format!(
                "Carryover source setup '{}' has ungenerated toolpaths; generate all toolpaths (Ctrl+G) first.",
                src_setup.name
            ));
        }

        let src_req = SetupSimRequest {
            project: req.project,
            setup_idx: src_idx,
            cad_doc: req.cad_doc,
            cad_revision: req.cad_revision,
            cache: req.cache,
            kinematics: resolve_kinematics_for_key(&src_setup.post_profile, project_dir),
            resolution: req.resolution,
            cancel: req.cancel,
        };
        let src_carryover = compute_setup_carryover_stock_with_cache(
            &src_req,
            project_dir,
            choice,
            chain_cache,
            progress,
            progress_ranges,
        )?;
        let src_resolved = src_setup
            .resolve_with_sketches(
                req.cad_doc,
                &req.project.sketches,
                req.project.display_units,
            )
            .map_err(|e| e.to_string())?;
        let src_progress = scope_for_setup(progress, progress_ranges, src_idx);
        let src_sim = simulate_single_setup_inner(
            &src_req,
            &src_resolved,
            &src_ready,
            project_dir,
            SimEngineKind::TriDexel,
            choice,
            SimVerificationConfig::default(),
            src_carryover.as_ref(),
            src_progress,
        )?;
        let src_td = src_sim
            .as_tridexel()
            .ok_or_else(|| {
                format!(
                    "carryover source setup '{}' did not produce a Tri-Dexel stock solid",
                    src_setup.name
                )
            })?
            .clone();
        chain_cache.insert(
            src_idx,
            (
                src_td,
                src_resolved.frame,
                src_sim.initial_stock_volume_mm3,
                src_setup.name.clone(),
            ),
        );
    }

    let (src_td, src_frame, raw_vol, src_name) = &chain_cache[&src_idx];
    let dst_td = src_td
        .transform_to_frame(*src_frame, dst_resolved.frame, req.resolution)
        .map_err(|e| e.to_string())?;

    Ok(Some(CarryoverStockResult {
        stock: dst_td,
        initial_raw_volume_mm3: *raw_vol,
        source_setup_id: src_id.clone(),
        source_setup_name: src_name.clone(),
    }))
}

/// Computes the Setup-WCS volumetric [`TriDexelStock`] obtained by sweeping the specified
/// preceding in-setup operations (`upstream_op_indices`) in `req.setup_idx` (starting from
/// cross-setup carryover stock when `setup.carryover_from_setup_id` is configured, else raw stock).
///
/// Used for in-setup cross-angle `Indexed3Plus1` volumetric rest roughing (`M73`).
pub fn compute_in_setup_carryover_stock(
    req: &SetupSimRequest<'_>,
    upstream_op_indices: &[usize],
    project_dir: Option<&Path>,
    choice: SimComputeBackendChoice,
) -> Result<TriDexelStock, String> {
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
    let base_carryover = compute_setup_carryover_stock(req, project_dir, choice)?;

    let mut ready_upstream = Vec::with_capacity(upstream_op_indices.len());
    for &op_idx in upstream_op_indices {
        let Some(op) = setup.operations.get(op_idx) else {
            continue;
        };
        if op.suppressed {
            continue;
        }
        let Some(tid) = &op.tool_id else {
            continue;
        };
        let Some(tool) = req.project.tools.iter().find(|t| &t.id == tid) else {
            return Err(format!(
                "Upstream operation '{}' references missing tool '{tid}'",
                op.name
            ));
        };
        let Some((_hash, toolpath)) =
            req.cache
                .get_for_operation(req.project, req.setup_idx, op_idx, req.cad_revision)
        else {
            return Err(format!(
                "Upstream operation '{}' has no cached toolpath for in-setup carryover",
                op.name
            ));
        };
        if let Some(stage) = &toolpath.clearing_stage
            && let Some(clear_tool) = req.project.tools.iter().find(|t| t.id == stage.tool_id)
        {
            ready_upstream.push(ReadySetupOperation {
                op_idx,
                op,
                tool: clear_tool,
                toolpath: Arc::new(stage.toolpath.clone()),
            });
        }
        ready_upstream.push(ReadySetupOperation {
            op_idx,
            op,
            tool,
            toolpath,
        });
    }

    let sim = simulate_single_setup_inner(
        req,
        &resolved,
        &ready_upstream,
        project_dir,
        SimEngineKind::TriDexel,
        choice,
        SimVerificationConfig::default(),
        base_carryover.as_ref(),
        None,
    )?;
    sim.as_tridexel().cloned().ok_or_else(|| {
        format!(
            "in-setup carryover for setup '{}' did not produce a Tri-Dexel stock solid",
            setup.name
        )
    })
}

#[allow(clippy::too_many_arguments)]
fn simulate_single_setup_inner(
    req: &SetupSimRequest<'_>,
    resolved: &chip_cam::ResolvedSetup,
    ready: &[ReadySetupOperation<'_>],
    project_dir: Option<&Path>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
    verify_config: SimVerificationConfig,
    carryover: Option<&CarryoverStockResult>,
    progress: Option<SimProgressScope<'_>>,
) -> Result<SetupSimulationResult, String> {
    let setup = &req.project.setups[req.setup_idx];
    let sim_inputs: Vec<SimOperationInput<'_>> = ready
        .iter()
        .map(|item| SimOperationInput::from_operation(item.op, item.tool, item.toolpath.as_ref()))
        .collect();

    let resolved_fixtures = if setup.fixtures.iter().any(|f| f.enabled) {
        let needs_imported = setup
            .fixtures
            .iter()
            .any(|f| f.enabled && matches!(f.kind, FixtureKind::ImportedModel { .. }));
        let fixture_docs: BTreeMap<String, CadDocument> = if needs_imported {
            req.project.load_fixture_cad_documents(project_dir)
        } else {
            BTreeMap::new()
        };
        resolve_setup_fixtures(setup, resolved, req.project.display_units, |id| {
            fixture_docs.get(id)
        })
        .map_err(|e| e.to_string())?
    } else {
        Vec::new()
    };

    run_setup_simulation_with_choice_and_progress(
        &SetupSimulationInput {
            doc: req.cad_doc,
            setup: resolved,
            target_bodies: &setup.target_bodies,
            fixtures: &resolved_fixtures,
            operations: &sim_inputs,
            resolution: req.resolution,
            kinematics: req.kinematics,
            verify_config,
            initial_stock: carryover.map(|c| &c.stock),
            initial_raw_volume_mm3: carryover.map(|c| c.initial_raw_volume_mm3),
            carried_over_from_setup: carryover.map(|c| c.source_setup_name.as_str()),
            cancel: req.cancel,
        },
        engine,
        choice,
        progress,
    )
    .map_err(|e| e.to_string())
}

/// Runs stock removal & 6-rule crash/gouge/fixture verification for a single Setup using the chosen
/// [`SimEngineKind`] and [`SimComputeBackendChoice`], resolving any `ImportedModel` workholding
/// fixtures relative to `project_dir` and automatically carrying over remaining volumetric stock
/// when `setup.carryover_from_setup_id` is set (`M62`).
pub fn simulate_project_setup_with_choice_in_dir(
    req: &SetupSimRequest<'_>,
    project_dir: Option<&Path>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
) -> Result<SetupSimulationResult, String> {
    simulate_project_setup_with_choice_and_progress_in_dir(req, project_dir, engine, choice, None)
}

/// Runs stock removal & 6-rule crash/gouge/fixture verification for a single Setup (including any
/// upstream carryover chain) while reporting live completion progress into `progress` (`0%`–`100%`).
pub fn simulate_project_setup_with_choice_and_progress_in_dir(
    req: &SetupSimRequest<'_>,
    project_dir: Option<&Path>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
    progress: Option<SimProgressScope<'_>>,
) -> Result<SetupSimulationResult, String> {
    simulate_project_setup_with_verify_and_progress_in_dir(
        req,
        project_dir,
        engine,
        choice,
        SimVerificationConfig::default(),
        progress,
    )
}

/// Runs stock removal & 6-rule crash/gouge/fixture verification for a single Setup with explicit
/// [`SimVerificationConfig`] tolerances and optional live progress reporting (`M89`).
pub fn simulate_project_setup_with_verify_and_progress_in_dir(
    req: &SetupSimRequest<'_>,
    project_dir: Option<&Path>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
    verify_config: SimVerificationConfig,
    progress: Option<SimProgressScope<'_>>,
) -> Result<SetupSimulationResult, String> {
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
    let mut chain_cache: HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)> = HashMap::new();
    let progress_ranges = progress.map(|_| compute_carryover_progress_ranges(req, &chain_cache));
    let carryover = compute_setup_carryover_stock_with_cache(
        req,
        project_dir,
        choice,
        &mut chain_cache,
        progress,
        progress_ranges.as_ref(),
    )?;
    let dst_progress = scope_for_setup(progress, progress_ranges.as_ref(), req.setup_idx);
    simulate_single_setup_inner(
        req,
        &resolved,
        &ready,
        project_dir,
        engine,
        choice,
        verify_config,
        carryover.as_ref(),
        dst_progress,
    )
}

/// Simulates all Setups in `project` sequentially, caching and reusing each Setup's final
/// `TriDexelStock` for downstream `carryover_from_setup_id` chains so upstream Setups are never
/// simulated redundantly (`M62`).
#[allow(clippy::too_many_arguments)]
pub fn simulate_all_setups_with_choice(
    project: &ProjectDocument,
    cad_doc: &CadDocument,
    cad_revision: u64,
    cache: &ToolpathCache,
    project_dir: Option<&Path>,
    resolution: SimResolution,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
    cancel: Option<&AtomicBool>,
) -> Result<Vec<SetupSimulationResult>, String> {
    simulate_all_setups_with_verify_and_progress(
        project,
        cad_doc,
        cad_revision,
        cache,
        project_dir,
        resolution,
        engine,
        choice,
        SimVerificationConfig::default(),
        cancel,
        None,
    )
}

/// Simulates all Setups in `project` sequentially with custom [`SimVerificationConfig`] tolerances
/// and optional live progress reporting (`M89`).
#[allow(clippy::too_many_arguments)]
pub fn simulate_all_setups_with_verify_and_progress(
    project: &ProjectDocument,
    cad_doc: &CadDocument,
    cad_revision: u64,
    cache: &ToolpathCache,
    project_dir: Option<&Path>,
    resolution: SimResolution,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
    verify_config: SimVerificationConfig,
    cancel: Option<&AtomicBool>,
    progress: Option<SimProgressScope<'_>>,
) -> Result<Vec<SetupSimulationResult>, String> {
    let n_setups = project.setups.len();
    let mut results = Vec::with_capacity(n_setups);
    let mut chain_cache: HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)> = HashMap::new();

    for (s_idx, setup) in project.setups.iter().enumerate() {
        if cancel.is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed)) {
            return Err("simulation cancelled".to_string());
        }
        let kin = if let Some(mid) = project.machine_id.as_deref()
            && let Ok(machine) = chip_tools::resolve_machine_definition(mid, project_dir)
        {
            MachineKinematicsConfig::from_machine_spec(&machine.kinematics)
        } else {
            resolve_kinematics_for_key(&setup.post_profile, project_dir)
        };
        let req = SetupSimRequest {
            project,
            setup_idx: s_idx,
            cad_doc,
            cad_revision,
            cache,
            kinematics: kin,
            resolution,
            cancel,
        };
        let resolved = setup
            .resolve_with_sketches(cad_doc, &project.sketches, project.display_units)
            .map_err(|e| e.to_string())?;
        let ready = collect_ready_setup_operations(project, s_idx, cache, cad_revision);
        let carryover = compute_setup_carryover_stock_with_cache(
            &req,
            project_dir,
            choice,
            &mut chain_cache,
            None,
            None,
        )?;
        let setup_scope = progress.map(|p| {
            let start = s_idx as f64 / n_setups.max(1) as f64;
            let end = (s_idx + 1) as f64 / n_setups.max(1) as f64;
            p.sub_scope(start, end)
        });
        let sim = simulate_single_setup_inner(
            &req,
            &resolved,
            &ready,
            project_dir,
            engine,
            choice,
            verify_config,
            carryover.as_ref(),
            setup_scope,
        )?;

        if let Some(td) = sim.as_tridexel() {
            chain_cache.insert(
                s_idx,
                (
                    td.clone(),
                    resolved.frame,
                    sim.initial_stock_volume_mm3,
                    setup.name.clone(),
                ),
            );
        }
        results.push(sim);
    }

    Ok(results)
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
        rotary: resolved.rotary,
        clearance_z_wcs: resolved.clearance_z_wcs,
        retract_z_wcs: resolved.retract_z_wcs,
        operations: post_ops,
    };

    post_process_setup(&post_input, req.post_profile, req.program_stem).map_err(|e| e.to_string())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::{Arc, Mutex};

    use chip_cad::CadDocument;
    use chip_cam::{
        BoxAnchor27, MotionRole, Operation, OperationKind, Setup, StockConfig, ToolpathBuilder,
        WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{DVec3, ParamLength};
    use chip_sim::{
        MachineKinematicsConfig, SimComputeBackendChoice, SimEngineKind, SimProgressStage,
        SimProgressTracker, SimResolution,
    };
    use chip_tools::default_starter_tool_library;

    use crate::{
        ProjectDocument, SetupSimRequest, ToolpathCache, compute_operation_input_hash,
        simulate_project_setup_with_choice_and_progress_in_dir,
    };

    #[test]
    fn carryover_chain_reports_single_monotonic_progress_to_100_percent() {
        let mut project = ProjectDocument::new("Carryover Progress Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let tool_id = project.tools[0].id.clone();

        let make_setup = |id: &str, name: &str, op_id: &str, op_name: &str| {
            let mut setup = Setup::new(id, name, WorkOffsetSlot::G54);
            setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::TOP_CENTER,
            };
            setup.stock = StockConfig::FixedBox {
                width_x: ParamLength::from_mm(100.0),
                depth_y: ParamLength::from_mm(100.0),
                height_z: ParamLength::from_mm(20.0),
                xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
                z_align: chip_cam::FixedBoxZAlign::CenterZ,
            };
            let mut op = Operation::new(op_id, op_name, OperationKind::Facing);
            op.tool_id = Some(tool_id.clone());
            setup.operations.push(op);
            setup
        };

        let setup1 = make_setup("s1", "Setup 1", "op1", "Op 1");
        let mut setup2 = make_setup("s2", "Setup 2", "op2", "Op 2");
        setup2.carryover_from_setup_id = Some("s1".to_string());
        project.setups = vec![setup1, setup2];

        let doc = CadDocument::empty();
        let mut cache = ToolpathCache::new(16);
        for (s_idx, op_id, z_cut) in [(0, "op1", -2.0), (1, "op2", -4.0)] {
            let mut b = ToolpathBuilder::new(op_id);
            b.rapid_to(DVec3::new(-20.0, 0.0, 5.0));
            b.linear_to(DVec3::new(-20.0, 0.0, z_cut), 300.0, MotionRole::Plunge);
            for i in 0..8 {
                let x = -20.0 + (i as f64 + 1.0) * 5.0;
                b.linear_to(DVec3::new(x, 0.0, z_cut), 600.0, MotionRole::Cut);
            }
            b.rapid_to(DVec3::new(20.0, 0.0, 5.0));
            let tp = b.finish();
            let key = compute_operation_input_hash(&project, s_idx, 0, 0).unwrap();
            cache.insert(op_id, key, Arc::new(tp));
        }

        let history = Arc::new(Mutex::new(Vec::<u8>::new()));
        let history_cb = Arc::clone(&history);
        let slot: Arc<Mutex<Option<Arc<SimProgressTracker>>>> = Arc::new(Mutex::new(None));
        let slot_cb = Arc::clone(&slot);
        let tracker = Arc::new(SimProgressTracker::with_callback(Arc::new(move || {
            if let Some(t) = slot_cb.lock().unwrap().as_ref() {
                history_cb
                    .lock()
                    .unwrap()
                    .push(t.snapshot().percent_rounded());
            }
        })));
        *slot.lock().unwrap() = Some(Arc::clone(&tracker));

        let res = simulate_project_setup_with_choice_and_progress_in_dir(
            &SetupSimRequest {
                project: &project,
                setup_idx: 1,
                cad_doc: &doc,
                cad_revision: 0,
                cache: &cache,
                kinematics: MachineKinematicsConfig::default(),
                resolution: SimResolution::Standard,
                cancel: None,
            },
            None,
            SimEngineKind::TriDexel,
            SimComputeBackendChoice::Cpu,
            Some(tracker.full_scope()),
        )
        .unwrap();

        assert_eq!(res.carried_over_from_setup.as_deref(), Some("Setup 1"));
        assert_eq!(tracker.snapshot().percent_rounded(), 100);
        assert_eq!(tracker.snapshot().stage, SimProgressStage::Complete);

        let pcts = history.lock().unwrap();
        assert!(
            pcts.iter().any(|&p| (10..=45).contains(&p))
                && pcts.iter().any(|&p| (55..=95).contains(&p)),
            "expected monotonic progress spanning both carryover setups, got {pcts:?}"
        );
        for w in pcts.windows(2) {
            assert!(
                w[1] >= w[0],
                "carryover progress must never regress: {pcts:?}"
            );
        }
    }
}
