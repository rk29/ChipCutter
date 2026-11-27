use std::{
    collections::{BTreeMap, HashMap},
    path::{Path, PathBuf},
    sync::{Arc, atomic::AtomicBool},
};

use chip_cad::CadDocument;
use chip_cam::{FixtureKind, GeneratedToolpath, Operation, resolve_setup_fixtures};
use chip_math::RigidFrame3;
use chip_post::{
    OperationPostInput, PostOutputFile, PostProfile, PostProfileSource, SetupPostInput,
    describe_wcs_origin, load_available_post_profiles, post_process_setup, resolve_post_profile,
    validate_safe_output_filename,
};
use chip_sim::{
    MachineKinematicsConfig, SetupSimulationInput, SetupSimulationResult, SimComputeBackendChoice,
    SimEngineKind, SimOperationInput, SimProgressScope, SimResolution, SimVerificationConfig,
    TriDexelStock, run_setup_simulation,
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
    profile.to_kinematics_spec()
}

/// Resolves machine kinematics for `post_profile_key` (falling back to default kinematics if unknown).
pub fn resolve_kinematics_for_key(
    post_profile_key: &str,
    project_dir: Option<&Path>,
) -> MachineKinematicsConfig {
    resolve_post_profile(post_profile_key, project_dir)
        .map(|p| p.to_kinematics_spec())
        .unwrap_or_default()
}

/// Canonical setup kinematics resolver: honors `project.machine_id` first when explicitly set
/// and resolvable, falling back to `setup.post_profile` kinematics otherwise.
pub fn resolve_setup_kinematics(
    project: &ProjectDocument,
    setup_idx: usize,
    project_dir: Option<&Path>,
) -> MachineKinematicsConfig {
    if let Some(mid) = project.machine_id.as_deref()
        && !mid.trim().is_empty()
        && let Ok(machine) = chip_tools::resolve_machine_definition(mid, project_dir)
    {
        return machine.kinematics;
    }
    project
        .setups
        .get(setup_idx)
        .map(|s| resolve_kinematics_for_key(&s.post_profile, project_dir))
        .unwrap_or_default()
}

/// Unified request struct for single-setup simulation via [`simulate_project_setup`].
#[derive(Clone, Copy)]
pub struct SetupSimRequest<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: &'a CadDocument,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_dir: Option<&'a Path>,
    pub kinematics: MachineKinematicsConfig,
    pub resolution: SimResolution,
    pub engine: SimEngineKind,
    pub backend_choice: SimComputeBackendChoice,
    pub verify_config: SimVerificationConfig,
    pub cancel: Option<&'a AtomicBool>,
    pub progress: Option<SimProgressScope<'a>>,
}

impl<'a> SetupSimRequest<'a> {
    /// Creates a [`SetupSimRequest`] with canonical resolved setup kinematics, `Standard`
    /// resolution, `Heightmap` engine, exact `Cpu` backend, and default verification tolerances.
    pub fn new(
        project: &'a ProjectDocument,
        setup_idx: usize,
        cad_doc: &'a CadDocument,
        cad_revision: u64,
        cache: &'a ToolpathCache,
    ) -> Self {
        Self {
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_dir: None,
            kinematics: resolve_setup_kinematics(project, setup_idx, None),
            resolution: SimResolution::Standard,
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            verify_config: SimVerificationConfig::default(),
            cancel: None,
            progress: None,
        }
    }
}

/// Unified request struct for sequential multi-setup simulation via [`simulate_all_setups`].
#[derive(Clone, Copy)]
pub struct AllSetupsSimRequest<'a> {
    pub project: &'a ProjectDocument,
    pub cad_doc: &'a CadDocument,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_dir: Option<&'a Path>,
    pub resolution: SimResolution,
    pub engine: SimEngineKind,
    pub backend_choice: SimComputeBackendChoice,
    pub verify_config: SimVerificationConfig,
    pub cancel: Option<&'a AtomicBool>,
    pub progress: Option<SimProgressScope<'a>>,
}

impl<'a> AllSetupsSimRequest<'a> {
    /// Creates an [`AllSetupsSimRequest`] with `Standard` resolution, `Heightmap` engine,
    /// `Auto` backend choice, and default verification tolerances.
    pub fn new(
        project: &'a ProjectDocument,
        cad_doc: &'a CadDocument,
        cad_revision: u64,
        cache: &'a ToolpathCache,
    ) -> Self {
        Self {
            project,
            cad_doc,
            cad_revision,
            cache,
            project_dir: None,
            resolution: SimResolution::Standard,
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Auto,
            verify_config: SimVerificationConfig::default(),
            cancel: None,
            progress: None,
        }
    }
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
) -> Result<Option<CarryoverStockResult>, String> {
    let mut chain_cache: HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)> = HashMap::new();
    compute_setup_carryover_stock_with_cache(req, &mut chain_cache, None, None)
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
            project_dir: req.project_dir,
            kinematics: resolve_setup_kinematics(req.project, src_idx, req.project_dir),
            resolution: req.resolution,
            engine: SimEngineKind::TriDexel,
            backend_choice: req.backend_choice,
            verify_config: SimVerificationConfig::default(),
            cancel: req.cancel,
            progress: scope_for_setup(progress, progress_ranges, src_idx),
        };
        let src_carryover = compute_setup_carryover_stock_with_cache(
            &src_req,
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
        let src_sim = simulate_single_setup_inner(
            &src_req,
            &src_resolved,
            &src_ready,
            src_carryover.as_ref(),
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
    let base_carryover = compute_setup_carryover_stock(req)?;

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

    let td_req = SetupSimRequest {
        engine: SimEngineKind::TriDexel,
        verify_config: SimVerificationConfig::default(),
        progress: None,
        ..*req
    };
    let sim =
        simulate_single_setup_inner(&td_req, &resolved, &ready_upstream, base_carryover.as_ref())?;
    sim.as_tridexel().cloned().ok_or_else(|| {
        format!(
            "in-setup carryover for setup '{}' did not produce a Tri-Dexel stock solid",
            setup.name
        )
    })
}

fn simulate_single_setup_inner(
    req: &SetupSimRequest<'_>,
    resolved: &chip_cam::ResolvedSetup,
    ready: &[ReadySetupOperation<'_>],
    carryover: Option<&CarryoverStockResult>,
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
            req.project.load_fixture_cad_documents(req.project_dir)
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

    let mut input = SetupSimulationInput::new(req.cad_doc, resolved, &sim_inputs);
    input.target_bodies = &setup.target_bodies;
    input.fixtures = &resolved_fixtures;
    input.resolution = req.resolution;
    input.engine = req.engine;
    input.backend_choice = req.backend_choice;
    input.kinematics = req.kinematics;
    input.verify_config = req.verify_config;
    input.initial_stock = carryover.map(|c| &c.stock);
    input.initial_raw_volume_mm3 = carryover.map(|c| c.initial_raw_volume_mm3);
    input.carried_over_from_setup = carryover.map(|c| c.source_setup_name.as_str());
    input.progress = req.progress;
    input.cancel = req.cancel;

    run_setup_simulation(&input).map_err(|e| e.to_string())
}

/// Runs stock removal & 6-rule crash/gouge/fixture verification for a single Setup using the
/// engine, compute backend, fixture project directory, verification tolerances, and progress
/// reporter configured on [`SetupSimRequest`], automatically carrying over remaining volumetric
/// stock when `setup.carryover_from_setup_id` is set (`M62`, `M89`).
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
    let mut chain_cache: HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)> = HashMap::new();
    let progress_ranges = req
        .progress
        .map(|_| compute_carryover_progress_ranges(req, &chain_cache));
    let carryover = compute_setup_carryover_stock_with_cache(
        req,
        &mut chain_cache,
        req.progress,
        progress_ranges.as_ref(),
    )?;
    let dst_req = SetupSimRequest {
        progress: scope_for_setup(req.progress, progress_ranges.as_ref(), req.setup_idx),
        ..*req
    };
    simulate_single_setup_inner(&dst_req, &resolved, &ready, carryover.as_ref())
}

/// Simulates all Setups in `req.project` sequentially, caching and reusing each Setup's final
/// `TriDexelStock` for downstream `carryover_from_setup_id` chains so upstream Setups are never
/// simulated redundantly (`M62`, `M89`).
pub fn simulate_all_setups(
    req: &AllSetupsSimRequest<'_>,
) -> Result<Vec<SetupSimulationResult>, String> {
    let n_setups = req.project.setups.len();
    let mut results = Vec::with_capacity(n_setups);
    let mut chain_cache: HashMap<usize, (TriDexelStock, RigidFrame3, f64, String)> = HashMap::new();

    for (s_idx, setup) in req.project.setups.iter().enumerate() {
        if req
            .cancel
            .is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed))
        {
            return Err("simulation cancelled".to_string());
        }
        let setup_scope = req.progress.map(|p| {
            let start = s_idx as f64 / n_setups.max(1) as f64;
            let end = (s_idx + 1) as f64 / n_setups.max(1) as f64;
            p.sub_scope(start, end)
        });
        let setup_req = SetupSimRequest {
            project: req.project,
            setup_idx: s_idx,
            cad_doc: req.cad_doc,
            cad_revision: req.cad_revision,
            cache: req.cache,
            project_dir: req.project_dir,
            kinematics: resolve_setup_kinematics(req.project, s_idx, req.project_dir),
            resolution: req.resolution,
            engine: req.engine,
            backend_choice: req.backend_choice,
            verify_config: req.verify_config,
            cancel: req.cancel,
            progress: setup_scope,
        };
        let resolved = setup
            .resolve_with_sketches(
                req.cad_doc,
                &req.project.sketches,
                req.project.display_units,
            )
            .map_err(|e| e.to_string())?;
        let ready = collect_ready_setup_operations(req.project, s_idx, req.cache, req.cad_revision);
        let carryover =
            compute_setup_carryover_stock_with_cache(&setup_req, &mut chain_cache, None, None)?;
        let sim = simulate_single_setup_inner(&setup_req, &resolved, &ready, carryover.as_ref())?;

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

/// Per-setup outcome produced by [`export_all_setups_to_dir`].
#[derive(Debug, Clone)]
pub struct BatchSetupExportItem {
    pub setup_idx: usize,
    pub setup_name: String,
    pub post_profile_id: String,
    pub files_written: Vec<PathBuf>,
    pub lines_written: usize,
    pub warnings: Vec<String>,
    pub error: Option<String>,
}

/// Aggregated result of exporting all Setups in a project into an output directory.
#[derive(Debug, Clone)]
pub struct BatchExportSummary {
    pub output_dir: PathBuf,
    pub setups: Vec<BatchSetupExportItem>,
    pub total_files_written: usize,
    pub total_lines: usize,
}

impl BatchExportSummary {
    pub fn succeeded_setup_count(&self) -> usize {
        self.setups
            .iter()
            .filter(|s| s.error.is_none() && !s.files_written.is_empty())
            .count()
    }

    pub fn failed_setup_count(&self) -> usize {
        self.setups.iter().filter(|s| s.error.is_some()).count()
    }

    pub fn total_warning_count(&self) -> usize {
        self.setups.iter().map(|s| s.warnings.len()).sum()
    }
}

/// Sanitizes an arbitrary project or setup label into a portable ASCII G-code filename stem.
pub fn sanitize_program_stem(raw: &str, fallback: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.trim().chars() {
        if ch.is_ascii_alphanumeric() || matches!(ch, '-' | '_') {
            out.push(ch);
        } else if (ch.is_whitespace() || ch == '.') && !out.ends_with('_') && !out.is_empty() {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches(|c| c == '_' || c == '-');
    if trimmed.is_empty() {
        fallback.to_string()
    } else {
        trimmed.to_string()
    }
}

/// Post-processes and writes every Setup's G-code file(s) into `output_dir`, enforcing
/// single-component filename & extension safety rules.
pub fn export_all_setups_to_dir(
    project: &ProjectDocument,
    cad_doc: &CadDocument,
    cad_revision: u64,
    cache: &ToolpathCache,
    project_file_path: Option<&Path>,
    extra_profiles: &[PostProfile],
    output_dir: &Path,
) -> Result<BatchExportSummary, String> {
    if project.setups.is_empty() {
        return Err("Project has no Setups to export.".to_string());
    }
    std::fs::create_dir_all(output_dir).map_err(|e| {
        format!(
            "Failed to create output directory '{}': {e}",
            output_dir.display()
        )
    })?;

    let proj_dir = project_file_path.and_then(Path::parent);
    let mut available = load_available_post_profiles(proj_dir);
    for custom in extra_profiles {
        if let Some(existing) = available
            .iter_mut()
            .find(|p| !p.is_builtin && p.id.eq_ignore_ascii_case(&custom.id))
        {
            *existing = custom.clone();
        } else if !available
            .iter()
            .any(|p| p.id.eq_ignore_ascii_case(&custom.id))
        {
            available.push(custom.clone());
        }
    }

    let proj_label = project_file_path
        .and_then(Path::file_name)
        .and_then(|s| s.to_str())
        .unwrap_or(&project.project_name);
    let raw_base_stem = project_file_path
        .and_then(Path::file_stem)
        .and_then(|s| s.to_str())
        .map(|s| s.trim_end_matches(".chip"))
        .unwrap_or(&project.project_name);
    let base_stem = sanitize_program_stem(raw_base_stem, "program");

    let mut setups_out = Vec::with_capacity(project.setups.len());
    let mut total_files_written = 0usize;
    let mut total_lines = 0usize;

    for (s_idx, setup) in project.setups.iter().enumerate() {
        let mut warnings = Vec::new();
        let Some(post_profile) = available
            .iter()
            .find(|p| p.id.eq_ignore_ascii_case(&setup.post_profile))
            .cloned()
        else {
            setups_out.push(BatchSetupExportItem {
                setup_idx: s_idx,
                setup_name: setup.name.clone(),
                post_profile_id: setup.post_profile.clone(),
                files_written: Vec::new(),
                lines_written: 0,
                warnings,
                error: Some(format!(
                    "Unknown post-processor profile '{}'",
                    setup.post_profile
                )),
            });
            continue;
        };

        if post_profile.source == PostProfileSource::ProjectOrBundle {
            warnings.push(format!(
                "Uses custom post profile '{}' loaded from project/bundle directory; review header/footer G-code before machining.",
                post_profile.id
            ));
        }

        let setup_slug = sanitize_program_stem(&setup.id, &format!("setup_{}", s_idx + 1));
        let setup_stem = if project.setups.len() > 1 {
            format!("{base_stem}_{setup_slug}")
        } else {
            base_stem.clone()
        };

        let nc_files = match post_process_project_setup(&SetupPostRequest {
            project,
            setup_idx: s_idx,
            cad_doc,
            cad_revision,
            cache,
            post_profile: &post_profile,
            project_label: proj_label,
            program_stem: &setup_stem,
        }) {
            Ok(files) => files,
            Err(err) => {
                setups_out.push(BatchSetupExportItem {
                    setup_idx: s_idx,
                    setup_name: setup.name.clone(),
                    post_profile_id: post_profile.id,
                    files_written: Vec::new(),
                    lines_written: 0,
                    warnings,
                    error: Some(err),
                });
                continue;
            }
        };

        let mut written = Vec::new();
        let mut setup_lines = 0usize;
        let mut write_error = None;
        for nc in nc_files {
            warnings.extend(nc.warnings);
            let safe_name = match validate_safe_output_filename(&nc.filename) {
                Ok(name) => name,
                Err(err) => {
                    write_error = Some(format!("Unsafe output filename rejected: {err}"));
                    break;
                }
            };
            let dest = output_dir.join(safe_name);
            if let Err(err) = std::fs::write(&dest, &nc.content) {
                write_error = Some(format!("Failed writing '{}': {err}", dest.display()));
                break;
            }
            setup_lines += nc.line_count;
            total_lines += nc.line_count;
            total_files_written += 1;
            written.push(dest);
        }

        setups_out.push(BatchSetupExportItem {
            setup_idx: s_idx,
            setup_name: setup.name.clone(),
            post_profile_id: post_profile.id,
            files_written: written,
            lines_written: setup_lines,
            warnings,
            error: write_error,
        });
    }

    Ok(BatchExportSummary {
        output_dir: output_dir.to_path_buf(),
        setups: setups_out,
        total_files_written,
        total_lines,
    })
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
        DEFAULT_PURE_2D_SHEET_MARGIN_MM, ProjectDocument, SetupSimRequest, ToolpathCache,
        compute_operation_input_hash, resolve_setup_kinematics, setup_requires_volumetric_sim,
        simulate_project_setup,
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

        let res = simulate_project_setup(&SetupSimRequest {
            project: &project,
            setup_idx: 1,
            cad_doc: &doc,
            cad_revision: 0,
            cache: &cache,
            project_dir: None,
            kinematics: MachineKinematicsConfig::default(),
            resolution: SimResolution::Standard,
            engine: SimEngineKind::TriDexel,
            backend_choice: SimComputeBackendChoice::Cpu,
            verify_config: chip_sim::SimVerificationConfig::default(),
            cancel: None,
            progress: Some(tracker.full_scope()),
        })
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

    #[test]
    fn resolve_setup_kinematics_prefers_machine_id_then_falls_back_to_post_profile() {
        let mut proj = ProjectDocument::new("Kinematics Precedence");
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.post_profile = "linuxcnc".to_string();
        proj.setups = vec![setup];

        // No machine_id -> falls back to setup.post_profile ("linuxcnc")
        let linuxcnc_prof = chip_post::resolve_post_profile("linuxcnc", None).unwrap();
        let from_post = resolve_setup_kinematics(&proj, 0, None);
        assert_eq!(from_post, linuxcnc_prof.to_kinematics_spec());

        // Explicit machine_id -> honors machine kinematics over post_profile
        let carvera = chip_tools::builtin_carvera_air_machine();
        proj.machine_id = Some(carvera.id.clone());
        let from_machine = resolve_setup_kinematics(&proj, 0, None);
        assert_eq!(from_machine, carvera.kinematics);
        assert_ne!(
            from_machine.max_rapid_z_mm_min,
            linuxcnc_prof.to_kinematics_spec().max_rapid_z_mm_min
        );
    }

    #[test]
    fn setup_requires_volumetric_sim_covers_carryover_strategies_and_undercut_tools() {
        let mut proj = ProjectDocument::new("Volumetric Sim Check");
        let lib = default_starter_tool_library();
        proj.tools = lib.tools.clone();
        let flat_id = proj.tools[0].id.clone();

        let mut thread_tool = proj.tools[0].clone();
        thread_tool.id = chip_tools::ToolId("t_thread".to_string());
        thread_tool.geometry = chip_tools::CutterGeometry::ThreadMill {
            diameter_mm: 5.0,
            neck_diameter_mm: 3.0,
            pitch_mm: 1.0,
            teeth_count: 3,
            flutes: 3,
            flute_length_mm: 4.0,
            neck_length_mm: 15.0,
            shank_diameter_mm: 6.0,
            overall_length_mm: 50.0,
            thread_angle_deg: 60.0,
        };
        let thread_id = thread_tool.id.clone();
        proj.tools.push(thread_tool);

        // Plain 2.5D facing setup -> false
        let mut s = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        let mut op_face = Operation::new("op1", "Facing", OperationKind::Facing);
        op_face.tool_id = Some(flat_id.clone());
        s.operations.push(op_face);
        proj.setups = vec![s];
        assert!(!setup_requires_volumetric_sim(&proj, 0));

        // Carryover setup -> true
        proj.setups[0].carryover_from_setup_id = Some("s0".to_string());
        assert!(setup_requires_volumetric_sim(&proj, 0));
        proj.setups[0].carryover_from_setup_id = None;

        // Undercut strategy (Dovetail) -> true, suppressed -> false
        let mut op_dove = Operation::new("op2", "Dovetail", OperationKind::Dovetail);
        op_dove.tool_id = Some(flat_id);
        proj.setups[0].operations.push(op_dove);
        assert!(setup_requires_volumetric_sim(&proj, 0));
        proj.setups[0].operations[1].suppressed = true;
        assert!(!setup_requires_volumetric_sim(&proj, 0));
        proj.setups[0].operations.pop();

        // ThreadMill cutter even on non-undercut op -> true
        proj.setups[0].operations[0].tool_id = Some(thread_id);
        assert!(setup_requires_volumetric_sim(&proj, 0));
    }

    #[test]
    fn new_pure_2d_sheet_applies_canonical_margin_wcs_and_clamped_thickness() {
        let lib = default_starter_tool_library();
        let proj = ProjectDocument::new_pure_2d_sheet(
            "Sign Project",
            0.01, // below 0.5 mm clamp
            chip_math::DisplayUnit::Millimeter,
            &lib.tools,
        );
        assert_eq!(proj.project_name, "Sign Project");
        assert!(proj.models.is_empty());
        assert_eq!(proj.setups.len(), 1);
        assert_eq!(proj.tools.len(), lib.tools.len());
        assert_eq!(
            proj.setups[0].wcs.origin,
            WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::TOP_FRONT_LEFT
            }
        );
        match &proj.setups[0].stock {
            StockConfig::RelativeBox {
                side_pos_x,
                side_neg_x,
                side_pos_y,
                side_neg_y,
                top_offset_z,
                bottom_offset_z,
                sheet_thickness_z,
                ..
            } => {
                assert!((side_pos_x.value_mm - DEFAULT_PURE_2D_SHEET_MARGIN_MM).abs() < 1e-9);
                assert!((side_neg_x.value_mm - DEFAULT_PURE_2D_SHEET_MARGIN_MM).abs() < 1e-9);
                assert!((side_pos_y.value_mm - DEFAULT_PURE_2D_SHEET_MARGIN_MM).abs() < 1e-9);
                assert!((side_neg_y.value_mm - DEFAULT_PURE_2D_SHEET_MARGIN_MM).abs() < 1e-9);
                assert!((top_offset_z.value_mm - 0.0).abs() < 1e-9);
                assert!((bottom_offset_z.value_mm - 0.0).abs() < 1e-9);
                assert!((sheet_thickness_z.value_mm - 0.5).abs() < 1e-9);
            }
            other => panic!("expected RelativeBox stock, got {other:?}"),
        }
    }
}
