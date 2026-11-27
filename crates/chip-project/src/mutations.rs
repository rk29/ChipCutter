//! Pure UI-agnostic [`ProjectDocument`] mutations, Rest Machining / carryover dirty invalidation,
//! slot reordering, shared avoid-region promotion/linking, and workholding wizard plan application
//! (`chip-project`).

use chip_cam::{
    FixtureBody, FixtureKind, GeometrySelectionItem, Operation, OperationKind, RestMachiningMode,
    Setup, SharedAvoidRegion, WorkOffsetSlot,
};
use chip_math::{DVec2, DVec3, ParamLength};
use chip_sketch::create_default_sketch_entity;
use chip_tools::{CutterGeometry, Tool, ToolId, ToolLibrary, match_best_preset_for_material};

use crate::{
    cache::resolve_upstream_rest_op_indices,
    document::ProjectDocument,
    history::UndoHistory,
    wizards::{FlipJigPlan, SoftJawPlan},
};

/// Valid workholding fixture presets insertable into a Setup.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuickFixturePreset {
    ViseJaws,
    ToeClamp,
    Spoilboard,
    BoxKeepOut,
    CylinderKeepOut,
}

impl QuickFixturePreset {
    pub const ALL: [Self; 5] = [
        Self::ViseJaws,
        Self::ToeClamp,
        Self::Spoilboard,
        Self::BoxKeepOut,
        Self::CylinderKeepOut,
    ];

    pub const fn menu_label(self) -> &'static str {
        match self {
            Self::ViseJaws => "Vise Jaws",
            Self::ToeClamp => "Toe Clamp",
            Self::Spoilboard => "Spoilboard",
            Self::BoxKeepOut => "Box Keep-Out",
            Self::CylinderKeepOut => "Cylinder Keep-Out",
        }
    }
}

/// Captures each `FromPrecedingOperations` Rest Machining operation's current upstream operation IDs
/// prior to an insert/delete/reorder mutation.
pub fn snapshot_rest_upstream_ids(setup: &Setup) -> Vec<(String, Vec<String>)> {
    setup
        .operations
        .iter()
        .enumerate()
        .filter(|(_, op)| {
            matches!(
                op.passes.rest_machining.mode,
                RestMachiningMode::FromPrecedingOperations { .. }
            )
        })
        .map(|(idx, op)| {
            let upstream_ids = resolve_upstream_rest_op_indices(setup, idx)
                .into_iter()
                .map(|u| setup.operations[u].id.clone())
                .collect();
            (op.id.clone(), upstream_ids)
        })
        .collect()
}

/// Marks any `FromPrecedingOperations` Rest Machining operation `Dirty` if its resolved upstream
/// operation list changed compared to `before`.
pub fn invalidate_changed_rest_dependents(setup: &mut Setup, before: &[(String, Vec<String>)]) {
    for idx in 0..setup.operations.len() {
        if !matches!(
            setup.operations[idx].passes.rest_machining.mode,
            RestMachiningMode::FromPrecedingOperations { .. }
        ) {
            continue;
        }
        let after_ids: Vec<String> = resolve_upstream_rest_op_indices(setup, idx)
            .into_iter()
            .map(|u| setup.operations[u].id.clone())
            .collect();
        let op_id = &setup.operations[idx].id;
        let unchanged = before
            .iter()
            .find(|(id, _)| id == op_id)
            .is_some_and(|(_, prev)| prev == &after_ids);
        if !unchanged {
            setup.operations[idx].mark_dirty();
        }
    }
}

/// Marks `changed_op_indices` in `setup` as `Dirty`, plus any downstream
/// `FromPrecedingOperations` Rest Machining operations whose upstream coverage includes them.
pub fn mark_ops_dirty_with_rest(setup: &mut Setup, changed_op_indices: &[usize]) {
    if changed_op_indices.is_empty() {
        return;
    }
    let mut dirty_mask = vec![false; setup.operations.len()];
    for &idx in changed_op_indices {
        if idx < setup.operations.len() {
            setup.operations[idx].mark_dirty();
            dirty_mask[idx] = true;
        }
    }
    for idx in 0..setup.operations.len() {
        if dirty_mask[idx] {
            continue;
        }
        if matches!(
            setup.operations[idx].passes.rest_machining.mode,
            RestMachiningMode::FromPrecedingOperations { .. }
        ) {
            let upstream = resolve_upstream_rest_op_indices(setup, idx);
            if upstream
                .iter()
                .any(|&u| dirty_mask.get(u).copied().unwrap_or(false))
            {
                setup.operations[idx].mark_dirty();
                dirty_mask[idx] = true;
            }
        }
    }
}

/// Marks every operation across all Setups referencing `shared_id` (plus downstream Rest ops) `Dirty`.
pub fn mark_shared_avoid_dependents_dirty(project: &mut ProjectDocument, shared_id: &str) {
    for setup in &mut project.setups {
        let affected: Vec<usize> = setup
            .operations
            .iter()
            .enumerate()
            .filter(|(_, op)| {
                op.avoid
                    .shared_region_ids
                    .iter()
                    .any(|rid| rid == shared_id)
            })
            .map(|(idx, _)| idx)
            .collect();
        mark_ops_dirty_with_rest(setup, &affected);
    }
}

/// Marks every operation across all Setups referencing `sketch_id` (plus downstream Rest ops) `Dirty`.
pub fn mark_sketch_dependent_ops_dirty(project: &mut ProjectDocument, sketch_id: &str) {
    for setup in &mut project.setups {
        let affected: Vec<usize> = setup
            .operations
            .iter()
            .enumerate()
            .filter(|(_, op)| {
                op.selections.iter().any(|s| match s {
                    GeometrySelectionItem::SketchRegion { sketch_id: id, .. }
                    | GeometrySelectionItem::SketchEntity { sketch_id: id, .. }
                    | GeometrySelectionItem::SketchLayerAll { sketch_id: id, .. } => {
                        id == sketch_id
                    }
                    _ => false,
                })
            })
            .map(|(idx, _)| idx)
            .collect();
        mark_ops_dirty_with_rest(setup, &affected);
    }
}

/// Remaps an existing item index `idx` after moving `from_idx` to `dst_idx` within a slice.
pub const fn remap_index_after_slot_move(idx: usize, from_idx: usize, dst_idx: usize) -> usize {
    if idx == from_idx {
        dst_idx
    } else if from_idx < dst_idx && idx > from_idx && idx <= dst_idx {
        idx - 1
    } else if dst_idx < from_idx && idx >= dst_idx && idx < from_idx {
        idx + 1
    } else {
        idx
    }
}

fn resolve_target_index(len: usize, from_idx: usize, to_slot: usize) -> Option<usize> {
    if from_idx >= len || to_slot > len || to_slot == from_idx || to_slot == from_idx + 1 {
        return None;
    }
    Some(if to_slot > from_idx {
        to_slot - 1
    } else {
        to_slot
    })
}

/// Moves `setup.operations[from_idx]` to insertion slot `to_slot` (`0..=operations.len()`),
/// returning the new operation index if its position actually changed, and marking any affected
/// `FromPrecedingOperations` Rest Machining dependents `Dirty`.
pub fn move_operation_to_slot(setup: &mut Setup, from_idx: usize, to_slot: usize) -> Option<usize> {
    let dst_idx = resolve_target_index(setup.operations.len(), from_idx, to_slot)?;
    let before_rest = snapshot_rest_upstream_ids(setup);
    let moved = setup.operations.remove(from_idx);
    setup.operations.insert(dst_idx, moved);
    invalidate_changed_rest_dependents(setup, &before_rest);
    Some(dst_idx)
}

/// Moves `project.setups[from_idx]` to insertion slot `to_slot` (`0..=setups.len()`),
/// marking any Setup using `carryover_from_setup_id` `Dirty` if its upstream setup ordering changed.
pub fn move_setup_to_slot_in_project(
    project: &mut ProjectDocument,
    from_idx: usize,
    to_slot: usize,
) -> Option<usize> {
    let dst_idx = resolve_target_index(project.setups.len(), from_idx, to_slot)?;
    let before_carryover: Vec<(String, Vec<String>)> = project
        .setups
        .iter()
        .enumerate()
        .filter(|(_, s)| s.carryover_from_setup_id.is_some())
        .map(|(idx, s)| {
            let upstream = project.setups[..idx].iter().map(|u| u.id.clone()).collect();
            (s.id.clone(), upstream)
        })
        .collect();

    let moved = project.setups.remove(from_idx);
    project.setups.insert(dst_idx, moved);

    let setup_ids: Vec<String> = project.setups.iter().map(|s| s.id.clone()).collect();
    for (idx, setup) in project.setups.iter_mut().enumerate() {
        if setup.carryover_from_setup_id.is_none() {
            continue;
        }
        let after_upstream: Vec<String> = setup_ids[..idx].to_vec();
        let unchanged = before_carryover
            .iter()
            .find(|(id, _)| id == &setup.id)
            .is_some_and(|(_, prev)| prev == &after_upstream);
        if !unchanged {
            for op in &mut setup.operations {
                op.mark_dirty();
            }
        }
    }

    Some(dst_idx)
}

/// Moves `project.sketches[from_idx]` to insertion slot `to_slot` (`0..=sketches.len()`).
pub fn move_sketch_to_slot_in_project(
    project: &mut ProjectDocument,
    from_idx: usize,
    to_slot: usize,
) -> Option<usize> {
    let dst_idx = resolve_target_index(project.sketches.len(), from_idx, to_slot)?;
    let moved = project.sketches.remove(from_idx);
    project.sketches.insert(dst_idx, moved);
    Some(dst_idx)
}

/// Moves `project.sketches[sketch_idx].entities[from_idx]` to insertion slot `to_slot`,
/// rebuilding sketch regions and dirtying dependent ops.
pub fn move_sketch_entity_to_slot_in_project(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    from_idx: usize,
    to_slot: usize,
) -> Option<usize> {
    let sk = project.sketches.get_mut(sketch_idx)?;
    let dst_idx = resolve_target_index(sk.entities.len(), from_idx, to_slot)?;
    let moved = sk.entities.remove(from_idx);
    sk.entities.insert(dst_idx, moved);
    sk.rebuild_regions(None);
    let sk_id = sk.id.clone();
    mark_sketch_dependent_ops_dirty(project, &sk_id);
    Some(dst_idx)
}

/// Moves `project.avoid_regions[from_idx]` to insertion slot `to_slot`.
pub fn move_avoid_region_to_slot_in_project(
    project: &mut ProjectDocument,
    from_idx: usize,
    to_slot: usize,
) -> Option<usize> {
    let dst_idx = resolve_target_index(project.avoid_regions.len(), from_idx, to_slot)?;
    let moved = project.avoid_regions.remove(from_idx);
    project.avoid_regions.insert(dst_idx, moved);
    Some(dst_idx)
}

/// Moves `project.setups[setup_idx].fixtures[from_idx]` to insertion slot `to_slot`.
pub fn move_fixture_to_slot_in_project(
    project: &mut ProjectDocument,
    setup_idx: usize,
    from_idx: usize,
    to_slot: usize,
) -> Option<usize> {
    let setup = project.setups.get_mut(setup_idx)?;
    let dst_idx = resolve_target_index(setup.fixtures.len(), from_idx, to_slot)?;
    let moved = setup.fixtures.remove(from_idx);
    setup.fixtures.insert(dst_idx, moved);
    Some(dst_idx)
}

/// Duplicates `setup.operations[op_idx]` immediately below `op_idx`, returning the new index and
/// propagating Rest Machining dirty invalidation to downstream dependents.
pub fn duplicate_operation_in_setup(setup: &mut Setup, op_idx: usize) -> Option<usize> {
    if op_idx >= setup.operations.len() {
        return None;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    let mut dup = setup.operations[op_idx].clone();
    let mut n = setup.operations.len() + 1;
    let base_id = dup.id.clone();
    dup.id = loop {
        let cand = format!("{base_id}_copy_{n}");
        if !setup.operations.iter().any(|o| o.id == cand) {
            break cand;
        }
        n += 1;
    };
    dup.name = format!("{} (Copy)", dup.name);
    dup.mark_dirty();
    let new_idx = op_idx + 1;
    setup.operations.insert(new_idx, dup);
    invalidate_changed_rest_dependents(setup, &before_rest);
    Some(new_idx)
}

/// Removes `setup.operations[op_idx]`, propagating Rest Machining dirty invalidation to downstream
/// dependents.
pub fn delete_operation_in_setup(setup: &mut Setup, op_idx: usize) -> bool {
    if op_idx >= setup.operations.len() {
        return false;
    }
    let before_rest = snapshot_rest_upstream_ids(setup);
    setup.operations.remove(op_idx);
    invalidate_changed_rest_dependents(setup, &before_rest);
    true
}

fn next_unique_op_id(setup: &Setup, setup_idx: usize, suffix: &str) -> String {
    let mut n = setup.operations.len() + 1;
    loop {
        let cand = format!("op_{setup_idx}_{n}_{suffix}");
        if !setup.operations.iter().any(|o| o.id == cand) {
            return cand;
        }
        n += 1;
    }
}

/// Creates a default [`Operation`] of `kind` inside `project.setups[setup_idx]`, automatically
/// selecting and embedding the first compatible cutter from `project.tools` or `global_tools`.
pub fn create_default_operation_in_setup(
    project: &mut ProjectDocument,
    global_tools: &ToolLibrary,
    setup_idx: usize,
    kind: OperationKind,
) -> Option<usize> {
    if setup_idx >= project.setups.len() {
        return None;
    }
    let next_op_num = project.setups[setup_idx].operations.len() + 1;
    let op_id = next_unique_op_id(
        &project.setups[setup_idx],
        setup_idx,
        &next_op_num.to_string(),
    );
    let mut op = Operation::new(
        op_id,
        format!("{} {next_op_num}", kind.default_name()),
        kind,
    );
    if let Some(compat) = project
        .tools
        .iter()
        .chain(global_tools.tools.iter())
        .find(|t| kind.is_compatible_with_tool(t))
        .cloned()
    {
        let material = project.effective_workpiece_material();
        project.ensure_tool_embedded(&compat);
        op.preset_id = match_best_preset_for_material(&compat, &material).map(|p| p.id.clone());
        op.tool_id = Some(compat.id);
    }
    project.setups[setup_idx].operations.push(op);
    Some(project.setups[setup_idx].operations.len() - 1)
}

/// Creates a **2D Adaptive Clearing** operation inside `project.setups[setup_idx]`.
pub fn create_adaptive_2d_operation_in_setup(
    project: &mut ProjectDocument,
    global_tools: &ToolLibrary,
    setup_idx: usize,
) -> Option<usize> {
    if setup_idx >= project.setups.len() {
        return None;
    }
    let next_op_num = project.setups[setup_idx].operations.len() + 1;
    let op_id = next_unique_op_id(&project.setups[setup_idx], setup_idx, "adaptive");
    let mut op = Operation::new_adaptive_2d(op_id, format!("2D Adaptive {next_op_num}"));
    if let Some(compat) = project
        .tools
        .iter()
        .chain(global_tools.tools.iter())
        .find(|t| OperationKind::Pocket2D.is_compatible_with_tool(t))
        .cloned()
    {
        let material = project.effective_workpiece_material();
        project.ensure_tool_embedded(&compat);
        op.preset_id = match_best_preset_for_material(&compat, &material).map(|p| p.id.clone());
        op.tool_id = Some(compat.id);
    }
    project.setups[setup_idx].operations.push(op);
    Some(project.setups[setup_idx].operations.len() - 1)
}

/// Allocates a unique Setup ID starting from `base` (`base`, `base_2`, `base_3`, ...).
pub fn next_unique_setup_id_with_base(project: &ProjectDocument, base: &str) -> String {
    if !project.setups.iter().any(|s| s.id == base) {
        return base.to_string();
    }
    let mut n = 2usize;
    loop {
        let cand = format!("{base}_{n}");
        if !project.setups.iter().any(|s| s.id == cand) {
            return cand;
        }
        n += 1;
    }
}

/// Allocates a unique numbered Setup ID (`setup_<n>`).
pub fn next_unique_setup_id(project: &ProjectDocument) -> String {
    let mut n = project.setups.len() + 1;
    loop {
        let cand = format!("setup_{n}");
        if !project.setups.iter().any(|s| s.id == cand) {
            return cand;
        }
        n += 1;
    }
}

/// Picks the next unused secondary [`WorkOffsetSlot`] (`G55`..=`G59`), falling back to `G55`.
pub fn next_work_offset_slot(project: &ProjectDocument) -> WorkOffsetSlot {
    for slot in [
        WorkOffsetSlot::G55,
        WorkOffsetSlot::G56,
        WorkOffsetSlot::G57,
        WorkOffsetSlot::G58,
        WorkOffsetSlot::G59,
    ] {
        if !project.setups.iter().any(|s| s.wcs_slot == slot) {
            return slot;
        }
    }
    WorkOffsetSlot::G55
}

/// Appends a new default clamping [`Setup`] to `project` and returns `(new_idx, setup_number)`.
pub fn add_setup_in_project(project: &mut ProjectDocument) -> (usize, usize) {
    let next_n = project.setups.len() + 1;
    let slot = match next_n {
        1 => WorkOffsetSlot::G54,
        2 => WorkOffsetSlot::G55,
        3 => WorkOffsetSlot::G56,
        4 => WorkOffsetSlot::G57,
        _ => WorkOffsetSlot::G58,
    };
    let id = next_unique_setup_id(project);
    project.setups.push(Setup::new(
        id,
        format!("Setup {next_n} - Op {next_n}"),
        slot,
    ));
    (project.setups.len() - 1, next_n)
}

/// Duplicates `project.setups[setup_idx]` immediately below it with unique ID and dirty cloned ops.
pub fn duplicate_setup_in_project(
    project: &mut ProjectDocument,
    setup_idx: usize,
) -> Option<usize> {
    let mut copy = project.setups.get(setup_idx)?.clone();
    copy.id = next_unique_setup_id(project);
    copy.name = format!("{} (Copy)", copy.name);
    for op in &mut copy.operations {
        op.mark_dirty();
    }
    let dst = setup_idx + 1;
    project.setups.insert(dst, copy);
    Some(dst)
}

/// Duplicates `project.sketches[sketch_idx]` immediately below it with a unique ID.
pub fn duplicate_sketch_in_project(
    project: &mut ProjectDocument,
    sketch_idx: usize,
) -> Option<usize> {
    let mut copy = project.sketches.get(sketch_idx)?.clone();
    let mut n = project.sketches.len() + 1;
    copy.id = loop {
        let cand = format!("sketch_{n}");
        if !project.sketches.iter().any(|s| s.id == cand) {
            break cand;
        }
        n += 1;
    };
    copy.name = format!("{} (Copy)", copy.name);
    let dst = sketch_idx + 1;
    project.sketches.insert(dst, copy);
    Some(dst)
}

/// Inserts a preset entity into `project.sketches[sketch_idx]`, rebuilds regions, and dirties ops.
pub fn add_sketch_entity_preset_in_project(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    spec: u8,
    sub_layer: Option<&str>,
) -> Option<usize> {
    let sk = project.sketches.get_mut(sketch_idx)?;
    let mut n = sk.entities.len() + 1;
    let entry = loop {
        let cand = create_default_sketch_entity(n, spec, sub_layer);
        if !sk.entities.iter().any(|e| e.id == cand.id) {
            break cand;
        }
        n += 1;
    };
    sk.entities.push(entry);
    sk.rebuild_regions(None);
    let sk_id = sk.id.clone();
    let new_idx = sk.entities.len() - 1;
    mark_sketch_dependent_ops_dirty(project, &sk_id);
    Some(new_idx)
}

/// Duplicates `project.sketches[sketch_idx].entities[entity_idx]` directly beneath it.
pub fn duplicate_sketch_entity_in_sketch(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    entity_idx: usize,
) -> Option<usize> {
    let sk = project.sketches.get_mut(sketch_idx)?;
    let mut copy = sk.entities.get(entity_idx)?.clone();
    let base_id = copy.id.clone();
    let mut n = sk.entities.len() + 1;
    copy.id = loop {
        let cand = format!("{base_id}_copy_{n}");
        if !sk.entities.iter().any(|e| e.id == cand) {
            break cand;
        }
        n += 1;
    };
    copy.name = format!("{} (Copy)", copy.name);
    let dst = entity_idx + 1;
    sk.entities.insert(dst, copy);
    sk.rebuild_regions(None);
    let sk_id = sk.id.clone();
    mark_sketch_dependent_ops_dirty(project, &sk_id);
    Some(dst)
}

fn next_unique_fixture_id(setup: &Setup, prefix: &str) -> String {
    let mut n = setup.fixtures.len() + 1;
    loop {
        let cand = format!("{prefix}_{n}");
        if !setup.fixtures.iter().any(|f| f.id == cand) {
            return cand;
        }
        n += 1;
    }
}

/// Adds a default workholding fixture of `preset` into `project.setups[setup_idx]` and marks setup ops Dirty.
pub fn add_fixture_preset_in_setup(
    project: &mut ProjectDocument,
    setup_idx: usize,
    preset: QuickFixturePreset,
) -> Option<usize> {
    let setup = project.setups.get_mut(setup_idx)?;
    let body = match preset {
        QuickFixturePreset::ViseJaws => {
            let id = next_unique_fixture_id(setup, "vise");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::ViseJaws { .. }))
                .count()
                + 1;
            FixtureBody::default_vise(id, format!("Machinist Vise {n}"))
        }
        QuickFixturePreset::ToeClamp => {
            let id = next_unique_fixture_id(setup, "clamp");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::ToeClamp { .. }))
                .count()
                + 1;
            let yaw = ((n - 1) % 4) as f64 * 90.0;
            FixtureBody::default_toe_clamp(id, format!("Toe Clamp {n}"), DVec2::new(40.0, 0.0), yaw)
        }
        QuickFixturePreset::Spoilboard => {
            let id = next_unique_fixture_id(setup, "spoilboard");
            FixtureBody::default_spoilboard(id, "MDF Spoilboard")
        }
        QuickFixturePreset::BoxKeepOut => {
            let id = next_unique_fixture_id(setup, "keepout");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::BoxKeepOut { .. }))
                .count()
                + 1;
            FixtureBody::default_box_keepout(
                id,
                format!("Keep-Out Box {n}"),
                DVec3::new(50.0, 0.0, 0.0),
            )
        }
        QuickFixturePreset::CylinderKeepOut => {
            let id = next_unique_fixture_id(setup, "pin");
            let n = setup
                .fixtures
                .iter()
                .filter(|f| matches!(f.kind, FixtureKind::CylinderKeepOut { .. }))
                .count()
                + 1;
            FixtureBody::default_cylinder_keepout(
                id,
                format!("Keep-Out Pin {n}"),
                DVec2::new(-30.0, 30.0),
                -15.0,
                10.0,
            )
        }
    };
    setup.fixtures.push(body);
    for op in &mut setup.operations {
        op.mark_dirty();
    }
    Some(setup.fixtures.len() - 1)
}

/// Duplicates `project.setups[setup_idx].fixtures[fixture_idx]` directly beneath it and marks ops Dirty.
pub fn duplicate_fixture_in_setup(
    project: &mut ProjectDocument,
    setup_idx: usize,
    fixture_idx: usize,
) -> Option<usize> {
    let setup = project.setups.get_mut(setup_idx)?;
    let mut copy = setup.fixtures.get(fixture_idx)?.clone();
    copy.id = next_unique_fixture_id(setup, "fixture");
    copy.name = format!("{} (Copy)", copy.name);
    if let FixtureKind::ToeClamp { anchor_xy, .. } = &mut copy.kind {
        anchor_xy[1] = ParamLength::from_mm(anchor_xy[1].value_mm + 20.0);
    } else if let FixtureKind::BoxKeepOut { center_wcs, .. } = &mut copy.kind {
        center_wcs[1] = ParamLength::from_mm(center_wcs[1].value_mm + 20.0);
    } else if let FixtureKind::CylinderKeepOut { center_xy_wcs, .. } = &mut copy.kind {
        center_xy_wcs[1] = ParamLength::from_mm(center_xy_wcs[1].value_mm + 20.0);
    }
    let dst = fixture_idx + 1;
    setup.fixtures.insert(dst, copy);
    for op in &mut setup.operations {
        op.mark_dirty();
    }
    Some(dst)
}

/// Generates a unique `SharedAvoidRegion` ID and default display name for `project`.
pub fn next_shared_avoid_id_and_name(
    project: &ProjectDocument,
    base_name: Option<&str>,
) -> (String, String) {
    let mut n = project.avoid_regions.len() + 1;
    let id = loop {
        let cand = format!("avoid_{n}");
        if !project.avoid_regions.iter().any(|r| r.id == cand) {
            break cand;
        }
        n += 1;
    };
    let name = match base_name.map(str::trim).filter(|s| !s.is_empty()) {
        Some(custom) => custom.to_string(),
        None => {
            let mut k = project.avoid_regions.len() + 1;
            loop {
                let cand = format!("Keep-Out {k}");
                if !project.avoid_regions.iter().any(|r| r.name == cand) {
                    break cand;
                }
                k += 1;
            }
        }
    };
    (id, name)
}

/// Promotes `project.setups[setup_idx].operations[op_idx]`'s local `avoid.selections` and
/// `avoid.fixture_ids` into a new named [`SharedAvoidRegion`], clears those promoted local fields,
/// and links the operation to the new shared ID in one atomic Undo step.
pub fn promote_local_avoid_to_shared(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    setup_idx: usize,
    op_idx: usize,
    requested_name: Option<&str>,
) -> Option<String> {
    let op = project
        .setups
        .get(setup_idx)
        .and_then(|s| s.operations.get(op_idx))?;
    if op.avoid.selections.is_empty() && op.avoid.fixture_ids.is_empty() {
        return None;
    }
    history.record_before_edit("Save Local Keep-Out as Shared", None, project);
    let (id, name) = next_shared_avoid_id_and_name(project, requested_name);
    let mut shared = SharedAvoidRegion::new(id.clone(), name);
    {
        let op_mut = &mut project.setups[setup_idx].operations[op_idx];
        shared.selections = std::mem::take(&mut op_mut.avoid.selections);
        shared.fixture_ids = std::mem::take(&mut op_mut.avoid.fixture_ids);
        if !op_mut.avoid.shared_region_ids.contains(&id) {
            op_mut.avoid.shared_region_ids.push(id.clone());
        }
    }
    project.avoid_regions.push(shared);
    mark_ops_dirty_with_rest(&mut project.setups[setup_idx], &[op_idx]);
    Some(id)
}

/// Creates a new empty [`SharedAvoidRegion`], links it to `(setup_idx, op_idx)`, and records Undo.
pub fn create_and_link_shared_avoid(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    setup_idx: usize,
    op_idx: usize,
    requested_name: Option<&str>,
) -> Option<String> {
    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        return None;
    }
    history.record_before_edit("New Shared Keep-Out Region", None, project);
    let (id, name) = next_shared_avoid_id_and_name(project, requested_name);
    project
        .avoid_regions
        .push(SharedAvoidRegion::new(id.clone(), name));
    project.setups[setup_idx].operations[op_idx]
        .avoid
        .shared_region_ids
        .push(id.clone());
    mark_ops_dirty_with_rest(&mut project.setups[setup_idx], &[op_idx]);
    Some(id)
}

/// Copies a linked [`SharedAvoidRegion`]'s selections & fixture IDs into `op.avoid` and unlinks `shared_id`
/// in one atomic Undo step.
pub fn detach_shared_avoid_to_local(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    setup_idx: usize,
    op_idx: usize,
    shared_id: &str,
) -> bool {
    let Some(shared_clone) = project.avoid_region(shared_id).cloned() else {
        return false;
    };
    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        return false;
    }
    history.record_before_edit("Detach Shared Keep-Out to Local", None, project);
    let op_mut = &mut project.setups[setup_idx].operations[op_idx];
    for sel in shared_clone.selections {
        if !op_mut.avoid.selections.contains(&sel) {
            op_mut.avoid.selections.push(sel);
        }
    }
    for fid in shared_clone.fixture_ids {
        if !op_mut.avoid.fixture_ids.contains(&fid) {
            op_mut.avoid.fixture_ids.push(fid);
        }
    }
    op_mut.avoid.shared_region_ids.retain(|id| id != shared_id);
    mark_ops_dirty_with_rest(&mut project.setups[setup_idx], &[op_idx]);
    true
}

/// Toggles whether `project.setups[setup_idx].operations[target_op_idx]` links `shared_id`,
/// pushing an atomic Undo snapshot and marking affected + downstream Rest operations `Dirty`.
pub fn toggle_shared_avoid_link(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    setup_idx: usize,
    target_op_idx: usize,
    shared_id: &str,
) -> bool {
    if setup_idx >= project.setups.len()
        || target_op_idx >= project.setups[setup_idx].operations.len()
    {
        return false;
    }
    let currently_linked = project.setups[setup_idx].operations[target_op_idx]
        .avoid
        .shared_region_ids
        .iter()
        .any(|id| id == shared_id);
    let label = if currently_linked {
        "Unlink Shared Keep-Out Region"
    } else {
        "Link Shared Keep-Out Region"
    };
    history.record_before_edit(label, None, project);
    let op_mut = &mut project.setups[setup_idx].operations[target_op_idx];
    if currently_linked {
        op_mut.avoid.shared_region_ids.retain(|id| id != shared_id);
    } else {
        op_mut.avoid.shared_region_ids.push(shared_id.to_string());
    }
    mark_ops_dirty_with_rest(&mut project.setups[setup_idx], &[target_op_idx]);
    true
}

/// Links or unlinks `shared_id` across every operation in `project.setups[setup_idx]`.
pub fn set_shared_avoid_for_all_setup_ops(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    setup_idx: usize,
    shared_id: &str,
    link: bool,
) -> bool {
    let Some(setup) = project.setups.get(setup_idx) else {
        return false;
    };
    let changed_indices: Vec<usize> = setup
        .operations
        .iter()
        .enumerate()
        .filter(|(_, op)| {
            let has = op.avoid.shared_region_ids.iter().any(|id| id == shared_id);
            has != link
        })
        .map(|(idx, _)| idx)
        .collect();
    if changed_indices.is_empty() {
        return false;
    }
    let label = if link {
        "Apply Shared Keep-Out to Setup Operations"
    } else {
        "Remove Shared Keep-Out from Setup Operations"
    };
    history.record_before_edit(label, None, project);
    let setup_mut = &mut project.setups[setup_idx];
    for &idx in &changed_indices {
        if link {
            setup_mut.operations[idx]
                .avoid
                .shared_region_ids
                .push(shared_id.to_string());
        } else {
            setup_mut.operations[idx]
                .avoid
                .shared_region_ids
                .retain(|id| id != shared_id);
        }
    }
    mark_ops_dirty_with_rest(setup_mut, &changed_indices);
    true
}

/// Selects the best dowel-pin drill or helical-boring endmill for `hole_dia_mm`.
pub fn pick_pin_tool(tools: &[Tool], hole_dia_mm: f64) -> (Option<ToolId>, Option<f64>) {
    if let Some(drill) = tools.iter().find(|t| {
        matches!(
            t.geometry,
            CutterGeometry::TwistDrill { diameter_mm, .. }
            if (diameter_mm - hole_dia_mm).abs() <= 0.15
        )
    }) {
        return (
            Some(drill.id.clone()),
            Some(drill.geometry.cutting_diameter_mm()),
        );
    }
    if let Some(em) = tools.iter().find(|t| {
        matches!(
            t.geometry,
            CutterGeometry::FlatEndmill { diameter_mm, .. }
            if diameter_mm < hole_dia_mm - 0.2
        )
    }) {
        return (Some(em.id.clone()), Some(em.geometry.cutting_diameter_mm()));
    }
    if let Some(first) = tools.first() {
        return (
            Some(first.id.clone()),
            Some(first.geometry.cutting_diameter_mm()),
        );
    }
    (None, None)
}

/// Selects the first Flat Endmill (or fallback first tool) for soft-jaw pocket machining.
pub fn pick_flat_endmill_id(tools: &[Tool]) -> Option<ToolId> {
    tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
        .or_else(|| tools.first())
        .map(|t| t.id.clone())
}

/// Applies a computed [`FlipJigPlan`] to `project` against `project.setups[setup_idx]`, returning
/// the index of the newly appended `setup2_flip`.
pub fn apply_flip_jig_plan(
    project: &mut ProjectDocument,
    setup_idx: usize,
    plan: FlipJigPlan,
) -> Option<usize> {
    if setup_idx >= project.setups.len() {
        return None;
    }
    if let Some(new_stock) = plan.updated_setup1_stock {
        project.setups[setup_idx].stock = new_stock;
    }
    project.sketches.push(plan.pin_sketch);
    project.setups[setup_idx]
        .operations
        .insert(0, plan.setup1_pin_op);
    for op in &mut project.setups[setup_idx].operations {
        op.mark_dirty();
    }
    let new_setup_idx = project.setups.len();
    project.setups.push(plan.setup2_flip);
    if let Some(sb_sk) = plan.spoilboard_sketch {
        project.sketches.push(sb_sk);
    }
    if let Some(sb_setup) = plan.spoilboard_setup {
        project.setups.push(sb_setup);
    }
    Some(new_setup_idx)
}

/// Applies a computed [`SoftJawPlan`]'s machining setup and pocket sketch to `project`, returning
/// the index of the newly appended soft-jaw Setup when present.
pub fn apply_soft_jaw_plan(project: &mut ProjectDocument, mut plan: SoftJawPlan) -> Option<usize> {
    let sj_setup = plan.machining_setup.take()?;
    if let Some(sk) = plan.pocket_sketch.take() {
        project.sketches.push(sk);
    }
    let new_setup_idx = project.setups.len();
    project.setups.push(sj_setup);
    Some(new_setup_idx)
}
