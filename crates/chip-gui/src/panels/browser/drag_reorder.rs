//! Generalized drag-and-drop reordering engine across all 6 semantically reorderable Project Tree
//! sections (`Setups`, `Operations`, `Sketches`, `Sketch Entities`, `Keep-Out Regions`, `Fixtures`),
//! enforcing strict parent-container boundaries and preserving selection/focus & Rest/Carryover dirty state.

use chip_project::{
    ProjectDocument, move_avoid_region_to_slot_in_project, move_fixture_to_slot_in_project,
    move_setup_to_slot_in_project, move_sketch_entity_to_slot_in_project,
    move_sketch_to_slot_in_project,
};
pub use chip_project::{move_operation_to_slot, remap_index_after_slot_move};
use egui::{CursorIcon, Id, Pos2, Rect, Stroke};

use crate::{
    panels::{InspectorFocus, browser::tree_widget::TREE_INDENT_STEP},
    theme::ThemeTokens,
};

/// Identifies a reorderable container in the Project Tree.
///
/// `Models` and `Project Tools` are intentionally omitted and never accept drag-and-drop reordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DragSection {
    Setups,
    Operations { setup_idx: usize },
    Sketches,
    SketchEntities { sketch_idx: usize },
    AvoidRegions,
    Fixtures { setup_idx: usize },
}

/// Active pointer drag payload stored in `egui` temp memory while dragging a tree row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DraggedTreeItem {
    pub section: DragSection,
    pub item_idx: usize,
}

/// Resolves the target insertion slot (`0..=row_rects.len()`) for `active` dropped onto
/// `target_section` at `pointer_pos`.
///
/// Enforces strict section boundaries: returns `None` if `active.section != target_section` or if
/// `row_rects` is empty.
pub fn resolve_drop_slot_for_section(
    active: DraggedTreeItem,
    target_section: DragSection,
    row_rects: &[Rect],
    pointer_pos: Pos2,
) -> Option<usize> {
    if active.section != target_section || row_rects.is_empty() {
        return None;
    }
    for (idx, r) in row_rects.iter().enumerate() {
        if pointer_pos.y < r.center().y {
            return Some(idx);
        }
    }
    Some(row_rects.len())
}

/// Renders the live accent-colored horizontal insertion indicator for `target_section` when
/// `active_drag` belongs to that section, returning `Some((from_idx, to_slot))` on pointer release.
pub(super) fn handle_section_drag_overlay(
    ui: &mut egui::Ui,
    active_drag: &mut Option<DraggedTreeItem>,
    target_section: DragSection,
    depth: usize,
    row_rects: &[Rect],
    drag_state_id: Id,
) -> Option<(usize, usize)> {
    let drag = (*active_drag)?;
    if drag.section != target_section || row_rects.is_empty() {
        return None;
    }
    ui.ctx().set_cursor_icon(CursorIcon::Grabbing);
    let ptr = ui.input(|i| i.pointer.interact_pos())?;
    let slot = resolve_drop_slot_for_section(drag, target_section, row_rects, ptr)?;
    let tok = ThemeTokens::from_ui(ui);

    if slot != drag.item_idx && slot != drag.item_idx + 1 {
        let y_line = if slot == 0 {
            row_rects[0].top() - 1.0
        } else if slot >= row_rects.len() {
            row_rects[row_rects.len() - 1].bottom() + 1.0
        } else {
            0.5 * (row_rects[slot - 1].bottom() + row_rects[slot].top())
        };
        let x_left = row_rects[0].left() + TREE_INDENT_STEP * (depth as f32) + 4.0;
        let x_right = row_rects[0].right() - 4.0;
        ui.painter().line_segment(
            [Pos2::new(x_left, y_line), Pos2::new(x_right, y_line)],
            Stroke::new(2.0, tok.accent),
        );
        ui.painter()
            .circle_filled(Pos2::new(x_left, y_line), 3.2, tok.accent);
    }

    if ui.input(|i| i.pointer.any_released()) {
        *active_drag = None;
        ui.ctx()
            .data_mut(|d| d.remove::<DraggedTreeItem>(drag_state_id));
        if slot != drag.item_idx && slot != drag.item_idx + 1 {
            return Some((drag.item_idx, slot));
        }
    }
    None
}

/// Moves `project.setups[from_idx]` to insertion slot `to_slot` (`0..=setups.len()`),
/// updating `active_setup_idx` and `focus`, and marking any Setup using `carryover_from_setup_id`
/// `Dirty` if its upstream setup ordering changed.
pub fn move_setup_to_slot(
    project: &mut ProjectDocument,
    from_idx: usize,
    to_slot: usize,
    active_setup_idx: &mut usize,
    focus: &mut InspectorFocus,
) -> Option<usize> {
    let dst_idx = move_setup_to_slot_in_project(project, from_idx, to_slot)?;
    *active_setup_idx = remap_index_after_slot_move(*active_setup_idx, from_idx, dst_idx);
    match focus {
        InspectorFocus::Setup { setup_idx } => {
            *setup_idx = remap_index_after_slot_move(*setup_idx, from_idx, dst_idx);
        }
        InspectorFocus::Operation { setup_idx, .. } => {
            *setup_idx = remap_index_after_slot_move(*setup_idx, from_idx, dst_idx);
        }
        _ => {}
    }
    Some(dst_idx)
}

/// Moves `project.sketches[from_idx]` to insertion slot `to_slot` (`0..=sketches.len()`),
/// updating `InspectorFocus::Sketch` cleanly.
pub fn move_sketch_to_slot(
    project: &mut ProjectDocument,
    from_idx: usize,
    to_slot: usize,
    focus: &mut InspectorFocus,
) -> Option<usize> {
    let dst_idx = move_sketch_to_slot_in_project(project, from_idx, to_slot)?;
    if let InspectorFocus::Sketch { sketch_idx } = focus {
        *sketch_idx = remap_index_after_slot_move(*sketch_idx, from_idx, dst_idx);
    }
    Some(dst_idx)
}

/// Moves `project.sketches[sketch_idx].entities[from_idx]` to insertion slot `to_slot`,
/// rebuilding sketch regions, updating `selected_sketch_entity_idx`, and dirtying dependent ops.
pub fn move_sketch_entity_to_slot(
    project: &mut ProjectDocument,
    sketch_idx: usize,
    from_idx: usize,
    to_slot: usize,
    selected_sketch_entity_idx: &mut Option<usize>,
) -> Option<usize> {
    let dst_idx = move_sketch_entity_to_slot_in_project(project, sketch_idx, from_idx, to_slot)?;
    if let Some(sel) = selected_sketch_entity_idx {
        *sel = remap_index_after_slot_move(*sel, from_idx, dst_idx);
    }
    Some(dst_idx)
}

/// Moves `project.avoid_regions[from_idx]` to insertion slot `to_slot`, updating
/// `InspectorFocus::KeepOutRegion` cleanly.
pub fn move_avoid_region_to_slot(
    project: &mut ProjectDocument,
    from_idx: usize,
    to_slot: usize,
    focus: &mut InspectorFocus,
) -> Option<usize> {
    let dst_idx = move_avoid_region_to_slot_in_project(project, from_idx, to_slot)?;
    if let InspectorFocus::KeepOutRegion { region_idx } = focus {
        *region_idx = remap_index_after_slot_move(*region_idx, from_idx, dst_idx);
    }
    Some(dst_idx)
}

/// Moves `project.setups[setup_idx].fixtures[from_idx]` to insertion slot `to_slot`,
/// updating `selected_fixture_idx` cleanly.
pub fn move_fixture_to_slot(
    project: &mut ProjectDocument,
    setup_idx: usize,
    from_idx: usize,
    to_slot: usize,
    selected_fixture_idx: &mut Option<usize>,
) -> Option<usize> {
    let dst_idx = move_fixture_to_slot_in_project(project, setup_idx, from_idx, to_slot)?;
    if let Some(sel) = selected_fixture_idx {
        *sel = remap_index_after_slot_move(*sel, from_idx, dst_idx);
    }
    Some(dst_idx)
}
