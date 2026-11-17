use std::collections::VecDeque;

use crate::document::ProjectDocument;

#[derive(Debug, Clone)]
struct SnapshotEntry {
    action_label: String,
    coalesce_key: Option<String>,
    document: ProjectDocument,
}

///Bounded (`100`-step default) `< 1 ms` Undo/Redo stack over lightweight [`ProjectDocument`] states (`08-ui-viewport.md`).
///
/// Heavy `CadDocument` meshes, `CadPicker` R-trees, and GPU buffers live out-of-band so
/// `Ctrl+Z` / `Ctrl+Y` never reloads or re-tessellates CAD models.
#[derive(Debug, Clone)]
pub struct UndoHistory {
    max_depth: usize,
    undo_stack: VecDeque<SnapshotEntry>,
    redo_stack: Vec<SnapshotEntry>,
}

impl Default for UndoHistory {
    fn default() -> Self {
        Self::new(100)
    }
}

impl UndoHistory {
    pub fn new(max_depth: usize) -> Self {
        Self {
            max_depth: max_depth.max(1),
            undo_stack: VecDeque::new(),
            redo_stack: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
    }

    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    pub fn peek_undo_label(&self) -> Option<&str> {
        self.undo_stack.back().map(|e| e.action_label.as_str())
    }

    pub fn peek_redo_label(&self) -> Option<&str> {
        self.redo_stack.last().map(|e| e.action_label.as_str())
    }

    pub fn undo_len(&self) -> usize {
        self.undo_stack.len()
    }

    /// Captures `before_mutation` onto the undo stack prior to applying a document edit.
    ///
    /// If `coalesce_key` matches the most recent undo entry (e.g., consecutive ticks of a slider),
    /// the earlier pre-edit snapshot is retained rather than pushing redundant intermediate states.
    pub fn record_before_edit(
        &mut self,
        action_label: impl Into<String>,
        coalesce_key: Option<&str>,
        before_mutation: &ProjectDocument,
    ) {
        self.redo_stack.clear();

        if let Some(key) = coalesce_key
            && let Some(last) = self.undo_stack.back()
            && last.coalesce_key.as_deref() == Some(key)
        {
            return;
        }

        if self.undo_stack.len() >= self.max_depth {
            self.undo_stack.pop_front();
        }
        self.undo_stack.push_back(SnapshotEntry {
            action_label: action_label.into(),
            coalesce_key: coalesce_key.map(str::to_string),
            document: before_mutation.clone(),
        });
    }

    /// Reverts `current_doc` to the previous snapshot and pushes the current state onto `redo_stack`.
    /// Returns the human-readable `action_label` that was undone.
    pub fn undo(&mut self, current_doc: &mut ProjectDocument) -> Option<String> {
        let entry = self.undo_stack.pop_back()?;
        let label = entry.action_label.clone();
        let old_current = std::mem::replace(current_doc, entry.document);
        self.redo_stack.push(SnapshotEntry {
            action_label: label.clone(),
            coalesce_key: None,
            document: old_current,
        });
        Some(label)
    }

    /// Re-applies the most recently undone edit onto `current_doc`.
    /// Returns the human-readable `action_label` that was redone.
    pub fn redo(&mut self, current_doc: &mut ProjectDocument) -> Option<String> {
        let entry = self.redo_stack.pop()?;
        let label = entry.action_label.clone();
        let old_current = std::mem::replace(current_doc, entry.document);
        if self.undo_stack.len() >= self.max_depth {
            self.undo_stack.pop_front();
        }
        self.undo_stack.push_back(SnapshotEntry {
            action_label: label.clone(),
            coalesce_key: None,
            document: old_current,
        });
        Some(label)
    }
}
