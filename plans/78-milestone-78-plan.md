# Milestone 78 Implementation Plan — Streamlined Tool Management, Single `"Tools"` Entry Point, `Project Tools` Controls, & Interactive Machine-Switch Tool Swapping

## Goal

Make selecting, managing, pruning, and switching tools across machines effortless:
1. **Single `"Tools"` Button:** Consolidate the three duplicate `"Tools"` buttons (`menus.rs`, `toolbar.rs`, `browser.rs`) down to **one** clear entry point.
2. **Actionable `"Project Tools"` Card:** Add direct inline **`+ Add Tool…`**, per-tool **`Remove`**, and 1-click **`Remove Unused (N)`** controls right inside the CAM Browser's `Project Tools` card.
3. **Easier Project & Operation Tool Selection:** Improve the operation `Tool` selector to group and prioritize compatible project & active-machine cutters with quick search/filtering instead of listing incompatible cutters inline.
4. **Interactive Machine-Switch Tool Swapping:** Whenever the user switches the project to a different CNC machine, automatically detect any project tools referenced by operations that are missing from the new machine and present a clean **Swap Machine Tools** dialog proposing smart replacements so the user can swap tools in one atomic, undoable step.

---

## Problem Diagnosis

1. **Three Competing `"Tools"` Buttons:**
   - Top menu bar has a top-level `"Tools"` menu (`crates/chip-gui/src/app/menus.rs:31`).
   - Top action bar has a `"Tools"` button (`crates/chip-gui/src/app/toolbar.rs:199`).
   - CAM Browser header has a third `"Tools"` button (`crates/chip-gui/src/panels/browser.rs:262`), plus a `"Manage Tool Library…"` button inside the `Project Tools` card (`browser.rs:570`).
2. **Read-Only `Project Tools` Card & No Unused-Tool Pruning (`browser.rs:552-576`):**
   - `Project Tools (N)` only renders a passive bullet list of `project.tools` with no way to add a tool from the machine library, see which tools are unused, remove a tool, or clean up stale tools left behind after editing operations or switching machines.
3. **Cluttered Operation Tool Picker (`panels/operation_inspector/tool_tab.rs:56-103`):**
   - Renders a flat ungrouped dropdown mixing `project.tools` and `global_tools.tools` with `[Incompat]` prefixes, making it tedious to locate compatible tools for the current strategy and active machine.
4. **Silent Machine Switching Without Tool Migration:**
   - All three machine-switch call sites (`app/modals.rs:79-83`, `app/modals.rs:155-165`, `panels/setup_inspector/mod.rs:351-361`) simply overwrite `project.machine_id` (or call `apply_machine_defaults`) and leave operations pointing at the previous machine's embedded tools with no prompt or guided replacement workflow.

---

## Proposed Changes

### 1. Core Project Tool Usage, Pruning, & Machine-Swap Planning (`chip-project`)

#### [MODIFY] `crates/chip-project/src/document.rs`
#### [MODIFY] `crates/chip-project/src/lib.rs`
- Add pure-data, UI-agnostic tool management & machine-swap helpers on `ProjectDocument`:
  - `pub fn tool_usage_map(&self) -> BTreeMap<ToolId, Vec<(usize, usize, String)>>` — maps each `ToolId` referenced by any operation to its `(setup_idx, op_idx, op_display_name)` list.
  - `pub fn unused_tool_ids(&self) -> Vec<ToolId>` — returns all embedded `project.tools` not referenced by any operation across `self.setups`.
  - `pub fn remove_tool_if_unused(&mut self, id: &ToolId) -> bool` — removes an unreferenced tool from `self.tools` (returns `false` if still referenced or absent).
  - `pub fn prune_unused_tools(&mut self) -> usize` — removes all unreferenced tools from `self.tools` and returns the count removed.
  - **Machine Tool Swap Planner (`MachineToolSwapPlan`, `ToolSwapCandidate`):**
    - `pub fn plan_machine_tool_swap(&self, target_machine: &MachineDefinition) -> MachineToolSwapPlan`
      - Inspects every `ToolId` referenced by operations in `self.setups` (and reports any additional unused embedded tools).
      - If a referenced tool exists in `target_machine.tools` with matching ID and geometry, marks it `ExactMatch` (no prompt needed for that tool).
      - Otherwise creates a `ToolSwapEntry` containing the current embedded `Tool`, the list of referencing operations, and a deterministically ranked list of candidate replacements from `target_machine.tools`:
        1. Same `CutterGeometry` discriminant first (e.g., `FlatEndmill` → `FlatEndmill`, `BallNose` → `BallNose`, `VBit` → `VBit`).
        2. Closest cutting diameter (`|cand_d - old_d|`) ascending.
        3. Closest flute count / corner radius / included angle as tie-breaker.
        4. Pre-selects the top-ranked compatible candidate (or `KeepExisting` when the target machine has no compatible cutter kind at all).
    - `pub fn apply_machine_tool_swap(&mut self, target_machine: &MachineDefinition, sync_setups: bool, replacements: &BTreeMap<ToolId, ToolSwapChoice>, prune_replaced_unused: bool) -> MachineToolSwapSummary`
      - Updates `self.machine_id` (and syncs setup defaults when `sync_setups == true`).
      - For each `ToolSwapChoice::ReplaceWith(new_tool)`: embeds `new_tool` via `ensure_tool_embedded`, rewrites `op.tool_id` on every operation referencing the old `ToolId`, updates the operation's `preset_id` to a matching/default preset on `new_tool` if needed, and calls `op.mark_dirty()`.
      - Optionally calls `prune_unused_tools()` on tools that were replaced (or all unused tools when requested), and keeps `self.tools` deterministically sorted.

---

### 2. Consolidate the Three `"Tools"` Buttons Down to One

#### [MODIFY] `crates/chip-gui/src/panels/browser.rs`
#### [MODIFY] `crates/chip-gui/src/app/menus.rs`
#### [MODIFY] `crates/chip-gui/src/app/toolbar.rs`
- **Keep ONE `"Tools"` button:** Keep the single prominent **`Tools`** button in the top action bar (`toolbar.rs`), opening the Tool Library & Machine Tools Manager.
- **Remove the two duplicates:**
  - Remove the `"Tools"` button from the CAM Browser header (`browser.rs:261-267`).
  - Remove the top-level `"Tools"` menu from the menu bar (`menus.rs:31-55`), relocating **`Machine Manager…`** cleanly under the `Setup` menu (right next to setup/machine configuration) and keeping `Tool Library…` reachable via the single toolbar `"Tools"` button, Command Palette (`Ctrl+K`), and `Project Tools` card.

---

### 3. Actionable `"Project Tools"` Card (`panels/browser.rs`)

#### [MODIFY] `crates/chip-gui/src/panels/browser.rs`
#### [MODIFY] `crates/chip-gui/src/app/pane_behavior.rs`
- Upgrade the `Project Tools (N)` collapsible card (`browser.rs:552-576`) into a real project tool manager:
  - **Header Controls Row:**
    - **`+ Add`** button/popup: opens a compact, searchable picker listing tools from the active machine's library (`global_tools`), grouped by cutter family with diameter & flute count; clicking any tool immediately embeds it into `project.tools` (undoable via `UndoHistory`) and marks already-embedded tools with a subtle `In Project` checkmark.
    - **`Remove Unused (N)`** button (shown/enabled whenever `unused_count > 0`): prunes all unused tools from `project.tools` in a single undoable step and logs the count removed.
  - **Per-Tool Rows:**
    - Show `T#`, tool name, concise geometry/diameter tag (`Ø6.0 Flat`), and a compact usage pill (`2 ops` in subtle neutral tint, or `Unused` in muted amber/gray).
    - Add a compact **`✕` / trash** button on each row:
      - If unused: removes the tool immediately (with Undo snapshot).
      - If in use: disabled with an informative hover tooltip listing the operations using it (so users never accidentally orphan an active operation's tool).
  - Replace the generic `"Manage Tool Library…"` button at the bottom with the inline `+ Add` / `Remove Unused` controls (users still have the single top-action-bar `"Tools"` button to edit tool geometries and cutting presets).

---

### 4. Easier Operation Tool Selection (`panels/operation_inspector/tool_tab.rs`)

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/tool_tab.rs`
- Upgrade the `Tool` selector inside Operation Inspector:
  - Group entries cleanly into:
    1. **Project Tools (Compatible)** — tools already embedded in `project.tools` that match the operation's `StrategyKind`.
    2. **Machine Library (Compatible)** — tools from the active machine not yet in `project.tools` (selecting one automatically embeds it into `project.tools`).
  - Add a compact search/filter field inside the picker popup when the combined list exceeds 6 tools, plus an opt-in **"Show incompatible cutters"** toggle at the bottom (default off) instead of prefixing `[Incompat]` across the main list.
  - Keep a subtle `"Edit Tool…"` link that opens the Tool Library focused directly on the currently selected tool.

---

### 5. Interactive Machine-Switch Tool Replacement Modal (`panels/machine_tool_swap.rs`)

#### [NEW] `crates/chip-gui/src/panels/machine_tool_swap.rs`
#### [MODIFY] `crates/chip-gui/src/panels/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/modals.rs`
#### [MODIFY] `crates/chip-gui/src/app/pane_behavior.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/mod.rs`
- Route **all three machine-switching entry points** (`Setup Inspector` machine combo, `Machine Manager` "Select for Project", and `Tool Library` machine scope selector) through a single `ChipCutterApp::request_machine_switch(new_machine_id, sync_setups)` helper:
  - Calls `project.plan_machine_tool_swap(&target_machine)`.
  - If `plan.needs_user_prompt()` is `false` (no operations reference tools missing on the target machine): applies the machine switch immediately (recording Undo) and reloads `self.global_tools` for the new machine.
  - If `plan.needs_user_prompt()` is `true`: opens `MachineToolSwapModalState` (`"Switch Machine — Select Replacement Tools"`):
    - Displays source machine → target machine summary (travel envelope, spindle RPM range, tool count).
    - Renders a clear table with one row per affected project tool:
      - **Current Tool & Usage:** `T# · Name (ØD · Type)` + list/count of operations using it.
      - **Replacement on `<Target Machine>`:** dropdown listing all compatible tools on the target machine first (with the closest match pre-selected!), followed by other machine tools, plus `"Keep current embedded tool"`.
    - Includes checkboxes:
      - `Remove replaced tools that are no longer used` (default `true`).
      - `Sync setup post-processor & rotary chuck defaults to <Target Machine>` (initialized from `sync_setups`).
    - Buttons: **`Switch Machine & Swap Tools`** (commits machine switch + all tool swaps + optional pruning as a single atomic `UndoHistory` entry, reloads `self.global_tools`, invalidates affected toolpaths) and **`Cancel`** (leaves project untouched).

---

## Verification Plan

1. **Core Tool Management & Machine-Swap Unit Tests (`chip-project`):**
   - Test `tool_usage_map`, `unused_tool_ids`, `remove_tool_if_unused` (blocks removal when referenced, succeeds when unused), and `prune_unused_tools`.
   - Test `plan_machine_tool_swap` ranking: exact ID match skipped, same-geometry closest-diameter tool ranked #1 (e.g., Ø6 mm Flat → Ø6.35 mm Flat over Ø3 mm Ball), incompatible-only target machine defaults cleanly to `KeepExisting`.
   - Test `apply_machine_tool_swap`: rewrites `op.tool_id`, marks affected ops `Dirty`, embeds replacement tools, prunes replaced unused tools when requested, and preserves untouched operations.
   - Verify full Undo/Redo restores original `machine_id`, `project.tools`, and operation `tool_id` assignments cleanly.
2. **UI & Workflow Checks (`chip-gui`):**
   - Confirm only **one** `"Tools"` button exists in the main workspace chrome (top action bar only; removed from CAM Browser header and top menu bar).
   - Confirm `Project Tools` card supports adding from active machine, deleting an unused tool, blocking deletion of an in-use tool, and pruning all unused tools in one click.
   - Confirm switching machines from Setup Inspector, Machine Manager, and Tool Library all trigger `MachineToolSwapModalState` when operations use tools absent from the new machine.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
