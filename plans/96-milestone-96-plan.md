# Milestone 96 Implementation Plan — Machinability Advisor UI, 1-Click Safe Fixes, Diagnostics Integration, Visual Verification, & Phase 26 Spec / Docs Sync

## Goal

Surface the Milestone 93–95 physics engine, project material, helical bore safety options, and measurement/reach overlays seamlessly across `chip-gui`—with a live collapsible **`Machinability & Safety`** card in the Operation Inspector offering **1-click `"Apply Safe Fix"`** buttons (atomic Undo), a Setup-wide Machinability Summary inside **`Diagnostics & Log`**, subtle warning indicators in the Project Tree, headless Vulkan visual verification in Studio Dark & Daylight Light, and full specification + tutorial synchronization for Phase 26.

---

## Proposed Changes

### 1. Operation Inspector Machinability Card & Helical Bore Safety Controls (`chip-gui`)

#### [NEW] `crates/chip-gui/src/panels/operation_inspector/machinability_card.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/tool_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab.rs`
- Render an aligned two-column collapsible **`Machinability & Safety`** `section_card` at the bottom of Operation Inspector **`[Tool]`** (and compact status banner at top of **`[Passes]`** whenever a `Warning` or `Critical` finding is active, auto-expanded on `Warning`/`Critical` and collapsed on `Ok`):
  - **Live Telemetry Strip:** `MRR (cm³/min)`, `Spindle Load (W / %)`, `Peak Radial Force (N)`, `Tip Deflection (µm)`, `Bending Safety Factor`, `Effective h_max (mm)`, and `Stickout L/D`.
  - **Actionable Findings List:** Color-coded badges (`Info` cyan, `Warning` amber, `Critical` crimson — using `ThemeTokens`, zero ad-hoc RGBs and zero exotic Unicode glyphs!) with plain-English shop-floor explanations and **1-click `"Apply Safe Fix"`** buttons executing `apply_machinability_fix` inside a single atomic Undo snapshot (`history.record_before_edit(...)`) and marking the operation `Dirty`.
  - **Quick Overlay Toggle:** 1-click **`Show Reach / Min-R Overlay`** toggle directly inside the card to inspect tight corners and flute/stickout reach on the 3D model.
- Expose the two new Milestone 94 helical bore safety controls in Operation Inspector **`[Passes]`** (for `DrillBore` in `HelicalBore` mode):
  - **`Inner-Arc Feed Compensation`** checkbox (`bore_inner_arc_feed_comp`) with live `R_hole / r_orbit` amplification readout when a hole is selected.
  - **`Chip-Clearing Retract Depth`** optional `ParamLength` input (`bore_chip_clear_depth`) for deep bores.

---

### 2. Setup-Wide Diagnostics Tab & Project Tree Warning Integration (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/panels/diagnostics.rs`
#### [MODIFY] `crates/chip-gui/src/panels/browser/mod.rs`
- Add a **`Machinability & Tool Safety`** section inside the bottom **`Diagnostics & Log`** tab summarizing every active operation in the active Setup (Operation, Tool, `L/D`, Spindle `%`, Deflection `µm`, Effective $h_{\max}$, Status) where clicking any row focuses that Operation in the right Inspector and 1-click fixes are directly accessible.
- In the Project Tree's `Operations (N)` items, augment the hover tooltip with concise physics warning summaries when an operation has a `Critical` or `Warning` machinability finding (keeping normal tree rows uncluttered).

---

### 3. Headless Vulkan Visual Verification, Tutorial / Docs Update, & Spec Sync

#### [MODIFY] `specs/04-tool-library.md`
#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/09-persistence-cli.md`
#### [MODIFY] `specs/10-roadmap.md`
#### [NEW] `docs/tutorials/23-physics-machinability-measurement.md` (and link from `docs/tutorials/README.md`)
#### [MODIFY] `scripts/capture_tutorial_screenshots.sh`
- Capture headless Vulkan verification screenshots in both **Studio Dark** and **Daylight Light** covering:
  1. `Project Inspector` showing the project-level **`Workpiece Material`** selector (`Aluminum 6063-T5 Extrusion`) and physics summary.
  2. Operation Inspector **`Machinability & Safety`** card flagging a high-`L/D` 1FL O-flute tool-snap / spindle-load hazard with its 1-click **`Apply Safe Fix`** button, plus the clean `Ok` state after applying the fix.
  3. 3D viewport **`Measure`** mode showing an active exact-`f64` hole-to-hole / depth measurement leader line + floating readout card.
  4. 3D viewport **`Reach / Min-R`** heatmap overlay highlighting tight concave corners (amber) and deep flute/stickout zones (amber/crimson).
- Publish illustrated Tutorial 23 (`23-physics-machinability-measurement.md`) with deterministic capture targets in `scripts/capture_tutorial_screenshots.sh` and link it in `docs/tutorials/README.md` (verified via `docs_samples_parity`).
- Synchronize all impacted living specifications (`04-tool-library.md`, `05-operations.md`, `08-ui-viewport.md`, `09-persistence-cli.md`, `10-roadmap.md`) and mark **Phase 26 (Completed — M93–M96)**.

---

## Verification Plan

1. **Headless `egui` & E2E Workflow Tests (`chip-gui`):**
   - Test rendering `machinability_card.rs` on both hazardous (`Critical`) and safe (`Ok`) operations in `Studio Dark` and `Daylight Light`; verify clicking **`Apply Safe Fix`** mutates the operation cleanly, records an atomic Undo snapshot, marks the operation `Dirty`, and resolves the `Critical` finding on the next frame.
   - Test `Diagnostics & Log` Machinability table rendering and click-to-focus routing.
   - Verify `cargo test --locked -p chip-gui --test docs_samples_parity` passes (`3/3`) with the new tutorial and screenshots.
2. **Headless Visual Inspection:**
   - Visually inspect captured Studio Dark & Daylight Light PNGs via `read_file` to verify crisp typography, zero tofu glyphs, zero layout overlap/truncation, and high contrast on both measurement leader lines and reach/min-radius heatmaps.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
