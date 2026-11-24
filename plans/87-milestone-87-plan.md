# Milestone 87 Implementation Plan — Feeds, Speeds, Tool Library CRUD, & Operation Strategy / Linking UI Completeness

## Goal

Expose every actively used backend feedrate override, cutting preset field, tool metadata/CRUD action, independent lead-out & pre-drilled entry option, and missing strategy parameter in `chip-gui` so machinists never have to edit `.chip.toml` by hand to tune plunge/ramp/lead feeds, G82/G73 drill parameters, independent lead-outs, or 2D/3D Adaptive and V-Carve clearing settings.

---

## Problem Diagnosis

A full backend-vs-GUI audit revealed high-impact active fields in `chip-cam` and `chip-tools` that have no UI controls anywhere in `chip-gui`:
1. **`OperationFeedOverrides` & `CuttingPreset` (`operation_inspector/tool_tab.rs`, `tool_library/presets_editor.rs`):** Plunge, ramp, and lead-in/out feedrates (`plunge_feed_mm_min`, `ramp_feed_mm_min`, `lead_feed_mm_min`), per-operation `stepdown_mm` override, and preset `material` / `coolant_override` cannot be viewed or edited anywhere in the GUI.
2. **`Tool` Metadata & CRUD (`tool_library/mod.rs`, `presets_editor.rs`):** `spindle_direction` (CW/CCW `M3`/`M4`), `default_coolant`, `vendor`, and `product_id` are missing, as are **Delete Tool**, **Duplicate Tool**, and **Delete Preset** buttons.
3. **Operation Linking & Entry (`operation_inspector/linking_tab.rs`):** Unchecking `"Mirror Lead-In"` (`lead_out_same_as_in == false`) renders zero controls for `op.linking.lead_out` (and `"Mirror Lead-In"` is gated behind `lead_in.enabled`, preventing lead-out-only setups). `EntryStrategy::PreDrilled { point_wcs }` is implemented in `Pocket2D` but absent from the Entry dropdown.
4. **Strategy Parameters (`operation_inspector/passes_tab/`):**
   - `DrillBore` omits 6 actively used fields (`dwell_seconds`, `chip_break_retract`, `decreasing_peck_factor`, `min_peck_depth`, `bore_pitch_per_rev`, `bore_finish_stepover`).
   - `Contour2D` omits `finish_feed_mm_min`.
   - `Adaptive2D` & `Adaptive3D` omit `min_trochoid_radius`; `Adaptive3D` also omits `no_engagement_feed_multiplier`, `max_thinning_feed_multiplier`, and `flat_area_min_mm2`.
   - `VCarve` omits `clearing_preset_id` for its Stage 1 flat-clearing tool.

---

## Proposed Changes

### 1. Feeds, Speeds, Cutting Presets, & Tool Library CRUD

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/tool_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/tool_library/presets_editor.rs`
#### [MODIFY] `crates/chip-gui/src/panels/tool_library/mod.rs`
- In `operation_inspector/tool_tab.rs`:
  - Expose optional override inputs (with inherited preset hint & quick reset-to-preset button) for `op.feed_overrides.lead_feed_mm_min`, `ramp_feed_mm_min`, `plunge_feed_mm_min`, and `stepdown_mm` alongside RPM, Cutting Feed, and Coolant.
- In `tool_library/presets_editor.rs`:
  - Expose `material`, `coolant_override`, `lead_feed_mm_min`, `ramp_feed_mm_min`, and `plunge_feed_mm_min` (with unit-aware mm/min or in/min formatting matching `cutting_feed_mm_min`).
  - Add a **Delete Preset** action (guarded when only 1 preset exists, or cleanly clearing selection).
- In `tool_library/mod.rs`:
  - Expose `spindle_direction` (`CW (M3)` / `CCW (M4)`), `default_coolant`, `vendor`, and `product_id` in the tool editor.
  - Add **Duplicate Tool** and **Delete Tool** buttons (disabled on read-only built-in machines, with usage warning if a tool is referenced by active operations).

---

### 2. Independent Lead-Out & Pre-Drilled Entry Controls

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/linking_tab.rs`
- Decouple the `"Lead-Out"` section / `"Mirror Lead-In"` checkbox from `if op.linking.lead_in.enabled` so users can enable independent lead-out even when lead-in is disabled.
- Whenever `!op.linking.lead_out_same_as_in`, render full controls for `op.linking.lead_out` (`enabled`, `horizontal_radius`, `horizontal_sweep_deg`, `linear_length`, `vertical_radius`).
- Add **`Pre-Drilled Point`** (`EntryStrategy::PreDrilled { point_wcs }`) to the Entry Strategy dropdown with X/Y WCS coordinate inputs (unit-aware) and clear hover tooltip.

---

### 3. Missing Strategy Parameters (`DrillBore`, `Contour2D`, `Adaptive2D`/`3D`, `VCarve`)

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/milling_2d.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/vcarve.rs`
- **`DrillBore` (`passes_tab/milling_2d.rs`):**
  - Show `dwell_seconds` (`Dwell (s)`) for `Spot` (G82) / `DeepPeck` / `ChipBreak` cycles.
  - Show `chip_break_retract` (`Chip-Break Retract`) when `ChipBreak` (G73) is selected.
  - Show optional `decreasing_peck_factor` and `min_peck_depth` when `DeepPeck` or `ChipBreak` is selected.
  - Show optional `bore_pitch_per_rev` (override for `bore_ramp_angle_deg`) and `bore_finish_stepover` (when `bore_finish_pass` is checked) under helical bore cycles.
- **`Contour2D` (`passes_tab/milling_2d.rs`):**
  - Expose optional `finish_feed_mm_min` (`Finish Pass Feed`) whenever `finish_passes > 0`.
- **`Adaptive2D` & `Adaptive3D` (`passes_tab/milling_2d.rs`, `passes_tab/surface_3d.rs`):**
  - Expose `adaptive.min_trochoid_radius` in both 2D and 3D Adaptive.
  - Expose `adaptive.no_engagement_feed_multiplier`, `adaptive.max_thinning_feed_multiplier`, and `flat_area_min_mm2` (when `detect_flats == true`) in `Adaptive3D`.
- **`VCarve` (`passes_tab/vcarve.rs`):**
  - Add a **Clearing Preset** `ComboBox` (`clearing_preset_id`) populated from the selected Stage 1 flat-clearing tool's `presets`, falling back cleanly to the first preset if `None`.

---

## Verification Plan

1. **Headless `egui` Regression Tests (`chip-gui`):**
   - Test `tool_tab.rs` & `presets_editor.rs` editing plunge/ramp/lead feeds, stepdown override, preset material/coolant, tool spindle direction/vendor/SKU, and Duplicate/Delete Tool + Delete Preset.
   - Test `linking_tab.rs` unchecking `"Mirror Lead-In"` renders & mutates `op.linking.lead_out` (both with `lead_in.enabled == true` and `false`), and selecting `EntryStrategy::PreDrilled` mutates `point_wcs`.
   - Test `passes_tab` rendering & mutating all 6 `DrillBore` fields, `Contour2D::finish_feed_mm_min`, `Adaptive2D`/`Adaptive3D` trochoid/feed/flat fields, and `VCarve::clearing_preset_id`, verifying atomic Undo labels on every mutation.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
