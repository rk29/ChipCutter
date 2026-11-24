# Milestone 88 Implementation Plan — Setup Inspector, WCS, Stock, Workholding Wizards, & Machine Manager UI Completeness

## Goal

Close every backend-vs-GUI gap across Setup configuration, WCS picking, Stock rounding, Setup heights & fixture styling, Workholding wizards, and Machine Manager definitions so custom post profiles, multi-body targets, precise WCS edge references, custom stock rounding steps, soft-jaw/flip-jig tuning, and machine kinematics/collet validation are all first-class GUI workflows.

---

## Problem Diagnosis

1. **Custom `.post.toml` Hidden in Setup Inspector (`setup_inspector/mod.rs:532`):** Setup Inspector calls `builtin_post_profiles()` instead of `load_available_post_profiles(...)`, blocking selection of user/project custom post profiles.
2. **Multi-Body `Setup::target_bodies` Unreachable (`setup_inspector/`):** Actively consumed across stock AABB, 3D strategies, soft-jaw wizard, simulation, and viewport rendering, yet has zero UI and is omitted from `SetupParamsSnapshot`.
3. **WCS Picking Gaps (`setup_inspector/wcs_tab.rs`):** `SelectXYPlane::x_edge_ref` is ignored (`..`) and hardcoded to `None`; `SelectZAndX::x_ref` has no clear-to-Auto button; edge origins cannot switch between `EdgeStart` / `EdgeMidpoint` / `EdgeEnd`.
4. **Stock Rounding, Retract Reference, & Fixture Color (`stock_tab.rs`, `heights_tab.rs`, `fixtures_tab.rs`):** `RelativeBox::round_increment_mm` is hardcoded to a `5.0 mm` checkbox (overwriting custom TOML values); `SetupHeights::retract` omits `HeightReference::SetupClearance`; `FixtureBody::color_rgba` has no override picker/reset.
5. **Workholding Wizards (`panels/workholding_wizards.rs`):** `SoftJawBoltPattern::Custom::cbore_depth_mm` is stored in state with no widget; `SoftJawConfig::z_band_bottom_offset_mm`, `FlipJigConfig::pin_protrusion_z_mm`, and optional tool/WCS slot overrides are absent.
6. **Machine Manager (`panels/machine_manager.rs`):** Actively validated/used fields `spindle.supported_shank_diameters_mm`, `kinematics.junction_deviation_mm`, `spindle.power_watts`, and `description` are not shown or editable.

---

## Proposed Changes

### 1. Setup Post Profiles, Multi-Body `target_bodies`, & WCS Picking Polish

#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/wcs_tab.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
- Replace `builtin_post_profiles()` with `load_available_post_profiles(project_dir)` in `setup_inspector/mod.rs` (matching `machine_manager.rs` and `gcode_preview.rs`), clearly badging custom `.post.toml` profiles.
- Add **`Target Bodies`** controls in `setup_inspector/` (shown whenever `doc.bodies.len() > 1` or `!setup.target_bodies.is_empty()`): toggle between `"All Part Bodies"` (empty `Vec`) and explicit per-body checkboxes, and include `target_bodies` in `SetupParamsSnapshot` for atomic Undo + cache invalidation.
- In `setup_inspector/wcs_tab.rs`:
  - Under `WcsOrientationMode::SelectXYPlane`, expose `"Pick X Edge"` / readout / `"Clear (Auto)"` for `x_edge_ref: Option<SelectionBinding>` and wire the pick target in `viewport_controller.rs`.
  - Under `WcsOrientationMode::SelectZAndX`, add a `"Clear (Auto)"` button beside `x_ref` when set.
  - Under `WcsOriginMode::SelectedCadFeature`, whenever the active feature is `EdgeStart` / `EdgeMidpoint` / `EdgeEnd`, render a 3-way segmented selector (**Start** | **Midpoint** | **End**) preserving the bound edge `SelectionBinding`.

---

### 2. Stock Rounding Increment, Retract `SetupClearance`, & Fixture Color Override

#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/stock_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/heights_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/fixtures_tab.rs`
- In `stock_tab.rs`: replace the hardcoded `5.0 mm` checkbox for `RelativeBox::round_increment_mm` with an enable checkbox + unit-aware numeric increment input (preserving custom values like `6.35 mm`, `10.0 mm`, `12.7 mm`, defaulting to `5.0 mm` only when newly enabled).
- In `heights_tab.rs`: allow `HeightReference::SetupClearance` in the `retract.reference` `ComboBox` (supported natively by `SetupHeights::resolve`).
- In `fixtures_tab.rs`: add a compact color swatch picker (`egui::color_edit_button_rgba_unmultiplied`) + `"Reset"` button for `FixtureBody::color_rgba`.

---

### 3. Workholding Wizards & Machine Manager Completeness

#### [MODIFY] `crates/chip-gui/src/panels/workholding_wizards.rs`
#### [MODIFY] `crates/chip-gui/src/panels/machine_manager.rs`
- In `workholding_wizards.rs`:
  - Render the missing `DragValue` widget for `custom_cbore_depth_mm` (`SoftJawBoltPattern::Custom::cbore_depth_mm`).
  - Expose `SoftJawConfig::z_band_bottom_offset_mm` (`Grip Band Bottom Offset`) and `FlipJigConfig::pin_protrusion_z_mm` (`Pin Protrusion Above Jig`).
  - Add a collapsible **`Advanced Tool & WCS Overrides`** section in both wizards for optional `pin_tool_id` / `roughing_tool_id` / `finishing_tool_id` and `wcs_slot` / `setup2_wcs_slot` / `spoilboard_wcs_slot` overrides (defaulting to `Auto`).
- In `machine_manager.rs`:
  - Expose `description` (multi-line/single-line text), `spindle.power_watts` (`Spindle Power (W)`), `spindle.supported_shank_diameters_mm` (editable chip list with `+ Add Shank Ø` + presets like `3.175`, `6.0`, `6.35`, `8.0 mm`), and `kinematics.junction_deviation_mm` (`Junction Deviation`).

---

## Verification Plan

1. **Headless `egui` Regression Tests (`chip-gui`):**
   - Test `setup_inspector` custom `.post.toml` selection, multi-body `target_bodies` toggling + Undo snapshot, `SelectXYPlane::x_edge_ref` + `SelectZAndX::x_ref` clear, `EdgeStart`/`EdgeMidpoint`/`EdgeEnd` switching, custom `round_increment_mm` preservation, `SetupClearance` retract option, and `FixtureBody::color_rgba` edit/reset.
   - Test `workholding_wizards` editing `custom_cbore_depth_mm`, `z_band_bottom_offset_mm`, `pin_protrusion_z_mm`, and tool/WCS overrides.
   - Test `machine_manager` editing `supported_shank_diameters_mm`, `junction_deviation_mm`, `power_watts`, and `description` with `.machine.toml` roundtrip.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
