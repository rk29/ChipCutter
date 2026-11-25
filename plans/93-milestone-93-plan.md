# Milestone 93 Implementation Plan — Project-Level Workpiece Material Library, 6063-T5 Preset, & Preset Auto-Matching

## Goal

Introduce a first-class, **project-level workpiece material** (`ProjectDocument::workpiece_material`) backed by a curated built-in material physics catalog in `chip-tools`—including **Aluminum 6063-T5 Extrusion** alongside 6061-T6, 7075-T6, brass, engineering plastics, wood, composites, and mild steel—so the machinist sets the stock material **once in `Project Properties` / `Project Inspector`** (never repeating material selection per operation), and every operation automatically inherits the material's cutting physics and default tool preset matching.

---

## Problem Diagnosis

1. **No Project-Level Workpiece Material (`crates/chip-project/src/document.rs`, `crates/chip-gui/src/panels/project_properties.rs`):**
   - `ProjectDocument` stores `display_units` and `machine_id`, but has no `workpiece_material` field. Today, material strings live only as free-form labels inside individual `CuttingPreset::material` entries, forcing users to hunt for matching presets on every operation and giving the CAM kernel no authoritative source of specific cutting energy ($u_s$ / $K_c$), chip-welding/rubbing bounds, or interrupted-cut sensitivity.
2. **Missing Built-In Physics Constants & Aluminum 6063-T5 Extrusion Support (`crates/chip-tools/src/`):**
   - Built-in machine tool libraries (`carvera`, `milo`, `voron_cascade`) carry `"Aluminum 6061"` presets, but lack a dedicated **Aluminum 6063-T5 Extrusion** profile (softer and gummier than 6061, highly prone to dry flute welding when $h_{\max}$ is too thin and tool snapping on high-`L/D` interrupted cuts).
   - `chip-tools` has no structured `WorkpieceMaterialProfile` carrying $K_c$ (specific cutting force), $u_s$ (specific cutting energy), $h_{\min}$ (rubbing/welding chip-thickness floor), radial-to-tangential force ratio $k_r$, recommended SFM / chipload bands, and coolant guidance.

---

## Proposed Changes

### 1. Curated Workpiece Material Physics Catalog (`chip-tools`)

#### [NEW] `crates/chip-tools/src/materials.rs`
#### [MODIFY] `crates/chip-tools/src/lib.rs`
#### [MODIFY] `crates/chip-tools/src/builtin_machines/carvera.rs`
- Define typed, deterministic, `serde`-serializable material types in `chip-tools::materials`:
  - `MaterialCategory`: `Aluminum`, `NonFerrousMetal`, `Steel`, `Plastic`, `Wood`, `Composite`, `Custom`.
  - `WorkpieceMaterialProfile`:
    - `id: String` (canonical slug, e.g., `"aluminum-6061-t6"`, `"aluminum-6063-t5"`), `name: String`, `category: MaterialCategory`, `notes: String`.
    - `specific_cutting_energy_w_s_mm3: f64` ($u_s$ in $\text{J/mm}^3$, e.g., `0.75` for 6061-T6, `0.85` for gummy 6063-T5, `0.90` for 7075-T6, `0.95` for Brass C360, `0.25` for Delrin/POM, `0.30` for Acrylic, `0.15` for Hardwood, `0.12` for MDF, `0.55` for G10/CFRP, `2.60` for Mild Steel 1018).
    - `specific_cutting_force_n_mm2: f64` ($K_c$ in $\text{N/mm}^2$ for instantaneous radial/tangential force & bending stress calculations).
    - `radial_force_ratio: f64` ($k_r = F_r / F_t$, typically `0.35–0.55` depending on material gumminess/ductility).
    - `min_chip_thickness_mm: f64` ($h_{\min}$ rubbing/chip-welding lower threshold — e.g., `0.010 mm` for 6063-T5 dry/air-blast vs. `0.008 mm` for 6061-T6).
    - `max_recommended_chipload_factor: f64` (multiplier against tool diameter for sanity bounds, e.g., `0.012 * D`).
    - `recommended_sfm_range_m_min: (f64, f64)` (carbide surface-speed envelope).
    - `gummy_chip_weld_risk: bool` (`true` for 6063-T5, 6061-T6, Acrylic, HDPE — triggers dry-cut chip-welding & thin-chip rubbing checks in M94).
    - `preferred_coolant: CoolantMode` (`AirBlast` / `Mist` for 6063-T5 & 6061-T6, `Off` for wood/MDF).
    - `preset_match_keywords: &'static [&'static str]` (deterministic fuzzy matching tokens against existing `CuttingPreset::material` strings, e.g., `["6063", "aluminum", "alu"]`).
  - `ProjectMaterialSelection`:
    - `material_id: String` (references a built-in profile ID or `"custom"`).
    - Optional custom overrides (`custom_name`, `custom_specific_cutting_energy`, `custom_specific_cutting_force`, `custom_min_chip_thickness_mm`) omitted in TOML when `None`.
- Ship **11 built-in read-only material profiles** (`builtin_workpiece_materials()`):
  1. `aluminum-6061-t6` — **Aluminum 6061-T6** (default fallback when unspecified for metal presets)
  2. `aluminum-6063-t5` — **Aluminum 6063-T5 Extrusion** (gummy/chip-welding-safe 1FL O-flute parameters, air-blast/mist defaults, lighter interrupted-cut engagement)
  3. `aluminum-7075-t6` — **Aluminum 7075-T6**
  4. `brass-c360` — **Brass C360 (Free-Machining)**
  5. `acetal-delrin` — **Acetal / Delrin (POM)**
  6. `acrylic-pmma` — **Acrylic (PMMA)**
  7. `hdpe-plastic` — **HDPE / Engineering Plastic**
  8. `hardwood-plywood` — **Hardwood / Baltic Birch Plywood**
  9. `mdf` — **MDF / Softwood**
  10. `g10-carbon-fiber` — **G10 / FR4 / Carbon Fiber**
  11. `mild-steel-1018` — **Mild Steel 1018**
- Add helper `match_best_preset_for_material<'a>(tool: &'a Tool, material: &WorkpieceMaterialProfile) -> Option<&'a CuttingPreset>` scoring exact ID/name match > keyword match > category match > first preset.
- Add dedicated **Aluminum 6063-T5** cutting presets to Makera / Carvera single-flute O-flute cutters in `builtin_machines/carvera.rs` (including the `Ø3.175 × 25 mm LOC` 1FL O-flute with safe reduced stepdown/stepover & air-blast defaults) without altering existing 6061 preset IDs or golden G-code outputs.

---

### 2. Project-Level Storage & Automatic Preset Matching (`chip-project`, `chip-gui`)

#### [MODIFY] `crates/chip-project/src/document.rs`
#### [MODIFY] `crates/chip-gui/src/panels/project_properties.rs`
#### [MODIFY] `crates/chip-gui/src/panels/browser/actions.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/tool_tab.rs`
#### [MODIFY] `crates/chip-gui/src/app/mod.rs`
- Add `#[serde(default, skip_serializing_if = "Option::is_none")] pub workpiece_material: Option<ProjectMaterialSelection>` to `ProjectDocument`:
  - Preserves 100% lossless loading of existing `schema_version = 1 / 2 / 3` `.chip.toml` and `.chipz` files (omitted when unset, defaulting to an inferred or `aluminum-6061-t6` effective material via `project.effective_workpiece_material()`).
  - Add helper `ProjectDocument::effective_workpiece_material(&self) -> WorkpieceMaterialProfile`.
- Expose **`Workpiece Material`** selector (grouped by `MaterialCategory`, with concise physics readout `Kc / us / h_min / coolant` and optional `Custom` physics inputs) in **both** `Edit → Project Properties...` and the right-hand **Project Inspector** (`show_project_properties_cards`), recording an atomic Undo snapshot and offering a 1-click **"Apply Matching Presets to Operations"** prompt/button (never silently clobbering user-tuned feed overrides!).
- Update operation creation (`create_default_operation_in_setup`, `create_adaptive_2d_operation_in_setup`) and tool switching to select `match_best_preset_for_material(tool, &project.effective_workpiece_material())` automatically.
- In Operation Inspector `[Tool]`, display a compact read-only **`Project Material: <name>`** chip (linking to `Project Properties` on click) instead of asking the user to pick a workpiece material per operation.

---

## Verification Plan

1. **Material Catalog & Persistence Tests (`chip-tools`, `chip-project`):**
   - Unit tests verifying all 11 built-in materials have finite positive $u_s$, $K_c$, $k_r$, $h_{\min}$, valid SFM envelopes, and deterministic ordering; verify `aluminum-6063-t5` carries `gummy_chip_weld_risk = true` and `CoolantMode::AirBlast`.
   - Unit tests verifying `match_best_preset_for_material` prefers `Aluminum 6063-T5` over `Aluminum 6061` when `aluminum-6063-t5` is selected on a 1FL O-flute and falls back cleanly when no exact material preset exists.
   - TOML roundtrip tests confirming `workpiece_material = None` omits the field cleanly (zero diff on legacy projects) and `Some(ProjectMaterialSelection)` (both built-in and `custom`) roundtrips losslessly.
2. **GUI & CLI Tests (`chip-gui`, `chip-cli`):**
   - Headless `egui` test verifying `Project Properties` / `Project Inspector` edits `project.workpiece_material` with atomic Undo and Operation Inspector `[Tool]` displays the inherited project material chip without per-operation material selectors.
   - Confirm zero regressions on all 21 golden G-code files (`chip-cli`).
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
