# Milestone 60 Implementation Plan — 3D Fixture, Vise, & Clamp Workholding Model, Persistence, & Viewport Rendering

## Goal

Introduce first-class, UI-agnostic 3D workholding fixtures on every ChipCutter `Setup`—supporting both parametric shop-floor hardware (**Vise Jaws + Parallels**, **Toe / Strap Clamps**, **Spoilboard / Tooling Plate**, and **Box / Cylinder Keep-Outs**) and **Imported 3D Fixture Solids** (`.step`/`.stl`/`.obj`/`.3mf`)—with exact `f64` `chip-bvh` collision geometry, schema-safe `.chip.toml` / `.chipz` persistence, a strict two-column **`[Fixtures]`** Setup Inspector tab with interactive 3D click-to-place, translucent `wgpu` viewport rendering, and CLI inspection.

---

## Non-Negotiable Invariants

- **100% Schema & Golden Backward Compatibility:** `Setup.fixtures` defaults to empty and uses `#[serde(default, skip_serializing_if = "Vec::is_empty")]`, so existing `schema_version = 1/2/3` `.chip.toml` files load cleanly and serialize with zero diff when no fixtures are configured.
- **UI-Agnostic Core:** All fixture types, parametric mesh builders, `FlatBvh` collision indices, and 2D keep-out footprint projections live in `chip-cam` (using `chip-math`, `chip-bvh`, `chip-geom2d`, and `chip-cad`), with zero `egui`/`wgpu` dependency.
- **No Target-Part Pollution:** Imported fixture models (`.step`/`.stl`/`.obj`/`.3mf`) are cleanly separated from machined target solids so `StockConfig::RelativeBox`, WCS model anchors, and `StockErrorReport` target gouge/excess metrics never mistake a vise or clamp for the machined part.
- **Deterministic Persistence & `< 1 ms` Undo/Redo:** Fixtures are stored as compact parametric/transform records (never serialized triangle blobs) so `git diff` stays crisp and Undo/Redo snapshots remain sub-millisecond.
- **Zero New External Dependencies:** Reuses `chip-bvh`, `chip-geom2d`, `chip-cad`, and existing `wgpu` pipelines.

---

## Proposed Changes

### 1. Core Workholding Data Model & Exact `f64` Resolution (`chip-cam`)

#### [NEW] `crates/chip-cam/src/workholding.rs`
#### [MODIFY] `crates/chip-cam/src/setup.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- **Data Types on `Setup`:**
  - Add `#[serde(default, skip_serializing_if = "Vec::is_empty")] pub fixtures: Vec<FixtureBody>` and `#[serde(default = "default_true", skip_serializing_if = "is_true")] pub auto_avoid_fixtures_on_rapids: bool` on `Setup`.
  - Define `FixtureBody`:
    - `id: String`, `name: String`, `enabled: bool` (default `true`)
    - `clearance_margin: ParamLength` (default `1.0 mm` safety halo for rapid avoidance and collision checks)
    - `allowed_overcut: ParamLength` (default `0.0 mm`; used on `Spoilboard` e.g. `0.25–0.5 mm` sacrificial through-cut allowance)
    - `kind: FixtureKind`
    - `color_rgba: Option<[f32; 4]>` (optional UI color override; deterministic defaults per kind: steel gray for vise jaws, anodized red for toe clamps, warm MDF tan for spoilboard, amber for keep-outs, slate blue for imported fixtures)
  - Define `FixtureKind` (`#[serde(tag = "type", rename_all = "snake_case")]`):
    1. **`ViseJaws`**:
       - `clamp_axis: ViseClampAxis` (`Y` = front/back jaws default, `X` = left/right jaws)
       - `jaw_width: ParamLength` (default `100 mm` / `4 in`), `jaw_thickness: ParamLength` (default `18 mm`), `jaw_height: ParamLength` (default `38 mm`)
       - `grip_depth: ParamLength` (how far below stock top the stock sits inside the jaws, or how much bottom stock is gripped — default `5.0 mm` from stock bottom upward!)
       - `step_shelf_depth: ParamLength` & `step_shelf_height: ParamLength` (optional stepped soft-jaw / Machinist step-jaw shelf; `0` = flat jaws)
       - `include_parallels: bool` + `parallel_thickness: ParamLength` (optional pair of parallels sitting beneath stock bottom inside the vise)
       - `center_offset_along_jaw: ParamLength` (slide stock left/right along jaw face)
       - `fixed_jaw_side: ViseFixedSide` (`Positive` = rear `+Y` fixed jaw, `Negative` = front `-Y` fixed jaw)
    2. **`ToeClamp`**:
       - `anchor_xy: [ParamLength; 2]` (clamp tip contact center in Setup WCS XY)
       - `z_reference: FixtureZRef` (`StockTop`, `StockBottom`, `WcsOrigin`) + `z_offset: ParamLength`
       - `yaw_deg: f64` (approach direction in Setup XY so clamp body extends outward away from the part)
       - `strap_length: ParamLength` (default `45 mm`), `strap_width: ParamLength` (default `16 mm`), `strap_height: ParamLength` (default `10 mm`)
       - `nose_overhang: ParamLength` (how far nose overlaps onto stock edge, default `6 mm`)
       - `stud_diameter: ParamLength` (default `8 mm`), `stud_height_above_strap: ParamLength` (default `12 mm` — catches low rapids that would clear the strap but hit the T-nut/bolt!)
    3. **`Spoilboard`**:
       - `z_top_ref: FixtureZRef` (default `StockBottom`) + `z_offset: ParamLength` (default `0.0 mm`)
       - `thickness: ParamLength` (default `18 mm`)
       - `margin_xy: ParamLength` (overhang past stock XY bounds, default `25 mm`)
    4. **`BoxKeepOut`**:
       - `center_wcs: [ParamLength; 3]`, `size_wcs: [ParamLength; 3]`, `yaw_deg: f64`
    5. **`CylinderKeepOut`**:
       - `center_xy_wcs: [ParamLength; 2]`, `z_bottom: ParamLength`, `z_top: ParamLength`, `diameter: ParamLength`
    6. **`ImportedModel`**:
       - `model_id: String` (references a `ModelEntry` in `ProjectDocument.models` marked `role: ModelRole::Fixture`, or a dedicated `ProjectDocument.fixture_models` entry so it never pollutes `target_bodies` or `RelativeBox`!)
       - `offset_wcs: [ParamLength; 3]`, `rotate_deg: [f64; 3]`
- **Exact Resolution (`ResolvedFixtureSolid` & `resolve_setup_fixtures`):**
  - Evaluates all `ParamLength` expressions against `ResolvedSetup::eval_context(display_unit)`.
  - Generates watertight exact `f64` `TriMesh3D` + `FlatBvh` (`chip_bvh`) in Setup WCS for every enabled fixture (for `ViseJaws`, builds fixed jaw + movable jaw + optional step shelves + parallels; for `ToeClamp`, builds strap body + riser block + stud/nut cylinder; for `ImportedModel`, extracts transformed STEP/STL/OBJ/3MF triangles into Setup WCS).
  - Computes `aabb_wcs: Aabb3`, `footprint_2d: RegionSet2D` (exact XY projection for 2D preview/avoidance), `clearance_margin_mm: f64`, and `allowed_overcut_mm: f64`.
  - Validates finite positive dimensions and non-empty meshes, returning clean `CamError` diagnostics on invalid configurations.

---

### 2. Project Persistence, Fixture Models, & `.chipz` Bundling (`chip-project`, `chip-cli`)

#### [MODIFY] `crates/chip-project/src/document.rs`
#### [MODIFY] `crates/chip-project/src/bundle.rs`
#### [MODIFY] `crates/chip-project/src/validation.rs`
#### [MODIFY] `crates/chip-project/src/cache.rs`
#### [MODIFY] `crates/chip-cli/src/main.rs`
#### [NEW] `crates/chip-cli/src/commands/workholding.rs`
- **Dedicated Fixture Model Role (`ModelRole`):**
  - Add `#[serde(default, skip_serializing_if = "ModelRole::is_part")] pub role: ModelRole` (`Part` default | `Fixture`) on `ModelEntry` in `ProjectDocument`.
  - Ensure `CadDocument` loading and `target_bodies` auto-resolution filter to `ModelRole::Part` for machining/stock sizing/target gouge metrology, while `ModelRole::Fixture` bodies feed `FixtureKind::ImportedModel` and are packed inside `.chipz` (`models/*`) with the exact same SHA-256 & zip-slip hardening.
- **Cache Invalidation & Validation:**
  - Include enabled fixtures + `auto_avoid_fixtures_on_rapids` in `compute_operation_input_hash` so editing or moving a clamp automatically marks affected operations `Dirty` when rapid avoidance is active.
  - Add Setup validation checks in `validation.rs` (missing `ImportedModel` ID, negative clearance margin, vise grip depth exceeding stock height, invalid dimensions).
- **Headless CLI Helpers (`chipcutter workholding`):**
  - Add `chipcutter workholding list <project.chip.toml>` (prints Setup fixtures, bounding boxes, margins, and triangle counts) and `chipcutter workholding add-vise` / `add-clamp` / `add-spoilboard` for reproducible scripting & testing.

---

### 3. Translucent `wgpu` Viewport Rendering & `[Fixtures]` Setup Inspector Tab (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/panels/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/mod.rs`
#### [NEW] `crates/chip-gui/src/panels/setup_inspector/fixtures_tab.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/highlight.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/scene_builders.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/pipeline.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
#### [MODIFY] `crates/chip-gui/src/app/mod.rs`
#### [MODIFY] `crates/chip-gui/src/capture.rs`
- **Setup Inspector `[Fixtures]` Tab:**
  - Add `SetupTab::Fixtures` alongside `WCS & Post`, `Stock`, `4th Axis`, and `Heights`.
  - Include `fixtures` & `auto_avoid_fixtures_on_rapids` in `SetupParamsSnapshot` for `< 1 ms` Undo/Redo with zero idle clones.
  - Strict two-column inspector layout with universal hover tooltips:
    - Quick-add action bar: `+ Vise`, `+ Toe Clamp`, `+ Spoilboard`, `+ Keep-Out`, `+ Import Fixture…`.
    - Compact fixture list with Enable checkbox, Kind badge, Delete/Duplicate buttons, and selected-fixture collapsible property cards (all `ParamLength` expression inputs supporting `stock_x`, `stock_y`, `stock_z`, `model_top_z`, `model_bottom_z`).
    - **`Place Clamp in Viewport…`** interactive pick mode (`ViewportPickMode::PlaceFixtureClamp`) with top-center Guidance HUD allowing machinists to click directly on a stock edge/corner or spoilboard to drop & orient a `ToeClamp` or `CylinderKeepOut`!
- **3D Viewport Translucent Fixture Rendering:**
  - Add `show_fixtures: bool` (default `true`, persisted in `prefs.toml`, toggleable via a `Fixtures` pill in the floating viewport HUD and `--capture-fixtures <true|false>` CLI capture flag).
  - Add `fixture_meshes: Vec<FixtureVisualMesh>` to `HighlightState` (`revision`, `fixture_id`, `mesh`, `base_rgba`, `colliding: bool`, `selected: bool`).
  - Render fixtures in `ViewportRenderer` with clean semi-transparent shaded surfaces + crisp dark silhouette wireframe edges so the vise/clamps are unmistakable while never obscuring the part, stock, or toolpaths inside the jaws.

---

## Verification Plan

1. **Unit & Geometry Tests (`chip-cam`, `chip-project`):**
   - Parametric watertightness & exact `Aabb3` / `FlatBvh` raycast tests for `ViseJaws` (with & without step shelf and parallels), `ToeClamp` (rotated `0°/90°/180°/270°` with stud tower), `Spoilboard`, `BoxKeepOut`, `CylinderKeepOut`, and `ImportedModel` (STEP & STL).
   - Lossless `.chip.toml` roundtrip test proving empty `fixtures` produces zero TOML diff on existing projects and populated `fixtures` roundtrips deterministically through `.chip.toml` and `.chipz` bundles.
   - Verification that `ModelRole::Fixture` bodies are excluded from `RelativeBox` stock sizing and target-part metrology.
2. **Headless Vulkan Visual Verification:**
   - Capture headless Vulkan screenshots showing a part held in `ViseJaws` + `ToeClamp`s + `Spoilboard` in Studio Dark and Daylight Light themes, and visually inspect via `read_file` to verify clean translucent shading and strict two-column `[Fixtures]` Inspector formatting.
3. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo check --locked -p chip-stock` (pure-CPU default)
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace` (preserving 100% byte-for-byte Phase 1–2 golden G-code parity)
