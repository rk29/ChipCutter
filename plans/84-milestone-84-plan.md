# Milestone 84 Implementation Plan — `Model Bounds` (`ModelBoundingBox`), Sketch Quick-Select Auto-Expand, & Indexed 4th-Axis Auto-Rotate Viewport

## Goal

Eliminate three high-friction geometry-setup papercuts uncovered during real rotary and 3D machining workflows:
1. Add an explicit rectangular **`Model Bounds`** (`MachiningBoundaryMode::ModelBoundingBox`) option alongside `Stock Bounds`, `Model Silhouette`, and `Selected 2D Contours` so parts with concave or wavy outer silhouettes (like a double helix in rectangular stock) can clear cleanly without leaving floating stock at the edges or cutting out to chuck ends.
2. Auto-expand **`2D Sketch Layers & Quick Select`** (with per-entity quick-add and empty-state `+ New Sketch` shortcut) in `[Geometry]` whenever switching to `Selected 2D Contours`.
3. Automatically rotate the displayed 3D model, raw setup stock wireframe, chuck visuals, picking raycasts, Z-height planes, and `BoundaryPreviewOverlay` around the active rotary axis to match the selected **`Indexed 3+1`** operation's index angle (`angle_deg`) so the machined side always faces upright under `+Z` during geometry selection, sketching, and inspection.

---

## Problem Diagnosis

1. **No Rectangular `Model Bounds` Option (`crates/chip-cam/src/params/surface_3d.rs`, `surface3d/levels.rs`):**
   - `MachiningBoundaryMode` currently offers `StockBox`, `ModelSilhouette`, and `SelectedRegions`. On undulating models (e.g., `DoubleHelix-Indexed.chip.toml`), `ModelSilhouette` leaves thin floating stock slivers inside concave exterior bays, while `StockBox` extends all the way into rotary chuck / tailstock stock ends—forcing the user to manually sketch a bounding rectangle.
2. **`2D Sketch Layers & Quick Select` Stays Collapsed (`crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs:316-375`):**
   - `geom_pick_card` and `geom_sel_card` include `pick_default_open` in their `id_salt` so switching `Boundary Source` to `Selected 2D Contours` expands them immediately, whereas `geom_sketch_card` omits `pick_default_open` from its `id_salt`, leaving sketch selection hidden when users switch from `Model Silhouette` to `Selected 2D Contours`.
   - In addition, `2D Sketch Layers & Quick Select` only exposes `+ All` and sub-layer buttons—not individual sketch entities (e.g., picking one rectangle out of a multi-entity sketch) or a `+ New Sketch` prompt when no sketch exists yet.
3. **Indexed 4th-Axis Operations Always Display Unrotated (`0°`) While Editing (`crates/chip-gui/src/app/viewport_controller.rs:174-334`, `viewport/gizmos.rs`):**
   - `HighlightState` already supports zero-reupload GPU rotary transforms (`sim_rotary_deg`, `stock_model_wcs_transform()`, `cad_world_transform()`, and `world_ray_to_cad_model_space()`), **but** `viewport_controller.rs:328` only populates `sim_rotary_deg` during Simulation playback (`vis.rotary_deg`).
   - When selecting a `90°`, `180°`, or `270°` `Indexed 3+1` operation outside simulation playback, the CAD model and raw setup stock remain stuck at `0°`, AND `BoundaryPreviewOverlay` / `ZPlaneVisual` resolve in the unrotated `0°` `ResolvedSetup` instead of `build_indexed_virtual_setup(rs, &rot, angle_deg)`.

---

## Proposed Changes

### 1. Explicit `Model Bounds` (`MachiningBoundaryMode::ModelBoundingBox`) with Per-Side Signed Offsets (`-X`, `+X`, `-Y`, `+Y`)

#### [MODIFY] `crates/chip-cam/src/params/surface_3d.rs`
#### [MODIFY] `crates/chip-cam/src/params/deburr3d.rs`
#### [MODIFY] `crates/chip-cam/src/params/undercut.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/levels.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/facing.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/boundary_preview.rs`
- Add `ModelBoundingBox` to `MachiningBoundaryMode` (`#[serde(rename_all = "snake_case")]` → `"model_bounding_box"`, label `"Model Bounds"`), updating `MachiningBoundaryMode::ALL` to `[StockBox, ModelBoundingBox, ModelSilhouette, SelectedRegions]`.
- Add four backward-compatible, signed per-side `ParamLength` offset fields to `MachiningBoundaryConfig` (each `#[serde(default = "default_zero_length")]`, reevaluated in `MachiningBoundaryConfig::reevaluate`, wired through `Deburr3DParamsDe` / `Undercut3DParamsDe`, and hashed in `compute_boundary_preview_signature`):
  - `pub model_bounds_offset_x_neg: ParamLength` (`-X` side: positive expands outward toward `-X`, negative insets inward toward `+X`)
  - `pub model_bounds_offset_x_pos: ParamLength` (`+X` side: positive expands outward toward `+X`, negative insets inward toward `-X`)
  - `pub model_bounds_offset_y_neg: ParamLength` (`-Y` side: positive expands outward toward `-Y`, negative insets inward toward `+Y`)
  - `pub model_bounds_offset_y_pos: ParamLength` (`+Y` side: positive expands outward toward `+Y`, negative insets inward toward `-Y`)
- In `resolve_3d_containment_region_impl()` (`levels.rs`):
  - Resolve `ModelBoundingBox` nominal rectangle from `surface.aabb_wcs` (falling back to `stock_xy_region(setup)` if `!surface.aabb_wcs.is_valid()`):
    - `x_min = aabb.min.x - offset_x_neg`, `x_max = aabb.max.x + offset_x_pos`
    - `y_min = aabb.min.y - offset_y_neg`, `y_max = aabb.max.y + offset_y_pos`
    - Guard against collapsed bounds (`x_max <= x_min + 1e-4` or `y_max <= y_min + 1e-4`), returning `RegionSet2D::empty()` with a clear diagnostic warning rather than panicking.
  - Apply `tool_containment` + uniform signed `boundary_offset` (`delta = tool_shift + boundary_offset`) cleanly as a sharp offset rectangle (`rect_region_2d(x_min - delta, y_min - delta, x_max + delta, y_max + delta)`, matching `StockBox` box-stock semantics so corners stay crisp with zero round-fillet shrinkage!).
- Handle `ModelBoundingBox` (including its four per-side signed offsets) in `Facing` (`strategies/facing.rs`) and `resolve_facing_preview` (`boundary_preview.rs`), plus `needs_surface_mesh` (`boundary_preview.rs` returns `true` for `ModelBoundingBox` so `surface.aabb_wcs` is exact in rotated indexed frames!).
- In `geometry_tab.rs`:
  - Update `boundary_source_ui_label` and `pick_card_default_open` (`ModelBoundingBox` keeps pick cards collapsed by default, just like `StockBox` and `ModelSilhouette`).
  - Whenever `boundary.boundary_mode == MachiningBoundaryMode::ModelBoundingBox`, render four signed `ParamLength` expression inputs (**`-X Offset`**, **`+X Offset`**, **`-Y Offset`**, **`+Y Offset`**) with hover tooltips explaining `+` expands outward / `-` insets inward, updating the live 3D cyan/amber rectangle and `XY: W × H` readout immediately.

---

### 2. Sketch Quick-Select Auto-Expand & Per-Entity Selection Polish

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/mod.rs`
- Include `pick_default_open` in `geom_sketch_card`'s `id_salt` (`("geom_sketch_card", setup_idx, op_idx, pick_default_open)`) so switching `Boundary Source` to `Selected 2D Contours` (or `RotaryAxialBoundaryMode::SelectedGeometry`) automatically opens **`2D Sketch Layers & Quick Select`**.
- Inside `2D Sketch Layers & Quick Select`:
  - Keep `+ All` and sub-layer buttons on each sketch row, and add compact per-entity quick-add chips (e.g., `+ Rect 1`, `+ Circle 2`) for non-construction entities so users can bind a single sketched boundary rectangle in 1 click without switching to viewport picking.
  - When `sketches.is_empty()` and `Selected 2D Contours` is active, show a concise inline hint (`"No 2D sketches in this project yet — click '+ Sketch' in the top action bar to draw a boundary rectangle."`) or emit an `OperationInspectorAction::CreateSketch` button directly inside the card!

---

### 3. Auto-Rotate Model & Stock for Selected Indexed 4th-Axis Operation

#### [MODIFY] `crates/chip-gui/src/prefs.rs`
#### [MODIFY] `crates/chip-gui/src/panels/settings.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/gizmos.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/highlight.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/boundary_preview.rs`
- Add `#[serde(default = "default_true")] pub auto_rotate_indexed_op: bool` to `UserPreferences` (`prefs.toml`, backward compatible), exposed in:
  - **`Settings → Viewport & Navigation`** (`Auto-rotate workpiece to active Indexed 3+1 angle`), AND
  - Inline checkbox inside **`4th-Axis Indexing / Wrapping`** (`passes_tab/mod.rs`) right next to the `A/B/C-Axis Index` angle control so users discover and control it right where they set index angles!
- **Effective Viewport Rotary Angle Precedence (`viewport_controller.rs`):**
  1. When `sim_panel` has an active 4th-axis simulation playback visual (`vis.stock.is_some()` or `vis.cutter.is_some()`), keep simulation playback in control of `highlight.sim_rotary_deg = vis.rotary_deg`.
  2. Otherwise, when `prefs.auto_rotate_indexed_op == true` and `InspectorFocus::Operation { setup_idx, op_idx }` points to an operation with `OperationRotaryMode::Indexed3Plus1 { angle_deg }` on a Setup with `rs.rotary.is_some()`, set `highlight.sim_rotary_deg = angle_deg`!
  3. Show a subtle HUD pill in the top-left viewport title badge when auto-rotated (e.g., `Setup 1 — Indexed A = 90.0° [G54] · mm`) so the user always knows the view is indexed.
- **Lockstep Rotation of Raw Setup Stock & Virtual-Setup Preview Frame:**
  - Update `emit_stock_visuals()` in `viewport/gizmos.rs` to apply `highlight.stock_model_wcs_transform()` to raw Setup stock wireframe vertices (so rectangular/box stock like `DoubleHelix-Indexed.chip.toml` visibly rotates by `90°`/`180°`/`270°` with the CAD solid!).
  - When inspecting an `Indexed3Plus1 { angle_deg }` operation with auto-rotate active, compute `virtual_rs = build_indexed_virtual_setup(rs, &rot, angle_deg)` and evaluate:
    - `BoundaryPreviewOverlay` (`boundary_preview_cache`) against `virtual_rs` (transforming the resulting wireframe loops into upright `+Z` Setup WCS using the indexed frame!) so `Model Bounds`, `Model Silhouette`, and `Stock Bounds` match the **exact upright rotated orientation** that `generate_operation_toolpath` will machine!
    - `ZPlaneVisual` (`OperationTab::Heights`) against `virtual_rs` so Z-height planes align cleanly with the upright indexed stock top/bottom.
  - Verify `world_ray_to_cad_model_space()` keeps 3D face/loop/edge/hole hover and click picking 100% accurate at `90°`, `180°`, `270°`, and arbitrary angles (e.g. `45°`, `120°`).

---

## Verification Plan

1. **`ModelBoundingBox` Unit & Simulation Tests (`chip-cam`, `chip-sim`):**
   - Double-helix / concave wavy-perimeter mesh test: assert `ModelBoundingBox` produces a sharp rectangular XY footprint matching `surface.aabb_wcs` (`width × depth`), honors independent signed per-side offsets (`-X`, `+X`, `-Y`, `+Y` — both positive outward expansions and negative inward insets, plus graceful empty-region warning when negative insets collapse an axis), honors `ToolInsideBoundary` / `ToolCenterOnBoundary` / `ToolOutsideBoundary` + uniform signed `boundary_offset`, clears outer concave bays cleanly without leaving floating edge slivers, and passes `chip-sim` with 0 crashes and 0 gouges.
   - Verify lossless TOML roundtrip of `boundary_mode = "model_bounding_box"` and all four `model_bounds_offset_*` fields (plus legacy TOML default `0.0 mm`) across all 8 boundary-aware strategies.
2. **Sketch Quick-Select & Indexed Auto-Rotate Tests (`chip-gui`):**
   - Headless `egui` test verifying switching `Boundary Source` from `Model Silhouette` → `Selected 2D Contours` expands `2D Sketch Layers & Quick Select` and per-entity quick-add buttons insert `GeometrySelectionItem::SketchEntity` / `SketchRegion` cleanly.
   - Viewport controller & picking tests verifying selecting an `Indexed3Plus1 { angle_deg: 90.0 }` operation sets `highlight.sim_rotary_deg = 90.0`, rotates `emit_stock_visuals()` box stock by `90°`, aligns `world_ray_to_cad_model_space` picking hits, and evaluates `BoundaryPreviewOverlay` in `build_indexed_virtual_setup` (`90°` rotated bounds, e.g. swapped Y/Z extents on an X-axis rotary setup!).
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
