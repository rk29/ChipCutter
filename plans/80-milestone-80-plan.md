# Milestone 80 Implementation Plan — Unified 3D Machining Boundary Schema, True Model Silhouette, & `Undercut3D`/`Deburr3D` Containment

## Goal

Eliminate schema fragmentation and geometric inaccuracies in 3D XY cutting-area boundaries across all 3-axis surfacing, undercut, and 3D deburring operations: unify `boundary_mode`, `tool_containment`, and `boundary_offset` into one backward-compatible `MachiningBoundaryConfig`, replace the fake rectangular AABB `ModelSilhouette` with true projected 2D mesh silhouettes, fix `Adaptive3D`'s `StockBox` special-case bugs, and extend full boundary controls to `Undercut3D` and `Deburr3D`.

---

## Problem Diagnosis

1. **Duplicated 3D Boundary Fields Across 5 Structs (`crates/chip-cam/src/params/surface_3d.rs`):**
   - `Waterline3DParams`, `Adaptive3DParams`, `Parallel3DParams`, `Scallop3DParams`, and `Pencil3DParams` each duplicate `boundary_mode: MachiningBoundaryMode`, `tool_containment: ToolContainmentMode`, and `boundary_offset: ParamLength` inline, while `Undercut3DParams` and `Deburr3DParams` lack them altogether.
2. **`ModelSilhouette` Is Only a Rectangular Mesh AABB (`crates/chip-cam/src/surface3d/levels.rs:89-99`):**
   - `resolve_3d_containment_region()` currently returns `rect_region_2d(surface.aabb_wcs.min.x, min.y, max.x, max.y)` instead of the actual projected 2D silhouette of the 3D solid/mesh.
   - This causes `Scallop3D` iso-contours to trace an artificial rectangle around circular/organic models, and causes `Parallel3D`/`Waterline3D` to air-cut empty AABB corners.
   - `slab_projection.rs` (`project_surface_z_slab_xy`) already exists in `chip-cam` and computes exact projected 2D polygons of a `SurfaceMesh3D`.
3. **`Adaptive3D` Breaks `ToolContainmentMode` & `boundary_offset` Under `StockBox` (`crates/chip-cam/src/strategies/adaptive3d.rs:62-72`):**
   - `Adaptive3D` bypasses `resolve_3d_containment_region()` when `boundary_mode == StockBox`, silently ignoring `tool_containment`, clamping `boundary_offset` via `.max(1.0)` (breaking zero and negative offsets), and turning cylindrical Z-axis stock into a square AABB.
4. **Missing 3D Boundary Support on `Undercut3D` & `Deburr3D`:**
   - `Undercut3D` (`strategies/undercut3d.rs:181-194`) calls `resolve_3d_containment_region()` with hardcoded `SelectedRegions`, `ToolCenterOnBoundary`, and `0.0 mm` offset whenever selections are non-empty, exposing zero boundary controls.
   - `Deburr3D` (`strategies/deburr3d.rs:600-740`) has no XY containment boundary when `auto_detect_edges` is enabled, so users cannot auto-deburr edges restricted to a chosen XY region or model silhouette.
5. **Silent Fallback on Empty `SelectedRegions` (`surface3d/levels.rs:101-117`):**
   - When `boundary_mode == SelectedRegions` and the user has picked zero regions, `resolve_3d_containment_region()` silently falls back to `ModelSilhouette` without emitting a diagnostic warning.

---

## Proposed Changes

### 1. Shared `MachiningBoundaryConfig` Schema (Lossless Backward Compatibility)

#### [MODIFY] `crates/chip-cam/src/params/surface_3d.rs`
#### [MODIFY] `crates/chip-cam/src/params/undercut.rs`
#### [MODIFY] `crates/chip-cam/src/params/deburr3d.rs`
#### [MODIFY] `crates/chip-cam/src/params/mod.rs`
- Define a shared struct in `params/surface_3d.rs` (re-exported from `params/mod.rs` and `chip_cam`):
  ```rust
  #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
  pub struct MachiningBoundaryConfig {
      #[serde(default)]
      pub boundary_mode: MachiningBoundaryMode,
      #[serde(default)]
      pub tool_containment: ToolContainmentMode,
      #[serde(default = "default_zero_length")]
      pub boundary_offset: ParamLength,
  }
  ```
- Embed `#[serde(flatten)] pub boundary: MachiningBoundaryConfig` in `Waterline3DParams`, `Adaptive3DParams`, `Parallel3DParams`, `Scallop3DParams`, `Pencil3DParams`, `Undercut3DParams`, and `Deburr3DParams`:
  - `#[serde(flatten)]` preserves exact TOML/JSON key names (`boundary_mode`, `tool_containment`, `boundary_offset`) so all existing `schema_version = 1 / 2` `.chip.toml` and `.chipz` files deserialize with 100% backward compatibility.
  - Preserve per-strategy defaults (`Adaptive3D`: `StockBox` + `ToolOutsideBoundary` + `1.0 mm` offset; `Waterline3D`/`Parallel3D`/`Scallop3D`/`Pencil3D`: `ModelSilhouette` + `ToolOutsideBoundary`/`ToolCenterOnBoundary`; `Undercut3D`/`Deburr3D`: `ModelSilhouette` + `ToolOutsideBoundary` + `0.0 mm` offset — auto-switching to `SelectedRegions` semantics when `Undercut3D` loads legacy projects with selections and omitted `boundary_mode` or keeping explicit `boundary_mode` cleanly defaulted).
- Provide `Operation::machining_boundary(&self)` / `Operation::machining_boundary_mut(&mut self)` accessors on `Operation` (used by M83's unified `[Geometry]` tab and viewport preview).
- Ensure `reevaluate(&mut self, ctx: &EvalContext)` delegates to `self.boundary.reevaluate(ctx)` everywhere.

---

### 2. True Projected `ModelSilhouette`, True Cylindrical Stock XY, & Structured Resolution Result

#### [MODIFY] `crates/chip-cam/src/surface3d/levels.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/slab_projection.rs`
- Introduce a structured resolution result so both toolpath strategies and M83's 3D viewport preview share the **exact** same boundary computation:
  ```rust
  pub struct ResolvedBoundary2D {
      /// Raw nominal boundary loops before tool-radius / offset dilation (for viewport display).
      pub nominal_region: RegionSet2D,
      /// Effective tool-center containment region after `tool_containment` + `boundary_offset`.
      pub effective_region: RegionSet2D,
      /// Non-fatal diagnostic warnings (e.g., `SelectedRegions` chosen with no selections).
      pub warnings: Vec<String>,
  }
  ```
- Upgrade `MachiningBoundaryMode::ModelSilhouette`:
  - Compute the true projected 2D silhouette of `surface` onto the setup XY plane using `project_surface_z_slab_xy` across `[surface.aabb_wcs.min.z - 1.0, surface.aabb_wcs.max.z + 1.0]` (falling back to `rect_region_2d` only if projection yields an empty polygon on a degenerate mesh).
  - Keep projection deterministic and fast (convex/concave triangle-XY union with outer-shell cleanup so interior triangulation seams disappear cleanly).
- Upgrade `stock_xy_region(setup)`:
  - When `setup.stock` is a Z-axis cylinder (`StockShape::Cylinder` oriented along Z), emit an exact circular `RegionSet2D` (preserving circular arcs / high-resolution circle polygon) instead of a square bounding box!
- When `mode == MachiningBoundaryMode::SelectedRegions` and `selected_features_xy_region(selection)` is empty:
  - Emit a structured warning in `ResolvedBoundary2D::warnings` (surfaced through `ToolpathBuilder::push_warning` / validation) while keeping safe fallback behavior so existing jobs don't hard-crash unexpectedly.

---

### 3. Fix `Adaptive3D` `StockBox` Handling & Wire `Undercut3D` + `Deburr3D`

#### [MODIFY] `crates/chip-cam/src/strategies/adaptive3d.rs`
- Remove the hardcoded `StockBox` bypass (`adaptive3d.rs:62-72`) and route all three modes cleanly through `resolve_3d_containment_region_detailed()`:
  - Honor `params.boundary.tool_containment` (`ToolInside`, `ToolCenter`, `ToolOutside`) and signed `params.boundary.boundary_offset` (including `0.0 mm` and negative inset offsets).
  - Set `Adaptive3DParams::default()` boundary to `StockBox` + `ToolOutsideBoundary` + `1.0 mm` offset so default open-air roughing behavior stays identical (preserving golden/integration tests) while allowing users full control when they change containment or offset!

#### [MODIFY] `crates/chip-cam/src/strategies/undercut3d.rs`
- Replace lines `180-194`'s hardcoded `SelectedRegions`/`ToolCenterOnBoundary`/`0.0` containment with `resolve_3d_containment_region_detailed()` using `params.boundary` and `sphere_r`.
- When `params.boundary.boundary_mode == StockBox` or `ModelSilhouette` with default `ToolOutsideBoundary`, clip overhang contact points cleanly against `effective_region`; when `SelectedRegions` is selected, restrict overhang scanning and passes to the resolved containment region + offset.

#### [MODIFY] `crates/chip-cam/src/strategies/deburr3d.rs`
- Apply `params.boundary` in `auto_detect_edges` mode: filter auto-detected 3D B-Rep edges (or trim edge segments) to those whose XY projection lies within `effective_region` (respecting `StockBox`, `ModelSilhouette`, `SelectedRegions`, `tool_containment`, and `boundary_offset`).
- Keep explicit edge/loop/face selections working identically when `auto_detect_edges == false`, and document cleanly when XY boundary applies.

#### [MODIFY] `crates/chip-project/src/validation.rs`
- Add non-blocking validation warning when any operation uses `MachiningBoundaryMode::SelectedRegions` with zero 2D/planar selections.

---

## Verification Plan

1. **Serialization & Backward Compatibility (`chip-cam`, `chip-project`):**
   - Verify legacy `.chip.toml` snippets with flat `boundary_mode`/`tool_containment`/`boundary_offset` deserialize into `MachiningBoundaryConfig` identically, and legacy `Undercut3D`/`Deburr3D` configs without boundary keys load with clean defaults.
2. **True `ModelSilhouette` & Cylindrical Stock Tests (`chip-cam`):**
   - Circular dome / boss mesh test: assert `ModelSilhouette` produces a circular footprint whose area matches $\pi R^2$ within 2% (NOT the $4 R^2$ square AABB!) and `Scallop3D` / `Parallel3D` emit zero air-cut passes in the empty AABB corners.
   - Cylindrical Z-stock test: assert `StockBox` produces a circular footprint matching stock diameter.
3. **`Adaptive3D`, `Undercut3D`, & `Deburr3D` Boundary Parity Tests (`chip-cam`, `chip-sim`):**
   - Assert `Adaptive3D` under `StockBox` honors `ToolInsideBoundary`, `ToolCenterOnBoundary`, `ToolOutsideBoundary`, and negative `boundary_offset` without clamping to `1.0 mm`.
   - Assert `Undercut3D` respects `SelectedRegions` + `ToolInsideBoundary`/`ToolOutsideBoundary` + `boundary_offset`.
   - Assert `Deburr3D` with `auto_detect_edges = true` and `SelectedRegions` only deburrs edges inside the selected XY boundary.
   - Verify zero `G0` rapid collisions and zero target B-Rep gouges via `chip-sim` across all new tests, plus full golden G-code parity.
4. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
