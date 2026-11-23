# Milestone 82 Implementation Plan — 2.5D Cutting Area Consistency & Inspector Control Hygiene

## Goal

Bring 2.5D milling operations (`Facing`, `Pocket2D`/`Adaptive2D`, `Contour2D`) up to the same cutting-area consistency as 3-axis strategies—upgrading `Facing` to support `MachiningBoundaryConfig` with exact polygon + raised-island clipping, enabling boss-clearing (`StockContour` outer boundary) and 2D sketch open-edge control on `Pocket2D`/`Adaptive2D`, adding optional XY containment clipping on `Contour2D`, and removing confusing/ignored parameter controls across all operation inspectors.

---

## Problem Diagnosis

1. **`Facing` Collapses Selections to a Rectangular `Aabb2` (`crates/chip-cam/src/strategies/facing.rs:34-52`):**
   - When the user selects an L-shaped planar face, a circular boss top, or a stepped shelf with raised islands, `generate_facing()` unions only `exact_aabb()`—machining outside the selected polygon and failing to respect raised islands unless `avoid_model_overhangs` happens to clip them.
   - `Facing` has no `MachiningBoundaryMode` (`StockBox`, `ModelSilhouette`, `SelectedRegions`), `ToolContainmentMode`, or lateral `boundary_offset`.
2. **`Pocket2D` / `Adaptive2D` Limitations (`crates/chip-cam/src/strategies/pocket2d.rs`, `adaptive2d.rs`):**
   - Machinists cannot clear material *around* a selected 2D/3D boss up to the Setup's Stock Box without manually drawing a stock-sized sketch rectangle around the boss.
   - Open outer edges are auto-detected for 3D B-Rep `PlanarFace` selections, but **2D Sketch / `ClosedLoop` selections are always treated as closed pockets** (no option to treat the outer boundary as open / approach from stock edge).
3. **`Contour2D` Lacks Optional XY Boundary Clipping:**
   - Users cannot optionally restrict a contour operation to a chosen XY containment box/mode without manually splitting sketch chains.
4. **Misleading / Ignored Controls in Operation Inspector (`crates/chip-gui/src/panels/operation_inspector/`):**
   - `Parallel3D`, `Scallop3D`, `Pencil3D`, and `ThreadMill` display `Multiple Depths` / `Max Stepdown` in the generic Depth/Stock card even though those kernels ignore `passes.multiple_depths` (`ThreadMill` also ignores `stock_to_leave_radial`).
   - `Adaptive3D` displays *both* `Max Stepdown` (in Depth/Stock) and `Rough Stepdown` (in Adaptive3D card), even though `Adaptive3D` only reads `params.max_roughing_stepdown`.
   - `SlotUndercut` and `Dovetail` read `passes.stock_to_leave_{radial,axial}` in code, yet hide the Depth/Stock card in UI.
   - `VCarve` ignores `heights.bottom_z_wcs` (using `flat_depth` instead) and `EdgeChamfer` ignores `heights.top_z_wcs` / `heights.bottom_z_wcs`, with no hint on the `[Heights]` tab.

---

## Proposed Changes

### 1. Upgrade `Facing` to `MachiningBoundaryConfig` + Exact Polygon & Island Clipping

#### [MODIFY] `crates/chip-cam/src/params/milling_2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/facing.rs`
- Embed `#[serde(flatten)] pub boundary: MachiningBoundaryConfig` on `FacingParams` (default: `StockBox`, `ToolOutsideBoundary`, `0.0 mm` offset — preserving exact default full-stock facing behavior and backward compatibility for legacy `.chip.toml` files where empty selections meant stock and non-empty selections meant selected faces!).
  - Backward-compatibility migration rule: when `boundary_mode` was omitted in a legacy project that has non-empty `op.selections`, honor the selections cleanly (or default `FacingBoundaryMode` behavior: `StockBox` uses stock XY, `ModelSilhouette` uses true projected model silhouette when a 3D model is loaded, `SelectedRegions` uses exact selected `PlanarFace` / closed `Contour` polygons including their `raised_islands_wcs`!).
- Upgrade `generate_facing()` pass generation:
  - Resolve the exact 2D polygon region (outer loops minus `raised_islands_wcs` eroded by `tool_r + radial_leave` when facing around raised bosses!).
  - Compute raster lanes along `u_axis` across the region's `(u, v)` span, **clipping each scanline to the exact polygon region** (dilated by `pass_extension` along open/outer edges and respecting `tool_containment` + `boundary_offset`), instead of cutting the full rectangular `Aabb2`.
  - Keep pure rectangular stock-box facing byte-for-byte compatible with existing Phase 1–5 golden G-code tests (`torture_01_T7_38mm_3_insert_apkt_face.nc`, `v4_thread_mill_01_*`).
- Expose `finishing_stepdown` cleanly in `passes_tab/milling_2d.rs` (currently a dead struct field on `FacingParams`).

---

### 2. `Pocket2D` / `Adaptive2D` Boss Clearing (`StockContour`) & Open-Boundary Support

#### [MODIFY] `crates/chip-cam/src/params/milling_2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/pocket2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/adaptive2d.rs`
- Add `PocketBoundaryMode` on `Pocket2DParams` (`#[serde(default)]`):
  - `SelectedPockets` *(default — exact existing behavior)*: clear inside selected faces/closed loops, preserving raised islands.
  - `ClearOutsideSelectedToStock`: treat selected closed loops / planar faces as **bosses (islands)** to leave standing, and clear the region `StockXY \ SelectedBosses` with all outer stock edges marked as open so the tool enters smoothly from open air!
- Add `#[serde(default)] pub treat_outer_boundary_as_open: bool` on `Pocket2DParams` (used for 2D Sketch / `ClosedLoop` selections so users can clear open-sided 2D pockets from outside the boundary without needing a 3D B-Rep solid).

---

### 3. Optional XY Containment on `Contour2D` (Clip Window)

#### [MODIFY] `crates/chip-cam/src/params/milling_2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/contour2d.rs`
- Add optional `#[serde(default)] pub clip_to_stock_bounds: bool` (and `#[serde(default)] pub open_chain_extension: ParamLength` for tangent start/end extension on open chains, matching `Deburr3D::edge_extension` and `SlotUndercut::entry_extension`), allowing users to cleanly extend open 2D profile chains past stock edges or trim passes to stock bounds.

---

### 4. Inspector Parameter & Height Control Hygiene

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/milling_2d.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/undercut_cards.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/heights_tab.rs`
- Conditionally hide unused controls in the shared `Depth Passes & Stock to Leave` card (`passes_tab/mod.rs:246-294`):
  - Hide `Multiple Depths` & `Max Stepdown` on `Parallel3D`, `Scallop3D`, `Pencil3D`, `ThreadMill`, and `Adaptive3D` (since `Adaptive3D` already owns `Rough Stepdown` / `Fine Stepdown` in its dedicated card).
  - Hide `Radial Stock to Leave` on `ThreadMill` (controlled by `Pitch Dia Offset`) and `Facing` (axial-only unless raised islands are present).
  - Surface `Stock to Leave` clearly on `SlotUndercut` and `Dovetail` (or unify with their existing `finish_allowance` field so no hidden value can silently shift cut depth/width).
- Add clear contextual inline banners on `[Heights]` when a strategy has specialized height semantics:
  - `VCarve`: note that `Bottom Height` is governed by `Flat Depth` in `[Passes]` (only `Top Height` sets the carving top plane).
  - `ChamferTrace` (`EdgeChamfer` mode): note that Z depth is derived automatically from the selected chamfer edge + `Tip Offset`.
  - `Rotary2Linear` & `Rotary4AxisSim`: show live equivalent **Outer Radius (`R_max`)** and **Core Floor Radius (`R_min`)** readouts right beside `Top Height` and `Bottom Height`!

---

## Verification Plan

1. **Exact Polygon `Facing` Tests (`chip-cam`, `chip-sim`):**
   - L-shaped planar face + raised central boss test: verify `Facing` clears the L-shape completely while producing **zero** passes over the missing quadrant and **zero** gouges into the raised boss.
   - Verify `Facing` with `ModelSilhouette` and `StockBox` modes + `ToolContainmentMode` + `boundary_offset`, while keeping existing golden G-code tests byte-for-byte identical.
2. **`Pocket2D` / `Adaptive2D` & `Contour2D` Tests (`chip-cam`, `chip-sim`):**
   - `ClearOutsideSelectedToStock` test on a circular/rectangular boss: verify stock outside the boss is cleared cleanly with open-air entry and zero boss gouges (`chip-sim` heightmap & Tri-Dexel verified).
   - 2D Sketch `treat_outer_boundary_as_open = true` test: verify cutter approaches cleanly from outside the sketch boundary.
   - `Contour2D` `open_chain_extension` test: verify tangent start/end extensions match requested length with zero gouges.
3. **GUI Hygiene Regression Checks (`chip-gui`):**
   - Verify hidden/shown controls per `OperationKind` (no duplicate stepdown on `Adaptive3D`, no dead `Multiple Depths` on `Parallel3D`/`Scallop3D`/`Pencil3D`/`ThreadMill`, radial readouts on rotary `[Heights]`).
4. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
