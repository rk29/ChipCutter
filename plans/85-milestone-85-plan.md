# Milestone 85 Implementation Plan — Per-Operation & Shared Avoid / Keep-Out Regions & Fixture-Trimmed Cutting Areas Kernel

## Goal

Introduce first-class, backward-compatible **Per-Operation & Shared Avoid / Keep-Out Regions (`Operation::avoid: AvoidRegionConfig` + `ProjectDocument::avoid_regions: Vec<SharedAvoidRegion>`)** across `chip-cam` and `chip-project`, allowing any 2.5D, 3-axis, undercut/deburr, or 4th-axis operation to exclude selected 3D faces, 2D sketch regions/loops, Setup workholding fixture bodies, or **reusable named shared keep-out regions linked across multiple operations** (with configurable radial & axial clearances) directly from its cutting area—complementing Phase 17's fixture-aware rapid lifting with true cutting-area exclusion.

---

## Problem Diagnosis

1. **Inclusion-Only Cutting Boundaries:**
   - Every cutting-area mechanism in `chip-cam` (`op.selections`, `MachiningBoundaryConfig`, `RotaryAxialBoundaryConfig`, `PocketBoundaryMode`) is strictly additive/inclusion-only. Machinists cannot say *"machine this Stock Box / Model Bounds, EXCEPT stay clear of these toe clamps, dowel bosses, or sketched keep-out zones"*.
2. **Fixtures Only Lift Rapids (`crates/chip-cam/src/fixture_avoidance.rs`):**
   - Phase 17's `route_toolpath_around_fixtures` lifts `G0` rapid moves over vises and clamps, and Rule 6 `FixtureCollision` blocks export if a cutting move hits a fixture—**but** cutting strategies themselves never trim their XY or rotary cutting regions against fixture solids, forcing users to manually craft complex sketch boundaries around every clamp!

---

## Proposed Changes

### 1. Backward-Compatible `AvoidRegionConfig`, `SharedAvoidRegion` Schema, & Cache Hashing

#### [MODIFY] `crates/chip-cam/src/params/common.rs`
#### [MODIFY] `crates/chip-cam/src/operation.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
#### [MODIFY] `crates/chip-project/src/document.rs`
#### [MODIFY] `crates/chip-project/src/cache.rs`
#### [MODIFY] `crates/chip-project/src/reload.rs`
#### [MODIFY] `crates/chip-project/src/validation.rs`
- Define `SharedAvoidRegion` and `AvoidRegionConfig` in `crates/chip-cam/src/params/common.rs`:
  ```rust
  /// Reusable named Avoid / Keep-Out region stored on `ProjectDocument::avoid_regions`
  /// and referenced by ID from any number of operations (within the same Setup or across Setups).
  #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
  pub struct SharedAvoidRegion {
      pub id: String,
      pub name: String,
      /// 3D faces, closed loops, holes, or 2D sketch regions/entities/layers to stay out of.
      #[serde(default, skip_serializing_if = "Vec::is_empty")]
      pub selections: Vec<GeometrySelectionItem>,
      /// Optional Setup `FixtureBody::id` entries grouped into this shared keep-out region.
      #[serde(default, skip_serializing_if = "Vec::is_empty")]
      pub fixture_ids: Vec<String>,
      /// Optional radial clearance override (`mm`); when `None`, inherits the operation's `avoid.radial_clearance`.
      #[serde(default, skip_serializing_if = "Option::is_none")]
      pub radial_clearance: Option<ParamLength>,
      /// Optional axial clearance override (`mm`); when `None`, inherits the operation's `avoid.axial_clearance`.
      #[serde(default, skip_serializing_if = "Option::is_none")]
      pub axial_clearance: Option<ParamLength>,
      /// Temporarily disable this shared region across all linked operations without deleting it.
      #[serde(default, skip_serializing_if = "std::ops::Not::not")]
      pub suppressed: bool,
  }

  #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
  pub struct AvoidRegionConfig {
      /// IDs of reusable `ProjectDocument::avoid_regions` linked to this operation.
      #[serde(default, skip_serializing_if = "Vec::is_empty")]
      pub shared_region_ids: Vec<String>,
      /// Operation-local 3D faces, closed loops, holes, or 2D sketch regions/layers to stay out of.
      #[serde(default, skip_serializing_if = "Vec::is_empty")]
      pub selections: Vec<GeometrySelectionItem>,
      /// Specific Setup `FixtureBody::id` entries to subtract from the operation's cutting area.
      #[serde(default, skip_serializing_if = "Vec::is_empty")]
      pub fixture_ids: Vec<String>,
      /// Automatically subtract ALL non-suppressed Setup fixtures overlapping the operation's Z band.
      #[serde(default)]
      pub avoid_all_fixtures: bool,
      /// Extra XY / radial clearance (`mm`, default `0.5 mm`) added around keep-out boundaries.
      #[serde(default = "default_avoid_clearance")]
      pub radial_clearance: ParamLength,
      /// Extra Z / axial clearance (`mm`, default `0.5 mm`) for Z-band overlap and rotary axial exclusion.
      #[serde(default = "default_avoid_clearance")]
      pub axial_clearance: ParamLength,
  }
  ```
- Store `#[serde(default, skip_serializing_if = "Vec::is_empty")] pub avoid_regions: Vec<SharedAvoidRegion>` on `ProjectDocument` (omitted from `.chip.toml` when empty, with lookup/helper methods `avoid_region(&id)`, `avoid_region_mut(&id)`, `remove_avoid_region(&id)` which also prunes stale IDs across all setups/operations, and `avoid_region_usage_count(&id)`).
- Embed `#[serde(default, skip_serializing_if = "AvoidRegionConfig::is_empty")] pub avoid: AvoidRegionConfig` on `Operation`:
  - `is_empty()` returns `true` when `shared_region_ids.is_empty() && selections.is_empty() && fixture_ids.is_empty() && !avoid_all_fixtures` so existing `.chip.toml` / `.chipz` files omit the section completely (100% deterministic & backward compatible!).
  - Implement `AvoidRegionConfig::reevaluate(&mut self, ctx: &EvalContext)` and `SharedAvoidRegion::reevaluate(&mut self, ctx: &EvalContext)`, invoking them alongside `heights`/`passes`/`linking`.
- Ensure `compute_operation_input_hash` (`crates/chip-project/src/cache.rs`) hashes:
  - `op.avoid`,
  - every `SharedAvoidRegion` referenced by `op.avoid.shared_region_ids` (so editing or toggling `suppressed` on a shared keep-out region immediately marks **all referencing operations** `Dirty`!), AND
  - active Setup fixtures whenever `op.avoid.avoid_all_fixtures`, `!op.avoid.fixture_ids.is_empty()`, or any linked non-suppressed `SharedAvoidRegion` references `fixture_ids`.
- Update associative STEP revision reload (`crates/chip-project/src/reload.rs`) so `GeometricSignature` re-matching also updates `selections` inside both `op.avoid.selections` and `ProjectDocument::avoid_regions[*].selections`.
- Add validation diagnostics in `crates/chip-project/src/validation.rs` if `avoid.shared_region_ids` references a missing shared region ID, `avoid.fixture_ids` (local or shared) references a deleted fixture ID, or any `radial_clearance` / `axial_clearance` is negative.

---

### 2. Shared Keep-Out Resolution Kernel (`chip-cam`)

#### [NEW] `crates/chip-cam/src/avoid_regions.rs`
#### [MODIFY] `crates/chip-cam/src/surface3d/levels.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
#### [MODIFY] `crates/chip-project/src/dispatcher.rs`
- Pass `&[ResolvedFixtureSolid]` and `&[SharedAvoidRegion]` into `OperationGenRequest` (defaulting to `&[]` on direct callers, populated automatically by `generate_operation_toolpath_with_fixtures` and `chip-project`'s `JobDispatcher` / `generate_project_blocking`).
- Implement `ResolvedAvoidRegion2D { pub nominal_region: RegionSet2D, pub effective_region: RegionSet2D, pub warnings: Vec<String> }` via `resolve_avoid_region_2d(...)`, unioning the operation's local keep-out config with every non-suppressed `SharedAvoidRegion` linked via `op.avoid.shared_region_ids`:
  1. **Selection Keep-Outs:** Extract local `op.avoid.selections` and each linked shared region's `selections` using `extract_selection_features_with_sketches` (`PlanarFace` outer loops, closed `Contour` loops, `CylindricalHole` circles) into `RegionSet2D`.
  2. **Fixture Keep-Outs:** For each `ResolvedFixtureSolid` selected by `avoid_all_fixtures`, local `avoid.fixture_ids`, or a linked shared region's `fixture_ids`:
     - Check whether `fixture.mesh_wcs.aabb` overlaps the operation's vertical Z band `[z_bottom - eff_axial_clearance, z_top + eff_axial_clearance]` (skipping spoilboards below `z_bottom - eff_axial_clearance` and clamps completely outside Z range!).
     - Project the overlapping fixture slab onto XY via `project_surface_z_slab_xy` (reusing the exact BVH-accelerated Z-slab projector!) and union into `nominal_region`.
  3. **Per-Source Tool-Center Inflation & Union:** Each source group (operation-local and each linked `SharedAvoidRegion`, using its optional `radial_clearance` / `axial_clearance` override or falling back to `op.avoid`'s clearances) inflates its nominal footprint by `tool_r + eff_radial_clearance` and unions into `effective_region` (so the cutter periphery stays strictly `>= eff_radial_clearance` away from every keep-out wall/fixture!).
- Extend `ResolvedBoundary2D` (`surface3d/levels.rs`) with `pub avoid_nominal_region: RegionSet2D` and `pub avoid_effective_region: RegionSet2D` so both toolpath strategies and M86's 3D viewport preview render the exact same keep-out footprints!
- Handle `Indexed3Plus1` cleanly: transform fixture solids and local/shared avoid selections into the rotated `virtual_setup` frame before projecting onto virtual XY!

---

### 3. Cross-Strategy Cutting-Area Subtraction

#### [MODIFY] `crates/chip-cam/src/strategies/adaptive3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/waterline3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/parallel3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/scallop3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/pencil3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/undercut3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/deburr3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/facing.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/pocket2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/contour2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/vcarve.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/drill_bore.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary2linear.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary4axis_sim.rs`
- **3-Axis & 3D Surface (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Undercut3D`, `Deburr3D`):**
  - Subtract `avoid.effective_region` from `resolved_boundary.effective_region` (`effective_region.difference(&avoid_effective)`) before pass generation; return `CamError::EmptyToolpath` with a descriptive diagnostic if keep-out subtraction removes 100% of the cutting region.
- **2.5D Milling:**
  - **`Facing`:** Union `avoid.effective_region` into `island_exclusion_region` so facing scanlines clip cleanly around keep-out regions/clamps with safe retracts over them.
  - **`Pocket2D` / `Adaptive2D`:** Subtract `avoid.effective_region` from the clearable pocket tool-center region so both `ContourParallel`/`ZigZag` and `Adaptive` treat keep-out zones as protected islands!
  - **`Contour2D` & `VCarve`:** Clip emitted 2D passes cleanly outside `avoid.effective_region` (`clip_polyline_outside_region_set`) with safe Z retract links across skipped spans.
  - **`DrillBore`:** Skip any discrete hole whose center lies inside `avoid.effective_region` and emit a structured warning (`"Skipped N hole(s) inside Avoid / Keep-Out region"`).
- **4th-Axis Rotary (`Rotary2Linear`, `Rotary4AxisSim`):**
  - Project `avoid.selections` and Z/radial-overlapping fixtures onto the rotary axis `u_vec` (`± (tool_r + axial_clearance)`) to build excluded axial intervals `[u_lo, u_hi]`; split/skip pass segments entering excluded axial bands with safe radial retracts (`safe_z_wcs`) across the keep-out zone!

---

## Verification Plan

1. **Keep-Out Kernel, Shared Regions, & Backward Compatibility Tests (`chip-cam`, `chip-project`):**
   - Verify legacy `.chip.toml` files without `avoid_regions` or `[setups.operations.avoid]` load cleanly with `avoid.is_empty() == true` and omit both sections when re-serialized.
   - Verify linking a `SharedAvoidRegion` to multiple operations (within the same Setup and across two Setups) subtracts the shared keep-out from every linked operation, respects optional per-region clearance overrides and `suppressed`, and merges cleanly with operation-local keep-outs.
   - Verify editing `avoid.selections`, `avoid.fixture_ids`, `avoid.shared_region_ids`, `avoid.avoid_all_fixtures`, mutating/suppressing a referenced `SharedAvoidRegion`, or moving a referenced Setup `FixtureBody` changes `OperationInputHash` and marks all affected operations `Dirty` (and removing a `SharedAvoidRegion` prunes stale `shared_region_ids` cleanly).
2. **Strategy + `chip-sim` Zero-Collision Tests (`chip-cam`, `chip-sim`, `chip-project`):**
   - **Fixture-Trimmed Cutting Area (`Adaptive3D`, `Facing`, `Pocket2D`, `Parallel3D`):** Place a `ToeClamp` fixture overlapping the workpiece edge; with `avoid_all_fixtures = true`, assert **zero Rule 6 `FixtureCollision`** diagnostics and **zero target gouges** in `chip-sim` (both Heightmap and Tri-Dexel) while all unblocked stock is machined cleanly!
   - **Sketch Keep-Out Zone (`Contour2D`, `VCarve`, `DrillBore`, `Deburr3D`):** Verify passes/holes inside a sketched keep-out rectangle are cleanly skipped/clipped by `>= tool_r + radial_clearance`.
   - **Rotary Axial Keep-Out (`Rotary2Linear`, `Rotary4AxisSim`):** Verify axial keep-out band splits passes with safe radial retracts over the excluded `u` span and passes `CylindricalStock` + `TriDexelStock` verification with 0 crashes/gouges.
   - Verify 100% byte-for-byte parity across all existing Phase 1–5 golden G-code suites in `chip-cli`.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
