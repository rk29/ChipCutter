# Milestone 61 Implementation Plan — Automatic Safe-Rapid Routing & 3D Fixture Collision Verification

## Goal

Turn Milestone 60's 3D workholding geometry into active physical CNC machine protection:
1. **Automatic Safe-Rapid & Linking Routing (`chip-cam`):** Automatically lift `G0` rapid and non-cutting link moves cleanly over clamps, vise jaws, bolt studs, and keep-outs (accounting for flute, shank, and holder/collet radii + fixture clearance margins) while never silently altering cutting geometry.
2. **Rule 6 — Full-Assembly 3D Fixture Collision Verification (`chip-sim`):** Detect any rapid, link, or cutting collision between the **flute**, **non-cutting shank**, or **holder/collet** and any enabled fixture body across **Heightfield**, **Tri-Dexel**, and **4th-Axis Cylindrical** engines with 100% CPU-vs-GPU parity, surfacing export-blocking `SimIssueKind::FixtureCollision` diagnostics and crimson fixture highlighting in the 3D viewport.

---

## Non-Negotiable Invariants

- **Zero Impact When No Fixtures Exist:** When `setup.fixtures` has no enabled bodies (or `auto_avoid_fixtures_on_rapids == false`), `route_toolpath_around_fixtures` is an exact no-op — preserving 100% byte-for-byte golden G-code parity on all existing projects.
- **Never Silently Mutate Cutting Passes:** Only non-cutting moves (`MotionRole::Rapid`, `Retract`, `StayDown`, `MicroLift`, `Link`) may be re-routed vertically over fixtures. If an actual `Cut`, `Plunge`, `Ramp`, or `LeadIn`/`LeadOut` move collides with a fixture, it is **never** silently skipped or distorted; instead it triggers a `SimIssueKind::FixtureCollision` `CriticalError` blocking G-code export so the machinist moves the clamp or edits the operation explicitly.
- **Exact Full-Assembly Clearance:** Both rapid routing and collision checking evaluate the full `AnalyticCutterEnvelope` (flute radius, shank radius above `flute_length`, and holder/collet cone radius above `stickout_length`) — catching a wide ER16 collet nut clipping a tall toe-clamp stud even when the small Ø3 mm endmill tip would have cleared it!
- **Exact CPU & GPU Diagnostic Parity:** `FixtureCollision` diagnostics (severity, location, excess penetration, coalescing, and offending `global_seg_idx`) must match identically between `CpuExactBackend` and `WgpuComputeBackend`.

---

## Proposed Changes

### 1. Full-Assembly Swept-Cutter vs. `FlatBvh` Collision Kernel (`chip-cam` / `chip-sim`)

#### [NEW] `crates/chip-cam/src/fixture_avoidance.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
#### [MODIFY] `crates/chip-cam/src/toolpath.rs`
#### [MODIFY] `crates/chip-project/src/dispatcher.rs`
- **Revolved Assembly vs. Fixture BVH Clearance Query (`min_safe_z_over_fixtures` & `check_segment_fixture_collision`):**
  - Given a `Tool` (`AnalyticCutterEnvelope` with `flute_length`, `shank_radius`, `stickout_length`, `holder_radius_at_z`) and a slice of `ResolvedFixtureSolid`s (each carrying its `FlatBvh`, `Aabb3`, `clearance_margin_mm`, and `allowed_overcut_mm`):
    - **Horizontal/Sloped Segment Sweep Check:** Decomposes linear/arc chords into conservative bounding capsules/slabs and queries each fixture's `FlatBvh` via `query_z_slab` / AABB traversal.
    - For every candidate fixture triangle `(v0, v1, v2)` within the inflated radial footprint (`R_max = holder_max_radius + clearance_margin_mm`), evaluates exact minimum horizontal distance `r_xy(z)` from the tool axis along the chord at key assembly heights (tip, flute top, stickout/holder bottom, holder top) against the triangle's exact segment/point distance.
    - Returns structured `FixtureHit { fixture_id, fixture_name, contact_part: ToolAssemblyPart (Flute | Shank | Holder), penetration_mm, required_lift_z_wcs, hit_point_wcs }`.
    - For `FixtureKind::Spoilboard`, permits downward penetration up to `allowed_overcut_mm` (e.g., `0.3 mm` sacrificial skim) before flagging a collision!
- **Automatic Safe-Rapid & Linking Re-Routing (`route_toolpath_around_fixtures`):**
  - Runs at the end of `generate_operation_toolpath_with_rest` (and inside `JobDispatcher`) whenever `setup.auto_avoid_fixtures_on_rapids` is enabled and `resolved_fixtures` is non-empty.
  - Scans contiguous non-cutting transition chains (`Rapid`, `Retract`, `MicroLift`, `StayDown`):
    - If a horizontal or diagonal transition between `p0` and `p1` intersects any fixture (flute, shank, or holder + `clearance_margin_mm`), computes `z_safe = max(resolved_setup.clearance_z_wcs, max_required_lift_z + 2.0)`.
    - Replaces the colliding direct move with an exact 3-move safe clearance bridge:
      1. Vertical `RapidMove` up to `(p0.x, p0.y, z_safe)` (only if `p0.z < z_safe - 1e-4`),
      2. Horizontal `RapidMove` across to `(p1.x, p1.y, z_safe)`,
      3. Vertical `RapidMove` down to `(p1.x, p1.y, p1.z)` (with safe feed-plane guard when descending near stock!).
    - On 4th-axis indexed setups, checks rotary clearance radius `rotary_clearance_r` against fixtures as well.
    - Records routing telemetry (`rapid_lifts_inserted: usize`, `max_lifted_z_wcs: f64`) on `GeneratedToolpath` so the GUI/CLI can show *"3 rapid moves automatically lifted over 2 clamps (max Z = +38.0 mm)"*.
  - Validates that the final lifted `z_safe` does not exceed the active machine's Z travel envelope (emitting a clear validation warning if a clamp is too tall for the machine's Z clearance!).

---

### 2. Rule 6: `SimIssueKind::FixtureCollision` Across All Engines & Backends (`chip-sim`, `chip-project`, `chip-cli`)

#### [MODIFY] `crates/chip-sim/src/verifier.rs`
#### [MODIFY] `crates/chip-sim/src/verify_common.rs`
#### [MODIFY] `crates/chip-sim/src/driver.rs`
#### [MODIFY] `crates/chip-project/src/pipeline.rs`
#### [MODIFY] `crates/chip-cli/src/commands/validate_sim_build.rs`
- **New Verification Rule (`SimIssueKind::FixtureCollision`):**
  - Add `FixtureCollision` variant to `SimIssueKind` (`label = "Fixture / Clamp Collision"`, `default_severity = SimSeverity::CriticalError`).
  - Add `fixtures: &'a [ResolvedFixtureSolid]` field to `SetupSimulationInput<'a>` (default `&[]` for existing callers/tests).
  - Run `rayon`-parallel exact swept-assembly collision verification against `input.fixtures` over all timeline motion spans (handling 3-axis XYZA chords and 4th-axis rotary transforms identically!), coalescing adjacent hits through `ViolationBatchCoalescer` into concise, clickable `SimDiagnostic` entries naming the exact fixture (`"Holder/collet collided with 'Front-Left Toe Clamp' by 4.25 mm"` or `"Cutter flutes cut into 'Vise Fixed Jaw' by 1.80 mm"`).
  - Because this check runs on the shared `SetupSimulationInput` chord stream in `driver.rs` for both `CpuExactBackend` and `WgpuComputeBackend` across `Heightfield`, `TriDexel`, and `Cylindrical`, all 3 engines × 2 backends produce **100% identical** `FixtureCollision` diagnostics with `< 1 ms` overhead when fixtures are present and `0.0 ms` when `fixtures.is_empty()`.
- **Export Gate & CLI Integration:**
  - `chipcutter build` and GUI G-code export automatically block when any `FixtureCollision` critical error is present (matching existing `RapidIntoStock` / `TargetModelGouge` / `HolderCollision` behavior).
  - `chipcutter simulate --json` includes `fixture_collision` diagnostics and colliding fixture IDs in its structured report.

---

### 3. Viewport Collision Highlighting & Inspector Feedback (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/panels/simulation/worker.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/ui.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/fixtures_tab.rs`
- Wire resolved `Setup` fixtures into `trigger_simulation` so GUI simulation runs Rule 6 automatically.
- Highlight any fixture referenced by an active `FixtureCollision` diagnostic in vivid crimson in the 3D viewport (and clicking the diagnostic in **Diagnostics & Log** jumps both the camera and timeline scrubber directly to the collision point!).
- Display an inline info banner in `[Fixtures]` summarizing automatic rapid lifts applied on the active Setup (e.g., *"Auto-Avoid Rapids enabled — 4 rapid jumps lifted cleanly over clamps"*), plus a toggle checkbox to test/disable auto-avoidance explicitly.

---

## Verification Plan

1. **Automated Rapid-Routing & Collision Tests (`chip-cam`, `chip-sim`):**
   - **Clamp-in-Rapid-Path Test:** Place a `ToeClamp` (with stud tower) directly between two pockets on `cam-torture` -> with `auto_avoid_fixtures_on_rapids = false`, `chip-sim` reports `SimIssueKind::FixtureCollision` (`CriticalError`) on both CPU and GPU across Heightfield and Tri-Dexel; with `auto_avoid_fixtures_on_rapids = true`, `route_toolpath_around_fixtures` lifts the connecting rapid cleanly over the stud and `chip-sim` reports **zero critical errors**!
   - **Holder-vs-Jaw Collision Test:** Place a tall `ViseJaws` fixture close to a deep pocket wall so the flute clears the jaw but the wider collet/holder hits it -> verify `SimIssueKind::FixtureCollision` catches the holder contact with accurate `penetration_mm` and `location_wcs`.
   - **Clamp-in-Cut-Path Safety Test:** Place a clamp across an actual cutting pass -> verify `route_toolpath_around_fixtures` does **not** alter cutting moves and `chip-sim` blocks export with `FixtureCollision`.
   - **Spoilboard Overcut Tolerance Test:** Verify `0.2 mm` through-cut into a spoilboard with `allowed_overcut = 0.3 mm` passes cleanly, while `1.0 mm` plunge into the spoilboard triggers `FixtureCollision`.
   - **CPU vs. GPU Parity Test:** Assert exact `FixtureCollision` diagnostic parity across `CpuExactBackend` and `WgpuComputeBackend` on Heightfield, Tri-Dexel, and Cylindrical engines.
2. **Headless Vulkan Visual Verification:**
   - Capture before/after screenshots showing (a) lifted yellow `G0` rapid arc jumping cleanly over red `ToeClamp`s, and (b) deliberate crimson-highlighted clamp collision diagnostic jump.
3. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`, `cargo check --locked -p chip-stock`, `cargo deny check` (default + `flatpak-portal`), `cargo clippy` (default + `gpu-compute` + `flatpak-portal`), and `WGPU_BACKEND=vulkan cargo test --locked --workspace` (100% Phase 1–2 golden G-code parity).
