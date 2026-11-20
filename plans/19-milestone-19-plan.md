# ChipCutter — Milestone 19 Implementation Plan

## Goal Description
Implement **Milestone 19 (4th-Axis Machine Kinematics, Rotary Stock/WCS, & Rotary CL-Data / Posts)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L123-L126)—establishing the complete 4th-axis kinematic, CL-IR, post-processing, polar stock simulation, and viewport foundation required for Milestones 20 (`Z+X`/`Z+Y` rotary wrapping, spiral surfacing, & `3+1` indexed machining) and 21 (simultaneous `X+Y+Z+Rotary` milling):

1. **Configurable 4th-Axis Kinematics & Rotary Setup Model (`chip-cam`, `chip-project`):**
   - Add Setup-level `RotaryConfig` supporting alignment along **`X` (`A`-axis)**, **`Y` (`B`-axis)**, or **`Z` (`C`-axis)**, configurable rotation polarity (`RightHandCcw` / `LeftHandCw`), rotary centerline offset, rotary clearance radius, drive mode (`Disabled3Axis`, `TwoLinearPlusRotary`, `Indexed3Plus1`, `Simultaneous4Axis`), unwind policy (`ContinuousMultiTurn`, `WrapShortestPath`, `Modulo0To360`), and optional swapped-axis mapping for 3-axis rotary controllers.
   - Implement pure `f64` kinematic & cylindrical coordinate transforms (`crates/chip-cam/src/rotary.rs`): exact SE(3) rotation about the configured rotary axis, `(x, y, z) <-> (u_axial, theta_deg, r_radial)` projection, exact `G93` inverse-time feed & `G94` angular feed calculations, and shortest-path / continuous multi-turn unwinding.
2. **Rotary `ClEvent` IR & 4-Axis Post-Processors (`chip-cam`, `chip-post`):**
   - Extend `ClEvent`, `ToolpathBuilder`, and `ToolpathSegment3D` with rotary angular coordinates (`RotaryIndex`, `RotaryRapidMove`, `RotaryLinearMove`, plus `#[serde(default)]` `start_rotary_deg` / `end_rotary_deg` on `ToolpathSegment3D`) without altering any existing 3-axis `ClEvent` variants or golden outputs.
   - Extend `PostProfile` with a `[rotary]` configuration block (`RotaryPostConfig`) supporting configurable axis letters (`A`/`B`/`C`), modal rotary tracking, automatic **`G93` inverse-time feedrate** emission on simultaneous linear+rotary blocks (with automatic `G94` restoration before pure linear/rapid/drill blocks), and **swapped-axis rotary wrapping** (`Y` or `X` driven from `theta_rad * wrap_radius_mm`).
   - Ship three built-in 4-axis `.post.toml` profiles: `linuxcnc_4axis`, `grblhal_4axis`, and `grbl_rotary_wrap`.
3. **Polar / Cylindrical Stock Simulation & 5-Rule Rotary Verifier (`chip-sim`):**
   - Implement a pure-Rust `f64` polar/cylindrical heightmap stock (`CylindricalStock`: `(u_axial, θ) -> r_max(u, θ)`) supporting both round bar stock and chucked rectangular/square bar stock along `X`, `Y`, or `Z`.
   - Sweep all cutter geometries (Flat, Ball, Bull-Nose, Chamfer/V-Bit, Drill) across interpolated rotary segments `(pos_wcs(t), rotary_deg(t))`, verify all 5 safety rules (`RapidIntoStock`, `TargetModelGouge` against a polar-rasterized STEP/mesh target surface, `NonCuttingFluteRub`, `HolderCollision`, `UnsafePlunge`), tessellate a watertight color-by-op 3D machined cylindrical stock mesh, and interpolate live `rotary_deg` in `SimulationTimeline` / `ScrubSnapshot`.
4. **Rotary Viewport Gizmo, Live Rotating Stock Playback, & Setup Rotary Inspector (`chip-gui`, `chip-cad`, `chip-project`):**
   - Render a 3D rotary axis centerline, rotation-direction arrow ring, rotary chuck collar silhouette, and rotary clearance cylinder wireframe in the `wgpu` viewport whenever rotary mode is active.
   - Rotate the 3D machined cylindrical stock mesh (and translucent target part) live around the configured rotary axis by `ScrubSnapshot::rotary_deg` during simulation playback while displaying the active rotary angle (`A=…°` / `B=…°` / `C=…°`) in the Simulation Timeline HUD.
   - Add a **`Rotary`** tab (`SetupTab::Rotary`) in the Setup Inspector with one-click cylindrical stock alignment and a built-in **`Rotary Indexed Shaft (STL Mesh)`** fixture/preview sample (`--sample rotary-shaft`) demonstrating the 4th-axis setup, polar stock simulation, and 4-axis G-code post-processing.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Zero Regression to 3-Axis Golden G-Code & Schema Compatibility:**
> - `Setup::rotary` uses `#[serde(default, skip_serializing_if = "RotaryConfig::is_disabled")]` and `PostProfile::rotary` uses `#[serde(default)]`, so every existing `schema_version = 1/2/3` `.chip.toml` file, `.post.toml` profile, and Phase 1 / Phase 2 / M18 v3 golden G-code snapshot stays **100% byte-for-byte identical**.
> - **Zero New Third-Party Dependencies:** All rotary kinematics, polar heightmap rasterization/tessellation, and 4-axis post-processing use existing workspace crates (`glam`, `rayon`, `serde`, `toml`) so `cargo deny check` remains unchanged.

> [!NOTE]
> **Clear Separation Between Milestone 19 and Milestones 20–21:**
> - **Milestone 19** delivers the complete 4th-axis **platform & verification infrastructure**: Setup kinematics (`A`/`B`/`C`), rotary `ClEvent` IR & `ToolpathBuilder`, `G93`/`G94`/swapped-axis post-processors, `CylindricalStock` polar simulator & 5-rule verifier, rotating 3D viewport gizmo/playback, and Setup Rotary Inspector UI.
> - **Milestone 20** builds the concrete `Z+X` / `Z+Y` cylindrical wrapping, continuous rotary spiral/axial surfacing, and `3+1` multi-face indexed operation strategies on top of M19's IR & simulator.
> - **Milestone 21** builds full simultaneous `X+Y+Z+Rotary` off-centerline/undercut/swarf milling, tutorials, and Phase 5 rotary golden suites.

> [!TIP]
> **Coordinate Convention for Polar Simulation & Viewport Playback:**
> - In machine coordinates, the spindle always points along `+Z_wcs` while the rotary chuck rotates the workpiece about the rotary axis by `theta_deg`.
> - Storing `CylindricalStock` in **workpiece-fixed cylindrical coordinates `(u_axial, θ_part)`** makes material removal trivial and exact: at machine rotary angle `theta_cmd`, a cutter at WCS `(x, y, z)` maps cleanly into part coordinates via `rotate_about_rotary_axis(pos_wcs, -theta_cmd)`, and rendering the machined stock in the 3D viewport simply rotates its mesh by `+theta_cmd` around the rotary axis!

---

## Proposed Changes

### 1. 4th-Axis Kinematics, Rotary Setup Model, & Rotary `ClEvent` IR (`chip-cam`)

#### [NEW] `crates/chip-cam/src/rotary.rs`
- Define core 4th-axis types (all `Serialize`/`Deserialize` with sensible defaults):
  - `RotaryAxisKind`: `XAxis` (default letter `'A'`), `YAxis` (default letter `'B'`), `ZAxis` (default letter `'C'`), with `axis_unit_vec() -> DVec3`, `default_letter() -> char`, `to_stock_axis() -> StockAxis`, `label()`.
  - `RotaryDirection`: `RightHandCcw` (standard right-hand rule around `+axis`), `LeftHandCw` (inverted polarity).
  - `RotaryDriveMode`:
    - `Disabled3Axis` (default — standard 3-axis Setup)
    - `TwoLinearPlusRotary` (centerline-locked `Z+X+A` or `Z+Y+B` mode)
    - `Indexed3Plus1` (positional multi-face indexing)
    - `Simultaneous4Axis` (full `X+Y+Z+Rotary`)
  - `RotaryUnwindMode`: `ContinuousMultiTurn`, `WrapShortestPath`, `Modulo0To360`.
  - `RotaryConfig`:
    - `mode: RotaryDriveMode`
    - `axis: RotaryAxisKind`
    - `direction: RotaryDirection`
    - `axis_letter_override: Option<char>` (defaults to `'A'`/`'B'`/`'C'` from `axis`)
    - `center_offset_wcs: [ParamLength; 2]` (perpendicular offset of the rotary spindle centerline in WCS; defaults to `[0.0, 0.0]`)
    - `rotary_clearance_radius: ParamLength` (safe radial clearance above stock outer radius for index rotations; defaults to `stock_radius + 10 mm` / `35.0 mm`)
    - `chuck_end_positive: bool` (`false` = chuck at `-u` end, tailstock at `+u` end)
    - `chuck_length: ParamLength` (`20.0 mm` visual & collision keep-out collar length)
    - `chuck_diameter: ParamLength` (`65.0 mm` visual & collision chuck outer diameter)
    - `unwind_mode: RotaryUnwindMode`
  - `ResolvedRotaryConfig`: resolved canonical `f64` axis unit vector, centerline point in WCS, active drive mode, polarity sign (`+1.0` / `-1.0`), clearance radius `mm`, chuck axial span `[u_chuck_min, u_chuck_max]`, and chuck radius `mm`.
- Implement pure `f64` kinematic & feed helpers:
  - `rotate_point_about_axis(pt_wcs, center_wcs, axis, angle_deg, polarity) -> DVec3`
  - `wcs_to_cylindrical(pt_wcs, rotary: &ResolvedRotaryConfig) -> (f64, f64, f64)` (`(u_axial, theta_rad, radius_mm)`)
  - `cylindrical_to_wcs(u_axial, theta_rad, radius_mm, rotary: &ResolvedRotaryConfig) -> DVec3`
  - `resolve_rotary_target_deg(current_deg, target_deg, unwind: RotaryUnwindMode) -> f64`
  - `compute_inverse_time_feed_g93(start_wcs, end_wcs, start_deg, end_deg, effective_radius_mm, target_surface_feed_mm_min) -> Option<f64>` (returns `F` in `1/min` per NIST RS274NGC / LinuxCNC / grblHAL spec, clamped to safe finite range)
  - `compute_rotary_angular_feed_deg_min(surface_feed_mm_min, effective_radius_mm, max_deg_min) -> f64`

#### [MODIFY] [crates/chip-cam/src/setup.rs](file:///workspace/ChipCutter/crates/chip-cam/src/setup.rs), [crates/chip-cam/src/cl_data.rs](file:///workspace/ChipCutter/crates/chip-cam/src/cl_data.rs), [crates/chip-cam/src/toolpath.rs](file:///workspace/ChipCutter/crates/chip-cam/src/toolpath.rs), [crates/chip-cam/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- Add `#[serde(default, skip_serializing_if = "RotaryConfig::is_disabled")] pub rotary: RotaryConfig` to `Setup`, and `pub rotary: Option<ResolvedRotaryConfig>` to `ResolvedSetup` (resolving rotary centerline, clearance radius, and exposing `rotary_radius` in `Setup::eval_context`).
- Add rotary `ClEvent` variants in [cl_data.rs](file:///workspace/ChipCutter/crates/chip-cam/src/cl_data.rs):
  - `RotaryIndex { angle_deg: f64, rapid: bool, feed_deg_min: Option<f64> }`
  - `RotaryRapidMove { end_wcs: DVec3, rotary_deg: f64 }`
  - `RotaryLinearMove { end_wcs: DVec3, rotary_deg: f64, feed_mm_min: f64, effective_radius_mm: f64, role: MotionRole }`
- Extend `ToolpathSegment3D` in [toolpath.rs](file:///workspace/ChipCutter/crates/chip-cam/src/toolpath.rs) with `#[serde(default)] pub start_rotary_deg: f64` and `#[serde(default)] pub end_rotary_deg: f64` (plus `sample_rotary_deg(t)` and rotary-aware `length_mm()`), and add `ToolpathBuilder::rotary_index_to`, `rotary_rapid_to`, `rotary_linear_to` (updating `current_rotary_deg` and cycle-time stats cleanly).

---

### 2. 4-Axis Post-Processors (`G93`, `G94`, & Swapped-Axis Wrapping) (`chip-post`)

#### [MODIFY] [crates/chip-post/src/profile.rs](file:///workspace/ChipCutter/crates/chip-post/src/profile.rs), [crates/chip-post/src/modal.rs](file:///workspace/ChipCutter/crates/chip-post/src/modal.rs), [crates/chip-post/src/emitter.rs](file:///workspace/ChipCutter/crates/chip-post/src/emitter.rs), [crates/chip-post/src/partition.rs](file:///workspace/ChipCutter/crates/chip-post/src/partition.rs), [crates/chip-post/src/lib.rs](file:///workspace/ChipCutter/crates/chip-post/src/lib.rs)
- Add `RotaryPostConfig` & `SwapAxisTarget` (`None`, `ReplaceY`, `ReplaceX`) to `PostProfile`:
  - `enabled: bool` (`false` default on existing 3-axis profiles)
  - `axis_letter: char` (`'A'` default, overridden by Setup's `RotaryConfig` axis letter when not swapped)
  - `angle_decimals: usize` (`3` default)
  - `supports_g93_inverse_time: bool`
  - `use_g93_for_simultaneous: bool`
  - `max_rotary_feed_deg_min: f64` (`3600.0` default)
  - `unwind_mode: RotaryUnwindMode`
  - `swap_linear_axis: Option<SwapAxisTarget>` (converts rotary angle `theta_deg` into linear mm `theta_rad * wrap_radius_mm` on `Y` or `X` for 3-axis controllers wired with a swapped rotary stepper!)
- Pass optional `rotary: Option<&ResolvedRotaryConfig>` in `SetupPostInput` (defaulting to `None` in existing 3-axis callers).
- Extend `ModalState` in [modal.rs](file:///workspace/ChipCutter/crates/chip-post/src/modal.rs) with `rotary_word: Option<String>` and `feed_mode_g93: bool`.
- Handle `ClEvent::RotaryIndex`, `ClEvent::RotaryRapidMove`, and `ClEvent::RotaryLinearMove` in [emitter.rs](file:///workspace/ChipCutter/crates/chip-post/src/emitter.rs):
  - Emit `G93` + per-block `F<1/min>` on simultaneous linear+rotary moves when `use_g93_for_simultaneous == true`, and automatically restore `G94` before pure linear, arc, or drill blocks.
  - Emit pure angular `G0 A...` / `G1 A... F<deg/min>` on `RotaryIndex`.
  - When `swap_linear_axis` is configured (e.g., `grbl_rotary_wrap`), substitute the swapped linear axis word cleanly and emit a setup-header warning noting the active wrap radius!
  - Emit rotary metadata (`(Rotary: A-Axis around X, Centerline, Clearance Radius)`) in the structured Setup-Sheet G-Code header ONLY when rotary is enabled (leaving all existing 3-axis headers untouched).

#### [NEW] Built-in 4-Axis Post Profiles in `posts/`
- `posts/linuxcnc_4axis.post.toml` (`id = "linuxcnc_4axis"`, `G93` inverse-time enabled, `M6` ATC, `G64` path blending, `A`/`B`/`C` axis support)
- `posts/grblhal_4axis.post.toml` (`id = "grblhal_4axis"`, `G93` inverse-time enabled, `M6` ATC, 4-axis grblHAL dialect)
- `posts/grbl_rotary_wrap.post.toml` (`id = "grbl_rotary_wrap"`, `SplitFilesByTool`, `swap_linear_axis = "replace_y"`, `G94` linear feed for 3-axis GRBL controllers with Y-axis swapped to rotary)
- Register all three in `builtin_post_profiles()` in [crates/chip-post/src/profile.rs](file:///workspace/ChipCutter/crates/chip-post/src/profile.rs).

---

### 3. Polar / Cylindrical Stock Simulation & 5-Rule Rotary Verifier (`chip-sim`)

#### [NEW] `crates/chip-sim/src/cylindrical.rs`
- **`CylindricalStock`:**
  - Polar grid over `nu × ntheta` cells (`SimResolution::Draft` → `180 × 180`, `Standard` → `320 × 360`, `High` → `512 × 540`) storing `r_max: Vec<f32>` (current machined stock radius from rotary centerline) and `last_op_idx: Vec<u16>`.
  - Initializes accurately from `ResolvedStockGeometry::Cylinder` (`r = radius_mm` everywhere) OR `ResolvedStockGeometry::Box` (exact ray-to-box radius `r(θ) = min(hx / |cos θ|, hy / |sin θ|)` for square/rectangular bar chucked in a 4-jaw rotary chuck!).
  - **`rasterize_cylindrical_target_surface`:** Samples the target STEP B-Rep or STL/OBJ/3MF mesh (`CadDocument`) into a polar target radius envelope `target_r: Vec<f32>` using outward/inward ray-triangle intersection (via `MeshBvh`) so `TargetModelGouge` verification works accurately all the way around `0°..360°`!
  - **Analytical Rotary Cutter Sweeper (`sweep_rotary_segment`):**
    - Subdivides each `ToolpathSegment3D` adaptively in both linear distance (`<= 0.5 * du`) and angular sweep (`<= 0.5 * dtheta`).
    - At each substep `t`, transforms tool tip `pos_wcs(t)` and spindle axis `+Z_wcs` into **part-fixed coordinates** by rotating `-rotary_deg(t)` about the rotary axis, then checks every polar cell `(iu, itheta)` within the cutter's radial/axial footprint against `RadialCutterProfile`:
      - Detects **`RapidIntoStock`** if `is_rapid` and tool penetrates `r_max(iu, itheta)` or chuck keep-out zone.
      - Detects **`TargetModelGouge`** if cutting tip dips below `target_r(iu, itheta) - gouge_tol_mm`.
      - Detects **`NonCuttingFluteRub`** and **`HolderCollision`** if shank/holder radius penetrates remaining stock `r_max` or chuck body.
      - Carves `r_max(iu, itheta)` down to the analytical tool-contact radius and stamps `last_op_idx`.
  - **`tessellate_cylindrical_stock_mesh(&CylindricalStock, &ResolvedRotaryConfig, palette) -> SimStockMesh`:**
    - Generates a smooth, watertight 3D cylindrical surface mesh (wrapping seamlessly across `θ = 2π → 0` plus triangulated `-u` and `+u` end caps) in part-fixed WCS coordinates with smooth averaged normals and per-op vertex coloring.
  - **`run_cylindrical_setup_simulation`:** Orchestrates checkpoints, timeline, volume removal calculation (`∑ 0.5 * (r_init² - r_final²) * du * dtheta`), and returns the exact same `SetupSimulationResult` struct used by 3-axis simulation!

#### [MODIFY] [crates/chip-sim/src/heightmap.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap.rs), [crates/chip-sim/src/timeline.rs](file:///workspace/ChipCutter/crates/chip-sim/src/timeline.rs), [crates/chip-sim/src/lib.rs](file:///workspace/ChipCutter/crates/chip-sim/src/lib.rs)
- Add `rotary_deg: f64` and `max_rotary_speed_deg_s: f64` / `rotary_accel_deg_s2: f64` to `ScrubSnapshot`, `MotionSpan`, and `MachineKinematicsConfig` in [timeline.rs](file:///workspace/ChipCutter/crates/chip-sim/src/timeline.rs) so scrubbing & cycle-time estimation smoothly interpolate rotary angle and account for rotary slewing time.
- Dispatch `run_setup_simulation` to `cylindrical::run_cylindrical_setup_simulation` automatically when `input.setup.rotary.is_some()` (and include `rotary_axis: Option<ResolvedRotaryConfig>` metadata on `SetupSimulationResult` / `SimStockMesh` or reuse `setup.rotary` for viewport rotation).

---

### 4. Rotary Viewport Gizmo, Animated Rotating Stock, & Setup Rotary Inspector (`chip-cad`, `chip-project`, `chip-gui`)

#### [MODIFY] [crates/chip-cad/src/mesh/fixtures.rs](file:///workspace/ChipCutter/crates/chip-cad/src/mesh/fixtures.rs) & [crates/chip-cad/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cad/src/lib.rs)
- Add `sample_rotary_shaft_stl_bytes()` — a clean watertight `Ø40 mm × 80 mm` cylindrical rotary shaft aligned along the **X (`A`) axis** featuring 4 machined indexed flats (`90°` apart) and a turned central collar groove, used for M19 verification, polar simulation tests, and the Rotary preview sample!

#### [MODIFY] [crates/chip-project/src/pipeline.rs](file:///workspace/ChipCutter/crates/chip-project/src/pipeline.rs), [crates/chip-project/src/validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs), [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs), [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Forward `resolved_setup.rotary.as_ref()` into `SetupPostRequest` / `SetupPostInput` in [pipeline.rs](file:///workspace/ChipCutter/crates/chip-project/src/pipeline.rs).
- Validate rotary configuration invariants in [validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs) (e.g., warn if a Setup enables 4th-axis rotary while using a 3-axis post profile like `grbl` without rotary support; error if `rotary_clearance_radius` is smaller than stock outer radius).
- Add `build_rotary_shaft_preview_project` in [demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs) configured with `X (A-Axis)` rotary kinematics, aligned `Ø44 × 84 mm` cylindrical bar stock, `linuxcnc_4axis` post profile, and a clean initial setup ready for Milestones 20–21 strategies.

#### [MODIFY] `crates/chip-gui`
- **Setup Inspector (`SetupTab::Rotary`)** in [panels/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/mod.rs) and [panels/setup_inspector.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/setup_inspector.rs):
  - Add `Rotary` tab alongside `WCS`, `Stock`, `Post`, `Heights` (captured in `SetupParamsSnapshot` for zero-cost undo/redo).
  - Two-column controls for Drive Mode, Rotary Axis (`X / A`, `Y / B`, `Z / C`), Rotation Polarity, Axis Letter override, Centerline Offset `(d1, d2)`, Rotary Clearance Radius, Unwind Mode, Chuck End / Diameter / Length, and a one-click **"Align Cylindrical Stock to Rotary Axis"** button.
- **3D Viewport Rotary Gizmo & Live Rotating Playback** in [viewport/overlays.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/overlays.rs), [viewport/scene_builders.rs](file:///workspace/ChipCutter/crates/chip-gui/src/viewport/scene_builders.rs), [app/viewport_sync.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/viewport_sync.rs), and [panels/sim_timeline.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/sim_timeline.rs):
  - Draw dashed rotary centerline axis, directional rotation arc arrow ring, 3-jaw/collar rotary chuck silhouette at the chuck end, and translucent rotary clearance cylinder wireframe.
  - When scrubbing a rotary simulation, transform the rendered `SimStockMesh` (and ghost part) by `rotary_deg` around the active rotary axis and display live `A=…°` / `B=…°` / `C=…°` readout in the Simulation Timeline HUD!
- **Sample Gallery & `--sample rotary-shaft`** in [panels/welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs), [app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs), [app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs), [main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs).

---

## Verification Plan

1. **Automated Unit & Physics Oracle Tests (`chip-cam`, `chip-post`, `chip-sim`, `chip-project`, `chip-cli`):**
   - `chip-cam::rotary::tests`:
     - Exact round-trip `(x, y, z) <-> (u, θ, r)` across all three axes (`X/A`, `Y/B`, `Z/C`), both polarities (`CCW`/`CW`), and non-zero centerline offsets (`< 1e-9 mm`).
     - Exact `G93` inverse-time feed (`1 / dt_min`) and `G94` angular feed math + `ContinuousMultiTurn`, `WrapShortestPath`, and `Modulo0To360` unwinding oracles.
   - `chip-post` tests:
     - Verify `linuxcnc_4axis` & `grblhal_4axis` emit valid `A`/`B`/`C` words, switch cleanly into `G93` with exact per-block inverse-time `F` on simultaneous linear+rotary moves, and restore `G94` immediately before pure linear/rapid/drill moves.
     - Verify `grbl_rotary_wrap` accurately swaps `A` angular commands into `Y` linear mm (`theta_rad * wrap_radius_mm`).
     - Verify **100% byte-for-byte parity** of all existing Phase 1, Phase 2, and M18 v3 3-axis golden snapshots!
   - `chip-sim::cylindrical::tests`:
     - Analytical cylindrical turning/grooving volume removal oracle (turning a `Ø44 mm` bar down to `Ø36 mm` over `20 mm` axial width removes `π * (22² - 18²) * 20 ≈ 10,053 mm³` within `< 2%`).
     - Chucked square bar (`30 × 30 mm`) polar initialization oracle.
     - **5-Rule Fault-Injection Oracle on `CylindricalStock`:** Inject (a) rapid crash into cylindrical bar, (b) target-model gouge into `sample_rotary_shaft_stl_bytes()`, and (c) chuck/holder collision, proving all are deterministically detected, while a clean rotary pass verifies with `0` critical errors!
   - `chip-project::tests`:
     - Deterministic Schema v3 `.chip.toml` round-trip of `RotaryConfig` (and zero-diff omission when disabled), validation warnings for mismatched 3-axis post on a 4-axis setup, and `build_rotary_shaft_preview_project` end-to-end load + validate.
2. **Full Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
3. **Headless Vulkan Visual Verification (`xvfb-run` + Lavapipe):**
   - Capture and visually inspect `--sample rotary-shaft` in Studio Dark & Daylight Light themes with `Setup -> Rotary` tab focused, verifying the 3D rotary axis centerline, rotation-direction ring, chuck collar gizmo, rotary clearance cylinder, and clean Inspector cards via `read_file`.
