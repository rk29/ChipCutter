# Milestone 46 Implementation Plan — Thread Mill Simulation (Tri-Dexel Undercuts, 3D Cutter Mesh) & Desktop GUI

## Goal

Bring **Helical Thread Milling** to life visually and physically in both the 3D simulation engine (`chip-sim`) and the desktop CAM interface (`chip-gui`): render an accurate relieved-neck 3D thread mill cutter, carve true 3D helical undercut thread grooves in Tri-Dexel volumetric simulation, handle relieved-neck clearance accurately in the 5-rule verifier, and expose full Tool Library + two-column Operation Inspector authoring with universal hover tooltips.

---

## Proposed Changes

### 1. Revolved 3D `ThreadMill` Cutter Profile & Undercut Simulation (`chip-sim`)

#### [MODIFY] [crates/chip-sim/src/cutter_profile.rs](file:///workspace/ChipCutter/crates/chip-sim/src/cutter_profile.rs)
#### [MODIFY] [crates/chip-sim/src/tridexel/subtract.rs](file:///workspace/ChipCutter/crates/chip-sim/src/tridexel/subtract.rs)
#### [MODIFY] [crates/chip-sim/src/heightmap/mod.rs](file:///workspace/ChipCutter/crates/chip-sim/src/heightmap/mod.rs)
#### [MODIFY] [crates/chip-sim/src/verifier.rs](file:///workspace/ChipCutter/crates/chip-sim/src/verifier.rs)
- **3D Cutter Assembly Mesh & Radial Envelope (`cutter_profile.rs`):**
  - Add `CutterGeometry::ThreadMill` support to `build_cutter_assembly_mesh`:
    - Cutting band (`z in [0, flute_length]`): crisp `60°` V-tooth ring profile(s) (1 tooth for single-point, `teeth_count` periodic V-teeth spaced by `pitch_mm` for multi-form) extending out to `cutting_diameter_mm / 2`.
    - Relieved neck (`z in [flute_length, neck_length]`): slim cylindrical neck at `neck_diameter_mm / 2` so internal thread clearance is visible in 3D playback!
    - Shank taper + cylindrical shank (`z in [neck_length, stickout]`) + collet/holder above `stickout`.
  - Add height-dependent radius lookup `cutter_radius_at_z(z_local)` supporting non-monotonic undercut profiles (`R_tooth` in cutting band, `R_neck < R_tooth` along relieved neck, `R_shank` above `neck_length`).
- **True 3D Undercut Carving in Tri-Dexel (`tridexel/subtract.rs`):**
  - Because `TriDexelStock` stores multi-interval spans along `X`, `Y`, and `Z` rays, sweeping the cutting-tooth band `[z_tip, z_tip + flute_length]` radially at `R_tooth` (while only sweeping `R_neck` along the relieved neck above it) naturally carves **actual 3D helical undercut thread grooves** inside pre-drilled bores and around external bosses!
- **Safe Heightmap Mode & 5-Rule Verifier (`heightmap/mod.rs`, `verifier.rs`):**
  - In fast 2.5D Heightmap mode (which cannot represent vertical overhangs), sweep the effective thread cutting envelope cleanly without triggering false shank/flute-rub errors inside the pre-drilled pilot bore.
  - In the 5-rule verifier (both Heightmap and Tri-Dexel):
    - Check `G0` rapid collisions against stock, holder collisions, and non-cutting shank collisions above `neck_length_mm`.
    - Check relieved-neck clearance (`R_neck`) against remaining stock so plunging into solid un-drilled material (forgot pilot drill!) is caught as a critical crash, while threading cleanly inside a properly pre-drilled minor-diameter bore reports **0 crashes**!
    - Exclude intentional internal/external thread groove engagement on target B-Rep pilot cylinders from spurious gouge alarms (parallel to `intentional_edge_break_mm` for chamfers).

---

### 2. Tool Library & Operation Inspector UI (`chip-gui`)

#### [MODIFY] [crates/chip-gui/src/widgets/icons.rs](file:///workspace/ChipCutter/crates/chip-gui/src/widgets/icons.rs)
#### [MODIFY] [crates/chip-gui/src/panels/tool_library.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/tool_library.rs)
#### [MODIFY] [crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs)
#### [NEW] `crates/chip-gui/src/panels/operation_inspector/passes_tab/thread_mill.rs`
#### [MODIFY] [crates/chip-gui/src/panels/browser.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/browser.rs)
#### [MODIFY] [crates/chip-gui/src/app/menus_toolbar.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/menus_toolbar.rs)
#### [MODIFY] [crates/chip-gui/src/panels/command_palette.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/command_palette.rs)
- **Procedural Vector Icon (`icons.rs`):** Add `IconKind::ThreadMill` (clean crisp threaded-tooth helix glyph) for browser rows, menus, and command palette.
- **Tool Library Manager (`tool_library.rs`):**
  - Add `"Thread Mill"` to the cutter type selector with editable fields for Cutting Diameter, Neck Diameter, Neck Length (Reach), Pitch, Teeth Count (`1` = Single-Point, `>1` = Multi-Form), Thread Angle (`°`), Flute Length, Overall Length, Shank Diameter, and Flutes—every control with clear hover tooltips.
  - Extend `draw_cutter_schematic` to render the relieved neck and V-tooth thread mill silhouette cleanly.
- **Strict Two-Column Thread Mill Passes Inspector (`passes_tab/thread_mill.rs`):**
  - **Thread Specification Card:**
    - `Thread Form`: `Internal (Hole)` / `External (Boss)`
    - `Standard`: `ISO Metric Coarse` / `ISO Metric Fine` / `UNC` / `UNF` / `Custom`
    - `Thread Size`: searchable/truncated ComboBox (e.g., `M8 x 1.25`, `1/4-20 UNC`) that auto-populates `Major Ø`, `Pitch`, and displays recommended tap-drill diameter (`Ø6.80 mm` / `Ø5.11 mm`).
    - `Major Ø`, `Pitch`, `Handedness` (`Right-Hand` / `Left-Hand`), `Cut Direction` (`Climb` / `Conventional`) with a live direction summary badge (e.g., `Bottom-Up G3 (Climb)`).
  - **Helical Passes & Fit Tuning Card:**
    - `Radial Passes` (`1..=8`), `Spring Passes` (`0..=3`), `Pitch Ø Offset` (signed gauge-fit tweak), `Lead-In Clearance`, and `Feed Comp` (`Compensate Centerline Feed` checkbox with live effective programmed feed readout).
    - Inline validation banner warning immediately if no compatible `ThreadMill` is assigned, if the selected cutter is larger than the minor diameter, or if thread depth exceeds `neck_length_mm`.
- **Operation Creation Wiring:** Wire `+ Thread Mill` into `Operations` menu, CAM Browser `+ Op...` menu, `CommandPalette`, and `default_pick_mode_for_op` (picking cylindrical holes / circular loops automatically).

---

## Verification Plan

1. **Unit & Simulation Tests:**
   - `chip-sim`: Test `ThreadMill` 3D cutter mesh generation, Tri-Dexel internal helical groove undercut carving (confirming actual grooved profile & volume removal without false crashes), and fault-injection catching a thread mill plunge into un-drilled solid stock.
   - `chip-gui`: Confirm `ThreadMill` icon, command palette entry, and headless offscreen Vulkan rendering of a simulated `ThreadMill` cutter & stock.
2. **Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim -p chip-gui`
