# Milestone 64 Implementation Plan — Undercut Cutter Geometries (`Lollipop`, `Slot / T-Slot`, `Dovetail`), Tri-Dexel Undercut Simulation (CPU + GPU), & Tool Library UI

## Goal

Extend ChipCutter's cutter catalog, shared `AnalyticCutterEnvelope`, CPU & GPU volumetric simulation kernels (`TriDexelStock`, `HeightfieldStock`, `CylindricalStock`), 3D revolved cutter meshes, and Tool Library Manager UI with three first-class relieved-neck undercut cutter geometries:
1. **`LollipopEndmill`** (spherical head + relieved cylindrical neck) — for 3D underside/back-chamfer deburring and concave 3D undercut milling,
2. **`SlotMill`** (horizontal Woodruff / keyway / T-slot cutter disc + corner radius + relieved neck) — for keyways and T-slots, and
3. **`DovetailCutter`** (inverted tapered frustum + corner radius + relieved neck) — for angled dovetail rails and slots.

---

## Non-Negotiable Invariants

- **100% Backward Compatibility:** Existing `schema_version = 1/2/3` `.chip.toml` projects, `.machine.toml` libraries, and Phase 1–5 golden G-code snapshots remain byte-for-byte identical. New starter library entries use non-colliding tool IDs/numbers appended cleanly without altering existing `T1..T12` indices relied upon by unit/golden tests.
- **Zero New External Dependencies & Pure-CPU Default:** `chip-stock` and `chip-cam` default builds stay 100% pure-CPU (zero `wgpu`), and `GpuToolProfile` keeps its exact **64-byte (`4 × 16B`)** `#[repr(C)] bytemuck::Pod` WGSL layout.
- **Exact Non-Monotonic Undercut Physics:** Both CPU (`tridexel/sweep.rs`) and GPU (`tridexel_sweep.wgsl`) must carve exact overhanging material cavities while preserving the overhanging roof above the cutter's upper head and detecting relieved-neck / non-cutting shank / holder collisions identically!
- **No Unchecked Math or Panics:** `#![deny(unsafe_code)]`, zero `unwrap()`/`expect()` outside tests, strict validation on all neck/angle/thickness parameters.

---

## Proposed Changes

### 1. Three New `CutterGeometry` Variants & Exact `AnalyticCutterEnvelope` (`chip-tools`)

#### [MODIFY] `crates/chip-tools/src/model.rs`
#### [MODIFY] `crates/chip-tools/src/envelope.rs`
#### [MODIFY] `crates/chip-tools/src/fusion_import.rs`
#### [MODIFY] `crates/chip-tools/src/starter.rs`
- **New `CutterGeometry` Variants (`model.rs`):**
  1. `#[serde(rename = "lollipop_endmill")] LollipopEndmill`:
     - `diameter_mm: f64` (spherical head diameter `D`), `neck_diameter_mm: f64` (`< D`), `neck_length_mm: f64` (total reach from tip to shank taper), `flute_length_mm: f64` (defaults to sphere height up to neck join `R + sqrt(R² - r_neck²)`), `overall_length_mm: f64`, `shank_diameter_mm: f64`, `flutes: u32`.
  2. `#[serde(rename = "slot_mill")] SlotMill`:
     - `diameter_mm: f64` (disc OD `D`), `cutting_thickness_mm: f64` (axial kerf/disc height `H_cut`), `corner_radius_mm: f64` (`<= 0.5 * H_cut`), `neck_diameter_mm: f64` (`< D`), `neck_length_mm: f64` (`>= H_cut`), `overall_length_mm: f64`, `shank_diameter_mm: f64`, `flutes: u32`.
  3. `#[serde(rename = "dovetail_cutter")] DovetailCutter`:
     - `diameter_mm: f64` (major bottom diameter `D_bot`), `side_angle_deg: f64` (flank angle from horizontal floor, e.g. `45.0°` or `60.0°`, clamped `(15.0, 85.0)`), `cutting_height_mm: f64` (axial flute height `H_cut`), `corner_radius_mm: f64`, `neck_diameter_mm: f64` (`<= D_top`), `neck_length_mm: f64` (`>= H_cut`), `overall_length_mm: f64`, `shank_diameter_mm: f64`, `flutes: u32`.
- **Operation Classes (`ToolOperationClass`):**
  - Add `Deburr3D`, `SlotUndercut`, `Dovetail`, `Undercut3D` and wire `supports_operation`:
    - `Deburr3D`: `ChamferMill`, `BallEndmill`, `LollipopEndmill`
    - `SlotUndercut`: `SlotMill` (plus `FlatEndmill` for open keyways)
    - `Dovetail`: `DovetailCutter`
    - `Undercut3D`: `LollipopEndmill`
- **Exact Non-Monotonic Envelope Math (`envelope.rs`):**
  - Add `CutterEnvelopeShape::{Lollipop, SlotMill, Dovetail}` and helper `has_relieved_neck(&self) -> bool` (true for `ThreadMill`, `Lollipop`, `SlotMill`, `Dovetail`).
  - Provide closed-form `f64` queries:
    - `cutting_radius_at_dz(dz)` & `assembly_radius_at_dz(dz)` (exact radial profile at height `dz` above tip across cutting head, relieved neck `[flute_top, neck_length]`, and shank above `neck_length`).
    - `cutting_z_interval_at_radius(r) -> Option<(f64, f64)>`:
      - **`Lollipop(R, r_neck)`**: for `r <= R`, `dz_lo = R - sqrt(R² - r²)`, `dz_hi = R + sqrt(R² - r²)` (and for `r <= r_neck`, upper join caps at `z_join = R + sqrt(R² - r_neck²)`).
      - **`SlotMill(R, H, rc, r_neck)`**: for `r <= R`, bottom corner fillet `dz_lo(r)` and top corner fillet `dz_hi(r) = H - dz_lo(r)` (symmetric flat disc with optional `rc` top/bottom corner radii!).
      - **`Dovetail(R_bot, H, tan_alpha, rc, r_neck)`**: for `r <= R_bot`, `dz_lo = corner_fillet(r)`, `dz_hi = ((R_bot - r) * tan_alpha).clamp(0.0, H)` (exact inverted cone tapering inward upward!).
    - Exact horizontal Minkowski dilation (`with_stock_allowance`) and validation (`validate()`).
- **Fusion 360 Import & Starter Presets (`fusion_import.rs`, `starter.rs`):**
  - Map Fusion 360 `"lollipop mill"`, `"slot mill"`, and `"dovetail mill"` JSON types cleanly in `fusion_import.rs`.
  - Add starter helper constructors (`starter_lollipop_6mm`, `starter_tslot_12mm`, `starter_dovetail_60deg_12mm`) and include them in `starter_undercut_tools()` (keeping `default_starter_tool_library()` indices `0..12` strictly unchanged so existing tests/goldens remain untouched!).

---

### 2. CPU & GPU Undercut Stock Sweeping, Relieved-Neck Verification, & 3D Cutter Meshes (`chip-stock`, `chip-sim`)

#### [MODIFY] `crates/chip-stock/src/types.rs`
#### [MODIFY] `crates/chip-stock/src/tridexel/sweep.rs`
#### [MODIFY] `crates/chip-stock/src/heightfield/sweep.rs`
#### [MODIFY] `crates/chip-stock/src/cylindrical/sweep.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/types.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/tridexel_sweep.wgsl`
#### [MODIFY] `crates/chip-stock/src/gpu/heightfield_sweep.wgsl`
#### [MODIFY] `crates/chip-stock/src/gpu/cylindrical_sweep.wgsl`
#### [MODIFY] `crates/chip-sim/src/cutter_profile.rs`
- **Unified Relieved-Neck Profile (`RadialCutterProfile` & `GpuToolProfile`):**
  - Generalize `is_thread_mill` checks to `has_relieved_neck` (retaining `is_thread_mill` specifically for V-tooth multi-band spacing).
  - Add `SHAPE_LOLLIPOP = 6`, `SHAPE_SLOTMILL = 7`, `SHAPE_DOVETAIL = 8` and `PROF_FLAG_RELIEVED_NECK = 1 << 3` in `gpu/types.rs` (exact same 64-byte `GpuToolProfile` layout!).
    - `Lollipop`: `param0 = z_neck_join`
    - `SlotMill`: `param0 = cutting_thickness_mm`, `param1 = corner_radius_mm`, `param2 = flat_radius_mm`
    - `Dovetail`: `param0 = cutting_height_mm`, `param1 = tan_side_angle`, `param2 = corner_radius_mm`, `param3 = top_radius_mm`
- **Exact CPU + GPU Tri-Dexel Undercut Subtraction & Overhang Verification:**
  - In both `tridexel/sweep.rs` and `tridexel_sweep.wgsl`:
    - For `Z`-rays: subtract the exact bounded vertical interval `[z_tip + dz_lo(r), z_tip + dz_hi(r)]` — carving true T-slot cavities, `60°` dovetail angled walls, and spherical lollipop recesses while leaving the solid ceiling above `z_tip + dz_hi(r)` completely intact!
    - For `X`/`Y`-rays: evaluate exact horizontal slice radius `cutting_radius_at_dz(z_ray - z_tip)` over `[0, flute_top]` so vertical/angled undercut side walls are razor-sharp on all 3 orthogonal dexel planes!
    - For relieved-neck verification: check material contact against the non-cutting neck (`r <= neck_radius_mm` for `dz ∈ (flute_top, neck_length]`) and full shank (`r <= shank_radius_mm` for `dz ∈ (neck_length, stickout_length]`) — flagging `FluteLengthExceeded` (neck/shank rub!) if the cutter plunges vertically into an un-cleared roof or rams the neck into an overhanging lip!
  - In `HeightfieldStock` & `CylindricalStock` (CPU & WGSL): safely evaluate the lower envelope `dz_lo(r)` for downward cuts while reporting clean diagnostics (and surfacing a GUI advisory when undercut cutters are simulated in 2.5D Heightmap mode recommending `Tri-Dexel`).
- **3D Revolved Cutter Assembly Meshes (`cutter_profile.rs`):**
  - Generate crisp revolved meshes in `build_cutter_assembly_mesh` for:
    - `LollipopEndmill`: smooth 16-station spherical ball head up to `z_neck_join` + relieved neck + taper + shank + holder,
    - `SlotMill`: flat-bottom/filleted horizontal disc + top shoulder step-in to relieved neck + shank + holder,
    - `DovetailCutter`: flat/filleted bottom + exact inverted conical flank tapering inward to relieved neck + shank + holder!

---

### 3. Tool Library Manager UI & 2D Cutter Schematics (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/panels/tool_library/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/tool_library/editor.rs`
#### [MODIFY] `crates/chip-gui/src/panels/tool_library/schematic.rs`
- Add `Lollipop Endmill`, `Slot / T-Slot Mill`, and `Dovetail Cutter` options to the Tool Library cutter-type dropdown and `+ New Tool` presets.
- Add strict two-column parameter controls with universal hover tooltips (`Cutting Ø`, `Neck Ø`, `Neck Length`, `Cutting Thickness` / `Corner Radius` for SlotMill, `Side Angle (°)` / `Cutting Height` for Dovetail, live derived `Top Ø` readout for Dovetail).
- Render crisp procedural 2D side-elevation schematics in `schematic.rs` clearly illustrating the spherical lollipop head, horizontal T-slot disc, inverted dovetail cone, and relieved neck.

---

## Verification Plan

1. **Analytical Envelope & Validation Tests (`chip-tools`):**
   - Exact `f64` closed-form assertions for `LollipopEndmill`, `SlotMill`, and `DovetailCutter` across `cutting_radius_at_dz`, `cutting_z_interval_at_radius`, `with_stock_allowance`, invalid-geometry rejection (`neck_diameter >= diameter`, bad dovetail angle), TOML roundtrip, and Fusion 360 JSON import.
2. **CPU vs. GPU Tri-Dexel Undercut & Fault-Injection Parity Tests (`chip-stock`, `chip-sim`):**
   - Carve a horizontal **T-Slot** with `SlotMill`, a **`60°` Dovetail** slot with `DovetailCutter`, and a concave **Spherical Undercut Groove** with `LollipopEndmill` into a pre-slotted block:
     - Assert exact overhang roof preservation (top Z-dexel span remains intact above the undercut cavity!), exact angled dovetail cross-section width vs. depth, exact removed volume within `< 1.5%`, and 100% CPU-vs-GPU parity across `CpuExactBackend` and `WgpuComputeBackend`!
   - Fault-injection tests proving (a) plunging a `SlotMill` / `DovetailCutter` vertically through uncut solid triggers neck/flute collision, and (b) crashing the relieved neck into an overhanging lip triggers a violation on both CPU and GPU.
   - Verify 3D revolved cutter meshes (`build_cutter_assembly_mesh`) produce valid non-empty outward-oriented geometry for all 10 cutter types.
3. **Headless Vulkan Visual Verification:**
   - Capture Tool Library modal screenshots showing the 2D schematics for Lollipop, Slot/T-Slot, and Dovetail cutters + a Tri-Dexel undercut groove screenshot, inspect via `read_file`, and clean up `/tmp/m64_*` temp files.
4. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`, pure-CPU `cargo check --locked -p chip-stock` & `-p chip-cam`, `cargo deny check` (default + `flatpak-portal`), `cargo clippy` (all targets & feature sets), `WGPU_BACKEND=vulkan cargo test --locked --workspace`, `chip-stock --features gpu-compute`, `chip-sim --features gpu-compute`.
