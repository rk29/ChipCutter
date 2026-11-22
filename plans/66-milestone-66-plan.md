# Milestone 66 Implementation Plan — Undercut Milling Strategies: Keyways, T-Slots, Dovetails, & Lollipop 3D Undercuts

## Goal

Deliver three dedicated, shop-floor-safe undercut machining strategies backed by Milestone 64's relieved-neck cutter geometries and CPU/GPU Tri-Dexel undercut simulation:
1. **`OperationKind::SlotUndercut` (Keyway & T-Slot Milling):** Precision keyways and T-slot undercuts with safe open-end / entry-bore lead-in, progressive multi-pass radial & axial stepping, standard metric/imperial presets, and guaranteed lateral exit before any `+Z` retract.
2. **`OperationKind::Dovetail` (Dovetail Slot & Rail Milling):** Angled dovetail slots and external dovetail guideways with strict throat-clearance guards, progressive lateral roughing stepovers + spring finish pass, and safe throat-centered retracts.
3. **`OperationKind::Undercut3D` (Lollipop 3D Undercut Finishing):** Horizontal Z-level contour finishing across overhanging downward-facing 3D B-Rep/mesh surfaces (`n_z < 0`) using exact upper-hemisphere contact and automatic relieved-neck ceiling-clearance clipping.

---

## Non-Negotiable Physical Machine Safety Invariants

- **NEVER Vertically Plunge or Retract Through an Undercut Roof:**
  - Every `SlotUndercut` and `Dovetail` pass MUST enter from an explicit **open slot end** (beyond stock/part boundary by `entry_clearance_mm`) OR a verified **pre-cleared entry bore / pocket**, feed horizontally into the undercut at cutting depth, and retract laterally back to the throat centerline / open entry zone **BEFORE** any `+Z` retract!
  - If an undercut geometry has no valid open end and no entry bore clearance (`entry_dia < cutter_dia + 0.5 mm`), return a clear `CamError` rather than emitting a catastrophic plunge/retract!
- **Strict Relieved-Neck & Throat Clearance Validation:**
  - Reject any `SlotUndercut` or `Dovetail` setup where `neck_diameter_mm >= throat_width_mm`, `neck_length_mm < total_slot_depth_mm`, or `DovetailCutter.side_angle_deg` mismatches a selected modeled dovetail wall angle beyond tolerance!
- **100% Backward Compatibility & Pure-CPU Core:** Additive `OperationKind` variants only; zero change to existing projects or golden G-code snapshots.

---

## Proposed Changes

### 1. New Operation Kinds & Parameter Schemas (`chip-cam`)

#### [MODIFY] `crates/chip-cam/src/operation.rs`
#### [NEW] `crates/chip-cam/src/params/undercut.rs`
#### [MODIFY] `crates/chip-cam/src/params/mod.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- Add 3 variants to `OperationKind`:
  1. `#[serde(rename = "slot_undercut")] SlotUndercut` (`label = "Slot / Keyway / T-Slot"`, `default_name = "T-Slot / Keyway"`)
  2. `#[serde(rename = "dovetail")] Dovetail` (`label = "Dovetail Slot / Rail"`, `default_name = "Dovetail"`)
  3. `#[serde(rename = "undercut_3d")] Undercut3D` (`label = "3D Lollipop Undercut"`, `default_name = "3D Undercut"`, `is_3d_surface = true`)
- Define `SlotUndercutParams`:
  - `mode: SlotUndercutMode` (`TSlot` | `Keyway` | `SideGroove`)
  - `preset: SlotUndercutPreset` (`Custom`, `TSlotM6`, `TSlotM8`, `TSlotM10`, `TSlotQuarterIn`, `TSlotThreeEighthIn`, `KeywayDin6885_4mm`, `KeywayDin6885_6mm`, `KeywayDin6885_8mm`)
  - `slot_width: ParamLength` (total undercut head width, e.g. `14.5 mm` for M8 T-slot)
  - `throat_width: ParamLength` (pre-cleared upper neck slot width, e.g. `8.5 mm`)
  - `slot_height: ParamLength` (axial height of undercut head, e.g. `6.0 mm`)
  - `radial_stepover: ParamLength` (lateral roughing stepover per side, default `0.8 mm`)
  - `axial_stepdown: ParamLength` (vertical overlap step when `slot_height > cutting_thickness_mm`, default `0.8 * cutting_thickness`)
  - `finish_allowance: ParamLength` (default `0.15 mm`), `spring_pass: bool`
  - `entry_mode: UndercutEntryMode` (`OpenEnd` default | `EntryBoreCenter` | `BothEndsThrough`)
  - `entry_extension: ParamLength` (default `tool_r + 2.0 mm` lead-in past open end!)
  - `climb_direction: bool` (default `true`)
- Define `DovetailParams`:
  - `mode: DovetailMode` (`InternalSlot` | `ExternalRail`)
  - `top_throat_width: ParamLength` (for `InternalSlot` — validated `> neck_diameter_mm + 0.4 mm`)
  - `dovetail_depth: ParamLength` (axial depth of angled flank)
  - `side_angle_deg: f64` (default `60.0°` — validated against `DovetailCutter.side_angle_deg`)
  - `radial_stepover: ParamLength` (progressive lateral stepover into the angled flank, default `0.6 mm`)
  - `finish_allowance: ParamLength` (default `0.1 mm`), `spring_pass: bool`
  - `entry_mode: UndercutEntryMode`, `entry_extension: ParamLength`
  - `climb_direction: bool`
- Define `Undercut3DParams`:
  - `z_stepdown: ParamLength` (default `0.4 mm`, or `use_cusp_height` with `target_cusp: ParamLength` default `0.025 mm`)
  - `min_overhang_angle_deg: f64` (default `5.0°` below horizontal equator so `n_z <= -sin(angle)` surfaces are targeted!)
  - `neck_clearance: ParamLength` (default `0.25 mm` minimum radial clearance between relieved neck/shank and overhanging lip!)
  - `stock_to_leave: ParamLength` (default `0.0 mm`)
  - `lead_out_distance: ParamLength` (safe horizontal pull-away away from undercut wall before `+Z` retract, default `tool_r + 1.0 mm`)
  - `climb_direction: bool`

---

### 2. Core Strategy Generators (`chip-cam`)

#### [NEW] `crates/chip-cam/src/strategies/slot_undercut.rs`
#### [NEW] `crates/chip-cam/src/strategies/dovetail.rs`
#### [NEW] `crates/chip-cam/src/strategies/undercut3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
- **`slot_undercut.rs` (`SlotUndercut`):**
  - Extracts centerline path (`ArcPolyline2D`) from selected B-Rep floor face / edge chain / 2D sketch centerline (with automatic width/depth inference when selecting a modeled STEP T-slot or keyway face!).
  - Validates `neck_diameter_mm < throat_width_mm`, `neck_length_mm >= total_depth`, `cutting_thickness_mm <= slot_height_mm`, and `cutter_diameter_mm <= slot_width_mm`.
  - Computes safe axial Z passes (bottom-up or top-down spanning `[z_bottom, z_bottom + slot_height - cutting_thickness]`) and progressive lateral offsets from throat centerline outward to both left & right walls (`±(0.5 * slot_width - tool_r)`), plus finish & optional spring pass.
  - Enforces strict **Enter-at-Open-End / Entry-Bore → Cut Horizontally → Return to Centerline → Exit Horizontally → Retract `+Z`** motion sequencing!
- **`dovetail.rs` (`Dovetail`):**
  - Extracts centerline or rail contour from selected STEP dovetail faces/edges or 2D sketch chain (auto-inferring dovetail angle & depth when a modeled angled planar B-Rep face is picked!).
  - Validates `|tool.side_angle_deg - params.side_angle_deg| <= 0.5°`, `neck_diameter_mm < top_throat_width` (for internal slots), `cutting_height_mm >= dovetail_depth`, and `neck_length_mm >= total_depth`.
  - Generates progressive lateral stepovers (`dx_k`) cutting from the pre-cleared vertical wall outward to the exact final dovetail wall offset (`±(0.5 * top_throat_width + dovetail_depth / tan(side_angle) - R_bottom)`), keeping `z_tip` at exact `z_bottom`, returning to safe throat centerline & exiting the open end before every `+Z` retract!
- **`undercut3d.rs` (`Undercut3D`):**
  - Identifies downward-facing / overhanging surface regions (`n_z < -sin(min_overhang_angle)`) on the target `TriMesh3D` (`FlatBvh`) within optional 2D containment boundary and Z height band `[z_bottom, z_top]`.
  - Slices overhanging surfaces at Z-levels (`z_k`), computes exact `f64` horizontal offset contours where the `LollipopEndmill` sphere (`center_z = z_tip + R`) contacts the overhanging wall tangentially (`p_center = p_surf + (R + stock_to_leave) * n_surf`).
  - Tests **every** contact station's relieved neck (`r_neck + neck_clearance` over `z ∈ [z_join, z_neck_top]`) and shank (`r_shank + neck_clearance` over `z > z_neck_top`) against the overhanging roof above via `FlatBvh` — trimming unreachable deep recesses cleanly and inserting horizontal `lead_out_distance` pull-aways into open air before any `+Z` retract!
- All 3 strategies automatically pass through Milestone 61 `route_toolpath_around_fixtures` and Milestone 58 `optimize_toolpath_feedrates`!

---

### 3. Operation Inspector UI, Icons, & Validation (`chip-gui`, `chip-project`)

#### [MODIFY] `crates/chip-gui/src/widgets/icons.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs`
#### [NEW] `crates/chip-gui/src/panels/operation_inspector/passes_tab/undercut_cards.rs`
#### [MODIFY] `crates/chip-gui/src/panels/browser.rs`
#### [MODIFY] `crates/chip-gui/src/app/{menus.rs, commands.rs}`
#### [MODIFY] `crates/chip-gui/src/panels/command_palette.rs`
#### [MODIFY] `crates/chip-project/src/validation.rs`
- Add procedural vector icons (`IconKind::SlotUndercut`, `IconKind::Dovetail`, `IconKind::Undercut3D`), Operations menu + CAM Browser + `Ctrl+K` Command Palette actions (`+ Slot / T-Slot`, `+ Dovetail`, `+ 3D Undercut`).
- Add strict two-column Inspector cards in `undercut_cards.rs` with universal hover tooltips, T-Slot/Keyway preset selector, live throat/neck/angle clearance banners, and an explicit advisory banner if `Tri-Dexel` engine isn't active when previewing undercut ops.
- Add comprehensive validation rules in `chip-project/src/validation.rs`.

---

## Verification Plan

1. **Strategy Invariant & Safety Unit Tests (`chip-cam`, `chip-sim`):**
   - **T-Slot Strategy Test:** Pre-clear a vertical throat slot with a Flat Endmill, then machine an M8 T-Slot with `SlotMill` -> assert zero vertical plunges/retracts inside the T-slot roof, exact left/right undercut widths in `TriDexelStock` (CPU & GPU parity), and `0` critical errors!
   - **Dovetail Strategy Test:** Pre-clear a vertical throat slot, then machine a `60°` Dovetail slot with `DovetailCutter` -> assert angled flank geometry matches `60°` within `< 0.05 mm` in `TriDexelStock`, zero vertical roof crashes, `0` critical errors, and mismatching angle (`45°` cutter on `60°` slot) returns a clear validation error!
   - **Lollipop 3D Undercut Test:** Machine an overhanging concave groove / angled underside pocket with `LollipopEndmill` -> assert exact roof preservation, automatic neck-clearance clipping on a deliberately too-deep undercut recess (zero neck rub!), and CPU/GPU Tri-Dexel parity!
   - **Safety Rejection Tests:** Verify blind T-slot with no entry bore, `neck_diameter >= throat_width`, and `neck_length < depth` all fail cleanly with descriptive `CamError` diagnostics.
2. **Headless Vulkan Visual Verification:**
   - Capture Tri-Dexel screenshots showing carved T-slot, `60°` dovetail, and lollipop undercut cavities with their 3D revolved cutters and strict two-column Inspector cards; inspect via `read_file` and clean up `/tmp/m66_*` temp files.
3. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`, pure-CPU `cargo check --locked -p chip-stock` & `-p chip-cam`, `cargo deny check` (default + `flatpak-portal`), `cargo clippy` (all targets & feature sets), `WGPU_BACKEND=vulkan cargo test --locked --workspace`, `chip-stock --features gpu-compute`, `chip-sim --features gpu-compute`.
