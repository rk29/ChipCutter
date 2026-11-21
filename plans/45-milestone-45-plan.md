# Milestone 45 Implementation Plan — `ThreadMill` Cutter Geometry, ISO Metric & Unified Thread Tables, & Helical Thread Milling Kernel

## Goal

Implement the complete UI-agnostic **Helical Thread Milling** data model, standard thread tables, toolpath generator, and post-processor verification across `chip-tools`, `chip-cam`, `chip-project`, and `chip-post`—supporting both single-point and multi-form thread mills, internal holes and external bosses, right-hand and left-hand threads, climb and conventional cutting, multi-step radial passes, spring passes, smooth tangent arc-in/arc-out, and centerline-compensated feedrates.

---

## Proposed Changes

### 1. `CutterGeometry::ThreadMill` & Fusion 360 Importer (`chip-tools`)

#### [MODIFY] [crates/chip-tools/src/model.rs](file:///workspace/ChipCutter/crates/chip-tools/src/model.rs)
#### [MODIFY] [crates/chip-tools/src/fusion_import.rs](file:///workspace/ChipCutter/crates/chip-tools/src/fusion_import.rs)
#### [MODIFY] [crates/chip-tools/src/builtin.rs](file:///workspace/ChipCutter/crates/chip-tools/src/builtin.rs)
#### [MODIFY] [crates/chip-tools/src/builtin_machines.rs](file:///workspace/ChipCutter/crates/chip-tools/src/builtin_machines.rs)
- Add a new `#[serde(rename = "thread_mill")]` variant to `CutterGeometry`:
  ```rust
  ThreadMill {
      diameter_mm: f64,        // Outer cutting crest diameter
      neck_diameter_mm: f64,   // Relieved neck diameter (< diameter_mm)
      neck_length_mm: f64,     // Usable reach from tip to shank taper
      pitch_mm: f64,           // Cutter pitch (nominal for multi-form; max/reference for single-point)
      teeth_count: u32,        // 1 = single-point (cuts any pitch >= cutter pitch), >1 = multi-form
      thread_angle_deg: f64,   // Included thread flank angle (default 60.0° for ISO/Unified)
      flute_length_mm: f64,    // Axial length of active cutting teeth (`pitch_mm * teeth_count` min)
      overall_length_mm: f64,
      shank_diameter_mm: f64,
      flutes: u32,
  }
  ```
- Implement validation invariants in `Tool::validate()`:
  - `diameter_mm > 0.0`, `neck_diameter_mm > 0.0 && neck_diameter_mm < diameter_mm`
  - `pitch_mm > 0.0`, `teeth_count >= 1`, `thread_angle_deg in 29.0..=90.0`
  - `neck_length_mm >= flute_length_mm`, `overall_length_mm > neck_length_mm`
- Map Fusion 360 `"thread mill"` tool JSON definitions in `fusion_import.rs` to `CutterGeometry::ThreadMill`.
- Add starter single-point thread mills (e.g., `Ø4.75 mm` single-point `60°` for `M6–M12` / `1/4-20`+, and `Ø3.10 mm` micro single-point for `M4–M6`) to `default_starter_tool_library()`, `generic`, `milo-2.0`, and `voron-cascade` (updating matching `machines/*.machine.toml`).

---

### 2. Standard Thread Tables (`chip-cam`)

#### [NEW] `crates/chip-cam/src/thread_tables.rs`
#### [MODIFY] [crates/chip-cam/src/lib.rs](file:///workspace/ChipCutter/crates/chip-cam/src/lib.rs)
- Provide deterministic, pure-Rust standard thread tables (canonical `f64` millimeters) with zero external dependencies:
  - **`ThreadStandardKind`:** `IsoMetricCoarse`, `IsoMetricFine`, `UnifiedUnc`, `UnifiedUnf`, `Custom`.
  - **`ThreadSpecEntry`:** `designation` (e.g. `"M6 x 1.0"`, `"M8 x 1.25"`, `"1/4-20 UNC"`, `"#10-32 UNF"`), `standard`, `major_diameter_mm`, `pitch_mm`, `minor_diameter_internal_mm`, `pitch_diameter_mm`, `recommended_tap_drill_mm`.
  - Cover standard sizes:
    - **ISO Metric Coarse:** `M2×0.4`, `M2.5×0.45`, `M3×0.5`, `M4×0.7`, `M5×0.8`, `M6×1.0`, `M8×1.25`, `M10×1.5`, `M12×1.75`, `M14×2.0`, `M16×2.0`, `M20×2.5`, `M24×3.0`.
    - **ISO Metric Fine:** `M6×0.75`, `M8×1.0`, `M10×1.0`, `M10×1.25`, `M12×1.25`, `M12×1.5`, `M16×1.5`.
    - **Unified UNC:** `#4-40`, `#6-32`, `#8-32`, `#10-24`, `1/4-20`, `5/16-18`, `3/8-16`, `7/16-14`, `1/2-13`, `5/8-11`, `3/4-10`.
    - **Unified UNF:** `#6-40`, `#8-36`, `#10-32`, `1/4-28`, `5/16-24`, `3/8-24`, `1/2-20`.
  - Provide helper `suggest_thread_for_hole_diameter(minor_dia_mm: f64)` to suggest matching standard threads from a picked cylindrical B-Rep hole!

---

### 3. `OperationKind::ThreadMill`, `ThreadMillParams`, & Helical Generator (`chip-cam`)

#### [MODIFY] [crates/chip-cam/src/operation.rs](file:///workspace/ChipCutter/crates/chip-cam/src/operation.rs)
#### [MODIFY] [crates/chip-cam/src/params.rs](file:///workspace/ChipCutter/crates/chip-cam/src/params.rs)
#### [NEW] `crates/chip-cam/src/strategies/thread_mill.rs`
#### [MODIFY] [crates/chip-cam/src/strategies/mod.rs](file:///workspace/ChipCutter/crates/chip-cam/src/strategies/mod.rs)
- Add `OperationKind::ThreadMill` (`#[serde(rename = "thread_mill")]`) and `StrategyParams::ThreadMill(ThreadMillParams)` with backward-compatible `.chip.toml` serialization.
- `ThreadMillParams` fields (all lengths as `ParamLength` for expression & unit support):
  - `form: ThreadFormKind` (`InternalHole` | `ExternalBoss`)
  - `handedness: ThreadHandedness` (`RightHand` | `LeftHand`)
  - `cut_direction: ThreadCutDirection` (`Climb` | `Conventional`)
  - `standard_preset: Option<String>` (e.g., `Some("M8 x 1.25")` or `None` for custom)
  - `major_diameter: ParamLength`
  - `pitch: ParamLength`
  - `pitch_diameter_offset: ParamLength` (signed gauge-fit tweak, default `0.0 mm`)
  - `radial_passes: u32` (`1..=8`, default `1`; splits thread depth progressively with equal-area or constant-step engagement)
  - `spring_passes: u32` (`0..=3`, default `0`; repeats final full-depth pass to eliminate tool deflection)
  - `lead_in_clearance: ParamLength` (radial clearance from crest before tangent arc-in, default `0.5 mm`)
  - `compensate_centerline_feed: bool` (default `true`; scales programmed center-path feedrate by `(D_major - D_tool) / D_major` for internal threads and `(D_minor + D_tool) / D_minor` for external threads so cutting-edge chipload matches the tool preset)
  - `optimize_hole_order: bool` (default `true`; nearest-neighbor + 2-opt ordering across multiple threaded holes)
- **Exact Helical Kinematics (`strategies/thread_mill.rs`):**
  - Determine helical traversal direction automatically from `(form, handedness, cut_direction)`:
    - **Internal + Right-Hand + Climb:** **Bottom-Up** `CCW` (`G3`, `+Z` by `pitch` per `360°`) — preferred standard machinist practice (chips clear downward, zero recutting).
    - **Internal + Right-Hand + Conventional:** **Top-Down** `CW` (`G2`, `-Z` by `pitch` per `360°`).
    - **External + Right-Hand + Climb:** **Top-Down** `CW` (`G2`, `-Z` by `pitch` per `360°`).
    - **External + Right-Hand + Conventional:** **Bottom-Up** `CCW` (`G3`, `+Z` by `pitch` per `360°`).
    - Mirror `CW`/`CCW` & vertical sense symmetrically for **Left-Hand** threads!
  - **Radial Tangent Arc-In & Arc-Out:**
    - Internal: rapid to Clearance Z over hole center `(cx, cy)`, feed down along centerline to `z_start`, execute a smooth **quarter/half-circle helical lead-in arc** (`G2`/`G3` advancing `pitch / 4` or `pitch / 8` in Z) from centerline onto the cutting radius `R_path = (D_major_eff - D_tool) / 2`, run full `360°` helical revolutions (single-point: `ceil(thread_depth / pitch)` turns; multi-form: `1` full turn when `teeth_count * pitch >= thread_depth`, else stepped axial passes), execute symmetric **radial tangent arc-out** back to hole center `(cx, cy)`, then rapid straight up to Retract/Clearance Z.
    - External: approach outside boss clearance radius `R_clear = (D_major_eff + D_tool) / 2 + lead_in_clearance`, helical arc-in onto `R_path = (D_minor_eff + D_tool) / 2`, helical turns, arc-out to `R_clear`, retract cleanly.
  - **Pre-Flight Safety Invariants:**
    - Reject non-`ThreadMill` cutters.
    - Reject internal `D_tool >= D_minor` (cannot enter pilot hole without crashing) or `D_tool >= D_major_eff`.
    - Reject multi-form cutter (`teeth_count > 1`) when `|tool.pitch_mm - op.pitch| > 0.02 mm` (pitch mismatch destroys threads).
    - Emit clear validation error/warning when thread depth exceeds `neck_length_mm`.

### 4. Post-Processor Helical Arc Verification (`chip-post`)

#### [MODIFY] [crates/chip-post/src/lib.rs](file:///workspace/ChipCutter/crates/chip-post/src/lib.rs)
- Verify and add targeted tests confirming that `ThreadMill` helical `G2`/`G3` arcs with simultaneous `Z` pitch advance emit cleanly across:
  - **`grbl` / `grblhal_atc`:** Quadrant-split helical arcs (`I`, `J`, proportional `Z` per sub-arc) staying `<= 80` chars/line.
  - **`linuxcnc`:** Exact helical `G2`/`G3` arcs (`X`, `Y`, `Z`, `I`, `J`) with `M6` + `G43 H`.

---

## Verification Plan

1. **Unit & Geometric Oracle Tests:**
   - `chip-tools`: `ThreadMill` geometry validation, TOML roundtrip, Fusion 360 import, and starter tool library checks.
   - `chip-cam`: Thread table sanity (`ISO Metric` & `UNC`/`UNF` formulas), exact helical pitch `ΔZ / Δθ = pitch / (2π)` verification across all 8 combinations of `(Internal|External) × (RH|LH) × (Climb|Conventional)`, radial multi-pass progression, multi-form single-turn optimization, centerline feedrate compensation accuracy, and rejection of oversized cutters or mismatched multi-form pitches.
   - `chip-post`: Helical `G2`/`G3` output verification for `grbl` and `linuxcnc`.
2. **Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo test --locked -p chip-tools -p chip-cam -p chip-post -p chip-project`
