# Milestone 94 Implementation Plan — Physics-Backed Spindle Power, MRR, Tool Deflection / Breakage Advisor, & Helical Bore Inner-Arc Compensation

## Goal

Build a UI-agnostic, physics-backed **Machinability & Tool-Breakage Advisor** (`chip-tools` + `chip-cam` + `chip-project` + `chip-cli`) that evaluates every operation against the project's workpiece material, the active machine's real spindle power/torque curve, and the cutter's true flute-count-aware bending stiffness—catching spindle stall risk, cubic stickout deflection/snap hazards, dry aluminum/6063 chip-welding & thin-chip rubbing, helical bore inner-arc feed amplification, deep full-depth bore finish passes, deep-hole chip packing, and slow open-air plunges before a single chip is cut—and add opt-in inner-arc feed compensation + Z-staged chip-clearing support to `DrillBore::HelicalBore`.

---

## Problem Diagnosis

1. **No Physics Guardrails Between Preset Numbers and Physical Machine/Cutter Limits:**
   - Today's pre-flight diagnostics (`chip-gui/src/panels/diagnostics.rs`, `chip-cli validate`) check only static RPM bounds, axis feed limits, and stock travel envelopes. They do not detect when a valid RPM + feedrate snaps a cutter (e.g., `Ø3.175 × 25 mm LOC` 1FL O-flute at `28 mm` stickout, `L/D = 8.8×`, cutting `ap = 4.04 mm × ae = 0.50 mm` in dry 6063 aluminum) or stalls a `150–200 W` desktop spindle.
2. **Helical Bore Inner-Arc Feed Amplification & Full-Depth Finish Pass Hazards (`crates/chip-cam/src/strategies/drill_bore.rs`):**
   - `feed_opt::optimize_toolpath_feedrates` explicitly skips `OperationKind::DrillBore`, and `emit_helical_bore` emits tool-center feedrates directly without scaling down by `r_orbit / R_hole`. On a `Ø5.0 mm` hole with a `Ø3.175 mm` cutter, the cutting edge moves **`2.74×–2.95×` faster** than programmed (`309 mm/min` center → `846–912 mm/min` wall, `0.077–0.083 mm/tooth`!).
   - When `bore_finish_pass = true`, `emit_helical_bore` spirals to `hole_bottom_z` and then takes the **entire hole depth in a single 360° concave finish pass**—a severe chatter/snap hazard on deep (`> 2.5 × D`) holes with long-reach cutters—and continuous deep bores (`> 2.5 × hole_d`) have no intermediate chip-clearing retract option.

---

## Proposed Changes

### 1. Core Cutter Stiffness, Spindle Power, & Chip-Thickness Physics (`chip-tools`)

#### [NEW] `crates/chip-tools/src/machinability.rs`
#### [MODIFY] `crates/chip-tools/src/machine.rs`
#### [MODIFY] `crates/chip-tools/src/lib.rs`
- Add optional `#[serde(default, skip_serializing_if = "Option::is_none")] pub rated_power_rpm: Option<f64>` and `pub spindle_efficiency: Option<f64>` to `MachineSpindleSpec` (with sensible defaults: e.g., constant-torque linear derating below rated RPM for desktop brushless spindles like Carvera Air `200 W` / Carvera Z1 `150 W` vs. `1.5 kW` VFD spindles).
- Implement pure `f64` physics functions in `chip-tools::machinability`:
  - **`effective_core_diameter_mm(geom: &CutterGeometry) -> f64`:**
    - Flute-count-aware core fraction for endmills: `1FL O-flute ≈ 0.54 * D`, `2FL ≈ 0.62 * D`, `3FL ≈ 0.72 * D`, `4+FL ≈ 0.78 * D`; ball/bullnose/tapered/V-bit/drill/thread-mill/lollipop/slot/dovetail handled explicitly (using `neck_diameter_mm` over `neck_length_mm` for relieved-neck cutters via two-segment stepped cantilever compliance!).
  - **`cantilever_deflection_and_stress(tool: &Tool, radial_force_n: f64, cut_z_centroid_mm: f64) -> DeflectionResult`:**
    - Uses true stickout length $L = \text{stickout\_length\_mm}$, stepped shank/flute-core/neck moments of area $I = \pi d^4 / 64$, carbide modulus $E = 600\text{ GPa}$ (with HSS option $210\text{ GPa}$), exact cubic deflection $\delta \propto L^3 / d_{\text{core}}^4$, extreme-fiber bending stress $\sigma = 32 F_r L / (\pi d_{\text{core}}^3)$, and safety factor against carbide transverse rupture strength ($\text{TRS} \approx 2800\text{ MPa}$ with fatigue/impact knockdown for interrupted cuts!).
  - **`available_spindle_power_watts(spindle: &MachineSpindleSpec, rpm: f64) -> Option<f64>`:**
    - Evaluates realistic power curve at `rpm` (derating linearly below `rated_power_rpm` so low-RPM stalls on high-speed spindles are caught accurately).
  - **`compute_engagement_physics(input: &CutPhysicsInput) -> CutPhysicsMetrics`:**
    - Computes exact engagement arc $\theta$ (both straight cuts and **concave internal bore arcs** via law of cosines!), radial chip thinning $h_{\max}$, effective wall feedrate $F_{\text{wall}} = F_{\text{center}} \cdot (R_{\text{hole}} / r_{\text{orbit}})$, MRR ($\text{cm}^3/\text{min}$), cutting power ($W$ and `%` of available spindle power), tangential/radial cutting forces ($F_t, F_r$), tip deflection ($\mu\text{m}$ and `%` of tool diameter), bending stress safety factor, and `L/D` stickout ratio.

---

### 2. Operation-Level Machinability Advisor & Actionable Safe Fixes (`chip-cam`)

#### [NEW] `crates/chip-cam/src/machinability_advisor.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- Define UI-agnostic advisory types (`MachinabilitySeverity::{Ok, Info, Warning, Critical}`, `MachinabilityCheckKind`, `MachinabilityFinding`, `MachinabilityFix`, `OperationMachinabilityReport`, `SetupMachinabilityReport`):
  - **Checks evaluated for every non-suppressed operation:**
    1. **Spindle Power / Stall Risk:** `Warning` above `70%` available power, `Critical` above `95%` (or exceeding rated torque at low RPM).
    2. **Tool Deflection & Carbide Bending-Stress Snap Hazard:** `Warning` when $\delta > 0.025\text{ mm}$ (or `> 0.8% D`) or bending safety factor `< 2.5`; `Critical` when $\delta > 0.060\text{ mm}$ (or `> 2% D`) or safety factor `< 1.5`—explicitly incorporating `L/D > 6×` high-stickout & interrupted-cut shock knockdown!
    3. **Thin-Chip Rubbing & Dry Aluminum / 6063 Chip-Welding:** `Warning` when $h_{\max} < h_{\min}$ (especially on `gummy_chip_weld_risk` materials like 6063-T5 / 6061-T6 cut dry or with air blast), or when nominal chipload exceeds tool/material upper bound (`Critical` overload).
    4. **Helical Bore Inner-Arc Wall Overload:** Detects uncompensated inner-arc periphery feed amplification (`R_hole / r_orbit > 1.35×`) causing wall chipload to exceed safe limits.
    5. **Helical Bore Full-Depth Radial Finish Hazard:** Flags `bore_finish_pass = true` when hole depth $> 2.5 \times D_{\text{tool}}$ (single-lap full-flute concave wall pass) as `Warning` (`Critical` when combined with `L/D > 6×` or 1FL O-flute).
    6. **Deep-Hole Chip-Evacuation Warning:** Flags helical bores with `depth > 2.5 * hole_d` and radial annulus clearance `< 1.5 mm/side` without intermediate retract / peck clearing.
    7. **Slow Open-Air Plunge Advisory (`Info`):** Flags operations whose entry plunges occur outside stock bounds (e.g., `Contour2D` with `open_chain_extension > 0` outside stock) at conservative in-material plunge feeds (`< 40%` of cutting feed), suggesting a safe faster open-air plunge override.
  - **Actionable 1-Click Fixes (`MachinabilityFix`):**
    - Typed, deterministic mutation descriptors (`SetMaxStepdown(f64)`, `SetStepover(f64)`, `SetCuttingFeed(f64)`, `SetRampFeed(f64)`, `SetPlungeFeed(f64)`, `EnableHelicalInnerArcComp`, `DisableDeepBoreFinishPass`, `EnableBorePeckRetract(f64)`) with `apply_machinability_fix(op: &mut Operation, fix: &MachinabilityFix)` so both GUI and tests apply exact safe fixes with atomic Undo!

---

### 3. Helical Bore Safety Enhancements (`chip-cam`)

#### [MODIFY] `crates/chip-cam/src/params/milling_2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/drill_bore.rs`
- Add two backward-compatible, serde-default fields to `DrillBoreParams` (default `false` / `None` on legacy TOML so all 21 golden G-code files stay byte-identical!):
  - `#[serde(default, skip_serializing_if = "std::ops::Not::not")] pub bore_inner_arc_feed_comp: bool`: scales helical ramp, bottom cleanup, finish, and lead arc center feeds by `(r_orbit / R_hole).clamp(0.20, 1.0)` so actual cutting-edge chipload matches the programmed feedrate.
  - `#[serde(default, skip_serializing_if = "Option::is_none")] pub bore_chip_clear_depth: Option<ParamLength>`: when set on deep helical bores, stages the helix in Z intervals of `bore_chip_clear_depth` with full retract to `r_plane_z` between stages to evacuate chips cleanly!
- Default `bore_inner_arc_feed_comp = true` only on **newly created** `DrillBore` operations (`Operation::new`), keeping deserialized legacy projects `false` unless opted in.

---

### 4. Project & CLI Integration (`chip-project`, `chip-cli`)

#### [MODIFY] `crates/chip-project/src/lib.rs`
#### [MODIFY] `crates/chip-cli/src/main.rs`
- Expose `analyze_project_machinability(project, machine, cad_doc)` in `chip-project`.
- Extend `chipcutter validate <project.chip.toml>` with `--physics` (and `--json` fields) reporting per-operation MRR, spindle load `%`, tip deflection `µm`, bending safety factor, effective $h_{\max}$, and all `Warning`/`Critical` machinability findings (with `--strict-physics` optional non-zero exit code for CI/batch shop validation).

---

## Verification Plan

1. **Real-World BarTrim Regression Tests (`chip-tools`, `chip-cam`):**
   - **BarTrim Facing Failure Case:** `Carvera Air` (`200 W`), `Aluminum 6063-T5`, `Ø3.175 × 25 mm LOC` 1FL O-flute (`28 mm` stickout, `L/D = 8.8×`):
     - Assert `ap = 4.04 mm × ae = 0.50 mm` triggers **`Critical`** deflection / tool-snap warning.
     - Assert tuned `ap = 2.02 mm × ae = 0.30 mm` drops cleanly to **`Ok`**.
   - **BarTrim Helical Bore Failure Case:** `Ø5.0 mm × 20.2 mm` through-hole with same 1FL O-flute:
     - Assert uncompensated `309 mm/min` + `bore_finish_pass = true` triggers **`Critical`** inner-arc wall overload + full-depth finish-pass snap warning + deep-hole chip-packing warning.
     - Assert tuned bore (`bore_finish_pass = false`, `bore_inner_arc_feed_comp = true`, `bore_chip_clear_depth = 10 mm`) passes cleanly with `0` critical warnings and exact wall chipload within `±5%` of target, verified crash-free via `chip-sim` Tri-Dexel!
   - **Spindle Stall & Open-Air Plunge Cases:** Verify `150 W` Carvera Z1 stall alert on heavy `Ø6 mm` slotting vs. `1.5 kW` Milo passing comfortably; verify open-air plunge `Info` advisory fires only on open-air entries.
2. **Golden Parity & Full Workspace Verification Gate:**
   - Confirm all 21 existing golden G-code files pass byte-for-byte unchanged.
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
