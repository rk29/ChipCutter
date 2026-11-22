# Milestone 58 Implementation Plan — Engagement-Aware Feedrate Optimization & 2.5D Corner Cleanup

## Goal

Introduce a clean, UI-agnostic **Engagement-Aware Feedrate Optimization** post-pass (`crates/chip-cam/src/feed_opt.rs`) providing **internal corner slowdown** (curvature-based perimeter-to-centerline scaling + smooth lookahead deceleration ramps) and **constant-chipload radial chip-thinning modulation**, expose opt-in schema-safe controls in `CuttingParams` and the Operation Inspector (preserving 100% backward compatibility with existing `.chip.toml` files and Phase 1–2 golden G-code snapshots), surface feed-modulation telemetry in `SimulationTimeline`, and polish 2.5D Adaptive/Rest & V-Carve corner residuals.

---

## Proposed Changes

### 1. Schema-Safe `FeedOptimizationConfig` & Core Optimizer (`chip-cam`)

#### [NEW] `crates/chip-cam/src/feed_opt.rs`
#### [MODIFY] `crates/chip-cam/src/params/common.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
- **Data Model (`FeedOptimizationConfig` on `CuttingParams` / `PassesParams`):**
  - `#[serde(default, skip_serializing_if = "FeedOptimizationConfig::is_default")]` so existing `.chip.toml` files (`schema_version = 1 / 2 / 3`) load cleanly and serialize with zero diff when disabled!
  - Fields:
    - `enabled: bool` (default `false` for backward compatibility on existing projects; easily toggled on any operation or preset)
    - `corner_slowdown: bool` (default `true` when enabled)
    - `min_feed_fraction: f64` (default `0.35`, clamped `0.15..=1.0`) — floor preventing rubbing/melting in tight corners
    - `corner_angle_threshold_deg: f64` (default `25.0`, clamped `5.0..=120.0`) — polyline turn angle triggering lookahead slowdown
    - `slowdown_distance_mm: ParamLength` (default `tool_d * 0.5`) — lookahead deceleration / recovery ramp length around sharp internal corners
    - `internal_arc_scaling: bool` (default `true`) — exact perimeter-to-centerline scaling `F_center = F_nom * (R_path - R_tool) / R_path` for concave/internal `G2`/`G3` moves (`R_path > R_tool`) so cutting-edge feed never spikes inside pocket fillets!
    - `chip_thinning_boost: bool` (default `false`) + `max_thinning_multiplier: f64` (default `1.5`, clamped `1.0..=2.5`) — constant-chipload boost on light radial-engagement finishing/peel moves, automatically throttled back inside corners.
- **Optimizer Algorithm (`optimize_toolpath_feedrates` in `feed_opt.rs`):**
  - Operates directly on the generated `ClEvent` stream in `crates/chip-cam/src/strategies/mod.rs` after strategy generation (working uniformly across 2.5D Pocket/Contour/Adaptive, 3-axis Surface, V-Carve, and 4th-axis Rotary strategies—never touching `G0` rapids, canned drill cycles, or explicit thread-mill centerline-compensated moves!):
  - **Internal Arcs (`ClEvent::ArcMove`):** Detect concave/internal arcs (based on CW/CCW vs. Climb/Conventional or curvature radius `R_arc < 4.0 * tool_r`), compute `scale = ((R_arc - tool_r) / R_arc).clamp(min_feed_fraction, 1.0)`, and emit scaled cutting feedrate.
  - **Sharp Corner Lookahead (`ClEvent::LinearMove`):** Identify turning corners where the 2D/3D deflection angle `θ >= corner_angle_threshold_deg`; split segments within `slowdown_distance_mm` before the corner (preserving exact geometry and arc endpoints!) to ramp feedrate down cleanly from `F_nom` to `F_nom * corner_scale(θ)` approaching the apex and recover smoothly after clearing the corner.
  - **Plunge / Ramp Safety:** Never accelerate plunge or steep Z-ramp moves above `plunge_feed_mm_min`; never exceed machine `max_cutting_feed_mm_min`.

---

### 2. 2.5D Adaptive/Rest & V-Carve Corner Cleanup Polish (`chip-cam`)

#### [MODIFY] `crates/chip-cam/src/strategies/adaptive2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/pocket2d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/vcarve.rs`
- **2.5D Adaptive / Rest Corner Residuals (`05-adaptive-rest`):**
  - Inspect why `05-adaptive-rest` leaves `maxEx = 0.985 mm` (`errVol = 131.0 mm³`) at fine Tri-Dexel resolution (typically tiny unpeeled cusps where crescent clipping `clip_eps` drops small corner slivers or where `FromPrecedingOperations` rest overlap under-extends onto the pocket wall) and ensure clean final wall/corner cleanup with zero gouging.
- **V-Carve Corner Climb Fidelity (`vcarve.rs`):**
  - Ensure sharp 3D V-bit corner climb climbs all the way to exact corner apexes with tighter Z-chord tolerance (`<= 0.005 mm`) so narrow serif tips carve cleanly with zero flattened blunting.

---

### 3. Operation Inspector UI & Simulation Feed Telemetry (`chip-sim`, `chip-gui`)

#### [MODIFY] `crates/chip-sim/src/timeline.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/ui.rs`
- Add `min_cutting_feed_mm_min`, `max_cutting_feed_mm_min`, and `corner_slowdown_segment_count` summary stats on `SimulationTimeline` (displayed cleanly in the Simulation Timeline HUD tooltip / Diagnostics).
- Add a strict two-column collapsible **Feed Optimization** card in `operation_inspector.rs` (Enable toggle, Corner Slowdown, Min Feed %, Internal Arc Scaling, Slowdown Distance expression input, Chip-Thinning Boost + Max Multiplier) with universal hover tooltips consistent with Phase 12 standards.

---

## Verification Plan

1. **Feed Optimizer Unit & Physics Tests (`chip-cam`, `chip-sim`, `chip-post`):**
   - Concave arc test asserting `F_center == F_nom * (R_arc - R_tool) / R_arc` (clamped at `min_feed_fraction`).
   - 90° internal pocket corner test verifying lookahead segment splitting, monotonic deceleration into the corner, exact original path geometry (zero deviation), and recovery after exit.
   - Verify `FeedOptimizationConfig::default()` (`enabled = false`) produces **100% byte-for-byte identical** `ClEvent` streams and golden G-code across all existing Phase 1–5 snapshots.
   - Verify `05-adaptive-rest` and `07-vcarve-medial` pass Tri-Dexel verification with zero crashes/gouges and reduced corner residuals.
   - Verify `.chip.toml` roundtrip with `FeedOptimizationConfig` both disabled (omitted cleanly) and enabled.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
