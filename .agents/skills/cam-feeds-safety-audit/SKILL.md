---
name: cam-feeds-safety-audit
description: >-
  Real-world CNC machinability, tool-deflection, and G-code safety audit
  checklist for authoring, reviewing, or tuning ChipCutter .chip.toml projects
  and feeds/speeds—especially on desktop mills (e.g., Carvera Air/Z1), long-reach
  single-flute O-flutes, aluminum 6063/6061, facing/contouring, and helical bores.
  Activate whenever the user asks to create, check, or tune CNC operations or
  feeds and speeds.
---

# CNC Feeds, Speeds, & Tool-Breakage Safety Audit

## Mandatory Physics & CAM Checklist

1. **Tool Core Stiffness & Stickout (`δ ∝ L³ / d_core⁴`):**
   - Never treat a 1-flute O-flute as having the same stiffness as a 3-flute endmill: a 1FL O-flute core is only `~0.52–0.58 × D` (`~4×` lower `I` than 3FL).
   - Flag high `L/D` stickouts (`L/D > 6×`, e.g., `Ø3.175 × 25 mm LOC` at `28 mm` stickout = `8.8×`) and keep `ap × ae` conservative to avoid bending snap and chatter.
2. **Workpiece Material & Chip-Thickness Bounds (Especially 6063 Aluminum):**
   - Account for radial chip thinning: `h_max = fz * sin(acos(1 - 2*ae/D))` (or exact concave-arc engagement angle inside bores).
   - In gummy **6063-T5** aluminum cut dry or with light air blast, avoid both **excessive radial load** (bending/chatter) and **ultra-thin rubbing chips** (`h_max < 0.008–0.012 mm`), which weld aluminum into the flute gullet.
   - Watch interrupted cuts and cantilevered setups (reduce `ap` and `ae` compared to rigid solid-vise cuts).
3. **Helical Bore Inner-Arc Feed Amplification & Full-Depth Finish Trap:**
   - Check `crates/chip-cam/src/strategies/drill_bore.rs` behavior: ChipCutter emits tool-center feedrate without inner-arc scaling.
   - Compute actual periphery feed: `F_wall = F_center * (R_hole / r_orbit)` where `r_orbit = R_hole - R_tool` (e.g., `2.74×–2.95×` for a `Ø5 mm` bore with a `Ø3.175 mm` tool!). Scale `ramp_feed_mm_min`, `cutting_feed_mm_min`, and `lead_feed_mm_min` overrides down by `r_orbit / R_hole`.
   - **`bore_finish_pass = true` hazard:** On deep bores (`depth > 2 × tool_d`), ChipCutter steps out radially at the bottom and takes the **entire hole depth in one 360° concave pass**—disable `bore_finish_pass` / `bore_spring_pass` on long-reach cutters unless `depth <= 2 × tool_d` with reduced stepover & wall-compensated feed.
   - **Deep-hole (`depth > 2.5 × hole_d`) chip evacuation:** Warn about trapped chips when radial annulus clearance is `< 1.5 mm/side`; recommend continuous air blast / mist or splitting into two Z-staged ops, and verify breakthrough margin (`-0.5 mm`) + fixture/collet clearance.
4. **Operation-Scoped Overrides & Stock-Margin Safety:**
   - Never copy an open-air `plunge_feed_mm_min` override onto an in-material plunge or boring operation.
   - Verify starting stock bounds cover actual saw-cut / oversize stock length, and ensure `heights.feed` doesn't waste excessive air-spiraling while keeping safe clearance.

## User Review & Validation Workflow

1. **Before modifying user `.chip.toml` files**, present a concise **Before / After** table listing every parameter change, effective wall feed / chipload, and rationale.
2. After user approval, apply edits cleanly across all requested project variants (preserving each file's existing `suppressed` states unless asked).
3. Verify every modified project with headless CLI validation and volumetric simulation:
   ```bash
   ./target/release/chipcutter validate <project.chip.toml>
   ./target/release/chipcutter simulate <project.chip.toml> --engine tri-dexel
   ```
   Confirm `0` errors and `0` critical crashes/gouges, and check removed volume / cycle time make physical sense.
