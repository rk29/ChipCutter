# 06 — Toolpath Simulation & Crash Verification

## Overview (`chip-sim`)
Before exporting G-code to a CNC controller, `chip-sim` provides both **interactive visual playback** and **automated geometric verification** against the stock and target STEP solid.

---

## 1. Animated Toolpath & Cutter Playback
- **Color-Coded Toolpath Overlay:**
  - `G0` Rapid moves: Dashed yellow/amber lines
  - `G1` Plunge / Ramp moves: Red/coral lines
  - `G1` Cutting feed moves: Royal blue / cyan lines
  - `G2` / `G3` Circular & Helical arcs: Emerald green curves
  - Lead-in / Lead-out transitions: Teal/magenta accents
- **Interactive Playback Scrubber:**
  - Play / Pause (`Space`), Step Forward / Step Backward by single motion segment (`←` / `→`), Jump to Next Operation / Tool Change.
  - Continuous timeline slider parameterized by **cumulative machining time** or **path distance**, with playback speed multiplier (`0.1x` to `50x`, plus "Instant Jump to End").
- **3D Cutter & Shank Visualization:**
  - Renders an accurate 3D revolution mesh of the active tool: cutting flutes (colored), non-cutting shank above `flute_length`, and simplified collet/holder cone above `stickout_length`.
  - Live HUD readout at current scrub position: `Operation`, `Tool`, `XYZ` (in WCS), motion type (`G0`/`G1`/`G2`/`G3`), commanded `Feedrate`, `RPM`, and `Elapsed / Total Time`.

---

## 2. Machining Time Estimator
- Computes per-operation and per-setup cycle time using a trapezoidal velocity profile approximation:
  - Uses programmed feedrate (`Vf`) on `G1`/`G2`/`G3` moves, machine rapid rate (configurable per post/machine profile, default `5000 mm/min`) on `G0` moves, dwell durations (`G4`), and an optional corner acceleration limit (`default 500 mm/s²`) + tool-change penalty (`default 15 s` ATC or `0 s` manual).

---

## 3. 2.5D Heightmap (Z-Dexel) Stock Material Removal
Because all v1 setups machine along a single Setup `+Z` spindle axis without undercuts, stock removal is modeled cleanly and accurately via a high-resolution **2D Z-Heightmap Grid** (`HeightfieldStock`):

### Data Structure
- Uniform grid aligned with the Setup's XY stock bounds at configurable resolution:
  - `Standard`: `~512 × 512` cells (instant updates)
  - `Fine`: `~1024 × 1024` cells
  - `Ultra`: `~2048 × 2048` cells
- Each grid cell `(i, j)` stores:
  - `z_stock: f32` — current top surface height of remaining stock (initialized to `Stock Top` inside stock footprint, `-∞` outside).
  - `last_op_id: u16` — operation index that last cut this cell (enables optional **Color Stock by Operation / Tool** visualization!).
  - `z_target: f32` — upper envelope of the target STEP solid in WCS (computed once per Setup by rasterizing the B-Rep faces onto the same XY grid).

### Swept Cutter Footprint Evaluation
For each linear or circular/helical segment `(P0 -> P1)` of tool profile `P(r)` (radial height function of Flat, Bull-Nose, Ball, Chamfer, or Drill tip):
1. Compute the 2D swept bounding box of `(P0, P1)` expanded by `tool_radius`.
2. Parallelize across grid rows via `rayon` (or optional `wgpu` compute shader): for each cell center `(x, y)` within `r <= tool_radius` of the 3D segment axis, compute minimum cutter envelope height `z_cut(x, y) = z_tip_at_closest + P(r)`.
3. Update `z_stock[i, j] = z_stock[i, j].min(z_cut)`.
4. Dirty tiles upload directly to a dynamic `wgpu` vertex/height texture buffer to render a shaded, crisp-walled 3D machined stock mesh in real time as the timeline scrubs!

---

## 4. Automated Crash & Gouge Verification
During simulation (and automatically during `chipcutter simulate` / pre-post verification), `chip-sim` evaluates four safety rules and logs structured **Simulation Diagnostics** clickable in the UI to jump the camera + timeline straight to the offending move:

| Check | Condition | Severity |
|---|---|---|
| **Rapid-into-Stock Crash** | A `G0` rapid move sweeps any cell where `z_cut(x, y) < z_stock(x, y) - tol` | **Critical Error** (blocks G-code export unless explicitly acknowledged) |
| **Target Model Gouge** | Any cutting or rapid move sweeps `z_cut(x, y) < z_target(x, y) - gouge_tol` (default `0.02 mm`) | **Critical Error** |
| **Flute Length Exceeded (Shank Rub)** | Engagement depth `z_stock_before(x, y) - z_tip > tool.flute_length` | **Warning** |
| **Holder / Collet Collision** | `z_stock(x, y) > z_tip + tool.stickout_length` within holder radius | **Critical Error** |
| **Excessive Plunge in Material** | Vertical plunge move (`ΔXY == 0`) with center-cutting = `false` or depth exceeding safe threshold | **Warning** |
