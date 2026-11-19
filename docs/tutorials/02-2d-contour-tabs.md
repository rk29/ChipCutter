# Tutorial 02 — 2D Contour / Profile & Holding Tabs

## 1. Goal & What You'll Learn

**2D Contour** machines along closed outer profiles, internal cutouts, or open edge chains at one or more Z stepdowns—with exact circular-arc (`G2`/`G3`) offsetting, entry ramps, tangent lead-in/out arcs, rough/finish wall passes, and **holding tabs** that keep cut-out parts from flying loose on the final breakthrough pass.

In this tutorial you will start from an **empty project**, import a minimal mounting plate (`100 × 70 × 12 mm` with one `R10 mm` rounded corner and one `45°` corner chamfer), and learn how to:

- Configure surrounding stock margins (`±4 mm`) around a profile cutout.
- Pick a closed outer contour loop (or open edge chain) and control cut side and direction.
- Configure **Compensation** (`Left / Climb`, `Right / Conventional`, `Center`), multiple Z stepdowns, **Rough/Finish Passes**, and **Triangular** or **Rectangular Holding Tabs** (Automatic or Manual 3D placement).
- Set up **Helical / Linear Ramps** and horizontal **Tangent Lead-In / Lead-Out** arcs to avoid vertical witness marks on finished walls.

---

## 2. Prerequisites & Starter Model

Start with an **empty project** and load the minimal contour plate included with this tutorial ([`02-contour-plate.step`](assets/02-contour-plate.step) — a clean `100 × 70 × 12 mm` plate with an `R10` arc corner, a `45°` chamfer corner, and no extra pockets or holes):

- **From the Welcome Overlay or Menu:** Launch `cargo run --release -p chip-gui`, click **Import 3D STEP Solid…**, and select `docs/tutorials/assets/02-contour-plate.step`.
- **Or open directly from the terminal:**
  ```bash
  cargo run --release -p chip-gui -- docs/tutorials/assets/02-contour-plate.step
  ```

---

## 3. Step-by-Step GUI Walkthrough

### Step 1 — Set Stock Margin & Pick the Outer Contour Loop (`Op 1 → Geometry`)

1. In **`Stock & WCS → Stock`**, set **`Side X Margin (±X)`** = `4 mm`, **`Side Y Margin (±Y)`** = `4 mm`, and **`Top Allowance (+Z)`** = `0 mm` so the `100 × 70 × 12 mm` plate sits cleanly inside a `108 × 78 × 12 mm` raw blank.
2. Add a single contour operation via **`Operations → 2D Contour`** (or **`+ Cont`** in the CAM Browser) and select **`T1 — 6mm 3FL Carbide Flat Endmill`** in the **`Tool`** tab.
3. Open the **`Geometry`** tab, click **`Closed Loops`**, and pick the bottom outer perimeter loop of the plate (so `Bottom Z` automatically snaps to `Model Bottom`):

![2D Contour Closed Loop Selection on Minimal Plate](../images/tutorials/02-2d-contour-tabs/01-contour-geometry.png)

- Use **`Open Chain`** instead of **`Closed Loops`** whenever you only want to machine part of a perimeter (click the start edge, then the end edge to auto-trace the shortest connected path).
- If a loop offsets onto the wrong side of the wall, click **`Flip Side`** or **`Rev`** next to `Closed Loop #0`.

### Step 2 — Configure Stepdowns, Wall Finishing, & Holding Tabs (`Op 1 → Passes`)

Switch to the **`Passes`** tab to inspect **Depth Passes & Stock to Leave**, **2D Contour Strategy**, and **Holding Tabs**:

![2D Contour Passes and Triangular Holding Tabs](../images/tutorials/02-2d-contour-tabs/02-contour-passes-tabs.png)

1. **Depth Passes:** Keep **`Multiple Depths`** enabled (`Maximum Stepdown = tool_d * 0.5` = `3.000 mm`) with **`Equalize Z Levels`** (4 equal `3.0 mm` passes across the `12 mm` plate), or enable **`Spiral Ramp`** for a continuous helical descent around the perimeter.
2. **Rough & Finish Wall Passes:** Increase **`Finish Passes`** to `1` with a light **`Finish Step`** (`0.15–0.25 mm`) and optional **`Spring Pass`** for tight external tolerances.
3. **Holding Tabs:** Scroll to the **Holding Tabs** card and enable **`Holding Tabs`**:
   - Choose **`Triangular`** (`25°` ramp angle) for smooth 3D ramped bridges without vertical stop marks, or **`Rectangular`** for compact vertical hop-overs.
   - Set **`Tab Width`** (bridge length along the perimeter) and **`Tab Height`** (bridge thickness above `Bottom Z`).
   - Use **`Automatic`** placement (by **`Tab Count`** or **`Tab Spacing`**), or switch to **`Manual`** and click **`Pick Tab Points in 3D Viewport`** to place tabs on straight edges where they are easy to trim afterward.

> [!TIP]
> The exact same **2D Contour + Holding Tabs** workflow works on **Pure-2D DXF / SVG** sheet jobs (`Import 2D DXF / SVG Artwork…` or `--sample pure-2d`)—just pick the outer vector loop and set `Bottom Height = Stock Bottom`.

### Step 3 — Tune Entry Ramps & Lead-In / Lead-Out Arcs (`Op 1 → Linking`)

Switch to the **`Linking`** tab to control how the cutter enters and leaves each Z level:

![2D Contour Linking Tab with Ramps and Tangent Lead-In Arcs](../images/tutorials/02-2d-contour-tabs/03-contour-linking.png)

- **Entry Mode:** Keep **`Helical Ramp`** or **`Linear Profile Ramp`** (`2.5°`) so the endmill eases into each depth level without vertical shock loading.
- **Lead-In / Lead-Out:** Keep **`Tangent Lead-In`** enabled (`Horiz. Radius = tool_d * 0.25`, `Horiz. Sweep = 90°`) so the tool sweeps smoothly onto and off the finished wall instead of leaving a plunge witness mark.

---

## 4. Key Parameters Reference

| Parameter | Location | Purpose & Recommended Starting Values |
|---|---|---|
| **Compensation** | `Passes → 2D Contour Strategy` | **`Left (Climb Milling)`** is standard on rigid CNC machines for best surface finish; **`Right (Conventional)`** reduces grab on flexible setups; **`Center`** cuts directly on the curve. |
| **G41/G42 Wear** | `Passes → 2D Contour Strategy` | Emits controller cutter-radius wear compensation metadata when dialing in tight outside dimensions at the machine. |
| **Spiral Ramp** | `Passes → 2D Contour Strategy` | Ramps continuously around closed contours down to `Bottom Z`, eliminating discrete Z-step witness lines. |
| **Rough / Finish Passes** | `Passes → 2D Contour Strategy` | Adds lateral roughing passes for wide stock margins and a light finishing pass (`0.1–0.25 mm`) + optional **`Spring Pass`**. |
| **Tab Profile & Ramp Angle** | `Passes → Holding Tabs` | **`Triangular` (`20°–30°`)** keeps smooth XY feed motion across tabs; **`Rectangular`** minimizes tab footprint. |
| **Tab Width / Tab Height** | `Passes → Holding Tabs` | Typical sheet/plate tabs: **`6–10 mm`** wide × **`1.5–2.5 mm`** tall in wood/plastic; **`3–5 mm`** wide × **`0.8–1.5 mm`** tall in aluminum. |
| **Entry Mode & Ramp Angle** | `Linking → Ramp & Cavity Entry` | Use **`Helical Ramp`** or **`Linear Profile Ramp`** at **`2°–3°`** in metal and **`5°–10°`** in wood/plastics. |

---

## 5. Generate (`Ctrl+G`) & 3D Simulation Verification

Press **`Ctrl+G`** and click **`Simulate`** (`Simulation Timeline` bottom tab), scrubbing near completion (`95%`):

![3D Simulation of Contoured Plate Retained by Holding Tabs](../images/tutorials/02-2d-contour-tabs/04-contour-simulation.png)

Verify in the 3D machined stock:
- Both the `R10 mm` rounded corner and the `45°` corner chamfer are cleanly cut to full `12 mm` depth.
- The **triangular holding tabs** visibly bridge the finished plate to the surrounding waste stock, and **`Verified Crash-Free`** reports zero rapid crashes or part gouges.

---

## 6. Post-Processing (GRBL & LinuxCNC)

Switch to **`G-Code Preview`**:

![Verified GRBL G-Code Output for 2D Contour and Tabs](../images/tutorials/02-2d-contour-tabs/05-contour-gcode.png)

- The `R10 mm` rounded corner and tangent lead-in/out moves emit exact **`G2` / `G3`** circular arcs (split into quadrants automatically on **GRBL 1.1 / FluidNC**).
- Triangular holding tabs emit smooth 3-axis `(X, Y, Z)` ramped moves up and over each bridge.

---

## 7. Headless CLI Equivalent (`chipcutter`)

Generate the minimal contour plate and project, validate, simulate, and post-process headlessly:

```bash
# 1. Generate the minimal 100x70x12mm contour plate + 1-operation .chip.toml project
cargo run -p chip-cli -- generate-sample-step /tmp/02-contour-plate.step --tutorial 02-contour \
  --project-out /tmp/02-contour.chip.toml

# 2. Validate, generate toolpath, and verify zero crashes/gouges
cargo run -p chip-cli -- validate --generate /tmp/02-contour.chip.toml
cargo run -p chip-cli -- simulate /tmp/02-contour.chip.toml --resolution standard

# 3. Post-process for GRBL and LinuxCNC
cargo run -p chip-cli -- build /tmp/02-contour.chip.toml -o /tmp/contour_grbl --post grbl
cargo run -p chip-cli -- build /tmp/02-contour.chip.toml -o /tmp/contour_linuxcnc --post linuxcnc
```

---

## Next Steps

- **Previous:** [Tutorial 01 — Facing (Stock Surfacing)](01-facing.md)
- **Next:** [Tutorial 03 — 2D Pocket (Multi-Island & Open-Shoulder)](03-2d-pocket.md)
- **Index:** [All Tutorials](README.md)
