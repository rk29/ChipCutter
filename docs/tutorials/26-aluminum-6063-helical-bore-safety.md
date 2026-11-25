# Tutorial 26: Aluminum 6063-T5 Extrusions, Deep Helical Bore Safety, & Custom Post Profiles

## 1. Goal & What You'll Learn

Extruded **Aluminum 6063-T5** (such as `2020` T-slot rail or `20 × 20 mm` architectural bar stock) behaves very differently from `6061-T6` on benchtop CNCs like the **Makera Carvera Air** (`200 W` spindle): it is softer and gummier, welds aggressively onto dry flutes if chips get too thin, and snaps long-reach **`Ø3.175 mm (1/8") × 25 mm LOC` 1-flute O-flute** cutters (`~1.7 mm` core, `28 mm` stickout, `L/D = 8.8x`) when axial/radial loads or internal bore wall speeds are underestimated.

In this hands-on shop-floor tutorial you will build and audit a complete **Pure-2D** `20 × 20 × 125 mm` 6063 bar-trimming + `Ø5.0 mm × 20 mm` through-hole job from start to finish:

1. **Project-Level `Aluminum 6063-T5 Extrusion` Material:** Set the alloy once in **Project Inspector** (`u_s = 0.85 J/mm^3`, `K_c = 850 N/mm^2`, `h_min = 0.010 mm`, `[Weld Risk]`, `Air Blast`) and use **`Apply Matching Presets to Ops`** so Carvera 1FL O-flutes automatically pick their 6063-T5 cutting presets.
2. **Catching & Fixing High-`L/D` End-Trim Breakage:** See how an aggressive `ap = 4.04 mm × ae = 0.50 mm` cut triggers a **`[CRITICAL]`** stepped-cantilever snap hazard in **`Machinability & Safety`**, and fix it in one Undoable click (**`Apply Safe Fix`**).
3. **Eliminating All Three Deep Helical Bore Traps:** Fix `2.74×–2.95×` internal-arc wall feed amplification (`R_hole / r_orbit`), disable hazardous full-depth (`20 mm`) single-lap radial finish passes, and enable **`Chip-Clear Retract`** (`10 mm`) to clear chips out of a `4 × D` deep `Ø5 mm` bore.
4. **Custom `.post.toml` Profiles, Batch Multi-Setup Export, & CI Validation:** Inspect or clone a custom post profile in the **Post Profile Inspector**, export G-code, and gate jobs headlessly with `chipcutter validate --strict-physics --json`.

---

## 2. Prerequisites & Starter Project

This is a **Pure-2D** workflow (no external CAD file needed—just a built-in `2D Sketch` and `Fixed-Size Box Stock`), or you can start from a blank workspace:

```bash
chip-gui
```

1. Select **`New Pure-2D Parametric Sketch`** from the Welcome Overlay (or click **`+ Sketch`** in the top toolbar).
2. In **`Setup 1 → [Stock]`**, choose **`Fixed-Size Box Stock`**: `Width X = 20 mm`, `Depth Y = 125 mm`, `Height Z = 20 mm`, aligned `Offset from Front-Left` (`dX = 0`, `dY = 5 mm`) with `Offset from Top = 0 mm`.
3. Add a rear **`Machinist Vise`** (`80 mm` jaw width) in **`Setup 1 → [Fixtures]`** gripping the rear `80 mm` of the bar so the cut end (`Y = -120 mm`) and `Ø5.0 mm` hole (`X = 10 mm, Y = -110 mm`) cantilever safely clear of the jaws.

---

## 3. Step-by-Step GUI Walkthrough

### Step 1 — Select `Aluminum 6063-T5 Extrusion` & `Makera Carvera Air` at the Project Level

1. Click the root **Project** node in the left **Project Tree** (or open **`Edit → Project Properties…`**).
2. Set **`Target CNC Machine`** to **`Makera Carvera Air`** (`200 W` spindle, `1,000–13,000 RPM`).
3. Under **`Workpiece Material`**, choose **`Aluminum 6063-T5 Extrusion`**.
   - Review the physics summary: `Kc = 850 N/mm^2 | us = 0.85 J/mm^3 | h_min = 0.010 mm | Air Blast [Weld Risk]`.
   - Click **`Apply Matching Presets to Ops`** whenever you switch materials on an existing project to update operation `preset_id` values cleanly without clobbering any custom `feed_overrides`.
4. Confirm that every operation's **`[Tool]`** tab now displays the read-only clickable **`Project Material: Aluminum 6063-T5 Extrusion`** pill—no repeated per-operation material selection required.

![Project Inspector showing Aluminum 6063-T5 Extrusion selected as the project-level workpiece material on Makera Carvera Air](../images/tutorials/26-aluminum-6063-helical-bore-safety/01-al6063-project-material-and-presets.png)

---

### Step 2 — Audit the End-Trim Facing Contour (`ap = 4.04 mm × ae = 0.50 mm` → Safe)

1. Assign **`T2 — 3.175mm (1/8") x 25mm 1FL O-Flute Flat`** (`Stickout = 28.0 mm`, `L/D = 8.8x`, core diameter `~1.71 mm`) to **Op 1 (`Trim -Y End to 120mm`)** (`Contour2D` with `open_chain_extension = 3.0 mm` for clean open-air entry).
2. Test an aggressive roughing setup (`max_stepdown = 4.1 mm`, `roughing_stepover = 0.50 mm`) and inspect **`Operation Inspector → [Tool] → Machinability & Safety`**:
   - The advisor immediately flags **`[CRITICAL]` Tool snap / severe deflection risk**, showing how the thin `1FL` core (`I ∝ d_core^4`, `~4×` less rigid than a 3-flute) plus cubic `L^3` stickout compliance drives tip deflection and carbide bending stress past safe limits in interrupted 6063 cuts.
   - Notice the **`[INFO]`** open-air plunge advisory as well: because Op 1 plunges outside the stock on its `3.0 mm` open-chain extension, raising `plunge_feed_mm_min` to `600 mm/min` on Op 1 **only** saves air-cut time without affecting Op 2's in-material bore!
3. Click **`Apply Safe Fix`** (or dial in `max_stepdown = 2.2 mm` → `2.02 mm × 10` passes and `roughing_stepover = 0.30 mm`, `finish_stepover = 0.10 mm`, `finish_feed = 280 mm/min`, `repeat_finish_pass = true`) and verify Op 1 transitions cleanly to **`Machinability & Safety [OK]`**.

![Operation Inspector Tool tab flagging a Critical high-L/D tool-snap hazard on the 25mm LOC 1FL O-flute in Aluminum 6063-T5 with 1-click Apply Safe Fix](../images/tutorials/26-aluminum-6063-helical-bore-safety/02-bar-trim-critical-hazard.png)

---

### Step 3 — Protect the Deep `Ø5.0 mm × 20 mm` Helical Bore (`Inner-Arc Feed Comp` + `Chip-Clear Retract`)

1. Select **Op 2 (`Helical Bore 5mm Through Hole`)** (`DrillBore` in **`Helical Bore Milling`** mode, `Bottom = Stock Bottom - 0.5 mm` = `-20.5 mm` for clean breakthrough across `20 mm ± 0.2` extrusion tolerance).
2. Notice why a naive helical bore breaks small cutters in a `Ø5.0 mm` hole with a `Ø3.175 mm` tool (`r_orbit = 0.9125 mm`):
   - **Inner-Arc Feed Amplification (`2.74×–2.95×`):** Without compensation, `309 mm/min` at tool center drives the cutting edge at **`846–912 mm/min`** (`0.077–0.083 mm/tooth` — nearly `3×` preset chipload!).
   - **Full-Depth Finish Trap:** Leaving `bore_finish_pass = true` on a `20.5 mm` (`6.5 × D`) hole steps out radially at the bottom and takes the *entire* `20.5 mm` wall in a single concave lap.
   - **Deep-Hole Chip Packing (`4 × D`):** Only `0.91 mm/side` annulus clearance exists around the cutter—dry 6063 chips can pack and weld without periodic evacuation.
3. Open **`Operation Inspector → [Passes]`** and configure all three protections (or click **`Apply Fix`** in the bottom **`Diagnostics & Log → Machinability & Tool Safety`** table):
   - Uncheck **`Finish Pass`** (`bore_finish_pass = false`, `bore_spring_pass = false`) so the helix cuts directly to `Ø5.00 mm` at a gentle `0.20 mm/rev` pitch.
   - Enable **`Inner-Arc Feed Comp`** (`bore_inner_arc_feed_comp = true`) — notice the live **`R_hole/r_orbit`** amplification readout and automatic wall-safe centerline feed scaling.
   - Enable **`Chip-Clear Retract`** (`bore_chip_clear_depth = 10.0 mm`) so halfway down the `20.5 mm` bore the cutter pulls smoothly to hole center, retracts to `R-plane` (`+1.0 mm`) to clear 6063 chips with air blast, and resumes safely.
4. Check the bottom **`Diagnostics & Log → Machinability & Tool Safety`** table to confirm both Op 1 and Op 2 report green **`OK`** status.

![Helical Bore Passes tab with Inner-Arc Feed Comp and Chip-Clear Retract alongside the bottom Diagnostics & Log Machinability & Tool Safety table](../images/tutorials/26-aluminum-6063-helical-bore-safety/03-helical-bore-safety-and-diagnostics.png)

---

### Step 4 — Simulate in Tri-Dexel, Customize Post Profiles, & Batch-Export

1. Press **`Ctrl+G`** and run volumetric **`Tri-Dexel`** simulation (`Edit → Application Settings… → Simulation → Tri-Dexel`) to verify:
   - Op 1 trims the `-Y` end cleanly to `120.0 mm` with zero vise jaw contact.
   - Op 2 bores cleanly through `Z = -20.5 mm` with `0` crashes, `0` gouges, and `0` fixture collisions.
2. Need custom coolant/air-blast (`M7`/`M8`), spindle warm-up, or ATC headers for your Carvera Air or LinuxCNC mill?
   - Open **`Post → Post Profile Inspector / Customizer…`** (or click **`Inspect Post…`** in **`G-Code Preview`**).
   - Built-in profiles (`grblhal_atc`, `grbl`, `linuxcnc`) are strictly read-only so upgrades never overwrite your tweaks—click **`Clone to Custom .post.toml`** to save an editable copy in your OS config folder (`~/.config/chipcutter/posts/`) and select it directly in **`Setup → [WCS & Post]`**.
   - For multi-setup jobs, use **`Post → Export All Setups…`** to write sanitized per-Setup `.nc`/`.ngc` files into one output folder.

![Completed Tri-Dexel volumetric simulation of the tuned 6063 bar-trim and 5mm helical bore project showing Verified Crash-Free and Machinability OK](../images/tutorials/26-aluminum-6063-helical-bore-safety/04-bar-trim-safe-tridexel-sim.png)

---

## 4. Key Parameters Reference (6063-T5 on Carvera Air, `Ø3.175 × 25 mm` 1FL O-Flute)

| Operation / Setting | Unsafe (Snap Risk) | Safe (Tuned) | Why |
|---|---|---|---|
| **Project Material** | Unset (`6061` defaults) | **`Aluminum 6063-T5 Extrusion`** | Models `K_c = 850`, `h_min = 0.010 mm`, and `[Weld Risk]` |
| **Op 1 `max_stepdown` × `stepover`** | `4.04 mm × 0.50 mm` | **`2.02 mm × 0.30 mm`** | Cuts bending deflection & snap stress by `~70%` at `L/D = 8.8x` |
| **Op 1 `plunge_feed` override** | `103 mm/min` | **`600 mm/min`** *(Op 1 open-air only!)* | Safe because Op 1 plunges in open air; **never** copy onto Op 2 |
| **Op 2 `bore_finish_pass`** | `true` (`20.2 mm` full-depth lap) | **`false`** | Eliminates `6.4 × D` full-flute concave chatter/snap pass |
| **Op 2 `bore_inner_arc_feed_comp`** | `false` (`846–912 mm/min` wall!) | **`true`** (`r_orbit / R_hole` scaling) | Keeps actual wall chipload at safe `~0.024 mm/tooth` |
| **Op 2 `bore_chip_clear_depth`** | `None` (`20.5 mm` blind continuous) | **`10.0 mm`** | Retracts halfway to clear gummy 6063 chips from `Ø5 mm` bore |
| **Op 2 `bottom` offset** | `-0.2 mm` (`-20.2 mm`) | **`-0.5 mm`** (`-20.5 mm`) | Clean breakthrough on `20 mm ± 0.2` extruded bar stock |

---

## 5. Generate (`Ctrl+G`) & 3D Simulation Verification

1. Press **`Ctrl+G`** and verify green status checks on every operation under **`Setups → Setup 1 → Operations (2)`**.
2. Run **`Simulate`** (`Tri-Dexel`) and confirm **`Verified Crash-Free`** with `0` collisions and green **`OK`** rows in **`Diagnostics & Log → Machinability & Tool Safety`**.

---

## 6. Post-Processing (GRBL / grblHAL ATC & LinuxCNC)

1. Open **`G-Code Preview`**, confirm the reduced compensated `F` feedrates on `G3` helical moves and the mid-depth `G0 Z1.000` chip-clearing retract inside Op 2, and export via **`Export G-Code…`** or **`Export All Setups…`**.
2. Keep continuous air blast (or light cutting oil / alcohol mist) aimed directly at the `Ø5 mm` bore while machining dry 6063 aluminum.

---

## 7. Headless CLI Equivalent (`chipcutter`)

```bash
# Run geometric + cutting-physics validation against Carvera Air and fail on any Critical snap/stall hazard
chipcutter validate job.chip.toml --strict-physics --machine carvera-air

# Emit machine-readable JSON physics telemetry (MRR, spindle load %, tip deflection um, safety factor)
chipcutter validate job.chip.toml --physics --json

# Verify volumetric Tri-Dexel simulation and build verified G-code
chipcutter simulate job.chip.toml --engine tri-dexel
chipcutter build job.chip.toml --output-dir ./gcode_out
```
