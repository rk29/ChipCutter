# Tutorial 07 — 3D Medial-Axis V-Carving, Two-Tool Flat Clearance, & Mated V-Inlays

## 1. Goal & What You'll Learn

**3D V-Carving** uses a sharp conical **V-Bit** (`T8`, `60°`) riding along the interior **Voronoi medial axis** of closed TrueType text or SVG artwork—automatically varying cutting depth `(X, Y, Z)` proportionally to stroke width and climbing smoothly into **razor-sharp 3D corners** that a cylindrical endmill can never cut.

In this tutorial you will start from an **empty Pure-2D project** (no 3D STEP model required!), author bold `"CHIP"` TrueType lettering, and learn how to:

- Generate **Full-Depth 3D Medial-Axis V-Carving** with simultaneous 3-axis corner climbs.
- Clamp wide letters to a clean flat floor (**`Flat Floor Limit`** / **`Flat Depth`**) and enable **`Two-Stage Flat Area Clearance`** (Stage 1 Flat Endmill bulk pocketing + Stage 2 Rest-Machined V-Bit bevels & tight corners).
- Create a second Setup (`G55`) to machine a mirrored **`Mated Male V-Inlay Plug`** (**`Start Depth`** bevel engagement + **`Bottom Glue Gap`** clearance) that drops tightly into the female plaque.
- Verify sloped conical material removal in **3D Simulation** and see **GRBL** automatically split a single Two-Tool V-Carve operation into two `.nc` files (`T3` clearing + `T8` V-bit).

---

## 2. Prerequisites & Starter Pure-2D Setup

No external CAD file is needed:

1. Launch ChipCutter (`chip-gui`) and click **`New Pure-2D Parametric Sketch`** on the **Welcome to ChipCutter** overlay.
2. In the left **CAM Browser**, click **`Stock & WCS`** under `Setup 1`:
   - In the right **Inspector**, switch to the **`Stock`** tab. Under **`Stock Geometry & Dimensions`**, change **`Stock Mode`** to **`Fixed-Size Box Stock`** (which displays **`Width (WCS X)`**, **`Depth (WCS Y)`**, and **`Height (WCS Z)`**), enter `120 mm`, `55 mm`, and `15 mm`, and under **`Stock Offsets`** confirm **`XY Alignment`** reads `Centered in XY` and set **`Top Allowance (+Z)`** = `0 mm`.
   - Switch to the **`WCS & Post`** tab in the right **Inspector**. Under **`WCS Origin (X0 Y0 Z0)`**, select **`Origin Reference`** = **`Stock 27-Pt`**, set **`Z Level:`** = **`Top`**, and click **`C`** in the **`Center Y`** row (so **`Active Anchor`** reads `Top Center`).
3. In the left **CAM Browser**, expand **`2D Sketches (1)`** and click the sketch layer. In the right **Inspector** (**2D Sketch Layer**), click **`Text`** under **`Interactive Authoring Tools` → `Quick Insert at (0, 0):`**, then in the **`TrueType & Hershey Typography`** card below set:
   - **`Text Content (multi-line):`** = `CHIP`
   - **`Font Family`** = **`DejaVu Sans Bold (Outline)`**
   - **`Cap Height`** = `24 mm`, **`Spacing`** = `1.5 mm`, **`Origin X`** = `0 mm`, **`Origin Y`** = `0 mm`, **`Alignment`** = **`Center`** / **`Middle`**.

---

## 3. Step-by-Step GUI Walkthrough

### Step 1 — Full-Depth 3D Medial-Axis V-Carving (`Setup 1 → Op 1`)

1. Click **`+ V-Carve`** in the **top toolbar** (or **`Operations → + V-Carve / Bevel Inlay`** in the **top menu bar**, or **`+ VCarve`** under `Setup 1` in the left **CAM Browser**). In the right **Inspector** **`Tool`** tab, confirm **`T8 — 6mm 60° Carbide Engraving V-Bit`** is selected in **`Active Cutter`**.
2. Open the **`Geometry`** tab in the right **Inspector** and click **`+ All`** on your sketch row inside the **`2D Sketch Layers & Quick Select`** card (ChipCutter automatically classifies closed outer letter contours and counter-islands like the inside of `'P'`).
3. Switch to the **`Passes`** tab in the right **Inspector**, uncheck **`Flat Floor Limit`** (**`Limit Max Depth`**) under **`V-Carve Strategy`** (it is checked by default), and press **`Ctrl+G`** (or click **`Generate`** in the **top toolbar**):

![Full-Depth 3D Medial-Axis V-Carving with Voronoi Corner Climbs](../images/tutorials/07-vcarve-inlays/01-vcarve-medial-axis.png)

Look closely at the 3D toolpath: the V-bit plunges deeper along the wide stems of `'C'`, `'H'`, `'I'`, and `'P'` and climbs cleanly upward in 3D `(X, Y, Z)` right out to every crisp corner vertex.

### Step 2 — Clamp `Flat Depth` & Enable Two-Stage Flat Area Clearance

For large bold lettering, full-depth V-carving can cut unnecessarily deep (or through thin wood). Clamping **`Flat Depth`** and pairing a flat endmill speeds up roughing dramatically and leaves a smooth flat pocket floor:

1. In the right **Inspector** **`Passes`** tab (`V-Carve Strategy` card), **first** check **`Flat Floor Limit`** (**`Limit Max Depth`**) so **`Flat Depth`**, **`V-Bit Stepover`**, and the **`Two-Stage Flat Area Clearance`** card appear below it, then set **`Flat Depth`** = `3.0 mm`.
2. In the **`Two-Stage Flat Area Clearance`** card:
   - **First** open **`Clearing Tool`** and select **`T3 — 1/8 in 2FL Carbide Flat`** (smaller than the stroke width so it fits inside the letters)—this reveals **`Stage 1 Pattern`** and **`Blend Allowance`** below it.
   - Keep **`Stage 1 Pattern`** = **`Contour-Parallel Offset`** (or **`2D Adaptive Trochoidal`**) and set **`Blend Allowance`** = `0.15 mm`.
3. Press **`Ctrl+G`**:

![Two-Stage Flat Area Clearance with T3 Flat Endmill and T8 V-Bit](../images/tutorials/07-vcarve-inlays/02-vcarve-two-stage-clearance.png)

ChipCutter generates both stages inside a single associative operation:
- **Stage 1 (`T3` Flat Endmill):** Pockets out the wide flat interiors down to `-3.0 mm` while staying safely inside the V-bit's sloped bevel margin.
- **Stage 2 (`T8` `60°` V-Bit):** Rest-machines only the angled `60°` sidewalls, tight corners, and narrow floor strips the flat endmill could not reach.

### Step 3 — Verify Female Cavity in 3D Simulation

Click **`Simulate`** in the **top toolbar** (which opens the bottom **`Simulation Timeline`** tab and runs ChipCutter's **analytical 3D conical V-bit sweeper**):

![3D Machined Stock Simulation of Female Two-Stage V-Carved Plaque](../images/tutorials/07-vcarve-inlays/03-vcarve-simulation-female.png)

Inspect the machined hardwood stock in the **3D CAD Viewport**: crisp `60°` beveled walls, razor-sharp corners, completely flat `-3.0 mm` letter floors, intact `'P'` counter-island, and **`Verified Crash-Free`** in **`Simulation Timeline`**.

### Step 4 — Generate a Mirrored Mated Male V-Inlay Plug (`Setup 2 [G55]`)

To create a contrasting wood inlay (for example, walnut `"CHIP"` inlaid into a maple plaque):

1. Click **`+ Setup`** at the top-right of the left **CAM Browser** (or **`Setup → New Setup`** in the **top menu bar**) to add **`Setup 2 — Op 2 [G55]`**. In the right **Inspector**, rename **`Setup Name`** to `Setup 2 — Male V-Inlay Plug`, configure matching `120 × 55 × 15 mm` **`Fixed-Size Box Stock`** (`Top Allowance (+Z)` = `0 mm`) on the **`Stock`** tab, and set **`Origin Reference`** = **`Stock 27-Pt`** → **`Top`** + **`Center Y` / `C`** (`Top Center`) on the **`WCS & Post`** tab.
2. Add a **`V-Carve / Bevel Inlay`** operation (**`+ V-Carve`** in the **top toolbar** or **`+ VCarve`** under `Setup 2` in the left **CAM Browser**), select **`T8 — 6mm 60° Carbide Engraving V-Bit`** in **`Tool`**, click **`+ All`** on your `"CHIP"` sketch in **`Geometry`**, and on the **`Passes`** tab check **`Flat Floor Limit`** (`Limit Max Depth`), set **`Flat Depth`** = `3.0 mm`, and select **`T3 — 1/8 in 2FL Carbide Flat`** in **`Clearing Tool`**.
3. Scroll to the **`Mated Male V-Inlay Plug`** card in **`Passes`**, click **`Male Plug`** on the **`Inlay Mode`** row **first** so the male inlay fields appear below it, then set:
   - **`Start Depth`** = `2.0 mm` — controls how deep the tapered male bevel seats into the `3.0 mm`-deep female cavity (`2.0 mm` wedge engagement leaves `1.0 mm` of bottom glue space inside the female pocket!).
   - **`Bottom Glue Gap`** = `1.0 mm` — extra clearance machined around the base of the male plug (`total plug height = Start Depth + Bottom Glue Gap = 3.0 mm`) so the plug seats tightly on its tapered `60°` walls and leaves a clean `1.0 mm` saw/flush-trim gap between the two boards before gluing.
   - **`Border Margin`** = `5.0 mm` — cleared frame width machined around the raised male inlay plug.
   - **`Mirror Artwork`** → keep **`Mirror X`** checked — automatically mirrors the artwork across `X = 0` so the flipped plug drops face-down into the female cavity in the correct reading orientation!
4. Press **`Ctrl+G`** and click **`Simulate`** in the **top toolbar** for `Setup 2`:

![Setup 2 Mirrored Male V-Inlay Plug Parameters and 3D Simulation](../images/tutorials/07-vcarve-inlays/04-vcarve-male-inlay-plug.png)

Notice the raised, mirrored `"CHIP"` prism standing proud of the cleared background pocket—ready to glue face-down into `Setup 1`, clamp, and flush-trim!

---

## 4. Key Parameters Reference

| Parameter | Location | Purpose & Recommended Starting Values |
|---|---|---|
| **`Flat Floor Limit`** (`Limit Max Depth`) & **`Flat Depth`** | `Right Inspector → Passes → V-Carve Strategy` | Unchecked = full-depth Voronoi medial-axis carving; checked = reveals **`Flat Depth`** (`2.0–4.0 mm`), **`V-Bit Stepover`**, and the **`Two-Stage Flat Area Clearance`** card. |
| **`V-Bit Stepover`** | `Right Inspector → Passes → V-Carve Strategy` | Shown when **`Limit Max Depth`** is checked. Radial stepover used when the V-bit clears tight flat floor corners (`0.25–0.5 mm`). |
| **`Clearing Tool`** & **`Stage 1 Pattern`** | `Right Inspector → Passes → Two-Stage Flat Area Clearance` | Card shown when **`Limit Max Depth`** is checked; pick a flat endmill (`T3` `1/8 in` or `T1` `6 mm`) to reveal **`Stage 1 Pattern`** (**`Contour-Parallel Offset`** or **`2D Adaptive Trochoidal`**) and **`Blend Allowance`**. |
| **`Blend Allowance`** | `Right Inspector → Passes → Two-Stage Flat Area Clearance` | Small radial margin (`0.1–0.2 mm`) keeping the Stage 1 flat endmill safely clear of the V-bit's finished bevel wall. |
| **`Inlay Mode`** (`Female Cavity` vs. `Male Plug`) | `Right Inspector → Passes → Mated Male V-Inlay Plug` | **`Female Cavity`** carves into the plaque; clicking **`Male Plug`** reveals **`Start Depth`**, **`Bottom Glue Gap`**, **`Border Margin`**, and **`Mirror Artwork`** (`Mirror X`). |
| **`Start Depth`** & **`Bottom Glue Gap`** | `Right Inspector → Passes → Mated Male V-Inlay Plug` | **`Start Depth`** (`1.5–2.5 mm`, always `< female Flat Depth`) sets bevel engagement; **`Bottom Glue Gap`** (`0.8–1.5 mm`) creates clamp/trim clearance. |
| **`Mirror Artwork`** (`Mirror X`) | `Right Inspector → Passes → Mated Male V-Inlay Plug` | Automatically mirrors the male plug horizontally so inverted text reads correctly once glued into the female plaque. |

---

## 5. Post-Processing: GRBL Automatic Two-File Split vs. LinuxCNC ATC

Click **`Stock & WCS`** under `Setup 1 [G54]` in the left **CAM Browser** to make `Setup 1` active, then click **`Post`** in the **top toolbar** (or select the bottom **`G-Code Preview`** tab) with **`GRBL 1.1 / FluidNC (Split by Tool)`** selected in **`Post Profile:`**:

![GRBL Split-by-Tool G-Code Output Splitting Stage 1 T3 and Stage 2 T8](../images/tutorials/07-vcarve-inlays/05-vcarve-gcode-grbl-split.png)

- **`GRBL 1.1 / FluidNC (Split by Tool)` (`grbl.post.toml`):** Even though Stage 1 and Stage 2 live inside a **single** V-Carve operation, ChipCutter automatically partitions them into two sequential files (`..._01_T3_...nc` for Stage 1 flat clearing, followed by `..._02_T8_...nc` for Stage 2 `60°` V-bit bevels) so manual-tool-change routers can re-zero `Z` cleanly between tools.
- **`LinuxCNC (RS274NGC)` (`linuxcnc.post.toml`) / `grblHAL / FluidNC (ATC / M6 Macro)` (`grblhal_atc.post.toml`):** Emits a single unified program containing `T3 M6 G43 H3` followed by `T8 M6 G43 H8`.

---

## 6. Headless CLI Equivalent (`chipcutter`)

Generate both the pure medial-axis and the two-setup V-Carve & Mated V-Inlay Pure-2D projects, simulate, and post-process headlessly:

```bash
# 1. Generate the Pure-2D Two-Stage V-Carve & Mated Male V-Inlay project (2 Setups: G54 + G55)
chipcutter generate-sample-step /tmp/07-vcarve-inlay.chip.toml \
  --tutorial 07-vcarve-inlay

# 2. Validate, generate toolpaths, and verify both Setups (0 crashes/gouges)
chipcutter validate --generate /tmp/07-vcarve-inlay.chip.toml
chipcutter simulate /tmp/07-vcarve-inlay.chip.toml --resolution standard

# 3. Build GRBL (auto-splits Stage 1 T3 + Stage 2 T8 files per Setup) and LinuxCNC ATC G-code
chipcutter build /tmp/07-vcarve-inlay.chip.toml -o /tmp/vcarve_grbl --post grbl
chipcutter build /tmp/07-vcarve-inlay.chip.toml -o /tmp/vcarve_linuxcnc --post linuxcnc
```

---

## Next Steps

- **Previous:** [Tutorial 06 — Edge Chamfering & 2D Curve Trace / Engraving](06-chamfer-curve-trace.md)
- **Next:** [Tutorial 08 — 3D Adaptive Roughing & Waterline / Z-Level Surfacing](08-3d-adaptive-waterline.md)
- **Index:** [All Tutorials](README.md)
