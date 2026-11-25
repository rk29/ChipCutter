# Tutorial 08 — 3D Adaptive Roughing & Waterline / Z-Level Surfacing (`STL` / `OBJ` / `3MF`)

## 1. Goal & What You'll Learn

When machining sculpted 3D parts—organic molds, domes, enclosures, and contoured reliefs—2.5D pocketing cannot follow curved walls or varying Z heights. ChipCutter's exact `f64` **Drop-Cutter & Push-Cutter** 3-axis surface kernel machines triangle meshes (**STL**, **OBJ**, and **3MF**) directly into gouge-free toolpaths with zero viewport-mesh approximation.

In this tutorial you will start from a bare binary STL mesh ([`08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) — an `R = 25 mm` hemispherical dome on a `70 × 70 × 8 mm` pedestal), and learn how to:

- Import a 3D mesh (`STL`/`OBJ`/`3MF`) and verify its **unit normalization**, triangle/vertex counts, and **watertightness diagnostics**.
- Draw a **2D `Setup XY` containment sketch** (`Dome Window`) to constrain 3D roughing strictly inside a chosen XY region.
- Rough bulk stock safely and rapidly with **3D Adaptive Roughing** (`Adaptive3D`) using coarse **`Rough Stepdown`**, sloped-terrace **`Fine Step-Up`**, **`Detect Flats`**, and controlled **`0.4 mm` radial / `0.25 mm` axial** stock-to-leave.
- Semi-finish and finish the sculpted dome with **3D Waterline / Z-Level** (`Waterline3D`) using **`Adaptive Stepdown`** (`Subdivide shallow Z bands`), **`Order by Level (Top-Down)`**, and **`0.0 mm`** stock-to-leave.
- Verify **0 collisions and 0 target-mesh gouges** in 3D Z-dexel simulation (`✓ Verified Crash-Free`) and post-process for **LinuxCNC (ATC)** and **GRBL (split by tool)**.

---

## 2. Prerequisites & Starter Model

Start with an **empty project** and load the watertight binary STL starter model included with this tutorial ([`08-sculpted-dome.stl`](assets/08-sculpted-dome.stl) — `70.000 × 70.000 × 33.000 mm`, `6,528 triangles`, `3,266 welded vertices`, `Watertight`):

- **From the Welcome Overlay or Menu:** Launch ChipCutter (`chip-gui`), click **`Import 3D Mesh (STL / OBJ / 3MF)…`** on the **Welcome to ChipCutter** overlay (or **`File → Import 3D Mesh (.stl / .obj / .3mf)…`** in the **top menu bar**, or the **`+`** / right-click menu on **`Models`** in the left **CAM Browser** → **`Import Mesh (.stl / .obj / .3mf)…`**), select `docs/tutorials/assets/08-sculpted-dome.stl`, verify **`Interpret coordinates as:`** = **`Millimeters (mm)`** in the **Import 3D Mesh (STL / OBJ / 3MF)** modal, and click **`Import 3D Mesh`**.
- **Or open directly from the terminal:**
  ```bash
  chip-gui docs/tutorials/assets/08-sculpted-dome.stl
  ```

---

## 3. Step-by-Step GUI Walkthrough

### Step 1 — Inspect Imported Mesh & Configure Setup Stock / WCS (`Setup 1 → Stock`)

1. Hover or click the mesh row under **`Models`** in the left **CAM Browser** (or check the right side of the **top menu bar**): ChipCutter confirms `Binary STL · 6528 tris · 3266 verts · Watertight`. Because binary STL files are unitless, ChipCutter defaults to **`Millimeters (mm)`** (`70.000 × 70.000 × 33.000 mm`), and you can override mesh units in the **Import 3D Mesh (STL / OBJ / 3MF)** modal or the right-panel **Model Inspector** whenever needed.
2. Click **`Setup 1`** under **`Setups`** in the left **CAM Browser**:
   - In the right **Inspector** **`WCS & Post`** tab (`WCS Origin (X0 Y0 Z0)` card), select **`Origin Reference`** = **`Stock 27-Pt`**, set **`Z Level:`** = **`Top`**, and click **`C`** in the **`Center Y`** row so **`Active Anchor`** shows `Top Center` (`Z = 0` sits on top of the billet, centered directly over the dome apex).
   - Switch to the **`Stock`** tab in the right **Inspector**, keep **`Stock Mode`** = **`Relative Box Stock`** under **`Stock Geometry & Dimensions`**, and under **`Stock Offsets`** (confirm **`Symmetric XY`** / `Mirror ±X / ±Y` is checked) set **`Side X Margin (±X)`** = `2 mm`, **`Side Y Margin (±Y)`** = `2 mm`, **`Top Allowance (+Z)`** = `1 mm`, and **`Bottom Holding (-Z)`** = `0 mm` (producing a `74.000 × 74.000 × 34.000 mm` stock billet):

![Imported Sculpted Dome STL with Watertightness Diagnostics and Relative Box Stock](../images/tutorials/08-3d-adaptive-waterline/01-mesh-import-stock.png)

### Step 2 — Draw a 2D Containment Sketch & Rough with 3D Adaptive (`Op 1 → Geometry / Passes`)

Before roughing, add a lightweight 2D containment boundary on the Setup XY plane so you can control exactly where the roughing cutter is allowed to travel:

1. Click **`+ Sketch`** in the **top toolbar** (or use the **`+`** / right-click menu on **`Sketches`** in the left **CAM Browser** → **`New 2D Sketch (Setup XY)`**). In the right **Inspector** (**2D Sketch Layer** → **`Sketch Anchor & Snap Grid`** card), set **`Sketch Name`** = **`Dome Window`**.
2. Under **`Interactive Authoring Tools` → `Quick Insert at (0, 0):`**, click **`Rect`**. In the **`Parametric Rectangle Editor`** card below, keep **`Anchor Mode`** = **`Center (X, Y)`** and set **`Position X`** = `0 mm`, **`Position Y`** = `0 mm`, **`Width`** = `68 mm`, **`Height`** = `68 mm`, and **`Fillet R`** = `4 mm` enclosing the dome and pedestal top flange.
3. With `Setup 1` **`Active`** in the left **CAM Browser**, click **`+ Op… → 3D Adaptive Roughing`** in the **top toolbar** (or **`Operations → + 3D Adaptive Roughing`** in the **top menu bar**) to add **`1. 3D Adaptive 1`**.
4. In the right **Inspector** **`Tool`** tab (`Cutter & Material Preset`), select **`T1 — 6mm 3FL Carbide Flat Endmill`** in **`Active Cutter`**.
5. Switch to the **`Geometry`** tab in the right **Inspector**:
   - Under **`Cutting Area / Machining Boundary`** at the top of **`Geometry`**, set **`Boundary Source`** = **`Selected Regions`** (which automatically expands **`Interactive 3D Pick Mode`** and **`2D Sketch Layers & Quick Select`** below it), **`Tool Containment`** = **`Tool Outside Boundary`**, and **`Boundary Offset`** = `0.5 mm`.
   - Under **`2D Sketch Layers & Quick Select`**, click **`+ All`** on the **`Dome Window`** row. Notice the live 3D viewport preview showing both the solid nominal boundary and the dashed effective tool-center boundary (alongside the live `XY` span and `Tool-Center Shift` summary in the card).
6. In the **`Heights`** tab (`Z-Height Planes & Offsets` card), set **`Bottom Ref`** = **`Model Top`** and **`Bottom Offset`** = `-25.0 mm` so roughing stops cleanly at the flat `Z = 8 mm` pedestal top flange (`33 mm - 25 mm = 8 mm`).
7. Switch to the **`Passes`** tab in the right **Inspector**:
   - Under **`Stock to Leave`** (`Adaptive3D` hides the unused `Multiple Depths` toggle because axial Z levels are controlled directly by **`Rough Stepdown`** and **`Fine Step-Up`**), set **`Radial Stock`** = `0.4 mm` and **`Axial Stock`** = `0.25 mm` to leave a clean, uniform finishing allowance over the dome and pedestal.
   - Under **`3D Adaptive Roughing`**, set **`Optimal Load (ae)`** = `1.8 mm` (`30%` of cutter diameter), **`Rough Stepdown`** = `5 mm`, **`Fine Step-Up`** = `1.5 mm`, and check **`Detect Flats`** (`Land on flat plateaus`).
8. Press **`Ctrl+G`** (or click **`Generate`** in the **top toolbar**) to generate the constant-load 3D trochoidal roughing toolpath:

![3D Adaptive Roughing Passes Tab and Trochoidal Roughing Toolpath with 2D Sketch Containment](../images/tutorials/08-3d-adaptive-waterline/02-adaptive3d-passes.png)

Notice how `Adaptive3D` clears coarse `5 mm` axial tiers first, then climbs upward in `1.5 mm` **`Fine Step-Up`** intermediate terraces to strip the stair-stepped stock smoothly off the curved hemisphere, and lands exactly on the horizontal pedestal plateau (`+0.25 mm` axial stock) thanks to **`Detect Flats`**.

### Step 3 — Finish the Dome with 3D Waterline / Z-Level (`Op 2 → Geometry / Passes`)

With bulk stock cleared down to a thin `0.4 / 0.25 mm` skin, add a ball-endmill waterline operation to slice clean concentric Z-level contours across the dome:

1. Click **`+ Op… → 3D Waterline / Z-Level`** in the **top toolbar** (or **`Operations → + 3D Waterline / Z-Level`** in the **top menu bar**).
2. In the right **Inspector** **`Tool`** tab, select **`T4 — 6mm 2FL Carbide Ball Endmill`** in **`Active Cutter`**.
3. Open the **`Geometry`** tab in the right **Inspector**: under **`Cutting Area / Machining Boundary`**, set **`Boundary Source`** = **`Model Silhouette`** (computes the true projected XY silhouette of the mesh), **`Tool Containment`** = **`Tool Outside Boundary`**, and **`Boundary Offset`** = `0.5 mm`, verifying the live 3D nominal + dashed effective tool-center boundary preview in the viewport.
4. In the **`Heights`** tab (`Z-Height Planes & Offsets` card), set **`Bottom Ref`** = **`Model Top`** and **`Bottom Offset`** = `-25.0 mm`.
5. Switch to the **`Passes`** tab in the right **Inspector**:
   - Under **`Depth Passes & Stock to Leave`**, keep **`Multiple Depths`** (`Enable Z Stepdowns`) checked so **`Maximum Stepdown`** is displayed, then set **`Maximum Stepdown`** = `1.5 mm`, **`Radial Stock`** = `0 mm`, and **`Axial Stock`** = `0 mm`.
   - Under **`3D Waterline / Z-Level Passes`**:
     - Keep **`Adaptive Stepdown`** (`Subdivide shallow Z bands`) checked **first** so **`Min Stepdown`** and **`Max Cusp Height`** are displayed, then set **`Min Stepdown`** = `0.4 mm` and **`Max Cusp Height`** = `0.05 mm`.
     - Check **`Detect Flats`** (`Land on flat plateaus`).
     - Keep **`Min Slope (°)`** = `0°` and **`Max Slope (°)`** = `90°` to finish the full dome in one operation (or narrow to `35°`–`90°` when pairing steep Waterline walls with shallow `Parallel3D` crests—covered in Tutorial 09).
     - Set **`Pass Ordering`** = **`Order by Level (Top-Down)`** and keep **`Cut Direction`** (`Climb Milling`) checked.
6. Press **`Ctrl+G`**:

![3D Waterline / Z-Level Passes Tab Showing Adaptive Shallow Z-Stepdown Contours on the Dome](../images/tutorials/08-3d-adaptive-waterline/03-waterline3d-passes.png)

Look closely at the contour spacing near the top of the hemisphere: on the steep lower flanks, Z levels step down at `1.5 mm`, while near the shallow dome apex **`Adaptive Stepdown`** automatically subdivides Z slices down toward `0.4 mm` so the residual scallop height stays below **`0.05 mm`** without wasting passes on steep walls.

---

## 4. Key Parameters Reference

| Parameter | Location | Purpose & Recommended Starting Values |
|---|---|---|
| **`Boundary Source`**, **`Tool Containment`**, & **`Boundary Offset`** | `Right Inspector → Geometry → Cutting Area / Machining Boundary` | Constrains XY travel to **`Stock Bounds`**, **`Model Bounds`** (rectangular model XY bounding box with independent signed `-X`/`+X`/`-Y`/`+Y` per-side offsets), true projected **`Model Silhouette`**, or **`Selected 2D Contours`** (**`Tool Inside Boundary`**, **`Tool Center on Boundary`**, or **`Tool Outside Boundary`** + signed **`Boundary Offset`**) with live 3D nominal + dashed effective tool-center boundary previews. |
| **`Avoid / Keep-Out Regions`** | `Right Inspector → Geometry → Avoid / Keep-Out Regions` | Optional per-operation or project-wide reusable **`SharedAvoidRegion`** exclusions (Setup fixtures, 3D faces/loops, or 2D sketch loops + **`Radial Clearance`** / **`Axial Clearance`**) subtracted directly from the cutting area and previewed in coral/red in the 3D viewport. |
| **`Optimal Load (ae)`** | `Right Inspector → Passes → 3D Adaptive Roughing` | Maximum radial engagement maintained by trochoidal peels. Start at **`0.20–0.35 × tool_d`** (`1.5–2.0 mm` for a `Ø6 mm` endmill in aluminum). |
| **`Rough Stepdown`** / **`Fine Step-Up`** | `Right Inspector → Passes → 3D Adaptive Roughing` | Coarse axial depth (`4–6 mm`) paired with intermediate upward terrace step-ups (`1.0–2.0 mm`) that eliminate heavy stair-steps before finishing (`Adaptive3D` hides the duplicate `Multiple Depths` toggle). |
| **`Detect Flats`** | `Right Inspector → Passes` (`3D Adaptive Roughing` & `3D Waterline / Z-Level Passes`) | Detects horizontal upward plateaus (`normal.z > 0.995`) directly from the mesh and injects exact Z levels (`plateau Z + Axial Stock`). |
| **`Radial Stock`** / **`Axial Stock`** | `Right Inspector → Passes` (`Stock to Leave` on `Adaptive3D`; `Depth Passes & Stock to Leave` on `Waterline3D`) | Roughing allowance (`0.3–0.5 mm` radial, `0.2–0.3 mm` axial for `Adaptive3D`; **`0.0 mm / 0.0 mm`** for finishing). |
| **`Adaptive Stepdown`**, **`Min Stepdown`**, **`Max Cusp Height`** | `Right Inspector → Passes → 3D Waterline / Z-Level Passes` | Check **`Adaptive Stepdown`** (`Subdivide shallow Z bands`) to reveal **`Min Stepdown`** and **`Max Cusp Height`** (`0.03–0.08 mm`). |
| **`Pass Ordering`** | `Right Inspector → Passes → 3D Waterline / Z-Level Passes` | **`Order by Level (Top-Down)`** completes each Z slice across the whole part before stepping down; **`Order by Depth / Island`** finishes each boss/pocket completely to minimize retracts. |

---

## 5. Generate (`Ctrl+G`) & 3D Simulation Verification

Press **`Ctrl+G`** (or click **`Generate`** in the **top toolbar**; **`Diagnostics & Log`** logs wall-clock generation time alongside estimated CNC cycle time), click **`Simulate`** in the **top toolbar** (opening the bottom **`Simulation Timeline`** tab; click the compact status chip or open **`Edit → Settings…` → `Simulation`** (`Ctrl+,`) to switch Engine/Detail Level or toggle **`Smooth Skinned Surfaces (Mesh Skinning)`**), confirm **`3D Stock`**, **`Color by Op`**, and **`3D Cutter`** are checked in the **`Simulation Timeline`** top bar, and scrub to `96%`:

![3D Z-Dexel Machined Stock Simulation Showing Adaptive Roughing and Smooth Waterline Dome Finish](../images/tutorials/08-3d-adaptive-waterline/04-simulation-verified.png)

ChipCutter's 5-rule simulation oracle checks every move against both the evolving 3D heightmap stock and the exact `MeshBvh` target surface:
- Confirm the smooth spherical contour carved by `Op 2` (`T4` Ø6 mm Ball) inside the roughing envelope cleared by `Op 1` (`T1` Ø6 mm Flat).
- Verify the green **`Verified Crash-Free`** badge in **`Simulation Timeline`** confirming **zero rapid-into-stock collisions**, **zero shank/holder interference**, and **zero target-mesh gouges**.

---

## 6. Post-Processing (LinuxCNC & GRBL)

Click **`Post`** in the **top toolbar** (or select the bottom **`G-Code Preview`** tab) with **`LinuxCNC (RS274NGC)`** selected in **`Post Profile:`**:

![LinuxCNC ATC G-Code Preview Showing Verified Multi-Tool T1 and T4 Program](../images/tutorials/08-3d-adaptive-waterline/05-gcode-preview.png)

- **`LinuxCNC (RS274NGC)`** (`linuxcnc`): Emits a single ATC `.ngc` file (`Tools: [1, 4]`, `✓ Sim Verified (0 Crashes)`) with `M6` tool change and `G43 H` length compensation transitioning from `T1` (`3D Adaptive Roughing`) to `T4` (`3D Waterline Finishing`).
- **`GRBL 1.1 / FluidNC (Split by Tool)`** (`grbl`): Automatically splits the setup into two sequential `.nc` programs (`01_T1` for `Adaptive3D`, `02_T4` for `Waterline3D`) with strict 80-character line guards for manual-tool-change machines.

---

## 7. Headless CLI Equivalent (`chipcutter`)

Generate the starter Sculpted Dome STL mesh and 2-operation `Adaptive3D` + `Waterline3D` project, inspect watertightness, validate, simulate, and post-process headlessly:

```bash
# 1. Generate the starter Sculpted Dome STL + seeded 2-op Adaptive3D/Waterline3D project
chipcutter generate-sample-step /tmp/08-sculpted-dome.stl \
  --tutorial 08-3d-adaptive-waterline --project-out /tmp/08-adaptive-waterline.chip.toml

# 2. Inspect mesh triangle count, bounding box, and watertightness
chipcutter inspect /tmp/08-sculpted-dome.stl

# 3. Validate, generate toolpaths, and verify 0 crashes/gouges headlessly
chipcutter validate --generate /tmp/08-adaptive-waterline.chip.toml
chipcutter simulate /tmp/08-adaptive-waterline.chip.toml --resolution standard

# 4. Post-process for LinuxCNC (single ATC .ngc) and GRBL (2 split .nc files: T1 + T4)
chipcutter build /tmp/08-adaptive-waterline.chip.toml -o /tmp/dome_linuxcnc --post linuxcnc
chipcutter build /tmp/08-adaptive-waterline.chip.toml -o /tmp/dome_grbl --post grbl
```

---

## Next Steps

- **Previous:** [Tutorial 07 — 3D Medial-Axis V-Carving, Two-Tool Flat Clearance, & Mated V-Inlays](07-vcarve-inlays.md)
- **Next:** [Tutorial 09 — 3D Surface Finishing: Parallel Raster, Constant-Scallop, 3D Rest, & Pencil Cleanup](09-3d-surface-finishing.md)
- **Index:** [All Tutorials](README.md)
