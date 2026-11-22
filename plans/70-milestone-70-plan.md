# Milestone 70 Implementation Plan — Illustrated Tutorials 21 & 22: 3D Edge Deburring & Undercut / T-Slot / Dovetail / Lollipop Milling

## Goal

Publish **Tutorial 21 (`docs/tutorials/21-3d-edge-deburring-nonplanar-chamfering.md`)** and **Tutorial 22 (`docs/tutorials/22-undercut-tslot-dovetail-lollipop.md`)**—giving machinists comprehensive, step-by-step illustrated guides for every Phase 18 cutter geometry (`LollipopEndmill`, `SlotMill`, `DovetailCutter`) and strategy (`Deburr3D`, `SlotUndercut`, `Dovetail`, `Undercut3D`), complete with deterministic headless Vulkan screenshots and verified CLI commands.

---

## Proposed Changes

### 1. Tutorial 21 — 3D Edge Deburring & Non-Planar Chamfering (`docs/tutorials/21-3d-edge-deburring-nonplanar-chamfering.md`)

#### [NEW] `docs/tutorials/21-3d-edge-deburring-nonplanar-chamfering.md`
#### [NEW] `docs/images/tutorials/21-3d-edge-deburring-nonplanar-chamfering/*.png`
- Follow ChipCutter's 7-part tutorial structure (installed `chip-gui` & `chipcutter` commands):
  1. **Goal & What You'll Learn:** Following curved 3D B-Rep edges and sloped intersections (`Deburr3D`) with Chamfer Mill, Ball Endmill, and Lollipop Endmill; automatic sharp-edge detection; automatic `FlatBvh` inside-corner wall collision trimming; and underside back-chamfering with a Lollipop cutter.
  2. **Prerequisites & Starter Projects:** `docs/tutorials/assets/21-sloped-boss-deburr.step`, `docs/tutorials/assets/21-overhang-flange.step`, and `--sample curved-deburr-3d`.
  3. **Step-by-Step GUI Walkthrough (6 annotated Vulkan screenshots):**
     - **Step 1 — Creating a Lollipop Endmill in the Tool Library:** Opening Tool Library (`Tools`), adding a `Ø6 mm` Lollipop Endmill, reviewing the 2D schematic, `Sphere Join Height`, `Neck Ø`, and `Neck Length`.
     - **Step 2 — Adding a `3D Edge Deburr / Chamfer` Operation (`ChamferMill`):** Choosing explicit 3D curved/sloped B-Rep edges vs. enabling **`Auto-Detect Sharp Edges`** (`Min Edge Angle = 15°`, `Exclude Vertical Edges = true`), setting `Chamfer Width = 0.5 mm`, `Tip Offset = 0.75 mm`, and `Wall Clearance = 0.15 mm`.
     - **Step 3 — Observing Automatic Wall Collision Trimming & Corner Rolling:** Viewing the generated 3D arch toolpath stopping cleanly before the vertical stepped boss wall with smooth circular corner rolls and tangent lead-in/out arcs.
     - **Step 4 — Non-Planar Deburring with a Ball Endmill:** Comparing Chamfer Mill vs. Ball Endmill spherical tangent contact across variable slopes.
     - **Step 5 — Underside Back-Chamfering with a Lollipop Endmill:** Loading `21-overhang-flange.step`, enabling **`Underside Back-Chamfer (Lollipop)`**, and verifying the upper hemisphere chamfers the downward-facing overhang lip cleanly while the relieved neck clears the vertical wall.
     - **Step 6 — GPU Tri-Dexel Verification (`intentional_edge_break_mm`):** Simulating in `Tri-Dexel (Fine)` mode and verifying `0` critical errors (`Verified Crash-Free`) with crisp 3D beveled edges!
  4. **Key Parameters Reference:** Complete `Deburr3D` parameter table (`Chamfer Width`, `Tip Offset`, `Wall Clearance`, `Edge Extension`, `Auto-Detect`, `Min Dihedral Angle`, `Exclude Vertical`, `Underside Back-Chamfer`, `Multiple Passes`, `Spring Pass`).
  5. **Generate (`Ctrl+G`) & Tri-Dexel Verification Gate.**
  6. **Post-Processing (`GRBL` & `LinuxCNC`).**
  7. **Headless CLI Equivalent (`chipcutter`).**

---

### 2. Tutorial 22 — Undercut Milling: Keyways, T-Slots, Dovetails, & Lollipop 3D Undercuts (`docs/tutorials/22-undercut-tslot-dovetail-lollipop.md`)

#### [NEW] `docs/tutorials/22-undercut-tslot-dovetail-lollipop.md`
#### [NEW] `docs/images/tutorials/22-undercut-tslot-dovetail-lollipop/*.png`
- Follow ChipCutter's 7-part tutorial structure:
  1. **Goal & What You'll Learn:** Configuring `SlotMill`, `DovetailCutter`, and `LollipopEndmill` cutters; pre-slotting throats safely before undercut tools enter; machining DIN 650 / ASME **T-Slots** and **Blind Keyways** (`SlotUndercut`), **`60°` Internal Dovetail Slots & External Dovetail Rails** (`Dovetail`), and **Lollipop 3D Overhang Undercuts** (`Undercut3D`); and verifying overhangs in GPU Tri-Dexel.
  2. **Prerequisites & Starter Projects:** `docs/tutorials/assets/22-undercut-deburr-block.step`, `--sample undercut-deburr`, and `--sample dovetail-rail-keyway`.
  3. **Step-by-Step GUI Walkthrough (7 annotated Vulkan screenshots):**
     - **Step 1 — Undercut Cutters in the Tool Library:** Inspecting the 2D schematics and relieved-neck dimensions for `SlotMill` (`Ø12 × 4 mm`, `Neck Ø6 mm`), `DovetailCutter` (`Ø12 mm`, `60°`, derived `Top Ø`), and `LollipopEndmill` (`Ø6 mm`, `Neck Ø3 mm`).
     - **Step 2 — Mandatory Throat Pre-Clearing (Why Order Matters):** Facing & pre-slotting the `8.5 mm` T-slot and dovetail throats with a `Ø6 mm` Flat Endmill before any undercut cutter enters—and explaining the Tri-Dexel neck-collision safety check that catches missing pre-slots.
     - **Step 3 — DIN 650 M8 T-Slot (`SlotUndercut` + `BothEndsThrough`) & Blind Keyway (`EntryBoreCenter`):** Using the `T-Slot M8` preset, inspecting the multi-pass radial & axial stepovers, and verifying safe open-end / entry-bore lead-in and horizontal exit before every `+Z` retract!
     - **Step 4 — `60°` Internal Dovetail Slot & External Dovetail Rail (`Dovetail`):** Matching cutter `side_angle_deg`, configuring progressive lateral flank stepovers + spring pass, and verifying throat-centered horizontal exit.
     - **Step 5 — Lollipop 3D Overhang Undercut Finishing (`Undercut3D`):** Configuring `Min Overhang Angle`, cusp-height Z-stepdown, `Neck Clearance`, and `Lead-Out Distance` to carve the concave `+Y` undercut groove while automatically clipping unreachable ceiling recesses.
     - **Step 6 — GPU Tri-Dexel Undercut Simulation & Mid-Cut Inspection:** Scrubbing mid-cut to inspect the 3D revolved `SlotMill` & `DovetailCutter` riding cleanly inside their undercut channels, then viewing the final GPU Tri-Dexel Error Heatmap (`99%+` net-shape In-Tol, `0` critical errors!).
     - **Step 7 — Auto-Tri-Dexel `chipcutter build` & G-Code Review (`GRBL` & `LinuxCNC`):** Inspecting safe horizontal lead-in/lead-out moves at cutting depth prior to `G0 Z` retracts.
  4. **Key Parameters & Safety Guards Reference:** Tables covering `SlotUndercut`, `Dovetail`, and `Undercut3D` parameters plus ChipCutter's mandatory non-negotiable undercut safety rules.
  5. **Generate (`Ctrl+G`) & Tri-Dexel Verification Gate.**
  6. **Post-Processing (`GRBL` & `LinuxCNC`).**
  7. **Headless CLI Equivalent (`chipcutter generate-sample-step --undercut-block`, `validate`, `simulate --engine tri-dexel`, `build`).**

---

### 3. Screenshot Pipeline & Curriculum Index Updates

#### [MODIFY] `scripts/capture_tutorial_screenshots.sh`
#### [MODIFY] `docs/tutorials/README.md`
- Add `capture_21_3d_deburr()` and `capture_22_undercut_milling()` to `scripts/capture_tutorial_screenshots.sh` (supporting `--milestone 70`, `--milestone all`, and per-tutorial slugs).
- Update `docs/tutorials/README.md` curriculum table (rows 21 & 22) and starter assets table (`21-sloped-boss-deburr.step`, `21-overhang-flange.step`, `22-undercut-deburr-block.step`).
- Visually inspect every single PNG captured under `docs/images/tutorials/21-*/` and `22-*/` via `read_file`.

---

## Verification Plan

1. **CLI Command Verification:**
   - Run every CLI command documented in Tutorials 21 & 22 and verify exit code `0`, `0` critical errors, and valid `GRBL` + `LinuxCNC` G-code output.
2. **Visual & Markdown Audit:**
   - Visually inspect all captured PNGs for Tutorials 21 & 22 via `read_file` (confirming crisp revolved cutters, visible undercut cavities, clean two-column Inspector cards, and zero clipped text).
   - Verify all relative links & image paths in `docs/tutorials/{20,21,22}-*.md` and `docs/tutorials/README.md` resolve on disk.
3. **Script Syntax Check:**
   - `bash -n scripts/capture_tutorial_screenshots.sh`
