# Milestone 95 Implementation Plan — In-Viewport Click-to-Measure Tool & Minimum-Radius / Reach Heatmap

## Goal

Give machinists direct geometric inspection inside the 3D viewport without switching back to CAD: an exact-`f64` **Click-to-Measure Tool** (`M` hotkey / Viewport HUD) for distances, hole diameters, Z depths, and face angles, paired with a cached **Minimum-Radius & Reach Heatmap** highlighting internal corners smaller than the active cutter and pockets/walls deeper than flute or stickout reach.

---

## Problem Diagnosis

1. **No In-Viewport Measurement Tool (`crates/chip-gui/src/app/viewport_controller.rs`, `viewport/overlays.rs`):**
   - Machinists verifying hole diameters, wall thicknesses, pocket depths, or chamfer angles before setting tool sizes and Z heights currently have to measure inside an external CAD app or estimate from the stock bounding box.
2. **No Visual Cutter-Fit or Reach Feedback:**
   - Tight internal vertical fillets ($r < R_{\text{tool}}$), narrow slots, and deep pockets exceeding `flute_length_mm` or `stickout_length_mm` are only discovered after generating a toolpath and inspecting leftover stock or collision diagnostics.

---

## Proposed Changes

### 1. Exact `f64` Geometry Measurement Kernel (`chip-cam` / `chip-cad`)

#### [NEW] `crates/chip-cam/src/measurement.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- Define UI-agnostic exact-`f64` measurement types expressed in active **Setup WCS** coordinates (respecting the `AGENTS.md` invariant: derive measurements from exact `f64` `CadDocument` B-Rep curves/surfaces and `SketchLayer` `ArcPolyline2D` entities, falling back to `f64`-promoted mesh vertices/planes only for STL/OBJ/3MF meshes):
  - `MeasureSnapTarget`:
    - `BRepVertex { pos_wcs: DVec3 }`
    - `BRepEdge { edge_id, midpoint_wcs: DVec3, length_mm: f64, arc_radius_mm: Option<f64>, arc_center_wcs: Option<DVec3> }`
    - `BRepPlanarFace { face_id, point_wcs: DVec3, normal_wcs: DVec3, z_wcs: f64, area_mm2: f64 }`
    - `BRepCylindricalFace { face_id, axis_origin_wcs: DVec3, axis_dir_wcs: DVec3, diameter_mm: f64, depth_mm: f64 }`
    - `SketchPoint { sketch_id, pos_wcs: DVec3 }` / `SketchCircle { sketch_id, center_wcs: DVec3, diameter_mm: f64 }`
    - `MeshPoint { pos_wcs: DVec3, normal_wcs: DVec3 }`
  - `MeasurementResult`:
    - **Single-Pick Inspection:** exact hole diameter & depth, arc/fillet radius, edge length, or planar face Z-height & area in Setup WCS.
    - **Two-Pick Measurement:**
      - `PointToPoint`: `distance_3d_mm`, `delta_wcs: DVec3` (`ΔX`, `ΔY`, `ΔZ`), `distance_xy_mm`.
      - `ParallelPlanes`: exact perpendicular gap (`ΔZ` when horizontal) + `distance_3d_mm`.
      - `AngledPlanes`: exact dihedral angle (`deg`, both acute and supplementary) + intersection hint.
      - `CylinderToCylinder`: exact center-to-center pitch (`2D XY` & `3D`) + both diameters + min wall thickness between bores!
      - `PointToPlane`: exact signed perpendicular height/depth.

---

### 2. Minimum-Radius & Flute/Stickout Reach Analyzer (`chip-cam`)

#### [NEW] `crates/chip-cam/src/reach_analysis.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- Implement a background-thread-safe, cacheable analyzer (`analyze_machinability_reach(cad_doc, sketches, setup, tool) -> ReachAnalysisResult`):
  - **Minimum-Radius / Corner-Fit Check:**
    - Scans B-Rep cylindrical concave vertical/near-vertical fillet faces (axis within `15°` of Setup `+Z`, concave normal pointing toward axis), sharp interior edge corners between vertical walls, and closed 2D Sketch concave corners/arcs.
    - Classifies each feature against `tool_r = 0.5 * tool.geometry.cutting_diameter_mm()`:
      - `TightCorner` (**Amber**): concave radius `r < tool_r - 0.02 mm` (cutter leaves fillet rest stock; reports exact `r`, leftover radial cusp, and suggested max cutter diameter `2 * r`).
      - `NarrowSlotBlocked` (**Crimson**): opposing wall gap or hole diameter `< tool_d + 0.02 mm` (cutter physically cannot enter).
  - **Flute-Length & Stickout-Reach Depth Check:**
    - Measures vertical Z depth from active Setup stock top (or local upper lip) to every machinable face/edge/hole bottom:
      - `ExceedsFluteLength` (**Amber**): depth `> flute_length_mm` (relieved neck check: OK if `neck_diameter_mm < cutting_diameter_mm` and within `neck_length_mm`; otherwise flags shank rubbing risk!).
      - `ExceedsStickout` (**Crimson**): depth `> stickout_length_mm` (collet/holder collision — unreachable without longer stickout or flip setup).
  - Outputs compact summary statistics (`min_internal_radius_mm`, `max_feature_depth_mm`, counts of amber/crimson regions) plus pre-built 3D highlight triangle/edge lists keyed by `(cad_revision, setup_hash, tool_hash)` so idle viewport frames perform zero recomputation.

---

### 3. Interactive Viewport Measure Mode & Heatmap Overlay (`chip-gui`)

#### [MODIFY] `crates/chip-gui/src/panels/mod.rs` (`ViewportPickMode::Measure`)
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/overlays.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/pipeline.rs`
#### [MODIFY] `crates/chip-gui/src/panels/command_palette.rs`
- **Click-to-Measure UX:**
  - Trigger via `M` hotkey, Viewport bottom HUD **`Measure`** pill, `View → Measure Tool (M)`, or `Ctrl+K` Command Palette (`Esc` exits cleanly).
  - Hovering highlights exact snap anchors (cyan crosshair dot for vertices/centers, ring for cylinders, outline for planar faces).
  - Clicking anchor 1 shows immediate single-entity readout; clicking anchor 2 draws a crisp 3D dimension line with end ticks and `ΔX` / `ΔY` / `ΔZ` projection legs + a compact floating **Measurement Card** (formatted in project display units `mm`/`in`) with **`Copy Value`**, **`Clear`**, and contextual **`Set Op Bottom Z`** helper when an Operation is focused.
- **Minimum-Radius & Reach Heatmap Overlay:**
  - Toggleable **`Reach / Min-R`** pill in the Viewport bottom HUD (and from Operation Inspector `[Tool]` / Machinability card), dispatching `analyze_machinability_reach` on a background `rayon` task (never blocking `egui`).
  - Renders translucent **Amber** (tight corner / flute-reach rub) and **Crimson** (blocked slot / stickout-unreachable) face & edge overlays in `overlay_pipeline` / `edge_pipeline` legible in both `Studio Dark` and `Daylight Light`, plus a compact top-right legend pill summarizing minimum internal radius and deepest feature depth.

---

## Verification Plan

1. **Exact Measurement & Reach Unit Tests (`chip-cam`):**
   - Synthetic STEP B-Rep bracket & 2D Sketch tests verifying exact `f64` hole diameter, hole-to-hole center pitch, parallel face `ΔZ` depth, `45°` bevel dihedral angle, and `1e-6 mm` tolerance accuracy.
   - Reach analyzer tests verifying a `R2.0 mm` concave pocket fillet flags `TightCorner` for a `Ø6 mm` (`R3.0`) cutter and clears for a `Ø3.175 mm` (`R1.5875`) cutter; verify `18 mm` pocket depth flags `ExceedsFluteLength` on a `12 mm LOC` cutter and `ExceedsStickout` on a `15 mm` stickout cutter while passing a relieved-neck cutter within `neck_length_mm`.
2. **Viewport & Headless `egui` Tests (`chip-gui`):**
   - Test `ViewportPickMode::Measure` state machine (single pick, two-point pick, reset, `Esc` cancel) and offscreen `wgpu` rendering of measurement leader lines and amber/crimson reach heatmap overlays in both `Studio Dark` and `Daylight Light`.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
