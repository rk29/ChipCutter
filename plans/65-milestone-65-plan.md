# Milestone 65 Implementation Plan — 3D Edge Deburring & Non-Planar Chamfering (`OperationKind::Deburr3D`)

## Goal

Add full **3D Edge Deburring & Non-Planar Chamfering** (`OperationKind::Deburr3D`) so curved 3D contours, stepped pocket rims, cylindrical intersections, sloped walls, and underside overhang edges come off the CNC machine cleanly broken/chamfered without manual filing—supporting **Chamfer Mill**, **Ball Endmill**, and **Lollipop Endmill** (including underside back-chamfering), exact `f64` B-Rep adjacent-face bisector offsets, automatic neighbor-wall collision trimming, and CPU/GPU Tri-Dexel verification.

---

## Non-Negotiable Invariants

- **Exact `f64` B-Rep Geometry:** 3D edge trajectories and adjacent-face normals (`n_1`, `n_2`) are computed strictly from `CadDocument`'s exact `f64` B-Rep curves and surfaces (never from `f32` viewport meshes).
- **Zero Adjacent-Wall Gouging:** Inside vertical corners, stepped bosses, and deep pockets must NEVER be gouged by the upper cone/sphere/shank of the deburring cutter; colliding ends of 3D edge chains are automatically trimmed back cleanly by `wall_clearance_mm` with smooth tangent arc lead-in/out!
- **Schema & Golden Parity:** `OperationKind::Deburr3D` and `StrategyParams::Deburr3D` are purely additive; all existing projects and Phase 1–5 golden G-code snapshots remain byte-for-byte unchanged.
- **Full Pipeline Integration:** Honors Setup WCS, 4th-axis indexed `3+1` orientation, Milestone 61 fixture-aware safe-rapid routing, and Milestone 58 engagement-aware feedrate optimization.

---

## Proposed Changes

### 1. `OperationKind::Deburr3D` & Parameter Model (`chip-cam`)

#### [MODIFY] `crates/chip-cam/src/operation.rs`
#### [NEW] `crates/chip-cam/src/params/deburr3d.rs`
#### [MODIFY] `crates/chip-cam/src/params/mod.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- Add `#[serde(rename = "deburr_3d")] Deburr3D` to `OperationKind` (`label = "3D Edge Deburr / Chamfer"`, `default_name = "3D Deburr"`, `is_3d_surface = true`).
- Define `Deburr3DParams`:
  - `chamfer_width: ParamLength` (default `0.5 mm` — constant edge break width along the edge!)
  - `tip_offset: ParamLength` (default `0.75 mm` — keeps a pointed chamfer mill cutting on its stronger flank rather than the fragile dead-center tip!)
  - `wall_clearance: ParamLength` (default `0.15 mm` — minimum clearance preserved from neighboring vertical walls / inside corners!)
  - `edge_extension: ParamLength` (default `0.0 mm` — optional tangent extension off open outer edges)
  - `auto_detect_edges: bool` (default `false` when explicit edges are selected; `true` when no edges are selected or toggled on!)
  - `min_dihedral_angle_deg: f64` (default `15.0°` — ignores smooth tangent seams when auto-detecting sharp edges)
  - `exclude_vertical_edges: bool` (default `true` — skips pure vertical side walls parallel to `±Z`)
  - `allow_underside_back_chamfer: bool` (default `false` — when `true` with a `LollipopEndmill`, enables machining downward-facing overhang rim edges from underneath!)
  - `climb_direction: bool` (default `true`)
  - `chord_tolerance_mm: f64` (default `0.005 mm`)
  - `multiple_passes: u32` (default `1`; `> 1` takes progressive radial/axial chamfer cuts for large bevels)
  - `spring_pass: bool` (default `false`)

---

### 2. Exact 3D B-Rep Edge Bisector Contact & Neighbor-Wall Collision Trimming (`chip-cam`)

#### [NEW] `crates/chip-cam/src/strategies/deburr3d.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
- **3D Edge Chain Extraction & Orientation:**
  - Collects explicitly selected `Edge` / `EdgeLoop` / `OpenEdgeChain` items from `Operation.selections`, OR (when `auto_detect_edges == true`) scans `CadDocument.edges` in Setup WCS using exact adjacent `FaceId` normals `(n_1, n_2)` from `TopologyGraph` to select convex exterior/rim edges (`dot(n_1, n_2) < cos(min_dihedral_angle)` and convex cross-product orientation, filtering out pure vertical edges when `exclude_vertical_edges == true` and filtering accessible top vs. underside edges by tool type!).
  - Chains connected edges into continuous ordered 3D polylines (`Vec<DeburrSample3D { pos_wcs, tangent, n1, n2, bisector_out, is_underside }>`) using exact `f64` curve evaluation with adaptive chord-sagitta subdivision (`<= chord_tolerance_mm`).
- **Exact Cutter-Specific 3D Contact Kinematics (`solve_deburr_tool_tip`):**
  - Given edge point `p`, unit outward bisector `b = normalize(n1 + n2)`, and chamfer plane at distance `d_bevel` inward along `-b` producing constant `chamfer_width`:
    - **Chamfer Mill (conical flank half-angle `α`):** Computes exact horizontal radial offset `r_contact` and tip height `z_tip` so the conical flute engages the bevel cleanly at `tip_offset` above the cutter tip, adapting smoothly across non-planar sloped/curved 3D edges!
    - **Ball Endmill (radius `R`):** Computes exact sphere-center offset `p_center = p_bevel + R * b_eff` (where `b_eff` is the bevel normal) and `z_tip = p_center.z - R`, guaranteeing exact spherical tangent contact on arbitrary 3D curves!
    - **Lollipop Endmill (radius `R`, neck `r_neck`):**
      - Top/accessible edges: identical exact spherical tangent contact!
      - Underside back-chamfer edges (`b.z < -0.15` when `allow_underside_back_chamfer == true`): solves exact upper-hemisphere tangent contact `p_center = p_bevel + R * b_eff` (with `z_tip = p_center.z - R`) and verifies the relieved neck (`r_neck`) clears the edge wall horizontally (`r_neck + wall_clearance < horizontal_reach`)!
- **Automatic Neighbor-Wall & Corner Collision Trimming (`chip-bvh`):**
  - Builds/reuses target `TriMesh3D` + `FlatBvh` (ignoring triangles strictly adjacent to the active bevel strip within `chamfer_width + 0.05 mm`).
  - Tests every station's full `AnalyticCutterEnvelope` (flute + neck + shank) against the non-adjacent target mesh inflated by `wall_clearance_mm`.
  - Splits chains cleanly across any colliding sub-segments (e.g., where a curved edge runs into a vertical boss wall), trimming both ends back to the exact last collision-free station (via binary refinement to `0.02 mm`!), and attaches smooth tangent arc lead-in / lead-out + safe rapid clearance bridges (fed through Milestone 61 `route_toolpath_around_fixtures` and Milestone 58 `optimize_toolpath_feedrates`!).
- **Intentional Edge-Break Metadata (`chip-sim` compatibility):**
  - Automatically populates `intentional_edge_break_mm = chamfer_width` on `SimOperationInput::from_operation` for `OperationKind::Deburr3D` so `chip-sim`'s Target Gouge verifier recognizes the intentional edge break cleanly while still catching any real wall gouge, shank rub, rapid crash, or fixture collision!

---

### 3. Operation Inspector UI & Browser Integration (`chip-gui`, `chip-project`)

#### [MODIFY] `crates/chip-gui/src/widgets/icons.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/mod.rs`
#### [NEW] `crates/chip-gui/src/panels/operation_inspector/passes_tab/deburr3d_card.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/browser.rs`
#### [MODIFY] `crates/chip-gui/src/app/{menus.rs, commands.rs}`
#### [MODIFY] `crates/chip-project/src/validation.rs`
- Add `IconKind::Deburr3D`, `+ 3D Deburr` actions in Operations menu / CAM Browser / Command Palette (`Ctrl+K`), and strict two-column **3D Deburr Passes** card with universal hover tooltips (`Chamfer Width`, `Tip Offset`, `Wall Clearance`, `Auto-Detect Sharp Edges`, `Min Edge Angle`, `Exclude Vertical Edges`, `Underside Back-Chamfer (Lollipop)`, `Passes`, `Spring Pass`, plus live inline compatibility banner when selecting tools!).
- Update `chip-project/src/validation.rs` to validate `Deburr3D` tool compatibility (Chamfer/Ball/Lollipop), `allow_underside_back_chamfer` requiring `LollipopEndmill`, and positive chamfer width / non-negative clearances.

---

## Verification Plan

1. **Exact 3D Edge Deburr Unit & Oracle Tests (`chip-cam`, `chip-sim`):**
   - **Curved 3D Wave / Sloped Boss Test:** Generate a STEP solid with a non-planar sloped top face, cylindrical boss intersection, and stepped pocket corner -> run `Deburr3D` with Chamfer Mill & Ball Endmill:
     - Assert toolpath Z tracks the exact 3D curve height smoothly (`chord_sag <= 0.01 mm`),
     - Assert automatic corner trimming stops cleanly before adjacent vertical walls with `wall_clearance_mm` preserved,
     - Assert `0` critical errors (`max_gouge <= 0.02 mm`) on both CPU & GPU Tri-Dexel simulation!
   - **Lollipop Underside Back-Chamfer Test:** Generate a stepped flange overhang -> run `Deburr3D` with `LollipopEndmill` (`allow_underside_back_chamfer = true`) -> assert the underside lip is cleanly chamfered in Tri-Dexel (`CPU` & `GPU` parity) with `0` neck/shank/target gouge errors!
   - **Auto-Detect Sharp Edges Test:** Verify `auto_detect_edges = true` selects all sharp convex edges while ignoring tangent fillet seams and vertical walls.
2. **Headless Vulkan Visual Verification:**
   - Capture headless Vulkan screenshots showing 3D non-planar deburr toolpaths tracing curved/stepped edges + Tri-Dexel machined bevels + strict two-column **3D Deburr** Inspector card; inspect via `read_file` and clean up `/tmp/m65_*` temp files.
3. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`, pure-CPU `cargo check --locked -p chip-stock` & `-p chip-cam`, `cargo deny check` (default + `flatpak-portal`), `cargo clippy` (all targets & feature sets), `WGPU_BACKEND=vulkan cargo test --locked --workspace`, `chip-sim --features gpu-compute`.
