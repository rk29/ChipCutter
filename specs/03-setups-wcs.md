# 03 — Setups, Stock, & Work Coordinate Systems (WCS)

## Multi-Setup & Pure-2D Job Model
A ChipCutter project contains:
- Zero or more imported 3D **STEP CAD bodies** (`models`, optional in v2!),
- Zero or more 2D **Sketch Layers** (`sketches`: DXF/SVG imports, parametric 2D shapes, or text), and
- An ordered list of **Setups** (e.g., `Setup 1 — Top Op`, `Setup 2 — Flip Bottom Op`, or `Sheet Panel Routing`).

- Each Setup represents a single physical clamping orientation on the CNC machine.
- **Pure-2D Projects (`models = []` — *v2*):** Router, laser/engraver-style mill, front-panel, and signmaking projects require **no 3D STEP model**—a Setup defines physical sheet/plate stock (`Fixed Size` or `Relative to Sketch Bounds`) and machines directly from 2D `SketchLayer` geometry!
- Operations belong strictly to one Setup and inherit its WCS transform, stock geometry, and Z clearance planes.
- Setups can be post-processed individually (separate `.nc` files per clamping op, the default recommended workflow) or combined when using distinct work offsets (`G54`, `G55`, …) in a single multi-vise setup.

---

## Work Coordinate System (WCS) Definition

### 1. Orientation (Machine Axes relative to CAD Body / 2D Sheet)
Each Setup computes a rigid orthonormal frame `T_world_to_wcs` (`SE(3)`, right-handed `X × Y = Z`):
- **Orientation Modes:**
  - `Model / Sheet XY Orientation` *(Default for Pure-2D Jobs)*: Identity rotation (`WCS XYZ == World/Sheet XYZ`, `+Z` up).
  - `Select Z Axis & X Axis` *(Default interactive 3D STEP workflow)*:
    - Pick any planar face (sets `Z` parallel to face normal) or linear edge / cylinder axis (sets `Z` along axis), with a **Flip Z** toggle.
    - Pick any perpendicular linear edge or planar face to clock **X** (with a **Flip X** toggle); `Y` is derived automatically via `Z × X`.
  - `Select XY Plane`: Pick a planar face directly as the machining horizon and clock X in increments of `90°` or from an edge.

### 2. WCS Origin (`X0 Y0 Z0` Zero Point)
Once oriented, the WCS origin point can be anchored to:
- **Stock Box Point (27-point grid):** Any corner, edge midpoint, face center, or volumetric center of the computed stock bounding box (e.g., *Top Center*, *Top Front-Left Corner*, *Bottom Front-Left Corner* / spoilboard `Z0`—very common for CNC router sheet goods!).
- **Model / Sketch Bounds Box Point (27-point grid):** Same 27-point selector over the tight axis-aligned bounding box of the target CAD body (or active 2D Sketch artwork when `models = []`) in WCS space.
- **Selected CAD / Sketch Feature:** Any B-Rep vertex, circular edge center, cylindrical hole axis intersection, or 2D Sketch circle center / origin point.
- **Additional XYZ Offset:** Optional signed offset `(ΔX, ΔY, ΔZ)` applied on top of the anchor point.

### 3. Controller Work Offset Mapping
- Each Setup specifies an integer/named WCS slot:
  - `0` / `Default` (emits controller default or `G54`)
  - `1..6` -> `G54`..`G59`
  - `7..9` -> `G59.1`..`G59.3` (LinuxCNC extended offsets)

---

## Stock Definition
Stock is defined in the Setup's oriented WCS frame (`X`, `Y`, `Z` aligned with WCS axes):

### 1. Box Stock
- **Relative Offset Mode *(Default when 3D STEP model or non-empty 2D Sketch exists)*:**
  - Computes the exact tight bounding box of the target 3D B-Rep solid(s) in WCS axes—or, in a **pure-2D project** (`models = []`), the XY bounding box of the project's 2D `SketchLayer` entities paired with an explicit `Sheet Thickness (Z)` expression (default `"12.0 mm"` / `"0.5 in"`):
    - `Side Offset (+X, -X, +Y, -Y)` — symmetric or per-side (e.g., `2.0 mm` or `10.0 mm` router margin)
    - `Top Offset (+Z)` — facing allowance (e.g., `1.0 mm` on 3D milled stock, `0.0 mm` on pre-surfaced sheet goods)
    - `Bottom Offset (-Z)` — holding/vise stock (e.g., `5.0 mm` vise grip, `0.0 mm` on spoilboard)
- **Fixed Size Mode *(Recommended for standard CNC router sheets & pre-cut blanks)*:**
  - Explicit `Width (X)`, `Depth (Y)`, `Height / Thickness (Z)` stock dimensions (e.g., `"24 in"`, `"12 in"`, `"0.75 in"`), with model/sketch alignment within stock (`Center XY`, `Offset from Front-Left`, `Offset from Top/Bottom`).

### 2. Cylindrical Stock
- Axis aligned with WCS `Z` (or `X`/`Y`), defined either by relative radial + axial offsets around the model/sketch bounds or fixed `Diameter` and `Length` (for round bar stock or round wood/brass medallions).

### 3. Multi-Setup Volumetric Stock Carryover (`carryover_from_setup_id`)
- Any Setup after Setup 1 can reference an earlier Setup via `carryover_from_setup_id` (e.g., `Setup 2 — Flip Bottom Op (G55)` carrying over from `setup_1`).
- During toolpath generation and simulation, Setup `N`'s final volumetric `TriDexelStock` is rigidly transformed (`T_{N+1 -> N}`) into Setup `N+1`'s oriented WCS frame (using an exact zero-blur plane permutation/flip fast path for `90°`/`180°` flips and BVH re-voxelization for arbitrary oblique orientations).
- Op-2 roughing/facing/pocketing strategies (`Facing`, `Pocket2D`, `Adaptive2D`, `Adaptive3D`, `Waterline3D`) automatically skip already-machined air regions while preserving rest-material cut accuracy.

---

## 3D Workholding Fixtures & Shop-Floor Wizards (`Setup.fixtures`)
Each Setup stores an optional `fixtures: Vec<FixtureBody>` list (omitted when empty for zero diff on legacy projects) and `auto_avoid_fixtures_on_rapids: bool` (default `true`):
- **Parametric Hardware Primitives:**
  - `ViseJaws`: Two-jaw machinist vise along WCS `Y` or `X` with configurable width, thickness, height, grip depth, optional stepped soft-jaw shelves, and under-stock parallels auto-clamped to stock bounds.
  - `ToeClamp`: Strap/toe clamp with nose overhang, heel riser, outward yaw orientation, and T-slot stud/nut tower.
  - `Spoilboard`: Sacrificial MDF/tooling-plate bed beneath stock with configurable `allowed_overcut` margin (e.g., `0.5 mm`).
  - `BoxKeepOut` & `CylinderKeepOut`: Generic oriented rectangular blocks and vertical cylindrical keep-out pillars (dowel pins, bolt heads, cam clamps).
  - `ImportedModel`: External `.step`/`.stl`/`.obj`/`.3mf` fixture solids persisted with `ModelRole::Fixture` (bundled inside `.chipz` archives and automatically excluded from target-part gouge metrics).
- **Exact `f64` Resolution & Safe-Rapid Avoidance:** Every enabled fixture resolves into an exact Setup-WCS `TriMesh3D` + `FlatBvh` and 2D XY `RegionSet2D` keep-out footprint with configurable `clearance_margin`. When `auto_avoid_fixtures_on_rapids` is enabled, `chip-cam` automatically lifts `G0` rapid and stay-down links cleanly over enabled fixtures (accounting for flute, shank, and holder/collet envelopes).
- **Workholding Automation Wizards (`chip-cam/src/workholding_wizards.rs`):**
  - **Two-Sided Flip-Jig Dowel-Pin Wizard:** Places collision-free dowel-pin registration holes (`2` on-axis or `4` symmetric, with optional poka-yoke asymmetry) strictly outside the part 2D silhouette and strictly `180°`-flip invariant (`< 1e-9 mm`), emitting the `"Flip-Jig Registration Pins"` sketch, Op-1 pin drilling/boring op, optional `"Spoilboard Dowel Holes"` setup, paired `180°` Op-2 Setup (`G55`) with `carryover_from_setup_id`, and matching `CylinderKeepOut` pin bodies.
  - **Auto Soft-Jaw Generator Wizard:** Projects the exact `f64` clamping-band part silhouette with radial slip-fit clearance and sharp-corner dogbone reliefs onto fixed & movable vise jaws, emitting both a ready-to-machine `"Soft Jaws — Fixed & Movable"` CNC Setup (Facing + Pocket2D + Contour2D Finish) AND watertight 3D-printable binary `.stl` jaw meshes (`fixed_jaw.stl`, `movable_jaw.stl`) with optional counterbored mounting bolt patterns (`Kurt 6"`, `4" / Milo`, `Carvera`, or `Custom`).

---

## Setup Z-Height Hierarchy
To prevent rapid crashes while minimizing air-cutting time, every Setup defines standard reference planes along `+Z` (overridable per operation):

```text
  +Z (Spindle Up)
   ▲
   │  ═══════════════  Clearance Height   (Default: Stock Top + 10 mm) — First/last G0 rapid moves & cross-part jumps
   │  ───────────────  Retract Height     (Default: Stock Top + 5 mm)  — Local G0 retracts between cuts within an operation
   │  ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄  Feed Height        (Default: Top Height + 2 mm) — Transition from G0 rapid plunge to G1 plunge feedrate
   │  ┏━━━━━━━━━━━━━┓  Stock Top
   │  ┃  ┌───────┐  ┃  Model / Sketch Top
   │  ┃  │ Part  │  ┃
   │  ┃  └───────┘  ┃  Model / Cut Bottom
   │  ┗━━━━━━━━━━━━━┛  Stock Bottom
```

**Invariant Enforced at Validation:** `Clearance Height >= Retract Height >= Feed Height >= Operation Top Height >= Operation Bottom Height`. Any violation produces a clear validation error before toolpath generation.
