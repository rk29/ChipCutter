# 03 — Setups, Stock, Workholding, 4th-Axis Rotary, & WCS

## Multi-Setup & Pure-2D Job Model
A ChipCutter project contains:
- Zero or more imported 3D models (`models: Vec<ModelEntry>`), supporting both **STEP** (`.step`, `.stp`) and **triangle meshes** (`.stl`, `.obj`, `.3mf`), each carrying a semantic `role: ModelRole::{Part, Fixture}`, optional mesh `unit_override: Option<MeshUnitKind>`, and non-destructive `transform: ModelTransform` (applied in exact `f64` mm after load),
- Zero or more 2D **Sketch Layers** (`sketches`: DXF/SVG imports, parametric 2D shapes, or text), and
- An ordered list of **Setups** (e.g., `Setup 1 — Top Op`, `Setup 2 — Flip Bottom Op`, or `Sheet Panel Routing`).

Each `Setup` also stores:
- **`target_bodies: Vec<BodyId>`**: Optional subset of target part bodies included in this Setup's stock bounds and 3D machining mesh (empty = all `ModelRole::Part` bodies in the document).
- **`post_profile: String`**: Post-processor profile ID for this Setup (default `"grbl"`, e.g., `"grblhal_atc"`, `"linuxcnc"`, `"linuxcnc_4axis"`, `"grblhal_4axis"`, `"grbl_rotary_wrap"`, or a user clone).
- **Pure-2D Projects (`models = []`):** Router, engraver, front-panel, and signmaking projects require **no 3D model**—a Setup defines physical sheet/plate stock (`Fixed Size` or `Relative to Sketch Bounds`) and machines directly from 2D `SketchLayer` geometry.
- Operations belong strictly to one Setup and inherit its WCS transform, stock geometry, optional rotary axis, fixtures, and Z clearance planes.
- Setups can be post-processed individually (separate `.nc` files per clamping op, the default recommended workflow) or combined when using distinct work offsets (`G54`, `G55`, …) in a single multi-vise setup.

---

## Work Coordinate System (WCS) Definition

### 1. Orientation (Machine Axes relative to CAD Body / 2D Sheet)
Each Setup computes a rigid orthonormal frame `T_world_to_wcs` (`RigidFrame3` / `SE(3)`, right-handed `X × Y = Z`) via `WcsOrientationMode`:
- **`ModelOrientation`** *(Default for Pure-2D Jobs and mesh parts)*: Starts from world/sheet axes (`+Z` up, `+X` right), supporting `flip_z`, `flip_x`, `clock_90` (`0..3` × `90°`), and arbitrary **`extra_z_rot_deg`**.
- **`SelectZAndX`** *(Default interactive 3D STEP workflow)*:
  - Pick any planar face (sets `Z` along face normal), cylinder/cone/torus face (sets `Z` along axis), or linear/circular edge (`z_ref: WcsAxisRef`), with `flip_z`.
  - Optionally pick a perpendicular reference (`x_ref: Option<WcsAxisRef>`) to clock `X`, with `flip_x`, `clock_90`, and arbitrary **`extra_z_rot_deg`** (`Y` is derived via `Z × X`).
- **`SelectXYPlane`**: Pick a planar B-Rep face (`face_ref`) directly as the machining horizon with `flip_z`, optional straight-edge X reference (`x_edge_ref`), `clock_90`, and arbitrary **`extra_z_rot_deg`**.

### 2. WCS Origin (`X0 Y0 Z0` Zero Point)
Once oriented, the WCS origin point (`WcsOriginMode`) can be anchored to:
- **`StockBoxPoint` (27-point `BoxAnchor27` grid):** Any corner, edge midpoint, face center, or volumetric center of the computed stock bounding box (e.g., *Top Center*, *Top Front-Left*, *Bottom Front-Left* / spoilboard `Z0`).
- **`ModelBoxPoint` (27-point `BoxAnchor27` grid):** Same 27-point selector over the tight axis-aligned bounding box of the target CAD/mesh body (or active 2D Sketch virtual sheet when `models = []`) in WCS space.
- **`SelectedCadFeature` (3D B-Rep only):** Anchors WCS origin to a 3D B-Rep `WcsFeaturePointRef`:
  - `EdgeStart(SelectionBinding)` / `EdgeMidpoint(SelectionBinding)` / `EdgeEnd(SelectionBinding)`
  - `CircleCenter(SelectionBinding)`
  - `HoleAxisCenter { hole_face, top_rim }`
  - Attempting to use `SelectedCadFeature` (or B-Rep orientation modes) on a Pure-2D project with no 3D B-Rep bodies returns `CamError::Pure2dRequiresBoxWcs`.
- **Additional XYZ Offset (`additional_offset_wcs: [ParamLength; 3]`):** Optional signed offset `(ΔX, ΔY, ΔZ)` applied on top of the anchor point (supports expressions referencing `stock_x`, `stock_y`, `stock_z`).

### 3. Controller Work Offset Mapping (`WorkOffsetSlot`)
- `Default` (`0`, controller default)
- `G54`..`G59` (slots `1..6`)
- `G59.1`..`G59.3` (slots `7..9`, LinuxCNC extended offsets)

---

## Stock Definition (`StockConfig`)
Stock is defined in the Setup's oriented WCS frame (`X`, `Y`, `Z` aligned with WCS axes):

### 1. Box Stock
- **`RelativeBox`** *(Default when a 3D model or non-empty 2D Sketch exists)*:
  - Computes the exact tight bounding box of the target 3D solid/mesh bodies in WCS axes—or, in a **pure-2D project** (`models = []`), the XY bounding box of visible Setup 2D `SketchLayer` entities paired with `sheet_thickness_z` (**default `12.0 mm`**):
    - `side_pos_x`, `side_neg_x`, `side_pos_y`, `side_neg_y` (`symmetric_xy` toggle, default `2.0 mm`)
    - `top_offset_z` — facing allowance (default `1.0 mm`)
    - `bottom_offset_z` — holding/vise stock (default `0.0 mm`)
    - **`round_increment_mm: Option<f64>`** — optional stock-size rounding increment (rounds computed box extents up to standard bar/plate increments)
- **`FixedBox`** *(Recommended for standard CNC router sheets & pre-cut blanks)*:
  - Explicit `width_x`, `depth_y`, `height_z` stock dimensions (e.g., `"24 in"`, `"12 in"`, `"0.75 in"`), with:
    - `xy_align: FixedBoxXyAlign::{CenterXy, OffsetFromFrontLeft { dx, dy }}`
    - `z_align: FixedBoxZAlign::{OffsetFromTop { top_allowance }, OffsetFromBottom { bottom_allowance }, CenterZ}`

### 2. Cylindrical Stock
- **`RelativeCylinder` / `FixedCylinder`:** Axis aligned with WCS `Z`, `X`, or `Y` (`StockAxis`), defined either by relative radial + axial (`top_offset`, `bottom_offset`) offsets around the model/sketch bounds or fixed `diameter`, `length`, and `axial_top_offset`.

### 3. Multi-Setup Volumetric Stock Carryover (`carryover_from_setup_id`)
- Any Setup after Setup 1 can reference an earlier Setup via `carryover_from_setup_id` (e.g., `Setup 2 — Flip Bottom Op (G55)` carrying over from `setup_1`).
- During toolpath generation and simulation, Setup `N`'s final volumetric `TriDexelStock` is rigidly transformed (`T_{N+1 -> N}`) into Setup `N+1`'s oriented WCS frame (using an exact zero-blur plane permutation/flip fast path for `90°`/`180°` flips and BVH re-voxelization for arbitrary oblique orientations).
- Downstream consumers (`Facing`, `Pocket2D`, 2D Adaptive, `VCarve` flat-area clearance, `Adaptive3D`, and `Waterline3D`—including rotated **`Indexed3Plus1`** operations, which transform the carryover stock into each indexed angle's virtual Setup frame) automatically skip already-machined air regions while preserving rest-material cut accuracy.

---

## 4th-Axis Rotary Setup Configuration (`Setup.rotary: RotaryConfig`)
Every Setup carries an optional `rotary: RotaryConfig` (`crates/chip-cam/src/rotary.rs`, omitted from TOML when disabled):
- **Drive Mode (`mode: RotaryDriveMode`):**
  - `Disabled3Axis` *(default)* — Standard 3-axis `X/Y/Z` setup.
  - `TwoLinearPlusRotary` — Centerline-locked rotary mode driving two linear axes + rotary (`Z+X+A` or `Z+Y+B`) for `Rotary2Linear` surfacing and `OperationRotaryMode::WrapCylinder` 2D wrapping.
  - `Indexed3Plus1` — Positional `3+1` multi-face indexing between 2.5D/3-axis operations (`OperationRotaryMode::Indexed3Plus1 { angle_deg }`, running any standard strategy in a virtual rotated `ResolvedSetup` with automatic safe-clearance-radius retracts).
  - `Simultaneous4Axis` — Full simultaneous 4-axis milling operating `X + Y + Z + Rotary` together (`Rotary4AxisSim`).
- **Axis, Polarity, & Unwinding:**
  - `axis: RotaryAxisKind::{XAxis, YAxis, ZAxis}` (default `XAxis` -> `A`, `YAxis` -> `B`, `ZAxis` -> `C`), plus optional `axis_letter_override: Option<char>`.
  - `direction: RotaryDirection::{RightHandCcw, LeftHandCw}` (default ISO right-hand rule `RightHandCcw`).
  - `unwind_mode: RotaryUnwindMode::{ContinuousMultiTurn, WrapShortestPath, Modulo0To360}` controlling multi-turn helical rollover vs shortest `±180°` path vs `[0°, 360°)` wrapping.
- **Centerline, Clearance, & Chuck Geometry:**
  - `center_offset_wcs: [ParamLength; 2]` — Perpendicular WCS offset of the rotary centerline along the two non-rotary axes (`[Y, Z]` for `XAxis`, `[X, Z]` for `YAxis`, `[X, Y]` for `ZAxis`).
  - `clearance_radial_offset: ParamLength` — Extra radial margin (default `10.0 mm`) added outside the stock's outer radius (`stock_outer_radius_mm`) to derive safe `clearance_radius_mm` for index rotations.
  - **Chuck Keep-Out Collar:** `chuck_end_positive: bool` (`false` = `-u` end, `true` = `+u` end), `chuck_length: ParamLength` (default `20.0 mm`), and `chuck_diameter: ParamLength` (default `65.0 mm`), used for viewport gizmos, rotary axial bounds, and collision avoidance.
- **Setup Expression Variables:** When rotary is enabled, `ResolvedSetup::eval_context` injects **`rotary_radius`** (`stock_outer_radius_mm`) and **`rotary_clearance_r`** (`clearance_radius_mm`) alongside `stock_x`, `stock_y`, `stock_z`, `model_top_z`, and `model_bottom_z`.

---

## 3D Workholding Fixtures & Shop-Floor Wizards (`Setup.fixtures`)
Each Setup stores an optional `fixtures: Vec<FixtureBody>` list (omitted when empty) and `auto_avoid_fixtures_on_rapids: bool` (default `true`). Every `FixtureBody` carries `id`, `name`, `enabled`, `clearance_margin: ParamLength` (default `1.0 mm`), **`allowed_overcut: ParamLength`** (sacrificial overcut allowance living on `FixtureBody`, default `0.0 mm`—and **`0.5 mm`** on `FixtureBody::default_spoilboard`), optional `color_rgba`, and `kind: FixtureKind`:
- **Parametric Hardware Primitives:**
  - `ViseJaws`: Two-jaw machinist vise along WCS `Y` or `X` with configurable width, thickness, height, grip depth, optional stepped soft-jaw shelves, and under-stock parallels auto-clamped to stock bounds.
  - `ToeClamp`: Strap/toe clamp with nose overhang, heel riser, outward yaw orientation, and T-slot stud/nut tower.
  - `Spoilboard`: Sacrificial MDF/tooling-plate bed beneath stock (`z_top_ref`, `z_offset`, `thickness`, `margin_xy`; paired with `FixtureBody::allowed_overcut = 0.5 mm` by default).
  - `BoxKeepOut` & `CylinderKeepOut`: Generic oriented rectangular blocks and vertical cylindrical keep-out pillars (dowel pins, bolt heads, cam clamps).
  - `ImportedModel`: External `.step`/`.stl`/`.obj`/`.3mf` fixture solids persisted with `ModelRole::Fixture` (bundled inside `.chipz` archives and automatically excluded from target-part bounds and gouge metrics).
- **Exact `f64` Resolution & Safe-Rapid Avoidance:** Every enabled fixture resolves into an exact Setup-WCS `TriMesh3D` + `FlatBvh` and 2D XY `RegionSet2D` keep-out footprint with `clearance_margin`. When `auto_avoid_fixtures_on_rapids` is enabled, `chip-cam` automatically lifts `G0` rapid and stay-down links cleanly over enabled fixtures (accounting for flute, shank, and holder/collet envelopes).
- **Workholding Automation Wizards (`crates/chip-cam/src/workholding_wizards.rs`):**
  - **Two-Sided Flip-Jig Dowel-Pin Wizard:** Places collision-free dowel-pin registration holes (`2` on-axis or `4` symmetric, with optional poka-yoke asymmetry) strictly outside the part 2D silhouette and strictly `180°`-flip invariant (`< 1e-9 mm`), emitting the `"Flip-Jig Registration Pins"` sketch, Op-1 pin drilling/boring op, optional `"Spoilboard Dowel Holes"` setup, paired `180°` Op-2 Setup (`G55`) with `carryover_from_setup_id`, and matching `CylinderKeepOut` pin bodies.
  - **Auto Soft-Jaw Generator Wizard:** Projects the exact `f64` clamping-band part silhouette with radial slip-fit clearance and sharp-corner dogbone reliefs onto fixed & movable vise jaws, emitting both a ready-to-machine `"Soft Jaws — Fixed & Movable"` CNC Setup (Facing + Pocket2D + Contour2D Finish) and watertight 3D-printable binary `.stl` jaw meshes (`fixed_jaw.stl`, `movable_jaw.stl`) with optional counterbored mounting bolt patterns (`Kurt 6"`, `4" / Milo`, `Carvera`, or `Custom`).

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

**Invariant Enforced at Validation:** `Clearance Height >= Retract Height >= Feed Height >= Operation Top Height >= Operation Bottom Height`. Any violation produces a clear `CamError::HeightHierarchyViolation` before toolpath generation.
