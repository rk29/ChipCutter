# 03 — Setups, Stock, & Work Coordinate Systems (WCS)

## Multi-Setup Job Model
A ChipCutter project contains one or more imported CAD bodies and an ordered list of **Setups** (e.g., `Setup 1 — Top Op`, `Setup 2 — Flip Bottom Op`).
- Each Setup represents a single physical clamping orientation on the CNC machine.
- Operations belong strictly to one Setup and inherit its WCS transform, stock geometry, and Z clearance planes.
- Setups can be post-processed individually (separate `.nc` files per clamping op, the default recommended workflow) or combined when using distinct work offsets (`G54`, `G55`, …) in a single multi-vise setup.

---

## Work Coordinate System (WCS) Definition

### 1. Orientation (Machine Axes relative to CAD Body)
CAD parts frequently arrive with `+Y` up or arbitrary assembly coordinates. Each Setup computes a rigid orthonormal frame `T_body_to_wcs` (`SE(3)`, right-handed `X × Y = Z`):
- **Orientation Modes:**
  - `Model Orientation`: Identity rotation (`WCS XYZ == CAD XYZ`).
  - `Select Z Axis & X Axis` *(Default interactive workflow)*:
    - Pick any planar face (sets `Z` parallel to face normal) or linear edge / cylinder axis (sets `Z` along axis), with a **Flip Z** toggle.
    - Pick any perpendicular linear edge or planar face to clock **X** (with a **Flip X** toggle); `Y` is derived automatically via `Z × X`.
  - `Select XY Plane`: Pick a planar face directly as the machining horizon and clock X in increments of `90°` or from an edge.

### 2. WCS Origin (`X0 Y0 Z0` Zero Point)
Once oriented, the WCS origin point can be anchored to:
- **Stock Box Point (27-point grid):** Any corner, edge midpoint, face center, or volumetric center of the computed stock bounding box (e.g., *Top Center*, *Top Front-Left Corner*, *Bottom Front-Left Corner* / spoilboard zero).
- **Model Box Point (27-point grid):** Same 27-point selector over the tight axis-aligned bounding box of the target CAD body in WCS space.
- **Selected CAD Point / Feature:** Any B-Rep vertex, circular edge center, or cylindrical hole axis intersection with Top/Bottom planes (critical for Op 2 flip setups probing a machined bore!).
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
- **Relative Offset Mode *(Default)*:**
  - Computes the exact tight bounding box of the target B-Rep solid(s) in WCS axes, then adds user margins:
    - `Side Offset (+X, -X, +Y, -Y)` — symmetric or per-side (e.g., `2.0 mm`)
    - `Top Offset (+Z)` — facing allowance (e.g., `1.0 mm`)
    - `Bottom Offset (-Z)` — holding/tab/vise stock (e.g., `5.0 mm`), with a "Round up total dimensions to increment" helper.
- **Fixed Size Mode:**
  - Explicit `Width (X)`, `Depth (Y)`, `Height (Z)` stock dimensions (e.g., `"4 in"`, `"6 in"`, `"0.5 in"`), with model alignment within stock (`Center XY`, `Offset from Front-Left`, `Offset from Top/Bottom`).

### 2. Cylindrical Stock
- Axis aligned with WCS `Z` (or `X`/`Y`), defined either by relative radial + axial offsets around the model bounds or fixed `Diameter` and `Length` (for round bar stock held in a 3-jaw chuck on the mill table).

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
   │  ┃  ┌───────┐  ┃  Model Top
   │  ┃  │ Part  │  ┃
   │  ┃  └───────┘  ┃  Model Bottom
   │  ┗━━━━━━━━━━━━━┛  Stock Bottom
```

**Invariant Enforced at Validation:** `Clearance Height >= Retract Height >= Feed Height >= Operation Top Height >= Operation Bottom Height`. Any violation produces a clear validation error before toolpath generation.
