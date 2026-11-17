# 04 — Tool Library & Cutting Presets

## Overview (`chip-tools`)
ChipCutter separates **Tool Geometry** (physical dimensions of a cutter) from **Cutting Presets** (feeds, speeds, and engagement targets per workpiece material).

### Library Scopes
1. **Global User Tool Libraries:** Stored as human-readable `.tools.toml` (or `.json`) files in the user's OS config directory (or any user-configured folder, e.g., synced via Git/Dropbox).
2. **Project-Embedded Tools:** Whenever an operation in a project uses a tool from a global library, a full snapshot of that tool and its active preset is copied into `.chip.toml`. Opening a project on another machine always reproduces identical toolpaths even if the global library changed or is absent, with an explicit "Sync from Global Library" action if desired.

---

## Supported v1 Cutter Types
All geometric dimensions are stored internally in `f64` millimeters (while preserving original display units `mm` or `in` so a `"1/4 in Flat Endmill"` displays cleanly as `0.25 in` rather than `6.35 mm`):

| Cutter Type | Key Geometric Parameters | Valid Operations |
|---|---|---|
| **Flat Endmill** | `diameter`, `flute_length`, `overall_length`, `shank_diameter`, `flutes` | Facing, Contour, Pocket, Helical Bore, Trace |
| **Bull-Nose (Corner Radius) Endmill** | `diameter`, `corner_radius` (`0 < R < D/2`), `flute_length`, `overall_length`, `shank_diameter`, `flutes` | Facing, Contour, Pocket, Helical Bore |
| **Ball Endmill** | `diameter` (`R = D/2`), `flute_length`, `overall_length`, `shank_diameter`, `flutes` | Contour, Pocket, Trace/Engrave *(+ v2 3D surfacing)* |
| **Chamfer / Spot / V-Bit** | `diameter`, `tip_diameter` (`>= 0`), `included_angle_deg` (e.g., `90°`, `60°`, `45°`), `flute_length`, `flutes` | Edge Chamfer, Trace/Engrave, Spot Drilling |
| **Twist Drill** | `diameter`, `point_angle_deg` (default `118°` or `135°`), `flute_length`, `overall_length`, `flutes` | Drilling (Simple, Peck) |
| **Face / Fly Cutter** | `cutting_diameter`, `outside_diameter`, `corner_radius`, `flute_length`, `inserts/flutes` | Facing, Open Contour |

### Tool Numbering & Machine Metadata
- `tool_number`: `u32` (`T1`, `T2`, …)
- `length_offset_number`: `u32` (defaults to `tool_number`, emitted as `H1`, `H2` in LinuxCNC `G43`)
- `diameter_offset_number`: `u32` (defaults to `tool_number`, `D1`, `D2`)
- `spindle_direction`: `CW (M3)` *(default)* | `CCW (M4)`
- `default_coolant`: `Disabled` | `Flood (M8)` | `Mist (M7)` | `Air Blast`
- `stickout_length`: Distance from collet/holder nose to tool tip (`mm`), checked during simulation for holder/collet clearance against Stock Top.

---

## Per-Material Cutting Presets
Each tool carries one or more named **Cutting Presets** (e.g., `Aluminum 6061 - Roughing`, `Aluminum 6061 - Finishing`, `Baltic Birch Plywood`, `Acetal / Delrin`, `Mild Steel 1018`):

| Parameter | Canonical Unit | Coupled Formula |
|---|---|---|
| `spindle_rpm` | `RPM` | `RPM = (SFM_m_min * 1000) / (π * D_mm)` |
| `surface_speed` | `m/min` (or `SFM`) | Coupled bidirectionally with `spindle_rpm` |
| `cutting_feedrate` | `mm/min` (or `IPM`) | `Feed = RPM * flutes * feed_per_tooth` |
| `feed_per_tooth` (`chipload`) | `mm/tooth` | Coupled bidirectionally with `cutting_feedrate` |
| `lead_in_out_feedrate` | `mm/min` | Defaults to `0.8 * cutting_feedrate` |
| `ramp_feedrate` | `mm/min` | Defaults to `0.5 * cutting_feedrate` |
| `plunge_feedrate` | `mm/min` | Defaults to `0.33 * cutting_feedrate` (or drill feed/rev `mm/rev * RPM`) |
| `default_stepdown` (`ap`) | `mm` | Axial depth of cut preset |
| `default_stepover` (`ae`) | `mm` | Radial width of cut preset |
| `coolant_override` | Optional `Coolant` | Overrides tool default coolant per material |

### Operation-Level Overrides
When a user picks a Tool + Preset inside an Operation, the operation copies those values as defaults. Any individual field in the Operation Inspector can be overridden locally (visually badged with an "overridden from preset" reset icon).

---

## Built-In SFM & Chipload Calculator
Inside both the Tool Library Editor and the Operation Inspector:
- Editing **Surface Speed** updates **Spindle RPM** (and vice versa), holding chipload constant (which proportionally updates linear feedrate) OR holding linear feedrate constant (user toggle, defaulting to preserving chipload).
- Editing **Chipload (`fz`)** updates **Cutting Feedrate (`Vf`)**, and vice versa.
- Optional **Radial Chip Thinning display** hint when `stepover < 0.5 * diameter`: displays effective chip thickness `h_ex` so users can compensate feedrates accurately on light radial passes.

---

## Fusion 360 JSON Tool Library Import
To avoid tedious manual re-entry of existing shop tooling, `chip-tools` includes a one-way importer (and CLI command `chipcutter tools import-fusion <file.json>`) supporting Fusion 360 `.json` (and `.tools` zipped JSON) exports:
- Maps `"flat end mill"`, `"bull nose end mill"`, `"ball end mill"`, `"chamfer mill"`, `"spot drill"`, `"drill"`, and `"face mill"` types.
- Preserves original tool `"unit"` (`"inches"` vs `"millimeters"`), flute counts, shaft/flute geometry, tool numbers, and all `"start-values"` material cutting presets.
- Unsupported exotic tool types (e.g., lollipop, dovetail, form mill) are skipped with a non-fatal import summary warning.
