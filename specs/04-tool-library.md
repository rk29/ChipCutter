# 04 — Tool Library & Cutting Presets

## Overview (`chip-tools`)
ChipCutter separates **Tool Geometry** (physical dimensions of a cutter) from **Cutting Presets** (feeds, speeds, and engagement targets per workpiece material).

### Library Scopes
1. **Global User Tool Libraries:** Stored as human-readable `.tools.toml` (or `.json`) files in the user's OS config directory (or any user-configured folder, e.g., synced via Git/Dropbox).
2. **Project-Embedded Tools:** Whenever an operation in a project uses a tool from a global library, a full snapshot of that tool and its active preset is copied into `.chip.toml` (including secondary `clearing_tool_id` flat endmills used by two-stage V-Carving). Opening a project on another machine always reproduces identical toolpaths even if the global library changed or is absent, with an explicit "Sync from Global Library" action if desired.

---

## Supported Cutter Types
All geometric dimensions are stored internally in `f64` millimeters (while preserving original display units `mm` or `in` so a `"1/4 in Flat Endmill"` displays cleanly as `0.25 in` rather than `6.35 mm`):

| Cutter Type | Key Geometric Parameters | Valid Operations |
|---|---|---|
| **Flat Endmill** | `diameter`, `flute_length`, `overall_length`, `shank_diameter`, `flutes` | Facing, Contour, Pocket, **2D Adaptive**, Helical Bore, Trace, **V-Carve Flat Clearance**, Keyway / Pre-Slot (`SlotUndercut`), 3D Surface & Rotary |
| **Bull-Nose (Corner Radius) Endmill** | `diameter`, `corner_radius` (`0 < R < D/2`), `flute_length`, `overall_length`, `shank_diameter`, `flutes` | Facing, Contour, Pocket, **2D Adaptive**, Helical Bore, 3D Surface & Rotary |
| **Ball Endmill** | `diameter` (`R = D/2`), `flute_length`, `overall_length`, `shank_diameter`, `flutes` | Contour, Pocket, Trace/Engrave, **3D Edge Deburr (`Deburr3D`)**, 3D Surface & Rotary |
| **Chamfer / Spot / V-Bit / Engraving Bit** | `diameter`, `tip_diameter` (`>= 0`, `0` for sharp V-bit or `> 0` for flat-tip engraving cutter), `included_angle_deg` (e.g., `90°`, `60°`, `45°`, `30°`), `flute_length`, `flutes` | **V-Carve**, Edge Chamfer, **3D Edge Deburr (`Deburr3D`)**, Trace/Engrave, Spot Drilling |
| **Twist Drill** | `diameter`, `point_angle_deg` (default `118°` or `135°`), `flute_length`, `overall_length`, `flutes` | Drilling (Simple, Peck, Chip-Break) |
| **Face / Fly Cutter** | `cutting_diameter`, `outside_diameter`, `corner_radius`, `flute_length`, `inserts/flutes` | Facing, Open Contour |
| **Thread Mill** | `diameter`, `neck_diameter` (`< diameter`), `neck_length`, `pitch`, `teeth_count` (`1` single-point or `> 1` multi-form), `thread_angle_deg` (default `60°`), `flute_length`, `shank_diameter`, `flutes` | **Helical Thread Milling (`ThreadMill`)** (Internal & External) |
| **Lollipop Endmill (Spherical Undercut)** | `diameter` (`R = D/2`), `neck_diameter` (`< diameter`), `neck_length`, `flute_length` (`R + √(R² − r_neck²)`), `overall_length`, `shank_diameter`, `flutes` | **3D Edge Deburr (`Deburr3D`, incl. Underside Back-Chamfer)**, **3D Lollipop Undercut (`Undercut3D`)**, 3D Surface Finishing |
| **Slot / T-Slot / Keyway Mill** | `diameter`, `cutting_thickness`, `corner_radius`, `neck_diameter` (`< diameter`), `neck_length`, `overall_length`, `shank_diameter`, `flutes` | **Slot / Keyway / T-Slot Undercut (`SlotUndercut`)** |
| **Dovetail Cutter** | `diameter`, `side_angle_deg` (e.g., `45°`, `60°`), `cutting_height`, `corner_radius`, `neck_diameter` (`< D_top`), `neck_length`, `overall_length`, `shank_diameter`, `flutes` | **Dovetail Slot & Rail Milling (`Dovetail`)** |

All 10 cutter geometries share a single unified `AnalyticCutterEnvelope` kernel in `chip-tools` supporting both monotonic top-down profiles and non-monotonic relieved-neck undercut heads (`axial_z_interval_at_radius`, Minkowski radial/axial stock dilation, and `dovetail_top_diameter_mm` / `lollipop_sphere_height_mm` analytic helpers). Starter shop presets for the relieved-neck undercut trio (`T13` 6 mm Lollipop, `T14` 12×4 mm T-Slot Mill, `T15` 12 mm 60° Dovetail) are exposed via `starter_undercut_tools()` without shifting `T1`–`T10` starter indices.

### Tool Numbering & Machine Metadata
- `tool_number`: `u32` (`T1`, `T2`, …)
- `length_offset_number`: `u32` (defaults to `tool_number`, emitted as `H1`, `H2` in LinuxCNC `G43`)
- `diameter_offset_number`: `u32` (defaults to `tool_number`, `D1`, `D2`)
- `spindle_direction`: `CW (M3)` *(default)* | `CCW (M4)`
- `default_coolant`: `Disabled` | `Flood (M8)` | `Mist (M7)` | `Air Blast`
- `stickout_length`: Distance from collet/holder nose to tool tip (`mm`), checked during simulation for holder/collet clearance against Stock Top.

---

## Per-Material Cutting Presets
Each tool carries one or more named **Cutting Presets** (e.g., `Aluminum 6061 - Adaptive Roughing`, `Aluminum 6061 - Slotting/Pocket`, `Aluminum 6061 - Finishing`, `Baltic Birch Plywood`, `Hardwood Maple / Walnut - V-Carve`, `Acetal / Delrin`, `Mild Steel 1018`):

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
| `default_stepover` (`ae`) | `mm` | Radial width of cut preset (contour-parallel / zig-zag) |
| `optimal_load` (`ae_adaptive`, *v2*) | `Option<mm>` | Recommended constant radial engagement for **2D Adaptive** (defaults to `0.20 * diameter` when unset) |
| `coolant_override` | Optional `Coolant` | Overrides tool default coolant per material |

### Operation-Level Overrides
When a user picks a Tool + Preset inside an Operation, the operation copies those values as defaults. Any individual field in the Operation Inspector can be overridden locally (visually badged with an "overridden from preset" reset icon).

---

## Built-In SFM, Chipload, & Radial Chip Thinning Calculator
Inside both the Tool Library Editor and the Operation Inspector:
- Editing **Surface Speed** updates **Spindle RPM** (and vice versa), holding chipload constant (which proportionally updates linear feedrate) OR holding linear feedrate constant (user toggle, defaulting to preserving chipload).
- Editing **Chipload (`fz`)** updates **Cutting Feedrate (`Vf`)**, and vice versa.
- **Radial Chip Thinning (`h_ex`):** When radial engagement `ae < 0.5 * diameter` (especially during **2D Adaptive Clearing**), effective chip thickness thins by $h_{\text{ex}} = f_z \cdot \frac{2\sqrt{a_e(D - a_e)}}{D}$. Both the calculator HUD and Adaptive's `auto_chip_thinning` option expose exact compensated feedrates capped by a safe `max_thinning_feed_multiplier`.

---

## Fusion 360 JSON Tool Library Import
`chip-tools` includes a one-way importer (and CLI command `chipcutter tools import-fusion <file.json>`) supporting Fusion 360 `.json` (and `.tools` zipped JSON) exports:
- Maps all 10 supported cutter families: `"flat end mill"`, `"bull nose end mill"`, `"ball end mill"`, `"chamfer mill"`, `"engraving"` / `"v-bit"`, `"spot drill"`, `"drill"`, `"face mill"`, `"thread mill"`, `"lollipop mill"`, `"slot mill"` (keyway / T-slot), and `"dovetail mill"`.
- Preserves original tool `"unit"` (`"inches"` vs `"millimeters"`), flute counts, shaft/flute/relieved-neck geometry (`neck_diameter_mm`, `neck_length_mm`, `cutting_thickness_mm`, `cutting_height_mm`, `side_angle_deg`), tool numbers, and all `"start-values"` material cutting presets (including `"optimal-load"` when present).
- Unsupported exotic tool types (e.g., arbitrary custom form mills or turning inserts) are skipped with a non-fatal import summary warning.
