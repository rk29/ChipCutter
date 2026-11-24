# 04 — Tool Library, Cutting Presets, & Machine Definitions

## Overview (`chip-tools`)
ChipCutter separates **Tool Geometry** (physical dimensions of a cutter) from **Cutting Presets** (feeds, speeds, and engagement targets per workpiece material), and pairs global/project tool libraries with **Machine Definitions** (`*.machine.toml`) that bundle machine envelopes, spindle/kinematics limits, and per-machine tool sets.

### Library Scopes & Storage (`crates/chip-tools/src/storage.rs`)
1. **Global User Tool Libraries (`.tools.toml`):** Native `ToolLibrary` persistence uses human-readable **`.tools.toml`** (`schema_version = 1`, e.g., `~/.config/chipcutter/default.tools.toml` on Linux, or any user-configured folder synced via Git/Dropbox), deterministically sorted by `(tool_number, id)`. *(Fusion 360 `.json` and zipped `.tools` archives are accepted by the one-way importer below.)*
2. **Per-Machine Tool Libraries (`.machine.toml`):** Each `MachineDefinition` embeds its own dedicated `tools: Vec<Tool>` set tuned to that machine's spindle, collet/shank sizes, and rigidity.
3. **Project-Embedded Tools:** Whenever an operation in a project uses a tool, a full snapshot of that `Tool` and its active preset is embedded in `.chip.toml` (including secondary `clearing_tool_id` flat endmills used by two-stage V-Carving). Opening a project on another machine always reproduces identical toolpaths even if the global/machine library changed or is absent, with an explicit "Sync from Global Library" action if desired.

---

## Supported Cutter Types (`CutterGeometry` in `crates/chip-tools/src/model.rs`)
All geometric dimensions are stored internally in `f64` millimeters (`*_mm`) while preserving `Tool::native_unit: DisplayUnit` (`Millimeter` or `Inch`) so a `"1/4 in Flat Endmill"` displays cleanly as `0.25 in` rather than `6.35 mm`. Every `CutterGeometry` variant includes `overall_length_mm`, `shank_diameter_mm`, and `flutes`:

| `CutterGeometry` Variant | Exact Geometric Fields | Valid Operations (`CutterGeometry::supports_operation`) |
|---|---|---|
| **`FlatEndmill`** | `diameter_mm`, `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Facing`, `Contour2D`, `Pocket2D` (incl. 2D Adaptive), `HelicalBore`, `Trace`, `VCarveFlatClearance`, `SlotUndercut`, `Waterline3D`, `Adaptive3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Rotary2Linear`, `Rotary4AxisSim` |
| **`BullNoseEndmill`** | `diameter_mm`, `corner_radius_mm` (`0 < R < D/2`), `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Facing`, `Contour2D`, `Pocket2D` (incl. 2D Adaptive), `HelicalBore`, `Trace`, `Waterline3D`, `Adaptive3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Rotary2Linear`, `Rotary4AxisSim` |
| **`BallEndmill`** | `diameter_mm` (`R = D/2`), `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Contour2D`, `Pocket2D`, `Trace`, `Deburr3D`, `Waterline3D`, `Adaptive3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Rotary2Linear`, `Rotary4AxisSim` |
| **`ChamferMill`** *(Chamfer / Spot / V-Bit / Engraving)* | `diameter_mm`, `tip_diameter_mm` (`0 <= tip < D`), `included_angle_deg`, `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `VCarve` *(requires `10° <= included_angle_deg <= 160°`)*, `Chamfer`, `Trace`, `Drill` (spot drilling), `Deburr3D`, `Parallel3D`, `Pencil3D`, `Rotary2Linear`, `Rotary4AxisSim` |
| **`TwistDrill`** | `diameter_mm`, `point_angle_deg` (e.g. `118°` / `135°`), `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Drill` (Simple, Peck, Chip-Break) |
| **`FaceMill`** | `cutting_diameter_mm`, `outside_diameter_mm`, `corner_radius_mm`, `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Facing`, `Contour2D` |
| **`ThreadMill`** | `diameter_mm`, `neck_diameter_mm` (`< D`), `neck_length_mm`, `pitch_mm`, `teeth_count`, `thread_angle_deg` (`29°..=90°`, default `60°`), `flute_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `ThreadMill` (Internal & External) |
| **`LollipopEndmill`** | `diameter_mm`, `neck_diameter_mm` (`< D`), `neck_length_mm`, `flute_length_mm` (`R + √(R² − r_neck²)`), `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Deburr3D` (incl. underside back-chamfer), `Undercut3D` *(only)* |
| **`SlotMill`** *(Slot / Keyway / T-Slot)* | `diameter_mm`, `cutting_thickness_mm`, `corner_radius_mm`, `neck_diameter_mm` (`< D`), `neck_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `SlotUndercut` |
| **`DovetailCutter`** | `diameter_mm`, `side_angle_deg` (`15° < α < 85°`), `cutting_height_mm`, `corner_radius_mm`, `neck_diameter_mm` (`<= D_top`), `neck_length_mm`, `overall_length_mm`, `shank_diameter_mm`, `flutes` | `Dovetail` |

All 10 cutter geometries share a single unified `AnalyticCutterEnvelope` kernel in `chip-tools` supporting both monotonic top-down profiles and non-monotonic relieved-neck undercut heads (`axial_z_interval_at_radius`, Minkowski radial/axial stock dilation, and `dovetail_top_diameter_mm` / `lollipop_sphere_height_mm` analytic helpers). Starter shop presets for the relieved-neck undercut trio (`T13` 6 mm Lollipop, `T14` 12×4 mm T-Slot Mill, `T15` 12 mm 60° Dovetail) are exposed via `starter_undercut_tools()` without shifting `T1`–`T10` starter indices.

### `Tool` Identity & Machine Metadata
- `id: ToolId` (`String`), `name: String`
- `vendor: Option<String>`, `product_id: Option<String>`
- `native_unit: DisplayUnit` (`Millimeter` | `Inch`)
- `tool_number: u32` (`T1`, `T2`, …)
- `length_offset_number: u32` (defaults to `tool_number`, emitted as `H1`, `H2` in LinuxCNC `G43`)
- `diameter_offset_number: u32` (defaults to `tool_number`, `D1`, `D2`)
- `spindle_direction: SpindleDirection` (`Cw` / `M3` default | `Ccw` / `M4`)
- `default_coolant: CoolantMode` (`Disabled` | `Flood` | `Mist` | `AirBlast`)
- `stickout_length_mm: f64` — Distance from collet/holder nose to tool tip (`mm`, must be `>= flute_length_mm`), checked during simulation for holder/collet clearance against stock and fixtures.

---

## Per-Material Cutting Presets (`CuttingPreset` in `crates/chip-tools/src/feeds_speeds.rs`)
Each tool carries one or more named `CuttingPreset` entries (`id`, `name`, `material`, plus coupled cutting fields):

| Field | Canonical Unit | Coupled Formula / Default (`CuttingPreset::from_surface_speed`) |
|---|---|---|
| `spindle_rpm` | `RPM` | `RPM = (surface_speed_m_min * 1000) / (π * D_mm)` |
| `surface_speed_m_min` | `m/min` (displayed as `m/min` or `SFM`) | Coupled bidirectionally with `spindle_rpm` |
| `cutting_feed_mm_min` | `mm/min` (displayed as `mm/min` or `IPM`) | `Vf = spindle_rpm * flutes * chipload_mm_tooth` |
| `chipload_mm_tooth` | `mm/tooth` | Coupled bidirectionally with `cutting_feed_mm_min` |
| `lead_feed_mm_min` | `mm/min` | Defaults to `0.8 * cutting_feed_mm_min` |
| `ramp_feed_mm_min` | `mm/min` | Defaults to `0.5 * cutting_feed_mm_min` |
| `plunge_feed_mm_min` | `mm/min` | Defaults to `(cutting_feed_mm_min / 3.0).max(10.0)` |
| `default_stepdown_mm` (`ap`) | `mm` | Axial depth of cut preset |
| `default_stepover_mm` (`ae`) | `mm` | Radial width of cut preset (contour-parallel / zig-zag) |
| `optimal_load_mm` (`Option<f64>`) | `mm` | Recommended constant radial engagement for Adaptive strategies (defaults to `0.20 * diameter` when unset) |
| `coolant_override` (`Option<CoolantMode>`) | `CoolantMode` | Overrides `Tool::default_coolant` per material |

### Operation-Level Overrides
When a user picks a Tool + Preset inside an Operation, the operation copies those values as defaults. Any individual field in the Operation Inspector can be overridden locally (visually badged with an "overridden from preset" reset icon).

---

## Built-In SFM, Chipload, & Radial Chip Thinning Calculator
Inside both the Tool Library Editor and the Operation Inspector:
- Editing **`surface_speed_m_min`** updates **`spindle_rpm`** (and vice versa), either preserving `chipload_mm_tooth` (`lock_chipload = true`, scaling linear feeds proportionally) or preserving `cutting_feed_mm_min` (`lock_chipload = false`, recalculating `chipload_mm_tooth`).
- Editing **`chipload_mm_tooth`** updates **`cutting_feed_mm_min`** (and scales secondary feeds), and vice versa.
- **Radial Chip Thinning (`h_ex`):** When radial engagement `ae < 0.5 * diameter`, effective chip thickness thins by $h_{\text{ex}} = f_z \cdot \frac{2\sqrt{a_e(D - a_e)}}{D}$. `compute_radial_chip_thinning` (capped at `2.5×` for calculator hints) and `compute_adaptive_thinning_feed` (capped by each operation's `max_thinning_feed_multiplier`, clamped to `[1.0, 5.0]`) compute exact compensated feedrates.

---

## Fusion 360 Tool Library Import (`crates/chip-tools/src/fusion_import.rs`)
`chip-tools` includes a one-way importer (and CLI command `chipcutter tools import-fusion <file.json|.tools>`) supporting Fusion 360 `.json` and zipped `.tools` exports (capped at `32 MiB` / `256` archive entries with symlink rejection):
- Maps all 10 supported cutter families by normalized `"type"`:
  - `"flat end mill"` -> `FlatEndmill`
  - `"bull nose end mill"` -> `BullNoseEndmill`
  - `"ball end mill"` -> `BallEndmill`
  - `"chamfer mill" | "spot drill" | "engraving"` -> `ChamferMill`
  - `"drill"` -> `TwistDrill`
  - `"face mill"` -> `FaceMill`
  - `"thread mill"` -> `ThreadMill`
  - `"lollipop mill"` -> `LollipopEndmill`
  - `"slot mill" | "t-slot mill"` -> `SlotMill`
  - `"dovetail mill"` -> `DovetailCutter`
- Preserves original tool `"unit"` (`"inches"` vs `"millimeters"`), flute counts, vendor/product metadata, shaft/flute/relieved-neck geometry (`neck_diameter_mm`, `neck_length_mm`, `cutting_thickness_mm`, `cutting_height_mm`, `side_angle_deg`, `pitch_mm`, `teeth_count`), tool/offset numbers, and all `"start-values"` material cutting presets (including `"optimal-load"` / `"v_ae"` when present).
- Unsupported exotic tool types (e.g., arbitrary custom form mills or turning inserts) are skipped with non-fatal warnings in `FusionImportReport::warnings`.

---

## Machine Definitions (`*.machine.toml`, `crates/chip-tools/src/machine.rs`)
ChipCutter models complete CNC machines as self-contained `.machine.toml` (`schema_version = 1`) documents bundling physical limits, default post-processing, optional rotary hardware, and a dedicated per-machine tool library:

- **Core Schema (`MachineDefinition`):**
  - Identity & metadata: `schema_version`, `id` (slug `[a-z0-9._-]`), `name`, `manufacturer`, `description`, `is_builtin` (runtime-only, read-only guard).
  - **`travel_mm: MachineTravelEnvelope`:** `{ x_mm, y_mm, z_mm }` physical XYZ axis envelope.
  - **`spindle: MachineSpindleSpec`:** `{ min_rpm, max_rpm, power_watts: Option<f64>, collet_type: String, supported_shank_diameters_mm: Vec<f64> }`.
  - **`kinematics: MachineKinematicsSpec`:** `{ max_rapid_xy_mm_min, max_rapid_z_mm_min, max_cut_feed_xy_mm_min, max_cut_feed_z_mm_min, accel_mm_s2, junction_deviation_mm, tool_change_time_sec, max_rotary_rate_deg_min }` (feeds into cycle-time estimation and feed/spindle limit checks).
  - Tool changing & post defaults: `tool_change: MachineToolChangeKind::{Manual, QuickChange, AutomaticAtc}`, `has_toolsetter: bool`, `default_post_profile: String`.
  - **`rotary: Option<MachineRotaryDefaults>`:** Optional 4th-axis module limits and default chuck dimensions (`axis`, `max_stock_diameter_mm`, `max_stock_length_mm`, `chuck_diameter_mm`, `chuck_length_mm`, `default_post_profile_4axis`).
  - **Per-Machine Tool Library:** `tools: Vec<Tool>` (convertible to a `ToolLibrary` via `to_tool_library()`).
- **7 Read-Only Built-In Machines & User Clones:**
  - Built-in compiled presets: **`generic`** (`DEFAULT_MACHINE_ID`), **`milo-1.5`**, **`milo-2.0`**, **`voron-cascade`**, **`carvera`**, **`carvera-air`**, and **`carvera-z1`**. Built-in IDs are read-only and cannot be shadowed by user files so upgrades stay clean.
  - Users clone any built-in or custom machine into editable `.machine.toml` files stored in `~/.config/chipcutter/machines/` (OS user config directory), where custom travel, spindle, kinematics, post defaults, and dedicated tool sets are persisted.
- **Project Binding & Interactive Machine-Switch Tool Swapping:**
  - `ProjectDocument::machine_id` links a project to a target machine (defaulting to `"generic"` when omitted) and syncs Setup post profiles and rotary chuck defaults via `apply_machine_defaults`.
  - Switching a project to a different machine in the GUI offers **interactive tool swapping**, matching or remapping each operation's embedded tool against the new machine's dedicated tool library (and checking shank/spindle compatibility) while keeping operations cleanly re-evaluated.
