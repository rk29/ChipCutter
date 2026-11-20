# Milestone 30 Implementation Plan — Built-In Machine Profiles: Generic, Millennium Milo v1.5 (LDO Kit), & Makera Carvera Air

## Goal

Ship three complete, shop-ready built-in **`MachineDefinition`** profiles—embedded cleanly in `chip-tools` so installed Flatpak, Windows, and macOS releases resolve them with zero external file dependencies—each paired with its authentic physical envelope, spindle limits, kinematics, post-processor defaults, and dedicated starter tool library:

1. **`generic`** — *Generic 3/4-Axis CNC Mill* (canonical baseline preserving existing starter tools and 100% test/golden compatibility).
2. **`milo-1.5`** — *Millennium Machines Milo v1.5 (LDO Edition Kit)* with the **LDO DLC-coated carbide starter endmill set**.
3. **`carvera-air`** — *Makera Carvera Air* (with optional harmonic-drive **4th-Axis Rotary Module**) and the **Makera Carvera Air 1/8" starter bit set**.

---

## Proposed Changes

### 1. Built-In Machine Definitions & Per-Machine Starter Tool Sets (`crates/chip-tools/`, `machines/`)

#### [NEW] `crates/chip-tools/src/builtin_machines.rs`
#### [MODIFY] `crates/chip-tools/src/machine.rs`, [crates/chip-tools/src/lib.rs](file:///workspace/ChipCutter/crates/chip-tools/src/lib.rs)
- Implement `builtin_generic_machine()`, `builtin_milo_1_5_machine()`, and `builtin_carvera_air_machine()`, exposed via `builtin_machine_definitions()` (sorted cleanly with `generic` first, then `milo-1.5`, `carvera-air`):

1. **`generic` — Generic 3/4-Axis CNC Mill:**
   - **Envelope:** `400 × 300 × 150 mm` (generous default envelope so all existing tutorials, torture blocks, and tests fit comfortably with zero spurious warnings).
   - **Spindle:** `1,000 – 24,000 RPM`, `1,500 W`, ER16/ER20 collet, supported shanks `[3.0, 3.175, 4.0, 6.0, 6.35, 8.0, 10.0, 12.0] mm`.
   - **Kinematics:** `5,000 mm/min` XY rapid, `2,500 mm/min` Z rapid, `3,000 mm/min` XY cut ceiling, `1,500 mm/min` Z plunge ceiling, `500 mm/s²` accel, `0.05 mm` junction deviation, `15 s` tool change, `3,600 deg/min` rotary slew.
   - **Rotary:** Enabled (`A`-axis, `max_stock_diameter_mm = 120.0`, `max_stock_length_mm = 250.0`, `chuck_diameter_mm = 65.0`, `chuck_length_mm = 20.0`, `default_post_profile_4axis = Some("linuxcnc_4axis")`).
   - **Default Post:** `"grbl"`.
   - **Tools:** Exact `default_starter_tool_library().tools` (preserving 100% compatibility across all existing demos, tutorials, and unit tests).

2. **`milo-1.5` — Millennium Machines Milo v1.5 (LDO Edition Kit):**
   - **Envelope:** `340 × 160 × 120 mm`.
   - **Spindle:** `10,000 – 24,000 RPM`, `1,500 W` air-cooled spindle, ER16 collet (`3.0 mm` & `6.0 mm` collets included in kit; `1.0 – 9.0 mm` range).
   - **Kinematics:** `2,000 mm/min` XY rapid, `1,000 mm/min` Z rapid, `1,000 mm/min` XY cut ceiling, `500 mm/min` Z plunge ceiling, `400 mm/s²` accel, `0.05 mm` junction deviation, `20 s` manual tool change with Long John optical toolsetter (`has_toolsetter = true`, `tool_change = Manual`).
   - **Rotary:** `None` (standard 3-axis benchtop mill).
   - **Default Post:** `"linuxcnc"` (clean RS274NGC dialect; users on RepRapFirmware/MillenniumOS can also drop in a custom `.post.toml`).
   - **LDO Kit Starter Tool Library (5 DLC-coated carbide cutters with realistic Aluminum 6061, Acetal/Delrin, and Hardwood presets clamped within `10,000–24,000 RPM` and `<= 1,000 mm/min`):**
     - **T1:** `6mm x 12mm LOC 1FL DLC Flat Endmill` (`Ø6.0 mm`, `12 mm` LOC, `50 mm` OAL, `Ø6.0 mm` shank, `1` flute, `20 mm` stickout)
     - **T2:** `6mm x 22mm LOC 1FL DLC Flat Endmill` (`Ø6.0 mm`, `22 mm` LOC, `60 mm` OAL, `Ø6.0 mm` shank, `1` flute, `28 mm` stickout)
     - **T3:** `3mm x 12mm LOC 1FL DLC Flat Endmill` (`Ø3.0 mm`, `12 mm` LOC, `50 mm` OAL, `Ø3.0 mm` shank, `1` flute, `18 mm` stickout)
     - **T4:** `3mm 2FL DLC Ball Endmill` (`Ø3.0 mm`, `12 mm` LOC, `50 mm` OAL, `Ø3.0 mm` shank, `2` flutes, `18 mm` stickout)
     - **T5:** `6mm 90° 2FL DLC Chamfer Mill` (`Ø6.0 mm`, `90°` included angle, `0.2 mm` tip flat, `6 mm` LOC, `50 mm` OAL, `Ø6.0 mm` shank, `2` flutes, `20 mm` stickout)

3. **`carvera-air` — Makera Carvera Air:**
   - **Envelope:** `300 × 200 × 130 mm`.
   - **Spindle:** `1,000 – 13,000 RPM` (closed-loop 200 W spindle), lever quick-change collet (`3.175 mm` / `1/8"` universal shank with tool collars; `tool_change = QuickChange`, `has_toolsetter = true`, `tool_change_time_sec = 12.0`).
   - **Kinematics:** `4,000 mm/min` XY rapid, `2,000 mm/min` Z rapid, `2,000 mm/min` XY cut ceiling, `800 mm/min` Z plunge ceiling, `500 mm/s²` accel, `0.05 mm` junction deviation, `3,600 deg/min` harmonic-drive rotary slew.
   - **Rotary:** Enabled (`A`-axis along `X`, `max_stock_diameter_mm = 92.0`, `max_stock_length_mm = 200.0`, `chuck_diameter_mm = 50.0`, `chuck_length_mm = 20.0`, `default_post_profile_4axis = Some("grblhal_4axis")`).
   - **Default Post:** `"grblhal_atc"` (or `"grblhal_4axis"` when 4th-axis rotary mode is enabled on a Setup).
   - **Makera Carvera Air Starter Bit Set (8 `1/8" / 3.175 mm`-shank carbide bits with conservative 200 W / `<= 13,000 RPM` presets for Aluminum 6061, Acetal/Acrylic, Hardwood/Epoxy Board, & Brass):**
     - **T1:** `3.175mm (1/8") x 12mm 1FL O-Flute Flat Endmill` (DLC metal/plastic roughing & pocketing)
     - **T2:** `3.175mm (1/8") x 25mm 1FL O-Flute Flat Endmill` (deep profiling & rotary roughing)
     - **T3:** `2.0mm x 8mm 1FL Flat Endmill (3.175mm Shank)` (fine pocketing & rest clearing)
     - **T4:** `1.0mm x 3mm 1FL Micro Flat Endmill (3.175mm Shank)` (fine detail & PCB/slot work)
     - **T5:** `3.175mm (1/8") 2FL Ball Nose Endmill` (3D relief finishing & 4th-axis rotary surfacing)
     - **T6:** `30° 0.2mm Tip V-Engraving Bit (3.175mm Shank)` (fine engraving & PCB isolation)
     - **T7:** `60° 0.1mm Tip V-Carve Bit (3.175mm Shank)` (V-carving & inlays)
     - **T8:** `3.175mm (1/8") 90° Chamfer / Deburring Bit` (edge breaking & countersinking)

#### [NEW] Reference TOML Snapshots in `machines/` (`machines/generic.machine.toml`, `machines/milo-1.5.machine.toml`, `machines/carvera-air.machine.toml`)
- Export checked-in reference `.machine.toml` files in `machines/` (paralleling `posts/*.post.toml`), verified in unit tests to stay 100% in sync with the statically embedded read-only built-ins (`is_builtin = true`). Users customize a built-in by cloning it into their user config space (`~/.config/chipcutter/machines/`) so future ChipCutter upgrades cleanly update built-in presets without overwriting user machines.

---

### 2. Machine-Preset Consistency & End-to-End Machining Smoke Tests (`chip-tools`, `chip-project`)

#### [MODIFY] [crates/chip-tools/src/lib.rs](file:///workspace/ChipCutter/crates/chip-tools/src/lib.rs), [crates/chip-project/src/lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
- Add unit tests verifying:
  - Every built-in machine (`generic`, `milo-1.5`, `carvera-air`) passes `MachineDefinition::validate()` with 0 errors.
  - Every cutting preset on every tool inside `milo-1.5` and `carvera-air` has RPM strictly within `[machine.spindle.min_rpm, machine.spindle.max_rpm]` and feedrates strictly within `[1.0, machine.kinematics.max_cut_feed_xy_mm_min]`! (Zero out-of-envelope presets out of the box!)
  - All built-in machine definitions resolve cleanly from an empty temp working directory (installed-bundle safety, matching `chip-post`).
  - End-to-end sample machining test: generating, validating (0 errors, 0 machine warnings!), simulating crash-free, and post-processing a 2.5D part using **`milo-1.5`** tools AND a 4th-axis rotary part using **`carvera-air`** tools!

---

## Verification Plan

1. **Unit & Preset Sanity Tests:**
   - `cargo test --locked -p chip-tools` — validates all 3 built-in machines, TOML roundtrips, reference `machines/*.machine.toml` sync, and RPM/feed envelope conformance of every starter tool preset.
   - `cargo test --locked -p chip-project` — verifies end-to-end machining & simulation on `milo-1.5` and `carvera-air` with zero crashes and zero machine warnings.
2. **CLI Out-of-Repo Smoke Test:**
   - Run `chipcutter machines list`, `chipcutter machines inspect milo-1.5`, and `chipcutter machines inspect carvera-air` from `/tmp` (outside the repo root).
3. **Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo deny check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace` (all existing tests & 15 golden snapshots stay 100% green).
