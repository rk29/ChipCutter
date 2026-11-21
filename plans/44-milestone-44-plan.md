# Milestone 44 Implementation Plan — Built-In Machine Definitions: Carvera, Carvera Z1, Milo 2.0, & Voron Cascade

## Goal

Expand ChipCutter's read-only built-in machine catalog (currently `generic`, `milo-1.5`, and `carvera-air`) with four additional widely used desktop CNC machines—**Makera Carvera (original)**, **Makera Carvera Z1**, **Millennium Machines Milo v2.0**, and **Voron Cascade**—complete with realistic travel envelopes, spindle limits, kinematics, optional 4th-axis rotary defaults, tailored starter tool sets, reference `.machine.toml` files, parity tests, and updated documentation.

---

## Proposed Machine Specifications

| Machine ID | Display Name | Travel X × Y × Z (mm) | Spindle & Collet | Tool Change / Toolsetter | Default Post | 4th-Axis Rotary |
|---|---|---|---|---|---|---|
| `carvera` | **Makera Carvera** | `360 × 240 × 140` | `200 W` · `1,000–15,000 RPM` · `1/8"` (opt. `3/4/6/6.35 mm`) | `AutomaticAtc` (6-tool carousel) · Fixed toolsetter | `grblhal_atc` | `A`-axis · `Ø92 × 240 mm` · chuck `Ø50 × 20 mm` · `grblhal_4axis` |
| `carvera-z1` | **Makera Carvera Z1** | `200 × 200 × 100` | `150 W` · `1,000–13,000 RPM` · `1/8"` quick-change (opt. `3/4/6/6.35 mm`) | `QuickChange` (lever) · Auto tool-height probe | `grblhal_atc` | `A`-axis · `Ø80 × 150 mm` · chuck `Ø50 × 20 mm` · `grblhal_4axis` |
| `milo-2.0` | **Millennium Machines Milo v2.0** | `348 × 210 × 120` | `1,500 W` · `10,000–24,000 RPM` · `ER16` (`3–8 mm`) | `Manual` · Fixed toolsetter | `linuxcnc` | None by default (user-clonable) |
| `voron-cascade` | **Voron Cascade** | `275 × 220 × 90` | `1,500 W` · `8,000–24,000 RPM` · `ER16` (`3–8 mm`) | `Manual` · Fixed toolsetter | `grblhal_atc` | None by default (user-clonable) |

---

## Proposed Changes

### 1. Built-In Machine Definitions & Starter Tool Sets (`chip-tools`)

#### [MODIFY] [crates/chip-tools/src/builtin_machines.rs](file:///workspace/ChipCutter/crates/chip-tools/src/builtin_machines.rs)
#### [MODIFY] [crates/chip-tools/src/lib.rs](file:///workspace/ChipCutter/crates/chip-tools/src/lib.rs)
- Add constants `CARVERA_MACHINE_ID = "carvera"`, `CARVERA_Z1_MACHINE_ID = "carvera-z1"`, `MILO_2_0_MACHINE_ID = "milo-2.0"`, and `VORON_CASCADE_MACHINE_ID = "voron-cascade"`.
- Implement four deterministic builder functions registered in `builtin_machines()` alongside `generic`, `milo-1.5`, and `carvera-air`:
  - **`builtin_carvera_machine()`:**
    - `360 × 240 × 140 mm`, `200 W` `1,000–15,000 RPM`, `AutomaticAtc` (`tool_change_time_sec = 8.0`), max rapid XY `4,500 mm/min`, Z `2,000 mm/min`, `grblhal_atc`, 4th-axis rotary (`Ø92 × 240 mm`).
    - Starter 8-tool Makera ATC set (`1/8"` 1FL `12 mm` & `25 mm` O-flute flats, `Ø2 mm` & `Ø1 mm` micro flats, `1/8"` ball endmill, `30°` & `60°` V-engraving bits, `90°` chamfer/spot bit) with RPM/feed presets validated strictly within `1,000–15,000 RPM` and machine cut-feed limits.
  - **`builtin_carvera_z1_machine()`:**
    - `200 × 200 × 100 mm`, `150 W` `1,000–13,000 RPM`, `QuickChange` (`tool_change_time_sec = 12.0`), max rapid XY `4,000 mm/min`, Z `1,800 mm/min`, `grblhal_atc`, 4th-axis rotary (`Ø80 × 150 mm`).
    - Starter 8-tool `1/8"` bit set tuned specifically for the lower-power `150 W` spindle (lighter aluminum stepdown/stepover and conservative plunge feeds so presets are shop-safe out of the box).
  - **`builtin_milo_2_0_machine()`:**
    - `348 × 210 × 120 mm` (expanded Y travel & ballscrew/anti-backlash drive), `1.5 kW` `10,000–24,000 RPM` `ER16`, `Manual` + toolsetter (`tool_change_time_sec = 20.0`), max rapid XY `3,000 mm/min`, Z `1,500 mm/min`, `linuxcnc`.
    - Starter metric DLC tool set (`Ø6 mm` 1FL/3FL flats, `Ø3 mm` flat, `Ø6 mm` & `Ø3 mm` ball nose, `Ø6 mm` 90° chamfer, `Ø5 mm` stub drill) validated within `10,000–24,000 RPM`.
  - **`builtin_voron_cascade_machine()`:**
    - `275 × 220 × 90 mm` (enclosed desktop mill with `340 × 225 mm` tooling plate, MGN15 linear rails, NEMA 23 steppers), `1.5 kW` `8,000–24,000 RPM` `ER16`, `Manual` + toolsetter (`tool_change_time_sec = 20.0`), max rapid XY `3,500 mm/min`, Z `1,500 mm/min`, `grblhal_atc`.
    - Starter metric/imperial aluminum & engineering-plastic tool set (`Ø6 mm` roughing/finishing flats, `Ø3.175 mm` 1FL O-flute flat & ball, `Ø6 mm` 90° chamfer/spot, `Ø5 mm` stub drill) validated within `8,000–24,000 RPM`.
- Note: In Milestone 45, once `CutterGeometry::ThreadMill` lands, starter thread mills will also be appended cleanly to `generic`, `milo-2.0`, and `voron-cascade`.

### 2. Checked-In Reference `.machine.toml` Profiles (`machines/`)

#### [NEW] `machines/carvera.machine.toml`
#### [NEW] `machines/carvera-z1.machine.toml`
#### [NEW] `machines/milo-2.0.machine.toml`
#### [NEW] `machines/voron-cascade.machine.toml`
- Export deterministic reference `.machine.toml` files matching the exact output of `builtin_machines()` so users can inspect, diff, or import them directly.
- Extend `builtin_machines_validate_conform_to_spindle_and_feed_limits_and_match_reference_toml` in `crates/chip-tools/src/lib.rs` to assert all **7** built-in machines (`generic`, `milo-1.5`, `milo-2.0`, `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`) pass full validation, spindle/feed conformance, and byte-exact parity against `machines/*.machine.toml`.

### 3. Documentation & CLI Verification

#### [MODIFY] [docs/tutorials/15-machine-definitions-tool-sets.md](file:///workspace/ChipCutter/docs/tutorials/15-machine-definitions-tool-sets.md)
#### [MODIFY] [README.md](file:///workspace/ChipCutter/README.md)
- Update the built-in machines comparison table in Tutorial 15 and the Highlights section in `README.md` to list all 7 built-in machines (`Generic 3/4-Axis CNC Mill`, `Makera Carvera`, `Makera Carvera Air`, `Makera Carvera Z1`, `Millennium Machines Milo v1.5`, `Millennium Machines Milo v2.0`, and `Voron Cascade`).

---

## Verification Plan

1. **Unit & Parity Tests (`chip-tools`, `chip-project`, `chip-cli`):**
   - `cargo test --locked -p chip-tools` verifying all 7 built-in machines validate, satisfy every RPM/feedrate/shank constraint, protect built-ins from overwrite, and match `machines/*.machine.toml`.
   - Smoke-test `chipcutter machines list`, `chipcutter machines inspect carvera`, `chipcutter machines inspect carvera-z1`, `chipcutter machines inspect milo-2.0`, and `chipcutter machines inspect voron-cascade`.
2. **Workspace Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo test --locked -p chip-tools -p chip-project -p chip-cli`
