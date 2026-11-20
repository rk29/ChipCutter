# Milestone 29 Implementation Plan — Machine Definition Schema, Persistence, & Per-Machine Tool Libraries

## Goal

Introduce the core UI-agnostic **`MachineDefinition`** model, TOML storage & discovery (`*.machine.toml`), project/setup integration (`machine_id`), portable `.chipz` bundling, machine envelope/spindle validation checks, and `chipcutter machines` CLI commands—while maintaining 100% backward compatibility with existing `.chip.toml` projects, golden G-code snapshots, and the `generic` baseline machine.

---

## Proposed Changes

### 1. Core `MachineDefinition` Schema & Storage (`crates/chip-tools/`)

#### [NEW] `crates/chip-tools/src/machine.rs`
#### [MODIFY] [crates/chip-tools/src/lib.rs](file:///workspace/ChipCutter/crates/chip-tools/src/lib.rs)
- Define the versioned, deterministic TOML-serializable `MachineDefinition` (`schema_version = 1`):
  - `id: String` (canonical slug, e.g., `"generic"`, `"milo-1.5"`, `"carvera-air"`), `name: String`, `manufacturer: Option<String>`, `description: Option<String>`, `is_builtin: bool` (`#[serde(skip)]`).
  - `travel_mm: MachineTravelEnvelope` (`x_mm`, `y_mm`, `z_mm`, all `> 0.0`).
  - `spindle: MachineSpindleSpec` (`min_rpm`, `max_rpm`, `power_watts: Option<f64>`, `collet_type: String`, `supported_shank_diameters_mm: Vec<f64>`).
  - `kinematics: MachineKinematicsSpec` (`max_rapid_xy_mm_min`, `max_rapid_z_mm_min`, `max_cut_feed_xy_mm_min`, `max_cut_feed_z_mm_min`, `accel_mm_s2`, `junction_deviation_mm`, `tool_change_time_sec`, `max_rotary_rate_deg_min`).
  - `tool_change: MachineToolChangeKind` (`Manual`, `QuickChange`, `AutomaticAtc`) + `has_toolsetter: bool`.
  - `default_post_profile: String` (e.g., `"grbl"`, `"linuxcnc"`, `"grblhal_4axis"`).
  - `rotary: Option<MachineRotaryDefaults>` (`axis: String` `"A"`/`"B"`, `max_stock_diameter_mm`, `max_stock_length_mm`, `chuck_diameter_mm`, `chuck_length_mm`, `default_post_profile_4axis: Option<String>`).
  - **`tools: Vec<Tool>`** — the machine's dedicated tool library, deterministically sorted by `(tool_number, id)` on save, with helpers `to_tool_library(&self) -> ToolLibrary`, `validate(&self) -> Result<(), MachineStorageError>`, and `MachineDefinition::to_toml_string()` / `from_toml_str()` / `load_from_path()` / `save_to_path()`.
- Discovery, Read-Only Built-In Protection, & Upgrade-Safe Persistence API:
  - `DEFAULT_MACHINE_ID: &str = "generic"`.
  - `user_machines_dir() -> Option<PathBuf>` (`dirs::config_dir()/chipcutter/machines` — persisted strictly in OS user configuration space so application upgrades never overwrite user machines).
  - `builtin_machine_definitions() -> Vec<MachineDefinition>` (initially shipping `generic` in M29; expanded with `milo-1.5` and `carvera-air` in M30). All built-in machines carry `is_builtin = true` and are strictly **read-only**.
  - `is_builtin_machine_id(id: &str) -> bool`: reserved built-in IDs cannot be overwritten or deleted in user config space.
  - `load_available_machines(extra_search_dir: Option<&Path>) -> Vec<MachineDefinition>`: returns all read-only built-ins first, followed by custom user machines loaded from `extra_search_dir` and `~/.config/chipcutter/machines/*.machine.toml` (ignoring any user file attempting to shadow a reserved built-in ID).
  - `clone_machine_definition(source: &MachineDefinition, new_id: impl Into<String>, new_name: impl Into<String>) -> Result<MachineDefinition, MachineStorageError>`: creates a mutable (`is_builtin = false`) copy with its full cloned tool set, validating that `new_id` is a valid slug and not a reserved built-in ID.
  - `save_user_machine(machine: &MachineDefinition) -> Result<PathBuf, MachineStorageError>` and `delete_user_machine(id: &str) -> Result<(), MachineStorageError>`: both reject `is_builtin` / reserved built-in IDs with `MachineStorageError::BuiltInReadOnly`.
  - **Shareable Export & Import Helpers:**
    - `export_machine_to_path(machine: &MachineDefinition, dest: &Path) -> Result<(), MachineStorageError>`: writes a clean, self-contained `.machine.toml` (specs + kinematics + rotary + full `[[tools]]` set) for sharing with other machinists—works for both built-ins and custom machines.
    - `import_machine_from_path(src: &Path, existing_ids: &[String], id_override: Option<&str>) -> Result<MachineDefinition, MachineStorageError>`: parses & validates an external `.machine.toml` (including every embedded tool), guarantees `is_builtin = false`, auto-disambiguates if the imported ID collides with a built-in or existing custom ID (e.g., `milo-1.5-imported` / `-2`, unless `id_override` is provided and free), and saves it into `~/.config/chipcutter/machines/`.
  - `resolve_machine_definition(id_or_path: &str, extra_search_dir: Option<&Path>) -> Result<MachineDefinition, MachineStorageError>`.

---

### 2. Project, Simulation Kinematics, Validation, & `.chipz` Integration (`chip-sim`, `chip-project`)

#### [MODIFY] [crates/chip-sim/src/timeline.rs](file:///workspace/ChipCutter/crates/chip-sim/src/timeline.rs)
- Add `MachineKinematicsConfig::from_machine_spec(spec: &MachineKinematicsSpec) -> Self` (or `From<&MachineKinematicsSpec>`) so simulation cycle-time estimation automatically honors the active machine's rapid rates, acceleration, junction deviation, tool-change time, and rotary slew rate.

#### [MODIFY] [crates/chip-project/src/document.rs](file:///workspace/ChipCutter/crates/chip-project/src/document.rs)
- Add `#[serde(default, skip_serializing_if = "Option::is_none")] pub machine_id: Option<String>` on `ProjectDocument` (omitted when `None` so all existing `.chip.toml` files and tests stay 100% backward-compatible).
- Add helpers `ProjectDocument::effective_machine_id(&self) -> &str` (returns `machine_id.as_deref().unwrap_or(DEFAULT_MACHINE_ID)`) and `ProjectDocument::apply_machine_defaults(&mut self, machine: &MachineDefinition)` (sets `machine_id`, syncs default post-processor on setups if desired, without clobbering already-embedded project tools).

#### [MODIFY] [crates/chip-project/src/validation.rs](file:///workspace/ChipCutter/crates/chip-project/src/validation.rs)
- Add machine-aware validation checks (non-blocking `Warning` diagnostics so jobs remain inspectable while alerting machinists before cutting):
  - **Spindle RPM out-of-range:** Operation spindle RPM `< machine.spindle.min_rpm` or `> machine.spindle.max_rpm`.
  - **Cut/Plunge feedrate exceeds machine ceiling:** Feedrate `> machine.kinematics.max_cut_feed_xy_mm_min` (or Z plunge `> max_cut_feed_z_mm_min`).
  - **Stock / Toolpath exceeds machine XYZ travel envelope:** Resolved stock bounding box (or generated toolpath AABB span) exceeds `(travel_mm.x_mm, travel_mm.y_mm, travel_mm.z_mm)`.
  - **Rotary capability check:** Setup enables 4th-axis rotary mode on a machine whose `rotary` capability is `None`, or cylindrical stock diameter/length exceeds `rotary.max_stock_diameter_mm` / `max_stock_length_mm`.
  - **Unknown `machine_id`:** Warning if `project.machine_id` cannot be resolved (falling back to `generic`).

#### [MODIFY] [crates/chip-project/src/bundle.rs](file:///workspace/ChipCutter/crates/chip-project/src/bundle.rs)
- Extend `.chipz` portable packing/unpacking: if the project references a custom (non-builtin) `machine_id`, pack `machines/<id>.machine.toml` into the `.chipz` archive and resolve it automatically on unpack/build.

---

### 3. Headless CLI Machine Commands (`crates/chip-cli/`)

#### [MODIFY] [crates/chip-cli/src/main.rs](file:///workspace/ChipCutter/crates/chip-cli/src/main.rs), `crates/chip-cli/src/commands/`
- Add `chipcutter machines` subcommand group:
  - `chipcutter machines list`: prints table of all built-in (`[built-in]`) and user custom (`[custom]`) machines (`id`, `name`, `travel XYZ`, `RPM range`, `tool count`, `default post`, `4th-axis`).
  - `chipcutter machines inspect <id>`: prints full machine specs and its complete tool list (`T#`, name, diameter, flutes, LOC, shank).
  - `chipcutter machines clone <source-id> <new-id> [--name <display-name>]`: clones any built-in or custom machine (plus its full tool set) into `~/.config/chipcutter/machines/<new-id>.machine.toml`.
  - `chipcutter machines export <id> -o <dest.machine.toml>`: exports a self-contained, shareable `.machine.toml` (specs + tools).
  - `chipcutter machines import <src.machine.toml> [--id <custom-id>]`: validates and imports a shared `.machine.toml` into `~/.config/chipcutter/machines/` (auto-disambiguating if the ID collides with a built-in or existing custom machine).
- Add optional `--machine <id>` flag on `generate-sample-step` and `validate` (validating an existing project against any machine definition without modifying the file).

---

## Verification Plan

1. **Unit & Roundtrip Tests (`chip-tools`, `chip-project`):**
   - `MachineDefinition` TOML serialize -> parse -> re-serialize deterministic roundtrip test and validation error tests (invalid travel, min_rpm > max_rpm, invalid tool).
   - Read-only built-in protection tests: `save_user_machine` / `delete_user_machine` reject built-in IDs (`BuiltInReadOnly`); `clone_machine_definition` produces a mutable custom copy with all tools preserved; `export_machine_to_path` -> `import_machine_from_path` roundtrips cleanly and auto-disambiguates colliding built-in/custom IDs.
   - `ProjectDocument` with `machine_id: None` omits field in TOML and resolves to `generic`; with `machine_id: Some(...)` roundtrips cleanly and bundles custom `.machine.toml` inside `.chipz`.
   - Validation tests triggering each new machine warning: RPM out of bounds, XYZ travel exceeded, feedrate exceeded, and rotary-on-3-axis-machine warning.
2. **CLI & Workspace Gates:**
   - Smoke-test `chipcutter machines list`, `inspect`, `clone`, `export`, and `import` from an empty temp directory outside the repo root.
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo deny check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace` (confirming 100% of existing tests and all 15 byte-for-byte golden G-code snapshots pass unchanged!).
