# Milestone 32 Implementation Plan — Post-Processor Integrity, Safe Output Paths, & G-Code Injection Defense

## Goal

Eliminate path-traversal writes during G-code export, prevent untrusted `.chipz` / project directories from silently hijacking built-in post-processors or reading arbitrary filesystem paths, and harden `chip-post` against G-code command injection and non-finite (`NaN`/`±Inf`) numeric emission—without breaking any existing built-in post profiles or golden G-code snapshots.

---

## Security Findings Addressed

- **Finding #1 (High):** Arbitrary file write via unsanitized `PostProfile::extension` path traversal.
- **Finding #2 (High):** Silent built-in post-processor override & arbitrary filesystem path loading via untrusted `setup.post_profile` / `project.machine_id`.
- **Finding #4 (Medium):** G-code line/command injection via template interpolation and non-finite numeric emission in `format_decimal`.

---

## Proposed Changes

### 1. Strict Output Filename & Extension Sanitization (`chip-post`, `chip-cli`, `chip-gui`)

#### [MODIFY] [crates/chip-post/src/partition.rs](file:///workspace/ChipCutter/crates/chip-post/src/partition.rs)
- Add `pub fn sanitize_file_extension(raw: &str) -> String`:
  - Strips leading `.` characters, retains only ASCII alphanumerics (`[A-Za-z0-9]`), caps length at `16` characters, and falls back to `"nc"` if empty.
- Update `partition_operations` to use `sanitize_file_extension(&profile.extension)` instead of `profile.extension.trim_start_matches('.')`.
- Add `pub fn validate_safe_output_filename(name: &str) -> Result<&str, PostError>` asserting that `name` is non-empty, contains no `/` or `\`, has exactly one `Component::Normal` path component, and is not `.` or `..`.

#### [MODIFY] [crates/chip-cli/src/commands/validate_sim_build.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/validate_sim_build.rs)
#### [MODIFY] [crates/chip-gui/src/panels/gcode_preview.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/gcode_preview.rs)
- Validate every `nc.filename` via `validate_safe_output_filename` before joining onto `args.output_dir` (CLI) or the user-selected export folder (GUI multi-file export), returning/logging an error rather than writing outside the target directory.

---

### 2. Read-Only Built-In Post-Processors & Safe ID vs. File Path Resolution (`chip-post`, `chip-tools`, `chip-cli`, `chip-gui`)

#### [MODIFY] [crates/chip-post/src/profile.rs](file:///workspace/ChipCutter/crates/chip-post/src/profile.rs)
- Add `#[serde(skip)] pub is_builtin: bool` and `pub source: PostProfileSource` (`BuiltIn`, `UserConfig`, `ProjectOrBundle`, `ExplicitFile`) on `PostProfile`.
- Add `pub fn is_builtin_post_id(id: &str) -> bool` and `pub fn validate_post_id(id: &str) -> Result<(), PostError>` (mirroring `validate_machine_id` in `chip-tools`).
- Update `load_available_post_profiles(extra_search_dir: Option<&Path>)`:
  - Mark compiled-in profiles (`grbl`, `grblhal_atc`, `linuxcnc`, `linuxcnc_4axis`, `grblhal_4axis`, `grbl_rotary_wrap`) with `is_builtin = true`.
  - **Remove** implicit current-working-directory `PathBuf::from("posts")` lookup so running `chipcutter` or `chip-gui` inside an untrusted folder cannot hijack post profiles.
  - **Never allow** any `.post.toml` discovered in `extra_search_dir` or `user_post_profiles_dir()` to overwrite a built-in profile ID (`if is_builtin_post_id(&prof.id) { continue; }`).
  - Prioritize `user_post_profiles_dir()` over `extra_search_dir` so an untrusted project/bundle cannot shadow a user's trusted custom post profile of the same ID.
- Split resolution into two explicit APIs:
  - `resolve_post_profile(id: &str, extra_search_dir: Option<&Path>)`: resolves strictly by profile ID (never probes arbitrary filesystem paths from an untrusted `.chip.toml`).
  - `resolve_post_profile_cli_override(id_or_path: &str, extra_search_dir: Option<&Path>)`: explicitly permits loading a `.post.toml` file path when passed directly by the operator on the CLI (`--post path/to/custom.post.toml`).

#### [MODIFY] [crates/chip-tools/src/machine.rs](file:///workspace/ChipCutter/crates/chip-tools/src/machine.rs)
- Split `resolve_machine_definition` (ID-only lookup for untrusted `project.machine_id`) from `resolve_machine_definition_cli_override` (allowing explicit `.machine.toml` paths only when explicitly supplied via CLI flags).

#### [MODIFY] [crates/chip-cli/src/commands/validate_sim_build.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/validate_sim_build.rs)
#### [MODIFY] [crates/chip-gui/src/panels/gcode_preview.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/gcode_preview.rs)
- Use `resolve_post_profile_cli_override` only when `--post` was explicitly passed on the CLI; use ID-only `resolve_post_profile` for `setup.post_profile`.
- Surface a visible advisory warning in CLI output and a subtle amber banner in the GUI G-Code Preview pane whenever the active post profile originates from `PostProfileSource::ProjectOrBundle` (an untrusted `.chipz` or project directory) so machinists review custom header/footer blocks before running on hardware.

---

### 3. G-Code Injection Defense & Non-Finite Numeric Guard (`chip-post`)

#### [MODIFY] [crates/chip-post/src/modal.rs](file:///workspace/ChipCutter/crates/chip-post/src/modal.rs)
#### [MODIFY] [crates/chip-post/src/emitter.rs](file:///workspace/ChipCutter/crates/chip-post/src/emitter.rs)
#### [MODIFY] [crates/chip-post/src/cycles_arcs.rs](file:///workspace/ChipCutter/crates/chip-post/src/cycles_arcs.rs)
- Add `PostError::UnsafeGCodeLine { reason: String }` and `PostError::NonFiniteValue { field: &'static str }`.
- Export `pub(crate) fn sanitize_comment_ascii(raw: &str) -> String` and apply it to `op_name`, `strategy_name`, and `tool_desc` **before** template substitution in `operation_header`, `tool_change_block`, and structured headers.
- In `LineWriter::push_code_line`:
  - Reject any line containing `\r`, `\n`, or NUL (`\0`) bytes with `PostError::UnsafeGCodeLine`.
- In `format_decimal` (or a checked wrapper `format_finite_decimal`):
  - Fail closed with `PostError::NonFiniteValue` whenever `!val.is_finite()`, preventing `NaN` or `inf` coordinates, feeds, spindle speeds, or rotary angles from ever being emitted.

---

## Verification Plan

1. **Unit Security Regression Tests (`chip-post`, `chip-tools`):**
   - `extension = "../../evil.sh"` / `"/tmp/pwn"` / `"nc/../../x"` sanitizes strictly to alphanumeric filename components and never escapes `output_dir`.
   - A directory containing `posts/grbl.post.toml` with `id = "grbl"` and modified `program_header` cannot shadow built-in `grbl`.
   - `resolve_post_profile("/etc/hosts", None)` and `resolve_machine_definition("/etc/hosts", None)` return `UnknownProfile` / `UnknownMachine` (do not read arbitrary files from untrusted project IDs).
   - Operation/tool names containing `)\nG0 Z-50\n(` are sanitized into benign inline comment characters and never emit a separate `G0 Z-50` block.
   - Passing `f64::NAN` or `f64::INFINITY` into `emit_linear_or_rapid` / `format_finite_decimal` returns `Err(PostError::NonFiniteValue)` instead of emitting `NaN`/`inf`.
2. **Backward Compatibility & Golden Suite:**
   - `cargo test --locked -p chip-post -p chip-tools -p chip-cli` — all 15 byte-for-byte golden G-code snapshots continue to match without diff.
   - `cargo fmt --all -- --check` and `cargo clippy --locked --workspace --all-targets -- -D warnings`.
