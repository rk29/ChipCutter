# Milestone 33 Implementation Plan — Untrusted Archive, Parser DoS, Temp Directory, & `.chipz` Packing Hardening

## Goal

Bound memory, disk, and stack usage across every parser and archive reader that handles untrusted user files (`.chipz`, Fusion 360 `.tools`, `.3mf`, `.dxf`, parametric expressions), eliminate predictable `/tmp` directory creation races during `.chipz` extraction, and prevent untrusted `.chip.toml` manifests from silently exfiltrating out-of-project local files when re-packed into `.chipz` bundles.

---

## Security Findings Addressed

- **Finding #3 (High):** Unbounded decompression in Fusion 360 `.tools` ZIP import & header-trusting zip-bomb bypass in `.chipz` unpacking.
- **Finding #5 (Medium):** Stack overflow in recursive expression parser (`chip-math`), exponential triangle explosion in 3MF `<component>` DAGs (`chip-cad`), and exponential entity expansion in DXF `INSERT` block grids (`chip-sketch`).
- **Finding #7 (Low):** Unintended local file inclusion when re-packing untrusted `.chip.toml` paths into `.chipz`.
- **Finding #8 (Low):** Predictable `/tmp` extraction directories without exclusive creation or `0o700` permissions (CWE-377).

---

## Proposed Changes

### 1. Cumulative Zip-Bomb & Entry-Count Defenses (`chip-project`, `chip-tools`)

#### [MODIFY] [crates/chip-tools/src/fusion_import.rs](file:///workspace/ChipCutter/crates/chip-tools/src/fusion_import.rs)
- Add constants `MAX_FUSION_ARCHIVE_ENTRIES: usize = 256` and `MAX_FUSION_JSON_BYTES: u64 = 32 * 1024 * 1024` (32 MiB) plus `FusionImportError::ArchiveTooLarge` and `FusionImportError::UnsafeArchiveEntry`.
- Reject archives with `archive.len() > MAX_FUSION_ARCHIVE_ENTRIES`, reject symlink entries (`file.is_symlink()`), and read `.json` payloads using `file.by_ref().take(MAX_FUSION_JSON_BYTES + 1)` with an explicit post-read length check (`buf.len() as u64 > MAX_FUSION_JSON_BYTES`).
- Also bound bare `.json` input bytes to `MAX_FUSION_JSON_BYTES`.

#### [MODIFY] [crates/chip-project/src/bundle.rs](file:///workspace/ChipCutter/crates/chip-project/src/bundle.rs)
- Add `MAX_CHIPZ_ENTRIES: usize = 1_024` and reject `archive.len() > MAX_CHIPZ_ENTRIES`.
- Fix `unpack_chipz_bytes` to track `actual_unpacked_bytes: u64`:
  - Compute `let remaining = MAX_CHIPZ_UNPACK_BYTES.saturating_sub(actual_unpacked_bytes);` before reading each entry.
  - Read via `entry.by_ref().take(remaining + 1)`.
  - If `buf.len() as u64 > remaining`, return `Err(BundleError::ArchiveTooLarge)` immediately *before* writing to disk.
  - Increment `actual_unpacked_bytes = actual_unpacked_bytes.saturating_add(buf.len() as u64)` from actual decompressed bytes rather than trusting the ZIP central directory `entry.size()`.

---

### 2. Bounded Recursion & Expansion Limits (`chip-math`, `chip-cad`, `chip-sketch`)

#### [MODIFY] [crates/chip-math/src/expr.rs](file:///workspace/ChipCutter/crates/chip-math/src/expr.rs)
- Add `MAX_EXPR_DEPTH: usize = 64` and `MAX_EXPR_TOKENS: usize = 4_096`, plus `ExprError::TooComplex { detail: &'static str }`.
- Enforce `tokens.len() <= MAX_EXPR_TOKENS` during `tokenize`.
- Track `depth: usize` on `Parser` and increment/decrement on entry to `parse_expr`, `parse_unary`, and `parse_primary` parenthesized/function sub-expressions, returning `Err(ExprError::TooComplex)` when `depth > MAX_EXPR_DEPTH`.

#### [MODIFY] [crates/chip-cad/src/mesh/threemf.rs](file:///workspace/ChipCutter/crates/chip-cad/src/mesh/threemf.rs)
- Add `MAX_3MF_TRIANGLES: usize = 5_000_000` and `MAX_3MF_ARCHIVE_ENTRIES: usize = 1_024`.
- In `emit_object_triangles`, check `out.len().saturating_add(obj.triangles.len()) > MAX_3MF_TRIANGLES` before pushing triangles so an acyclic diamond `<component>` DAG fails cleanly with `CadError::MeshParse` instead of OOMing.
- Also track cumulative actual bytes read in `read_archive_entry_utf8` across `_rels/.rels` and `.model`.

#### [MODIFY] [crates/chip-sketch/src/dxf_import.rs](file:///workspace/ChipCutter/crates/chip-sketch/src/dxf_import.rs)
- Add `MAX_DXF_IMPORTED_ITEMS: usize = 250_000` and `MAX_DXF_INSERT_EXPANSIONS: usize = 100_000`.
- Pass a mutable counter/limit into `collect_dxf_entity_with_env` so deeply nested `64×64` `INSERT` block grids stop cleanly with a diagnostic warning (or `DxfImportError::DxfParse`) rather than hanging or exhausting RAM.

---

### 3. Exclusive Private Temp Extraction & Out-of-Project `.chipz` Packing Guard (`chip-project`, `chip-cli`, `chip-gui`)

#### [MODIFY] [crates/chip-project/src/bundle.rs](file:///workspace/ChipCutter/crates/chip-project/src/bundle.rs)
- Add `pub fn create_private_temp_extract_dir(prefix: &str) -> Result<PathBuf, BundleError>`:
  - Generates a non-predictable directory name under `std::env::temp_dir()` using OS randomness (or hashed high-res monotonic/process entropy with retry loop on `ErrorKind::AlreadyExists`).
  - Creates the directory exclusively via `std::fs::create_dir` (never `create_dir_all` on an existing path, eliminating symlink pre-creation races) and sets `0o700` permissions on Unix (`#[cfg(unix)] std::os::unix::fs::PermissionsExt`).
- Add `allow_external_paths: bool` (default `false` in `ChipzPackOptions` for programmatic/untrusted packing, or explicit opt-in with inspection list `external_files_referenced(&ProjectDocument, Option<&Path>) -> Vec<PathBuf>`):
  - Detect any model, vector, or `SketchFontSource::CustomFile` path that resolves outside `project_dir` (via canonicalization or `..` / absolute paths) and validate expected file extensions (`.step`/`.stp`/`.stl`/`.obj`/`.3mf`, `.dxf`/`.svg`, `.ttf`/`.otf`) before packing.

#### [MODIFY] [crates/chip-cli/src/commands/mod.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/mod.rs)
#### [MODIFY] [crates/chip-gui/src/app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs)
- Switch `.chipz` temporary extraction in both `chip-cli` and `chip-gui` to use `create_private_temp_extract_dir(...)`.
- In `chip-gui` `export_chipz_dialog` and `chip-cli` `bundle pack`, warn clearly (or require `--allow-external-assets` in CLI / confirmation prompt in GUI) if a `.chip.toml` references files outside the project directory before bundling them into `.chipz`.

---

## Verification Plan

1. **Negative-Security Unit Tests (No External Dependencies Added):**
   - `chip-tools`: Craft a tiny in-memory ZIP containing a highly compressible `.json` stream exceeding `MAX_FUSION_JSON_BYTES` and assert `import_fusion_library_bytes` returns `Err(FusionImportError::ArchiveTooLarge)` with bounded memory usage.
   - `chip-project`: Craft an in-memory multi-entry `.chipz` where entries exceed cumulative `MAX_CHIPZ_UNPACK_BYTES` and assert `unpack_chipz_bytes` aborts cleanly; test `create_private_temp_extract_dir` verifies `0o700` mode on Unix and refuses symlink collisions; test external-asset detection rejects/flags `/etc/passwd`-style paths in `models` and `CustomFile` fonts.
   - `chip-math`: Test `evaluate_length_mm` with 2,000 nested parentheses `((...1...))` and 2,000 unary minuses `---...1` returns `Err(ExprError::TooComplex)` cleanly without stack overflow.
   - `chip-cad`: Craft a 15-level diamond `<component>` 3MF XML DAG (`2^15 × 200` triangles > `MAX_3MF_TRIANGLES`) and assert `parse_3mf` returns `Err(CadError::MeshParse)` quickly with minimal RAM.
   - `chip-sketch`: Craft a minimal nested `INSERT` DXF with `64×64` grids at depth 3 and assert `import_dxf_bytes` terminates promptly within `MAX_DXF_IMPORTED_ITEMS`.
2. **Full Regression Gate:**
   - `cargo deny check`, `cargo fmt --all -- --check`, `cargo clippy --locked --workspace --all-targets -- -D warnings`, and `WGPU_BACKEND=vulkan cargo test --locked --workspace`.
