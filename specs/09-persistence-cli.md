# 09 — Project Persistence, Headless CLI, & Testing Strategy

## Project Persistence (`chip-project`)

### 1. Human-Readable `.chip.toml` Format
The primary project file is a clean, deterministic, Git-diff-friendly `.chip.toml` manifest:
- **Deterministic Ordering:** Maps and lists serialize in stable order so minor parameter edits produce minimal, readable `git diff`s.
- **Relative CAD Paths:** Imported STEP file paths are stored relative to the `.chip.toml` directory whenever under the same project tree (plus SHA-256 content hash to detect external CAD file updates on load!).
- **No Binary Blobs in `.chip.toml`:** Ephemeral cached toolpaths and tessellations are NOT dumped into `.chip.toml` (optional local cache in `.chipcutter-cache/` ignored by Git, or regenerated cleanly via `Ctrl+G`).
- **Schema Versioning:** Top-level `schema_version = 1` field with explicit migration hooks for future releases.

#### Example `.chip.toml` Excerpt
```toml
schema_version = 1
project_name = "Motor Mount Plate"
display_units = "mm"

[[models]]
id = "model_1"
path = "cad/motor_mount_v3.step"
sha256 = "8f434346648f6b96df89dda901c5176b10a6d83961dd3c1ac88b59b2dc327aa4"

[[tools]]
id = "t1_6mm_flat_3fl"
tool_number = 1
length_offset_number = 1
diameter_offset_number = 1
name = "6mm 3FL Carbide Flat Endmill"
stickout_length_mm = 24.0
geometry = { type = "flat_endmill", diameter_mm = 6.0, flute_length_mm = 18.0, overall_length_mm = 50.0, shank_diameter_mm = 6.0, flutes = 3 }

[[setups]]
id = "setup_1"
name = "Setup 1 — Top Op"
wcs_slot = "G54"
post_profile = "grbl"

[[setups.operations]]
id = "op_1_pocket"
name = "Center Cavity Pocket"
kind = "pocket_2d"
tool_id = "t1_6mm_flat_3fl"
```

### 2. Portable `.chipz` Bundle Archive
For sharing self-contained bug reports, archiving shop jobs, or moving projects across machines:
- **File -> Export Portable Bundle (`.chipz`):** Creates a standard ZIP archive containing:
  - `project.chip.toml`
  - `models/<imported_step_files>.step`
  - `posts/<custom_post_profiles>.post.toml` (if using a non-builtin post)
- ChipCutter can open `.chipz` archives directly (prompting where to unpack or editing in-place and re-saving the bundle).

---

## Headless CLI (`chip-cli`)
Binary name: `chipcutter`

| Subcommand | Purpose |
|---|---|
| `chipcutter validate [--generate] <project.chip.toml>` | Loads project + STEP models, checks CAD SHA-256 / geometric signature bindings, validates height/tool/parameter invariants (and optionally generates all enabled 2.5D toolpaths in parallel when `--generate` is passed), exiting non-zero on error. |
| `chipcutter build <project.chip.toml> [-o <out_dir>] [--setup <name>] [--post <profile>]` | Generates all enabled operations in parallel, runs crash/gouge verification (failing the build on critical crashes unless `--allow-warnings`), and writes `.nc`/`.ngc` G-code files + JSON summary stats. |
| `chipcutter simulate <project.chip.toml> [--resolution fine] [--report report.json]` | Runs headless heightmap stock removal & gouge/crash checks, outputting cycle times, remaining stock volume, and any collision diagnostics. |
| `chipcutter tools import-fusion <input.json> -o <output.tools.toml>` | Converts a Fusion 360 JSON/tools library into a native ChipCutter `.tools.toml` library. |

---

## Verification & Testing Strategy
Because subtle CAM bugs break physical cutters and scrap workpieces, every crate in the workspace enforces layered automated verification:

1. **Unit & Property Tests (`chip-math`, `chip-tools`, `chip-post`):**
   - Expression parser & mixed-unit round-trip accuracy (`1/4 in == 6.35 mm`).
   - Chamfer tip-offset trigonometry, drill breakthrough math, SFM/chipload coupling.
   - Post-processor modal state machine, quadrant arc splitting, line-length limits (GRBL `<= 80` chars), and comment sanitization.
2. **Synthetic STEP Fixture Suite (`chip-cad`, `chip-cam`):**
   - Programmatically constructed (via `truck-modeling`) and CAD-exported STEP test parts stored in `tests/fixtures/step/`:
     - Prismatic block with blind pocket + circular island + open step + counterbored holes + `45°` chamfers.
     - Rotated/tilted CAD body testing Setup WCS alignment and Z-height inference.
     - "v1 -> v2 revised STEP" pair testing geometric signature re-matching after moving a hole and resizing a pocket.
3. **Physical Invariant Tests (`chip-sim` as Test Oracle!):**
   - Every generated toolpath in integration tests is automatically swept through `chip-sim`'s heightmap verifier asserting:
     - **Zero `G0` rapid collisions** with remaining stock.
     - **Zero target B-Rep model gouges** beyond `0.02 mm`.
     - **Pocket floor completeness:** target pocket region cells reach `Bottom Z ± tolerance` with no uncut slivers when `pocket_width >= tool_d`.
4. **Golden G-Code Snapshot Tests (`chip-cli`):**
   - End-to-end `chipcutter build` snapshots comparing emitted `.nc` (GRBL) and `.ngc` (LinuxCNC) outputs against reviewed golden files.
