# 09 — Project Persistence, Headless CLI, & Testing Strategy

## Project Persistence (`chip-project`)

### 1. Human-Readable `.chip.toml` Format (`schema_version = 2`)
The primary project file is a clean, deterministic, Git-diff-friendly `.chip.toml` manifest:
- **Deterministic Ordering:** Maps and lists serialize in stable order so minor parameter edits produce minimal, readable `git diff`s.
- **Relative Asset Paths:** Imported `.step`, `.dxf`, `.svg`, and custom `.ttf`/`.otf` paths are stored relative to the `.chip.toml` directory whenever under the same project tree (plus SHA-256 content hashes to detect external updates).
- **No Binary Blobs in `.chip.toml`:** Ephemeral cached toolpaths, Voronoi graphs, and tessellations are NOT dumped into `.chip.toml`.
- **Backward-Compatible Schema Migration (`v1 -> v2`):**
  - `schema_version = 2` adds optional `[[sketches]]` layers, Pure-2D (`models = []`) setups, `PocketStrategy::Adaptive`, `RestMachiningConfig`, and `OperationKind::VCarve`.
  - **100% Seamless v1 Compatibility:** Loading any `schema_version = 1` `.chip.toml` or `.chipz` file migrates cleanly in memory with defaulted `sketches = []` and `rest_machining = "disabled"`, preserving identical generated toolpaths and G-code.

#### Example `.chip.toml` (`v2`) Excerpt
```toml
schema_version = 2
project_name = "Front Panel & V-Carve Badge"
display_units = "mm"
models = [] # Optional in v2 for pure-2D router/panel projects!

[[sketches]]
id = "sketch_1_panel"
name = "Panel Cutouts & Text"
anchor = { kind = "setup_xy", setup_id = "setup_1", plane_ref = "stock_top", z_offset_mm = 0.0 }
source_asset = { path = "vectors/panel_outline.dxf", sha256 = "...", import_units = "mm", join_tolerance_mm = 0.05 }

[[sketches.entities]]
id = "ent_badge_title"
kind = "text"
content = "CHIPCUTTER V2"
font = { mode = "bundled_outline_sans" }
cap_height_mm = 12.0

[[setups]]
id = "setup_1"
name = "Setup 1 — Sheet Top"
wcs_slot = "G54"
post_profile = "grbl"

[[setups.operations]]
id = "op_1_vcarve"
name = "Badge Title V-Carve"
kind = "v_carve"
tool_id = "t4_6mm_90deg_vbit"
clearing_tool_id = "t2_3mm_flat_2fl"
```

### 2. Portable `.chipz` Bundle Archive
For sharing self-contained bug reports, archiving shop jobs, or moving projects across machines:
- **File -> Export Portable Bundle (`.chipz`):** Creates a deterministic, zip-slip-safe ZIP archive containing:
  - `project.chip.toml`
  - `models/<imported_step_files>.step` (if any)
  - `vectors/<imported_dxf_or_svg_files>` (if referenced by `SketchLayer`s)
  - `fonts/<custom_font_files>.ttf` (if using non-bundled outline fonts)
  - `posts/<custom_post_profiles>.post.toml` (if using a non-builtin post)

---

## Headless CLI (`chip-cli`)
Binary name: `chipcutter`

| Subcommand | Purpose |
|---|---|
| `chipcutter inspect <file.step \| file.dxf \| file.svg> [--units mm\|in] [--dpi 96]` | Inspects a 3D STEP solid or 2D DXF/SVG vector file, reporting detected units, bounding box, layers, and entity/loop/open-chain counts. |
| `chipcutter generate-sample-step <out.step> [...] [--project-out <job.chip.toml>]` | Writes deterministic programmatic STEP fixtures (and optional seeded `.chip.toml` jobs) to disk. |
| `chipcutter import-2d <input.dxf \| input.svg> -o <project.chip.toml> [--units mm\|in] [--dpi 96] [--join-tol 0.05] [--stock-z 12.0]` *(v2)* | Creates or appends a `SketchLayer` from a DXF/SVG file (with gap-healing & biarc fitting) into a pure-2D or hybrid `.chip.toml` project. |
| `chipcutter validate [--generate] <project.chip.toml \| bundle.chipz>` | Loads project + STEP/vector assets, checks SHA-256 / signature bindings, validates height/tool/parameter/DAG invariants, and optionally generates all enabled toolpaths. |
| `chipcutter simulate <project.chip.toml \| bundle.chipz> [--resolution standard\|fine\|ultra] [--report report.json]` | Runs headless heightmap stock removal & 5-rule gouge/crash checks across all operations (including Adaptive, Rest Machining, and two-stage V-Carve). |
| `chipcutter build <project.chip.toml \| bundle.chipz> [-o <out_dir>] [--setup <name>] [--post <profile>] [--allow-warnings]` | Generates all enabled operations in topological DAG order, runs mandatory heightmap crash/gouge verification, and writes `.nc`/`.ngc` G-code files + `build_summary.json`. |
| `chipcutter reload-step <project.chip.toml> [--step <revised.step>] [-o <out.chip.toml>] [--allow-needs-repick]` | Reloads a revised STEP file and associatively re-binds Setup WCS, face-anchored `SketchLayer`s, and Operation selections. |
| `chipcutter bundle pack / unpack ...` | Packs or safely extracts deterministic `.chipz` archives (including `models/`, `vectors/`, `fonts/`, and `posts/`). |
| `chipcutter tools import-fusion <input.json> -o <output.tools.toml>` | Converts a Fusion 360 JSON/tools library into a native ChipCutter `.tools.toml` library. |

---

## Verification & Testing Strategy
Every crate in the workspace enforces layered automated verification:

1. **Unit & Property Tests (`chip-math`, `chip-sketch`, `chip-tools`, `chip-post`):**
   - Expression parser & mixed-unit round-trip accuracy (`1/4 in == 6.35 mm`).
   - DXF bulge/block parsing, SVG DPI/transform flattening, cubic Bézier biarc approximation error $\le \text{tol}$, gap-healing chainer, TrueType island classification, and Hershey stroke metrics.
   - Radial chip-thinning math, chamfer/V-carve trigonometry, and post-processor modal/splitting invariants.
2. **Synthetic 3D STEP & 2D Vector Fixture Suite (`chip-cad`, `chip-sketch`, `chip-cam`):**
   - Prismatic torture block, tilted WCS block, `v1 -> v2` revised STEP pair, plus synthetic DXF (with deliberate `0.03 mm` open micro-gaps, `INSERT` blocks, and islands) and SVG/Typography V-carve fixtures.
3. **Physical Invariant Tests (`chip-sim` as Test Oracle!):**
   - Every generated toolpath (including **2D Adaptive**, **Rest Machining**, and **Two-Stage V-Carving**) is swept through `chip-sim`'s heightmap verifier asserting:
     - **Zero `G0` rapid collisions** with remaining stock (strictly validating Adaptive `micro_lift_z` traverses and Rest Machining entries!).
     - **Zero target B-Rep model gouges** beyond `0.02 mm`.
     - **Instantaneous Engagement Bound (Adaptive):** Radial engagement width along cutting passes stays bounded by `optimal_load + tolerance` even into sharp `90°`/acute corners.
     - **Pocket & Rest Completeness:** Roughing + Rest Machining leaves zero uncut corner material outside the final tool's physical corner radius.
4. **Golden G-Code Snapshot Tests (`chip-cli`):**
   - All v1 golden `.nc` (GRBL) and `.ngc` (LinuxCNC) snapshots remain byte-for-byte pinned alongside new v2 golden snapshots covering Adaptive + Rest Machining and Pure-2D DXF/Text V-Carving.
