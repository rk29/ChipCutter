# 09 — Project Persistence, Headless CLI, & Testing Strategy

## Project Persistence (`chip-project`)

### 1. Human-Readable `.chip.toml` Format (`schema_version = 3`)
The primary project file is a clean, deterministic, Git-diff-friendly `.chip.toml` manifest (`ProjectDocument::CURRENT_SCHEMA_VERSION = 3`, auto-migrating `v1` and `v2` manifests in memory to `v3`):
- **Deterministic Ordering:** Maps and lists (including embedded `[[tools]]`) serialize in stable order so minor parameter edits produce minimal, readable `git diff`s.
- **Relative Asset Paths:** Imported `.step`/`.stp`, `.stl`/`.obj`/`.3mf`, `.dxf`, `.svg`, and custom `.ttf`/`.otf` paths are stored relative to the `.chip.toml` directory whenever under the same project tree (plus SHA-256 content hashes to detect external updates).
- **No Binary Blobs in `.chip.toml`:** Ephemeral cached toolpaths, Voronoi graphs, BVH trees, volumetric stocks, and tessellations are NOT dumped into `.chip.toml`.
- **Persisted `schema_version = 3` Project, Model, & Setup Fields:**
  - `ProjectDocument.machine_id: Option<String>` (omitted when `None`, defaulting to `"generic"`).
  - `ProjectDocument.workpiece_material: Option<ProjectMaterialSelection>` (*Phase 26*, omitted when `None`, defaulting to built-in `aluminum-6061-t6` for backward compatibility).
  - `ProjectDocument.avoid_regions: Vec<SharedAvoidRegion>` (omitted when empty) for reusable named keep-out zones referenced across operations and Setups.
  - `ModelEntry` supports both ISO STEP B-Rep solids and `.stl`/`.obj`/`.3mf` triangle meshes with optional `role` (`"part"` default/omitted vs. `"fixture"` for imported 3D workholding solids), `unit_override: Option<MeshUnitKind>` (omitted when `None`), and non-destructive `transform: ModelTransform` (`translate_mm`, `rotate_deg`, `scale`, omitted when identity).
  - `Setup.target_bodies: Vec<BodyId>`, optional `Setup.rotary: RotaryConfig` (omitted when disabled), `Setup.fixtures: Vec<FixtureBody>` (omitted when empty), `Setup.auto_avoid_fixtures_on_rapids` (default `true`, omitted when `true`), and `Setup.carryover_from_setup_id: Option<String>` (omitted when `None`)—preserving 100% lossless loading and minimal diffs on legacy projects.
- **Backward-Compatible Schema Migration (`v1 -> v2 -> v3`):**
  - `schema_version = 2` added optional `[[sketches]]` layers, Pure-2D (`models = []`) setups, `PocketStrategyKind::Adaptive(...)`, `RestMachiningConfig` (defaulting to `mode = { mode = "disabled" }`), and `OperationKind::VCarve` (`kind = "vcarve"`).
  - `schema_version = 3` added triangle-mesh models, `ModelEntry.{role, unit_override, transform}`, `ProjectDocument.{machine_id, avoid_regions}`, and `Setup.{target_bodies, rotary, fixtures, auto_avoid_fixtures_on_rapids, carryover_from_setup_id}`.
  - **100% Seamless Legacy Compatibility:** Loading any `schema_version = 1` or `2` `.chip.toml` or `.chipz` file migrates cleanly in memory to `schema_version = 3` with defaulted `sketches = []`, `avoid_regions = []`, `machine_id = None` (`"generic"`), and `rest_machining.mode = { mode = "disabled" }`, preserving identical generated toolpaths and G-code.

#### Example `.chip.toml` (`schema_version = 3`) Excerpt
```toml
schema_version = 3
project_name = "Front Panel & V-Carve Badge"
display_units = "millimeter"
machine_id = "generic"
models = [] # Optional for pure-2D router/panel projects

[[sketches]]
id = "sketch_1_panel"
name = "Panel Cutouts & Text"
anchor = { kind = "setup_xy", setup_id = "setup_1", plane_ref = "stock_top", z_offset = { value_mm = 0.0 } }
source_asset = { path = "vectors/panel_outline.dxf", sha256 = "...", format = "dxf", units = { mode = "millimeters" }, join_tolerance_mm = 0.05 }

[[sketches.entities]]
id = "ent_badge_title"
name = "Badge Title"
entity = { type = "text", content = "CHIPCUTTER V3", font = { kind = "builtin_sans_bold" }, cap_height = { value_mm = 12.0 } }

[[setups]]
id = "setup_1"
name = "Setup 1 — Sheet Top"
wcs_slot = "G54"
post_profile = "grbl"

[[setups.operations]]
id = "op_1_adaptive"
name = "Pocket Clearing"
kind = "pocket_2d"
tool_id = "t1_6mm_flat_3fl"

[setups.operations.passes.rest_machining]
mode = { mode = "disabled" }

[setups.operations.strategy]
strategy_kind = "pocket_2d"
strategy = { type = "adaptive", optimal_load = { expr = "tool_d * 0.2", value_mm = 1.2 } }

[[setups.operations]]
id = "op_2_vcarve"
name = "Badge Title V-Carve"
kind = "vcarve"
tool_id = "t8_6mm_60deg_vbit"

[setups.operations.strategy]
strategy_kind = "vcarve"
flat_depth = { value_mm = 2.0 }
clearing_tool_id = "t1_6mm_flat_3fl"
```

### 2. Portable `.chipz` Bundle Archive
For sharing self-contained bug reports, archiving shop jobs, or moving projects across machines:
- **File -> Export Portable Bundle (`.chipz`):** Creates a deterministic, zip-slip-safe ZIP archive (with cumulative decompression-byte and entry-count caps) containing:
  - `project.chip.toml`
  - `models/<imported_models>` (`.step`/`.stp`, `.stl`/`.obj`/`.3mf` target parts and imported 3D workholding fixture models, if any)
  - `vectors/<imported_dxf_or_svg_files>` (if referenced by `SketchLayer`s)
  - `fonts/<custom_font_files>` (`.ttf`/`.otf` if using non-bundled outline fonts)
  - `posts/<custom_post_profiles>.post.toml` (if using a non-builtin post)
  - `machines/<custom>.machine.toml` (if using a non-builtin machine definition)

---

## Headless CLI (`chip-cli`)
Binary name: `chipcutter` (14 top-level subcommands)

| Subcommand | Purpose |
|---|---|
| `chipcutter inspect <file.step \| .stp \| .stl \| .obj \| .3mf \| .dxf \| .svg> [--units mm\|in] [--mesh-units auto\|mm\|in\|cm\|m\|ft\|um] [--svg-dpi 96] [--join-tol 0.05]` | Inspects a 3D STEP solid, triangle mesh (`.stl`/`.obj`/`.3mf`), or 2D DXF/SVG vector file, reporting detected units, bounding box, watertightness/topology diagnostics, layers, and entity/loop/open-chain counts. |
| `chipcutter import-mesh <input.stl \| .obj \| .3mf> -o <project.chip.toml> [--mesh-units auto\|mm\|in\|...] [--rot-x/--rot-y/--rot-z <deg>] [--scale <f>] [--y-up-to-z-up] [--center-xy] [--drop-z0]` | Imports a 3D triangle mesh (`.stl`, `.obj`, or `.3mf`) into a new `schema_version = 3` `.chip.toml` project with optional unit override and non-destructive initial transform. |
| `chipcutter transform-model <project.chip.toml> [-o <out.chip.toml>] [--rot-x/--rot-y/--rot-z] [--scale \| --scale-x/--scale-y/--scale-z] [--fit-size-x/--fit-size-y/--fit-size-z] [--move-x/--move-y/--move-z] [--y-up-to-z-up] [--center-xy] [--drop-z0 \| --align-top-z0] [--reset]` | Non-destructively rotates, scales, fits, or translates the primary 3D model (STEP or STL/OBJ/3MF) in a `.chip.toml` project and marks dependent operations `Dirty`. |
| `chipcutter import-2d <input.dxf \| input.svg> -o <project.chip.toml> [--project <existing.chip.toml>] [--units auto\|auto_in\|mm\|in\|cm] [--svg-dpi 96] [--join-tol 0.05] [--thickness 6.0]` | Creates a Pure-2D `.chip.toml` project (default sheet thickness `6.0 mm`) or appends a `SketchLayer` onto `--project` from a DXF/SVG file with gap healing and biarc fitting. |
| `chipcutter generate-sample-step <out.step> [--multi-feature \| --tilted \| --cam-torture \| --undercut-block \| --revision-v1 \| --revision-v2 \| --tutorial <slug>] [--project-out <job.chip.toml>] [--empty-project] [--machine <id>]` | Writes deterministic programmatic STEP/mesh/vector fixtures (and optional seeded or empty starter `.chip.toml` jobs) to disk. |
| `chipcutter validate <project.chip.toml \| bundle.chipz> [--generate] [--machine <id>] [--physics] [--physics-json] [--fail-on-physics-critical]` | Loads project + CAD/mesh/vector assets, checks SHA-256 / signature bindings, validates height/tool/parameter/DAG/machine-envelope invariants (optionally against `--machine <id>`), optionally runs cutting-physics machinability analysis (`--physics` / `--physics-json`, returning non-zero on critical tool-snap/stall hazards with `--fail-on-physics-critical`), and optionally generates all enabled toolpaths. |
| `chipcutter simulate <project.chip.toml \| bundle.chipz> [--engine heightmap\|tri-dexel] [--backend auto\|gpu\|cpu] [--resolution standard\|fine\|ultra] [--tolerance <mm>] [--all-setups] [--benchmark] [--json] [--report report.json]` | Runs headless Heightmap, volumetric Tri-Dexel, or 4th-axis Cylindrical stock removal & 6-rule crash/gouge/fixture verification with metrology at `±tolerance` (`0.05 mm` default) across a single Setup or sequentially across `--all-setups` with volumetric carryover. |
| `chipcutter build <project.chip.toml \| bundle.chipz> [-o <out_dir>] [--setup <name>] [--post <profile>] [--allow-warnings]` | Generates all enabled operations in topological DAG order, runs mandatory 6-rule crash/gouge/fixture verification (automatically promoting to volumetric `TriDexel` for multi-setup carryover and undercut ops/cutters: `SlotUndercut`, `Dovetail`, `Undercut3D`, `LollipopEndmill`, `SlotMill`, `DovetailCutter`), and writes `.nc`/`.ngc` G-code files + `build_summary.json`. |
| `chipcutter reload-step <project.chip.toml> [--step <revised.step>] [-o <out.chip.toml>] [--allow-needs-repick]` | Reloads a revised STEP file and associatively re-binds Setup WCS, face-anchored `SketchLayer`s, and Operation selections while preserving `ModelEntry.transform`. |
| `chipcutter bundle pack <project.chip.toml> -o <out.chipz> [--allow-external-assets]` / `unpack <archive.chipz> -o <dir>` | Packs or safely extracts deterministic, zip-slip-hardened `.chipz` archives (including `models/`, `vectors/`, `fonts/`, `posts/`, and `machines/`), blocking out-of-project asset exfiltration unless `--allow-external-assets` is passed. |
| `chipcutter tools import-fusion <input.json \| input.tools> -o <output.tools.toml> [--name <label>]` | Converts a Fusion 360 `.json` or `.tools` (ZIP) tool library into a native ChipCutter `.tools.toml` library. |
| `chipcutter machines list \| inspect <id> \| clone <src> <new-id> [--name <label>] \| export <id> -o <file> \| import <file> [--id <id>]` | Lists, inspects, clones (into user config space), exports, and imports `.machine.toml` CNC machine definitions and their scoped tool libraries. |
| `chipcutter workholding list \| add-vise \| add-clamp \| add-spoilboard \| flip-jig \| soft-jaws ...` | Lists/resolves Setup workholding fixtures, adds parametric vises/clamps/spoilboards, generates symmetric Flip-Jig dowel-pin setups (`flip-jig`), and generates CNC soft-jaw setups and/or watertight 3D-printable `.stl` soft jaws (`soft-jaws --export-stl-dir`). |
| `chipcutter list-posts` | Lists all built-in and discovered custom `.post.toml` post-processor profiles with their ID, display name, and output file extension. |

---

## Verification & Testing Strategy
Every crate in the 13-crate workspace enforces layered automated verification:

1. **Core Kernel Unit & Property Tests (`chip-math`, `chip-bvh`, `chip-geom2d`, `chip-stock`, `chip-sketch`, `chip-tools`, `chip-post`):**
   - Expression parser, recursion/token DoS bounds, and mixed-unit round-trip accuracy (`1/4 in == 6.35 mm`).
   - Flat SAH BVH raycast / closest-point / drop-cutter / push-cutter queries (`chip-bvh`), exact 2D arc-preserving offsetting, boolean `RegionSet2D`, and Voronoi medial-axis extraction (`chip-geom2d`), plus flat fixed-`K` Tri-Dexel / Heightfield / Cylindrical stock kernels and crease-preserving mesh skinning (`chip-stock`).
   - DXF bulge/block parsing, SVG DPI/transform flattening, cubic Bézier biarc approximation error $\le \text{tol}$, gap-healing chainer, TrueType island classification, and Hershey stroke metrics (`chip-sketch`).
   - All 10 cutter `AnalyticCutterEnvelope` geometries (including non-monotonic `ThreadMill`, `LollipopEndmill`, `SlotMill`, `DovetailCutter`), machine definition & Fusion `.tools` ZIP import invariants (`chip-tools`), and post-processor modal/splitting/injection-safety invariants (`chip-post`).
2. **Synthetic 3D STEP, Triangle Mesh, Workholding, & 2D Vector Fixture Suites (`chip-cad`, `chip-sketch`, `chip-cam`, `chip-project`):**
   - Prismatic torture block, tilted WCS block, `v1 -> v2` revised STEP pair, watertight STL/OBJ/3MF sculpted & rotary shafts, multi-blade overhanging helical-loft fixtures, undercut/deburr block, threaded manifold, synthetic DXF/SVG vector fixtures, and cutting-area / keep-out / overhang-safety suites (`strategies_invariants`, `surface3d_quality_m57`, `m72_overhang_safety`, `m74_finishing_overhangs`, `m80_machining_boundary`, `m82_cutting_area_25d`, `m85_avoid_keepout_regions`).
3. **Physical Invariant, CPU-vs-GPU Parity, & Metrology Benchmarks (`chip-stock`, `chip-sim`, `chip-project`):**
   - Generated toolpaths across 2.5D, Adaptive/Rest, V-Carve/Inlay, 3-axis surfacing, 4th-axis rotary (indexed, wrapped, simultaneous), thread milling, deburring, and undercuts are verified across **Heightmap**, volumetric **Tri-Dexel**, and 4th-axis **Cylindrical** engines (with **GPU-vs-CPU differential parity** when `gpu-compute` is enabled) asserting:
     - **All 6 physical crash/gouge/fixture verification rules:** zero rapid-into-stock collisions, zero target model gouges, zero flute/shank/relieved-neck rubs, zero holder/chuck collisions, zero non-center-cutting plunges, and zero **Rule 6 `FixtureCollision`** strikes against vises/clamps/spoilboards/keep-outs (`workholding_e2e`, `expanded_machining_e2e`, `helical_loft_overhang_e2e`, `m73_indexed_carryover`).
     - **Quantitative Metrology Regression Benchmarks (`quality_benchmarks`):** hard bounds on error volume, `RMS`, `max_gouge_mm`, `gouge_volume_mm3`, and `in_tolerance_surface_pct` across 2.5D Adaptive/Rest, 3-axis finishing, 4th-axis rotary, and thread-milling jobs.
4. **Golden G-Code Snapshots & Sample / Documentation Parity (`chip-cli`, `chip-project`, `chip-gui`):**
   - Byte-pinned **`v1`–`v4` golden snapshots** in `crates/chip-cli/tests/golden/` for GRBL (`.nc`) and LinuxCNC (`.ngc`) covering the v1 torture block, v2 Adaptive + Rest Machining and V-Carve/Inlay, v3 3-axis mesh surfacing and 4th-axis rotary (indexed, wrapped, simultaneous), and v4 helical thread milling.
   - End-to-end sample coverage (`samples_coverage_e2e`) and GUI/documentation parity (`docs_samples_parity` in `chip-gui`) verifying every built-in `SAMPLE_CATALOG` entry loads, generates, simulates with zero critical errors, and matches its published `docs/samples/<slug>.md` page and screenshot assets.
