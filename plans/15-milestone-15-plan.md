# ChipCutter — Milestone 15 Implementation Plan

## Goal Description
Implement **Milestone 15 (Advanced & Specialty Tutorials: Adaptive/Rest, Chamfer/Trace, V-Carving/Inlays, & Final Phase 4 Verification)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L96-L100)—completing **Phase 4 (Illustrated Toolpath Strategy Tutorials)** using the same clean, minimal-model, empty-project approach established in Milestone 14:

1. **Minimal Starter Models & Tutorial Project Builders (`chip-cad`, `chip-project`, `chip-cli`):**
   - **`05-adaptive-pocket.step` (`docs/tutorials/assets/`):** Minimal `100 × 80 × 20 mm` pre-squared block containing **only** one deep `10 mm` closed pocket (`Z = 10..20 mm`) with sharp `90°` corners and a single raised `Ø16 mm` central boss—exercising **2D Adaptive Clearing** (`Ø10 mm` full-depth roughing) followed by **2.5D Rest Machining** (`Ø3 mm` corner cleanup via `FromPrecedingOperations`) with only 2 operations in the CAM Browser.
   - **`06-chamfer-engrave-block.step` (`docs/tutorials/assets/`):** Minimal `100 × 70 × 18 mm` block containing **only** one raised boss with a modeled `45° × 2.0 mm` bevel, one sharp `90°` step edge dropping to a `Z = 14 mm` shelf (for `Floor Guard` verification), and a flat top face for an associative single-line **Hershey** engraving sketch (`"CHIP-06"`)—exercising all 3 `ChamferTrace` modes using a single `T5` `90°` Chamfer Mill.
   - **Minimal Pure-2D V-Carve & Mated V-Inlay Starter (`07-vcarve-inlays`):** Pure-2D project (zero STEP required) starting from a single `2D Sketch` containing bold TrueType text (`"CHIP"` via bundled `DejaVuSans-Bold`) on `120 × 55 × 15 mm` hardwood stock—demonstrating full-depth 3D medial-axis V-carving, Two-Stage Flat Endmill Area Clearance (`Flat Depth = 3.0 mm`), and a mirrored Male V-Inlay Plug (`Setup 2 [G55]`) with zero decorative clutter.
2. **Three Illustrated Advanced & Specialty Tutorials (`docs/tutorials/`):**
   - **`docs/tutorials/05-adaptive-rest-machining.md`** (5 screenshots in `docs/images/tutorials/05-adaptive-rest-machining/`).
   - **`docs/tutorials/06-chamfer-curve-trace.md`** (5 screenshots in `docs/images/tutorials/06-chamfer-curve-trace/`).
   - **`docs/tutorials/07-vcarve-inlays.md`** (5 screenshots in `docs/images/tutorials/07-vcarve-inlays/`).
3. **End-to-End Verification, Link Audit, & Phase 4 Completion (`chip-project`, `scripts/`, Docs):**
   - Extend [capture_tutorial_screenshots.sh](file:///workspace/ChipCutter/scripts/capture_tutorial_screenshots.sh) with `--milestone 15`, `--milestone all`, and per-tutorial targets for `05`, `06`, and `07` (15 new screenshots; 41 total across Phase 4).
   - Expand `minimal_tutorial_projects_pass_heightmap_sim_with_zero_crashes` in `chip-project` to verify all 7 strategy tutorials (`01`–`07`, including both V-Carve setups and GRBL two-stage file splitting) pass `chip-sim` with `0` critical crashes/gouges and emit valid GRBL + LinuxCNC G-code.
   - Mark all 8 tutorials (`00`–`07`) **Available** in [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md), link `04` → `05` → `06` → `07`, audit all links/images/assets (`0` errors), and update [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md), [specs/00-overview.md](file:///workspace/ChipCutter/specs/00-overview.md), [AGENTS.md](file:///workspace/ChipCutter/AGENTS.md), and [README.md](file:///workspace/ChipCutter/README.md) for **Phase 4 Completed**.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Minimal-Feature Models & Zero Unrelated Operations:**
> - Following the Milestone 14 standard, every strategy tutorial starts from an empty project (or blank Pure-2D sketch for V-Carve), loads a minimal starter asset containing only the geometric features needed for that strategy, and adds only the 1–3 operations under study—keeping both the CAM Browser and 3D viewport completely uncluttered.

> [!NOTE]
> **Zero New External Crate Dependencies & Zero Core Algorithm Changes:**
> - All new STEP builders reuse `monstertruck_modeling` already inside `chip-cad`; no core CAM, simulation, or post-processing algorithms are altered; `cargo deny check` and all Phase 1 + Phase 2 golden G-code snapshots remain 100% intact.

---

## Proposed Changes

### 1. Minimal Tutorial STEP Fixtures & Project Builders (`chip-cad`, `chip-project`, `chip-cli`)

#### [MODIFY] [fixtures.rs](file:///workspace/ChipCutter/crates/chip-cad/src/fixtures.rs) & [lib.rs](file:///workspace/ChipCutter/crates/chip-cad/src/lib.rs)
Add two minimal STEP builders + unit tests in `chip-cad`:
- `tutorial_adaptive_pocket_step()`: `100 × 80 × 20 mm` block with a single deep closed pocket `[14..86] × [14..66]` (`Z = 10..20 mm`, sharp `90°` corners) and one raised `Ø16 mm` central cylindrical boss at `(50, 40)`.
- `tutorial_chamfer_engrave_block_step()`: `100 × 70 × 18 mm` block with:
  - Right open step shelf `[76..100] × [0..70]` at `Z = 14 mm` (giving a crisp `X = 76 mm, Z = 18 mm` sharp `90°` edge above a `4 mm`-lower shelf for sharp-edge deburring & `Floor Guard`),
  - Left raised boss `[14..44] × [16..54]` rising from `Z = 18 mm` to `[16..42] × [18..52]` at `Z = 20 mm` with exact `45° × 2.0 mm` sloped planar chamfer walls,
  - Flat open top zone `[48..74] × [10..60]` at `Z = 18 mm` for associative single-line Hershey engraving.

#### [MODIFY] [demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs) & [lib.rs](file:///workspace/ChipCutter/crates/chip-project/src/lib.rs)
Add minimal tutorial project builders (exported from `chip-project`):
- `build_tutorial_adaptive_rest_project(doc, model_path, sha, tools, empty_ops)`:
  - Embeds a `Ø10 mm` 3FL Carbide Rougher (`T10`) and a `Ø3 mm` 2FL Carbide Corner Finisher (`T3`).
  - `Op 1`: **`1. Ø10mm Adaptive Rough`** (`OperationKind::Pocket2D`, `Adaptive`, single full-depth `10 mm` axial cut, `Optimal Load = tool_d * 0.2` = `2.0 mm`, `Chip Thinning` enabled, `Radial Stock = 0.2 mm`).
  - `Op 2`: **`2. Ø3mm Rest Corners`** (`OperationKind::Pocket2D`, `Adaptive`, `Rest Mode = FromPrecedingOperations([op_1])`, clearing only the 4 uncut `R5 mm` corner wedges + wall stock).
- `build_tutorial_chamfer_trace_project(doc, model_path, sha, tools, empty_ops)`:
  - Embeds `T5` (`6mm 90° Carbide Chamfer / Spot Mill`).
  - Attaches one associative `SketchLayer` (`SketchAnchor::PlanarFace` on the `Z = 18 mm` top face) with a single `HersheySimplexSans` text entity `"CHIP-06"`.
  - `Op 1`: **`1. Modeled 45° Boss Bevel`** (`ChamferTrace`, `EdgeChamfer`, `ModeledFace`).
  - `Op 2`: **`2. Sharp Step Edge Deburr`** (`ChamferTrace`, `EdgeChamfer`, `SharpEdge { width: 0.5 mm }`, `Floor Guard = true`).
  - `Op 3`: **`3. Hershey Text Engraving`** (`ChamferTrace`, `CurveTrace`, `Compensation = Center`, `Axial Depth = 0.4 mm`).
- `build_tutorial_vcarve_inlay_project(tools, medial_only: bool)`:
  - Pure-2D project (`models = []`) with a single `SketchLayer` containing `"CHIP"` TrueType outline text (`DejaVuSans-Bold`, `cap_height = 24 mm`) centered on `120 × 55 × 15 mm` hardwood stock.
  - When `medial_only = true`: 1 setup (`G54`) with ONLY `1. Full-Depth Medial V-Carve` (`T8` `60° V-Bit`, `flat_depth = None`) for the pure medial-axis corner-climb walkthrough.
  - When `medial_only = false`:
    - `Setup 1 [G54]`: ONLY `1. Two-Stage Flat Clearance V-Carve` (`T8` `60° V-Bit`, `flat_depth = 3.0 mm`, `clearing_tool = T3 1/8 in Flat Endmill`).
    - `Setup 2 [G55]`: ONLY `1. Male V-Inlay Plug` (`T8` `60° V-Bit` + `T3` clearing tool, `VCarveInlayMode::MalePlug { start_depth: 2.0 mm, glue_gap_bottom: 1.0 mm, auto_mirror_x: true }`).

#### [MODIFY] [inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs) & [main.rs](file:///workspace/ChipCutter/crates/chip-cli/src/main.rs)
- Extend `chipcutter generate-sample-step --tutorial` to accept `05-adaptive-rest`, `06-chamfer-trace`, `07-vcarve-medial`, and `07-vcarve-inlay` (for Pure-2D V-Carve tutorials, writes the `.chip.toml` project cleanly without requiring a `.step` body).

---

### 2. Headless Capture Support & Milestone 15 Shot Recipes (`scripts/capture_tutorial_screenshots.sh`)

#### [MODIFY] [capture_tutorial_screenshots.sh](file:///workspace/ChipCutter/scripts/capture_tutorial_screenshots.sh)
Add `--milestone 15`, `--milestone all` (`13` + `14` + `15`), and per-tutorial slugs (`05-adaptive-rest-machining`, `06-chamfer-curve-trace`, `07-vcarve-inlays`), writing `docs/tutorials/assets/05-adaptive-pocket.step` and `06-chamfer-engrave-block.step` and capturing 15 crisp screenshots:

1. **`docs/images/tutorials/05-adaptive-rest-machining/` (5 shots — `05-adaptive-pocket.step`):**
   - `01-adaptive-geometry.png` — `Op 1` `Geometry` tab showing selected deep pocket floor + `1 island(s)` on minimal pocket block.
   - `02-adaptive-passes.png` — `Op 1` `Passes` tab isolated showing **2D Adaptive Clearing** card (`Optimal Load`, `Cut Direction`, `Micro-Lift Z`, `Chip Thinning` + live RCT feed banner) and trochoidal toolpath.
   - `03-rest-machining-passes.png` — `Op 2` `Passes` tab isolated showing `Rest Mode: Preceding` and toolpaths restricted exclusively to the 4 uncut corner wedges + wall stock.
   - `04-adaptive-rest-simulation.png` — 3D Simulation (`Color by Op`, `sim_progress=0.98`) distinguishing `Ø10mm` Adaptive bulk roughing vs. `Ø3mm` Rest corner cleanup (`Verified Crash-Free`).
   - `05-adaptive-rest-gcode.png` — `G-Code Preview` (`LinuxCNC`) showing `Tools: [10, 3]` and `Sim Verified (0 Crashes)`.

2. **`docs/images/tutorials/06-chamfer-curve-trace/` (5 shots — `06-chamfer-engrave-block.step`):**
   - `01-modeled-chamfer-passes.png` — `Op 1` `Passes` tab isolated showing `Modeled Bevel`, `Tip Offset`, and `Floor Guard` around the `45°` beveled boss.
   - `02-sharp-edge-chamfer.png` — `Op 2` `Passes` tab isolated showing `Sharp Edge`, `Chamfer Width = 0.5 mm`, `Tip Offset`, and `Floor Guard` along the sharp `X = 76 mm` step edge.
   - `03-hershey-sketch-trace.png` — `Op 3` `Passes` tab isolated showing `2D Curve Trace`, `Compensation = Center`, `Axial Depth = 0.4 mm` over associative single-line `"CHIP-06"` Hershey text.
   - `04-chamfer-trace-simulation.png` — 3D Simulation (`sim_progress=0.95`) showing the beveled boss, deburred step edge, and engraved `"CHIP-06"` serial text (`Verified Crash-Free`).
   - `05-chamfer-trace-gcode.png` — `G-Code Preview` (`GRBL`) showing single-tool `T5` verified G-code (`Sim Verified (0 Crashes)`).

3. **`docs/images/tutorials/07-vcarve-inlays/` (5 shots — minimal Pure-2D `"CHIP"` V-Carve projects):**
   - `01-vcarve-medial-axis.png` — Full-depth medial-axis V-Carve (`Flat Floor Limit` off) showing Voronoi bisectors climbing cleanly into sharp serif/letter corners (`T8` `60° V-Bit`).
   - `02-vcarve-two-stage-clearance.png` — `Setup 1 [G54]` `Passes` tab with `Flat Depth = 3.0 mm` + `Two-Stage Flat Area Clearance` (`Clearing Tool = T3 1/8 in Flat Endmill`) showing Stage 1 flat pocketing + Stage 2 rest-machined V-bit bevels.
   - `03-vcarve-simulation-female.png` — 3D Simulation (`Setup 1 [G54]`, `sim_progress=1.0`) showing crisp `60°` beveled female letters with flat floors (`Verified Crash-Free`).
   - `04-vcarve-male-inlay-plug.png` — `Setup 2 [G55]` `Passes` tab (`Mated Male V-Inlay Plug`, `Start Depth = 2.0 mm`, `Glue Gap = 1.0 mm`, `Auto-Mirror X`) + 3D simulated mirrored raised male V-inlay plug.
   - `05-vcarve-gcode-grbl-split.png` — `G-Code Preview` (`GRBL 1.1 / FluidNC (Split by Tool)`) on `Setup 1 [G54]` showing automatic Stage 1 (`T3`) + Stage 2 (`T8`) two-file emission from a single Two-Tool V-Carve operation (`Sim Verified (0 Crashes)`).

---

### 3. Three Illustrated Advanced Tutorials & Phase 4 Completion Docs (`docs/tutorials/`, `specs/`, `README.md`, `AGENTS.md`)

#### [NEW] `docs/tutorials/05-adaptive-rest-machining.md`
- Walks users from an empty project + [`assets/05-adaptive-pocket.step`](../docs/tutorials/assets/05-adaptive-pocket.step) through full-depth `Ø10 mm` 2D Adaptive Clearing (`Optimal Load`, `Cut Direction`, `Micro-Lift Z`, `Stay-Down`, radial `Chip Thinning` feed boost), chaining `Ø3 mm` `FromPrecedingOperations` Rest Machining, observing DAG wave dirty-invalidation & regeneration, verifying corner cleanup in 3D Simulation, and reproducing via `chipcutter` CLI.

#### [NEW] `docs/tutorials/06-chamfer-curve-trace.md`
- Walks users from an empty project + [`assets/06-chamfer-engrave-block.step`](../docs/tutorials/assets/06-chamfer-engrave-block.step) through `T5` `90°` Chamfer Mill tip-offset kinematics (`tip_dia`, `Tip Offset`, `Floor Guard`) on both **Modeled Bevel** faces and **Sharp Edge** deburring, plus anchoring an associative `2D Sketch` on a planar STEP face to engrave single-line **Hershey** text via `2D Curve Trace`.

#### [NEW] `docs/tutorials/07-vcarve-inlays.md`
- Walks users from an empty **Pure-2D** sketch (`"CHIP"` TrueType outline text) through:
  1. Full-depth 3D medial-axis V-carving with sharp corner climbs (`60° V-Bit`),
  2. Clamped `Flat Depth` + **Two-Stage Flat Area Clearance** (Stage 1 Flat Endmill + Stage 2 V-Bit bevels/corners),
  3. **Mated Male V-Inlay Plug** generation (`Setup 2 [G55]`, `Start Depth`, `Glue Gap`, `Auto-Mirror X`) and physical mating clearance explanation, and
  4. Comparing **GRBL split-by-tool** (automatic 2-file split for Stage 1 + Stage 2) vs. **LinuxCNC ATC (`M6`)** output.

#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md), [docs/tutorials/04-drilling-helical-bore.md](file:///workspace/ChipCutter/docs/tutorials/04-drilling-helical-bore.md), [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md), [specs/00-overview.md](file:///workspace/ChipCutter/specs/00-overview.md), [AGENTS.md](file:///workspace/ChipCutter/AGENTS.md), & [README.md](file:///workspace/ChipCutter/README.md)
- Mark all 8 tutorials (`00`–`07`) **Available** with clickable links and list all 6 minimal `.step` assets in `docs/tutorials/README.md`.
- Link `04-drilling-helical-bore.md` *Next Steps* directly to `05-adaptive-rest-machining.md`.
- Mark **Phase 4 — Illustrated Toolpath Strategy Tutorials (Completed — M13–M15)** across `specs/10-roadmap.md`, `specs/00-overview.md`, `AGENTS.md`, and `README.md`.

---

## Verification Plan

1. **Automated Unit & Heightmap Oracle Tests:**
   - `chip-cad`: Test `tutorial_adaptive_pocket_step` and `tutorial_chamfer_engrave_block_step` round-trip dimensions, island pocket, modeled `45°` bevel strip, and adjacent lower step floor detection.
   - `chip-project`: Expand `minimal_tutorial_projects_pass_heightmap_sim_with_zero_crashes` to verify all 7 strategy tutorial projects (`01`–`07`, including both V-Carve setups + GRBL two-stage splitting) generate cleanly, pass `chip-sim` with `0` critical crashes/gouges, and post cleanly to GRBL & LinuxCNC.
2. **Workspace Lint, License, & Regression Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
3. **Headless Screenshot Capture & Visual Inspection:**
   - Run `./scripts/capture_tutorial_screenshots.sh --milestone 15 --debug` under Xvfb + Vulkan Lavapipe.
   - Visually inspect **all 15** new PNGs across `docs/images/tutorials/05-adaptive-rest-machining/`, `06-chamfer-curve-trace/`, and `07-vcarve-inlays/` via `read_file` to verify clean framing, zero UI glitches, and `Sim Verified (0 Crashes)` on every G-code preview.
4. **Full Phase 4 CLI & Markdown Link Audit:**
   - Execute every `chipcutter` CLI workflow documented across `05`–`07` in `/tmp` (`0` critical crashes/gouges).
   - Run the Python link/image/asset validator across ALL 9 files in `docs/tutorials/` (`README.md` + `00`–`07`) and `README.md`, confirming `0` broken links or missing files.
