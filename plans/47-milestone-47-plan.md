# Milestone 47 Implementation Plan — Built-In Thread Milling Sample, Illustrated Tutorial 18, Gallery Update, & Golden Verification

## Goal

Deliver a complete, self-contained **Thread Milling** starter STEP fixture, built-in Sample Gallery project (`--sample thread-mill`), golden G-code regression coverage (`GRBL` & `LinuxCNC`), and illustrated **Tutorial 18** (`docs/tutorials/18-thread-milling.md`) with visually verified headless Vulkan screenshots—including Tri-Dexel undercut thread-groove inspection—and update the Sample Projects Gallery and GitHub Pages site.

---

## Proposed Changes

### 1. Starter STEP Fixture & Built-In `thread-mill` Sample (`chip-cad`, `chip-project`, `chip-cli`, `chip-gui`)

#### [MODIFY] [crates/chip-cad/src/fixtures.rs](file:///workspace/ChipCutter/crates/chip-cad/src/fixtures.rs)
#### [NEW] `docs/tutorials/assets/18-threaded-manifold.step`
#### [MODIFY] [crates/chip-project/src/demos.rs](file:///workspace/ChipCutter/crates/chip-project/src/demos.rs)
#### [MODIFY] [crates/chip-cli/src/main.rs](file:///workspace/ChipCutter/crates/chip-cli/src/main.rs)
#### [MODIFY] [crates/chip-gui/src/panels/welcome_gallery.rs](file:///workspace/ChipCutter/crates/chip-gui/src/panels/welcome_gallery.rs)
#### [MODIFY] [crates/chip-gui/src/app/mod.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/mod.rs)
#### [MODIFY] [crates/chip-gui/src/app/loader.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/loader.rs)
- Add deterministic STEP fixture `sample_threaded_manifold_step()` (`80 × 60 × 20 mm` aluminum manifold block):
  - Two pre-drilled `M8 × 1.25` pilot holes (`Ø6.80 mm` blind/through)
  - Two pre-drilled `1/4-20 UNC` pilot holes (`Ø5.11 mm`)
  - One proud `Ø12.0 mm` circular boss for external `M12 × 1.75` thread milling
- Wire `chipcutter generate-sample-step <out.step> --tutorial 18-thread-milling` and check in `docs/tutorials/assets/18-threaded-manifold.step`.
- Add `build_thread_mill_manifold_project()` (`5` operations in `G54`):
  1. `Face Stock Top` (`Facing`)
  2. `Drill / Bore Pilot Holes` (`DrillBore` — `Ø6.8 mm` & `Ø5.1 mm` pilot bores)
  3. `Internal M8 x 1.25 Thread Mill` (`ThreadMill` — single-point, bottom-up climb, 2 radial passes + 1 spring pass)
  4. `Internal 1/4-20 UNC Thread Mill` (`ThreadMill` — single-point, bottom-up climb)
  5. `External M12 x 1.75 Boss Thread` (`ThreadMill` — external boss, top-down climb)
- Register **`SampleProjectId::ThreadMillManifold`** (`--sample thread-mill`) in `SAMPLE_CATALOG`, `CommandPalette`, and `ChipCutterApp::load_sample_project`.

---

### 2. Golden G-Code Regression Tests (`chip-cli`)

#### [MODIFY] [crates/chip-cli/tests/cli_end_to_end.rs](file:///workspace/ChipCutter/crates/chip-cli/tests/cli_end_to_end.rs)
#### [NEW] `crates/chip-cli/tests/golden/grbl/v4_thread_mill_*.nc`
#### [NEW] `crates/chip-cli/tests/golden/linuxcnc/v4_thread_mill.ngc`
- Add end-to-end CLI test generating, simulating (both Heightmap and Tri-Dexel, `0` crashes), and building G-code for `thread-mill` across `grbl` and `linuxcnc`, locking byte-exact golden snapshots for helical `G2`/`G3` thread passes.

---

### 3. Illustrated Tutorial 18, Sample Gallery Entry, & Site Updates (`docs/`, `scripts/`)

#### [NEW] `docs/tutorials/18-thread-milling.md`
#### [NEW] `docs/images/tutorials/18-thread-milling/*.png`
#### [NEW] `docs/samples/thread-mill.md`
#### [NEW] `docs/images/samples/thread-mill/*.png`
#### [MODIFY] [scripts/capture_tutorial_screenshots.sh](file:///workspace/ChipCutter/scripts/capture_tutorial_screenshots.sh)
#### [MODIFY] [scripts/capture_sample_gallery.sh](file:///workspace/ChipCutter/scripts/capture_sample_gallery.sh)
#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md)
#### [MODIFY] [docs/samples/README.md](file:///workspace/ChipCutter/docs/samples/README.md)
#### [MODIFY] [docs/index.html](file:///workspace/ChipCutter/docs/index.html)
#### [MODIFY] [README.md](file:///workspace/ChipCutter/README.md)
#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Publish **Tutorial 18: Helical Thread Milling (Internal Holes & External Bosses)** following the standard 7-part tutorial structure:
  - Defining a single-point / multi-form `ThreadMill` in Tool Library Manager
  - Picking ISO Metric (`M8 × 1.25`) & Unified (`1/4-20 UNC`) presets and verifying tap-drill pilot diameter
  - Understanding bottom-up climb vs. top-down cuts, radial multi-passes, spring passes, pitch-diameter gauge tuning, and centerline feedrate compensation
  - Simulating helical undercut grooves in **Tri-Dexel (Volumetric 3D)** mode
  - Posting verified helical `G2`/`G3` G-code for GRBL & LinuxCNC and running headless `chipcutter` CLI equivalents
- Capture all Tutorial 18 screenshots and the `thread-mill` Sample Gallery hero, thumbnail, and 5 per-operation completion screenshots; visually inspect every image via `read_file`.
- Update `docs/samples/README.md` (now 16 built-in samples), `docs/tutorials/README.md` (`00`–`18`), `docs/index.html`, `README.md`, and mark **Phase 13 (Completed — M44–M47)** in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Visual & Documentation Integrity Audit:**
   - Visually inspect Tutorial 18 and `thread-mill` Sample Gallery PNGs via `read_file`.
   - Run automated Markdown & PNG link/orphan audit across `docs/samples/`, `docs/tutorials/`, `docs/index.html`, `README.md`, and `specs/10-roadmap.md` (`0` broken links, `0` orphaned PNGs).
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
