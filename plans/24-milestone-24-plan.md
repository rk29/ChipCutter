# Milestone 24 Implementation Plan — Production Workflow Tutorials (`DXF`/`SVG`, Associative Reload, `.chipz`) & Automated Screenshot Harness

## Goal

Complete **Phase 6** by adding two essential production-workflow illustrated tutorials—covering **Pure-2D `DXF`/`SVG` vector jobs + portable `.chipz` bundles** and **Associative STEP Revision Reload (`Ctrl+R`)**—and extending ChipCutter's deterministic headless Vulkan screenshot harness (`scripts/capture_tutorial_screenshots.sh`) and verification suite across **all 15 tutorials (`00`–`14`)**:

1. **`docs/tutorials/13-pure2d-dxf-svg-workflow.md`** — Importing `.dxf` and `.svg` drawings without any 3D STEP solid (units, SVG DPI, sub-layers, endpoint gap healing, island nesting), assigning 2D operations, simulating on a virtual sheet, and packing/building portable `.chipz` bundles.
2. **`docs/tutorials/14-associative-step-reload.md`** — Reloading a revised 3D STEP model (`revision-v1` → `revision-v2` via `Ctrl+R` and `chipcutter reload-step`), reviewing `GeometricSignature` fuzzy re-matching, selective `Dirty` regeneration, and 1-click `NeedsRepick` recovery for deleted/modified features.
3. **Unified Phase 4 + Phase 6 Screenshot Harness & Final Docs Audit** — Extending [scripts/capture_tutorial_screenshots.sh](file:///workspace/ChipCutter/scripts/capture_tutorial_screenshots.sh), verifying every tutorial CLI workflow passes `chip-sim` with `0` crashes, and auditing all links/images across the repository.

---

## Proposed Changes

### 1. Minimal Starter Assets & CLI Support (`docs/tutorials/assets/`, `chip-cli`)

#### [NEW] Deterministic Starter Assets in `docs/tutorials/assets/`
- `docs/tutorials/assets/13-router-panel.dxf` — Exported cleanly from `chip_sketch::sample_router_panel_dxf()` (clean 2D router front panel with outer contour, island pocket, vent slot, and mounting holes).
- `docs/tutorials/assets/14-bracket-v1.step` & `docs/tutorials/assets/14-bracket-v2.step` — Exported cleanly from `chip_cad::sample_revised_part_pair_step()` (`v1` original bracket + `v2` resized pocket, shifted mounting holes, and removed feature demonstrating `NeedsRepick` recovery).

#### [MODIFY] [crates/chip-cli/src/commands/inspect_sample.rs](file:///workspace/ChipCutter/crates/chip-cli/src/commands/inspect_sample.rs)
- Add `--tutorial 13-pure2d-dxf` (writes `13-router-panel.dxf` + optional Pure-2D `--project-out`) and `--tutorial 14-reload-v1` / `14-reload-v2` (writes `14-bracket-v1.step` / `14-bracket-v2.step` + seeded `v1` project) so every single asset in `docs/tutorials/assets/` has a 1-line reproducible `chipcutter` generator command.

---

### 2. Production Workflow Illustrated Tutorials & Screenshots (`docs/tutorials/`, `docs/images/tutorials/`)

#### [NEW] `docs/tutorials/13-pure2d-dxf-svg-workflow.md`
- Follows the uniform 7-part tutorial structure.
- **Step-by-step screenshots (`docs/images/tutorials/13-pure2d-dxf-svg-workflow/`, 5 PNGs):**
  1. `01-dxf-import-inspector.png` — Imported `13-router-panel.dxf` in a Pure-2D job (`0` STEP files) with Sketch Inspector focused (`--capture-focus sketch:0`) showing closed regions, islands, and sub-layers.
  2. `02-pure2d-sheet-stock.png` — `Setup 1 → Stock` showing Pure-2D virtual sheet Z thickness (`6.0 mm`) & Stock Top WCS origin.
  3. `03-operations-assigned.png` — All 4 Pure-2D operations (Drilling/Holes, Island Pocket, Vent Slot, Outer Profile with Holding Tabs) generated with clean arc toolpaths.
  4. `04-pure2d-simulation.png` — 3D Z-dexel virtual-sheet stock removal with `✓ Verified Crash-Free`.
  5. `05-gcode-and-chipz.png` — `G-Code Preview` tab (`grbl`) + walkthrough of **File → Export Portable Bundle (.chipz)…** and headless `chipcutter bundle pack` / `chipcutter build job.chipz`.

#### [NEW] `docs/tutorials/14-associative-step-reload.md`
- Follows the uniform 7-part tutorial structure.
- **Step-by-step screenshots (`docs/images/tutorials/14-associative-step-reload/`, 4 PNGs):**
  1. `01-revision-v1-clean.png` — `revision-v1` loaded and generated with all operations `Clean` (`[OK]`) and `✓ Verified Crash-Free`.
  2. `02-revision-v2-rebind-banner.png` — After associative reload to `revision-v2` (`--sample revision-v2`), showing the non-blocking reload summary banner, automatically re-matched `Dirty` operations (preserving tool/feeds/tabs), and the flagged `NeedsRepick` operation for the deleted hole.
  3. `03-needs-repick-inspector.png` — Focusing the `NeedsRepick` operation showing the **`Clear Orphaned & Re-Pick in 3D`** recovery card.
  4. `04-rebuilt-v2-simulation.png` — Regenerated `v2` toolpaths verified crash-free on the updated STEP solid.

---

### 3. Unified Screenshot Harness & Documentation Synchronization (`scripts/`, `docs/tutorials/README.md`, `README.md`, `specs/10-roadmap.md`)

#### [MODIFY] [scripts/capture_tutorial_screenshots.sh](file:///workspace/ChipCutter/scripts/capture_tutorial_screenshots.sh)
- Add `capture_08_3d_adaptive_waterline`, `capture_09_3d_surface_finishing`, `capture_10_rotary_wrapping_surfacing`, `capture_11_indexed_3plus1_milling`, `capture_12_simultaneous_4axis_milling`, `capture_13_pure2d_dxf_svg_workflow`, and `capture_14_associative_step_reload`.
- Extend `--milestone` CLI flag to accept `13|14|15|22|23|24|all` and `--tutorial <slug>` for every tutorial `00`–`14`.

#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md), [README.md](file:///workspace/ChipCutter/README.md), [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Update the complete 15-row Tutorial Curriculum table (`00`–`14`) and Starter Assets table in `docs/tutorials/README.md`.
- Update `README.md` tutorial index count (`00`–`14`) and `plans/` reference (`01`–`24`).
- Mark **Phase 6 (Milestones 22–24)** completed in `specs/10-roadmap.md` once all verification gates pass.

---

## Verification Plan

1. **Automated Harness & CLI End-to-End Verification:**
   - Run `./scripts/capture_tutorial_screenshots.sh --milestone 24 --debug` (and verify `--milestone 22` / `23` / `all` work cleanly under headless Xvfb + Vulkan Lavapipe).
   - Execute every CLI command in Tutorials `13` and `14` (`inspect`, `import-2d`, `validate`, `simulate`, `build`, `bundle pack`, `reload-step`) verifying exit code `0` and `0` simulation crashes.
2. **Visual Inspection & Full Docs Link Audit:**
   - Visually inspect all 9 new PNGs for Tutorials `13` and `14` via `read_file`.
   - Run automated Markdown link & image audit across `README.md` and all 16 files in `docs/tutorials/*.md` (`0` broken links, `0` orphaned image references).
3. **Full Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (preserving all Phase 1, Phase 2, and Phase 5 CLI goldens byte-for-byte).
