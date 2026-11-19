# ChipCutter — Milestone 14 Implementation Plan

## Goal Description
Implement **Milestone 14 (Core 2.5D Milling Tutorials: Facing, 2D Contour, 2D Pocket, & Drilling/Boring)** from [10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md#L90-L94)—delivering four complete, illustrated Markdown tutorials under `docs/tutorials/` with deterministic headless screenshots under `docs/images/tutorials/`:

1. **Tutorial 01 — Facing (Stock Surfacing) (`docs/tutorials/01-facing.md`):**
   - End-to-end walkthrough of decking raw billet stock flat using `CAM Torture Block`'s `Op 1: Face Stock Top` (`T7` `38mm 3-Insert APKT Face Mill`)—covering stock top allowance, WCS origin, pass extension, radial stepover, zig-zag vs. one-way direction & corner roll arcs, Z-height safety planes, 3D simulation check, and GRBL/LinuxCNC post-processing.
2. **Tutorial 02 — 2D Contour / Profile & Holding Tabs (`docs/tutorials/02-2d-contour-tabs.md`):**
   - End-to-end walkthrough of closed outer profiles and open edge chains across both 3D STEP (`CAM Torture Block` `Op 6: Outer Profile + Tabs`) and Pure-2D DXF sheet stock (`Pure-2D CNC Router Front Panel` `Op 4: Outer Panel Profile + Tabs`)—covering closed-loop vs. open-chain picking, side/climb direction, multi-depth stepdowns, rough/finish wall passes, helical/linear ramps, horizontal & vertical tangent lead-in/out arcs, and automatic + click-on-contour rectangular/triangular holding tabs.
3. **Tutorial 03 — 2D Pocket (Multi-Island & Open-Shoulder) (`docs/tutorials/03-2d-pocket.md`):**
   - End-to-end walkthrough of closed multi-island cavities and open-shoulder ledges using `CAM Torture Block` (`Op 4: Island Cavity Pocket`, `Op 3: Open Shoulder Step`, and `Op 2: Plateau Deck Around Boss`)—covering planar face selection & automatic Z-top/Z-bottom/island extraction, contour-parallel offset vs. zig-zag clearing, overhang air-entry expansion on open boundaries, and radial/axial stock-to-leave (`wall`/`floor`) before a finishing pass.
4. **Tutorial 04 — Drilling & Helical Bore Milling (`docs/tutorials/04-drilling-helical-bore.md`):**
   - End-to-end walkthrough of cylindrical holes and 2D bolt-circle point patterns using `CAM Torture Block` (`Op 7: 4x Shelf Through-Holes` with `T6` Ø6mm Stub Drill and `Op 5: Ø18mm Helical Counterbore` with `T1` Ø6mm Flat Endmill) plus `Engraved Nameplate` (`Op 1: 6x Ø6mm Flange Bolt Circle`)—covering `+ Same Ø` batch picking, spot drilling, simple (`G81`) & peck (`G83`/`G73`) cycles, tip-through breakthrough, multi-pitch helical bore interpolation + flat-bottom cleanup, and comparing LinuxCNC native canned cycles against GRBL expanded G-code.
5. **Capture Pipeline Extensions & Index Sync (`crates/chip-gui`, `scripts/capture_tutorial_screenshots.sh`, `docs/tutorials/README.md`):**
   - Add two tiny headless-only flags to `CaptureSpec` (`--capture-post <profile_id>` to switch `G-Code Preview` between `grbl`, `linuxcnc`, and `grblhal_atc`, and `--capture-sim-cutter <true|false>` to keep clean toolpath views unobstructed by the 3D cutter mesh).
   - Extend `scripts/capture_tutorial_screenshots.sh` with `--milestone 14` and per-tutorial slugs (`01-facing`, `02-2d-contour-tabs`, `03-2d-pocket`, `04-drilling-helical-bore`) generating 19 verified screenshots.
   - Update [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md) and [docs/tutorials/00-getting-started.md](file:///workspace/ChipCutter/docs/tutorials/00-getting-started.md) to link directly to tutorials `01`–`04`.

---

## Key Architectural Decisions & Scope Guardrails

> [!IMPORTANT]
> **Zero Core-Crate Changes & Zero New Dependencies:**
> - Only `crates/chip-gui` receives the two tiny headless capture flags (`--capture-post` and `--capture-sim-cutter`); `chip-math`, `chip-cad`, `chip-sketch`, `chip-tools`, `chip-cam`, `chip-sim`, `chip-post`, `chip-project`, and `chip-cli` stay 100% untouched.
> - No new third-party dependencies (`cargo deny check` unchanged).
> - Interactive GUI behavior, `prefs.toml`, and existing `--self-test-screenshot` behavior remain 100% unchanged.

> [!TIP]
> **Consistent 7-Section Structure Across All Four Guides:**
> - Every tutorial follows the standard format documented in [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md#L56-L66):
>   1. *Goal & What You'll Learn*
>   2. *Prerequisites & Starter Project*
>   3. *Step-by-Step GUI Walkthrough* (exact menu/tab names, shortcuts, and verified screenshots)
>   4. *Key Parameters Reference* (concise machinist parameter table + safety callouts)
>   5. *Generate (`Ctrl+G`) & 3D Simulation Verification*
>   6. *Post-Processing (GRBL & LinuxCNC)*
>   7. *Headless CLI Equivalent (`chipcutter`)* + *Next Steps* navigation links.

---

## Proposed Changes

### 1. Tiny Headless Capture Extensions (`crates/chip-gui`)

#### [MODIFY] [capture.rs](file:///workspace/ChipCutter/crates/chip-gui/src/capture.rs), [main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs), & [pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs)
- Add two optional fields to `CaptureSpec`:
  - `pub post_profile: Option<String>` (`--capture-post <grbl|grblhal_atc|linuxcnc>`): When set in `apply_capture_spec`, assigns `self.project.setups[self.active_setup_idx].post_profile` and calls `self.gcode_panel.invalidate_preview()` so G-Code Preview screenshots can compare **LinuxCNC** (`M6`, native `G83`/`G73`) vs. **GRBL** (split-file-per-tool, expanded pecks).
  - `pub show_sim_cutter: Option<bool>` (`--capture-sim-cutter <true|false>`): Controls `self.sim_panel.show_cutter_assembly` so isolated toolpath diagrams remain crisp and unobstructed while 3D simulation shots showcase the animated cutter + holder.
- Add unit tests in `capture.rs` covering `--capture-post` normalization/validation (`grbl`, `grblhal_atc`, `linuxcnc`) and `show_sim_cutter` default/override behavior.

---

### 2. Milestone 14 Screenshot Recipes (`scripts/capture_tutorial_screenshots.sh`)

#### [MODIFY] [capture_tutorial_screenshots.sh](file:///workspace/ChipCutter/scripts/capture_tutorial_screenshots.sh)
Extend the script to support `--milestone 14`, `--milestone all` (`13` + `14`), and `--tutorial 01-facing | 02-2d-contour-tabs | 03-2d-pocket | 04-drilling-helical-bore`, capturing 19 deterministic screenshots:

1. **`docs/images/tutorials/01-facing/` (5 shots — `--sample cam-torture`):**
   - `01-stock-allowance.png` — `setup:stock`, `show_sim_stock=false`, `show_sim_cutter=false`, `show_z_planes=true` (Stock top allowance above `CAM Torture Block`).
   - `02-facing-passes.png` — `op:0:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Facing stepover, pass extension, zig-zag/one-way controls + isolated facing path).
   - `03-facing-heights.png` — `op:0:heights`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false`, `show_z_planes=true` (Clearance/Retract/Feed/`StockTop`/`ModelTop` Z planes).
   - `04-facing-simulation.png` — `op:0:passes`, `pane=sim`, `show_sim_stock=true`, `show_sim_cutter=true`, `sim_progress=0.09` (Freshly decked stock top with `T7` 38mm APKT face mill).
   - `05-facing-gcode.png` — `op:0:passes`, `pane=gcode`, `post_profile=grbl`, `show_sim_stock=true`, `sim_progress=0.09` (Verified GRBL facing `.nc` output for `T7`).

2. **`docs/images/tutorials/02-2d-contour-tabs/` (5 shots — `--sample cam-torture` & `--sample pure-2d`):**
   - `01-contour-geometry.png` — `cam-torture`, `op:5:geometry`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Closed outer bottom loop selection & side/direction controls).
   - `02-contour-passes-tabs.png` — `cam-torture`, `op:5:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Stepdowns, rough/finish wall passes, and Triangular Holding Tabs with 3D tab markers).
   - `03-contour-linking.png` — `cam-torture`, `op:5:linking`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Helical/linear ramps and tangent lead-in/out arcs).
   - `04-pure2d-contour-tabs.png` — `pure-2d`, `op:3:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false`, `view=iso` (Pure-2D DXF `OUTLINE` sheet profile with holding tabs).
   - `05-contour-simulation.png` — `pure-2d`, `pane=sim`, `show_sim_stock=true`, `show_sim_cutter=true`, `sim_progress=0.98`, `view=iso` (Routed sheet panel retained by triangular tabs in 3D machined stock).

3. **`docs/images/tutorials/03-2d-pocket/` (4 shots — `--sample cam-torture`):**
   - `01-island-pocket-geometry.png` — `cam-torture`, `op:3:geometry`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Planar pocket floor selection with inferred Top/Bottom Z & `1 island(s)`).
   - `02-island-pocket-passes.png` — `cam-torture`, `op:3:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Offset vs. Zig-Zag clearing, stepover `tool_d * 0.4`, radial/axial stock-to-leave, & multi-island rings).
   - `03-open-shoulder-pocket.png` — `cam-torture`, `op:2:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Open Shoulder Step showing overhang air-entry expansion past open edges).
   - `04-pocket-simulation.png` — `cam-torture`, `pane=sim`, `show_sim_stock=true`, `show_sim_cutter=true`, `sim_progress=0.58`, `view=iso` (Machined plateau, open shoulder, and island cavity pocket preserving the raised central boss).

4. **`docs/images/tutorials/04-drilling-helical-bore/` (5 shots — `--sample cam-torture` & `--sample nameplate`):**
   - `01-drill-holes-geometry.png` — `cam-torture`, `op:6:geometry`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (`4x Shelf Through-Holes` selected via `+ Same Ø`).
   - `02-drill-peck-passes.png` — `cam-torture`, `op:6:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (Spot / `G81` / `G83` / `G73` cycle options, peck depth, breakthrough, & dwell).
   - `03-helical-bore-passes.png` — `cam-torture`, `op:4:passes`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false` (`Ø18mm Helical Counterbore` in `HelicalBore` mode with multi-pitch spiral `G2`/`G3` path).
   - `04-bolt-circle-2d.png` — `nameplate`, `op:0:geometry`, `--capture-isolate-op`, `show_sim_stock=false`, `show_sim_cutter=false`, `view=iso` (`6x Ø6mm Flange Bolt Circle` parametric 2D point pattern).
   - `05-drill-gcode-linuxcnc.png` — `cam-torture`, `pane=gcode`, `post_profile=linuxcnc`, `show_sim_stock=true`, `show_sim_cutter=true`, `sim_progress=0.95`, `view=iso` (Verified LinuxCNC `.ngc` preview showing native canned cycles & finished drilled/bored stock).

---

### 3. Four Illustrated Core 2.5D Milling Tutorials (`docs/tutorials/`)

#### [NEW] `docs/tutorials/01-facing.md`
- Covers stock top allowance, `T7` face mill vs. flat endmill choice, pass extension (`pass_extension`), radial stepover, zig-zag vs. one-way direction, corner roll-in arcs, `StockTop` → `ModelTop` Z heights, 3D simulation check, GRBL/LinuxCNC G-code output, and `chipcutter` CLI commands.

#### [NEW] `docs/tutorials/02-2d-contour-tabs.md`
- Covers 3D STEP outer profiling + Pure-2D DXF sheet cutouts, closed-loop vs. open-edge chain picking (with side-flip & direction controls), multiple Z stepdowns, rough/finish wall passes, helical/linear entry ramps, horizontal & vertical tangent lead-in/out arcs, and automatic vs. click-on-contour rectangular/triangular holding tabs—verified in 3D simulation & CLI.

#### [NEW] `docs/tutorials/03-2d-pocket.md`
- Covers closed multi-island cavities (`Island Cavity Pocket`), open-shoulder ledges (`Open Shoulder Step`), and open-outer + island combinations (`Plateau Deck Around Boss`), explaining automatic Z-top/Z-bottom inference, contour-parallel offset vs. zig-zag clearing, overhang air-entry expansion, helical/ramp entry, and radial/axial stock-to-leave paired with a finishing pass.

#### [NEW] `docs/tutorials/04-drilling-helical-bore.md`
- Covers 3D cylindrical hole picking (`+ Same Ø` auto-expand) and 2D parametric `PointPattern::BoltCircle` drilling, comparing Spot, Simple (`G81`), Peck (`G83`), Chip-Break (`G73`), and `HelicalBore` interpolation (pitch + flat-bottom cleanup circle), tip-through breakthrough allowance, and side-by-side G-code differences between LinuxCNC native canned cycles and GRBL expanded peck moves.

#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md) & [docs/tutorials/00-getting-started.md](file:///workspace/ChipCutter/docs/tutorials/00-getting-started.md)
- Update the curriculum table in `docs/tutorials/README.md` to mark Tutorials `01`–`04` as **Available** with clickable links, and link `00-getting-started.md`'s *Next Steps* directly to `01-facing.md`.

---

## Verification Plan

1. **Automated Unit Tests & Workspace Gates:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (`0` new third-party dependencies)
   - `WGPU_BACKEND=vulkan cargo test --workspace` (including new `capture.rs` unit tests and byte-identical golden G-code snapshots).
2. **Headless Screenshot Generation & Visual Inspection:**
   - Run `./scripts/capture_tutorial_screenshots.sh --milestone 14 --debug` (or `--release`) under Xvfb + Vulkan Lavapipe.
   - Visually inspect **all 19** captured PNGs across `docs/images/tutorials/01-facing/`, `02-2d-contour-tabs/`, `03-2d-pocket/`, and `04-drilling-helical-bore/` via `read_file` to verify clean framing, legible parameters, isolated toolpaths, machined stock, and G-code previews.
3. **CLI Command & Markdown Link Audit:**
   - Run every `chipcutter` CLI workflow documented in `01`–`04` in `/tmp` and confirm exit code `0` with `0 critical crashes/gouges` and valid GRBL + LinuxCNC G-code output.
   - Run a Python link/image checker across all files in `docs/tutorials/` to verify `0` broken relative links or missing screenshots.
