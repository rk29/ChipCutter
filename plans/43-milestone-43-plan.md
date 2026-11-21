# Milestone 43 Implementation Plan — Illustrated Sample Projects Gallery Documentation

## Goal

Create a visual, browseable **Sample Projects Gallery** under `docs/samples/` showing thumbnails of every built-in sample project, linking each thumbnail to a dedicated sample page containing a full-resolution screenshot, an explanation of what the sample demonstrates, and—for every sample with multiple Setups or Operations—an individual simulation screenshot captured at the exact moment **each Setup / Operation completes**.

---

## Proposed Changes

### 1. Deterministic Per-Operation / Per-Setup Completion Capture (`chip-gui`, `scripts/`)

#### [MODIFY] [crates/chip-gui/src/capture.rs](file:///workspace/ChipCutter/crates/chip-gui/src/capture.rs)
#### [MODIFY] [crates/chip-gui/src/main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs)
#### [MODIFY] [crates/chip-gui/src/app/pane_behavior.rs](file:///workspace/ChipCutter/crates/chip-gui/src/app/pane_behavior.rs)
#### [NEW] `scripts/capture_sample_gallery.sh`
- Extend `CaptureSpec` and `chip-gui` CLI capture flags with:
  - `--capture-setup <SETUP_IDX>` — selects a specific Setup (`0`-indexed) before generating & simulating.
  - `--capture-sim-after-op <OP_IDX>` — scrubs the simulation timeline precisely to the `t_end_sec` timestamp where operation `OP_IDX` finishes (using `SimulationTimeline::operations[op_idx].end_time_sec` or last motion span of `op_idx`), so every operation's progressive stock removal is captured deterministically!
- Write `scripts/capture_sample_gallery.sh` to drive `chip-gui --self-test-screenshot` under headless Vulkan (`WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a`) and generate:
  - Full-resolution overview + per-operation / per-setup completion PNGs under `docs/images/samples/<slug>/`.
  - Compact thumbnail PNGs (e.g., `480 px` wide via Python Pillow or ImageMagick/Rust helper) for fast loading on the gallery index page.

---

### 2. Sample Gallery Index & Per-Sample Detail Pages (`docs/samples/`)

#### [NEW] `docs/samples/README.md`
#### [NEW] `docs/samples/<slug>.md` (all 15 built-in samples from `SAMPLE_CATALOG`)
#### [NEW] `docs/images/samples/<slug>/*.png`
#### [MODIFY] [README.md](file:///workspace/ChipCutter/README.md)
#### [MODIFY] [docs/tutorials/README.md](file:///workspace/ChipCutter/docs/tutorials/README.md)
#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- **`docs/samples/README.md`:** Organized visual grid/table grouped by category (**3D STEP Solids**, **3D Meshes**, **4th-Axis Rotary**, **Pure-2D & DXF**, **V-Carve & Typography**) displaying each sample's thumbnail, category badge, setup/operation count, 1-line description, and link to its detail page:
  1. `cam-torture` — CAM Torture Block (8 operations)
  2. `adaptive-rest` — 2D Adaptive Clearing + Rest Machining (multi-op)
  3. `pure-2d` — Pure-2D Router Front Panel (DXF, multi-op)
  4. `nameplate` — Engraved Nameplate (TrueType + Hershey, multi-op)
  5. `vcarve` — 3D V-Carve Sign & Male Inlay (2 Setups / multi-op)
  6. `multi-feature` — Prismatic Multi-Feature Bracket (multi-op)
  7. `tilted-bracket` — Tilted 30°/25° STEP Bracket (WCS alignment)
  8. `revision-v1` — Associative Reload Bracket v1 (multi-op)
  9. `revision-v2` — Associative Reload Bracket v2 (multi-op)
  10. `sample-plate` — Sample Machined Plate
  11. `mesh-dome` — Sculpted Dome STL (3D Adaptive + Waterline + Pencil, 3 ops)
  12. `mesh-wave` — Relief Wave OBJ (3D Parallel / Scallop surfacing, multi-op)
  13. `rotary-shaft` — 4th-Axis Indexed Shaft (continuous spiral + 4× indexed 3+1 flats)
  14. `rotary-wrap` — 4th-Axis Wrapped Cylinder Engraving (`Z+X+A`)
  15. `rotary-4axis` — Simultaneous 4-Axis Helical Flute Shaft (`X+Y+Z+A`)
- **Each `docs/samples/<slug>.md` page includes:**
  - Title, category, launch command (`cargo run --release -p chip-gui -- --sample <slug>`), and full-resolution hero screenshot.
  - Clear explanation of what the part/project demonstrates, stock & WCS setup, tools used, and key takeaways.
  - **Progressive Setup & Operation Walkthrough:** For every sample with `> 1` Setup or `> 1` Operation, a section showing the exact simulation screenshot at the completion of each operation/setup alongside a brief description of what material that operation removed.
- Link `docs/samples/README.md` prominently from `README.md` and `docs/tutorials/README.md`, and mark **Phase 12 (`M41`–`M43`)** completed in `specs/10-roadmap.md`.

---

## Verification Plan

1. **Visual & Documentation Integrity Audit:**
   - Visually inspect representative full screenshots, thumbnails, and per-operation completion frames via `read_file` to verify crisp rendering, accurate progressive stock removal, and zero missing-glyph boxes or UI clipping.
   - Run automated Markdown & PNG link/orphan audit across `docs/samples/`, `docs/tutorials/`, `README.md`, and `specs/10-roadmap.md` (`0` broken links, `0` orphaned PNGs).
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
