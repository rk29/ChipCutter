# Milestone 71 Implementation Plan — Full Sample Projects Gallery Parity (`docs/samples/`), GitHub Pages Sync, & Documentation Verification Gate

## Goal

Bring ChipCutter's **Illustrated Sample Projects Gallery (`docs/samples/`)**, **GitHub Pages landing site (`docs/index.html`)**, **Tutorials index (`docs/tutorials/README.md`)**, and root **`README.md`** into **100% parity** with every built-in sample project and every shipped capability across Phases 1–18—backed by an automated Rust parity test & Markdown link/image audit so samples, gallery pages, and tutorials can never drift out of sync again.

---

## Proposed Changes

### 1. Harden & Extend Sample Gallery Screenshot Pipeline (`scripts/capture_sample_gallery.sh`)

#### [MODIFY] `scripts/capture_sample_gallery.sh`
#### [NEW] `docs/images/samples/{two-sided-vise,undercut-deburr,feed-opt-pocket,flip-jig-softjaws,curved-deburr-3d,dovetail-rail-keyway,svg-badge,mesh-3mf-bracket}/*.png`
- Replace bare `xvfb-run -a` in `scripts/capture_sample_gallery.sh` with a clean display-number selector checking both `/tmp/.X<N>-lock` and `/tmp/.X11-unix/X<N>` so stale X11 sockets never hang headless captures.
- Add deterministic hero, `480 px` thumbnail, and per-Setup / per-Operation completion capture blocks for all **8** previously undocumented or newly added samples:
  1. **`two-sided-vise`** (Hero + Setup 1 Op 1–2 with vise + toe clamps + Setup 2 `t = 0.0` carried-over stock + Setup 2 Op 1–2 completion in Tri-Dexel!)
  2. **`undercut-deburr`** (Hero + Ops 1–6 completion in Tri-Dexel: Face, Pre-Slot Throats, M8 T-Slot, `60°` Dovetail, Lollipop Undercut, 3D Edge Deburr!)
  3. **`feed-opt-pocket`** (Hero + per-op completion showing modulated feedrate telemetry in Simulation Timeline!)
  4. **`flip-jig-softjaws`** (Hero + Setup 1 pins & spoilboard + Setup 2 flipped carryover + Setup 3 Auto Soft Jaws!)
  5. **`curved-deburr-3d`** (Hero + ChamferMill wall-trimmed arch + BallEndmill + Lollipop underside back-chamfer in Tri-Dexel!)
  6. **`dovetail-rail-keyway`** (Hero + Face + Entry Bore + Blind Keyway + External `60°` Dovetail Rail in Tri-Dexel!)
  7. **`svg-badge`** (Hero + per-op Pure-2D `.svg` biarc pocket, V-carve accent, & perimeter tabs!)
  8. **`mesh-3mf-bracket`** (Hero + 3D Adaptive + 3D Constant-Scallop completion on `.3mf` mesh!)
- Run `./scripts/capture_sample_gallery.sh <slug>` for all 8 samples and visually inspect every hero & key per-operation PNG via `read_file`.

---

### 2. Dedicated Sample Gallery Pages, Index, GitHub Pages, & Root README Sync

#### [NEW] `docs/samples/two-sided-vise.md`
#### [NEW] `docs/samples/undercut-deburr.md`
#### [NEW] `docs/samples/feed-opt-pocket.md`
#### [NEW] `docs/samples/flip-jig-softjaws.md`
#### [NEW] `docs/samples/curved-deburr-3d.md`
#### [NEW] `docs/samples/dovetail-rail-keyway.md`
#### [NEW] `docs/samples/svg-badge.md`
#### [NEW] `docs/samples/mesh-3mf-bracket.md`
#### [MODIFY] `docs/samples/README.md`
#### [MODIFY] `docs/tutorials/README.md`
#### [MODIFY] `docs/index.html`
#### [MODIFY] `README.md`
#### [MODIFY] `specs/10-roadmap.md`
- Publish standardized per-sample pages matching existing `docs/samples/thread-mill.md` / `cam-torture.md` format (hero screenshot, part/setup/tool summary table, progressive per-Setup & per-Operation completion gallery, CLI launch & simulation commands, and links to the corresponding tutorial!).
- Organize `docs/samples/README.md` cleanly with two new category sections (**Workholding & Multi-Setup Stock Carryover** and **Undercuts & 3D Edge Deburring**), plus adding `feed-opt-pocket`, `svg-badge`, and `mesh-3mf-bracket` to their matching categories—bringing documented gallery pages to **100% parity** with `SAMPLE_CATALOG`!
- Update `docs/index.html` (accurate sample & tutorial counts, updated Key Capabilities cards highlighting GPU Compute, Feed Optimization, 3D Workholding + Stock Carryover, and 3D Deburr / Undercut Milling, plus featured gallery cards for `two-sided-vise` and `undercut-deburr`) and `README.md` (updated tutorial list `00`–`22` and sample count).
- Synchronize `specs/10-roadmap.md` marking Phase 19 (M68–M71) Completed.

---

### 3. Automated Sample/Docs Parity Test & Link Audit (`chip-gui`, `chip-project`)

#### [NEW] `crates/chip-gui/tests/docs_samples_parity.rs` (or unit test inside `welcome_gallery.rs` + `chip-project`)
- Add a durable automated test asserting:
  1. Every `SampleProjectId` in `SAMPLE_CATALOG` (plus `CadRevisionV2Reload`) has a corresponding `docs/samples/<slug>.md` page, a non-empty `docs/images/samples/<slug>/hero.png` and `thumb.png`, and is linked from `docs/samples/README.md`.
  2. Every Markdown link and image reference across `README.md`, `docs/tutorials/*.md`, and `docs/samples/*.md` resolves to an existing file on disk (zero broken links or orphaned image references!).
  3. Every checked-in starter asset in `docs/tutorials/assets/` is documented in `docs/tutorials/README.md`.

---

## Verification Plan

1. **Automated Docs/Gallery Parity & Link Audit:**
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-gui` (including `docs_samples_parity`)
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-project --test samples_coverage_e2e`
   - `bash -n scripts/capture_sample_gallery.sh && bash -n scripts/capture_tutorial_screenshots.sh`
2. **Visual Verification:**
   - Visually inspect captured hero & per-operation screenshots for all 8 newly documented samples via `read_file`.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo check --locked -p chip-stock` & `cargo check --locked -p chip-cam`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-stock --features gpu-compute --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
