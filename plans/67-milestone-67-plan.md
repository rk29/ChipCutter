# Milestone 67 Implementation Plan — Built-In Undercut & 3D Deburr Sample, E2E Regression Suite, Visual Verification, & Spec Sync

## Goal

Bring Milestones 64–66 together into a cohesive, verifiable showcase and lock down permanent automated regression gates:
1. **Programmatic Undercut & 3D Deburr STEP Fixture & Built-In Sample (`--sample undercut-deburr`):** A single realistic multi-feature aluminum test block containing a **T-Slot**, a **`60°` Dovetail guideway**, a **concave overhang undercut groove**, and **stepped/curved 3D deburrable rims**—programmed end-to-end with `0` critical errors on CPU & GPU Tri-Dexel!
2. **End-to-End Regression & Fault-Injection Suite (`chip-project`, `chip-cli`):** Quantitative Tri-Dexel accuracy gates, CPU-vs-GPU parity across all 10 cutter geometries, fault-injection crash tests for undercut roof/neck violations, and GRBL/LinuxCNC G-code verification (preserving all Phase 1–5 golden snapshots byte-for-byte!).
3. **Headless Vulkan Visual Verification & Spec Synchronization:** Visually verify every new feature in release Vulkan screenshots and synchronize `specs/04-tool-library.md`, `specs/05-operations.md`, `specs/06-simulation.md`, `specs/08-ui-viewport.md`, and `specs/10-roadmap.md` (leaving `docs/tutorials/` completely untouched so parallel tutorial work stays conflict-free!).

---

## Scope & Conflict-Free Documentation Note

- **No `docs/tutorials/` Edits:** Existing and in-progress tutorial files remain untouched; all Phase 18 documentation updates land cleanly in `specs/` and Sample Gallery metadata.
- **No Unplanned Scope:** **2D Sheet Nesting & Multi-Part Arrays** and **PCB Isolation Milling & Drilling (Gerber / Excellon)** remain strictly in `specs/10-roadmap.md`'s `Future Phases`.

---

## Proposed Changes

### 1. Programmatic STEP Fixture & Built-In Sample (`chip-cad`, `chip-project`, `chip-gui`, `chip-cli`)

#### [MODIFY] `crates/chip-cad/src/fixtures.rs`
#### [MODIFY] `crates/chip-cad/src/lib.rs`
#### [NEW] `crates/chip-project/src/demos/undercut_deburr.rs`
#### [MODIFY] `crates/chip-project/src/demos/mod.rs`
#### [MODIFY] `crates/chip-project/src/lib.rs`
#### [MODIFY] `crates/chip-gui/src/panels/welcome_gallery.rs`
#### [MODIFY] `crates/chip-gui/src/app/loader/sample_loaders.rs`
#### [MODIFY] `crates/chip-gui/src/main.rs`
#### [MODIFY] `crates/chip-cli/src/main.rs`
#### [MODIFY] `crates/chip-cli/src/commands/generate_sample.rs`
- **Programmatic STEP Fixture (`sample_undercut_deburr_block_step_bytes`):**
  - Clean, watertight B-Rep aluminum manifold/fixture block (`100 × 80 × 28 mm`) featuring:
    - Stepped top plateau & sloped bevel rim (for **3D Edge Deburring** with Chamfer Mill / Ball Endmill),
    - Through **T-Slot** channel (`8.5 mm` throat × `14.5 mm` head × `6 mm` head height at bottom),
    - Through **`60°` Dovetail** guideway slot (`10 mm` throat × `6 mm` dovetail depth),
    - Side-accessible overhanging flange/groove (for **Lollipop 3D Undercut** & underside back-chamfer).
  - Exposed via `chipcutter generate-sample-step <out.step> --undercut-block [--project-out <out.chip.toml>]`.
- **Built-In Sample Project (`--sample undercut-deburr`, `SampleProjectId::UndercutAndDeburrBlock`):**
  - Single Setup (`Setup 1 — Top Op [G54]`) using `Tri-Dexel` engine with sequenced crash-free operations:
    1. `Face Top` (Flat Endmill)
    2. `Pre-Slot Throats (T-Slot & Dovetail)` (Flat Endmill clearing throats safely before undercut tools enter!)
    3. `M8 T-Slot Undercut` (`OperationKind::SlotUndercut` + `SlotMill`)
    4. `60° Dovetail Guideway` (`OperationKind::Dovetail` + `DovetailCutter`)
    5. `Lollipop Overhang Undercut` (`OperationKind::Undercut3D` + `LollipopEndmill`)
    6. `3D Edge Deburr & Back-Chamfer` (`OperationKind::Deburr3D` + `ChamferMill` / `LollipopEndmill`)
  - Listed in the Sample Projects Gallery (`welcome_gallery.rs`) under a crisp category badge.

---

### 2. End-to-End Regression, Parity, & CLI Verification Suite (`chip-project`, `chip-cli`)

#### [NEW] `crates/chip-project/tests/expanded_machining_e2e.rs`
#### [MODIFY] `crates/chip-cli/tests/cli_end_to_end.rs`
- **`expanded_machining_e2e.rs`:**
  - Full `undercut-deburr` pipeline test: generate all 6 operations -> simulate on **both `CpuExactBackend` and `WgpuComputeBackend` (`TriDexel` Fine)** -> assert:
    - `0` critical errors (`RapidIntoStock`, `TargetModelGouge`, `HolderCollision`, `FixtureCollision`),
    - `max_gouge_mm <= 0.02 mm`,
    - High volumetric accuracy (`In-Tol >= 90%`) verifying T-slot roof intact, `60°` dovetail flank geometry accurate, lollipop undercut cleared, and 3D deburred bevel smooth,
    - Exact CPU-vs-GPU volume & `StockErrorReport` parity,
    - Clean `GRBL` and `LinuxCNC` G-code post-processing (`G0`/`G1`/`G2`/`G3`, valid modal coordinates, zero `NaN`/non-finite output).
  - Fault-injection tests proving:
    - Omitting Op 2 (pre-slot throat clearing) causes `SlotMill` / `DovetailCutter` to trigger neck/flute collision errors in `chip-sim` on both CPU & GPU;
    - Deliberate vertical retract through an undercut roof triggers `RapidIntoStock` / `TargetModelGouge` on both CPU & GPU.
- **CLI E2E Test (`cli_end_to_end.rs`):**
  - `chipcutter generate-sample-step --undercut-block --project-out ...` -> `chipcutter validate` -> `chipcutter simulate --engine tri-dexel --backend auto --json` -> `chipcutter build --post linuxcnc` & `--post grbl`, asserting exit code `0`, `0` critical errors, and clean `.chipz` roundtrip.

---

### 3. Headless Vulkan Visual Verification & Spec Sync

#### [MODIFY] `specs/04-tool-library.md`
#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/06-simulation.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/10-roadmap.md`
- Capture and inspect release headless Vulkan screenshots (`timeout --foreground -k 3 60 env WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -n 220..223 ...`) of `--sample undercut-deburr` showing:
  1. Full setup with Tri-Dexel machined T-slot, `60°` dovetail, lollipop undercut, and 3D deburred edges (Metallic + Error Heatmap),
  2. Close-up / mid-cut frame showing the 3D revolved Dovetail & SlotMill cutters inside their respective undercut channels,
  3. Tool Library Manager showing the new Lollipop / SlotMill / Dovetail schematics.
  Remove all `/tmp/m67_*` temporary PNG files after inspection.
- Synchronize `specs/04-tool-library.md` (10 cutter geometries), `specs/05-operations.md` (`Deburr3D`, `SlotUndercut`, `Dovetail`, `Undercut3D`), `specs/06-simulation.md` (10-cutter CPU & GPU Tri-Dexel undercut & relieved-neck verification), `specs/08-ui-viewport.md` (new icons, Inspector cards, cutter schematics), and `specs/10-roadmap.md` (mark Phase 18 Complete M64–M67 while keeping 2D Sheet Nesting & PCB Isolation Milling in Future Phases).

---

## Verification Plan

1. **Expanded Machining E2E & CLI Tests:**
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-project --test expanded_machining_e2e`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-cli --test cli_end_to_end` (preserving 100% byte-for-byte Phase 1–5 golden G-code parity!)
2. **Headless Vulkan Visual Verification:**
   - Inspect all `/tmp/m67_*.png` captures via `read_file` and clean up afterward.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo check --locked -p chip-stock` & `cargo check --locked -p chip-cam` (pure-CPU default, zero `wgpu`)
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-stock --features gpu-compute --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
