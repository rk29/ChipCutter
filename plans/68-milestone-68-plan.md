# Milestone 68 Implementation Plan — Gap-Closing Built-In Sample Projects & Minimal Tutorial Starter Assets

## Goal

Eliminate every remaining built-in sample and starter-asset coverage gap across ChipCutter's entire feature matrix so **every** shipped strategy mode, cutter geometry, workholding wizard, simulation/feed-optimization feature, and CAD/mesh/vector import format has a 1-click, crash-free built-in sample project and reproducible minimal tutorial starter asset—while preserving 100% backward compatibility and Phase 1–5 golden G-code parity.

---

## Feature Coverage Audit (Gaps Addressed by Milestone 68)

| Shipped Feature | Prior Sample Status | Added in Milestone 68 |
|---|---|---|
| Phase 16 Engagement-Aware `FeedOptimizationConfig` (internal-arc scaling, sharp-corner lookahead ramps, radial chip-thinning, timeline feed telemetry) | No dedicated sample | **`feed-opt-pocket`** + `20-feed-opt-pocket.step` |
| Phase 17 `Spoilboard` + **Flip-Jig Dowel-Pin Wizard** (`CylinderKeepOut` pins) + **Auto Soft-Jaw Machining Setup** | `two-sided-vise` covers vise + clamps + carryover only | **`flip-jig-softjaws`** (3 Setups: Op-1 pins + Op-2 carryover + Op-3 Soft Jaws) |
| Phase 18 Non-Planar Sloped-Boss `Deburr3D` (`ChamferMill` inside-corner wall trimming, `BallEndmill`, `LollipopEndmill` underside back-chamfer) | `undercut-deburr` covers 1 planar/overhang op only | **`curved-deburr-3d`** + `21-sloped-boss-deburr.step` & `21-overhang-flange.step` |
| Phase 18 `DovetailMode::ExternalRail` & blind `SlotUndercutMode::Keyway` with `UndercutEntryMode::EntryBoreCenter` | `undercut-deburr` covers `InternalSlot` & `BothEndsThrough` only | **`dovetail-rail-keyway`** + `22-undercut-deburr-block.step` |
| Phase 2 Pure-2D `.svg` Import (cubic Bézier biarc `G2`/`G3` fitting & `96 DPI` scaling) | `pure-2d` uses `.dxf` only | **`svg-badge`** + `13-badge-logo.svg` |
| Phase 5 Unit-Aware `.3mf` 3D Mesh Import | `mesh-dome` is `.stl`, `mesh-wave` is `.obj` | **`mesh-3mf-bracket`** + `08-bracket.3mf` |

---

## Proposed Changes

### 1. Six New Built-In Sample Projects (`chip-project`, `chip-cad`, `chip-gui`)

#### [NEW] `crates/chip-project/src/demos/feature_samples.rs`
#### [MODIFY] `crates/chip-project/src/demos/mod.rs`
#### [MODIFY] `crates/chip-project/src/lib.rs`
#### [MODIFY] `crates/chip-cad/src/fixtures.rs`
#### [MODIFY] `crates/chip-cad/src/lib.rs`
#### [MODIFY] `crates/chip-gui/src/panels/welcome_gallery.rs`
#### [MODIFY] `crates/chip-gui/src/app/loader/{mod.rs, sample_loaders.rs, cad_apply.rs}`
#### [MODIFY] `crates/chip-gui/src/main.rs`
- **Programmatic Fixtures (`chip-cad`):**
  - Add watertight STEP fixture `sample_dovetail_rail_keyway_step_bytes()` (aluminum slide block with an external `60°` dovetail rail along both flanks and a blind top keyway terminating at a pre-drilled entry bore!).
  - Add programmatic watertight `.3mf` generator `sample_sculpted_mount_3mf_bytes()` (unit-aware millimeter 3MF zip archive containing a clean sculpted mounting bracket mesh) and clean `.svg` fixture `sample_svg_badge_bytes()` (cubic Bézier emblem with nested counter-islands at `96 DPI`).
- **Six New `SampleProjectId` Variants & Loaders (`chip-project`, `chip-gui`):**
  1. **`feed-opt-pocket`** (`SampleProjectId::FeedOptPocket`): `05-adaptive-pocket` / sharp-corner pocket + small-radius internal-arc boss contour with `FeedOptimizationConfig` enabled (internal-arc centerline scaling, sharp-corner lookahead deceleration, radial chip-thinning boost) so `SimulationTimeline` clearly reports wide min/avg/max modulated cutting feedrates with `0` critical errors!
  2. **`flip-jig-softjaws`** (`SampleProjectId::FlipJigSoftJaws`): Two-sided contour plate (`02-contour-plate`) mounted on a `Spoilboard` with **Flip-Jig Wizard** `TwoOnAxis` registration pins (`CylinderKeepOut` fixtures + poka-yoke offset), Setup 1 Op-1 pin boring + top contour, Setup 2 `180°` flip with `carryover_from_setup_id = Some("setup_1")`, and Setup 3 generated **Auto Soft-Jaw Machining Setup**!
  3. **`curved-deburr-3d`** (`SampleProjectId::CurvedDeburr3D`): Built on `sample_sloped_boss_stepped_pocket_step` (with overhang step), exercising `Deburr3D` across (a) `ChamferMill` tracing curved non-planar sloped edges with automatic `FlatBvh` inside-corner wall trimming, (b) `BallEndmill` curved rim break, and (c) `LollipopEndmill` underside back-chamfering!
  4. **`dovetail-rail-keyway`** (`SampleProjectId::DovetailRailKeyway`): Built on `sample_dovetail_rail_keyway_step`, sequencing Facing → Entry Bore (`DrillBore`) → Blind Keyway (`SlotUndercut` `Keyway` + `EntryBoreCenter`) → External `60°` Dovetail Rail (`Dovetail` `ExternalRail`) in `Tri-Dexel` mode with `0` critical errors!
  5. **`svg-badge`** (`SampleProjectId::SvgBadgeSign`): Pure-2D job imported directly from `sample_svg_badge_bytes()` (`96 DPI`, cubic Bézier biarc `G2`/`G3` fitting, counter-islands) with Pocket + V-Carve accent + outer Contour + Tabs!
  6. **`mesh-3mf-bracket`** (`SampleProjectId::Mesh3mfBracket`): Unit-aware `.3mf` mesh imported directly from `sample_sculpted_mount_3mf_bytes()` machined with 3D Adaptive Roughing + 3D Constant-Scallop Finishing!
- **Dynamic Sample Count UI (`welcome_gallery.rs`):**
  - Replace hardcoded `"Explore 17 ready-to-machine..."` text with `format!("Explore {} ready-to-machine...", SAMPLE_CATALOG.len())` and add unit tests verifying every `SampleProjectId` slug roundtrips cleanly.

---

### 2. Minimal Tutorial Starter Assets & CLI Generators (`docs/tutorials/assets/`, `chip-cli`)

#### [MODIFY] `crates/chip-cli/src/commands/generate_sample.rs`
#### [NEW] `docs/tutorials/assets/20-feed-opt-pocket.step`
#### [NEW] `docs/tutorials/assets/21-sloped-boss-deburr.step`
#### [NEW] `docs/tutorials/assets/21-overhang-flange.step`
#### [NEW] `docs/tutorials/assets/22-undercut-deburr-block.step`
#### [NEW] `docs/tutorials/assets/13-badge-logo.svg`
#### [NEW] `docs/tutorials/assets/08-bracket.3mf`
- Extend `chipcutter generate-sample-step --tutorial <slug>` with reproducible generators:
  - `20-feed-opt` → `20-feed-opt-pocket.step`
  - `21-deburr-boss` → `21-sloped-boss-deburr.step`
  - `21-deburr-flange` → `21-overhang-flange.step`
  - `22-undercut-block` → `22-undercut-deburr-block.step`
  - `13-pure2d-svg` → `13-badge-logo.svg`
  - `08-bracket-3mf` → `08-bracket.3mf`
- Generate and check in all 6 minimal starter assets under `docs/tutorials/assets/`.

---

## Verification Plan

1. **Automated All-Samples Crash-Free E2E Test (`crates/chip-project/tests/samples_coverage_e2e.rs`):**
   - Generate, validate, simulate (CPU + GPU where applicable), and post-process (`GRBL` + `LinuxCNC`) all 6 new built-in samples asserting `0` critical errors, modulated feedrate range `min_feed < max_feed` on `feed-opt-pocket`, exact carryover & soft-jaw ops on `flip-jig-softjaws`, true `G2`/`G3` biarc emission on `svg-badge`, watertight `.3mf` import on `mesh-3mf-bracket`, and exact Tri-Dexel parity on `curved-deburr-3d` & `dovetail-rail-keyway`.
2. **CLI Starter Generator Verification:**
   - Run `chipcutter generate-sample-step --tutorial ...` for every new starter asset and verify byte-identical match against checked-in `docs/tutorials/assets/*`.
3. **Full Workspace Gate:**
   - `cargo fmt --all -- --check`, pure-CPU `cargo check --locked -p chip-stock` & `-p chip-cam`, `cargo deny check` (default + `flatpak-portal`), `cargo clippy` (all targets & feature sets), `WGPU_BACKEND=vulkan cargo test --locked --workspace` (preserving 100% Phase 1–5 golden G-code parity).
