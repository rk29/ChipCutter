# Milestone 63 Implementation Plan — Two-Sided Flip-Jig & Auto Soft-Jaw Wizards, Built-In Workholding Sample, & Spec Sync

## Goal

Deliver two high-leverage shop-floor workholding automation wizards built on Milestones 60–62—a **Two-Sided Flip-Jig Dowel-Pin Wizard** and an **Auto Soft-Jaw Generator Wizard** (supporting both CNC-machined aluminum soft jaws and 3D-printable `.stl` soft jaws)—showcase the complete Phase 17 workflow in a new built-in sample project (`two-sided-vise`), lock down automated regression tests and visual verification, and synchronize all ChipCutter specifications.

---

## Scope & Conflict-Free Documentation Note

- **No In-Flight Tutorial Churn:** Existing `docs/tutorials/` files are intentionally left untouched in this milestone so parallel tutorial edits remain conflict-free. All new Phase 17 documentation updates go strictly into `specs/` (`03-setups-wcs.md`, `06-simulation.md`, `08-ui-viewport.md`, `09-persistence-cli.md`, `10-roadmap.md`) and `AGENTS.md` (with `G38.2` probing operations explicitly retained in `Future Phases`).

---

## Proposed Changes

### 1. Two-Sided Flip-Jig Dowel-Pin & Auto Soft-Jaw Wizards (`chip-cam`)

#### [NEW] `crates/chip-cam/src/workholding_wizards.rs`
#### [MODIFY] `crates/chip-cam/src/lib.rs`
- **Two-Sided Flip-Jig Dowel-Pin Wizard (`FlipJigConfig` & `build_two_sided_flip_jig`):**
  - Inputs:
    - `flip_axis: FlipAxis` (`XAxis` = flip front-to-back around `Y = y_mid`, `YAxis` = flip left-to-right around `X = x_mid`)
    - `pin_diameter_mm: f64` (standard presets: `4 mm`, `6 mm`, `8 mm`, `1/8"`, `1/4"`), `pin_fit_clearance_mm: f64` (default `0.02 mm` slip/press tuning)
    - `pin_count: FlipJigPinCount` (`TwoOnAxis` = 2 pins lying directly on the flip centerline for guaranteed `180°` symmetry, or `FourSymmetric` = 4 pins mirrored symmetrically across the flip axis; optional asymmetric X/Y offset along the axis so the flipped part can ONLY seat in the single right orientation!)
    - `stock_pin_depth_mm: f64` (through-stock or blind depth), `spoilboard_pin_depth_mm: f64` (default `10 mm`), `edge_margin_mm: f64` (distance outside part silhouette + cutter radius)
  - Exact `f64` Computation:
    - Projects the target part's 2D silhouette (`RegionSet2D`) + stock bounds in Setup 1 WCS.
    - Places pins strictly inside the stock footprint (or expands stock margins cleanly if requested) and strictly outside the part silhouette + `edge_margin_mm`, verifying exact `180°` flip invariance (`mirror(pin_set) == pin_set` within `< 1e-9 mm`!).
  - Outputs (`FlipJigPlan`):
    - Ready-to-insert `SketchLayer` (`"Flip-Jig Registration Pins"`) containing exact `Circle` entities at each pin location.
    - Ready-to-insert Op-1 `Drilling` / `HelicalBore` operation (stock pin holes) + optional `"Spoilboard Dowel Holes"` setup/operation.
    - Ready-to-insert paired **`Setup 2 — Flip Op (G55)`** rotated `180°` around `flip_axis`, with `carryover_from_setup_id = Some(setup_1.id)`, `WcsOrigin` anchored cleanly to the mirrored datum, and matching `FixtureKind::CylinderKeepOut` pin bodies (so Op-1 and Op-2 automatically route rapids around protruding dowel pins!).
- **Auto Soft-Jaw Generator Wizard (`SoftJawConfig` & `build_soft_jaw_plan`):**
  - Inputs:
    - `jaw_width_mm`, `jaw_depth_mm`, `jaw_height_mm`, `jaw_gap_mm` (closed spacer gap between fixed & movable jaws, default `6.0 mm`)
    - `grip_depth_mm` (vertical pocket step depth into top of soft jaws, default `4.0 mm`)
    - `part_clearance_mm` (radial slip fit around part silhouette, default `0.05 mm` aluminum / `0.15 mm` 3D-printed)
    - `corner_relief_radius_mm` (dogbone / corner relief radius at sharp part corners so square corners seat dead-flat without binding, default `1.5 mm` or `tool_r + 0.2 mm`)
    - `mounting_bolt_pattern: SoftJawBoltPattern` (`None`, `Kurt6Inch` `98.425 mm / 3.875"` spacing, `Milo4Inch`, `CarveraModVise`, or `Custom { spacing_mm, bolt_dia_mm, cbore_dia_mm, cbore_depth_mm }`)
    - `output_mode: SoftJawOutputMode` (`MachiningSetup` | `PrintableStl` | `Both`)
  - Exact `f64` Silhouette Projection & Pocket Generation (`chip-geom2d`):
    - Slices the target part B-Rep/mesh across the clamping Z band `[z_clamp_bottom, z_clamp_top]` to compute the exact outer 2D clamping silhouette `RegionSet2D` (handling both prismatic STEP parts and organic STL meshes!).
    - Offsets outward by `part_clearance_mm`, adds internal sharp-corner relief fillets (`corner_relief_radius_mm`), and boolean-intersects with the fixed-jaw and movable-jaw rectangles (separated by `jaw_gap_mm`).
  - Outputs (`SoftJawPlan`):
    - **CNC-Machinable Soft-Jaw Setup (`MachiningSetup`):** Adds a `"Soft Jaws — Fixed & Movable"` Setup + `SketchLayer` containing the exact left/right (or rear/front) jaw pocket contours + ready-to-generate `Facing`, `Pocket2D` / `Adaptive2D` roughing, and `Contour2D` finish operations (with `WcsOrigin` on fixed-jaw datum!).
    - **3D-Printable `.stl` Export (`PrintableStl`):** Generates exact watertight binary STL byte buffers (`fixed_jaw.stl` & `movable_jaw.stl`) with the contoured part pocket, corner reliefs, and optional mounting counterbores—verified watertight (zero open boundary edges, positive volume)!

---

### 2. GUI Wizard Modals & Headless CLI Subcommands (`chip-gui`, `chip-cli`, `chip-project`)

#### [NEW] `crates/chip-gui/src/panels/workholding_wizards.rs`
#### [MODIFY] `crates/chip-gui/src/panels/setup_inspector/fixtures_tab.rs`
#### [MODIFY] `crates/chip-gui/src/app/{mod.rs, modals.rs, menus_toolbar.rs}`
#### [MODIFY] `crates/chip-cli/src/main.rs`
#### [MODIFY] `crates/chip-cli/src/commands/workholding.rs`
- **GUI Wizard Actions & Modals (`chip-gui`):**
  - Accessible both from the **`Setup`** top menu (`Flip-Jig Wizard…`, `Auto Soft-Jaw Wizard…`) and directly inside the Setup Inspector **`[Fixtures]`** tab!
  - Strict two-column modal layout with live preview summary (pin coordinates / symmetry check, jaw pocket depth / clearance / bolt pattern) and 1-click **`Apply to Project`** (recorded as a single atomic `< 1 ms` Undo step!) plus **`Export Soft Jaws (.stl)…`** native/portal file dialog.
- **CLI Subcommands (`chipcutter workholding`):**
  - `chipcutter workholding flip-jig <project.chip.toml> --axis <x|y> --pin-dia <mm> --count <2|4> -o <out.chip.toml>`
  - `chipcutter workholding soft-jaws <project.chip.toml> --grip-depth <mm> --clearance <mm> [--add-setup -o <out.chip.toml>] [--export-stl-dir <dir>]`

---

### 3. Built-In Workholding Sample, Regression Suite, Visual Verification, & Spec Sync

#### [NEW] `crates/chip-project/src/demos/two_sided_vise.rs`
#### [MODIFY] `crates/chip-project/src/demos/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/welcome_gallery.rs`
#### [MODIFY] `crates/chip-gui/src/main.rs`
#### [NEW] `crates/chip-project/tests/workholding_e2e.rs`
#### [MODIFY] `specs/03-setups-wcs.md`
#### [MODIFY] `specs/06-simulation.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/09-persistence-cli.md`
#### [MODIFY] `specs/10-roadmap.md`
#### [MODIFY] `AGENTS.md`
- **Built-In Sample Project (`--sample two-sided-vise`):**
  - Multi-setup two-sided machined aluminum bracket/manifold:
    - **`Setup 1 — Top Op (G54)`:** Held in `ViseJaws` (with parallels) + `ToeClamp`s, showcasing automatic rapid-lift avoidance over clamps and zero `FixtureCollision` errors.
    - **`Setup 2 — Flip Bottom Op (G55)`:** Flipped `180°` with `carryover_from_setup_id = Some("setup_1")`, contoured soft-jaw `ViseJaws` fixture, rest-aware zero-air-cut hat-stock facing & profiling, and cumulative end-to-end `StockErrorReport` (`In-Tol >= 95%`, `0` critical errors).
- **End-to-End Regression Suite (`crates/chip-project/tests/workholding_e2e.rs`):**
  - Full `two-sided-vise` Op-1 → Op-2 generation, fixture avoidance, CPU+GPU Tri-Dexel carryover simulation, zero `FixtureCollision` / `RapidIntoStock` / `TargetModelGouge` gate, and valid `GRBL` + `LinuxCNC` G-code emission.
  - `FlipJigWizard` symmetry invariance test (`mirror(pins) == pins`), part-silhouette clearance test, and generated Op-1 + Op-2 crash-free simulation test.
  - `SoftJawWizard` test asserting both generated aluminum soft-jaw machining operations simulate crash-free AND exported `fixed_jaw.stl` / `movable_jaw.stl` reload cleanly via `chip-cad`'s STL importer as watertight manifolds with exact pocket depth and radial clearance.
- **Spec Synchronization:**
  - Update `specs/03-setups-wcs.md` (workholding fixtures & multi-setup stock carryover), `specs/06-simulation.md` (Rule 6 `FixtureCollision`, multi-setup carryover simulation), `specs/08-ui-viewport.md` (`[Fixtures]` tab, translucent fixture rendering, wizard modals), `specs/09-persistence-cli.md` (`ModelRole::Fixture`, `chipcutter workholding`, `--all-setups`), `specs/10-roadmap.md` (mark Phase 17 complete, keeping `G38.2` probing in Future Phases), and `AGENTS.md` (update scope guardrail item #4).

---

## Verification Plan

1. **Automated Workholding & Wizard E2E Suite:**
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-project --test workholding_e2e`
   - CLI roundtrip smoke tests for `chipcutter workholding flip-jig`, `chipcutter workholding soft-jaws --export-stl-dir`, and `chipcutter simulate --all-setups --json`.
2. **Headless Vulkan Visual Verification:**
   - Capture headless Vulkan screenshots of `--sample two-sided-vise` (Setup 1 with vise + toe clamps + raised rapids, Setup 2 at `t = 0.0` showing flipped carried-over Op-1 stock, and Setup 2 at `t = 1.0` with Error Heatmap) to `/tmp/m63_*.png` and inspect via `read_file`.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo check --locked -p chip-stock` (pure-CPU default, zero `wgpu`)
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-stock --features gpu-compute --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-stock --features gpu-compute`
   - `WGPU_BACKEND=vulkan cargo test --locked -p chip-sim --features gpu-compute`
