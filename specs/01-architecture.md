# 01 — System Architecture & Workspace Design

## Architectural Principles
1. **UI-Agnostic Core:** All CAD loading, coordinate transforms, toolpath generation, simulation, and post-processing live in pure-Rust library crates with zero `egui` or `wgpu` dependencies. Both the desktop GUI and headless CLI consume the exact same engine.
2. **Canonical `f64` Millimeter Kernel:** Every internal coordinate, vector, tolerance, arc radius, and feedrate (mm/min) is stored and computed in `f64` millimeters. Unit conversion happens strictly at UI display/input boundaries and post-processor emission (`G20` vs `G21`).
3. **True Arc Preservation:** 2D boundaries, offsets, helical bores, and lead-ins preserve analytic line and circular arc segments (`ArcPolyline2D`) all the way through Cutter Location (CL) output so controllers receive smooth `G2`/`G3` moves rather than faceted micro-segments.
4. **Explicit Dirty-State Computation:** Document edits mutate lightweight parametric specs instantly (enabling cheap snapshot/command Undo/Redo) and mark dependent operations `Dirty`. Heavy toolpath generation runs only on explicit user action (`Ctrl+G` or CLI build) over a `rayon` thread pool with cancellation tokens.
5. **Permissive Dependency Hygiene:** Enforced via `cargo-deny` (`MIT`, `Apache-2.0`, `BSD-2-Clause`, `BSD-3-Clause`, `MPL-2.0`, `ISC`, `Zlib`, `Unicode-3.0`; `GPL`/`LGPL`/`AGPL` denied by default).

---

## Cargo Workspace Layout
```text
ChipCutter/
├── Cargo.toml                  # Workspace root + shared dependency versions
├── deny.toml                   # License & advisory enforcement
├── specs/                      # Living design specifications
├── posts/                      # Built-in TOML post-processor profiles (grbl, linuxcnc)
└── crates/
    ├── chip-math/              # Units, f64 tolerances, transforms, 2D line/arc primitives, expression parser
    ├── chip-cad/               # CadKernel trait, `truck` STEP loader, B-Rep topology IDs, tessellation, reload matching
    ├── chip-tools/             # Tool geometry, material presets, SFM/chipload math, Fusion 360 JSON importer
    ├── chip-cam/               # Setups, WCS, 2.5D feature extraction, 5 operation strategies, CL-data IR
    ├── chip-sim/               # Motion time estimator, 2.5D heightmap dexel stock removal, crash/gouge verifier
    ├── chip-post/              # TOML post-processor engine, modal state machine, G-code formatter
    ├── chip-project/           # `.chip.toml` & `.chipz` serialization, dirty-state graph, Undo/Redo history
    ├── chip-cli/               # Headless CLI binary (`chipcutter`)
    └── chip-gui/               # `eframe` + `wgpu` + `egui_tiles` desktop application
```

---

## End-to-End Data Pipeline
```mermaid
flowchart LR
    STEP["STEP File (.step/.stp)"] --> CAD["chip-cad: BRepModel + Tessellation"]
    CAD --> SETUP["Setup & WCS Transform (World -> WCS)"]
    SETUP --> SEL["B-Rep Selection + Geometric Signatures"]
    SEL --> FEAT["2.5D Planar Contours / Holes (f64 ArcPolyline2D)"]
    TOOLS["chip-tools: Tool + Preset"] --> OP["chip-cam: 2.5D Operation Generator"]
    FEAT --> OP
    OP --> TP["Toolpath IR (Rapids, Linear, Helical/Planar Arcs)"]
    TP --> SIM["chip-sim: Heightmap Stock & Crash Check"]
    TP --> CL["Typed CL-Data Stream"]
    CL --> POST["chip-post: TOML Profile -> .nc / .ngc"]
```

---

## Units, Precision, & Expression Engine (`chip-math`)
- **Internal Storage:**
  - Length: `f64` (`1.0 == 1.0 mm`)
  - Angle: `f64` radians internally, degrees in serialized TOML and UI
  - Linear Feed: `f64` (`mm/min`)
  - Spindle Speed: `f64` (`RPM`)
- **Geometric Tolerances:**
  - Curve chaining / coincidence epsilon: `1e-4 mm` (`0.1 µm`)
  - Toolpath arc fitting / discretization fallback tolerance: configurable per-operation, default `0.01 mm` (`0.0004 in`)
  - 3D viewport tessellation chord tolerance: default `0.02 mm` (visual + ray-picking mesh only; toolpaths derive from exact B-Rep curves/surfaces, never from viewport triangles)
- **Inline Expression & Mixed-Unit Evaluator:**
  - Parameter fields store both the raw user expression string (optional, e.g., `"1/4 in"`, `"tool_d * 0.4"`, `"stock_z - 0.5mm"`) and the resolved `f64` millimeter value.
  - Supported unit suffixes: `mm`, `cm`, `in` / `"` (inch), `mil` / `thou` (`0.0254 mm`), `deg`, `rpm`, `mm/min`, `ipm`, `m/min`, `sfm`.
  - Scoped variables available per operation:
    - Tool scope: `tool_d` (diameter), `tool_r` (radius), `flute_len`, `flutes`, `tip_angle`
    - Setup scope: `stock_x`, `stock_y`, `stock_z`, `model_top_z`, `model_bottom_z`
    - Feature scope: `feature_top_z`, `feature_bottom_z`, `hole_d`

---

## Concurrency & Cancellation Model
- **UI Thread (`chip-gui`):** Never runs STEP parsing, tessellation, polygon offsetting, or heightmap simulation synchronously.
- **Background Worker Pool:** `rayon` thread pool managed by a central `JobDispatcher`.
- **Generation Jobs:**
  - Each `Dirty` operation queued via `Ctrl+G` receives an `Arc<AtomicBool>` cancellation token and emits progress events (`mpsc` channel) to update `egui` spinners.
  - Independent operations within a Setup generate in parallel across `rayon` workers.
  - If the user modifies an operation while it is currently generating, its in-flight token is cancelled and its state returns to `Dirty`.

---

## Core Third-Party Crate Selection
| Responsibility | Crate(s) | License |
|---|---|---|
| GUI Shell & Docking | `eframe`, `egui`, `egui_wgpu`, `egui_tiles`, `rfd` | MIT / Apache-2.0 |
| 3D GPU Rendering | `wgpu`, `bytemuck`, `glam` (f32 GPU / f64 math bridge) | MIT / Apache-2.0 |
| STEP & B-Rep Kernel | `monstertruck-io` (`step`), `monstertruck-modeling`, `monstertruck-meshing` (`truck` family) | Apache-2.0 |
| 2D Arc-Aware Offsetting | `cavalier_contours` | MIT / Apache-2.0 |
| Math & Spatial Queries | `nalgebra` or `glam::DVec3`/`DAffine3`, `rstar` (R-tree spatial index) | Apache-2.0 / MIT |
| Serialization, Hashing, & Paths | `serde`, `toml`, `serde_json`, `zip`, `sha2`, `dirs` | MIT / Apache-2.0 |
| Parallelism & CLI | `rayon`, `crossbeam-channel`, `clap` | MIT / Apache-2.0 |
