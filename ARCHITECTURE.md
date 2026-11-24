# ChipCutter Architecture

The living specification documents under [`specs/`](specs/) are the single source of truth for ChipCutter's architecture and subsystem design. To avoid duplication and documentation drift, architectural details are maintained directly in those specs:

- **System Architecture & Layered Crate Design:** [`specs/01-architecture.md`](specs/01-architecture.md) — architectural principles, Cargo workspace layout & feature flags, layered crate architecture & dependency rule, end-to-end data pipeline, units/precision/expression engine, DAG threading & caching, and licensing.
- **3D / 2D `wgpu` Viewport & UI Shell:** [`specs/08-ui-viewport.md`](specs/08-ui-viewport.md) — `eframe` + `egui_tiles` + `wgpu` rendering stages, HUDs, modals, Undo/Redo, and preferences.

## Specification Index

| Specification | Scope |
|---|---|
| [`specs/00-overview.md`](specs/00-overview.md) | Vision, product scope, and non-goals |
| [`specs/01-architecture.md`](specs/01-architecture.md) | Workspace layout, layered crate dependencies, data pipeline, units, threading, and licensing |
| [`specs/02-cad-selection.md`](specs/02-cad-selection.md) | `CadKernel` trait, STEP/mesh loaders, unified 3D/2D selection, and associative reload |
| [`specs/03-setups-wcs.md`](specs/03-setups-wcs.md) | Multi-setup & pure-2D jobs, WCS orientation/origin, stock, rotary axes, and workholding |
| [`specs/04-tool-library.md`](specs/04-tool-library.md) | Cutter geometries, analytic envelopes, cutting presets, machines, and Fusion 360 import |
| [`specs/05-operations.md`](specs/05-operations.md) | `Dirty`/`Clean` lifecycle, 2.5D/3D/undercut/deburr/rotary strategies, cutting boundaries, and keep-outs |
| [`specs/06-simulation.md`](specs/06-simulation.md) | Heightfield, Tri-Dexel, and Cylindrical stock kernels (CPU/GPU), timeline, metrology, and 6-rule verifier |
| [`specs/07-post-processor.md`](specs/07-post-processor.md) | Typed `ClEvent` IR, `.post.toml` profiles, ATC vs. split-by-tool, and controller dialects |
| [`specs/08-ui-viewport.md`](specs/08-ui-viewport.md) | Desktop UI shell, theme tokens, 3D/2D viewport render stages, inspectors, and modals |
| [`specs/09-persistence-cli.md`](specs/09-persistence-cli.md) | `.chip.toml` & `.chipz` formats, headless `chipcutter` CLI, and test suites |
| [`specs/10-roadmap.md`](specs/10-roadmap.md) | Completed phases/milestones and future phases |
| [`specs/11-sketches-vectors.md`](specs/11-sketches-vectors.md) | 2D Sketch Layers, parametric shapes, DXF/SVG import, and TrueType/Hershey typography |

For contributor build setup and architectural invariants, see [`DEVELOPMENT.md`](DEVELOPMENT.md) and [`AGENTS.md`](AGENTS.md).
