# AGENTS.md — ChipCutter Agent Guidelines

**ChipCutter** is an offline desktop CAM application (`eframe` + `egui` + `wgpu`) and headless CLI (`chipcutter`) written in Rust that converts 3D **STEP** B-Rep models into verified 2.5D CNC milling/routing G-code.

---

## 1. Spec-First Workflow
- **Authoritative Source of Truth:** All product, algorithmic, and architectural decisions live in `specs/`. Before proposing changes or writing code for any subsystem, read its corresponding spec document below.
- **No Unsolicited Code Generation:** While iterating on specifications, do **not** scaffold `Cargo.toml` or `.rs` files until the user explicitly asks to begin implementation.
- **Keep Specs Synchronized:** Whenever an architectural decision, file schema, or scope boundary changes during implementation, update the relevant `specs/*.md` file in the same task.

### Specification Map
| File | Subsystem |
|---|---|
| `specs/00-overview.md` | Vision, v1 scope boundaries, and explicit non-goals |
| `specs/01-architecture.md` | Cargo workspace layout, data pipeline, units/expressions, `rayon` threading, licensing |
| `specs/02-cad-selection.md` | `CadKernel` trait, `truck` STEP pipeline, 3D B-Rep picking, geometric reload re-matching |
| `specs/03-setups-wcs.md` | Multi-setup model, box/cylinder stock, WCS orientation & origin, Z clearance hierarchy |
| `specs/04-tool-library.md` | Cutter types, material presets, coupled SFM/chipload math, Fusion 360 `.json` import |
| `specs/05-operations.md` | `Dirty`/`Clean` lifecycle, ramps/leads/tabs, and the 5 v1 2.5D toolpath strategies |
| `specs/06-simulation.md` | Playback timeline, cycle time estimation, 2.5D Z-heightmap stock removal & crash checks |
| `specs/07-post-processor.md` | Typed `ClEvent` IR, `.post.toml` profiles, ATC vs split-by-tool, GRBL & LinuxCNC dialects |
| `specs/08-ui-viewport.md` | `eframe` + `wgpu` + `egui_tiles` layout, CAD/CAM render passes, camera presets, Undo/Redo |
| `specs/09-persistence-cli.md` | `.chip.toml` & `.chipz` formats, headless CLI subcommands, and testing strategy |
| `specs/10-roadmap.md` | Phased v1 implementation milestones (M1–M5) and deferred v2+ horizon |

---

## 2. Non-Negotiable Architectural Invariants

1. **UI-Agnostic Core Crates:**
   `chip-math`, `chip-cad`, `chip-tools`, `chip-cam`, `chip-sim`, `chip-post`, and `chip-project` must **never** depend on `egui`, `eframe`, `egui_tiles`, or `wgpu` (except optional headless GPU compute behind an explicit feature flag if ever introduced). Every core capability must work identically from `chip-cli` and `chip-gui`.
2. **Canonical `f64` Millimeter Kernel:**
   Store and compute all internal coordinates, radii, tolerances, and linear feedrates (`mm/min`) in `f64` millimeters. Convert units (`in`, `ipm`, `sfm`) **only** at UI display/expression boundaries and inside `chip-post` when emitting `G20` vs `G21`.
3. **Exact B-Rep Geometry for Toolpaths (Never Viewport Meshes):**
   `f32` triangle tessellations are strictly for `wgpu` rendering and raycast hover/click picking. Toolpaths, Z-heights, and hole centers must always derive from exact `f64` B-Rep curves and surfaces.
4. **True Circular & Helical Arc Preservation:**
   Preserve analytic lines and circular arcs (`ArcPolyline2D` / `cavalier_contours` bulge polylines) through 2D offsetting and into `ClEvent::ArcMove` so CNC controllers receive smooth `G2`/`G3` arcs instead of dense `G1` micro-segments.
5. **CAD Kernel Encapsulation (`CadKernel` Trait):**
   Isolate `truck` (`ruststep`, `truck-topology`, `truck-modeling`, `truck-meshalgo`) strictly inside `chip-cad`. Downstream crates interact only with `CadDocument`, `BodyId`, `FaceId`, `LoopId`, `EdgeId`, and `GeometricSignature`.
6. **Explicit Regeneration (`Ctrl+G`) & Non-Blocking UI:**
   Editing parameters, tools, or setups mutates lightweight document state immediately (for `< 1 ms` Undo/Redo) and marks affected operations `Dirty`. Never run STEP loading, tessellation, polygon offsetting, or simulation synchronously on the `egui` thread; dispatch background `rayon` jobs with cancellation tokens.
7. **Typed CL-Data Post Boundary:**
   Toolpath generators in `chip-cam` emit typed `ClEvent` streams—never G-code strings. Dialect formatting, modal tracking, canned-cycle expansion, and ATC (`M6`) vs. split-file-per-tool partitioning belong exclusively in `chip-post`.

---

## 3. v1 Scope & Dependency Guardrails

- **Permissive Licenses Only:** Third-party crates must use permissive or weak-copyleft licenses (`MIT`, `Apache-2.0`, `BSD-2-Clause`, `BSD-3-Clause`, `MPL-2.0`, `ISC`, `Zlib`, `Unicode-3.0`). Do **not** add `GPL`, `LGPL`, or `AGPL` dependencies (enforced via `deny.toml`).
- **Do Not Introduce Deferred v2+ Scope into v1:**
  - No triangle mesh imports (`STL`, `OBJ`, `3MF`).
  - No 2D sketch drawing tools, text layout, V-carving, or `DXF`/`SVG` importers.
  - No constant-engagement Adaptive/Trochoidal clearing (keep strategies behind `PocketStrategy` so it slots in cleanly in v2).
  - No USB/serial/network machine streaming or jogging (pure offline CAM).
  - No standalone HTML/PDF setup sheets (emit operator setup/tool summaries as structured G-code header comments).

---

## 4. CNC Safety, Rust Style, & Verification Standards

- **Physical Machine Safety First:** Bad CAM output breaks endmills, ruins workpieces, and damages spindles.
  - Never silently fallback to a plunge rapid or clamp invalid heights (`Clearance >= Retract >= Feed >= Top >= Bottom` must be validated explicitly).
  - Guard all geometric kernels against `NaN`, `Inf`, zero-length segments, degenerate zero-radius arcs, and collinear normals; return structured `Result<T, CamError>` diagnostics rather than panicking (`unwrap()`/`expect()` are forbidden in library crates outside tests).
  - Never compare floating-point geometry with bare `==`; use named epsilon constants from `chip-math`.
- **Deterministic Persistence:** Ensure `.chip.toml` and `.post.toml` serialization uses deterministic ordering (`BTreeMap` / sorted slices) so `git diff` stays clean.
- **Verification Checklist (Once Code Exists):**
  - `cargo fmt --all -- --check`
  - `cargo clippy --workspace --all-targets -- -D warnings`
  - `cargo test --workspace`
  - `cargo deny check`
  - Every new or modified toolpath strategy must include integration tests verified against `chip-sim`'s heightmap oracle asserting **zero `G0` rapid-into-stock collisions** and **zero target B-Rep gouges** beyond tolerance.
