# AGENTS.md — ChipCutter Agent Guidelines

**ChipCutter** is an offline desktop CAM application (`eframe` + `egui` + `wgpu`) and headless CLI (`chipcutter`) written in Rust that converts 3D **STEP** B-Rep models, **2D DXF/SVG vector drawings**, and built-in **2D Sketch/Text layouts** into verified 2.5D CNC milling, adaptive roughing, routing, and V-carving G-code.

---

## 1. Spec-First Workflow
- **Authoritative Source of Truth:** All product, algorithmic, and architectural decisions live in `specs/`. Before proposing changes or writing code for any subsystem, read its corresponding spec document below.
- **Build & Environment Setup:** Contributor toolchain, native dependencies, and headless test setup live in `DEVELOPMENT.md`.
- **Implementation Plans Location:** Store milestone and feature implementation plans in `plans/` (`<nn>-milestone-<nn>-plan.md`, numbered sequentially across phases).
- **Phase Git & Delivery Workflow (Normal Mode of Operation):**
  1. Commit and push roadmap (`specs/10-roadmap.md`) and milestone plans (`plans/`) on `main` first.
  2. Create a local phase branch from `main` (e.g., `phase-<N>-<slug>`) for implementation—do not push the temporary phase branch.
  3. **Subagent & Isolated Git Worktree Execution:** Delegate milestone implementation to subagents running inside isolated `git worktree` checkouts branched off the local phase branch (e.g., `/workspace/ChipCutter-wt-m<NN>` on branch `phase-<N>-m<NN>`, sharing `CARGO_TARGET_DIR=/workspace/ChipCutter/target` for fast incremental builds). When a milestone or adjacent tasks partition cleanly across disjoint files or crates, spawn parallel subagents in separate worktrees with explicit crate/file ownership. Review, verify, land **each milestone in its own verified commit** on the local phase branch, and remove each temporary worktree and branch as soon as its work lands.
  4. When ready to publish (after user confirmation), `git fetch origin`, rebase the phase branch cleanly onto `origin/main`, fast-forward `main`, delete the local phase branch, and push `main`.
- **Phase-Matched Versioning:** Workspace minor versions track the completed phase number (`0.<phase>.0`, tag `v0.<phase>.0`). When bumping versions, synchronize `Cargo.toml`, `Cargo.lock`, golden G-code header comments (`crates/chip-cli/tests/golden/`), `packaging/flatpak/io.github.rk29.ChipCutter.metainfo.xml`, `.github/workflows/release.yml`, and `scripts/package_windows.ps1` in a dedicated commit before tagging.
- **No Unsolicited Code Generation:** While iterating on specifications or plans, do **not** scaffold `Cargo.toml` or `.rs` files until the user explicitly asks to begin implementation.
- **Keep Specs Synchronized:** Whenever an architectural decision, file schema, or scope boundary changes during implementation, update the relevant `specs/*.md` file in the same task.

### Specification Map
| File | Subsystem |
|---|---|
| `specs/00-overview.md` | Vision, Phase 1–4 scope boundaries, and future-phase non-goals |
| `specs/01-architecture.md` | Cargo workspace layout, data pipeline, units/expressions, DAG `rayon` threading, licensing |
| `specs/02-cad-selection.md` | `CadKernel` trait, `monstertruck` STEP pipeline, unified 3D B-Rep & 2D Sketch picking, reload re-matching |
| `specs/03-setups-wcs.md` | Multi-setup & pure-2D job model, box/cylinder stock, WCS orientation & origin, Z clearance hierarchy |
| `specs/04-tool-library.md` | Cutter types, material presets, coupled SFM/chipload & chip-thinning math, Fusion 360 `.json` import |
| `specs/05-operations.md` | `Dirty`/`Clean` lifecycle, Adaptive roughing, 2.5D Rest Machining, V-Carving, and core 2.5D strategies |
| `specs/06-simulation.md` | Playback timeline, cycle time estimation, 2.5D Z-heightmap stock removal & crash/gouge checks |
| `specs/07-post-processor.md` | Typed `ClEvent` IR, `.post.toml` profiles, ATC vs split-by-tool, GRBL & LinuxCNC dialects |
| `specs/08-ui-viewport.md` | `eframe` + `wgpu` + `egui_tiles` shell, Dark/Light design system, Viewport HUDs, Command Palette, Undo/Redo |
| `specs/09-persistence-cli.md` | `.chip.toml` (`schema_version = 1 / 2`) & `.chipz` formats, headless CLI subcommands, testing |
| `specs/10-roadmap.md` | Completed phases/milestones (`M1`+), active phase plans, and future phases |
| `specs/11-sketches-vectors.md` | `chip-sketch` 2D Sketch Layers, parametric shapes, DXF/SVG importers, TrueType & Hershey text |

---

## 2. Non-Negotiable Architectural Invariants

1. **UI-Agnostic Core Crates:**
   `chip-math`, `chip-cad`, `chip-sketch`, `chip-tools`, `chip-cam`, `chip-sim`, `chip-post`, and `chip-project` must **never** depend on `egui`, `eframe`, `egui_tiles`, or `wgpu` (except optional headless GPU compute behind an explicit feature flag if ever introduced). Every core capability must work identically from `chip-cli` and `chip-gui`.
2. **Canonical `f64` Millimeter Kernel:**
   Store and compute all internal coordinates, radii, tolerances, and linear feedrates (`mm/min`) in `f64` millimeters. Convert units (`in`, `ipm`, `sfm`, SVG `DPI`) **only** at UI display/expression boundaries, DXF/SVG import boundaries, and inside `chip-post` when emitting `G20` vs `G21`.
3. **Exact Geometry for Toolpaths (Never Viewport Meshes):**
   `f32` triangle tessellations are strictly for `wgpu` rendering and raycast hover/click picking. Toolpaths, Z-heights, and hole centers must always derive from exact `f64` B-Rep curves/surfaces or exact `f64` `chip-sketch` `ArcPolyline2D` entities.
4. **CAD-Agnostic 2.5D Strategy Boundary & True Arc Preservation:**
   `chip-cam` strategies operate on `ExtractedFeature25D` (never directly on `CadDocument`) and preserve analytic lines and circular arcs (`ArcPolyline2D` / `cavalier_contours` bulge polylines / `kurbo` biarcs) into `ClEvent::ArcMove` so CNC controllers receive smooth `G2`/`G3` moves.
5. **CAD Kernel Encapsulation (`CadKernel` Trait):**
   Isolate `monstertruck` (`monstertruck-io`, `monstertruck-modeling`, `monstertruck-meshing`) strictly inside `chip-cad`.
6. **Explicit Regeneration (`Ctrl+G`), Zero Idle Clones, & Non-Blocking UI:**
   Editing parameters, sketches, tools, or setups mutates lightweight document state immediately (for `< 1 ms` Undo/Redo with zero idle per-frame document clones) and marks affected + downstream Rest Machining operations `Dirty`. Never run STEP/DXF/SVG loading, tessellation, polygon offsetting, Voronoi construction, or simulation synchronously on the `egui` thread; dispatch background `rayon` jobs with cancellation tokens.
7. **Typed CL-Data Post Boundary:**
   Toolpath generators in `chip-cam` emit typed `ClEvent` streams—never G-code strings. Dialect formatting, modal tracking, canned-cycle expansion, and ATC (`M6`) vs. split-file-per-tool partitioning belong exclusively in `chip-post`.
8. **Read-Only Built-In Presets & Upgrade-Safe User Config Space:**
   Compiled-in built-in presets (machine definitions in `chip-tools`, post-processor profiles in `chip-post`) are strictly **read-only** and must never be shadowed or overwritten by project, `.chipz`, or user files so application upgrades update them cleanly. User clones, edits, and imported `.machine.toml` / `.post.toml` files live in OS user configuration space (`dirs::config_dir()/chipcutter/`).

---

## 3. Scope & Future-Phase Guardrails

- **Permissive Licenses Only:** Third-party crates must use permissive or weak-copyleft licenses (`MIT`, `Apache-2.0`, `BSD-2-Clause`, `BSD-3-Clause`, `MPL-2.0`, `BSL-1.0`, `ISC`, `Zlib`, `Unicode-3.0`). Do **not** add `GPL`, `LGPL`, or `AGPL` dependencies (enforced via `deny.toml`).
- **Backward Compatibility:** Preserve lossless loading of `schema_version = 1` and `schema_version = 2` `.chip.toml`/`.chipz` projects and byte-for-byte parity on existing Phase 1 and Phase 2 golden G-code tests unless an intentional bugfix is approved.
- **Unplanned Scope Guardrails:** Do **not** introduce deferred future-phase features without an approved milestone plan:
  - No nonlinear 2D CAD geometric constraint solver (keep 2D sketches direct, parametric, and expression-driven).
  - No USB/serial/network machine streaming or jogging (pure offline CAM).
  - No standalone HTML/PDF setup sheets (emit operator setup/tool summaries as structured G-code header comments).
  - No in-machine `G38.2` probing cycles until scheduled (Phase 17 3D workholding fixtures, safe-rapid avoidance, Rule 6 `FixtureCollision`, multi-setup volumetric Tri-Dexel stock carryover, and Flip-Jig / Auto Soft-Jaw wizards are implemented).

---

## 4. CNC Safety, Rust Style, & Verification Standards

- **Physical Machine Safety First:** Bad CAM output breaks endmills, ruins workpieces, and damages spindles.
  - Never silently fallback to a plunge rapid or clamp invalid heights (`Clearance >= Retract >= Feed >= Top >= Bottom` must be validated explicitly).
  - Guard all geometric kernels against `NaN`, `Inf`, zero-length segments, degenerate zero-radius arcs, and collinear normals; return structured `Result<T, CamError>` diagnostics rather than panicking (`unwrap()`/`expect()` are forbidden in library crates outside tests).
  - Never compare floating-point geometry with bare `==`; use named epsilon constants from `chip-math`.
- **Deterministic Persistence:** Ensure `.chip.toml` and `.post.toml` serialization uses deterministic ordering (`BTreeMap` / sorted slices) so `git diff` stays clean.
- **Verification Checklist:**
  - `cargo fmt --all -- --check`
  - `cargo clippy --locked --workspace --all-targets -- -D warnings`
  - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
  - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
  - `cargo deny check` and `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
  - Every new or modified toolpath strategy must include integration tests verified against `chip-sim`'s heightmap oracle asserting **zero `G0` rapid-into-stock collisions** and **zero target B-Rep gouges** beyond tolerance.
  - When capturing headless GUI screenshots via `xvfb-run` on Linux, always pass `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a` so `winit` renders strictly inside Xvfb rather than opening visible windows on a host Wayland session.
