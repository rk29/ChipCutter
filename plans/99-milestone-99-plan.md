# Milestone 99 Implementation Plan — Focused Crate Extractions (`chip-cl`, `chip-fixtures`, `chip-demos`, `chip-machines`) & Presentation Separation

## Goal

Introduce four focused, cycle-free workspace crates (`chip-cl`, `chip-fixtures`, `chip-demos`, `chip-machines`) that sever heavyweight transitive dependencies (`chip-post → chip-cam`, `chip-cad → monstertruck-modeling`, `chip-stock → dirs/zip/serde_json/machine catalogs`), unify duplicated CLI/GUI sample catalogs and user-config preset CRUD, and move viewport-only GPU stock/cutter presentation cleanly into `chip-gui`.

---

## Proposed Changes

### 1. New Crate `crates/chip-cl` — Severing `chip-post → chip-cam`
- Create `crates/chip-cl` (dependencies: `chip-math`, `chip-tools`, `serde` ONLY):
  - Move `crates/chip-cam/src/cl_data.rs`, `crates/chip-cam/src/toolpath.rs` (+ `rebuild_segments_from_events` from `feed_opt.rs`), `WorkOffsetSlot` (`wcs.rs`), and `RotaryUnwindMode` (`rotary.rs`) into `chip-cl` (re-export from `chip-cam` for compatibility).
- Move `resolve_rotary_target_deg`, `compute_inverse_time_feed_g93`, and `compute_rotary_angular_feed_deg_min` from `chip-cam/src/rotary.rs` → `crates/chip-post/src/rotary.rs` (zero production callers inside `chip-cam`).
- Move `describe_wcs_origin(&WcsConfig)` from `chip-post/src/partition.rs` → `WcsConfig::describe_origin` in `chip-cam/src/wcs.rs`.
- Replace `OperationPostInput.op: &Operation` with `op_name: &'a str` and replace `ResolvedStockGeometry` / `ResolvedRotaryConfig` on `SetupPostInput` with lightweight `PostStockSummary` / `PostRotarySetup` structs.
- **Remove `chip-cam` from `crates/chip-post/Cargo.toml`**, replacing it with `chip-cl`.

### 2. New Crates `crates/chip-fixtures` (Layer 1) & `crates/chip-demos` (Layer 5.5)
- Create `crates/chip-fixtures` (dependencies: `chip-math`, `monstertruck-modeling`, `zip`, `thiserror` — zero `chip-cad`/`chip-sketch` dependency so all core crates can `[dev-dependencies]` on it cleanly):
  - Move `chip-cad/src/fixtures.rs` (26 STEP builders + `sample_svg_badge_bytes`, keeping `chip-cad` topology tests in `chip-cad`), `chip-cad/src/mesh/fixtures.rs` (8 STL/OBJ/3MF builders), `chip-sketch::sample_{router_panel_dxf, badge_logo_svg}`, and `chip-project::sample_two_sided_vise_part_stl_bytes` into `chip-fixtures/{step_*, mesh, vector}.rs`.
  - **Remove `monstertruck-modeling` from `crates/chip-cad/Cargo.toml`!**
- Create `crates/chip-demos` (dependencies: `chip-project`, `chip-fixtures`, `chip-cam`, `chip-cad`, `chip-sketch`, `chip-tools`, `chip-machines`, `chip-math`):
  - Move `crates/chip-project/src/demos/` (6.8k LOC, 45.5% of `chip-project/src`) into `crates/chip-demos`.
  - Add a declarative `SampleCatalog` / `SampleId` registry unifying raw asset bytes, metadata, and project builders, eliminating ~1,400 LOC of parallel sample dispatch in `chip-cli/src/commands/generate_sample.rs` and `chip-gui/src/app/loader/{sample_loaders.rs, cad_apply.rs}`.

### 3. New Crate `crates/chip-machines` & Shared `PresetStore`
- Create `crates/chip-machines` (dependencies: `chip-math`, `chip-tools`, `dirs`, `serde`, `toml`, `thiserror`):
  - Move `machine.rs` and `builtin_machines/` (7 built-in machine profiles) from `chip-tools` into `chip-machines` (optionally loading `machines/*.machine.toml` via `include_str!` or keeping canonical built-in builders, validated by existing parity tests).
  - Add a shared `PresetStore` / config-dir utility (used by both `chip-machines` and `chip-post/src/profile.rs`), eliminating ~300 LOC of duplicated user-config TOML CRUD, unifying ID validation/sanitization rules, and replacing repeated full-TOML parsing / struct reconstruction inside `is_builtin_post_id` & `is_builtin_machine_id` with constant ID tables.
  - Feature-gate `fusion_import` (`serde_json`, `zip`) and global config-dir helpers (`dirs`) in `chip-tools` (enabled by `chip-cli`/`chip-gui`/`chip-project`) so `chip-stock` and `chip-post` never pull `zip`, `serde_json`, or `dirs`.

### 4. Presentation vs. Compute Separation (`chip-stock`, `chip-sim`, `chip-gui`)
- Move `crates/chip-sim/src/cutter_profile.rs` (`SimToolMesh`, `build_cutter_assembly_mesh` — 100% viewport presentation) to `crates/chip-gui/src/viewport/tool_mesh.rs`.
- Move `crates/chip-stock/src/gpu/render_handle.rs` + `tridexel_emit.wgsl` + `TriDexelEmitResources` + GUI combobox/tooltip descriptions (`chip-stock/src/types.rs:309–449`) into `crates/chip-gui/src/viewport/sim_gpu_render/`, exposing a minimal raw read-only `GpuStockBufferSnapshot` from `chip-stock::gpu` so `tridexel_emit.wgsl` and `shaders.wgsl` live side-by-side in one crate with a single shared uniform/quad ABI.
- Move `SimStockColorMode` and `deviation_to_heatmap_rgba` out of `chip-stock/src/metrics.rs` into `chip-stock/src/visual.rs` (formerly `mesher_common.rs`) so `metrics.rs` is purely quantitative metrology.
- Trim the 53 unused / test-only `chip-stock` re-exports from `crates/chip-sim/src/lib.rs` and add `chip-stock` directly to `crates/chip-gui/Cargo.toml`.

---

## Verification Plan
- `cargo fmt --all -- --check`
- `cargo clippy --locked --workspace --all-targets -- -D warnings`
- `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
- `cargo deny check && cargo deny --no-default-features --features chip-gui/flatpak-portal check`
- `WGPU_BACKEND=vulkan cargo test --locked --workspace`
- Verify `cargo tree -p chip-post` has zero `chip-cam` / `monstertruck-*` nodes and `cargo tree -p chip-cad` has zero `monstertruck-modeling` nodes.
