# Milestone 53 Implementation Plan — GPU Flat Fixed-`K` Tri-Dexel Compute Sweeper, GPU BVH Distance, & GPU Error Metrics

## Goal

Extend `WgpuComputeBackend` to cover full 3D volumetric **Tri-Dexel** simulation (`tridexel_sweep.wgsl`) over Phase 14's axis-generic flat fixed-`K` (`K = 4`) `FlatDexelSpan` pools—with a lossless hybrid CPU overflow safety valve for rare `> K` multi-undercut rays—plus **GPU BVH target rasterization/distance evaluation** (`bvh_distance.wgsl`) and **GPU parallel reductions** (`stock_metrics.wgsl`) computing `StockErrorReport` metrics in `< 1 ms`.

---

## Proposed Changes

### 1. Axis-Generic Flat Fixed-`K` Tri-Dexel WGSL Compute Sweeper

#### [NEW] `crates/chip-stock/src/gpu/tridexel_sweep.wgsl`
#### [NEW] `crates/chip-stock/src/gpu/tridexel.rs`
- **Direct 1:1 Buffer Layout:** Upload each `DexelPlane` (`Axis3::X`, `Axis3::Y`, `Axis3::Z`) directly from its existing Phase 14 flat buffers:
  - `slots: array<FlatDexelSpan>` (`nu * nv * 4`, 16 bytes each: `min_local`, `max_local`, `op_packed`, `_pad`)
  - `counts: array<u32>` (`nu * nv`)
  - Target solid buffers (`target_slots`, `target_counts`) for Rules 1–4 target-gouge verification.
- **Single Axis-Parameterized Shader (`Axis3` Uniform):**
  - Parameterize the WGSL sweeper by `axis_id` (`0 = X`, `1 = Y`, `2 = Z`) using the exact same `(u, v, w)` coordinate permutation as `tridexel/sweep.rs` so there is zero X/Y/Z shader duplication!
  - Use `16×16` `(u, v)` ray tiles with CPU coarse tile-binning over each batch's chords.
  - For each ray `(iu, iv)` and intersecting chord, compute the exact swept cutter interval `[w_lo, w_hi]` along axis `w` (covering all 7 cutter geometries including non-monotonic single-point & multi-form `ThreadMill` teeth!), perform inline span subtraction in registers (`var local_spans: array<FlatDexelSpan, 4>`), and update `slots` / `counts`.
- **Lossless Hybrid Overflow Safety Valve:**
  - Allocate a compact `overflow_bits: array<atomic<u32>>` bitset (`ceil(nu * nv / 32)` `u32`s per plane, ~10 KB at Ultra `448³`) plus a single `any_overflow: atomic<u32>` summary flag.
  - Whenever subtracting an interval would split a ray beyond `K = 4` spans (e.g., 6-tooth multi-form internal thread mill or stacked cross-holes), the shader sets the ray's overflow bit and leaves the ray flagged for exact CPU resolution.
  - On the rare batch where `any_overflow != 0` (0% on typical 2.5D/3D parts; `< 0.1%` of rays on multi-form threads), `WgpuComputeBackend` reads back only the flagged ray indices and replays that batch's intersecting chords on those specific rays in the exact CPU `BTreeMap<u32, Vec<DexelSpan>>` overflow side-table—guaranteeing **100% exact multi-span undercuts with zero silent span loss**!
- **Atomic Rules 1–4 Verification:**
  - Evaluate rapid-into-stock overlap, target-Tri-Dexel gouge depth, flute-length reach, and holder collision directly on the GPU per batch and report via `GpuViolationAccum`.

---

### 2. GPU Target-Mesh BVH Upload & Parallel Reduction Metrology

#### [NEW] `crates/chip-stock/src/gpu/bvh_distance.wgsl`
#### [NEW] `crates/chip-stock/src/gpu/stock_metrics.wgsl`
#### [NEW] `crates/chip-stock/src/gpu/metrics.rs`
- **Direct `chip-bvh` GPU Upload (`bvh_distance.wgsl`):**
  - Upload `TriMesh3D::bvh_nodes()` (`FlatBvhNode`, 32B `Pod`) and `TriMesh3D::gpu_triangles()` (`GpuTriangle`, 48B `Pod`) directly to GPU storage buffers with zero CPU conversion!
  - Implement stack-based iterative BVH raycast (`raycast_all_hits` even-odd parity for Tri-Dexel target voxelization; inward radial raycast for Cylindrical `r_target`; top-down raycast for Heightfield `z_target`) and exact point-to-mesh closest-distance queries on GPU—accelerating Ultra `448³` target solid construction from seconds to milliseconds!
- **GPU Parallel Reduction for Volume & `StockErrorReport` (`stock_metrics.wgsl`):**
  - Two-stage workgroup shared-memory reduction (`256` threads/workgroup -> small partial-sum buffer read back to CPU) across `Heightfield`, `Cylindrical`, and `TriDexel` buffers computing:
    - Remaining stock volume, excess volume, gouge volume
    - Sum of squared errors (`RMS`), sum of absolute errors (`MAE`), signed sum (`Bias`)
    - Max excess & max gouge magnitude + exact 3D WCS coordinates
    - Fixed-bin logarithmic error histogram (64–128 bins covering `0.001 mm .. 50 mm`) for instant `in_tolerance_surface_pct(tol_mm)`, `P95`, and `P99` computation across any tolerance threshold without scanning millions of cells on CPU!
- **Wire Tri-Dexel into `WgpuComputeBackend`:**
  - All three engines (`Heightfield25D`, `TriDexel3D`, `Cylindrical4Axis`) now execute end-to-end on `WgpuComputeBackend` through the single unified `SimStockBackend` driver!

---

## Verification Plan

1. **Tri-Dexel CPU-vs-GPU Differential Oracle Tests (`chip-stock`, `chip-sim`):**
   - Box & cylinder volume accuracy across all 3 axes on GPU.
   - Multi-strategy parity: `cam-torture` (2.5D prismatic), `v3_mesh_dome_3axis` (3-axis sculpted mesh), and `thread-mill` (internal & external helical undercuts) comparing GPU vs. CPU volumes (`< 0.1%`), surface error metrics (`RMS` / `MAE` within `< 0.005 mm`), and identical Rule 1–5 crash/gouge detection.
   - **Overflow Safety-Valve Stress Test:** Carve 6 stacked undercut grooves (7 spans per ray, `> K = 4`) through `WgpuComputeBackend`, asserting `overflow_ray_count > 0`, zero lost spans, and exact volume parity against `CpuExactBackend`.
   - **Multi-Body Mesh `target_bodies` Parity:** Verify GPU BVH target rasterization matches CPU `extract_tridexel_target_from_cad` / `rasterize_target_mesh` on multi-body STL meshes.
2. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
