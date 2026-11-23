# Milestone 79 Implementation Plan — Simulation Mesh Skinning (Smooth Surface Display on CPU & GPU), Visual Verification, & Phase 21 Spec Sync

## Goal

Add an optional **Mesh Skinning** visual display mode for 3D stock simulation (controlled from **Settings → Simulation** added in Milestone 76) that renders curved, helical, sloped, and organic machined surfaces smoothly across **both CPU and GPU** `TriDexelStock`, `HeightfieldStock`, and `CylindricalStock` engines—while keeping 90° prismatic walls, pockets, and stock edges razor-sharp, preserving live toggle responsiveness without re-running simulation sweeps, and leaving volumetric metrology (`StockErrorReport`) and crash detection 100% invariant.

---

## Problem Diagnosis

1. **Flat Per-Quad Faceting in `TriDexelStock` (`crates/chip-stock/src/tridexel/mesher.rs`, `crates/chip-stock/src/gpu/tridexel_emit.wgsl`):**
   - Both CPU and GPU Surface Nets currently compute a single `crease_aware_smooth_normal` per quad face and write that same normal to all 4 vertices of the quad (`push_quad` on CPU; `corner0_nx / corner1_ny / corner2_nz` on GPU).
   - Dual vertices sit at unrelaxed local crossing averages, so diagonally curved walls, fillets, ball-end scallops, and helical lofts display visible voxel-grid micro-faceting and Mach bands even at `Standard` and `Fine` resolutions.
2. **Flat Stair-Stepped Top Quads in `HeightfieldStock` (`crates/chip-stock/src/heightfield/mesher.rs`, `crates/chip-gui/src/viewport/shaders.wgsl`):**
   - `HeightfieldStock` merges horizontal cell tops with fixed `[0, 0, 1]` normals and discrete step risers even on gentle 3D slopes, making 3D ball-end or sloped cuts look terraced compared to a continuous skinned surface.
3. **Uncreased Radial Averaging in `CylindricalStock` (`crates/chip-stock/src/cylindrical/mesher.rs`):**
   - `CylindricalStock` averages all adjacent triangle normals unconditionally (which can blur sharp turned shoulders) and does not relax raw radial grid noise along smooth curved helical lobes.
4. **No Visual Smoothing Option Wired Through Stock/Sim/GUI:**
   - Mesh extraction APIs only accept `SimStockColorMode`; there is no visual display toggle (`SimMeshOptions { color_mode, mesh_skinning }`) connecting `prefs.sim_mesh_skinning` to live CPU & GPU stock meshers.

---

## Proposed Changes

### 1. Visual-Only `SimMeshOptions` Plumbing (`chip-stock`, `chip-sim`, `chip-gui`)

#### [MODIFY] `crates/chip-stock/src/types.rs`
#### [MODIFY] `crates/chip-stock/src/backend.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/backend.rs`
#### [MODIFY] `crates/chip-stock/src/lib.rs`
#### [MODIFY] `crates/chip-sim/src/driver.rs`
#### [MODIFY] `crates/chip-sim/src/lib.rs`
#### [MODIFY] `crates/chip-gui/src/panels/simulation/{mod.rs, playback.rs, worker.rs}`
#### [MODIFY] `crates/chip-gui/src/panels/settings.rs`
- Define a small `Copy` visual options struct in `chip-stock`:
  ```rust
  #[derive(Debug, Clone, Copy, PartialEq, Eq)]
  pub struct SimMeshOptions {
      pub color_mode: SimStockColorMode,
      pub mesh_skinning: bool,
  }
  ```
- Thread `mesh_skinning: bool` (via `SimMeshOptions` or `_with_options` methods with backward-compatible wrappers keeping existing `extract_stock_mesh(color_mode)` signatures intact) through `StockBackend`, `SimulatedStock`, `SimulationResult`, and `SimulationPanelState`.
- **Live Toggle Without Re-Simulating:**
  - When the user toggles **Smooth Skinned Surfaces** in `Settings → Simulation`, `SimulationPanelState` updates `self.mesh_skinning`, persists `prefs.sim_mesh_skinning`, and immediately triggers a visual-only re-mesh (`< 2 ms` GPU compute re-emit / uniform update, or async CPU re-mesh on the existing `sim_worker` thread) without invalidating or re-running the simulation timeline!
- **Strict Metrology Isolation:**
  - Neither `TriDexelStock`, `HeightfieldStock`, `CylindricalStock` volumetric spans/heights, nor `StockErrorReport` / 5-rule crash verification ever reads `mesh_skinning`.

---

### 2. CPU `TriDexelStock` Crease-Preserving Mesh Skinning (`tridexel/mesher.rs`)

#### [MODIFY] `crates/chip-stock/src/tridexel/mesher.rs`
- Keep the existing crisp path intact when `mesh_skinning == false`.
- When `mesh_skinning == true`, run a two-stage feature-preserving skinning pass over the extracted Surface Nets quads before emitting `SimStockMesh`:
  1. **Crease-Preserving Tangential Taubin (`λ`/`μ`) Relaxation:**
     - Identify each unique dual vertex `(gx, gy, gz)` and collect its incident quad face normals + 1-ring surface neighbor dual vertices.
     - Classify a dual vertex as **pinned** if it lies on the outer stock bounding box (`x/y/z` at `0` or `dim`) or if any pair of its incident quad normals disagrees beyond the sharp-crease angle (`dot(n_a, n_b) < cos(38°) ≈ 0.788`), guaranteeing 90° pocket walls, step floors, slots, and stock edges stay geometrically crisp.
     - On unpinned smooth surface vertices, execute 2 volume-preserving Taubin iterations (`λ = 0.45`, `μ = -0.47`) projecting displacement tangentially/locally and clamping total offset from the original dual position to `<= 0.45 * pitch` (zero shrinkage, zero penetration through thin walls).
  2. **Per-Vertex Crease-Aware Area-Weighted Smooth Normals:**
     - Recompute incident quad area-weighted normals from the relaxed dual vertex positions.
     - For each quad corner `(v_id, quad_normal)`, sum area-weighted normals of all quads incident to `v_id` satisfying `dot(n_adj, quad_normal) >= 0.788`, normalize, and emit distinct per-corner normals `(n0, n1, n2, n3)` on each quad's 4 vertices.
     - Result: smooth continuous Gouraud/Phong shading across curved fillets, spheres, bosses, and helical lofts, with crisp split normals along sharp machined edges.

---

### 3. GPU `TriDexelStock` Mesh Skinning Parity (`tridexel_emit.wgsl`, `render_handle.rs`, `shaders.wgsl`)

#### [MODIFY] `crates/chip-stock/src/gpu/render_handle.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/tridexel.rs`
#### [MODIFY] `crates/chip-stock/src/gpu/tridexel_emit.wgsl`
#### [MODIFY] `crates/chip-gui/src/viewport/shaders.wgsl`
#### [MODIFY] `crates/chip-gui/src/viewport/sim_gpu_render.rs`
- **Zero-Readback Per-Corner Normal & Relaxation Encoding:**
  - Use `TriDexelEmitUniforms.mode_and_cap.z` to pass `mesh_skinning` (`0u` = Crisp, `1u` = Smooth Skinned).
  - Upgrade `PackedTriDexelQuad` from 24 bytes to 32-byte aligned layout (`TRIDEXEL_PACKED_QUAD_BYTES = 32`):
    - `ray_dw_flags: u32`, `color_rgba8: u32`
    - `corner0_pad: u32`, `corner1_pad: u32`, `corner2_pad: u32`, `corner3_pad: u32` (snorm8x4 corner dual offsets)
    - `n01_oct: u32` (`snorm8x2` octahedral-encoded unit normals for corners 0 & 1)
    - `n23_oct: u32` (`snorm8x2` octahedral-encoded unit normals for corners 2 & 3)
  - Keep bind group layout strictly unchanged (`1 uniform + 6 storage buffers <= 8` WebGPU default limit) and keep all WGSL helpers single-return for Windows D3DCompile/FXC compatibility.
- **Shader Evaluation (`tridexel_emit.wgsl` + `vs_sim_tridexel_gpu`):**
  - When `mesh_skinning == 0u`: skip relaxation and pack the existing single quad `n_final` into all 4 octahedral corner normal slots (matching existing crisp appearance).
  - When `mesh_skinning == 1u`:
    - Evaluate a deterministic **local crease-preserving dual-corner relaxation** per corner `(iu + ou, iv + ov, dw)` by averaging adjacent surface-cell dual positions that share a compatible smooth local normal (`dot >= 0.788`), pinning stock-boundary corners and sharp-crease corners, and clamping displacement to `<= 0.45 * pitch` (deterministic per shared dual corner so adjacent quads stay 100% watertight with zero cracks!).
    - Evaluate **per-corner crease-aware smooth normals** `(n0, n1, n2, n3)` at each relaxed corner `(p0, p1, p2, p3)` and octahedral-pack them into `n01_oct` and `n23_oct`.
  - Unpack the per-vertex octahedral normal corresponding to `corner_idx ∈ 0..4` in `vs_sim_tridexel_gpu` (`shaders.wgsl`), delivering silky smooth shading directly from `draw_indirect` with zero CPU readback.
- Update `compute_tridexel_max_quads` for the 32-byte quad record size so GPU buffer sizing stays safely within `max_storage_buffer_binding_size`.

---

### 4. `HeightfieldStock` (CPU & GPU) & `CylindricalStock` Mesh Skinning

#### [MODIFY] `crates/chip-stock/src/heightfield/mesher.rs`
#### [MODIFY] `crates/chip-stock/src/cylindrical/mesher.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/shaders.wgsl`
#### [MODIFY] `crates/chip-gui/src/viewport/sim_gpu_render.rs`
- **CPU `HeightfieldStock` (`heightfield/mesher.rs`):**
  - When `mesh_skinning == false`: keep existing crisp greedy X-merged quads + sharp vertical risers.
  - When `mesh_skinning == true`: detect continuous sloped surface cells (`|z_neighbor - z_cell| <= 1.5 * max(vdx, vdy)`), apply 2 passes of crease-preserving Taubin Z-smoothing across continuous neighbors (skipping cliff edges and flat uncut top boundaries), and emit per-vertex crease-aware central-difference smooth normals while keeping vertical step walls razor-sharp.
- **GPU `HeightfieldStock` (`vs_sim_heightfield_gpu` in `shaders.wgsl` + `SimStockVisualUniformsGpu`):**
  - Add a `mesh_skinning` flag bit in `SimStockVisualUniforms` (`sim_gpu_render.rs` / `shaders.wgsl`).
  - When enabled, evaluate crease-aware smooth central-difference normals (clamping neighbor samples across steep cliffs `> 1.5 * pitch` so vertical pocket walls remain sharp) directly in `vs_sim_heightfield_gpu`.
- **`CylindricalStock` (`cylindrical/mesher.rs`):**
  - When `mesh_skinning == true`: apply 2 passes of crease-preserving radial Taubin relaxation on smooth continuous bands (pinning end caps and steep radial shoulders `|Δr| > 1.5 * du`) and compute crease-aware per-corner normals so sharp turned shoulders stay crisp while helical/curved rotary surfaces render smoothly.

---

### 5. Documentation & Specification Synchronization

#### [MODIFY] `specs/06-simulation.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/10-roadmap.md`
- Document `SimMeshOptions` / Mesh Skinning (crease-preserving Taubin relaxation + per-vertex crease-aware normals + 32-byte octahedral `PackedTriDexelQuad` GPU parity + strict metrology invariance) in `specs/06-simulation.md`.
- Document the unified **Settings** page (`prefs.toml`), decluttered viewport HUD / simulation bar, active-setup card styling + top action bar operation creation with separators, single `"Tools"` entry point, `Project Tools` card controls, and machine-switch tool replacement dialog in `specs/08-ui-viewport.md`.
- Mark Phase 21 (Milestones 76–79) as **Completed** in `specs/10-roadmap.md` upon implementation completion.

---

## Verification Plan

1. **Mesh Skinning Geometry, Crease Preservation, & Metrology Invariance Tests (`chip-stock`, `chip-sim`):**
   - **Smoothness & Zero-Crack Watertightness:** Carved spherical dome / ball-nose fillet / helical loft in CPU & GPU `TriDexelStock`, `HeightfieldStock`, and `CylindricalStock`:
     - Verify `mesh_skinning = true` produces distinct interpolated per-corner normals across curved quads (non-flat shading) and strictly smaller normal discontinuity across adjacent curved quads than `mesh_skinning = false`.
     - Verify adjacent quads sharing a dual corner emit identical relaxed corner positions (zero T-junctions or hairline cracks).
     - Verify relaxed vertex displacement never exceeds `0.45 * pitch`.
   - **Sharp-Crease Preservation:** Rectangular pocket with 90° vertical walls and flat floor:
     - Verify wall quads remain orthogonal (`|normal.z| < 0.05`) and floor quads remain flat (`normal.z > 0.99`) with `mesh_skinning = true`.
   - **CPU vs. GPU Parity & Live Toggle:**
     - Verify GPU `TriDexel` octahedral normal pack/unpack round-trips within `< 0.5°` across unit-sphere directions and produces matching smooth-surface shading to CPU `TriDexel`.
     - Verify toggling `mesh_skinning` updates the rendered mesh without re-running `simulate_setup`.
   - **Strict Metrology & Golden G-Code Invariance:**
     - Assert `StockErrorReport` (`excess_count`, `gouge_count`, `max_excess_mm`, `max_gouge_mm`) is bit-identical between `mesh_skinning = false` and `mesh_skinning = true`.
     - Confirm 100% byte-for-byte parity on all Phase 1–5 golden G-code snapshots.
2. **Headless Vulkan Visual Verification:**
   - Render side-by-side headless Vulkan screenshots of a curved/helical + prismatic benchmark part under `Crisp` (`mesh_skinning = false`) vs. `Smooth Skinned` (`mesh_skinning = true`) on both CPU and GPU Tri-Dexel backends and visually inspect them to confirm smooth curved skins and razor-sharp 90° edges.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
