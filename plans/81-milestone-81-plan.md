# Milestone 81 Implementation Plan — 4th-Axis Rotary Axial Boundary Unification (`-U`/`+U` Bounds, Containment, Radial Ceiling, & Wrapper Chuck Safety)

## Goal

Give both native 4th-axis rotary strategies (**`Rotary2Linear`** and **`Rotary4AxisSim`**) complete, consistent `-U`/`+U` axial cutting-area controls matching the ergonomics of 3-axis machining boundaries (`StockBounds`, `ModelBounds`, `SelectedGeometry` projected onto `-U..+U`, or `ExplicitRange`), axial `ToolContainmentMode` (`Inside` / `Center` / `Outside`), signed start/end offsets (allowing shoulder overrun while strictly enforcing the rotary chuck keep-out guard), and outer radial ceiling enforcement via `heights.top_z_wcs`—plus chuck keep-out and optional axial/angular limits on the `WrapCylinder` and `Indexed3Plus1` rotary wrappers.

---

## Problem Diagnosis

1. **Inward-Only Model-Clamped Axial Margins (`crates/chip-cam/src/rotary.rs:802-845`):**
   - `resolve_rotary_axial_window()` currently computes `u_start = stock_u_min + margin.max(0.0)`, `u_end = stock_u_max - margin.max(0.0)`, and then forcibly clamps inward to `[mesh_u_min, mesh_u_max]`.
   - Users cannot choose **Stock Bounds** vs. **Model Bounds** vs. **Selected Faces/Loops (`-U..+U` span)** vs. **Explicit `-U`/`+U` Coordinates**.
   - Users cannot specify a negative/outward offset to overrun past a shoulder or model end (`.max(0.0)` silently strips negative values!), nor choose whether the cutter center, inside flank, or outside flank aligns to `-U`/`+U`.
2. **Ignored `heights.top_z_wcs` (No Outer Radial Ceiling):**
   - Both `Rotary2Linear` (`strategies/rotary2linear.rs:55-95`) and `Rotary4AxisSim` (`strategies/rotary4axis_sim.rs:68-107`) honor `heights.bottom_z_wcs` as `r_floor`, but completely ignore `heights.top_z_wcs`—always starting roughing passes at `rot.stock_outer_radius_mm`, wasting cycle time air-cutting stepped or pre-turned shafts.
3. **Unprotected Rotary Wrappers (`WrapCylinder` & `Indexed3Plus1`):**
   - `OperationRotaryMode::WrapCylinder` (`strategies/rotary_wrap.rs`) performs zero axial/angular bounds checking and **does not enforce the rotary chuck keep-out guard** (wrapped paths can collide with the chuck or overlap beyond `360°`).
   - `OperationRotaryMode::Indexed3Plus1` (`strategies/mod.rs:230-277`) does not enforce automatic chuck axial keep-out when running 3-axis/2.5D strategies indexed.

---

## Proposed Changes

### 1. Shared `RotaryAxialBoundaryConfig` Schema (Backward Compatible)

#### [MODIFY] `crates/chip-cam/src/params/rotary.rs`
#### [MODIFY] `crates/chip-cam/src/params/mod.rs`
#### [MODIFY] `crates/chip-cam/src/operation.rs`
- Define the axial boundary source enum strictly targeting the rotary axis ends (`-U`, `+U`):
  ```rust
  #[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
  #[serde(rename_all = "snake_case")]
  pub enum RotaryAxialBoundaryMode {
      /// Full setup cylindrical stock axial span `[stock_u_min, stock_u_max]`.
      StockBounds,
      /// Active 3D model axial span `[mesh_u_min, mesh_u_max]` (clamped to stock).
      #[default]
      ModelBounds,
      /// Axial `-U..+U` span projected from selected 3D faces, loops, or holes.
      SelectedGeometry,
      /// Explicit user-entered `[explicit_u_start, explicit_u_end]` coordinates along the rotary axis.
      ExplicitRange,
  }
  ```
- Define shared `RotaryAxialBoundaryConfig` (embedded via `#[serde(flatten)]` in both `Rotary2LinearParams` and `Rotary4AxisSimParams`):
  - `#[serde(default)] pub axial_boundary_mode: RotaryAxialBoundaryMode` (defaults to `ModelBounds` for exact parity with legacy files).
  - `#[serde(default)] pub axial_containment: ToolContainmentMode` (`ToolCenterOnBoundary` default, or `ToolInsideBoundary` / `ToolOutsideBoundary` applied to `-U` and `+U` ends!).
  - Keep existing serialized field names `axial_margin_start: ParamLength` and `axial_margin_end: ParamLength` — now **signed** inward margins (`> 0` trims inward toward part center, `< 0` extends outward past the `-U`/`+U` boundary for clean shoulder/end overrun, always bounded by the hard chuck guard!).
  - `#[serde(default)] pub explicit_u_start: ParamLength` and `#[serde(default)] pub explicit_u_end: ParamLength` (used when `axial_boundary_mode == ExplicitRange`).
  - `start_angle_deg: f64` and `end_angle_deg: f64` (kept in place with `0.0..360.0` defaults).
- Add `Operation::rotary_axial_boundary(&self)` / `Operation::rotary_axial_boundary_mut(&mut self)` accessors on `Operation` for M83's unified `[Geometry]` tab and viewport overlay.

---

### 2. Upgraded `resolve_rotary_axial_window()` & Shared Resolved Window Struct

#### [MODIFY] `crates/chip-cam/src/rotary.rs`
- Introduce `ResolvedRotaryWindow` shared by toolpath kernels AND M83's live 3D viewport preview:
  ```rust
  pub struct ResolvedRotaryWindow {
      /// Nominal axial bounds before tool-containment & margin shifts.
      pub nominal_u_min: f64,
      pub nominal_u_max: f64,
      /// Final chuck-safe tool-center axial span `[u_start, u_end]`.
      pub u_start: f64,
      pub u_end: f64,
      /// Effective angular span `[start_angle_deg, end_angle_deg]`.
      pub start_angle_deg: f64,
      pub end_angle_deg: f64,
      /// Effective radial envelope `[r_floor, r_ceiling]`.
      pub r_floor: f64,
      pub r_ceiling: f64,
      /// Hard chuck keep-out boundary plane coordinate along rotary axis.
      pub chuck_guard_u: f64,
      /// Whether the chuck keep-out guard actively clamped `u_start` or `u_end`.
      pub clamped_by_chuck: bool,
      pub warnings: Vec<String>,
  }
  ```
- Implement deterministic resolution rules for each `RotaryAxialBoundaryMode`:
  1. **`StockBounds`:** `nominal = [rot.stock_u_min, rot.stock_u_max]`.
  2. **`ModelBounds`:** `nominal = [mesh_u_min.max(rot.stock_u_min), mesh_u_max.min(rot.stock_u_max)]` (falling back to stock bounds if mesh empty).
  3. **`SelectedGeometry`:** Project all vertices of `selection` (selected 3D `PlanarFace`, `ClosedLoop`, `OpenChain`, `CylindricalHole`, or raw CAD face vertices from `cad_doc` / `surface`) onto `u_vec` to obtain `[sel_u_min, sel_u_max]`. Emit a structured warning and fall back to `ModelBounds` if selections are empty.
  4. **`ExplicitRange`:** `nominal = [explicit_u_start.min(explicit_u_end), explicit_u_start.max(explicit_u_end)]`.
- Apply `axial_containment` shift to both ends:
  - `ToolInsideBoundary`: `u_start += tool_r`, `u_end -= tool_r` (flank stays inside `[nominal_u_min, nominal_u_max]`).
  - `ToolCenterOnBoundary`: `±0.0` (tool centerline reaches the boundary ends).
  - `ToolOutsideBoundary`: `u_start -= tool_r`, `u_end += tool_r` (cutter clears fully past both ends).
- Apply signed margins: `u_start += axial_margin_start_mm`, `u_end -= axial_margin_end_mm` (positive trims inward, negative extends outward).
- Enforce the **mandatory hard chuck keep-out guard** (`tool_r + 0.5 mm` from `rot.chuck_u_min` / `rot.chuck_u_max`) *after* containment and margins so no combination of `StockBounds`, `ToolOutsideBoundary`, or negative margin can ever drive the cutter into the rotary chuck! Record `clamped_by_chuck = true` and emit an informative warning when clamping occurs.

---

### 3. Honor Outer Radial Ceiling (`heights.top_z_wcs`) & Harden Rotary Wrappers

#### [MODIFY] `crates/chip-cam/src/strategies/rotary2linear.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary4axis_sim.rs`
- Switch both strategies to `resolve_rotary_window(...)`, forwarding any warnings into `ToolpathBuilder`.
- Compute `r_floor = (ctx.heights.bottom_z_wcs - rot.center_wcs.z).max(0.0)` and `r_ceiling`:
  - When `heights.top` is explicitly lowered below `rot.center_wcs.z + rot.stock_outer_radius_mm`, set `r_ceiling = (ctx.heights.top_z_wcs - rot.center_wcs.z).clamp(r_floor, rot.stock_outer_radius_mm.max(r_max_surf))` so multi-depth radial roughing starts at `r_ceiling` instead of air-cutting from full stock OD!
  - Preserve full stock OD when `heights.top` is at or above stock top (`StockTop` default) so existing projects behave identically.

#### [MODIFY] `crates/chip-cam/src/rotary.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/rotary_wrap.rs`
#### [MODIFY] `crates/chip-cam/src/strategies/mod.rs`
- **`WrapCylinder`:** Enforce the mandatory rotary chuck keep-out guard on wrapped `u` coordinates (returning a structured `CamError::InvalidRotarySetup` or warning + safe validation diagnostic if wrapped geometry intrudes into the chuck keep-out zone, plus optional `clamp_to_stock_axial: bool` / angular wrap-overlap warning if swept span exceeds `360°`).
- **`Indexed3Plus1`:** Pass the rotary chuck keep-out `u` limit into the rotated virtual setup's safety check so indexed 2.5D/3-axis operations cannot plunge into the chuck jaws.

#### [MODIFY] `crates/chip-project/src/validation.rs`
- Warn if `SelectedGeometry` is chosen on a rotary op with zero selections, if `ExplicitRange` has `u_start >= u_end`, or if a `WrapCylinder` operation intrudes into the chuck keep-out zone.

---

## Verification Plan

1. **Axial Window Resolution Unit Tests (`chip-cam`):**
   - Test all 4 modes (`StockBounds`, `ModelBounds`, `SelectedGeometry`, `ExplicitRange`) on both `+X`/`-X` and `+Y`/`-Y` rotary axes.
   - Verify `ToolInsideBoundary`, `ToolCenterOnBoundary`, and `ToolOutsideBoundary` shift `[u_start, u_end]` by exact `±tool_r`.
   - Verify negative `axial_margin_start/end` extends past model bounds for shoulder overrun while hard-clamping cleanly at `chuck_u_max + tool_r + 0.5 mm` (`clamped_by_chuck == true`).
2. **Radial Envelope & Wrapper Safety Tests (`chip-cam`, `chip-sim`):**
   - Stepped-shaft test: lowering `heights.top` reduces roughing passes cleanly without gouging, verified via `CylindricalStock` and `TriDexelStock` simulation (`0` crashes, `0` gouges).
   - `WrapCylinder` & `Indexed3Plus1` chuck-intrusion test: verify chuck keep-out violation is caught deterministically.
   - Verify legacy rotary `.chip.toml` files (including `rotary-shaft` and `rotary-4axis` built-in samples & golden G-code tests) maintain 100% backward compatibility.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
