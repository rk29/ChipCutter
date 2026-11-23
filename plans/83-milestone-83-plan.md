# Milestone 83 Implementation Plan — Unified `[Geometry]` Tab Boundary Controls, Live 3D Viewport Previews, & Phase 22 Spec Sync

## Goal

Consolidate all cutting-area and machining-boundary controls into the Operation Inspector **`[Geometry]`** tab so machinists configure *what to cut* in one consistent place across 2.5D, 3-axis, undercut/deburr, and 4th-axis rotary operations—backed by live **3D viewport cutting-area overlays** (nominal + effective tool-center boundary loops for 2.5D/3-axis, and cylindrical `-U..+U` axial sector bands + chuck keep-out plane for 4th-axis rotary) and full Phase 22 specification synchronization.

---

## Problem Diagnosis

1. **Split Mental Model Between `[Geometry]` and `[Passes]`:**
   - Today, users select 2D regions/faces on `[Geometry]`, then have to switch to `[Passes]` and scroll to the bottom to find `3D Machining Boundary` (`passes_tab/surface_3d.rs:517-576`) or hunt mid-card for rotary `-U`/`+U` axial margins and angular sector bounds (`passes_tab/surface_3d.rs:640-675, 780-815`).
2. **Misleading Rotary `[Geometry]` Banner (`crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs:62-69`):**
   - `Rotary2Linear` and `Rotary4AxisSim` fall into `op.kind.is_3d_surface()` and display a generic 3D banner telling users to pick 2D contours and choose `'Selected 2D Contours'` in Passes.
3. **Zero Visual Feedback for Cutting Areas in the 3D Viewport (`crates/chip-gui/src/app/viewport_controller.rs:127-170`):**
   - Only Z-height planes preview in the 3D viewport (when `[Heights]` is active). Users have zero visual preview of how `StockBox` vs. `ModelSilhouette` vs. `SelectedRegions`, `ToolContainmentMode`, `boundary_offset`, or rotary `-U`/`+U` bounds + chuck keep-out actually constrain their cut until after generating the toolpath!

---

## Proposed Changes

### 1. Unified Cutting Area Section in `[Geometry]` Tab

#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/mod.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/geometry_tab.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/surface_3d.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/milling_2d.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/deburr3d_card.rs`
#### [MODIFY] `crates/chip-gui/src/panels/operation_inspector/passes_tab/undercut_cards.rs`
- Extend `GeometryTabCtx` with the resolved `Setup` / `ResolvedSetup` & active `Tool` diameter (read-only) so `[Geometry]` can compute and display live resolved cutting-area dimensions and warnings inline.
- Place a consistent **`Cutting Area / Machining Boundary`** `section_card` at the top of `[Geometry]` for every strategy that supports boundary controls, cleanly tailored by category:
  1. **3-Axis (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`), `Undercut3D`, `Deburr3D` (when Auto-Detect), & `Facing`:**
     - **Boundary Source:** `Stock Bounds` | `Model Silhouette` | `Selected Regions`
     - **Tool Containment:** `Tool Inside Boundary` | `Tool Center on Boundary` | `Tool Outside Boundary`
     - **Boundary Offset:** signed `ParamLength` expression input (`+` expands, `-` insets)
     - Contextual inline status: when `Selected Regions` is active and `op.selections.is_empty()`, show a clear warning banner prompting the user to pick regions below; when `Stock Bounds` or `Model Silhouette` is active on a 3D surface op, collapse the 3D pick card by default so the inspector stays clean and focused.
  2. **4th-Axis Rotary (`Rotary2Linear`, `Rotary4AxisSim`):**
     - Replace the misleading generic 3D banner with a dedicated **4th-Axis Rotary Cutting Area (`-U / +U`)** card:
       - **Axial Boundary Source:** `Stock Bounds` | `Model Bounds` | `Selected Geometry (-U..+U)` | `Explicit Range`
       - **Axial Tool Containment:** `Tool Inside` | `Tool Center` | `Tool Outside`
       - **Start / End Offsets (`-U` / `+U`):** signed `ParamLength` inputs (tooltip clearly noting positive trims inward, negative extends outward for shoulder relief while always respecting chuck keep-out)
       - When `Explicit Range`: show `Start U` / `End U` expression inputs.
       - **Angular Window:** `Start Angle (°)` & `End Angle (°)` with quick preset chips (`Full 360°`, `0–180°`, `±90°`).
       - Live resolved readout line: e.g., `Effective U: [12.50, 84.00] mm · Chuck Guard: 10.50 mm` (highlighted with a warning pill if clamped by the rotary chuck!).
  3. **`Pocket2D` / `Adaptive2D`:**
     - **Pocket Area Mode:** `Inside Selected Pockets` | `Clear Outside Selected Bosses to Stock`
     - **Open Outer Boundary** checkbox (when 2D Sketch / `ClosedLoop` selections are present).
  4. **`Contour2D`:**
     - Optional `Clip to Stock Bounds` toggle and `Open Chain Extension` input.
- Remove the old duplicated `show_3d_boundary_card` and rotary axial/angular fields from `[Passes]` (`passes_tab/surface_3d.rs`) so there is a single authoritative home for cutting-area controls on `[Geometry]`.

---

### 2. Live 3D Viewport Cutting-Area Overlays

#### [MODIFY] `crates/chip-gui/src/viewport/mod.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/scene.rs`
#### [MODIFY] `crates/chip-gui/src/viewport/overlays.rs`
#### [MODIFY] `crates/chip-gui/src/app/viewport_controller.rs`
- Add a lightweight `BoundaryPreviewOverlay` computed from the active operation's resolved boundary (cached and invalidated only when the active operation's selections, boundary config, setup stock, or tool diameter changes — **zero per-frame mesh projection or allocations!**):
  - **2.5D & 3-Axis Preview (visible when `OperationTab::Geometry` is active):**
    - Render the **nominal boundary polygon** (`ResolvedBoundary2D::nominal_region`) as a crisp accent-colored 3D wireframe loop at `heights.top_z_wcs` (with subtle vertical drop-lines to `heights.bottom_z_wcs` at bounding corners or translucent top-plane contour).
    - Render the **effective tool-center containment boundary** (`ResolvedBoundary2D::effective_region`) as a distinct dashed/secondary-color loop whenever `effective != nominal` (immediately showing the visual effect of `Tool Inside` vs `Tool Center` vs `Tool Outside` and `Boundary Offset`!).
  - **4th-Axis Rotary Preview (visible when `OperationTab::Geometry` is active on `Rotary2Linear` or `Rotary4AxisSim`):**
    - Render the **resolved axial + angular cutting sector** (`[u_start, u_end] × [start_angle_deg, end_angle_deg]`) as a translucent cylindrical shell/wireframe sleeve at `r_ceiling` with crisp `-U` and `+U` endcap rings (and inner `r_floor` reference rings when `r_floor > 0`).
    - Highlight the **rotary chuck keep-out plane** (`chuck_guard_u`) with a distinct amber/red warning ring whenever the boundary approaches or is clamped by the chuck!
- Ensure overlay rendering is toggleable/respects viewport visibility cleanly and stays legible in both `Studio Dark` and `Daylight Light` themes.

---

### 3. Specification Synchronization & Visual Verification

#### [MODIFY] `specs/05-operations.md`
#### [MODIFY] `specs/08-ui-viewport.md`
#### [MODIFY] `specs/10-roadmap.md`
- Document `MachiningBoundaryConfig`, true `ModelSilhouette` projection, `RotaryAxialBoundaryConfig`, `Facing` exact polygon boundaries, `Pocket2D` `ClearOutsideSelectedToStock`, and `WrapCylinder`/`Indexed3Plus1` chuck safety in `specs/05-operations.md`.
- Document the consolidated `[Geometry]` cutting-area card and live 3D viewport boundary/rotary-sector overlays in `specs/08-ui-viewport.md`.
- Mark **Phase 22 (M80–M83)** completed in `specs/10-roadmap.md` once implementation lands.

---

## Verification Plan

1. **GUI Inspector & Overlay Unit Tests (`chip-gui`):**
   - Headless `egui` rendering tests verifying `[Geometry]` displays the Cutting Area card for `Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`, `Pencil3D`, `Undercut3D`, `Deburr3D`, `Facing`, `Pocket2D`, `Rotary2Linear`, and `Rotary4AxisSim`, and `[Passes]` no longer duplicates boundary controls.
   - Verify changing `boundary_mode`, `tool_containment`, `boundary_offset`, or `axial_boundary_mode` marks the operation `Dirty`, records an Undo step, and refreshes the cached `BoundaryPreviewOverlay` without per-frame recomputation.
2. **Headless Vulkan Visual Verification:**
   - Capture headless screenshots (`WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan xvfb-run -a`) showing:
     1. A 3-axis curved model (`mesh-dome`) on `[Geometry]` with true circular `ModelSilhouette` + `Tool Outside` dashed effective loop overlay.
     2. A 4th-axis rotary project (`rotary-4axis` or `rotary-shaft`) on `[Geometry]` with the `-U..+U` cylindrical sector sleeve, endcap rings, and chuck keep-out indicator.
3. **Full Workspace Verification Gate:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --locked --workspace --all-targets -- -D warnings`
   - `cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings`
   - `cargo deny check` & `cargo deny --no-default-features --features chip-gui/flatpak-portal check`
   - `WGPU_BACKEND=vulkan cargo test --locked --workspace`
