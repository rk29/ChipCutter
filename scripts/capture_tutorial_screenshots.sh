#!/usr/bin/env bash
# Repeatable headless tutorial screenshot capture pipeline for ChipCutter Phases 4 & 6.
#
# Usage:
#   ./scripts/capture_tutorial_screenshots.sh [--milestone 13|14|15|22|23|24|47|63|69|70|all] [--tutorial <slug>] [--theme dark|light] [--debug]
#
# Renders offscreen inside Xvfb using Vulkan Lavapipe (never opens windows on a host Wayland session).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MILESTONE="13"
TUTORIAL=""
THEME="dark"
PROFILE="release"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --milestone)
      MILESTONE="${2:-}"
      shift 2
      ;;
    --tutorial)
      TUTORIAL="${2:-}"
      shift 2
      ;;
    --theme)
      THEME="${2:-dark}"
      shift 2
      ;;
    --debug)
      PROFILE="debug"
      shift
      ;;
    --release)
      PROFILE="release"
      shift
      ;;
    -h|--help)
      sed -n '2,8p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

TARGET_DIR="${CARGO_TARGET_DIR:-$REPO_ROOT/target}"
echo "[capture] Building chip-gui and chip-cli ($PROFILE)..."
if [[ "$PROFILE" == "release" ]]; then
  cargo build --manifest-path "$REPO_ROOT/Cargo.toml" --release -p chip-gui -p chip-cli
  BIN="$TARGET_DIR/release/chip-gui"
  CLI_BIN="$TARGET_DIR/release/chipcutter"
else
  cargo build --manifest-path "$REPO_ROOT/Cargo.toml" -p chip-gui -p chip-cli
  BIN="$TARGET_DIR/debug/chip-gui"
  CLI_BIN="$TARGET_DIR/debug/chipcutter"
fi

TMP_PROJ_DIR="$(mktemp -d "${TMPDIR:-/tmp}/chipcutter_tutorial_proj.XXXXXX")"
trap 'rm -rf "$TMP_PROJ_DIR"' EXIT

pick_free_x_display() {
  local d=199
  while [[ -e "/tmp/.X${d}-lock" || -L "/tmp/.X${d}-lock" || -e "/tmp/.X11-unix/X${d}" || -S "/tmp/.X11-unix/X${d}" ]]; do
    d=$((d + 1))
  done
  echo "$d"
}

CAPTURE_TIMEOUT_SEC="${CAPTURE_TIMEOUT_SEC:-180}"

capture_shot() {
  local out_rel="$1"
  shift
  local out_abs="$REPO_ROOT/$out_rel"
  mkdir -p "$(dirname "$out_abs")"
  local disp
  disp="$(pick_free_x_display)"
  echo "[capture] -> $out_rel (display :$disp)"
  WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan \
    timeout --foreground "$CAPTURE_TIMEOUT_SEC" \
    xvfb-run -n "$disp" -s "-screen 0 1440x900x24" \
    "$BIN" --theme "$THEME" --self-test-screenshot "$out_abs" "$@"
}

capture_00_getting_started() {
  local dir="docs/images/tutorials/00-getting-started"

  # 1. Quick-Start Welcome Overlay on bare launch
  capture_shot "$dir/01-welcome-overlay.png" \
    --welcome

  # 2. Filterable Sample Projects Gallery modal
  capture_shot "$dir/02-sample-gallery.png" \
    --sample cam-torture \
    --capture-modal samples

  # 3. Full 3D CAM workspace overview (Browser, Viewport, Inspector, Bottom Tabs)
  capture_shot "$dir/03-workspace-overview.png" \
    --sample multi-feature \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-sim-stock false

  # 4. Setup WCS, stock bounds, 27-point origin dots, & Z-height planes
  capture_shot "$dir/04-setup-wcs-stock.png" \
    --sample multi-feature \
    --capture-view iso \
    --capture-focus setup:wcs \
    --capture-sim-stock false \
    --capture-27pt-dots true \
    --capture-z-planes true

  # 5. Tool Library modal (cutters, material presets, SFM/chipload calculator)
  capture_shot "$dir/05-tool-library.png" \
    --sample multi-feature \
    --capture-modal tools

  # 6. 3D Z-dexel machined stock simulation & animated cutter assembly
  capture_shot "$dir/06-simulation-stock.png" \
    --sample multi-feature \
    --capture-generate \
    --capture-view iso \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-progress 0.85

  # 7. Verified G-Code Preview tab (LinuxCNC / GRBL post-processor output)
  capture_shot "$dir/07-gcode-preview.png" \
    --sample multi-feature \
    --capture-generate \
    --capture-view iso \
    --capture-pane gcode \
    --capture-sim-stock true \
    --capture-sim-progress 1.0
}

capture_01_facing() {
  local dir="docs/images/tutorials/01-facing"
  local step="$REPO_ROOT/docs/tutorials/assets/01-facing-billet.step"
  local proj_empty="$TMP_PROJ_DIR/01-facing-empty.chip.toml"
  local proj_full="$TMP_PROJ_DIR/01-facing.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 01-facing --empty-project --project-out "$proj_empty"
  "$CLI_BIN" generate-sample-step "$step" --tutorial 01-facing --project-out "$proj_full"

  # 1. Empty 0-operation project with ONLY 01-facing-billet.step + Stock Top Allowance (2 mm)
  capture_shot "$dir/01-stock-allowance.png" \
    "$proj_empty" \
    --capture-view iso \
    --capture-focus setup:stock \
    --capture-sim-stock false \
    --capture-sim-cutter false \
    --capture-z-planes true

  # 2. Facing Passes tab + clean facing toolpath (ONLY Op 1: Face Stock Top, T7 38mm APKT Face Mill)
  capture_shot "$dir/02-facing-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Facing Heights tab showing StockTop -> ModelTop Z planes
  capture_shot "$dir/03-facing-heights.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:heights \
    --capture-sim-stock false \
    --capture-sim-cutter false \
    --capture-z-planes true

  # 4. 3D Simulation of T7 38mm APKT Face Mill decking the billet flat
  capture_shot "$dir/04-facing-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.85

  # 5. Verified GRBL Facing G-code preview (single T7 .nc file)
  capture_shot "$dir/05-facing-gcode.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane gcode \
    --capture-post grbl \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_02_2d_contour_tabs() {
  local dir="docs/images/tutorials/02-2d-contour-tabs"
  local step="$REPO_ROOT/docs/tutorials/assets/02-contour-plate.step"
  local proj_full="$TMP_PROJ_DIR/02-contour.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 02-contour --project-out "$proj_full"

  # 1. Closed outer loop geometry selection on minimal 100x70x12mm plate (ONLY 1 op)
  capture_shot "$dir/01-contour-geometry.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:geometry \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. 2D Contour Passes & Triangular Holding Tabs configuration + 3D tab markers
  capture_shot "$dir/02-contour-passes-tabs.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Linking tab: helical/linear ramps & horizontal/vertical tangent lead-in/out arcs
  capture_shot "$dir/03-contour-linking.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:linking \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Simulation of contoured plate retained by triangular holding tabs
  capture_shot "$dir/04-contour-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.95

  # 5. Verified GRBL G-code preview (single T1 contour file)
  capture_shot "$dir/05-contour-gcode.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane gcode \
    --capture-post grbl \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_03_2d_pocket() {
  local dir="docs/images/tutorials/03-2d-pocket"
  local step="$REPO_ROOT/docs/tutorials/assets/03-pocket-block.step"
  local proj_full="$TMP_PROJ_DIR/03-pocket.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 03-pocket --project-out "$proj_full"

  # 1. Closed multi-island pocket floor geometry selection (ONLY 2 pocket ops)
  capture_shot "$dir/01-island-pocket-geometry.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:geometry \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. 2D Pocket Passes tab: contour-parallel offset, stepover, radial/axial stock-to-leave
  capture_shot "$dir/02-island-pocket-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Open-shoulder pocket showing air overhang expansion past 3 open edges
  capture_shot "$dir/03-open-shoulder-pocket.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Simulation of both pocket ops (closed island pocket + open-shoulder step)
  capture_shot "$dir/04-pocket-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.95
}

capture_04_drilling_helical_bore() {
  local dir="docs/images/tutorials/04-drilling-helical-bore"
  local step="$REPO_ROOT/docs/tutorials/assets/04-drill-bore-plate.step"
  local proj_full="$TMP_PROJ_DIR/04-drill-bore.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 04-drill-bore --project-out "$proj_full"

  # 1. Batch-picked 4x Ø6mm corner through-holes (+ Same Ø) on minimal hole plate
  capture_shot "$dir/01-drill-holes-geometry.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:geometry \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Peck drilling cycle parameters (Spot / G81 / G83 / G73, breakthrough)
  capture_shot "$dir/02-drill-peck-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Helical Bore milling on central Ø18mm blind counterbore (T1 Flat Endmill)
  capture_shot "$dir/03-helical-bore-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Simulation of drilled corner holes + helically bored central counterbore
  capture_shot "$dir/04-drill-bore-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.95

  # 5. LinuxCNC G-Code Preview showing native G83 canned cycles & helical bore (T6 + T1 only)
  capture_shot "$dir/05-drill-gcode-linuxcnc.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane gcode \
    --capture-post linuxcnc \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_05_adaptive_rest_machining() {
  local dir="docs/images/tutorials/05-adaptive-rest-machining"
  local step="$REPO_ROOT/docs/tutorials/assets/05-adaptive-pocket.step"
  local proj_full="$TMP_PROJ_DIR/05-adaptive-rest.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 05-adaptive-rest --project-out "$proj_full"

  # 1. Deep pocket floor selection on minimal 100x80x20mm block (ONLY 2 ops)
  capture_shot "$dir/01-adaptive-geometry.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:geometry \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. 2D Adaptive Clearing Passes tab (Optimal Load, Micro-Lift Z, Chip Thinning RCT banner)
  capture_shot "$dir/02-adaptive-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. 2.5D Rest Machining Passes tab (Rest Mode: Preceding, Ø3mm corner-only cleanup)
  capture_shot "$dir/03-rest-machining-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Simulation (Color by Op): Ø10mm Adaptive bulk roughing vs. Ø3mm Rest corner cleanup
  capture_shot "$dir/04-adaptive-rest-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.98

  # 5. Verified LinuxCNC G-code preview (Tools: [10, 3])
  capture_shot "$dir/05-adaptive-rest-gcode.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane gcode \
    --capture-post linuxcnc \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_06_chamfer_curve_trace() {
  local dir="docs/images/tutorials/06-chamfer-curve-trace"
  local step="$REPO_ROOT/docs/tutorials/assets/06-chamfer-engrave-block.step"
  local proj_full="$TMP_PROJ_DIR/06-chamfer-trace.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 06-chamfer-trace --project-out "$proj_full"

  # 1. Modeled 45° x 1.5 mm front bevel chamfer (Passes tab + isolated path)
  capture_shot "$dir/01-modeled-chamfer-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Sharp unmodeled back edge deburr (Sharp Edge, Chamfer Width, Tip Offset, Floor Guard)
  capture_shot "$dir/02-sharp-edge-chamfer.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. 1-Pass Single-Line Hershey Text Engraving (`2D Curve Trace`, Center compensation)
  capture_shot "$dir/03-hershey-sketch-trace.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Simulation of all 3 T5 operations (front bevel, back deburr, engraved CHIP-06)
  capture_shot "$dir/04-chamfer-trace-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.95

  # 5. Verified GRBL single-tool T5 G-code preview
  capture_shot "$dir/05-chamfer-trace-gcode.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-pane gcode \
    --capture-post grbl \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_07_vcarve_inlays() {
  local dir="docs/images/tutorials/07-vcarve-inlays"
  local proj_medial="$TMP_PROJ_DIR/07-vcarve-medial.chip.toml"
  local proj_inlay="$TMP_PROJ_DIR/07-vcarve-inlay.chip.toml"

  "$CLI_BIN" generate-sample-step "$proj_medial" --tutorial 07-vcarve-medial
  "$CLI_BIN" generate-sample-step "$proj_inlay" --tutorial 07-vcarve-inlay

  # 1. Pure Full-Depth Medial-Axis V-Carving (Voronoi bisectors + sharp 3D corner climbs)
  capture_shot "$dir/01-vcarve-medial-axis.png" \
    "$proj_medial" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Two-Stage Flat Area Clearance (Flat Depth = 3.0 mm, Stage 1 T3 + Stage 2 T8 V-Bit)
  capture_shot "$dir/02-vcarve-two-stage-clearance.png" \
    "$proj_inlay" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:0:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. 3D Simulation of Female Plaque (crisp 60° beveled walls & flat letter bottoms)
  capture_shot "$dir/03-vcarve-simulation-female.png" \
    "$proj_inlay" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.96

  # 4. Setup 2 (G55): Mirrored Mated Male V-Inlay Plug (Start Depth, Glue Gap, Auto-Mirror X)
  capture_shot "$dir/04-vcarve-male-inlay-plug.png" \
    "$proj_inlay" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.96

  # 5. Verified GRBL Split-by-Tool G-code preview (automatic Stage 1 T3 + Stage 2 T8 two-file split)
  capture_shot "$dir/05-vcarve-gcode-grbl-split.png" \
    "$proj_inlay" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:0:passes \
    --capture-pane gcode \
    --capture-post grbl \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_08_3d_adaptive_waterline() {
  local dir="docs/images/tutorials/08-3d-adaptive-waterline"
  local mesh="$REPO_ROOT/docs/tutorials/assets/08-sculpted-dome.stl"
  local proj_empty="$TMP_PROJ_DIR/08-adaptive-waterline-empty.chip.toml"
  local proj_full="$TMP_PROJ_DIR/08-adaptive-waterline.chip.toml"

  "$CLI_BIN" generate-sample-step "$mesh" --tutorial 08-3d-adaptive-waterline --empty-project --project-out "$proj_empty"
  "$CLI_BIN" generate-sample-step "$mesh" --tutorial 08-3d-adaptive-waterline --project-out "$proj_full"

  # 1. Bare 08-sculpted-dome.stl imported with Setup Stock/WCS
  capture_shot "$dir/01-mesh-import-stock.png" \
    "$proj_empty" \
    --capture-view iso \
    --capture-focus setup:stock \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Op 1: 3D Adaptive Roughing Passes tab + generated trochoidal roughing toolpath
  capture_shot "$dir/02-adaptive3d-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Op 2: 3D Waterline / Z-Level Passes tab + dense contour Z-level toolpath
  capture_shot "$dir/03-waterline3d-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Z-dexel machined stock colored by operation with Verified Crash-Free
  capture_shot "$dir/04-simulation-verified.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.98

  # 5. Bottom G-Code Preview tab (LinuxCNC ATC T1 -> T4)
  capture_shot "$dir/05-gcode-preview.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane gcode \
    --capture-post linuxcnc \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_09_3d_surface_finishing() {
  local dir="docs/images/tutorials/09-3d-surface-finishing"
  local mesh="$REPO_ROOT/docs/tutorials/assets/09-relief-wave.obj"
  local proj_empty="$TMP_PROJ_DIR/09-surface-finishing-empty.chip.toml"
  local proj_full="$TMP_PROJ_DIR/09-surface-finishing.chip.toml"

  "$CLI_BIN" generate-sample-step "$mesh" --tutorial 09-3d-surface-finishing --empty-project --project-out "$proj_empty"
  "$CLI_BIN" generate-sample-step "$mesh" --tutorial 09-3d-surface-finishing --project-out "$proj_full"

  # 1. 09-relief-wave.obj imported in Setup 1 with stock & WCS
  capture_shot "$dir/01-relief-wave-import.png" \
    "$proj_empty" \
    --capture-view iso \
    --capture-focus setup:stock \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. 3D Parallel Finishing Inspector + raster finishing paths
  capture_shot "$dir/02-parallel3d-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. 3D Constant-Scallop Inspector + geodesic iso-contours & 3D Rest Machining card
  capture_shot "$dir/03-scallop3d-rest.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Pencil Cleanup on 08-sculpted-dome.stl (bi-tangent pedestal-dome crease path)
  capture_shot "$dir/04-pencil3d-fillet.png" \
    --sample mesh-dome \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 5. Final machined Relief Wave Z-dexel stock with Verified Crash-Free
  capture_shot "$dir/05-finishing-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_10_rotary_wrapping_surfacing() {
  local dir="docs/images/tutorials/10-rotary-wrapping-surfacing"
  local shaft="$REPO_ROOT/docs/tutorials/assets/09-rotary-shaft.stl"
  local proj_wrap="$TMP_PROJ_DIR/10-rotary-wrap.chip.toml"
  local proj_axial="$TMP_PROJ_DIR/10-rotary-axial.chip.toml"

  "$CLI_BIN" generate-sample-step "$proj_wrap" --tutorial 10-rotary-wrap
  "$CLI_BIN" generate-sample-step "$shaft" --tutorial 10-rotary-axial --project-out "$proj_axial"

  # 1. Setup 1 -> Rotary tab showing 2-Linear + Rotary, X-Axis (A), chuck & clearance gizmo
  capture_shot "$dir/01-setup-rotary-tab.png" \
    "$proj_wrap" \
    --capture-view iso \
    --capture-focus setup:rotary \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Wrapped V-Carve 'CHIP' with Wrap 2D card + wrapped 3D toolpath
  capture_shot "$dir/02-wrap2d-vcarve-inspector.png" \
    "$proj_wrap" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. CylindricalStock simulation of wrapped 'CHIP' + flute pocket
  capture_shot "$dir/03-wrap2d-sim-and-m6.png" \
    "$proj_wrap" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.96

  # 4. 4th-Axis Rotary Surfacing Inspector (Op 2: Axial Parallel + -60°..+60° partial angular sector & 3D cutting-area preview in [Geometry])
  capture_shot "$dir/04-rotary2linear-axial-parallel.png" \
    "$proj_axial" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:geometry \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 5. G-Code Preview set to grbl_rotary_wrap (swapped Y axis, zero A words)
  capture_shot "$dir/05-gcode-grbl-wrap.png" \
    "$proj_wrap" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane gcode \
    --capture-post grbl_rotary_wrap \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_11_indexed_3plus1_milling() {
  local dir="docs/images/tutorials/11-indexed-3plus1-milling"
  local step_block="$REPO_ROOT/docs/tutorials/assets/11-indexed-multiface-block.step"
  local shaft="$REPO_ROOT/docs/tutorials/assets/09-rotary-shaft.stl"
  local proj_step_empty="$TMP_PROJ_DIR/11-indexed-step-empty.chip.toml"
  local proj_step_full="$TMP_PROJ_DIR/11-indexed-step.chip.toml"
  local proj_shaft="$TMP_PROJ_DIR/11-indexed-shaft.chip.toml"

  "$CLI_BIN" generate-sample-step "$step_block" --tutorial 11-indexed-step --empty-project --project-out "$proj_step_empty"
  "$CLI_BIN" generate-sample-step "$step_block" --tutorial 11-indexed-step --project-out "$proj_step_full"
  "$CLI_BIN" generate-sample-step "$shaft" --tutorial 11-indexed-shaft --project-out "$proj_shaft"

  # 1. 11-indexed-multiface-block.step chucked in square bar stock with rotary centerline & clearance cage
  capture_shot "$dir/01-indexed-step-setup.png" \
    "$proj_step_empty" \
    --capture-view iso \
    --capture-focus setup:rotary \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. 4th-Axis Indexing / Wrapping card highlighting Quick Index and Align to Selected 3D Face
  capture_shot "$dir/02-align-3d-face-inspector.png" \
    "$proj_step_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:geometry \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Generated multi-face 3+1 pockets & holes around all indexed faces
  capture_shot "$dir/03-multi-face-toolpaths.png" \
    "$proj_step_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. CylindricalStock simulation of 4 wrenching flats + continuous spiral collar/cam
  capture_shot "$dir/04-rotary-shaft-flats-sim.png" \
    "$proj_shaft" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.98

  # 5. G-Code Preview (linuxcnc_4axis) showing clearance retracts + G0 A90/A180/A270 index blocks
  capture_shot "$dir/05-gcode-indexed-linuxcnc.png" \
    "$proj_step_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane gcode \
    --capture-post linuxcnc_4axis \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_12_simultaneous_4axis_milling() {
  local dir="docs/images/tutorials/12-simultaneous-4axis-milling"
  local mesh="$REPO_ROOT/docs/tutorials/assets/10-helical-flute-shaft.stl"
  local proj_empty="$TMP_PROJ_DIR/12-sim-4axis-empty.chip.toml"
  local proj_full="$TMP_PROJ_DIR/12-sim-4axis.chip.toml"

  "$CLI_BIN" generate-sample-step "$mesh" --tutorial 12-simultaneous-4axis --empty-project --project-out "$proj_empty"
  "$CLI_BIN" generate-sample-step "$mesh" --tutorial 12-simultaneous-4axis --project-out "$proj_full"

  # 1. 10-helical-flute-shaft.stl in Simultaneous 4-Axis (X+Y+Z+Rotary) Setup
  capture_shot "$dir/01-helical-setup-sim4ax.png" \
    "$proj_empty" \
    --capture-view iso \
    --capture-focus setup:rotary \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Simultaneous 4-Axis Inspector (Op 2: Auto Surface-Normal Tilt 18° + Simultaneous Helical Spiral)
  capture_shot "$dir/02-auto-tilt-spiral-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Simultaneous 4-Axis Inspector (Op 3: Axial / Helical Flowline + Fixed Lateral Offset 2.5 mm)
  capture_shot "$dir/03-fixed-offset-flowline.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Live cylindrical simulation highlighting off-centerline X, Y != 0, Z, A HUD readout
  capture_shot "$dir/04-simulation-xyza-hud.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.85

  # 5. G-Code Preview (grblhal_4axis) inspecting G93 inverse-time X/Y/Z/A blocks
  capture_shot "$dir/05-gcode-g93-grblhal.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-pane gcode \
    --capture-post grblhal_4axis \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_13_pure2d_dxf_svg_workflow() {
  local dir="docs/images/tutorials/13-pure2d-dxf-svg-workflow"
  local dxf="$REPO_ROOT/docs/tutorials/assets/13-router-panel.dxf"
  local proj_empty="$TMP_PROJ_DIR/13-pure2d-empty.chip.toml"
  local proj_full="$TMP_PROJ_DIR/13-pure2d.chip.toml"

  "$CLI_BIN" generate-sample-step "$dxf" --tutorial 13-pure2d-dxf --empty-project --project-out "$proj_empty"
  "$CLI_BIN" generate-sample-step "$dxf" --tutorial 13-pure2d-dxf --project-out "$proj_full"

  # 1. Imported 13-router-panel.dxf in Pure-2D job with Sketch Inspector focused
  capture_shot "$dir/01-dxf-import-inspector.png" \
    "$proj_empty" \
    --capture-view top \
    --capture-focus sketch:0 \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Setup 1 -> Stock showing Pure-2D virtual sheet Z thickness (6.0 mm) & Stock Top WCS origin
  capture_shot "$dir/02-pure2d-sheet-stock.png" \
    "$proj_empty" \
    --capture-view iso \
    --capture-focus setup:stock \
    --capture-sim-stock false \
    --capture-sim-cutter false \
    --capture-z-planes true

  # 3. All 4 Pure-2D operations generated with clean arc toolpaths (Drilling, Island Pocket, Vent Slot, Outer Profile + Tabs)
  capture_shot "$dir/03-operations-assigned.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. 3D Z-dexel virtual-sheet stock removal with Verified Crash-Free
  capture_shot "$dir/04-pure2d-simulation.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.96

  # 5. G-Code Preview tab (grbl) + portable .chipz bundle workflow
  capture_shot "$dir/05-gcode-and-chipz.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-pane gcode \
    --capture-post grbl \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_14_associative_step_reload() {
  local dir="docs/images/tutorials/14-associative-step-reload"
  local step_v1="$REPO_ROOT/docs/tutorials/assets/14-bracket-v1.step"
  local step_v2="$REPO_ROOT/docs/tutorials/assets/14-bracket-v2.step"
  local proj_v1="$TMP_PROJ_DIR/14-bracket-v1.chip.toml"
  local proj_v2="$TMP_PROJ_DIR/14-bracket-v2.chip.toml"

  "$CLI_BIN" generate-sample-step "$step_v1" --tutorial 14-reload-v1 --project-out "$proj_v1"
  "$CLI_BIN" generate-sample-step "$step_v2" --tutorial 14-reload-v2 --project-out "$proj_v2"

  # 1. revision-v1 loaded and generated with all operations Clean and Verified Crash-Free
  capture_shot "$dir/01-revision-v1-clean.png" \
    "$proj_v1" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0

  # 2. After associative reload to revision-v2 showing non-blocking reload summary banner, Dirty ops, & NeedsRepick
  capture_shot "$dir/02-revision-v2-rebind-banner.png" \
    --sample revision-v2 \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Focusing the NeedsRepick operation showing Clear Orphaned & Re-Pick in 3D recovery card
  capture_shot "$dir/03-needs-repick-inspector.png" \
    --sample revision-v2 \
    --capture-view iso \
    --capture-focus op:4:geometry \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Regenerated v2 toolpaths verified crash-free on the updated STEP solid
  capture_shot "$dir/04-rebuilt-v2-simulation.png" \
    "$proj_v2" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_15_machine_definitions() {
  local dir="docs/images/tutorials/15-machine-definitions-tool-sets"
  local part_step="$TMP_PROJ_DIR/part.step"
  local proj_milo="$TMP_PROJ_DIR/15-milo.chip.toml"
  local proj_carvera="$TMP_PROJ_DIR/15-carvera.chip.toml"

  "$CLI_BIN" generate-sample-step "$part_step" --cam-torture --project-out "$proj_milo" --machine milo-1.5
  "$CLI_BIN" generate-sample-step "$part_step" --cam-torture --project-out "$proj_carvera" --machine carvera-air

  # 1. Machine Manager modal showing built-in Generic, Milo v1.5, Carvera Air, etc.
  capture_shot "$dir/01-machine-manager-modal.png" \
    "$proj_milo" \
    --capture-modal machines

  # 2. Tool Library modal scoped to Milo v1.5 (LDO Kit)
  capture_shot "$dir/02-machine-scoped-tool-library.png" \
    "$proj_milo" \
    --capture-modal tools

  # 3. Setup Inspector Target CNC Machine card showing Makera Carvera Air
  capture_shot "$dir/03-setup-target-machine.png" \
    "$proj_carvera" \
    --capture-view iso \
    --capture-focus setup:wcs \
    --capture-27pt-dots true \
    --capture-sim-stock false \
    --capture-sim-cutter false
}

capture_16_model_orientation() {
  local dir="docs/images/tutorials/16-model-orientation-positioning"
  local yup_stl="$REPO_ROOT/docs/tutorials/assets/16-yup-bracket.stl"
  local tilted_step="$REPO_ROOT/docs/tutorials/assets/16-tilted-bracket.step"
  local proj_stl_empty="$TMP_PROJ_DIR/16-yup-empty.chip.toml"
  local proj_stl_full="$TMP_PROJ_DIR/16-yup-full.chip.toml"
  local proj_step_full="$TMP_PROJ_DIR/16-tilted-full.chip.toml"

  "$CLI_BIN" generate-sample-step "$yup_stl" --tutorial 16-orient-stl --empty-project --project-out "$proj_stl_empty"
  "$CLI_BIN" generate-sample-step "$yup_stl" --tutorial 16-orient-stl --project-out "$proj_stl_full"
  "$CLI_BIN" generate-sample-step "$tilted_step" --tutorial 16-orient-step --project-out "$proj_step_full"

  # 1. Sideways Y-up STL imported before orientation
  capture_shot "$dir/01-yup-stl-before-transform.png" \
    "$proj_stl_empty" \
    --capture-view iso \
    --capture-focus model \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Upright 1.5x-scaled STL centered at Z=0 with Model Transform Inspector
  capture_shot "$dir/02-model-transform-inspector.png" \
    "$proj_stl_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus model \
    --capture-pane sim \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Tilted STEP bracket aligned via Setup 1 G54 WCS
  capture_shot "$dir/03-tilted-step-wcs-align.png" \
    "$proj_step_full" \
    --capture-generate \
    --capture-view iso \
    --capture-setup 0 \
    --capture-focus setup:0:wcs \
    --capture-27pt-dots true \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Setup 2 G55 flipped upside down with live 3D machined stock
  capture_shot "$dir/04-setup2-flipped-op2.png" \
    "$proj_step_full" \
    --capture-generate \
    --capture-view iso \
    --capture-setup 1 \
    --capture-focus setup:1:wcs \
    --capture-pane sim \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.85
}

capture_17_tridexel_error_metrics() {
  local dir="docs/images/tutorials/17-tridexel-simulation-error-metrics"

  # 1. Tri-Dexel volumetric simulation of CAM Torture Block
  capture_shot "$dir/01-tridexel-volumetric-stock.png" \
    --sample cam-torture \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-resolution standard \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0

  # 2. Tri-Dexel signed Error Heatmap on CAM Torture Block
  capture_shot "$dir/02-deviation-error-heatmap.png" \
    --sample cam-torture \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-resolution standard \
    --capture-sim-heatmap \
    --capture-sim-stock true \
    --capture-sim-cutter false \
    --capture-sim-progress 1.0

  # 3. Tri-Dexel Error Heatmap on 3D Sculpted Dome STL + Diagnostics & Log
  capture_shot "$dir/03-sculpted-dome-heatmap-diagnostics.png" \
    --sample mesh-dome \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane diagnostics \
    --capture-sim-engine tri-dexel \
    --capture-sim-resolution standard \
    --capture-sim-heatmap \
    --capture-sim-stock true \
    --capture-sim-cutter false \
    --capture-sim-progress 1.0
}

capture_18_thread_milling() {
  local dir="docs/images/tutorials/18-thread-milling"
  local step="$REPO_ROOT/docs/tutorials/assets/18-threaded-manifold.step"
  local proj_full="$TMP_PROJ_DIR/18-thread-milling.chip.toml"

  "$CLI_BIN" generate-sample-step "$step" --tutorial 18-thread-milling --project-out "$proj_full"

  # 1. Tool Library Manager showing single-point ThreadMill cutters & relieved-neck schematic
  capture_shot "$dir/01-thread-mill-tool-library.png" \
    "$proj_full" \
    --capture-focus op:3:tool \
    --capture-modal tools

  # 2. Internal M8 x 1.25 Thread Mill Passes tab (ISO Metric preset, Tap Drill, Bottom-Up G3 Climb, 2 radial + 1 spring, arc feed comp)
  capture_shot "$dir/02-internal-m8-passes.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. External M12 x 1.75 Boss Thread Passes tab (External Boss, Top-Down G2 Climb, external tangent arc-in/out)
  capture_shot "$dir/03-external-m12-boss.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:5:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Tri-Dexel (Volumetric 3D) simulation showing helical undercut thread grooves & Verified Crash-Free
  capture_shot "$dir/04-tridexel-undercut-sim.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:5:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0

  # 5. Verified LinuxCNC / GRBL helical G2/G3 G-code preview
  capture_shot "$dir/05-gcode-helical-preview.png" \
    "$proj_full" \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-pane gcode \
    --capture-post linuxcnc \
    --capture-sim-engine tri-dexel \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_19_workholding_flip_jigs_soft_jaws() {
  local dir="docs/images/tutorials/19-workholding-flip-jigs-soft-jaws"
  local plate_step="$REPO_ROOT/docs/tutorials/assets/02-contour-plate.step"
  local proj_plate="$TMP_PROJ_DIR/19-contour-plate.chip.toml"
  local proj_flip="$TMP_PROJ_DIR/19-contour-plate-flip.chip.toml"

  "$CLI_BIN" generate-sample-step "$plate_step" --tutorial 02-contour --project-out "$proj_plate"
  "$CLI_BIN" workholding flip-jig "$proj_plate" -o "$proj_flip" \
    --axis x --pin-dia 6.0 --count 2 --poka-yoke 4.0 --edge-margin 6.0

  # 1. Setup 1 (G54) Fixtures tab with Machinist Vise + Parallels + End Toe Clamps & Auto-Avoid Rapids
  capture_shot "$dir/01-setup1-fixtures-vise-clamps.png" \
    --sample two-sided-vise \
    --capture-generate \
    --capture-view iso \
    --capture-focus setup:0:fixtures \
    --capture-fixtures true \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 2. Setup 2 (G55) 180-deg flip Stock Carryover & Contoured Soft Jaws
  capture_shot "$dir/02-setup2-carryover-soft-jaws.png" \
    --sample two-sided-vise \
    --capture-generate \
    --capture-view iso \
    --capture-setup 1 \
    --capture-focus setup:1:stock \
    --capture-fixtures true \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Two-Sided Flip-Jig Dowel-Pin Wizard modal with live symmetry check
  capture_shot "$dir/03-flip-jig-wizard.png" \
    "$proj_plate" \
    --capture-view iso \
    --capture-focus setup:0:fixtures \
    --capture-modal flip-jig \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Paired 180-deg Setup 2 (G55) created by Flip-Jig Wizard showing CylinderKeepOut registration pins
  capture_shot "$dir/04-flip-jig-op2-pins.png" \
    "$proj_flip" \
    --capture-view iso \
    --capture-setup 1 \
    --capture-focus setup:1:fixtures \
    --capture-fixtures true \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 5. Auto Soft-Jaw Generator Wizard modal on the two-sided bracket showing pocket area & STL preview
  capture_shot "$dir/05-soft-jaw-wizard.png" \
    --sample two-sided-vise \
    --capture-view iso \
    --capture-focus setup:0:fixtures \
    --capture-modal soft-jaws \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 6. Tri-Dexel 3D multi-setup volumetric simulation (Setup 2 G55) with stock carryover & Verified Crash-Free
  capture_shot "$dir/06-tridexel-carryover-sim.png" \
    --sample two-sided-vise \
    --capture-generate \
    --capture-view iso \
    --capture-setup 1 \
    --capture-focus setup:1:fixtures \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_20_gpu_feed_opt() {
  local dir="docs/images/tutorials/20-gpu-simulation-feed-optimization"

  # 1. Simulation Timeline showing GPU Compute adapter badge + Tri-Dexel Fine stock on Sculpted Dome (Verified Crash-Free)
  capture_shot "$dir/01-gpu-backend-toolbar.png" \
    --sample mesh-dome \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0

  # 2. 4th-axis Cylindrical GPU Ultra simulation + live Error Heatmap + high-precision metrology
  capture_shot "$dir/02-gpu-ultra-heatmap.png" \
    --sample rotary-4axis \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-backend gpu \
    --capture-sim-resolution ultra \
    --capture-sim-heatmap \
    --capture-sim-stock true \
    --capture-sim-cutter false \
    --capture-sim-progress 1.0

  # 3. Feed-Optimized Pocket Operation Inspector with expanded Feed Optimization card on Op 1
  capture_shot "$dir/03-feed-opt-inspector.png" \
    --sample feed-opt-pocket \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:0:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Simulation Timeline mid-cut showing modulated cutting feedrate range & active corner/arc F readout
  capture_shot "$dir/04-modulated-feed-timeline.png" \
    --sample feed-opt-pocket \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution standard \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.88

  # 5. G-Code Preview showing verified program ready to post modulated F words
  capture_shot "$dir/05-gcode-modulated-feeds.png" \
    --sample feed-opt-pocket \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane gcode \
    --capture-post linuxcnc \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution standard \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_21_3d_deburr() {
  local dir="docs/images/tutorials/21-3d-edge-deburring-nonplanar-chamfering"

  # 1. Tool Library modal showing Lollipop Endmill 2D schematic & relieved-neck fields
  capture_shot "$dir/01-lollipop-tool-library.png" \
    --sample curved-deburr-3d \
    --capture-focus op:5:tool \
    --capture-modal tools

  # 2. 3D Edge Deburr / Chamfer Passes card + curved sloped arch toolpath & automatic wall collision trimming
  capture_shot "$dir/02-deburr3d-passes-chamfer.png" \
    --sample curved-deburr-3d \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 3. Ball Endmill curved non-planar arch rim pass
  capture_shot "$dir/03-deburr3d-ball-rim.png" \
    --sample curved-deburr-3d \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:4:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 4. Lollipop underside back-chamfer op with Underside Back-Chamfer enabled
  capture_shot "$dir/04-deburr3d-lollipop-backchamfer.png" \
    --sample curved-deburr-3d \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:5:passes \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false

  # 5. GPU Tri-Dexel Fine verification showing crisp beveled edges & Verified Crash-Free
  capture_shot "$dir/05-deburr3d-tridexel-sim.png" \
    --sample curved-deburr-3d \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:5:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0

  # 6. G-Code Preview tab (LinuxCNC post profile + Sim Verified 0 Crashes)
  capture_shot "$dir/06-deburr3d-gcode.png" \
    --sample curved-deburr-3d \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:5:passes \
    --capture-pane gcode \
    --capture-post linuxcnc \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 1.0
}

capture_22_undercut_milling() {
  local dir="docs/images/tutorials/22-undercut-tslot-dovetail-lollipop"

  # 1. Tool Library modal showing DovetailCutter 2D schematic & derived Top Ø
  capture_shot "$dir/01-undercut-tools-library.png" \
    --sample undercut-deburr \
    --capture-focus op:3:tool \
    --capture-modal tools

  # 2. After Op 2 Pre-Slot Throats showing cleared 8.5 mm throats before undercut cutters enter
  capture_shot "$dir/02-preslot-throats.png" \
    --sample undercut-deburr \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:1:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-after-op 1

  # 3. Op 3 M8 T-Slot Undercut Passes card + mid-cut revolved SlotMill inside the T-slot
  capture_shot "$dir/03-tslot-undercut-passes.png" \
    --sample undercut-deburr \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:2:passes \
    --capture-isolate-op \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.36

  # 4. Op 4 60° Dovetail Guideway Passes card + mid-cut revolved DovetailCutter
  capture_shot "$dir/04-dovetail-slot-passes.png" \
    --sample undercut-deburr \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-isolate-op \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.56

  # 5. External 60° Dovetail Rail & Blind Keyway with Entry Bore (dovetail-rail-keyway)
  capture_shot "$dir/05-dovetail-rail-keyway.png" \
    --sample dovetail-rail-keyway \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:3:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.92

  # 6. Op 5 Lollipop Overhang Undercut Passes card + Tri-Dexel groove
  capture_shot "$dir/06-lollipop-undercut3d.png" \
    --sample undercut-deburr \
    --capture-generate \
    --capture-view back \
    --capture-focus op:4:passes \
    --capture-isolate-op \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-sim-progress 0.88

  # 7. Final GPU Tri-Dexel Fine Error Heatmap showing 98-99% In-Tol & Verified Crash-Free
  capture_shot "$dir/07-undercut-tridexel-heatmap.png" \
    --sample undercut-deburr \
    --capture-generate \
    --capture-view iso \
    --capture-focus op:5:passes \
    --capture-pane sim \
    --capture-sim-engine tri-dexel \
    --capture-sim-backend gpu \
    --capture-sim-resolution fine \
    --capture-sim-heatmap \
    --capture-sim-stock true \
    --capture-sim-cutter false \
    --capture-sim-progress 1.0
}

if [[ -n "$TUTORIAL" ]]; then
  case "$TUTORIAL" in
    00-getting-started)
      capture_00_getting_started
      ;;
    01-facing)
      capture_01_facing
      ;;
    02-2d-contour-tabs)
      capture_02_2d_contour_tabs
      ;;
    03-2d-pocket)
      capture_03_2d_pocket
      ;;
    04-drilling-helical-bore)
      capture_04_drilling_helical_bore
      ;;
    05-adaptive-rest-machining)
      capture_05_adaptive_rest_machining
      ;;
    06-chamfer-curve-trace)
      capture_06_chamfer_curve_trace
      ;;
    07-vcarve-inlays)
      capture_07_vcarve_inlays
      ;;
    08-3d-adaptive-waterline)
      capture_08_3d_adaptive_waterline
      ;;
    09-3d-surface-finishing)
      capture_09_3d_surface_finishing
      ;;
    10-rotary-wrapping-surfacing)
      capture_10_rotary_wrapping_surfacing
      ;;
    11-indexed-3plus1-milling)
      capture_11_indexed_3plus1_milling
      ;;
    12-simultaneous-4axis-milling)
      capture_12_simultaneous_4axis_milling
      ;;
    13-pure2d-dxf-svg-workflow)
      capture_13_pure2d_dxf_svg_workflow
      ;;
    14-associative-step-reload)
      capture_14_associative_step_reload
      ;;
    15-machine-definitions-tool-sets)
      capture_15_machine_definitions
      ;;
    16-model-orientation-positioning)
      capture_16_model_orientation
      ;;
    17-tridexel-simulation-error-metrics)
      capture_17_tridexel_error_metrics
      ;;
    18-thread-milling)
      capture_18_thread_milling
      ;;
    19-workholding-flip-jigs-soft-jaws)
      capture_19_workholding_flip_jigs_soft_jaws
      ;;
    20-gpu-simulation-feed-optimization)
      capture_20_gpu_feed_opt
      ;;
    21-3d-edge-deburring-nonplanar-chamfering)
      capture_21_3d_deburr
      ;;
    22-undercut-tslot-dovetail-lollipop)
      capture_22_undercut_milling
      ;;
    *)
      echo "Unknown tutorial slug: $TUTORIAL" >&2
      exit 2
      ;;
  esac
else
  case "$MILESTONE" in
    13)
      capture_00_getting_started
      ;;
    14)
      capture_01_facing
      capture_02_2d_contour_tabs
      capture_03_2d_pocket
      capture_04_drilling_helical_bore
      ;;
    15)
      capture_05_adaptive_rest_machining
      capture_06_chamfer_curve_trace
      capture_07_vcarve_inlays
      ;;
    22)
      capture_08_3d_adaptive_waterline
      capture_09_3d_surface_finishing
      ;;
    23)
      capture_10_rotary_wrapping_surfacing
      capture_11_indexed_3plus1_milling
      capture_12_simultaneous_4axis_milling
      ;;
    24)
      capture_13_pure2d_dxf_svg_workflow
      capture_14_associative_step_reload
      ;;
    47)
      capture_18_thread_milling
      ;;
    63)
      capture_19_workholding_flip_jigs_soft_jaws
      ;;
    69)
      capture_20_gpu_feed_opt
      ;;
    70)
      capture_21_3d_deburr
      capture_22_undercut_milling
      ;;
    all)
      capture_00_getting_started
      capture_01_facing
      capture_02_2d_contour_tabs
      capture_03_2d_pocket
      capture_04_drilling_helical_bore
      capture_05_adaptive_rest_machining
      capture_06_chamfer_curve_trace
      capture_07_vcarve_inlays
      capture_08_3d_adaptive_waterline
      capture_09_3d_surface_finishing
      capture_10_rotary_wrapping_surfacing
      capture_11_indexed_3plus1_milling
      capture_12_simultaneous_4axis_milling
      capture_13_pure2d_dxf_svg_workflow
      capture_14_associative_step_reload
      capture_15_machine_definitions
      capture_16_model_orientation
      capture_17_tridexel_error_metrics
      capture_18_thread_milling
      capture_19_workholding_flip_jigs_soft_jaws
      capture_20_gpu_feed_opt
      capture_21_3d_deburr
      capture_22_undercut_milling
      ;;
    *)
      echo "Unsupported milestone: $MILESTONE (available: 13, 14, 15, 22, 23, 24, 47, 63, 69, 70, all)" >&2
      exit 2
      ;;
  esac
fi

echo "[capture] Done."
