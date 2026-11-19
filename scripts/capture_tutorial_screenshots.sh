#!/usr/bin/env bash
# Repeatable headless tutorial screenshot capture pipeline for ChipCutter Phase 4.
#
# Usage:
#   ./scripts/capture_tutorial_screenshots.sh [--milestone 13|14|15|all] [--tutorial <slug>] [--theme dark|light] [--debug]
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

echo "[capture] Building chip-gui and chip-cli ($PROFILE)..."
if [[ "$PROFILE" == "release" ]]; then
  cargo build --manifest-path "$REPO_ROOT/Cargo.toml" --release -p chip-gui -p chip-cli
  BIN="$REPO_ROOT/target/release/chip-gui"
  CLI_BIN="$REPO_ROOT/target/release/chipcutter"
else
  cargo build --manifest-path "$REPO_ROOT/Cargo.toml" -p chip-gui -p chip-cli
  BIN="$REPO_ROOT/target/debug/chip-gui"
  CLI_BIN="$REPO_ROOT/target/debug/chipcutter"
fi

TMP_PROJ_DIR="$(mktemp -d "${TMPDIR:-/tmp}/chipcutter_tutorial_proj.XXXXXX")"
trap 'rm -rf "$TMP_PROJ_DIR"' EXIT

capture_shot() {
  local out_rel="$1"
  shift
  local out_abs="$REPO_ROOT/$out_rel"
  mkdir -p "$(dirname "$out_abs")"
  echo "[capture] -> $out_rel"
  WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan \
    xvfb-run -a -s "-screen 0 1440x900x24" \
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
    all)
      capture_00_getting_started
      capture_01_facing
      capture_02_2d_contour_tabs
      capture_03_2d_pocket
      capture_04_drilling_helical_bore
      capture_05_adaptive_rest_machining
      capture_06_chamfer_curve_trace
      capture_07_vcarve_inlays
      ;;
    *)
      echo "Unsupported milestone: $MILESTONE (available: 13, 14, 15, all)" >&2
      exit 2
      ;;
  esac
fi

echo "[capture] Done."
