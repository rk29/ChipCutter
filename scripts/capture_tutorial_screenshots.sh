#!/usr/bin/env bash
# Repeatable headless tutorial screenshot capture pipeline for ChipCutter Phase 4.
#
# Usage:
#   ./scripts/capture_tutorial_screenshots.sh [--milestone 13|all] [--tutorial <slug>] [--theme dark|light] [--debug]
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

echo "[capture] Building chip-gui ($PROFILE)..."
if [[ "$PROFILE" == "release" ]]; then
  cargo build --manifest-path "$REPO_ROOT/Cargo.toml" --release -p chip-gui
  BIN="$REPO_ROOT/target/release/chip-gui"
else
  cargo build --manifest-path "$REPO_ROOT/Cargo.toml" -p chip-gui
  BIN="$REPO_ROOT/target/debug/chip-gui"
fi

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

if [[ -n "$TUTORIAL" ]]; then
  case "$TUTORIAL" in
    00-getting-started)
      capture_00_getting_started
      ;;
    *)
      echo "Unknown tutorial slug: $TUTORIAL" >&2
      exit 2
      ;;
  esac
else
  case "$MILESTONE" in
    13|all)
      capture_00_getting_started
      ;;
    *)
      echo "Unsupported milestone: $MILESTONE (available: 13, all)" >&2
      exit 2
      ;;
  esac
fi

echo "[capture] Done."
