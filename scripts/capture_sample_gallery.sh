#!/usr/bin/env bash
# Deterministic headless screenshot capture for the ChipCutter Sample Projects Gallery (`docs/samples/`).
# Builds `chip-gui` in release mode and captures hero images, 480px Lanczos3 thumbnails, and
# per-Setup / per-Operation completion simulation frames for all 15 built-in sample projects.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

OUT_ROOT="$REPO_ROOT/docs/images/samples"
mkdir -p "$OUT_ROOT"

echo "==> Building chip-gui (release)..."
cargo build --release --locked -p chip-gui

BIN="$REPO_ROOT/target/release/chip-gui"
if [[ ! -x "$BIN" ]]; then
    echo "ERROR: expected release binary at $BIN" >&2
    exit 1
fi

run_capture() {
    local slug="$1"
    local filename="$2"
    shift 2
    local dest="$OUT_ROOT/$slug/$filename"
    mkdir -p "$OUT_ROOT/$slug"
    echo "  -> [$slug] $filename"
    WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan \
        xvfb-run -a "$BIN" \
        --sample "$slug" \
        --theme dark \
        --self-test-screenshot "$dest" \
        "$@"
}

capture_hero_sim() {
    local slug="$1"
    local view="${2:-iso}"
    run_capture "$slug" "hero.png" \
        --capture-thumb "$OUT_ROOT/$slug/thumb.png" \
        --capture-generate \
        --capture-sim-progress 1.0 \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view "$view" \
        --capture-focus "setup:0:wcs"
}

capture_op_done() {
    local slug="$1"
    local setup_idx="$2"
    local op_idx="$3"
    local filename="$4"
    local view="${5:-iso}"
    run_capture "$slug" "$filename" \
        --capture-generate \
        --capture-setup "$setup_idx" \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view "$view" \
        --capture-focus "op:${setup_idx}:${op_idx}:passes"
}

echo "==> 1/15: cam-torture (8 operations)"
capture_hero_sim "cam-torture" "iso"
capture_op_done "cam-torture" 0 0 "op_01_face.png"
capture_op_done "cam-torture" 0 1 "op_02_plateau.png"
capture_op_done "cam-torture" 0 2 "op_03_open_step.png"
capture_op_done "cam-torture" 0 3 "op_04_island_pocket.png"
capture_op_done "cam-torture" 0 4 "op_05_helical_bore.png"
capture_op_done "cam-torture" 0 5 "op_06_outer_contour.png"
capture_op_done "cam-torture" 0 6 "op_07_shelf_holes.png"
capture_op_done "cam-torture" 0 7 "op_08_step_chamfer.png"

echo "==> 2/15: adaptive-rest (5 operations)"
capture_hero_sim "adaptive-rest" "iso"
capture_op_done "adaptive-rest" 0 0 "op_01_face.png"
capture_op_done "adaptive-rest" 0 1 "op_02_plateau_adaptive.png"
capture_op_done "adaptive-rest" 0 2 "op_03_cavity_rough.png"
capture_op_done "adaptive-rest" 0 3 "op_04_cavity_rest.png"
capture_op_done "adaptive-rest" 0 4 "op_05_open_shoulder.png"

echo "==> 3/15: pure-2d (4 operations)"
capture_hero_sim "pure-2d" "iso"
capture_op_done "pure-2d" 0 0 "op_01_mounting_holes.png"
capture_op_done "pure-2d" 0 1 "op_02_display_pocket.png"
capture_op_done "pure-2d" 0 2 "op_03_vent_slot.png"
capture_op_done "pure-2d" 0 3 "op_04_outer_profile.png"

echo "==> 4/15: nameplate (5 operations)"
capture_hero_sim "nameplate" "iso"
capture_op_done "nameplate" 0 0 "op_01_bolt_circle.png"
capture_op_done "nameplate" 0 1 "op_02_gauge_window.png"
capture_op_done "nameplate" 0 2 "op_03_logo_text.png"
capture_op_done "nameplate" 0 3 "op_04_serial_trace.png"
capture_op_done "nameplate" 0 4 "op_05_plaque_profile.png"

echo "==> 5/15: vcarve (2 setups, 4 operations)"
capture_hero_sim "vcarve" "iso"
capture_op_done "vcarve" 0 0 "setup_01_op_01_female_vcarve.png"
capture_op_done "vcarve" 0 1 "setup_01_op_02_subtitle.png"
capture_op_done "vcarve" 0 2 "setup_01_op_03_sign_profile.png"
capture_op_done "vcarve" 1 0 "setup_02_op_01_male_plug.png"

echo "==> 6/15: multi-feature (2 operations)"
capture_hero_sim "multi-feature" "iso"
capture_op_done "multi-feature" 0 0 "op_01_center_pocket.png"
capture_op_done "multi-feature" 0 1 "op_02_mounting_holes.png"

echo "==> 7/15: tilted-bracket (WCS alignment sandbox)"
run_capture "tilted-bracket" "hero.png" \
    --capture-thumb "$OUT_ROOT/tilted-bracket/thumb.png" \
    --capture-view iso \
    --capture-focus "setup:0:wcs" \
    --capture-27pt-dots true

echo "==> 8/15: revision-v1 (6 operations)"
capture_hero_sim "revision-v1" "iso"
capture_op_done "revision-v1" 0 0 "op_01_face.png"
capture_op_done "revision-v1" 0 1 "op_02_blind_pocket.png"
capture_op_done "revision-v1" 0 2 "op_03_helical_bore.png"
capture_op_done "revision-v1" 0 3 "op_04_hole_keep.png"
capture_op_done "revision-v1" 0 4 "op_05_hole_deleted.png"
capture_op_done "revision-v1" 0 5 "op_06_front_chamfer.png"

echo "==> 9/15: revision-v2 (associative reload, 5 surviving + 1 NeedsRepick)"
capture_hero_sim "revision-v2" "iso"
capture_op_done "revision-v2" 0 0 "op_01_face.png"
capture_op_done "revision-v2" 0 1 "op_02_resized_pocket.png"
capture_op_done "revision-v2" 0 2 "op_03_shifted_bore.png"
capture_op_done "revision-v2" 0 3 "op_04_hole_keep.png"
run_capture "revision-v2" "op_05_hole_needs_repick.png" \
    --capture-generate \
    --capture-setup 0 \
    --capture-sim-after-op 3 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:4:geometry"
capture_op_done "revision-v2" 0 5 "op_06_front_chamfer.png"

echo "==> 10/15: sample-plate (blank STEP sandbox)"
run_capture "sample-plate" "hero.png" \
    --capture-thumb "$OUT_ROOT/sample-plate/thumb.png" \
    --capture-view iso \
    --capture-focus "setup:0:wcs" \
    --capture-27pt-dots true

echo "==> 11/15: mesh-dome (3 operations)"
capture_hero_sim "mesh-dome" "iso"
capture_op_done "mesh-dome" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-dome" 0 1 "op_02_waterline_3d.png"
capture_op_done "mesh-dome" 0 2 "op_03_pencil_3d.png"

echo "==> 12/15: mesh-wave (3 operations)"
capture_hero_sim "mesh-wave" "iso"
capture_op_done "mesh-wave" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-wave" 0 1 "op_02_parallel_3d.png"
capture_op_done "mesh-wave" 0 2 "op_03_scallop_3d.png"

echo "==> 13/15: rotary-shaft (5 operations)"
capture_hero_sim "rotary-shaft" "iso"
capture_op_done "rotary-shaft" 0 0 "op_01_rotary_spiral.png"
capture_op_done "rotary-shaft" 0 1 "op_02_flat_0deg.png"
capture_op_done "rotary-shaft" 0 2 "op_03_flat_90deg.png"
capture_op_done "rotary-shaft" 0 3 "op_04_flat_180deg.png"
capture_op_done "rotary-shaft" 0 4 "op_05_flat_270deg.png"

echo "==> 14/15: rotary-wrap (2 operations)"
capture_hero_sim "rotary-wrap" "iso"
capture_op_done "rotary-wrap" 0 0 "op_01_wrap_vcarve.png"
capture_op_done "rotary-wrap" 0 1 "op_02_wrap_flute.png"

echo "==> 15/15: rotary-4axis (2 operations)"
capture_hero_sim "rotary-4axis" "iso"
capture_op_done "rotary-4axis" 0 0 "op_01_4ax_rough.png"
capture_op_done "rotary-4axis" 0 1 "op_02_4ax_finish.png"

echo "==> Done! Captured $(find "$OUT_ROOT" -name '*.png' | wc -l) PNG images under $OUT_ROOT"
