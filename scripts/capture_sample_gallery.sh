#!/usr/bin/env bash
# Deterministic headless screenshot capture for the ChipCutter Sample Projects Gallery (`docs/samples/`).
# Builds `chip-gui` in release mode and captures hero images, 480px Lanczos3 thumbnails, and
# per-Setup / per-Operation completion simulation frames for all 16 built-in sample projects.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

OUT_ROOT="$REPO_ROOT/docs/images/samples"
mkdir -p "$OUT_ROOT"
ONLY_SLUG="${1:-}"

should_capture() {
    [[ -z "$ONLY_SLUG" || "$ONLY_SLUG" == "$1" ]]
}

TARGET_DIR="${CARGO_TARGET_DIR:-$REPO_ROOT/target}"
echo "==> Building chip-gui (release)..."
cargo build --release --locked -p chip-gui

BIN="$TARGET_DIR/release/chip-gui"
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

if should_capture "cam-torture"; then
echo "==> 1/16: cam-torture (8 operations)"
capture_hero_sim "cam-torture" "iso"
capture_op_done "cam-torture" 0 0 "op_01_face.png"
capture_op_done "cam-torture" 0 1 "op_02_plateau.png"
capture_op_done "cam-torture" 0 2 "op_03_open_step.png"
capture_op_done "cam-torture" 0 3 "op_04_island_pocket.png"
capture_op_done "cam-torture" 0 4 "op_05_helical_bore.png"
capture_op_done "cam-torture" 0 5 "op_06_outer_contour.png"
capture_op_done "cam-torture" 0 6 "op_07_shelf_holes.png"
capture_op_done "cam-torture" 0 7 "op_08_step_chamfer.png"
fi

if should_capture "adaptive-rest"; then
echo "==> 2/16: adaptive-rest (5 operations)"
capture_hero_sim "adaptive-rest" "iso"
capture_op_done "adaptive-rest" 0 0 "op_01_face.png"
capture_op_done "adaptive-rest" 0 1 "op_02_plateau_adaptive.png"
capture_op_done "adaptive-rest" 0 2 "op_03_cavity_rough.png"
capture_op_done "adaptive-rest" 0 3 "op_04_cavity_rest.png"
capture_op_done "adaptive-rest" 0 4 "op_05_open_shoulder.png"
fi

if should_capture "pure-2d"; then
echo "==> 3/16: pure-2d (4 operations)"
capture_hero_sim "pure-2d" "iso"
capture_op_done "pure-2d" 0 0 "op_01_mounting_holes.png"
capture_op_done "pure-2d" 0 1 "op_02_display_pocket.png"
capture_op_done "pure-2d" 0 2 "op_03_vent_slot.png"
capture_op_done "pure-2d" 0 3 "op_04_outer_profile.png"
fi

if should_capture "nameplate"; then
echo "==> 4/16: nameplate (5 operations)"
capture_hero_sim "nameplate" "iso"
capture_op_done "nameplate" 0 0 "op_01_bolt_circle.png"
capture_op_done "nameplate" 0 1 "op_02_gauge_window.png"
capture_op_done "nameplate" 0 2 "op_03_logo_text.png"
capture_op_done "nameplate" 0 3 "op_04_serial_trace.png"
capture_op_done "nameplate" 0 4 "op_05_plaque_profile.png"
fi

if should_capture "vcarve"; then
echo "==> 5/16: vcarve (2 setups, 4 operations)"
capture_hero_sim "vcarve" "iso"
capture_op_done "vcarve" 0 0 "setup_01_op_01_female_vcarve.png"
capture_op_done "vcarve" 0 1 "setup_01_op_02_subtitle.png"
capture_op_done "vcarve" 0 2 "setup_01_op_03_sign_profile.png"
capture_op_done "vcarve" 1 0 "setup_02_op_01_male_plug.png"
fi

if should_capture "multi-feature"; then
echo "==> 6/16: multi-feature (2 operations)"
capture_hero_sim "multi-feature" "iso"
capture_op_done "multi-feature" 0 0 "op_01_center_pocket.png"
capture_op_done "multi-feature" 0 1 "op_02_mounting_holes.png"
fi

if should_capture "tilted-bracket"; then
echo "==> 7/16: tilted-bracket (WCS alignment sandbox)"
run_capture "tilted-bracket" "hero.png" \
    --capture-thumb "$OUT_ROOT/tilted-bracket/thumb.png" \
    --capture-view iso \
    --capture-focus "setup:0:wcs" \
    --capture-27pt-dots true
fi

if should_capture "revision-v1"; then
echo "==> 8/16: revision-v1 (6 operations)"
capture_hero_sim "revision-v1" "iso"
capture_op_done "revision-v1" 0 0 "op_01_face.png"
capture_op_done "revision-v1" 0 1 "op_02_blind_pocket.png"
capture_op_done "revision-v1" 0 2 "op_03_helical_bore.png"
capture_op_done "revision-v1" 0 3 "op_04_hole_keep.png"
capture_op_done "revision-v1" 0 4 "op_05_hole_deleted.png"
capture_op_done "revision-v1" 0 5 "op_06_front_chamfer.png"
fi

if should_capture "revision-v2"; then
echo "==> 9/16: revision-v2 (associative reload, 5 surviving + 1 NeedsRepick)"
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
fi

if should_capture "sample-plate"; then
echo "==> 10/16: sample-plate (blank STEP sandbox)"
run_capture "sample-plate" "hero.png" \
    --capture-thumb "$OUT_ROOT/sample-plate/thumb.png" \
    --capture-view iso \
    --capture-focus "setup:0:wcs" \
    --capture-27pt-dots true
fi

if should_capture "mesh-dome"; then
echo "==> 11/16: mesh-dome (3 operations)"
capture_hero_sim "mesh-dome" "iso"
capture_op_done "mesh-dome" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-dome" 0 1 "op_02_waterline_3d.png"
capture_op_done "mesh-dome" 0 2 "op_03_pencil_3d.png"
fi

if should_capture "mesh-wave"; then
echo "==> 12/16: mesh-wave (3 operations)"
capture_hero_sim "mesh-wave" "iso"
capture_op_done "mesh-wave" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-wave" 0 1 "op_02_parallel_3d.png"
capture_op_done "mesh-wave" 0 2 "op_03_scallop_3d.png"
fi

if should_capture "rotary-shaft"; then
echo "==> 13/16: rotary-shaft (5 operations)"
capture_hero_sim "rotary-shaft" "iso"
capture_op_done "rotary-shaft" 0 0 "op_01_rotary_spiral.png"
capture_op_done "rotary-shaft" 0 1 "op_02_flat_0deg.png"
capture_op_done "rotary-shaft" 0 2 "op_03_flat_90deg.png"
capture_op_done "rotary-shaft" 0 3 "op_04_flat_180deg.png"
capture_op_done "rotary-shaft" 0 4 "op_05_flat_270deg.png"
fi

if should_capture "rotary-wrap"; then
echo "==> 14/16: rotary-wrap (2 operations)"
capture_hero_sim "rotary-wrap" "iso"
capture_op_done "rotary-wrap" 0 0 "op_01_wrap_vcarve.png"
capture_op_done "rotary-wrap" 0 1 "op_02_wrap_flute.png"
fi

if should_capture "rotary-4axis"; then
echo "==> 15/16: rotary-4axis (2 operations)"
capture_hero_sim "rotary-4axis" "iso"
capture_op_done "rotary-4axis" 0 0 "op_01_4ax_rough.png"
capture_op_done "rotary-4axis" 0 1 "op_02_4ax_finish.png"
fi

if should_capture "thread-mill"; then
echo "==> 16/16: thread-mill (6 operations, Tri-Dexel volumetric undercuts)"
run_capture "thread-mill" "hero.png" \
    --capture-thumb "$OUT_ROOT/thread-mill/thumb.png" \
    --capture-generate \
    --capture-sim-engine tri-dexel \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:5:passes"
for op_pair in \
    "0:op_01_face.png" \
    "1:op_02_clear_boss_deck.png" \
    "2:op_03_bore_pilots.png" \
    "3:op_04_thread_m8.png" \
    "4:op_05_thread_unc.png" \
    "5:op_06_thread_m12_boss.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "thread-mill" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-engine tri-dexel \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:passes"
done
fi

echo "==> Done! Captured $(find "$OUT_ROOT" -name '*.png' | wc -l) PNG images under $OUT_ROOT"
