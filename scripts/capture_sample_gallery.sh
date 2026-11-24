#!/usr/bin/env bash
# Deterministic headless screenshot capture for the ChipCutter Sample Projects Gallery (`docs/samples/`).
# Builds `chip-gui` in release mode and captures hero images, 480px Lanczos3 thumbnails, and
# per-Setup / per-Operation completion simulation frames for all 26 documented sample projects.

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

pick_free_x_display() {
    local d=199
    while [[ -e "/tmp/.X${d}-lock" || -L "/tmp/.X${d}-lock" || -e "/tmp/.X11-unix/X${d}" || -S "/tmp/.X11-unix/X${d}" ]]; do
        d=$((d + 1))
    done
    echo "$d"
}

CAPTURE_TIMEOUT_SEC="${CAPTURE_TIMEOUT_SEC:-180}"

run_capture() {
    local slug="$1"
    local filename="$2"
    shift 2
    local dest="$OUT_ROOT/$slug/$filename"
    mkdir -p "$OUT_ROOT/$slug"
    local disp
    disp="$(pick_free_x_display)"
    echo "  -> [$slug] $filename (display :$disp)"
    WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan \
        timeout --foreground "$CAPTURE_TIMEOUT_SEC" \
        xvfb-run -n "$disp" -s "-screen 0 1440x900x24" "$BIN" \
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
echo "==> 1/26: cam-torture (8 operations)"
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
echo "==> 2/26: adaptive-rest (5 operations)"
capture_hero_sim "adaptive-rest" "iso"
capture_op_done "adaptive-rest" 0 0 "op_01_face.png"
capture_op_done "adaptive-rest" 0 1 "op_02_plateau_adaptive.png"
capture_op_done "adaptive-rest" 0 2 "op_03_cavity_rough.png"
capture_op_done "adaptive-rest" 0 3 "op_04_cavity_rest.png"
capture_op_done "adaptive-rest" 0 4 "op_05_open_shoulder.png"
fi

if should_capture "pure-2d"; then
echo "==> 3/26: pure-2d (4 operations)"
capture_hero_sim "pure-2d" "iso"
capture_op_done "pure-2d" 0 0 "op_01_mounting_holes.png"
capture_op_done "pure-2d" 0 1 "op_02_display_pocket.png"
capture_op_done "pure-2d" 0 2 "op_03_vent_slot.png"
capture_op_done "pure-2d" 0 3 "op_04_outer_profile.png"
fi

if should_capture "nameplate"; then
echo "==> 4/26: nameplate (5 operations)"
capture_hero_sim "nameplate" "iso"
capture_op_done "nameplate" 0 0 "op_01_bolt_circle.png"
capture_op_done "nameplate" 0 1 "op_02_gauge_window.png"
capture_op_done "nameplate" 0 2 "op_03_logo_text.png"
capture_op_done "nameplate" 0 3 "op_04_serial_trace.png"
capture_op_done "nameplate" 0 4 "op_05_plaque_profile.png"
fi

if should_capture "vcarve"; then
echo "==> 5/26: vcarve (2 setups, 4 operations)"
capture_hero_sim "vcarve" "iso"
capture_op_done "vcarve" 0 0 "setup_01_op_01_female_vcarve.png"
capture_op_done "vcarve" 0 1 "setup_01_op_02_subtitle.png"
capture_op_done "vcarve" 0 2 "setup_01_op_03_sign_profile.png"
capture_op_done "vcarve" 1 0 "setup_02_op_01_male_plug.png"
fi

if should_capture "multi-feature"; then
echo "==> 6/26: multi-feature (2 operations)"
capture_hero_sim "multi-feature" "iso"
capture_op_done "multi-feature" 0 0 "op_01_center_pocket.png"
capture_op_done "multi-feature" 0 1 "op_02_mounting_holes.png"
fi

if should_capture "tilted-bracket"; then
echo "==> 7/26: tilted-bracket (WCS alignment sandbox)"
run_capture "tilted-bracket" "hero.png" \
    --capture-thumb "$OUT_ROOT/tilted-bracket/thumb.png" \
    --capture-view iso \
    --capture-focus "setup:0:wcs" \
    --capture-27pt-dots true
fi

if should_capture "revision-v1"; then
echo "==> 8/26: revision-v1 (6 operations)"
capture_hero_sim "revision-v1" "iso"
capture_op_done "revision-v1" 0 0 "op_01_face.png"
capture_op_done "revision-v1" 0 1 "op_02_blind_pocket.png"
capture_op_done "revision-v1" 0 2 "op_03_helical_bore.png"
capture_op_done "revision-v1" 0 3 "op_04_hole_keep.png"
capture_op_done "revision-v1" 0 4 "op_05_hole_deleted.png"
capture_op_done "revision-v1" 0 5 "op_06_front_chamfer.png"
fi

if should_capture "revision-v2"; then
echo "==> 9/26: revision-v2 (associative reload, 5 surviving + 1 NeedsRepick)"
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
echo "==> 10/26: sample-plate (blank STEP sandbox)"
run_capture "sample-plate" "hero.png" \
    --capture-thumb "$OUT_ROOT/sample-plate/thumb.png" \
    --capture-view iso \
    --capture-focus "setup:0:wcs" \
    --capture-27pt-dots true
fi

if should_capture "mesh-dome"; then
echo "==> 11/26: mesh-dome (3 operations)"
capture_hero_sim "mesh-dome" "iso"
capture_op_done "mesh-dome" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-dome" 0 1 "op_02_waterline_3d.png"
capture_op_done "mesh-dome" 0 2 "op_03_pencil_3d.png"
fi

if should_capture "mesh-wave"; then
echo "==> 12/26: mesh-wave (3 operations)"
capture_hero_sim "mesh-wave" "iso"
capture_op_done "mesh-wave" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-wave" 0 1 "op_02_parallel_3d.png"
capture_op_done "mesh-wave" 0 2 "op_03_scallop_3d.png"
fi

if should_capture "rotary-shaft"; then
echo "==> 13/26: rotary-shaft (5 operations)"
capture_hero_sim "rotary-shaft" "iso"
capture_op_done "rotary-shaft" 0 0 "op_01_rotary_spiral.png"
capture_op_done "rotary-shaft" 0 1 "op_02_flat_0deg.png"
capture_op_done "rotary-shaft" 0 2 "op_03_flat_90deg.png"
capture_op_done "rotary-shaft" 0 3 "op_04_flat_180deg.png"
capture_op_done "rotary-shaft" 0 4 "op_05_flat_270deg.png"
fi

if should_capture "rotary-wrap"; then
echo "==> 14/26: rotary-wrap (2 operations)"
capture_hero_sim "rotary-wrap" "iso"
capture_op_done "rotary-wrap" 0 0 "op_01_wrap_vcarve.png"
capture_op_done "rotary-wrap" 0 1 "op_02_wrap_flute.png"
fi

if should_capture "rotary-4axis"; then
echo "==> 15/26: rotary-4axis (2 operations)"
capture_hero_sim "rotary-4axis" "iso"
capture_op_done "rotary-4axis" 0 0 "op_01_4ax_rough.png"
capture_op_done "rotary-4axis" 0 1 "op_02_4ax_finish.png"
fi

if should_capture "thread-mill"; then
echo "==> 16/26: thread-mill (6 operations, Tri-Dexel volumetric undercuts)"
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

if should_capture "two-sided-vise"; then
echo "==> 17/26: two-sided-vise (2 setups, 4 operations, Tri-Dexel carryover + fixtures)"
run_capture "two-sided-vise" "hero.png" \
    --capture-thumb "$OUT_ROOT/two-sided-vise/thumb.png" \
    --capture-generate \
    --capture-setup 1 \
    --capture-sim-engine tri-dexel \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "setup:1:fixtures"
for op_pair in \
    "0:setup_01_op_01_face.png" \
    "1:setup_01_op_02_adaptive_top.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "two-sided-vise" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-engine tri-dexel \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-fixtures true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:passes"
done
run_capture "two-sided-vise" "setup_02_t0_carryover.png" \
    --capture-generate \
    --capture-setup 1 \
    --capture-sim-engine tri-dexel \
    --capture-sim-progress 0.0 \
    --capture-sim-stock true \
    --capture-sim-cutter false \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "setup:1:stock"
for op_pair in \
    "0:setup_02_op_01_face_hat.png" \
    "1:setup_02_op_02_rest_skirt.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "two-sided-vise" "$op_file" \
        --capture-generate \
        --capture-setup 1 \
        --capture-sim-engine tri-dexel \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-fixtures true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:1:${op_idx}:passes"
done
fi

if should_capture "undercut-deburr"; then
echo "==> 18/26: undercut-deburr (6 operations, Tri-Dexel volumetric undercuts & 3D deburr)"
run_capture "undercut-deburr" "hero.png" \
    --capture-thumb "$OUT_ROOT/undercut-deburr/thumb.png" \
    --capture-generate \
    --capture-sim-engine tri-dexel \
    --capture-sim-resolution fine \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:5:passes"
for op_spec in \
    "0:iso:op_01_face.png" \
    "1:iso:op_02_preslot_throats.png" \
    "2:iso:op_03_tslot_m8.png" \
    "3:iso:op_04_dovetail_60.png" \
    "4:back:op_05_lollipop_undercut.png" \
    "5:iso:op_06_deburr_3d.png"; do
    IFS=':' read -r op_idx op_view op_file <<< "$op_spec"
    run_capture "undercut-deburr" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-engine tri-dexel \
        --capture-sim-resolution fine \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view "$op_view" \
        --capture-focus "op:0:${op_idx}:passes"
done
fi

if should_capture "feed-opt-pocket"; then
echo "==> 19/26: feed-opt-pocket (2 operations, engagement-aware feedrate telemetry)"
run_capture "feed-opt-pocket" "hero.png" \
    --capture-thumb "$OUT_ROOT/feed-opt-pocket/thumb.png" \
    --capture-generate \
    --capture-sim-engine tri-dexel \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:1:passes"
for op_pair in \
    "0:op_01_adaptive_feed_opt.png" \
    "1:op_02_pocket_finish_feed_opt.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "feed-opt-pocket" "$op_file" \
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

if should_capture "flip-jig-softjaws"; then
echo "==> 20/26: flip-jig-softjaws (3 setups, 8 operations, Flip-Jig + Carryover + Auto Soft Jaws)"
run_capture "flip-jig-softjaws" "hero.png" \
    --capture-thumb "$OUT_ROOT/flip-jig-softjaws/thumb.png" \
    --capture-generate \
    --capture-setup 1 \
    --capture-sim-engine tri-dexel \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "setup:1:fixtures"
for op_pair in \
    "0:setup_01_op_01_dowel_pins.png" \
    "1:setup_01_op_02_face.png" \
    "2:setup_01_op_03_top_contour.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "flip-jig-softjaws" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-engine tri-dexel \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-fixtures true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:passes"
done
run_capture "flip-jig-softjaws" "setup_02_t0_carryover.png" \
    --capture-generate \
    --capture-setup 1 \
    --capture-sim-engine tri-dexel \
    --capture-sim-progress 0.0 \
    --capture-sim-stock true \
    --capture-sim-cutter false \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "setup:1:stock"
for op_pair in \
    "0:setup_02_op_01_face_hat.png" \
    "1:setup_02_op_02_skirt_contour.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "flip-jig-softjaws" "$op_file" \
        --capture-generate \
        --capture-setup 1 \
        --capture-sim-engine tri-dexel \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-fixtures true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:1:${op_idx}:passes"
done
for op_pair in \
    "0:setup_03_op_01_face_jaws.png" \
    "1:setup_03_op_02_rough_pockets.png" \
    "2:setup_03_op_03_finish_walls.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "flip-jig-softjaws" "$op_file" \
        --capture-generate \
        --capture-setup 2 \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:2:${op_idx}:passes"
done
fi

if should_capture "curved-deburr-3d"; then
echo "==> 21/26: curved-deburr-3d (6 operations, Tri-Dexel curved & underside 3D deburring)"
run_capture "curved-deburr-3d" "hero.png" \
    --capture-thumb "$OUT_ROOT/curved-deburr-3d/thumb.png" \
    --capture-generate \
    --capture-sim-engine tri-dexel \
    --capture-sim-resolution fine \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:5:passes"
for op_pair in \
    "0:op_01_adaptive_3d.png" \
    "1:op_02_parallel_3d.png" \
    "2:op_03_side_overhang_groove.png" \
    "3:op_04_deburr_chamfer_mill.png" \
    "4:op_05_deburr_ball_arch.png" \
    "5:op_06_deburr_lollipop_underside.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "curved-deburr-3d" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-engine tri-dexel \
        --capture-sim-resolution fine \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:passes"
done
fi

if should_capture "dovetail-rail-keyway"; then
echo "==> 22/26: dovetail-rail-keyway (4 operations, Tri-Dexel blind keyway & external 60° dovetail rail)"
run_capture "dovetail-rail-keyway" "hero.png" \
    --capture-thumb "$OUT_ROOT/dovetail-rail-keyway/thumb.png" \
    --capture-generate \
    --capture-sim-engine tri-dexel \
    --capture-sim-resolution fine \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:3:passes"
for op_pair in \
    "0:op_01_face.png" \
    "1:op_02_entry_bore.png" \
    "2:op_03_blind_keyway.png" \
    "3:op_04_external_dovetail_rail.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "dovetail-rail-keyway" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-engine tri-dexel \
        --capture-sim-resolution fine \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:passes"
done
fi

if should_capture "svg-badge"; then
echo "==> 23/26: svg-badge (3 operations, Pure-2D 96 DPI SVG biarc pocket, V-carve, & tabs)"
capture_hero_sim "svg-badge" "iso"
capture_op_done "svg-badge" 0 0 "op_01_emblem_pocket.png"
capture_op_done "svg-badge" 0 1 "op_02_vcarve_accents.png"
capture_op_done "svg-badge" 0 2 "op_03_outer_contour.png"
fi

if should_capture "mesh-3mf-bracket"; then
echo "==> 24/26: mesh-3mf-bracket (2 operations, unit-aware 3MF 3D Adaptive + Constant-Scallop)"
capture_hero_sim "mesh-3mf-bracket" "iso"
capture_op_done "mesh-3mf-bracket" 0 0 "op_01_adaptive_3d.png"
capture_op_done "mesh-3mf-bracket" 0 1 "op_02_scallop_3d.png"
fi

if should_capture "keepout-clamps"; then
echo "==> 25/26: keepout-clamps (4 operations, Shared Avoid / Keep-Out Region + Toe Clamps + Boss)"
run_capture "keepout-clamps" "hero.png" \
    --capture-thumb "$OUT_ROOT/keepout-clamps/thumb.png" \
    --capture-generate \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:0:geometry"
run_capture "keepout-clamps" "geometry_keepout_preview.png" \
    --capture-generate \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false \
    --capture-fixtures true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:0:geometry"
for op_pair in \
    "0:op_01_face.png" \
    "1:op_02_adaptive_model_bounds.png" \
    "2:op_03_parallel_finish.png" \
    "3:op_04_contour_avoid_fixtures.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "keepout-clamps" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-fixtures true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:geometry"
done
fi

if should_capture "indexed-model-bounds"; then
echo "==> 26/26: indexed-model-bounds (4 operations, Fixed-Box Center-Z + Model Bounds & Auto-Rotate 3+1)"
run_capture "indexed-model-bounds" "hero.png" \
    --capture-thumb "$OUT_ROOT/indexed-model-bounds/thumb.png" \
    --capture-generate \
    --capture-sim-progress 1.0 \
    --capture-sim-stock true \
    --capture-sim-cutter true \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:1:geometry"
run_capture "indexed-model-bounds" "geometry_indexed_90_model_bounds.png" \
    --capture-generate \
    --capture-isolate-op \
    --capture-sim-stock false \
    --capture-sim-cutter false \
    --capture-pane sim \
    --capture-view iso \
    --capture-focus "op:0:1:geometry"
for op_pair in \
    "0:op_01_idx0_rough.png" \
    "1:op_02_idx90_rough.png" \
    "2:op_03_idx180_rough.png" \
    "3:op_04_idx270_finish.png"; do
    op_idx="${op_pair%%:*}"
    op_file="${op_pair#*:}"
    run_capture "indexed-model-bounds" "$op_file" \
        --capture-generate \
        --capture-setup 0 \
        --capture-sim-after-op "$op_idx" \
        --capture-sim-stock true \
        --capture-sim-cutter true \
        --capture-pane sim \
        --capture-view iso \
        --capture-focus "op:0:${op_idx}:geometry"
done
fi

echo "==> Done! Captured $(find "$OUT_ROOT" -name '*.png' | wc -l) PNG images under $OUT_ROOT"
