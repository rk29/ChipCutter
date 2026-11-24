#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::path::PathBuf;

#[cfg(all(target_os = "linux", feature = "gtk3"))]
mod linux_gsettings;

use chip_gui::{
    app::{ChipCutterApp, SelfTestConfig, StartupDemoMode},
    capture::{
        CaptureFocus, CaptureModal, CapturePane, CaptureSpec, CaptureView,
        parse_capture_post_profile, parse_capture_sim_backend, parse_capture_sim_engine,
        parse_capture_sim_resolution,
    },
    panels::welcome_gallery::SampleProjectId,
    theme::AppTheme,
};
use clap::Parser;

#[derive(Debug, Parser)]
#[command(
    name = "chip-gui",
    version,
    about = "ChipCutter Desktop CAM Application"
)]
struct GuiArgs {
    /// Optional `.chip.toml` project, `.chipz` bundle, `.dxf`/`.svg` vector, `.stl`/`.obj`/`.3mf` mesh, or `.step`/`.stp` model to load on startup.
    #[arg(value_name = "PATH")]
    step_file: Option<PathBuf>,

    /// Launch directly into any built-in sample project (`cam-torture`, `adaptive-rest`, `pure-2d`, `nameplate`, `vcarve`, `multi-feature`, `tilted-bracket`, `revision-v1`, `revision-v2`, `sample-plate`, `mesh-dome`, `mesh-wave`, `rotary-shaft`, `rotary-wrap`, `rotary-4axis`, `thread-mill`, `two-sided-vise`, `undercut-deburr`, `feed-opt-pocket`, `flip-jig-softjaws`, `curved-deburr-3d`, `dovetail-rail-keyway`, `svg-badge`, `mesh-3mf-bracket`, `keepout-clamps`, `indexed-model-bounds`).
    #[arg(long, value_parser = SampleProjectId::from_cli_slug)]
    sample: Option<SampleProjectId>,

    /// Launch directly into the Phase 5 Sculpted Dome (STL Mesh) sample preview.
    #[arg(long)]
    demo_mesh_dome: bool,

    /// Launch directly into the 3D STEP CAM Torture Block demo job.
    #[arg(long)]
    demo_torture: bool,

    /// Launch directly into the Milestone 7 Pure-2D CNC Router Front Panel (.dxf) demo job.
    #[arg(long)]
    demo_pure_2d: bool,

    /// Launch directly into the Milestone 8 Engraved Nameplate & CAM Typography demo job.
    #[arg(long)]
    demo_nameplate: bool,

    /// Launch directly into the Milestone 9 V-Carve Sign & Mated V-Inlay Plug demo job.
    #[arg(long)]
    demo_vcarve: bool,

    /// Force-open the Welcome & Quick-Start Overlay on startup (useful with `--self-test-screenshot`).
    #[arg(long)]
    welcome: bool,

    /// Optional workspace color theme override (`dark` or `light`); defaults to `prefs.toml` (`dark`).
    #[arg(long)]
    theme: Option<String>,

    /// Render the workspace, capture a full-window PNG screenshot to this path, and exit.
    #[arg(long)]
    self_test_screenshot: Option<PathBuf>,

    /// Select a specific Setup (0-indexed) before generating & simulating for screenshot capture.
    #[arg(long)]
    capture_setup: Option<usize>,

    /// Headless capture camera view (`iso`, `top`, `bottom`, `front`, `back`, `right`, `left`, `setup-top`, `sketch`).
    #[arg(long, value_parser = CaptureView::parse)]
    capture_view: Option<CaptureView>,

    /// Headless capture inspector focus (`setup:<wcs|stock|heights>`, `op:<idx>:<tab>`, `sketch:<idx>`).
    #[arg(long, value_parser = CaptureFocus::parse)]
    capture_focus: Option<CaptureFocus>,

    /// Headless capture bottom workspace tab (`sim`, `gcode`, `diagnostics`).
    #[arg(long, value_parser = CapturePane::parse)]
    capture_pane: Option<CapturePane>,

    /// Headless capture modal overlay (`welcome`, `samples`, `tools`, `shortcuts`, `about`).
    #[arg(long, value_parser = CaptureModal::parse)]
    capture_modal: Option<CaptureModal>,

    /// Hide other setup operations' toolpaths so only the focused operation is displayed during capture.
    #[arg(long)]
    capture_isolate_op: bool,

    /// Show or hide the 3D simulated machined stock mesh during capture (`true` / `false`).
    #[arg(long)]
    capture_sim_stock: Option<bool>,

    /// Show or hide the animated 3D cutter & holder assembly during capture (`true` / `false`).
    #[arg(long)]
    capture_sim_cutter: Option<bool>,

    /// Scrub simulation timeline progress in `[0.0, 1.0]` during capture (default `0.82`).
    #[arg(long)]
    capture_sim_progress: Option<f64>,

    /// Scrub simulation timeline to the exact completion timestamp of operation `OP_IDX` (0-indexed).
    #[arg(long)]
    capture_sim_after_op: Option<usize>,

    /// Select simulation engine during capture (`heightmap` or `tri-dexel`).
    #[arg(long, value_parser = parse_capture_sim_engine)]
    capture_sim_engine: Option<chip_sim::SimEngineKind>,

    /// Select simulation compute backend during capture (`auto`, `gpu`, or `cpu`).
    #[arg(long, value_parser = parse_capture_sim_backend)]
    capture_sim_backend: Option<chip_sim::SimComputeBackendChoice>,

    /// Select simulation resolution during capture (`standard`, `fine`, or `ultra`).
    #[arg(long, value_parser = parse_capture_sim_resolution)]
    capture_sim_resolution: Option<chip_sim::SimResolution>,

    /// Enable signed stock-vs-target error heatmap coloring during capture.
    #[arg(long)]
    capture_sim_heatmap: bool,

    /// Override active setup post-processor profile for G-Code Preview capture (`grbl`, `grblhal_atc`, `linuxcnc`).
    #[arg(long, value_parser = parse_capture_post_profile)]
    capture_post: Option<String>,

    /// Auto-dispatch toolpath generation (`Ctrl+G`) and simulation after loading a project before capture.
    #[arg(long)]
    capture_generate: bool,

    /// Show or hide Z-height reference planes during capture (`true` / `false`).
    #[arg(long)]
    capture_z_planes: Option<bool>,

    /// Show or hide 27-point WCS origin dots during capture (`true` / `false`).
    #[arg(long)]
    capture_27pt_dots: Option<bool>,

    /// Show or hide translucent 3D workholding fixtures during capture (`true` / `false`).
    #[arg(long)]
    capture_fixtures: Option<bool>,

    /// Show or hide translucent ghost-part comparison during capture (`true` / `false`).
    #[arg(long)]
    capture_ghost_part: Option<bool>,

    /// Also save a 480px-wide Lanczos3 thumbnail PNG of the captured screenshot to this path.
    #[arg(long)]
    capture_thumb: Option<PathBuf>,
}

const APP_ID: &str = "io.github.rk29.ChipCutter";
const APP_ICON_PNG: &[u8] = include_bytes!("../../../packaging/icons/chipcutter-256.png");

fn load_app_icon() -> Option<egui::IconData> {
    let img = image::load_from_memory_with_format(APP_ICON_PNG, image::ImageFormat::Png).ok()?;
    let rgba = img.to_rgba8();
    let (width, height) = rgba.dimensions();
    Some(egui::IconData {
        rgba: rgba.into_raw(),
        width,
        height,
    })
}

fn main() -> eframe::Result {
    #[cfg(all(target_os = "linux", feature = "gtk3"))]
    linux_gsettings::configure_gsettings_backend_for_gtk();

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn")),
        )
        .init();

    let args = GuiArgs::parse();
    let capture_spec = CaptureSpec {
        setup_idx: args.capture_setup,
        view: args.capture_view,
        focus: args.capture_focus,
        pane: args.capture_pane,
        modal: args.capture_modal,
        isolate_op: args.capture_isolate_op,
        show_sim_stock: args.capture_sim_stock,
        show_sim_cutter: args.capture_sim_cutter,
        sim_progress: args.capture_sim_progress,
        sim_after_op: args.capture_sim_after_op,
        sim_engine: args.capture_sim_engine,
        sim_backend: args.capture_sim_backend,
        sim_resolution: args.capture_sim_resolution,
        sim_heatmap: args.capture_sim_heatmap,
        post_profile: args.capture_post,
        auto_generate: args.capture_generate,
        show_z_planes: args.capture_z_planes,
        show_27pt_dots: args.capture_27pt_dots,
        show_fixtures: args.capture_fixtures,
        ghost_part: args.capture_ghost_part,
        thumb_path: args.capture_thumb,
    };
    let self_test = args
        .self_test_screenshot
        .map(|path| SelfTestConfig::with_capture(path, capture_spec));
    let demo_mode = if args.welcome {
        StartupDemoMode::WelcomeScreen
    } else if let Some(sample_id) = args.sample {
        StartupDemoMode::Sample(sample_id)
    } else if args.demo_mesh_dome {
        StartupDemoMode::Sample(SampleProjectId::SculptedMeshDome)
    } else if args.demo_vcarve {
        StartupDemoMode::VCarveSign
    } else if args.demo_nameplate {
        StartupDemoMode::EngravedNameplate
    } else if args.demo_pure_2d {
        StartupDemoMode::Pure2dPanel
    } else if args.demo_torture {
        StartupDemoMode::CamTorture
    } else {
        StartupDemoMode::None
    };

    let theme_override = args.theme.as_deref().map(|t| {
        if t.eq_ignore_ascii_case("light") {
            AppTheme::Light
        } else {
            AppTheme::Dark
        }
    });

    let mut viewport = egui::ViewportBuilder::default()
        .with_app_id(APP_ID)
        .with_title(format!(
            "ChipCutter v{} — 2.5D, 3-Axis, & 4th-Axis CAM",
            chip_gui::panels::about::APP_VERSION
        ))
        .with_inner_size([1360.0, 840.0])
        .with_min_inner_size([900.0, 560.0]);
    if let Some(icon) = load_app_icon() {
        viewport = viewport.with_icon(std::sync::Arc::new(icon));
    }

    #[cfg(target_os = "linux")]
    let event_loop_builder: Option<eframe::EventLoopBuilderHook> =
        if self_test.is_some() && std::env::var_os("DISPLAY").is_some() {
            // When running under `xvfb-run` in a host session that also exports `WAYLAND_DISPLAY`,
            // `winit` defaults to Wayland unless explicitly bound to X11, which would open visible
            // windows on the user's desktop instead of inside Xvfb.
            Some(Box::new(|builder| {
                winit::platform::x11::EventLoopBuilderExtX11::with_x11(builder);
            }))
        } else {
            None
        };
    #[cfg(not(target_os = "linux"))]
    let event_loop_builder: Option<eframe::EventLoopBuilderHook> = None;

    let native_options = eframe::NativeOptions {
        viewport,
        renderer: eframe::Renderer::Wgpu,
        event_loop_builder,
        ..Default::default()
    };

    eframe::run_native(
        "ChipCutter",
        native_options,
        Box::new(move |cc| {
            Ok(Box::new(ChipCutterApp::new_with_options(
                cc,
                args.step_file,
                demo_mode,
                theme_override,
                self_test,
            )))
        }),
    )
}
