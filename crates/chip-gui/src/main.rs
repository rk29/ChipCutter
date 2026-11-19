use std::path::PathBuf;

use chip_gui::{
    app::{ChipCutterApp, SelfTestConfig, StartupDemoMode},
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
    /// Optional `.chip.toml` project, `.chipz` bundle, `.dxf`/`.svg` vector, or `.step`/`.stp` model to load on startup.
    #[arg(value_name = "PATH")]
    step_file: Option<PathBuf>,

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
}

fn main() -> eframe::Result {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn")),
        )
        .init();

    let args = GuiArgs::parse();
    let self_test = args.self_test_screenshot.map(SelfTestConfig::new);
    let demo_mode = if args.welcome {
        StartupDemoMode::WelcomeScreen
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

    let is_self_test = self_test.is_some();
    let mut native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("ChipCutter — 2.5D & V-Carve CAM")
            .with_inner_size([1360.0, 840.0])
            .with_min_inner_size([900.0, 560.0]),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };

    #[cfg(target_os = "linux")]
    if is_self_test && std::env::var_os("DISPLAY").is_some() {
        // When running under `xvfb-run` in a host session that also exports `WAYLAND_DISPLAY`,
        // `winit` defaults to Wayland unless explicitly bound to X11, which would open visible
        // windows on the user's desktop instead of inside Xvfb.
        native_options.event_loop_builder = Some(Box::new(|builder| {
            winit::platform::x11::EventLoopBuilderExtX11::with_x11(builder);
        }));
    }

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
