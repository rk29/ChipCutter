use std::path::PathBuf;

use chip_gui::app::{ChipCutterApp, SelfTestConfig};
use clap::Parser;

#[derive(Debug, Parser)]
#[command(
    name = "chip-gui",
    version,
    about = "ChipCutter Desktop CAM Application"
)]
struct GuiArgs {
    /// Optional `.step` / `.stp` model to load on startup.
    #[arg(value_name = "STEP_FILE")]
    step_file: Option<PathBuf>,

    /// Render the sample part, capture a full-window PNG screenshot to this path, and exit.
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

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("ChipCutter — 2.5D CAM")
            .with_inner_size([1360.0, 840.0])
            .with_min_inner_size([900.0, 560.0]),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };

    eframe::run_native(
        "ChipCutter",
        native_options,
        Box::new(move |cc| Ok(Box::new(ChipCutterApp::new(cc, args.step_file, self_test)))),
    )
}
