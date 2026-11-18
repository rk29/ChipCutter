# ChipCutter Development & Build Environment Guide

This document describes the toolchain, native libraries, and setup steps required to build, test, and run **ChipCutter** (`chip-gui` and `chip-cli`) from source across Linux, macOS, Windows, and headless CI containers.

---

## 1. Requirements Overview

### Rust Toolchain
- **Rust `1.85+` stable** (ChipCutter uses **Rust Edition 2024** and Cargo workspace resolver `"2"`).
- **`rustfmt`** & **`clippy`**: Enforced across all crates (`#!-deny` on `unwrap_used` and `expect_used` in library code, `-D warnings` in CI).
- **`cargo-deny`**: Enforces ChipCutter's permissive-only dependency policy (`MIT`, `Apache-2.0`, `BSD`, `MPL-2.0`, `ISC`, `Zlib`, `Unicode-3.0`) defined in `deny.toml`.

### Pure-Rust CAD & Geometry Kernel (No C++ CAD Dependencies)
Unlike CAM tools built on C++ OpenCASCADE, ChipCutter's STEP B-Rep kernel (`monstertruck-*`) and arc-preserving 2D offsetter (`cavalier_contours`) are **100% pure Rust**. You do **not** need CMake, Ninja, OCCT, or CGAL installed.

### Native Graphics, Windowing, & System Dependencies
Only the `chip-gui` desktop shell (`eframe` / `winit` / `wgpu` / `rfd`) requires platform windowing and GPU libraries:

| Dependency Category | Purpose in ChipCutter | Linux | macOS | Windows |
|---|---|---|---|---|
| **C Linker & `pkg-config`** | Linking Rust binaries & querying native library flags | `build-essential`, `pkg-config` | Xcode Command Line Tools | MSVC Build Tools |
| **Windowing & Input** | Creating desktop windows, cursor icons, HiDPI, keyboard layouts (`eframe`/`winit`) | `libx11-dev`, `libxcb1-dev`, `libxcursor-dev`, `libxi-dev`, `libxrandr-dev`, `libxkbcommon-dev`, `libxkbcommon-x11-dev`, `libwayland-dev` | Built into Cocoa / AppKit | Built into Win32 |
| **GPU Rendering (`wgpu 30`)** | 4x MSAA 3D CAD B-Rep & toolpath viewport | `libvulkan-dev` + Vulkan ICD driver | Built into Metal | Built into DirectX 12 / Vulkan |
| **Native File Dialogs (`rfd`)** | Opening `.step` files and saving `.chip.toml` / `.nc` files | `libgtk-3-dev` (or XDG Desktop Portal) | Built into AppKit | Built into Common Item Dialog |
| **Headless GPU Testing *(Optional / CI)*** | Running offscreen `wgpu` tests & GUI screenshots without a physical GPU/monitor | `mesa-vulkan-drivers` (**Lavapipe** software Vulkan) + `xvfb` | Not needed on desktop | WARP DX12 software adapter |

---

## 2. Platform Setup Instructions

### Ubuntu (24.04 LTS / 26.04 LTS) & Debian (12+)
```bash
# 1. Install compiler toolchain, X11/Wayland/Vulkan headers, GTK3 file-dialog headers,
#    and Mesa Lavapipe + Xvfb for headless GPU testing
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
  ca-certificates curl git build-essential pkg-config libssl-dev \
  libx11-dev libxcb1-dev libxcursor-dev libxi-dev libxrandr-dev \
  libxkbcommon-dev libxkbcommon-x11-dev libwayland-dev libvulkan-dev libgtk-3-dev \
  mesa-vulkan-drivers xvfb

# 2. Install Rust stable toolchain via rustup with rustfmt and clippy
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
  sh -s -- -y --profile default --component rustfmt,clippy
. "$HOME/.cargo/env"

# 3. Install cargo-deny for license and advisory verification
cargo install --locked cargo-deny
```

### Fedora (40+) / RHEL-compatible
```bash
sudo dnf install -y \
  curl git gcc gcc-c++ make pkgconf-pkg-config openssl-devel \
  libX11-devel libxcb-devel libXcursor-devel libXi-devel libXrandr-devel \
  libxkbcommon-devel libxkbcommon-x11-devel wayland-devel \
  vulkan-loader-devel mesa-vulkan-drivers gtk3-devel xorg-x11-server-Xvfb

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
  sh -s -- -y --profile default --component rustfmt,clippy
. "$HOME/.cargo/env"

cargo install --locked cargo-deny
```

### Arch Linux
```bash
sudo pacman -S --needed \
  base-devel git curl pkgconf openssl \
  libx11 libxcb libxcursor libxi libxrandr libxkbcommon wayland \
  vulkan-icd-loader vulkan-swrast gtk3 xorg-server-xvfb

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
  sh -s -- -y --profile default --component rustfmt,clippy
. "$HOME/.cargo/env"

cargo install --locked cargo-deny
```

### macOS (Apple Silicon & Intel)
macOS includes AppKit and **Metal** out of the box—no X11, Wayland, or Vulkan packages are required:
```bash
# 1. Install Xcode Command Line Tools
xcode-select --install

# 2. Install Rust stable toolchain + components
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
  sh -s -- -y --profile default --component rustfmt,clippy
. "$HOME/.cargo/env"

# 3. Install cargo-deny
cargo install --locked cargo-deny
```

### Windows 10 / 11
1. Install **Visual Studio Build Tools 2022+** (select workload **Desktop development with C++**, which provides `link.exe` and the Windows 10/11 SDK for DirectX 12).
2. Download and run [`rustup-init.exe`](https://rustup.rs) using the default `x86_64-pc-windows-msvc` target, then ensure components and `cargo-deny` are installed in PowerShell:
   ```powershell
   rustup component add rustfmt clippy
   cargo install --locked cargo-deny
   ```

---

## 3. Verifying Your Environment & Common Workflows

From `/workspace/ChipCutter` (or your cloned `ChipCutter` repository root):

```bash
# Verify toolchain versions
rustc --version
cargo --version
cargo deny --version

# Check permissive license compliance across the entire dependency graph
cargo deny check licenses

# Check formatting and strict Clippy lints
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

# Run all unit, STEP round-trip, picking, and offscreen GPU render tests
cargo test --workspace

# Run the headless CLI (no window server or GPU required):
#   STEP generation, inspection, validation, heightmap simulation, G-code build, & .chipz bundling
cargo run -p chip-cli -- generate-sample-step /tmp/torture.step --cam-torture --project-out /tmp/torture.chip.toml
cargo run -p chip-cli -- inspect /tmp/torture.step
cargo run -p chip-cli -- validate --generate /tmp/torture.chip.toml
cargo run -p chip-cli -- simulate /tmp/torture.chip.toml --resolution fine --report /tmp/sim_report.json
cargo run -p chip-cli -- build /tmp/torture.chip.toml -o /tmp/gcode_out --post linuxcnc
cargo run -p chip-cli -- bundle pack /tmp/torture.chip.toml -o /tmp/torture.chipz
cargo run -p chip-cli -- build /tmp/torture.chipz -o /tmp/from_bundle --post grbl

# Launch the interactive desktop CAM application (optional .chip.toml, .chipz, or .step path)
cargo run -p chip-gui
```

### Running GUI / Viewport Checks in Headless Linux Environments (CI / Containers)
When no physical monitor or `DISPLAY` is attached on Linux, Mesa's `mesa-vulkan-drivers` package supplies **Lavapipe** (a CPU-based Vulkan 1.3 implementation) and `xvfb-run` supplies a virtual X11 display:

```bash
# Offscreen wgpu unit tests work directly without X11:
WGPU_BACKEND=vulkan cargo test -p chip-gui

# Full eframe window launch + automated screenshot self-test under Xvfb:
WGPU_BACKEND=vulkan xvfb-run -a cargo run -p chip-gui -- --self-test-screenshot /tmp/preview.png
```

---

## 4. Troubleshooting
- **`pkg-config` could not find `wayland-client` / `xkbcommon` / `gtk+-3.0`:** Double-check that the `-dev` (Debian/Ubuntu) or `-devel` (Fedora) packages from Section 2 are installed. Notice that building *only* the headless CLI (`cargo build -p chip-cli`) skips all GUI/windowing crates completely.
- **`wgpu` fails to find a suitable GPU adapter in a VM or container:** Install `mesa-vulkan-drivers` (Ubuntu/Debian) or `vulkan-swrast` (Arch) and export `WGPU_BACKEND=vulkan` so `wgpu` selects the Lavapipe software Vulkan adapter.
