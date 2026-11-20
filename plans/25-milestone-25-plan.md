# Milestone 25 Implementation Plan — Release Engineering, App Metadata, & GitHub Actions CI/Release Foundation

## Goal

Establish the shared cross-platform release engineering foundation required by Milestones 26–28: legal license files, vector & raster application icons, embedded GUI window icon and cross-platform compilation cleanliness, tuned release build profiles, installed-bundle runtime path safety, and the GitHub Actions CI & tag-triggered GitHub Releases workflows (`.github/workflows/ci.yml` and `.github/workflows/release.yml`).

---

## Proposed Changes

### 1. License Files, Release Profile, & Cross-Platform Cargo Polish

#### [NEW] `LICENSE-MIT`, `LICENSE-APACHE`
- Add standard `MIT` and `Apache-2.0` license texts at the repository root matching `[workspace.package].license = "MIT OR Apache-2.0"` in [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml), ready to be bundled into Flatpak `/app/share/licenses/`, Windows ZIP/MSI installers, and macOS `.app`/`.dmg` releases.

#### [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml)
- Add workspace package metadata (`description`, `repository = "https://github.com/rk29/ChipCutter"`, `homepage = "https://github.com/rk29/ChipCutter"`, `readme = "README.md"`) so `cargo` metadata, Windows version resources, and installers share a single source of truth.
- Add a tuned `[profile.release]` profile:
  - `lto = "thin"`
  - `codegen-units = 1`
  - `strip = "symbols"`
  - `panic = "abort"` (optional — evaluate if any test/CLI uses unwinding; keep default `unwind` if `chip-cli` or `eframe` relies on `catch_unwind`!).

#### [MODIFY] [crates/chip-gui/src/main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs)
- Fix cross-platform compiler/clippy warnings on Windows and macOS:
  - Gate `is_self_test` and `mut native_options` cleanly under `#[cfg(target_os = "linux")]` (or restructure so non-Linux targets compile with zero `unused_variables` / `unused_mut` warnings under `cargo clippy --workspace --all-targets -- -D warnings`).
- Update the main window title from `"ChipCutter — 2.5D & V-Carve CAM"` to `"ChipCutter — 2.5D, 3-Axis, & 4th-Axis CAM"`.
- Embed the application icon via `eframe::icon_data::from_png_bytes(include_bytes!(...))` (using `packaging/icons/chipcutter-256.png`) and set `.with_icon(...)` + `.with_app_id("io.github.rk29.ChipCutter")` on `egui::ViewportBuilder` so Wayland/Flatpak, X11, Windows taskbar, and macOS Dock display the crisp ChipCutter icon.
- Verify `chip-gui --version` and `chipcutter --version` both report the exact workspace SemVer (`CARGO_PKG_VERSION`).

---

### 2. Shared Application Icons & Packaging Asset Generator (`packaging/`)

#### [NEW] `packaging/icons/chipcutter.svg`
- Design a clean, high-contrast vector application icon (stylized CNC endmill flute + metallic chip arc on a dark slate badge) legible from `16×16` taskbar size up to `512×512` AppStream/macOS Retina display.

#### [NEW] `scripts/generate_release_icons.py` (or `scripts/generate_release_icons.sh`)
- Deterministic generator converting `packaging/icons/chipcutter.svg` into checked-in release icon artifacts:
  - `packaging/icons/hicolor/{64x64,128x128,256x256,512x512}/apps/chipcutter.png` (and `packaging/icons/chipcutter-256.png` embedded by `chip-gui`),
  - `packaging/windows/chipcutter.ico` (multi-resolution 16/32/48/64/128/256 px ICO),
  - `packaging/macos/ChipCutter.icns` (or generated cleanly in CI via `iconutil` from a `.iconset` directory).
- Check in the generated master assets (`chipcutter.svg`, `chipcutter-256.png`, `chipcutter.ico`, and PNG sizes) so downstream CI jobs on any OS have zero fragile vector-rasterizer dependencies.

---

### 3. Installed-Bundle Runtime Path Audit (`chip-post`, `chip-tools`, `chip-gui`)

#### [VERIFY / MODIFY] [crates/chip-post/src/profile.rs](file:///workspace/ChipCutter/crates/chip-post/src/profile.rs), [crates/chip-gui/src/prefs.rs](file:///workspace/ChipCutter/crates/chip-gui/src/prefs.rs), [crates/chip-tools/src/storage.rs](file:///workspace/ChipCutter/crates/chip-tools/src/storage.rs)
- Confirm built-in post-processors (`include_str!`) and starter tools work identically when `chip-gui` / `chipcutter` is launched with an arbitrary read-only working directory (e.g., `/`, `/app`, `C:\Program Files\ChipCutter`, or `/Applications/ChipCutter.app`).
- Verify `dirs::config_dir().join("chipcutter")` gracefully handles first-run directory creation on Linux/Flatpak (`$XDG_CONFIG_HOME/chipcutter`), Windows (`%APPDATA%\chipcutter`), and macOS (`~/Library/Application Support/chipcutter`).
- Add a unit test verifying `load_all_post_profiles` and `chipcutter list-posts` succeed when executed from an empty temporary working directory with no `./posts/` folder present.

---

### 4. GitHub Actions CI & Tag-Triggered Release Skeleton (`.github/workflows/`)

#### [NEW] `.github/workflows/ci.yml`
- Runs on `push` to `main` and `pull_request`:
  - **Lint & Audit job (`ubuntu-latest`):** `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo deny check`.
  - **Cross-Platform Build & Test Matrix (`ubuntu-latest`, `windows-latest`, `macos-latest`):**
    - Installs minimal OS build prerequisites (`libgtk-3-dev` on Ubuntu for `rfd`/`gtk3`).
    - Runs `cargo clippy --workspace --all-targets -- -D warnings` on all 3 OSes to guarantee zero platform-specific warnings.
    - Runs `cargo test --workspace --exclude chip-gui` (plus `chip-gui` unit tests) across Linux, Windows, and macOS.
  - Uses `Swatinem/rust-cache@v2` for fast incremental builds.

#### [NEW] `.github/workflows/release.yml`
- Triggered on `push.tags: ['v*.*.*']` and `workflow_dispatch` (with an optional `tag` / `dry_run` input so maintainers can test builds without publishing).
- **Stage 1 — `prepare`:** Validates tag SemVer matches `Cargo.toml` `[workspace.package].version` (or reports mismatch cleanly), extracts release version string (`VERSION=${GITHUB_REF_NAME#v}`), and configures matrix outputs.
- **Stage 2 — Platform Build Jobs (wired in M26, M27, M28):**
  - Placeholder/extensible job hooks (`linux-flatpak`, `windows-x64`, `macos-universal`) that upload build artifacts via `actions/upload-artifact@v4`.
- **Stage 3 — `publish-release`:**
  - Downloads all platform artifacts, generates a unified `SHA256SUMS.txt` across every release archive/installer/bundle, and creates/updates the GitHub Release via `softprops/action-gh-release@v2` (with `generate_release_notes: true` and `draft: true` when triggered manually or configurable prerelease detection for `-rc` / `-beta` tags).

---

## Verification Plan

1. **Local Workspace Verification:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check`
   - `WGPU_BACKEND=vulkan cargo test --workspace`
   - Launch `target/release/chip-gui --self-test-screenshot /tmp/m25-icon-check.png --sample cam-torture` from `/tmp` (outside the repo root!) under headless Vulkan to verify zero dependency on `./posts/` or repo working directory, and confirm `chip-gui --version` & `chipcutter --version`.
2. **Icon & Workflow Validation:**
   - Visually inspect `packaging/icons/chipcutter-256.png` via `read_file`.
   - Validate `.github/workflows/ci.yml` and `.github/workflows/release.yml` YAML syntax (`python3 -c 'import yaml; ...'` or `actionlint` if installed).
   - Confirm CI passes on GitHub Actions across `ubuntu-latest`, `windows-latest`, and `macos-latest` after pushing.
