# Milestone 26 Implementation Plan — Linux Flatpak Packaging & Release Bundle

## Goal

Package ChipCutter (`chip-gui` + `chipcutter` CLI) as a sandboxed, portal-native **Linux Flatpak** (`ChipCutter-<version>-linux-x86_64.flatpak`) built automatically in GitHub Actions and attached to every GitHub Release, complete with Wayland/X11 + Vulkan GPU support, native `xdg-desktop-portal` file dialogs, `.desktop` integration, MIME associations for `.chip.toml` and `.chipz`, and validated AppStream metadata.

---

## Proposed Changes

### 1. Flatpak Sandbox & `xdg-desktop-portal` File Dialog Support (`Cargo.toml`, `chip-gui`)

#### [MODIFY] [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml), [crates/chip-gui/Cargo.toml](file:///workspace/ChipCutter/crates/chip-gui/Cargo.toml)
- Currently `rfd` is configured with `features = ["gtk3", "common-controls-v6"]`, which uses in-process `GtkFileChooserDialog`. Inside a strict Flatpak sandbox (without `--filesystem=host`), `GtkFileChooserDialog` cannot traverse the host filesystem via the Flatpak Document Portal.
- Add a clean portal-ready configuration for Linux Flatpak builds:
  - Option A (preferred): Enable `rfd`'s pure-Rust `xdg-portal` feature via an explicit Cargo feature on `chip-gui` (e.g., `flatpak-portal = ["rfd/xdg-portal"]`, or switch default Linux `rfd` to `xdg-portal` if all transitive crates pass `cargo deny check` with permissive MIT/Apache-2.0 licenses!).
  - Verify `cargo deny check` passes with zero license/advisory issues when the portal feature is enabled, and keep `linux_gsettings` cleanly gated so GTK3-specific workarounds only run when `gtk3` is active.
- Ensure `egui::ViewportBuilder::with_app_id(...)` matches the exact Flatpak reverse-DNS App ID so GNOME/KDE/Sway match the `.desktop` launcher and window icon natively on Wayland.

---

### 2. Flatpak Manifest, Desktop Entry, MIME Types, & AppStream Metadata (`packaging/flatpak/`)

> **App ID:** `io.github.rk29.ChipCutter` (matching `https://github.com/rk29/ChipCutter`).

#### [NEW] `packaging/flatpak/io.github.rk29.ChipCutter.yml`
- Base runtime: `org.freedesktop.Platform` `24.08` (or `org.gnome.Platform` `47` if GTK3 fallback is retained) + `org.freedesktop.Sdk` + `org.freedesktop.Sdk.Extension.rust-stable` (matching Rust Edition 2024 toolchain requirements).
- Sandbox `finish-args`:
  - `--share=ipc`
  - `--socket=wayland`
  - `--socket=fallback-x11`
  - `--device=dri` (Vulkan / OpenGL GPU acceleration for `wgpu`)
  - Note: No `--filesystem=host` required once `xdg-desktop-portal` file dialogs are enabled!
- Build module installs into `/app`:
  - `/app/bin/chip-gui`
  - `/app/bin/chipcutter` (headless CLI accessible via `flatpak run --command=chipcutter io.github.rk29.ChipCutter ...`)
  - `/app/share/applications/io.github.rk29.ChipCutter.desktop`
  - `/app/share/metainfo/io.github.rk29.ChipCutter.metainfo.xml`
  - `/app/share/mime/packages/io.github.rk29.ChipCutter.xml`
  - `/app/share/icons/hicolor/{scalable,64x64,128x128,256x256,512x512}/apps/io.github.rk29.ChipCutter.{svg,png}`
  - `/app/share/licenses/io.github.rk29.ChipCutter/{LICENSE-MIT,LICENSE-APACHE}`
- Provide a companion helper script `scripts/build_flatpak.sh` that builds the Flatpak locally or in CI, runs `flatpak build-export`, and produces the single-file bundle `ChipCutter-<version>-linux-x86_64.flatpak`.
  - Support both GitHub Release CI builds (using `build-args: ["--share=network"]` or pre-vendored `cargo vendor` directory) so the manifest is self-contained and reproducible.

#### [NEW] `packaging/flatpak/io.github.rk29.ChipCutter.desktop`
- Standard FreeDesktop desktop entry (`Categories=Graphics;3DGraphics;Engineering;Science;`, `Exec=chip-gui %f`, `Icon=io.github.rk29.ChipCutter`, `Terminal=false`, `StartupWMClass=io.github.rk29.ChipCutter`, `MimeType=application/x-chipcutter-project;application/x-chipcutter-bundle;model/step;model/stl;model/obj;image/vnd.dxf;image/svg+xml;`).

#### [NEW] `packaging/flatpak/io.github.rk29.ChipCutter.xml` (Shared MIME Info)
- Registers `*.chip.toml` (`application/x-chipcutter-project`) and `*.chipz` (`application/x-chipcutter-bundle`) so double-clicking ChipCutter projects in Nautilus/Dolphin launches `chip-gui`.

#### [NEW] `packaging/flatpak/io.github.rk29.ChipCutter.metainfo.xml`
- Complete AppStream 1.0 metadata: summary, description, MIT/Apache-2.0 project license (`metadata_license: CC0-1.0`), URLs pointing to `https://github.com/rk29/ChipCutter`, screenshots referencing repo tutorial/hero captures, `<provides><binary>chip-gui</binary><binary>chipcutter</binary></provides>`, `<launchable type="desktop-id">io.github.rk29.ChipCutter.desktop</launchable>`, `<content_rating type="oars-1.1" />`, and `<releases>` block updated with release version/date.

---

### 3. GitHub Actions Linux Flatpak Release Job (`.github/workflows/release.yml`)

#### [MODIFY] `.github/workflows/release.yml`
- Wire the `linux-flatpak` job (`runs-on: ubuntu-latest`):
  1. Installs `flatpak`, `flatpak-builder`, `appstream`, and `desktop-file-utils`.
  2. Validates desktop & AppStream metadata (`desktop-file-validate`, `appstreamcli validate --pedantic --no-net`).
  3. Builds the Flatpak repo with `flatpak-builder` (caching `.flatpak-builder` and Cargo vendor/target artifacts).
  4. Runs a smoke test inside the built Flatpak sandbox:
     - `flatpak run --command=chipcutter io.github.rk29.ChipCutter --version`
     - `flatpak run --command=chipcutter io.github.rk29.ChipCutter list-posts`
     - `flatpak run --command=chipcutter io.github.rk29.ChipCutter generate-sample-step ...` + `validate --generate` inside a temp directory.
  5. Exports single-file bundle `ChipCutter-${VERSION}-linux-x86_64.flatpak` (with `--runtime-repo=https://flathub.org/repo/flathub.flatpakrepo` so `flatpak install ChipCutter-*.flatpak` automatically pulls the runtime on end-user machines!).
  6. Also packages a lightweight headless CLI tarball `chipcutter-${VERSION}-linux-x86_64.tar.gz` (`chipcutter` + licenses) for server/CI Linux users who don't want Flatpak.
  7. Uploads both artifacts for the `publish-release` job and `SHA256SUMS.txt`.

---

## Verification Plan

1. **Metadata & Portal Audits:**
   - `desktop-file-validate packaging/flatpak/*.desktop`
   - `appstreamcli validate --no-net packaging/flatpak/*.metainfo.xml`
   - `cargo deny check` (confirming any `rfd/xdg-portal` dependency additions remain strictly permissive).
2. **Workspace & Build Verification:**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings` (both default features and `--features flatpak-portal` if feature-gated)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
3. **Flatpak Bundle Smoke Test:**
   - Run `scripts/build_flatpak.sh` locally (if `flatpak-builder` is available) and via GitHub Actions `workflow_dispatch` dry-run on `ubuntu-latest`.
   - Verify `flatpak install --user` of the generated `.flatpak` bundle succeeds cleanly, `flatpak run --command=chipcutter io.github.rk29.ChipCutter --version` and `list-posts` succeed, and `flatpak run io.github.rk29.ChipCutter --self-test-screenshot ...` renders cleanly.
