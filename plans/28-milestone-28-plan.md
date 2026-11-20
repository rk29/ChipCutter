# Milestone 28 Implementation Plan — macOS App Bundle, Universal DMG, & Release Docs

## Goal

Package ChipCutter as a native **macOS Universal2 (`arm64` + `x86_64`)** application bundle (`ChipCutter.app`) inside a polished drag-to-Applications `.dmg` disk image (`ChipCutter-<version>-macos-universal.dmg`) built automatically in `.github/workflows/release.yml`, complete with Metal `wgpu` rendering, `.icns` Retina icons, `.chip.toml` / `.chipz` document type associations, bundled `chipcutter` CLI, optional Apple Developer ID signing & notarization hooks (with ad-hoc Apple Silicon fallback), and end-to-end installation documentation in `README.md`.

---

## Proposed Changes

### 1. macOS `.app` Bundle Metadata & Packaging Script (`packaging/macos/`, `scripts/`)

#### [NEW] `packaging/macos/Info.plist.in`
- Property list template populated with the release version (`@VERSION@`):
  - `CFBundleName`: `ChipCutter`
  - `CFBundleDisplayName`: `ChipCutter`
  - `CFBundleIdentifier`: `io.github.rk29.ChipCutter` (matching Milestone 26)
  - `CFBundleExecutable`: `chip-gui`
  - `CFBundleIconFile`: `ChipCutter.icns`
  - `CFBundlePackageType`: `APPL`
  - `CFBundleShortVersionString` / `CFBundleVersion`: `@VERSION@`
  - `LSMinimumSystemVersion`: `12.0`
  - `NSHighResolutionCapable`: `true` (Retina HiDPI viewport rendering)
  - `LSApplicationCategoryType`: `public.app-category.graphics-design`
  - `CFBundleDocumentTypes` + `UTExportedTypeDeclarations` registering `.chip.toml` and `.chipz` documents (plus viewer/import declarations for `.step`, `.stp`, `.stl`, `.obj`, `.3mf`, `.dxf`, `.svg`).

#### [NEW] `scripts/package_macos.sh`
- Deterministic macOS bundling script executed on `macos-latest`:
  1. Converts `packaging/icons/hicolor/*/apps/chipcutter.png` into a full Retina `ChipCutter.iconset` (`16`, `32`, `128`, `256`, `512` at `1x` & `2x`) and compiles `ChipCutter.icns` via `iconutil -c icns`.
  2. Combines `aarch64-apple-darwin` and `x86_64-apple-darwin` release binaries into Universal2 Mach-O executables via `lipo -create`:
     - `ChipCutter.app/Contents/MacOS/chip-gui`
     - `ChipCutter.app/Contents/MacOS/chipcutter`
  3. Renders `packaging/macos/Info.plist.in` into `ChipCutter.app/Contents/Info.plist` (validated with `plutil -lint`), copies `ChipCutter.icns`, `LICENSE-MIT`, and `LICENSE-APACHE` into `Contents/Resources/`.
  4. Signs `ChipCutter.app`:
     - If `MACOS_SIGN_IDENTITY` / certificate secrets are configured: signs with hardened runtime + timestamp (`codesign --force --options runtime --timestamp --sign "$MACOS_SIGN_IDENTITY"`), submits to Apple notarization (`xcrun notarytool submit --wait`), and staples (`xcrun stapler staple`).
     - Otherwise: applies clean ad-hoc signing (`codesign --force --deep --sign -`) required by Apple Silicon (`arm64`) Gatekeeper execution rules, and verifies with `codesign --verify --deep --strict`.
  5. Stages a clean DMG folder (`ChipCutter.app` + `/Applications` symlink + `LICENSE-MIT` / `LICENSE-APACHE`) and builds a compressed UDZO disk image `dist/ChipCutter-<version>-macos-universal.dmg` via `hdiutil create`.
  6. Also packages a lightweight Universal2 CLI tarball `dist/chipcutter-<version>-macos-universal.tar.gz` for Homebrew / terminal workflows.

---

### 2. GitHub Actions macOS Release Job & End-to-End Release Pipeline (`.github/workflows/release.yml`)

#### [MODIFY] `.github/workflows/release.yml`
- Wire the `macos-universal` job (`runs-on: macos-latest` — Apple Silicon):
  1. Installs Rust `stable` with both `aarch64-apple-darwin` and `x86_64-apple-darwin` targets and `Swatinem/rust-cache@v2`.
  2. Sets `MACOSX_DEPLOYMENT_TARGET=12.0` and builds `chip-gui` and `chip-cli` for both targets in `--release --locked`.
  3. Runs `scripts/package_macos.sh "$VERSION"` to assemble, sign (Developer ID if secrets present, otherwise ad-hoc), and package `ChipCutter-${VERSION}-macos-universal.dmg` + `chipcutter-${VERSION}-macos-universal.tar.gz`.
  4. Runs automated macOS verification checks:
     - `lipo -info ChipCutter.app/Contents/MacOS/chip-gui` and `chipcutter` (confirms both `arm64` and `x86_64` slices present),
     - `plutil -lint ChipCutter.app/Contents/Info.plist`,
     - `codesign --verify --deep --strict ChipCutter.app`,
     - End-to-end CLI smoke test using `ChipCutter.app/Contents/MacOS/chipcutter`: `--version`, `list-posts`, `generate-sample-step --tutorial 01-facing`, `validate --generate`, and `build --post grbl`.
  5. Uploads both artifacts for the `publish-release` job so `SHA256SUMS.txt` covers all Linux Flatpak, Windows ZIP/MSI, and macOS Universal DMG/CLI release assets.

---

### 3. Release Installation Documentation & Roadmap Completion (`README.md`, `specs/10-roadmap.md`)

#### [MODIFY] [README.md](file:///workspace/ChipCutter/README.md)
- Add a clear **Download & Install (Prebuilt Releases)** section near the top of `README.md` covering all three platforms:
  - **Linux (Flatpak):** `flatpak install --user ChipCutter-<version>-linux-x86_64.flatpak`, launching GUI (`flatpak run io.github.rk29.ChipCutter`) and headless CLI (`flatpak run --command=chipcutter io.github.rk29.ChipCutter ...`).
  - **Windows:** Installing via `ChipCutter-<version>-windows-x86_64.msi` or extracting portable `ChipCutter-<version>-windows-x86_64.zip` (plus a note on SmartScreen for unsigned community releases).
  - **macOS:** Dragging `ChipCutter.app` to `/Applications` from `ChipCutter-<version>-macos-universal.dmg`, optional CLI symlink (`ln -s /Applications/ChipCutter.app/Contents/MacOS/chipcutter /usr/local/bin/chipcutter`), and Gatekeeper instructions for ad-hoc signed releases.
  - **Checksum Verification:** Verifying downloaded artifacts against `SHA256SUMS.txt`.

#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Mark **Phase 7 — GitHub Releases & Cross-Platform Distribution (`M25`–`M28`)** as completed once all three release jobs pass end-to-end.

---

## Verification Plan

1. **Workspace & Script Audits:**
   - `bash -n scripts/package_macos.sh`
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check`
   - `WGPU_BACKEND=vulkan cargo test --workspace`
2. **Full Multi-OS Release Dry-Run (`workflow_dispatch` on GitHub Actions):**
   - Run `.github/workflows/release.yml` in dry-run mode across `linux-flatpak`, `windows-x64`, `macos-universal`, and `publish-release`.
   - Verify every expected artifact is produced and listed in `SHA256SUMS.txt`:
     - `ChipCutter-<version>-linux-x86_64.flatpak` & `chipcutter-<version>-linux-x86_64.tar.gz`
     - `ChipCutter-<version>-windows-x86_64.msi` & `ChipCutter-<version>-windows-x86_64.zip`
     - `ChipCutter-<version>-macos-universal.dmg` & `chipcutter-<version>-macos-universal.tar.gz`
     - `SHA256SUMS.txt`
   - Confirm macOS Universal2 `lipo -info`, `plutil -lint`, `codesign --verify`, and `chipcutter` CLI smoke test all pass cleanly.
