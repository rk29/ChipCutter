# Milestone 27 Implementation Plan — Windows Release Binaries & Installer

## Goal

Produce native, self-contained **Windows (`x86_64-pc-windows-msvc`)** release binaries (`chip-gui.exe` and `chipcutter.exe`), packaged as both a portable `.zip` archive and a proper Windows `.msi` installer with embedded application icons, Start Menu shortcuts, `PATH` registration for `chipcutter.exe`, and `.chip.toml` / `.chipz` file associations—built and uploaded automatically in `.github/workflows/release.yml`.

---

## Proposed Changes

### 1. Native Windows Executable Metadata & Subsystem Polish (`chip-gui`, `chip-cli`)

#### [MODIFY] [crates/chip-gui/src/main.rs](file:///workspace/ChipCutter/crates/chip-gui/src/main.rs)
- Add `#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]` so double-clicking `chip-gui.exe` on Windows launches cleanly without spawning a stray console window (while debug builds still print `tracing` logs to the terminal, and `chipcutter.exe` remains a standard console binary).
- Verify all `rfd::FileDialog` calls work cleanly with `rfd`'s `common-controls-v6` feature on Windows (`Vista+` native file dialogs).

#### [NEW] `crates/chip-gui/build.rs`, `crates/chip-cli/build.rs`
#### [MODIFY] [crates/chip-gui/Cargo.toml](file:///workspace/ChipCutter/crates/chip-gui/Cargo.toml), [crates/chip-cli/Cargo.toml](file:///workspace/ChipCutter/crates/chip-cli/Cargo.toml), [Cargo.toml](file:///workspace/ChipCutter/Cargo.toml)
- Add a Windows-only build dependency (`[target.'cfg(windows)'.build-dependencies] winresource = "0.1"`, MIT licensed — verified via `cargo deny check`) that embeds:
  - `packaging/windows/chipcutter.ico` into `chip-gui.exe` (Explorer icon, taskbar, Alt+Tab, Start Menu),
  - Windows PE `VERSIONINFO` resources (`ProductName = "ChipCutter"`, `FileDescription`, `FileVersion`, `ProductVersion`, `License = "MIT OR Apache-2.0"`) into both `chip-gui.exe` and `chipcutter.exe`.
- Evaluate `-C target-feature=+crt-static` (via `RUSTFLAGS` in the Windows release CI job) so the binaries run on fresh Windows 10/11 installations without requiring a separate Visual C++ Redistributable download; verify full `wgpu` DX12 + `monstertruck` + `rfd` compatibility under static CRT on `windows-latest`.

---

### 2. Windows Portable ZIP & WiX MSI Installer (`packaging/windows/`)

#### [NEW] `packaging/windows/chipcutter.wxs` (WiX Toolset Installer Definition)
- Clean WiX manifest (`WiX v4`/`v5` via `dotnet tool install --global wix` or preinstalled WiX on `windows-latest`) that installs:
  - `chip-gui.exe`, `chipcutter.exe`, `LICENSE-MIT`, `LICENSE-APACHE`, `README.md` into `[ProgramFiles64Folder]\ChipCutter\` (with stable `UpgradeCode` GUID for clean in-place upgrades via `MajorUpgrade`),
  - Start Menu shortcut (`ChipCutter`) pointing to `chip-gui.exe` with embedded icon,
  - System/User `PATH` append for `[INSTALLFOLDER]` so `chipcutter` works immediately in PowerShell / Windows Terminal,
  - Shell file associations (`ProgId`) for `.chip.toml` and `.chipz` opening directly in `chip-gui.exe "%1"`.

#### [NEW] `scripts/package_windows.ps1`
- Reproducible PowerShell packaging script invoked both locally on Windows and by GitHub Actions:
  1. Stages `target/x86_64-pc-windows-msvc/release/{chip-gui.exe,chipcutter.exe}` + `LICENSE-MIT`, `LICENSE-APACHE`, `README.md`.
  2. Creates portable archive `dist/ChipCutter-<version>-windows-x86_64.zip`.
  3. Compiles `packaging/windows/chipcutter.wxs` into `dist/ChipCutter-<version>-windows-x86_64.msi` (passing `-d Version=<version>`).
  4. Runs an optional Authenticode signing step (`signtool.exe`) when `WINDOWS_SIGN_PFX_BASE64` and `WINDOWS_SIGN_PASSWORD` secrets are present (cleanly skipped with an informational notice when unset).

---

### 3. GitHub Actions Windows Release Job (`.github/workflows/release.yml`)

#### [MODIFY] `.github/workflows/release.yml`
- Wire the `windows-x64` job (`runs-on: windows-latest`):
  1. Checks out repository and sets up Rust `stable` (`x86_64-pc-windows-msvc`) with `Swatinem/rust-cache@v2`.
  2. Builds `cargo build --release --locked --target x86_64-pc-windows-msvc -p chip-gui -p chip-cli`.
  3. Runs automated Windows smoke tests:
     - `chipcutter.exe --version`
     - `chipcutter.exe list-posts`
     - `chipcutter.exe generate-sample-step $env:TEMP\smoke.step --tutorial 01-facing --project-out $env:TEMP\smoke.chip.toml`
     - `chipcutter.exe validate --generate $env:TEMP\smoke.chip.toml`
     - `chipcutter.exe build $env:TEMP\smoke.chip.toml -o $env:TEMP\smoke_out --post grbl`
  4. Runs `scripts/package_windows.ps1 -Version $VERSION` to build both `ChipCutter-${VERSION}-windows-x86_64.zip` and `ChipCutter-${VERSION}-windows-x86_64.msi`.
  5. Verifies the `.msi` installer via silent administrative extract (`msiexec /a ... /qn TARGETDIR=...`) confirming `chip-gui.exe` and `chipcutter.exe` unpack intact.
  6. Uploads both artifacts for the `publish-release` job and `SHA256SUMS.txt`.

---

## Verification Plan

1. **Dependency & Workspace Gates (Linux + Cross-Check):**
   - `cargo fmt --all -- --check`
   - `cargo clippy --workspace --all-targets -- -D warnings`
   - `cargo deny check` (verifying `winresource` and its transitive tree are strictly permissive with zero advisories)
   - `WGPU_BACKEND=vulkan cargo test --workspace`
2. **Windows CI & Installer Smoke Tests (`windows-latest`):**
   - Trigger `.github/workflows/ci.yml` and `.github/workflows/release.yml` (`workflow_dispatch` dry-run) on `windows-latest`.
   - Confirm zero compiler/clippy warnings on MSVC, `chipcutter.exe` end-to-end STEP -> G-code smoke test passes, `ChipCutter-<version>-windows-x86_64.zip` contains both executables + licenses, and `ChipCutter-<version>-windows-x86_64.msi` extracts cleanly via `msiexec /a`.
