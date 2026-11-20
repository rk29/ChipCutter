# Milestone 34 Implementation Plan — CI/CD Hardening, Packaging Semver Validation, & Security Policy

## Goal

Harden ChipCutter's GitHub Actions workflows and release packaging scripts against expression injection, over-broad token permissions, and mutable third-party action tags, and publish a concise `SECURITY.md` documenting ChipCutter's threat model and vulnerability reporting process.

---

## Security Findings Addressed

- **Finding #6 (Medium):** GitHub Actions expression injection (`${{ inputs.tag }}` / `${{ needs.prepare.outputs.version }}` in `run:` scripts), workflow-wide `permissions: contents: write`, mutable third-party action tags (`@stable`, `@v2`, `@v3`), and unvalidated `VERSION` interpolation into `sed` in `scripts/package_macos.sh`.

---

## Proposed Changes

### 1. Harden Release & CI Workflows (`.github/workflows/`)

#### [MODIFY] [.github/workflows/release.yml](file:///workspace/ChipCutter/.github/workflows/release.yml)
- **Least-Privilege `permissions:`:**
  - Change top-level workflow permissions to `permissions: contents: read`.
  - Add `permissions: contents: write` **only** on the `publish-release` job so Linux, Windows, and macOS build jobs (which hold code-signing secrets) cannot mutate repository contents or releases.
- **Eliminate Script Expression Injection:**
  - In `prepare`: pass `INPUT_TAG: ${{ inputs.tag }}` and `INPUT_DRY_RUN: ${{ inputs.dry_run }}` via step `env:` instead of inline `${{ ... }}` interpolation in Bash.
  - Enforce strict semver validation on `TAG_VERSION` in `prepare`:
    `[[ "${TAG_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$ ]]` (exiting non-zero otherwise).
  - In `windows-x64` (`Package Windows ZIP and MSI installer` & `Verify MSI installer administrative extract`): pass `RELEASE_VERSION: ${{ needs.prepare.outputs.version }}` via `env:` and reference `$env:RELEASE_VERSION` in PowerShell.
  - In `macos-universal` (`Package macOS Universal2 .app, .dmg, & CLI archive`): pass `RELEASE_VERSION: ${{ needs.prepare.outputs.version }}` via `env:` and call `./scripts/package_macos.sh "${RELEASE_VERSION}" --out-dir dist`.
- **Immutable Commit-SHA Pinning:**
  - Pin all actions in `release.yml` (`actions/checkout`, `dtolnay/rust-toolchain`, `Swatinem/rust-cache`, `actions/upload-artifact`, `actions/download-artifact`, `softprops/action-gh-release`) to full 40-character commit SHAs with trailing `# vX.Y.Z` comments.

#### [MODIFY] [.github/workflows/ci.yml](file:///workspace/ChipCutter/.github/workflows/ci.yml)
- Add explicit top-level `permissions: contents: read`.
- Pin third-party actions (`actions/checkout`, `dtolnay/rust-toolchain`, `Swatinem/rust-cache`, `EmbarkStudios/cargo-deny-action`) to full 40-character commit SHAs with trailing version comments.
- Add `cargo deny --no-default-features --features chip-gui/flatpak-portal check` alongside the default `cargo deny check` so Flatpak portal dependencies are continuously audited in CI as well.

---

### 2. Packaging Script Input Validation (`scripts/`)

#### [MODIFY] [scripts/package_macos.sh](file:///workspace/ChipCutter/scripts/package_macos.sh)
- Validate `VERSION` immediately after stripping leading `v` against `^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$` so characters like `|`, `&`, `/`, or whitespace can never corrupt `sed "s|@VERSION@|${VERSION}|g"` or artifact paths.

#### [MODIFY] [scripts/build_flatpak.sh](file:///workspace/ChipCutter/scripts/build_flatpak.sh)
- Apply the same strict semver validation to `VERSION` before constructing `${BUNDLE_PATH}` and invoking `flatpak-builder`.

---

### 3. Security Policy & Threat Model Documentation

#### [NEW] `SECURITY.md`
- Document:
  - Supported versions and how to report security or physical-safety vulnerabilities privately.
  - ChipCutter's trust boundaries for untrusted CAM files (`.chipz`, `.chip.toml`, `.machine.toml`, `.post.toml`, `.tools`, `.step`, `.stl`, `.obj`, `.3mf`, `.dxf`, `.svg`).
  - Operator CNC safety guidance when inspecting custom `.post.toml` profiles bundled in third-party `.chipz` archives.

#### [MODIFY] [specs/10-roadmap.md](file:///workspace/ChipCutter/specs/10-roadmap.md)
- Mark Phase 9 (`M32`–`M34`) completed once all three milestones pass verification.

---

## Verification Plan

1. **Workflow Static Audit:**
   - Verify zero direct `${{ inputs.* }}` or `${{ needs.*.outputs.* }}` interpolations inside `run:` blocks in `.github/workflows/*.yml`.
   - Verify top-level `permissions: contents: read` on both `ci.yml` and `release.yml`, with `contents: write` scoped solely to `publish-release`.
   - Verify every `uses:` entry in `.github/workflows/*.yml` is pinned to a 40-hex-character commit SHA.
   - Validate YAML syntax (`python3 -c 'import yaml; ...'`) and shell syntax (`bash -n scripts/package_macos.sh scripts/build_flatpak.sh`).
2. **Script Rejection Checks:**
   - Test that `scripts/build_flatpak.sh --version '0.8.0|evil'` and `scripts/package_macos.sh '0.8.0|evil'` exit non-zero immediately during version validation.
3. **CI Dry-Run:**
   - Trigger `workflow_dispatch` dry-run on `release.yml` (after push) to confirm all SHA-pinned actions and `env:`-passed version variables build cleanly across Linux, Windows, and macOS.
