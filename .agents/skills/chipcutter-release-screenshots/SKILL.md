---
name: chipcutter-release-screenshots
description: >-
  Runbook for refreshing all ChipCutter headless Vulkan screenshots (tutorials,
  Sample Gallery, and landing hero) and cutting a verified linear v0.<phase>.0
  release in an isolated git worktree. Activate when the user asks to refresh
  screenshots or release/tag a new ChipCutter version.
---

# ChipCutter Screenshot Refresh & Release Runbook

## Part 1 — Full Headless Screenshot Refresh

1. Run both capture scripts using the shared target directory and copy the landing hero from `rotary-4axis/hero.png`:
   ```bash
   CARGO_TARGET_DIR=/workspace/ChipCutter/target ./scripts/capture_tutorial_screenshots.sh --milestone all </dev/null \
     && CARGO_TARGET_DIR=/workspace/ChipCutter/target ./scripts/capture_sample_gallery.sh </dev/null \
     && cp docs/images/samples/rotary-4axis/hero.png docs/images/chipcutter-gui.png
   ```
   - For ad-hoc single captures, always wrap with:
     `WAYLAND_DISPLAY= WINIT_UNIX_BACKEND=x11 WGPU_BACKEND=vulkan timeout --foreground -k 5 180s xvfb-run -a ... </dev/null`
2. Restore any timestamp-only DXF asset regeneration:
   ```bash
   git checkout -- docs/tutorials/assets/13-router-panel.dxf
   ```
3. Validate every PNG header, visually inspect representative screenshots (`docs/images/chipcutter-gui.png`, `docs/images/tutorials/00-getting-started/03-workspace-overview.png`, and at least one Sample Gallery hero), and run docs parity:
   ```bash
   python3 -c "import glob; bad=[p for p in glob.glob('docs/images/**/*.png', recursive=True) if open(p,'rb').read(8)!=b'\x89PNG\r\n\x1a\n']; assert not bad, bad; print('OK:', len(glob.glob('docs/images/**/*.png', recursive=True)))"
   cargo test --locked -p chip-gui --test docs_samples_parity
   ```
4. Stage only `docs/images/` (explicit path — never `git add -A`) and commit on the phase branch.

## Part 2 — Linear Isolated-Worktree Release (`v0.<phase>.0`)

1. Verify linear ancestry on `origin/main` and create an isolated release worktree so local untracked files (e.g., `torture/`) remain untouched:
   ```bash
   git fetch origin
   git merge-base --is-ancestor origin/main HEAD   # rebase first if not linear
   git worktree add -b release-0.<N>.0 /workspace/ChipCutter-release-0.<N>.0 HEAD
   ```
2. Synchronize all **25 release files** from `0.<prev>.0` → `0.<N>.0` inside `/workspace/ChipCutter-release-0.<N>.0`:
   - `Cargo.toml`
   - `docs/index.html` (hero version badge + Flatpak install filename)
   - `packaging/flatpak/io.github.rk29.ChipCutter.metainfo.xml` (prepend `<release version="0.<N>.0" date="YYYY-MM-DD">`)
   - All 21 golden G-code headers under `crates/chip-cli/tests/golden/{grbl,linuxcnc}/*.{nc,ngc}`
   - Regenerate `Cargo.lock` via `CARGO_TARGET_DIR=/workspace/ChipCutter/target cargo check --workspace`
   - Confirm `git diff --stat` reports exactly `25 files changed`.
3. Run the full 5-command verification gate:
   ```bash
   CARGO_TARGET_DIR=/workspace/ChipCutter/target cargo fmt --all -- --check \
     && CARGO_TARGET_DIR=/workspace/ChipCutter/target cargo clippy --locked --workspace --all-targets -- -D warnings \
     && CARGO_TARGET_DIR=/workspace/ChipCutter/target cargo clippy --locked -p chip-gui --no-default-features --features flatpak-portal --all-targets -- -D warnings \
     && CARGO_TARGET_DIR=/workspace/ChipCutter/target cargo deny check \
     && CARGO_TARGET_DIR=/workspace/ChipCutter/target cargo deny --no-default-features --features chip-gui/flatpak-portal check \
     && CARGO_TARGET_DIR=/workspace/ChipCutter/target WGPU_BACKEND=vulkan cargo test --locked --workspace
   ```
4. Commit, create annotated tag `v0.<N>.0`, and push linearly:
   ```bash
   git add Cargo.toml Cargo.lock docs/index.html packaging/flatpak/io.github.rk29.ChipCutter.metainfo.xml crates/chip-cli/tests/golden/
   git commit -m "Release v0.<N>.0"
   git tag -a v0.<N>.0 -m "Release v0.<N>.0 — Phase <N>: <Title>"
   git push origin HEAD:main v0.<N>.0
   ```
5. Fast-forward local `main`, clean up worktree and temporary branches, and verify `git status -sb` preserves local `torture/` files:
   ```bash
   cd /workspace/ChipCutter
   git fetch origin && git checkout main && git merge --ff-only origin/main
   git worktree remove /workspace/ChipCutter-release-0.<N>.0
   git branch -d release-0.<N>.0 <phase-branch>
   ```
