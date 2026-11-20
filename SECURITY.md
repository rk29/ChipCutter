# Security Policy

## Reporting a Vulnerability

If you discover a security vulnerability—or a CNC physical-safety bug that could bypass simulation verification or inject unintended G-code commands—please report it responsibly rather than posting a public exploit:

- Open a **private Security Advisory** via GitHub's **Security → Advisories → Report a vulnerability** page on [rk29/ChipCutter](https://github.com/rk29/ChipCutter/security/advisories).
- Include the ChipCutter version (`chipcutter --version` or **Help → About ChipCutter**), platform, minimal reproduction steps, and expected vs. observed behavior.

## Supported Versions

Security and physical-safety fixes are provided for the latest minor release on the `main` branch.

---

## Threat Model & Trust Boundaries

ChipCutter is an offline desktop CAM (`chip-gui`) and CLI (`chipcutter`) application with zero runtime network dependencies. Its primary attack surface is opening untrusted external files shared between machinists:

- **Project & Bundle Files:** `.chip.toml`, `.chipz` (ZIP), `.machine.toml`, `.post.toml`, `.tools.toml`, and Fusion 360 `.json` / `.tools` (ZIP) libraries.
- **CAD, Mesh, & Vector Inputs:** `.step` / `.stp`, `.stl`, `.obj`, `.3mf` (ZIP), `.dxf`, `.svg`, and custom `.ttf` / `.otf` fonts.

### Built-In Defenses

- **Memory Safety:** Workspace-wide `#![deny(unsafe_code)]` (`Cargo.toml`), strict Clippy lints (`unwrap_used = "deny"`, `expect_used = "deny"`), and automated `cargo-deny` advisory & license audits on every CI run.
- **Read-Only Built-In Profiles:** Built-in machine definitions (`generic`, `milo-1.5`, `carvera-air`) and built-in post-processors (`grbl`, `grblhal_atc`, `linuxcnc`, `linuxcnc_4axis`, `grblhal_4axis`, `grbl_rotary_wrap`) cannot be shadowed or overwritten by untrusted project folders, `.chipz` archives, or user files.
- **Safe Output Paths & G-Code Emission:** Post-processor extensions and output filenames are restricted to single safe basenames with no path separators; operation/tool metadata is ASCII-sanitized before template substitution; control characters (`\r`, `\n`, `\0`) and non-finite (`NaN`/`±Inf`) numeric values are rejected before G-code emission.
- **Bounded Archives & Parsers:** `.chipz`, `.3mf`, and Fusion `.tools` ZIP readers reject symlinks, zip-slip paths, excessive entry counts, and cumulative decompressed payloads exceeding strict byte limits (without trusting central-directory size headers alone). Parametric expressions, 3MF `<component>` DAGs, and DXF `INSERT` block grids enforce bounded depth/expansion limits.
- **Private Temporary Extraction:** `.chipz` archives are extracted into exclusively created, owner-only (`0o700` on Unix) temporary directories.
- **Flatpak Sandbox:** The Linux Flatpak package runs without `--share=network` or `--filesystem=host`, relying on `xdg-desktop-portal`.

### Operator CNC Safety Notice

> [!WARNING]
> Custom `.post.toml` files define raw controller header, tool-change, spindle, and footer templates that are emitted *after* ChipCutter's 5-rule heightmap/cylindrical simulation verifier runs on cutter-location (`ClEvent`) toolpaths. Always inspect the emitted G-code preview—especially whenever ChipCutter flags a custom post profile loaded from a third-party project or `.chipz` bundle—before executing any program on physical CNC hardware.
