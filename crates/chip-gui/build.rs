fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=../../packaging/windows/chipcutter.ico");

    #[cfg(windows)]
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        let version = std::env::var("CARGO_PKG_VERSION")?;
        let mut res = winresource::WindowsResource::new();
        res.set_icon("../../packaging/windows/chipcutter.ico")
            .set("ProductName", "ChipCutter")
            .set("FileDescription", "ChipCutter Desktop CAM Application")
            .set("OriginalFilename", "chip-gui.exe")
            .set("InternalName", "chip-gui")
            .set("FileVersion", &version)
            .set("ProductVersion", &version)
            .set(
                "LegalCopyright",
                "Copyright (c) 2026 ChipCutter Contributors (MIT OR Apache-2.0)",
            );
        res.compile()?;
    }

    Ok(())
}
