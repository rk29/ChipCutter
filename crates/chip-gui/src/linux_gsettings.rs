//! Linux `GSettings` backend safeguard for native GTK3 file dialogs (`rfd`).
//!
//! `rfd`'s `gtk3` backend opens in-process `GtkFileChooserDialog` windows, which
//! read and write `org.gtk.Settings.FileChooser` via GLib's `GSettings`. On Linux
//! systems with `dconf` installed, GLib selects the `dconf` backend by default.
//! When no D-Bus session bus is reachable (e.g., pure Wayland sessions without
//! X11 `$DISPLAY` or minimal containers) or when `$XDG_RUNTIME_DIR/dconf` cannot
//! be created or written, `dconf` emits repeated `dconf-CRITICAL` and
//! `dconf-WARNING` messages to stderr during file dialog interaction.
//!
//! Calling [`configure_gsettings_backend_for_gtk`] at process startup (before
//! spawning threads or initializing GTK/GLib) falls back to GLib's built-in
//! `memory` backend only when `GSETTINGS_BACKEND` is not explicitly configured
//! and `dconf` cannot operate in the current environment.

use std::ffi::{OsStr, OsString};
use std::fs::{self, OpenOptions};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

static PROBE_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Snapshot of environment variables relevant to GLib's `GSettings` / `dconf` backend.
#[derive(Debug, Default, Clone)]
pub(crate) struct GsettingsEnv {
    pub gsettings_backend: Option<OsString>,
    pub dbus_session_bus_address: Option<OsString>,
    pub xdg_runtime_dir: Option<OsString>,
    pub xdg_cache_home: Option<OsString>,
    pub home: Option<OsString>,
}

impl GsettingsEnv {
    pub(crate) fn from_process() -> Self {
        Self {
            gsettings_backend: std::env::var_os("GSETTINGS_BACKEND"),
            dbus_session_bus_address: std::env::var_os("DBUS_SESSION_BUS_ADDRESS"),
            xdg_runtime_dir: std::env::var_os("XDG_RUNTIME_DIR"),
            xdg_cache_home: std::env::var_os("XDG_CACHE_HOME"),
            home: std::env::var_os("HOME"),
        }
    }
}

/// Sets `GSETTINGS_BACKEND=memory` when `dconf` is unavailable in the current
/// Linux environment so native GTK3 file dialogs stay silent on stderr.
///
/// Must be called at the very start of `main()` before any worker threads,
/// `eframe`, or GTK/GLib state have been initialized.
#[allow(unsafe_code)]
pub(crate) fn configure_gsettings_backend_for_gtk() {
    let env = GsettingsEnv::from_process();
    if should_use_memory_backend(&env) {
        // SAFETY: Invoked at the start of `main()` before any threads or C libraries
        // are started, so mutating the process environment cannot race with readers.
        unsafe {
            std::env::set_var("GSETTINGS_BACKEND", "memory");
        }
    }
}

/// Returns `true` when `chip-gui` should select GLib's in-memory `GSettings`
/// backend instead of letting GLib attempt to use `dconf`.
pub(crate) fn should_use_memory_backend(env: &GsettingsEnv) -> bool {
    // Respect any explicit non-empty `GSETTINGS_BACKEND` chosen by the user.
    if env
        .gsettings_backend
        .as_deref()
        .is_some_and(|val| !val.is_empty())
    {
        return false;
    }

    if !has_reachable_dbus_session(env) {
        return true;
    }

    match effective_glib_runtime_dir(env) {
        Some(runtime_dir) => !is_dconf_runtime_dir_writable(&runtime_dir),
        None => true,
    }
}

fn has_reachable_dbus_session(env: &GsettingsEnv) -> bool {
    if let Some(addr_os) = env.dbus_session_bus_address.as_deref()
        && !addr_os.is_empty()
    {
        let Some(addr_str) = addr_os.to_str() else {
            return true;
        };
        return dbus_address_looks_reachable(addr_str, env);
    }

    // When `DBUS_SESSION_BUS_ADDRESS` is unset, GDBus checks `$XDG_RUNTIME_DIR/bus`
    // before falling back to `dbus-launch --autolaunch` (which fails without X11
    // `$DISPLAY` or `dbus-launch`).
    absolute_env_path(env.xdg_runtime_dir.as_deref())
        .is_some_and(|runtime_dir| runtime_dir.join("bus").exists())
}

fn dbus_address_looks_reachable(addr_str: &str, env: &GsettingsEnv) -> bool {
    addr_str.split(';').any(|entry| {
        let entry = entry.trim();
        if entry.is_empty() || entry.starts_with("autolaunch:") {
            return false;
        }
        if let Some(rest) = entry.strip_prefix("unix:") {
            for param in rest.split(',') {
                if let Some(path) = param.strip_prefix("path=") {
                    if path.contains('%') {
                        return true;
                    }
                    return Path::new(path).exists();
                }
                if param == "runtime=yes" {
                    return absolute_env_path(env.xdg_runtime_dir.as_deref())
                        .is_some_and(|runtime_dir| runtime_dir.join("bus").exists());
                }
            }
            // Abstract Unix sockets (`unix:abstract=...`) do not live on the filesystem.
            return true;
        }
        true
    })
}

fn effective_glib_runtime_dir(env: &GsettingsEnv) -> Option<PathBuf> {
    if let Some(runtime_dir) = absolute_env_path(env.xdg_runtime_dir.as_deref()) {
        return Some(runtime_dir);
    }
    if let Some(cache_home) = absolute_env_path(env.xdg_cache_home.as_deref()) {
        return Some(cache_home);
    }
    absolute_env_path(env.home.as_deref()).map(|home| home.join(".cache"))
}

fn absolute_env_path(val: Option<&OsStr>) -> Option<PathBuf> {
    let val = val?;
    if val.is_empty() {
        return None;
    }
    let path = PathBuf::from(val);
    path.is_absolute().then_some(path)
}

fn is_dconf_runtime_dir_writable(runtime_dir: &Path) -> bool {
    let dconf_dir = runtime_dir.join("dconf");
    if let Ok(meta) = fs::metadata(&dconf_dir) {
        return meta.is_dir() && probe_dir_writable(&dconf_dir);
    }
    if fs::symlink_metadata(&dconf_dir).is_ok() {
        // Broken symlink at `$XDG_RUNTIME_DIR/dconf` blocks `g_mkdir_with_parents`.
        return false;
    }

    // `dconf` does not exist yet: find the deepest existing ancestor that GLib
    // would need to create directories beneath.
    let mut current = Some(runtime_dir);
    while let Some(dir) = current {
        if let Ok(meta) = fs::metadata(dir) {
            return meta.is_dir() && probe_dir_writable(dir);
        }
        current = dir.parent();
    }
    false
}

fn probe_dir_writable(dir: &Path) -> bool {
    let seq = PROBE_COUNTER.fetch_add(1, Ordering::Relaxed);
    let probe_path = dir.join(format!(
        ".chip-gui-dconf-probe-{}-{seq}",
        std::process::id()
    ));
    match OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&probe_path)
    {
        Ok(_file) => {
            let _ = fs::remove_file(&probe_path);
            true
        }
        Err(_) => false,
    }
}

#[cfg(test)]
mod tests {
    use std::ffi::OsString;
    use std::fs;
    use std::os::unix::fs::PermissionsExt;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};

    use super::{GsettingsEnv, should_use_memory_backend};

    static TEST_DIR_COUNTER: AtomicU64 = AtomicU64::new(0);

    struct TempDirGuard(PathBuf);

    impl TempDirGuard {
        fn new(label: &str) -> Self {
            let seq = TEST_DIR_COUNTER.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "chip_gui_gsettings_{label}_{}_{seq}",
                std::process::id()
            ));
            let _ = fs::remove_dir_all(&path);
            assert!(fs::create_dir_all(&path).is_ok());
            Self(path)
        }
    }

    impl Drop for TempDirGuard {
        fn drop(&mut self) {
            let _ = fs::set_permissions(&self.0, fs::Permissions::from_mode(0o700));
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn respects_explicit_non_empty_gsettings_backend() {
        let env = GsettingsEnv {
            gsettings_backend: Some(OsString::from("keyfile")),
            ..Default::default()
        };
        assert!(!should_use_memory_backend(&env));
    }

    #[test]
    fn empty_gsettings_backend_falls_back_when_dconf_unavailable() {
        let env = GsettingsEnv {
            gsettings_backend: Some(OsString::new()),
            ..Default::default()
        };
        assert!(should_use_memory_backend(&env));
    }

    #[test]
    fn falls_back_when_no_dbus_session_is_available() {
        let rt = TempDirGuard::new("no_dbus");
        let env = GsettingsEnv {
            xdg_runtime_dir: Some(rt.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(should_use_memory_backend(&env));
    }

    #[test]
    fn falls_back_when_dbus_unix_socket_path_is_missing() {
        let rt = TempDirGuard::new("missing_dbus_sock");
        let missing_sock = rt.0.join("missing-bus");
        let env = GsettingsEnv {
            dbus_session_bus_address: Some(OsString::from(format!(
                "unix:path={}",
                missing_sock.display()
            ))),
            xdg_runtime_dir: Some(rt.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(should_use_memory_backend(&env));
    }

    #[test]
    fn falls_back_when_dconf_path_is_blocked_by_regular_file() {
        let rt = TempDirGuard::new("blocked_dconf");
        assert!(fs::write(rt.0.join("bus"), b"").is_ok());
        assert!(fs::write(rt.0.join("dconf"), b"not-a-dir").is_ok());

        let env = GsettingsEnv {
            xdg_runtime_dir: Some(rt.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(should_use_memory_backend(&env));
    }

    #[test]
    fn falls_back_when_runtime_dir_is_read_only() {
        let rt = TempDirGuard::new("ro_runtime");
        let bus = rt.0.join("bus");
        assert!(fs::write(&bus, b"").is_ok());
        assert!(fs::set_permissions(&rt.0, fs::Permissions::from_mode(0o555)).is_ok());

        // When running as root (UID 0), POSIX mode bits do not prevent writes; skip the
        // permission assertion in that case so containerized root test runs stay green.
        let probe = rt.0.join("root-check");
        if fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&probe)
            .is_ok()
        {
            let _ = fs::remove_file(&probe);
            return;
        }

        let env = GsettingsEnv {
            xdg_runtime_dir: Some(rt.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(should_use_memory_backend(&env));
    }

    #[test]
    fn keeps_default_dconf_when_bus_and_runtime_dir_are_usable() {
        let rt = TempDirGuard::new("healthy_xdg");
        assert!(fs::write(rt.0.join("bus"), b"").is_ok());

        let env = GsettingsEnv {
            xdg_runtime_dir: Some(rt.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(!should_use_memory_backend(&env));
    }

    #[test]
    fn keeps_default_dconf_with_explicit_dbus_path_and_writable_dconf_dir() {
        let rt = TempDirGuard::new("healthy_explicit");
        let bus = rt.0.join("session-bus");
        assert!(fs::write(&bus, b"").is_ok());
        assert!(fs::create_dir(rt.0.join("dconf")).is_ok());

        let env = GsettingsEnv {
            dbus_session_bus_address: Some(OsString::from(format!(
                "unix:path={},guid=0123456789abcdef",
                bus.display()
            ))),
            xdg_runtime_dir: Some(rt.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(!should_use_memory_backend(&env));
    }

    #[test]
    fn keeps_default_dconf_with_abstract_dbus_and_writable_cache_fallback() {
        let cache = TempDirGuard::new("abstract_cache");
        let env = GsettingsEnv {
            dbus_session_bus_address: Some(OsString::from("unix:abstract=/tmp/dbus-test")),
            xdg_cache_home: Some(cache.0.as_os_str().to_os_string()),
            ..Default::default()
        };
        assert!(!should_use_memory_backend(&env));
    }
}
