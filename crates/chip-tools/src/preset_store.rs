//! Shared OS user-config preset store utilities used by both `chip-machines` (`.machine.toml`)
//! and `chip-post` (`.post.toml`), enforcing read-only built-in protection, deterministic
//! discovery order, and upgrade-safe user config persistence (`AGENTS.md` §2.8).

use std::path::{Path, PathBuf};

/// Origin tier of a discovered preset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PresetOrigin {
    BuiltIn,
    ProjectOrBundle,
    UserConfig,
}

/// Declarative specification of a preset collection stored in OS user config space.
#[derive(Debug, Clone, Copy)]
pub struct PresetStoreSpec {
    /// Subdirectory under `<config_dir>/chipcutter/` and `<extra_search_dir>/` (e.g. `"machines"`, `"posts"`).
    pub subdir_name: &'static str,
    /// Filename suffix including leading dot (e.g. `".machine.toml"`, `".post.toml"`).
    pub file_suffix: &'static str,
    /// Reserved read-only built-in preset IDs (case-insensitive match).
    pub builtin_ids: &'static [&'static str],
    /// Word separator used when sanitizing arbitrary user strings into slugs (`'-'` or `'_'`).
    pub slug_separator: char,
    /// Fallback slug when sanitization produces an empty string.
    pub default_slug: &'static str,
    /// Suffix appended on first collision (e.g. `"-custom"` or `"_custom"`).
    pub custom_suffix: &'static str,
    /// Fallback suffix if 9,999 numeric suffixes are exhausted.
    pub fallback_suffix: &'static str,
    /// Whether uppercase ASCII letters are accepted by [`validate_slug_id`].
    pub allow_uppercase_ids: bool,
    /// Whether `sanitize_slug_id` preserves interior dots (`'.'`).
    pub preserve_dots_in_sanitize: bool,
}

impl PresetStoreSpec {
    /// Returns the OS user configuration directory for this preset collection
    /// (e.g. `~/.config/chipcutter/<subdir_name>` on Linux).
    pub fn user_dir(&self) -> Option<PathBuf> {
        chipcutter_config_dir().map(|base| base.join(self.subdir_name))
    }

    /// Returns `true` if `id` matches a reserved read-only built-in ID (case-insensitive).
    pub fn is_builtin_id(&self, id: &str) -> bool {
        let needle = id.trim();
        self.builtin_ids
            .iter()
            .any(|b| b.eq_ignore_ascii_case(needle))
    }
}

/// Structured error returned by generic [`PresetStoreSpec`] CRUD operations.
#[derive(Debug)]
pub enum PresetStoreError<E> {
    InvalidId {
        id: String,
    },
    BuiltInReadOnly {
        id: String,
    },
    AlreadyExists {
        id: String,
    },
    NotFound {
        id: String,
    },
    Io {
        path: String,
        source: std::io::Error,
    },
    Codec(E),
}

/// Returns the root OS user configuration directory for ChipCutter
/// (e.g. `~/.config/chipcutter` on Linux).
pub fn chipcutter_config_dir() -> Option<PathBuf> {
    dirs::config_dir().map(|base| base.join("chipcutter"))
}

/// Validates a preset slug ID (`[a-z0-9._-]` or `[A-Za-z0-9._-]`, non-empty, no leading `.`, no `..`).
pub fn validate_slug_id(id: &str, allow_uppercase: bool) -> bool {
    !id.is_empty()
        && !id.starts_with('.')
        && !id.contains("..")
        && id.bytes().all(|b| {
            let letter_ok = if allow_uppercase {
                b.is_ascii_alphabetic()
            } else {
                b.is_ascii_lowercase()
            };
            letter_ok || b.is_ascii_digit() || matches!(b, b'-' | b'_' | b'.')
        })
}

/// Converts an arbitrary user string into a safe slug ID honoring `spec`'s separator and dot policy.
pub fn sanitize_slug_id(raw: &str, spec: &PresetStoreSpec) -> String {
    let sep = spec.slug_separator;
    let mut out = String::with_capacity(raw.len());
    let mut prev_sep = false;

    for ch in raw.trim().chars() {
        let lower = ch.to_ascii_lowercase();
        if spec.preserve_dots_in_sanitize {
            if lower.is_ascii_lowercase() || lower.is_ascii_digit() || matches!(lower, '_' | '.') {
                if out.is_empty() && lower == '.' {
                    continue;
                }
                if lower == '.' && out.ends_with('.') {
                    continue;
                }
                out.push(lower);
                prev_sep = false;
            } else if !out.is_empty() && !prev_sep {
                out.push(sep);
                prev_sep = true;
            }
        } else if ch.is_ascii_alphanumeric() {
            out.push(lower);
        } else if matches!(ch, '-' | '_' | ' ' | '.')
            && !out.ends_with('_')
            && !out.ends_with('-')
            && !out.is_empty()
        {
            out.push(sep);
        }
    }

    while out.ends_with('-') || out.ends_with('_') || out.ends_with('.') {
        out.pop();
    }
    if out.is_empty() {
        spec.default_slug.to_string()
    } else {
        out
    }
}

/// Picks a unique non-builtin custom preset ID given `base_id` and `existing_ids`.
pub fn allocate_unique_custom_id(
    base_id: &str,
    existing_ids: &[String],
    spec: &PresetStoreSpec,
) -> String {
    let clean = sanitize_slug_id(base_id, spec);
    let is_taken = |cand: &str| -> bool {
        spec.is_builtin_id(cand) || existing_ids.iter().any(|e| e.eq_ignore_ascii_case(cand))
    };
    if !is_taken(&clean) {
        return clean;
    }
    let custom = format!("{clean}{}", spec.custom_suffix);
    if !is_taken(&custom) {
        return custom;
    }
    for n in 2..10_000 {
        let cand = format!("{clean}{}{n}", spec.slug_separator);
        if !is_taken(&cand) {
            return cand;
        }
    }
    format!("{clean}{}", spec.fallback_suffix)
}

/// Discovers presets by merging `builtins` with files matching `spec.file_suffix` in
/// `extra_search_dir` (project/bundle) and `user_dir` (OS user config).
///
/// Built-in IDs are strictly read-only and can never be shadowed. User config presets override
/// project/bundle presets sharing the same custom ID.
#[allow(clippy::too_many_arguments)]
pub fn discover_presets_in<T, FLoad, FId, FIsBuiltin, FApplyOrigin>(
    builtins: Vec<T>,
    user_dir: Option<&Path>,
    extra_search_dir: Option<&Path>,
    spec: &PresetStoreSpec,
    load_from_path: FLoad,
    get_id: FId,
    is_builtin: FIsBuiltin,
    mut apply_origin: FApplyOrigin,
) -> Vec<T>
where
    FLoad: Fn(&Path) -> Option<T>,
    FId: Fn(&T) -> &str,
    FIsBuiltin: Fn(&T) -> bool,
    FApplyOrigin: FnMut(&mut T, PresetOrigin),
{
    let mut presets = builtins;

    let mut search_dirs: Vec<(PathBuf, PresetOrigin)> = Vec::new();
    if let Some(extra) = extra_search_dir {
        search_dirs.push((extra.to_path_buf(), PresetOrigin::ProjectOrBundle));
        search_dirs.push((extra.join(spec.subdir_name), PresetOrigin::ProjectOrBundle));
    }
    if let Some(udir) = user_dir {
        search_dirs.push((udir.to_path_buf(), PresetOrigin::UserConfig));
    }

    for (dir, origin) in search_dirs {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        let mut paths: Vec<PathBuf> = entries
            .filter_map(|e| e.ok().map(|de| de.path()))
            .filter(|p| {
                p.file_name()
                    .and_then(|s| s.to_str())
                    .is_some_and(|n| n.ends_with(spec.file_suffix))
            })
            .collect();
        paths.sort();

        for p in paths {
            let Some(mut preset) = load_from_path(&p) else {
                continue;
            };
            let id_owned = get_id(&preset).to_string();
            if !validate_slug_id(&id_owned, spec.allow_uppercase_ids)
                || spec.is_builtin_id(&id_owned)
            {
                continue;
            }
            apply_origin(&mut preset, origin);
            if let Some(existing) = presets
                .iter_mut()
                .find(|x| !is_builtin(x) && get_id(x).eq_ignore_ascii_case(&id_owned))
            {
                *existing = preset;
            } else {
                presets.push(preset);
            }
        }
    }

    presets
}

/// Resolves a preset strictly by ID (case-insensitive) from `available`.
pub fn resolve_preset_in<T, FId>(id: &str, available: Vec<T>, get_id: FId) -> Option<T>
where
    FId: Fn(&T) -> &str,
{
    let needle = id.trim().to_ascii_lowercase();
    available
        .into_iter()
        .find(|item| get_id(item).to_ascii_lowercase() == needle)
}

/// Saves a validated custom user preset into `user_dir/<id><file_suffix>`.
/// Rejects built-in / reserved IDs.
pub fn save_user_preset_in<T, FId, FIsBuiltin, FSerialize, E>(
    preset: &T,
    user_dir: &Path,
    spec: &PresetStoreSpec,
    get_id: FId,
    is_builtin: FIsBuiltin,
    serialize: FSerialize,
) -> Result<PathBuf, PresetStoreError<E>>
where
    FId: Fn(&T) -> &str,
    FIsBuiltin: Fn(&T) -> bool,
    FSerialize: Fn(&T) -> Result<String, E>,
{
    let id = get_id(preset);
    if is_builtin(preset) || spec.is_builtin_id(id) {
        return Err(PresetStoreError::BuiltInReadOnly { id: id.to_string() });
    }
    if !validate_slug_id(id, spec.allow_uppercase_ids) {
        return Err(PresetStoreError::InvalidId { id: id.to_string() });
    }
    let raw = serialize(preset).map_err(PresetStoreError::Codec)?;
    std::fs::create_dir_all(user_dir).map_err(|source| PresetStoreError::Io {
        path: user_dir.display().to_string(),
        source,
    })?;
    let dest = user_dir.join(format!("{id}{}", spec.file_suffix));
    std::fs::write(&dest, raw).map_err(|source| PresetStoreError::Io {
        path: dest.display().to_string(),
        source,
    })?;
    Ok(dest)
}

/// Deletes a custom user preset file `user_dir/<id><file_suffix>`.
/// Rejects invalid or reserved built-in IDs.
pub fn delete_user_preset_in(
    id: &str,
    user_dir: &Path,
    spec: &PresetStoreSpec,
) -> Result<(), PresetStoreError<std::convert::Infallible>> {
    if !validate_slug_id(id, spec.allow_uppercase_ids) {
        return Err(PresetStoreError::InvalidId { id: id.to_string() });
    }
    if spec.is_builtin_id(id) {
        return Err(PresetStoreError::BuiltInReadOnly { id: id.to_string() });
    }
    let path = user_dir.join(format!("{id}{}", spec.file_suffix));
    if !path.exists() {
        return Err(PresetStoreError::NotFound { id: id.to_string() });
    }
    std::fs::remove_file(&path).map_err(|source| PresetStoreError::Io {
        path: path.display().to_string(),
        source,
    })
}

/// Exports any preset (built-in or custom) to `dest`, creating parent directories as needed.
pub fn export_preset_to_path<T, FSerialize, E>(
    preset: &T,
    dest: &Path,
    serialize: FSerialize,
) -> Result<(), PresetStoreError<E>>
where
    FSerialize: Fn(&T) -> Result<String, E>,
{
    let raw = serialize(preset).map_err(PresetStoreError::Codec)?;
    if let Some(parent) = dest.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|source| PresetStoreError::Io {
            path: parent.display().to_string(),
            source,
        })?;
    }
    std::fs::write(dest, raw).map_err(|source| PresetStoreError::Io {
        path: dest.display().to_string(),
        source,
    })
}

/// Loads, assigns a non-colliding custom ID (or validates `id_override`), marks as user-owned,
/// and saves a preset from `src` into `user_dir`.
#[allow(clippy::too_many_arguments)]
pub fn import_preset_from_path_in<T, FLoad, FId, FSetId, FMarkUser, FIsBuiltin, FSerialize, E>(
    src: &Path,
    user_dir: &Path,
    existing_ids: &[String],
    id_override: Option<&str>,
    spec: &PresetStoreSpec,
    load: FLoad,
    get_id: FId,
    set_id: FSetId,
    mark_user: FMarkUser,
    is_builtin: FIsBuiltin,
    serialize: FSerialize,
) -> Result<T, PresetStoreError<E>>
where
    FLoad: Fn(&Path) -> Result<T, E>,
    FId: Fn(&T) -> &str,
    FSetId: Fn(&mut T, String),
    FMarkUser: Fn(&mut T),
    FIsBuiltin: Fn(&T) -> bool,
    FSerialize: Fn(&T) -> Result<String, E>,
{
    let mut item = load(src).map_err(PresetStoreError::Codec)?;
    mark_user(&mut item);

    if let Some(custom_id) = id_override {
        if !validate_slug_id(custom_id, spec.allow_uppercase_ids) {
            return Err(PresetStoreError::InvalidId {
                id: custom_id.to_string(),
            });
        }
        if spec.is_builtin_id(custom_id) {
            return Err(PresetStoreError::BuiltInReadOnly {
                id: custom_id.to_string(),
            });
        }
        if existing_ids
            .iter()
            .any(|e| e.eq_ignore_ascii_case(custom_id))
        {
            return Err(PresetStoreError::AlreadyExists {
                id: custom_id.to_string(),
            });
        }
        set_id(&mut item, custom_id.to_string());
    } else {
        let unique = allocate_unique_custom_id(get_id(&item), existing_ids, spec);
        if unique != get_id(&item) {
            set_id(&mut item, unique);
        }
    }

    save_user_preset_in(&item, user_dir, spec, get_id, is_builtin, serialize)?;
    Ok(item)
}
