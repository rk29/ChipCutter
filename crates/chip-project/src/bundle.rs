use std::{
    collections::BTreeMap,
    io::{Read as _, Write as _},
    path::{Component, Path, PathBuf},
};

use chip_post::{PostProfile, builtin_post_profiles, resolve_post_profile};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use zip::{CompressionMethod, DateTime, ZipArchive, ZipWriter, write::SimpleFileOptions};

use crate::{
    document::{ProjectDocument, compute_sha256_hex, resolve_model_path},
    toml_io::ProjectIoError,
};

/// Maximum allowed total uncompressed payload when unpacking a `.chipz` archive (`256 MiB`).
pub const MAX_CHIPZ_UNPACK_BYTES: u64 = 256 * 1024 * 1024;

/// Errors raised when packing or unpacking a portable `.chipz` bundle (`09-persistence-cli.md` §2).
#[derive(Debug, Error)]
pub enum BundleError {
    #[error("I/O error accessing '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("ZIP archive error: {0}")]
    Zip(#[from] zip::result::ZipError),
    #[error(transparent)]
    ProjectIo(#[from] ProjectIoError),
    #[error(
        "unsafe or invalid archive entry path '{entry_path}' (rejected zip-slip / absolute path)"
    )]
    UnsafeEntryPath { entry_path: String },
    #[error(
        "`.chipz` archive exceeds maximum allowed uncompressed size ({MAX_CHIPZ_UNPACK_BYTES} bytes)"
    )]
    ArchiveTooLarge,
    #[error("`.chipz` archive does not contain `project.chip.toml`")]
    MissingProjectToml,
    #[error("missing STEP model bytes for '{model_id}' ('{path}') while packing `.chipz`")]
    MissingModelBytes { model_id: String, path: String },
    #[error("SHA-256 mismatch on bundled model '{path}': expected {expected}, found {actual}")]
    ModelHashMismatch {
        path: String,
        expected: String,
        actual: String,
    },
}

/// Summary returned after exporting a `.chipz` archive.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChipzExportSummary {
    pub archive_size_bytes: usize,
    pub model_files: Vec<String>,
    pub custom_post_files: Vec<String>,
}

/// Extracted `.chipz` bundle metadata returned by [`unpack_chipz_bundle`].
#[derive(Debug, Clone)]
pub struct UnpackedChipzBundle {
    pub extract_dir: PathBuf,
    pub project_toml_path: PathBuf,
    pub project: ProjectDocument,
    pub extracted_files: Vec<PathBuf>,
}

/// Packs `project`, its referenced STEP models, and any non-builtin custom `.post.toml` profiles
/// into a deterministic, self-contained `.chipz` ZIP byte buffer (`09-persistence-cli.md` §2).
///
/// `fallback_model_bytes` allows unsaved in-memory GUI projects (e.g., built-in sample parts)
/// to supply `(model_id_or_name, raw_step_bytes)` without needing an external file on disk first.
pub fn pack_chipz_bundle_bytes(
    project: &ProjectDocument,
    project_dir: Option<&Path>,
    fallback_model_bytes: &[(&str, &[u8])],
    extra_custom_posts: &[PostProfile],
) -> Result<(Vec<u8>, ChipzExportSummary), BundleError> {
    let mut bundled_proj = project.clone();
    // Map archive entry path -> raw bytes in deterministic BTreeMap order
    let mut entries: BTreeMap<String, Vec<u8>> = BTreeMap::new();
    let mut model_files = Vec::new();
    let mut used_model_names: Vec<String> = Vec::new();

    for model in &mut bundled_proj.models {
        let raw_bytes: Vec<u8> = if let Some((_, mem_bytes)) = fallback_model_bytes
            .iter()
            .find(|(key, _)| *key == model.id || *key == model.path)
        {
            mem_bytes.to_vec()
        } else {
            let disk_path = resolve_model_path(project_dir, &model.path);
            std::fs::read(&disk_path).map_err(|source| {
                if source.kind() == std::io::ErrorKind::NotFound {
                    BundleError::MissingModelBytes {
                        model_id: model.id.clone(),
                        path: disk_path.display().to_string(),
                    }
                } else {
                    BundleError::Io {
                        path: disk_path.display().to_string(),
                        source,
                    }
                }
            })?
        };

        let base_fname = Path::new(&model.path)
            .file_name()
            .and_then(|s| s.to_str())
            .map(sanitize_bundle_filename)
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| format!("{}.step", sanitize_bundle_filename(&model.id)));

        let mut unique_fname = base_fname.clone();
        let mut suffix = 2usize;
        while used_model_names.contains(&unique_fname) {
            unique_fname = format!("{suffix}_{base_fname}");
            suffix += 1;
        }
        used_model_names.push(unique_fname.clone());

        let bundle_rel = format!("models/{unique_fname}");
        model.sha256 = compute_sha256_hex(&raw_bytes);
        model.path = bundle_rel.clone();
        model_files.push(bundle_rel.clone());
        entries.insert(bundle_rel, raw_bytes);
    }

    // Bundle any non-builtin custom `.post.toml` profiles referenced by Setups!
    let builtins = builtin_post_profiles();
    let mut custom_post_files = Vec::new();
    for setup in &bundled_proj.setups {
        let pid = setup.post_profile.trim();
        if pid.is_empty() || builtins.iter().any(|b| b.id.eq_ignore_ascii_case(pid)) {
            continue;
        }
        let custom_opt = extra_custom_posts
            .iter()
            .find(|p| p.id.eq_ignore_ascii_case(pid))
            .cloned()
            .or_else(|| resolve_post_profile(pid, project_dir).ok());

        if let Some(custom_prof) = custom_opt
            && let Ok(toml_text) = custom_prof.to_toml_string()
        {
            let fname = format!(
                "posts/{}.post.toml",
                sanitize_bundle_filename(&custom_prof.id)
            );
            if let std::collections::btree_map::Entry::Vacant(vac) = entries.entry(fname.clone()) {
                custom_post_files.push(fname);
                vac.insert(toml_text.into_bytes());
            }
        }
    }

    let proj_toml = bundled_proj.to_toml_string()?;
    entries.insert("project.chip.toml".to_string(), proj_toml.into_bytes());

    let mut zip_buf = std::io::Cursor::new(Vec::new());
    {
        let mut writer = ZipWriter::new(&mut zip_buf);
        let opts = SimpleFileOptions::default()
            .compression_method(CompressionMethod::Deflated)
            .last_modified_time(DateTime::default());

        // Write `project.chip.toml` first for easy human inspection, followed by sorted `models/*` and `posts/*`
        if let Some(proj_bytes) = entries.remove("project.chip.toml") {
            writer.start_file("project.chip.toml", opts)?;
            writer
                .write_all(&proj_bytes)
                .map_err(|source| BundleError::Io {
                    path: "project.chip.toml".to_string(),
                    source,
                })?;
        }
        for (entry_path, data) in entries {
            writer.start_file(&entry_path, opts)?;
            writer.write_all(&data).map_err(|source| BundleError::Io {
                path: entry_path,
                source,
            })?;
        }
        writer.finish()?;
    }

    let archive_bytes = zip_buf.into_inner();
    let summary = ChipzExportSummary {
        archive_size_bytes: archive_bytes.len(),
        model_files,
        custom_post_files,
    };
    Ok((archive_bytes, summary))
}

/// Exports a `.chipz` portable archive directly to `dest_chipz` on disk.
pub fn export_chipz_bundle(
    project: &ProjectDocument,
    project_dir: Option<&Path>,
    fallback_model_bytes: &[(&str, &[u8])],
    extra_custom_posts: &[PostProfile],
    dest_chipz: &Path,
) -> Result<ChipzExportSummary, BundleError> {
    let (bytes, summary) = pack_chipz_bundle_bytes(
        project,
        project_dir,
        fallback_model_bytes,
        extra_custom_posts,
    )?;
    if let Some(parent) = dest_chipz.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|source| BundleError::Io {
            path: parent.display().to_string(),
            source,
        })?;
    }
    std::fs::write(dest_chipz, bytes).map_err(|source| BundleError::Io {
        path: dest_chipz.display().to_string(),
        source,
    })?;
    Ok(summary)
}

/// Validates and unpacks a `.chipz` archive from `bytes` into `dest_dir`, rejecting path traversal
/// (`..` / absolute paths) and verifying every bundled model's SHA-256 hash.
pub fn unpack_chipz_bytes(
    bytes: &[u8],
    dest_dir: &Path,
) -> Result<UnpackedChipzBundle, BundleError> {
    std::fs::create_dir_all(dest_dir).map_err(|source| BundleError::Io {
        path: dest_dir.display().to_string(),
        source,
    })?;

    let cursor = std::io::Cursor::new(bytes);
    let mut archive = ZipArchive::new(cursor)?;

    let mut total_unpacked: u64 = 0;
    let mut extracted_files = Vec::new();
    let mut found_project_toml = false;

    for idx in 0..archive.len() {
        let mut entry = archive.by_index(idx)?;
        let raw_name = entry.name()?.into_owned();
        if entry.is_dir() || raw_name.ends_with('/') {
            validate_safe_relative_entry_path(&raw_name)?;
            continue;
        }
        if entry.is_symlink() {
            return Err(BundleError::UnsafeEntryPath {
                entry_path: raw_name,
            });
        }

        let clean_rel = validate_safe_relative_entry_path(&raw_name)?;
        total_unpacked = total_unpacked.saturating_add(entry.size());
        if total_unpacked > MAX_CHIPZ_UNPACK_BYTES {
            return Err(BundleError::ArchiveTooLarge);
        }

        let out_path = dest_dir.join(&clean_rel);
        if let Some(parent) = out_path.parent() {
            std::fs::create_dir_all(parent).map_err(|source| BundleError::Io {
                path: parent.display().to_string(),
                source,
            })?;
        }

        let mut buf = Vec::with_capacity((entry.size() as usize).min(16 * 1024 * 1024));
        entry
            .by_ref()
            .take(MAX_CHIPZ_UNPACK_BYTES + 1)
            .read_to_end(&mut buf)
            .map_err(|source| BundleError::Io {
                path: raw_name.clone(),
                source,
            })?;
        if buf.len() as u64 > MAX_CHIPZ_UNPACK_BYTES {
            return Err(BundleError::ArchiveTooLarge);
        }

        std::fs::write(&out_path, &buf).map_err(|source| BundleError::Io {
            path: out_path.display().to_string(),
            source,
        })?;

        if clean_rel == Path::new("project.chip.toml") {
            found_project_toml = true;
        }
        extracted_files.push(out_path);
    }

    if !found_project_toml {
        return Err(BundleError::MissingProjectToml);
    }

    let project_toml_path = dest_dir.join("project.chip.toml");
    let project = ProjectDocument::load_from_path(&project_toml_path)?;

    // Verify SHA-256 of every bundled STEP model
    for model in &project.models {
        let model_rel = validate_safe_relative_entry_path(&model.path)?;
        let model_path = dest_dir.join(model_rel);
        let model_bytes = std::fs::read(&model_path).map_err(|source| BundleError::Io {
            path: model_path.display().to_string(),
            source,
        })?;
        let actual_sha = compute_sha256_hex(&model_bytes);
        if !model.sha256.is_empty() && !actual_sha.eq_ignore_ascii_case(&model.sha256) {
            return Err(BundleError::ModelHashMismatch {
                path: model.path.clone(),
                expected: model.sha256.clone(),
                actual: actual_sha,
            });
        }
    }

    Ok(UnpackedChipzBundle {
        extract_dir: dest_dir.to_path_buf(),
        project_toml_path,
        project,
        extracted_files,
    })
}

/// Unpacks a `.chipz` file on disk into `dest_dir`.
pub fn unpack_chipz_bundle(
    chipz_path: &Path,
    dest_dir: &Path,
) -> Result<UnpackedChipzBundle, BundleError> {
    let bytes = std::fs::read(chipz_path).map_err(|source| BundleError::Io {
        path: chipz_path.display().to_string(),
        source,
    })?;
    unpack_chipz_bytes(&bytes, dest_dir)
}

/// Returns `true` if `path` has a `.chipz` extension (case-insensitive).
pub fn is_chipz_path(path: &Path) -> bool {
    path.extension()
        .and_then(|s| s.to_str())
        .is_some_and(|ext| ext.eq_ignore_ascii_case("chipz"))
}

fn validate_safe_relative_entry_path(raw: &str) -> Result<PathBuf, BundleError> {
    if raw.contains('\\') || raw.starts_with('/') || raw.trim().is_empty() {
        return Err(BundleError::UnsafeEntryPath {
            entry_path: raw.to_string(),
        });
    }
    let p = Path::new(raw);
    let mut out = PathBuf::new();
    for comp in p.components() {
        match comp {
            Component::Normal(seg) => {
                let s = seg.to_string_lossy();
                if s.is_empty() || s == "." || s == ".." {
                    return Err(BundleError::UnsafeEntryPath {
                        entry_path: raw.to_string(),
                    });
                }
                out.push(seg);
            }
            Component::CurDir
            | Component::ParentDir
            | Component::RootDir
            | Component::Prefix(_) => {
                return Err(BundleError::UnsafeEntryPath {
                    entry_path: raw.to_string(),
                });
            }
        }
    }
    if out.as_os_str().is_empty() {
        return Err(BundleError::UnsafeEntryPath {
            entry_path: raw.to_string(),
        });
    }
    Ok(out)
}

fn sanitize_bundle_filename(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        if ch.is_ascii_alphanumeric() || matches!(ch, '.' | '_' | '-') {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches(['.', '_']);
    if trimmed.is_empty() {
        "file".to_string()
    } else {
        trimmed.to_string()
    }
}
