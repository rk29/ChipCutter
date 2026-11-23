#![deny(unsafe_code)]
#![allow(clippy::unwrap_used, clippy::expect_used)]

//! Automated parity and Markdown link/image audit for the Sample Projects Gallery (`docs/samples/`),
//! Tutorials library (`docs/tutorials/`), and root `README.md` (Milestone 71).

use std::{
    collections::BTreeSet,
    fs,
    path::{Path, PathBuf},
};

use chip_gui::panels::welcome_gallery::{SAMPLE_CATALOG, SampleProjectId};

const PNG_SIGNATURE: &[u8; 8] = b"\x89PNG\r\n\x1a\n";

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .canonicalize()
        .expect("failed to canonicalize repo root from CARGO_MANIFEST_DIR")
}

fn assert_valid_png(path: &Path) {
    let bytes = fs::read(path)
        .unwrap_or_else(|e| panic!("expected PNG image at {} to exist: {e}", path.display()));
    assert!(
        bytes.len() > PNG_SIGNATURE.len() && bytes.starts_with(PNG_SIGNATURE),
        "expected non-empty valid PNG header at {} (size={})",
        path.display(),
        bytes.len()
    );
}

#[test]
fn all_catalog_samples_have_gallery_pages_images_and_index_links() {
    let root = repo_root();
    let samples_dir = root.join("docs/samples");
    let images_dir = root.join("docs/images/samples");
    let gallery_index_path = samples_dir.join("README.md");
    let gallery_index = fs::read_to_string(&gallery_index_path).unwrap_or_else(|e| {
        panic!("failed to read {}: {e}", gallery_index_path.display());
    });

    let mut expected_ids: Vec<SampleProjectId> = SAMPLE_CATALOG.iter().map(|s| s.id).collect();
    if !expected_ids.contains(&SampleProjectId::CadRevisionV2Reload) {
        expected_ids.push(SampleProjectId::CadRevisionV2Reload);
    }

    let mut seen_slugs = BTreeSet::new();
    for id in expected_ids {
        let slug = id.cli_slug();
        assert!(
            seen_slugs.insert(slug),
            "duplicate sample slug '{slug}' in SAMPLE_CATALOG"
        );
        assert_eq!(
            SampleProjectId::from_cli_slug(slug),
            Ok(id),
            "SampleProjectId::from_cli_slug('{slug}') did not roundtrip to {id:?}"
        );

        let page_path = samples_dir.join(format!("{slug}.md"));
        let page_body = fs::read_to_string(&page_path).unwrap_or_else(|e| {
            panic!(
                "missing dedicated Sample Gallery page {} for {id:?}: {e}",
                page_path.display()
            )
        });
        assert!(
            !page_body.trim().is_empty(),
            "Sample Gallery page {} must not be empty",
            page_path.display()
        );

        assert_valid_png(&images_dir.join(slug).join("hero.png"));
        assert_valid_png(&images_dir.join(slug).join("thumb.png"));

        let page_link = format!("]({slug}.md)");
        assert!(
            gallery_index.contains(&page_link),
            "{} must link to '{slug}.md' via '{page_link}'",
            gallery_index_path.display()
        );
        let thumb_link = format!("../images/samples/{slug}/thumb.png");
        assert!(
            gallery_index.contains(&thumb_link),
            "{} must reference thumbnail '{thumb_link}'",
            gallery_index_path.display()
        );
    }

    assert_eq!(
        seen_slugs.len(),
        SAMPLE_CATALOG.len() + 1,
        "expected SAMPLE_CATALOG ({}) + revision-v2 = {} unique documented samples",
        SAMPLE_CATALOG.len(),
        SAMPLE_CATALOG.len() + 1
    );
}

/// Extract all Markdown `[label](target)` and `![alt](target)` destinations outside fenced code
/// blocks and inline code spans, returning `(1-based line number, raw target)`.
fn extract_markdown_link_targets(markdown: &str) -> Vec<(usize, String)> {
    let mut results = Vec::new();
    let mut in_fence = false;

    for (zero_line, line) in markdown.lines().enumerate() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("```") {
            in_fence = !in_fence;
            continue;
        }
        if in_fence {
            continue;
        }

        let bytes = line.as_bytes();
        let len = bytes.len();
        let mut i = 0;
        let mut in_inline_code = false;

        while i < len {
            if bytes[i] == b'`' {
                in_inline_code = !in_inline_code;
                i += 1;
                continue;
            }
            if in_inline_code {
                i += 1;
                continue;
            }
            if bytes[i] == b']' && i + 1 < len && bytes[i + 1] == b'(' {
                let start = i + 2;
                let mut depth = 1usize;
                let mut j = start;
                while j < len && depth > 0 {
                    match bytes[j] {
                        b'(' => depth += 1,
                        b')' => depth -= 1,
                        _ => {}
                    }
                    if depth > 0 {
                        j += 1;
                    }
                }
                if depth == 0 && j > start {
                    let raw = line[start..j].trim();
                    if !raw.is_empty() {
                        results.push((zero_line + 1, raw.to_string()));
                    }
                    i = j + 1;
                    continue;
                }
            }
            i += 1;
        }
    }
    results
}

fn normalize_local_target(raw: &str) -> Option<&str> {
    let trimmed = raw.trim();
    // Strip optional Markdown title: `path/to/file.md "Optional Title"`
    let dest = if let Some(inner) = trimmed.strip_prefix('<') {
        inner.split('>').next().unwrap_or(inner).trim()
    } else {
        trimmed.split_whitespace().next().unwrap_or(trimmed)
    };

    if dest.is_empty()
        || dest.starts_with('#')
        || dest.starts_with("http://")
        || dest.starts_with("https://")
        || dest.starts_with("mailto:")
    {
        return None;
    }

    // Strip `#fragment` and `?query` suffixes
    let without_frag = dest.split('#').next().unwrap_or(dest);
    let without_query = without_frag.split('?').next().unwrap_or(without_frag);
    if without_query.is_empty() {
        None
    } else {
        Some(without_query)
    }
}

fn collect_markdown_files_in_dir(dir: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    let entries = fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("failed to list directory {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.expect("failed to read dir entry").path();
        if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("md") {
            files.push(path);
        }
    }
    files.sort();
    files
}

#[test]
fn all_markdown_links_and_images_resolve_on_disk() {
    let root = repo_root();
    let mut md_files = vec![root.join("README.md")];
    md_files.extend(collect_markdown_files_in_dir(&root.join("docs/tutorials")));
    md_files.extend(collect_markdown_files_in_dir(&root.join("docs/samples")));

    assert!(
        md_files.len() >= 45,
        "expected README.md + tutorials + 24 sample docs, found only {} markdown files",
        md_files.len()
    );

    let mut broken = Vec::new();
    let mut checked_links = 0usize;

    for md_path in &md_files {
        let content = fs::read_to_string(md_path)
            .unwrap_or_else(|e| panic!("failed to read {}: {e}", md_path.display()));
        let base_dir = md_path
            .parent()
            .expect("markdown file must have a parent directory");

        for (line_no, raw_target) in extract_markdown_link_targets(&content) {
            let Some(local_rel) = normalize_local_target(&raw_target) else {
                continue;
            };
            checked_links += 1;
            let resolved = base_dir.join(local_rel);
            if !resolved.exists() {
                let rel_doc = md_path
                    .strip_prefix(&root)
                    .unwrap_or(md_path)
                    .display()
                    .to_string();
                broken.push(format!(
                    "{rel_doc}:{line_no} -> '{local_rel}' (resolved: {})",
                    resolved.display()
                ));
            }
        }
    }

    assert!(
        checked_links > 100,
        "expected to audit > 100 relative Markdown links/images, checked only {checked_links}"
    );
    assert!(
        broken.is_empty(),
        "found {} broken relative Markdown link(s)/image(s):\n  {}",
        broken.len(),
        broken.join("\n  ")
    );
}

#[test]
fn all_tutorial_starter_assets_are_documented_in_tutorials_readme() {
    let root = repo_root();
    let assets_dir = root.join("docs/tutorials/assets");
    let tutorials_readme_path = root.join("docs/tutorials/README.md");
    let tutorials_readme = fs::read_to_string(&tutorials_readme_path)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", tutorials_readme_path.display()));

    let mut assets = Vec::new();
    for entry in fs::read_dir(&assets_dir)
        .unwrap_or_else(|e| panic!("failed to read {}: {e}", assets_dir.display()))
    {
        let path = entry.expect("failed to read asset entry").path();
        if path.is_file() {
            let name = path
                .file_name()
                .and_then(|s| s.to_str())
                .expect("asset filename must be valid UTF-8")
                .to_string();
            if !name.starts_with('.') {
                assets.push(name);
            }
        }
    }
    assets.sort();

    assert!(
        assets.len() >= 20,
        "expected >= 20 checked-in starter assets under {}, found {}",
        assets_dir.display(),
        assets.len()
    );

    let mut missing = Vec::new();
    for asset in &assets {
        let needle = format!("assets/{asset}");
        if !tutorials_readme.contains(&needle) {
            missing.push(needle);
        }
    }

    assert!(
        missing.is_empty(),
        "the following starter assets in docs/tutorials/assets/ are not documented in docs/tutorials/README.md:\n  {}",
        missing.join("\n  ")
    );
}
