use chip_math::DVec3;

use crate::{
    kernel::CadError,
    mesh::{MeshFormat, RawMeshData},
};

/// Parses a binary or ASCII STL byte buffer into raw triangles.
///
/// Reliably handles binary STLs whose 80-byte header happens to begin with `"solid"` by checking
/// the exact binary record size (`84 + 50 * tri_count == bytes.len()`) and verifying ASCII syntax
/// before choosing the ASCII parser.
pub(crate) fn parse_stl(source_name: &str, bytes: &[u8]) -> Result<RawMeshData, CadError> {
    if bytes.is_empty() {
        return Err(CadError::EmptyInput);
    }

    let looks_like_exact_binary = if bytes.len() >= 84 {
        let count_bytes: [u8; 4] = [bytes[80], bytes[81], bytes[82], bytes[83]];
        let tri_count = u32::from_le_bytes(count_bytes) as usize;
        tri_count
            .checked_mul(50)
            .and_then(|n| n.checked_add(84))
            .is_some_and(|expected| expected == bytes.len())
    } else {
        false
    };

    let looks_like_ascii = looks_like_ascii_stl(bytes);

    if looks_like_ascii && !looks_like_exact_binary {
        parse_ascii_stl(source_name, bytes)
    } else if looks_like_exact_binary {
        // Guard against the astronomical coincidence where a valid ASCII STL happens to have
        // `len == 84 + 50 * k`: if it parses cleanly as ASCII STL, prefer ASCII!
        if looks_like_ascii && let Ok(ascii) = parse_ascii_stl(source_name, bytes) {
            return Ok(ascii);
        }
        parse_binary_stl(source_name, bytes)
    } else if looks_like_ascii {
        parse_ascii_stl(source_name, bytes)
    } else {
        Err(CadError::MeshParse {
            format: "STL",
            detail: format!(
                "file size ({} bytes) does not match binary STL layout (`84 + 50 * N`) and is not valid ASCII STL",
                bytes.len()
            ),
        })
    }
}

fn looks_like_ascii_stl(bytes: &[u8]) -> bool {
    let trimmed = skip_leading_ascii_ws(bytes);
    if trimmed.len() < 5 || !trimmed[..5].eq_ignore_ascii_case(b"solid") {
        return false;
    }
    let probe_len = bytes.len().min(4096);
    let Ok(prefix) = std::str::from_utf8(&bytes[..probe_len]) else {
        return false;
    };
    let lower = prefix.to_ascii_lowercase();
    lower.contains("facet") && lower.contains("vertex")
}

fn skip_leading_ascii_ws(bytes: &[u8]) -> &[u8] {
    let mut i = 0usize;
    // Skip UTF-8 BOM if present
    if bytes.starts_with(&[0xEF, 0xBB, 0xBF]) {
        i = 3;
    }
    while i < bytes.len() && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    &bytes[i..]
}

fn parse_binary_stl(source_name: &str, bytes: &[u8]) -> Result<RawMeshData, CadError> {
    if bytes.len() < 84 {
        return Err(CadError::MeshParse {
            format: "Binary STL",
            detail: "file shorter than 84-byte header".to_string(),
        });
    }
    let tri_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let expected = tri_count
        .checked_mul(50)
        .and_then(|n| n.checked_add(84))
        .ok_or_else(|| CadError::MeshParse {
            format: "Binary STL",
            detail: "triangle count overflow".to_string(),
        })?;
    if bytes.len() < expected {
        return Err(CadError::MeshParse {
            format: "Binary STL",
            detail: format!(
                "truncated binary STL: expected {expected} bytes for {tri_count} triangles, found {}",
                bytes.len()
            ),
        });
    }

    let mut raw_triangles = Vec::with_capacity(tri_count);
    for idx in 0..tri_count {
        let base = 84 + idx * 50;
        // Bytes [0..12] are facet normal (ignored; exact f64 geometric normal computed from vertices).
        let v0 = read_f32_vec3(&bytes[base + 12..base + 24])?;
        let v1 = read_f32_vec3(&bytes[base + 24..base + 36])?;
        let v2 = read_f32_vec3(&bytes[base + 36..base + 48])?;
        raw_triangles.push([v0, v1, v2]);
    }

    let name = stem_or_default(source_name, "STL Mesh");
    Ok(RawMeshData {
        name,
        format: MeshFormat::StlBinary,
        detected_unit: None,
        raw_triangles,
    })
}

fn read_f32_vec3(slice: &[u8]) -> Result<DVec3, CadError> {
    if slice.len() < 12 {
        return Err(CadError::MeshParse {
            format: "Binary STL",
            detail: "unexpected EOF reading vertex coordinates".to_string(),
        });
    }
    let x = f32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]) as f64;
    let y = f32::from_le_bytes([slice[4], slice[5], slice[6], slice[7]]) as f64;
    let z = f32::from_le_bytes([slice[8], slice[9], slice[10], slice[11]]) as f64;
    if !x.is_finite() || !y.is_finite() || !z.is_finite() {
        return Err(CadError::MeshParse {
            format: "Binary STL",
            detail: "non-finite (NaN/Inf) vertex coordinate encountered".to_string(),
        });
    }
    Ok(DVec3::new(x, y, z))
}

fn parse_ascii_stl(source_name: &str, bytes: &[u8]) -> Result<RawMeshData, CadError> {
    let text = std::str::from_utf8(bytes).map_err(|e| CadError::MeshParse {
        format: "ASCII STL",
        detail: format!("invalid UTF-8: {e}"),
    })?;

    let mut solid_name: Option<String> = None;
    let mut raw_triangles: Vec<[DVec3; 3]> = Vec::new();
    let mut current_loop: Vec<DVec3> = Vec::with_capacity(4);

    for (line_no, raw_line) in text.lines().enumerate() {
        let line = raw_line.trim().trim_start_matches('\u{feff}');
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(keyword) = parts.next() else {
            continue;
        };
        if keyword.eq_ignore_ascii_case("solid") {
            if solid_name.is_none() {
                let rest = line[keyword.len()..].trim();
                if !rest.is_empty() {
                    solid_name = Some(rest.to_string());
                }
            }
        } else if keyword.eq_ignore_ascii_case("outer") {
            current_loop.clear();
        } else if keyword.eq_ignore_ascii_case("vertex") {
            let (Some(sx), Some(sy), Some(sz)) = (parts.next(), parts.next(), parts.next()) else {
                return Err(CadError::MeshParse {
                    format: "ASCII STL",
                    detail: format!(
                        "line {}: expected 3 coordinates after `vertex`",
                        line_no + 1
                    ),
                });
            };
            let x: f64 = sx.parse().map_err(|_| CadError::MeshParse {
                format: "ASCII STL",
                detail: format!("line {}: invalid X float '{sx}'", line_no + 1),
            })?;
            let y: f64 = sy.parse().map_err(|_| CadError::MeshParse {
                format: "ASCII STL",
                detail: format!("line {}: invalid Y float '{sy}'", line_no + 1),
            })?;
            let z: f64 = sz.parse().map_err(|_| CadError::MeshParse {
                format: "ASCII STL",
                detail: format!("line {}: invalid Z float '{sz}'", line_no + 1),
            })?;
            if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                return Err(CadError::MeshParse {
                    format: "ASCII STL",
                    detail: format!("line {}: non-finite vertex coordinate", line_no + 1),
                });
            }
            current_loop.push(DVec3::new(x, y, z));
        } else if keyword.eq_ignore_ascii_case("endloop") {
            if current_loop.len() >= 3 {
                // Fan triangulate in case an ASCII STL facet contains > 3 vertices
                let v0 = current_loop[0];
                for i in 1..(current_loop.len() - 1) {
                    raw_triangles.push([v0, current_loop[i], current_loop[i + 1]]);
                }
            }
            current_loop.clear();
        }
    }

    if raw_triangles.is_empty() {
        return Err(CadError::MeshParse {
            format: "ASCII STL",
            detail: "no valid triangles parsed from ASCII STL".to_string(),
        });
    }

    let name = solid_name.unwrap_or_else(|| stem_or_default(source_name, "STL Mesh"));
    Ok(RawMeshData {
        name,
        format: MeshFormat::StlAscii,
        detected_unit: None,
        raw_triangles,
    })
}

pub use chip_math::encode_binary_stl;

fn stem_or_default(source_name: &str, fallback: &str) -> String {
    std::path::Path::new(source_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .filter(|s| !s.trim().is_empty())
        .map(ToOwned::to_owned)
        .unwrap_or_else(|| fallback.to_string())
}
