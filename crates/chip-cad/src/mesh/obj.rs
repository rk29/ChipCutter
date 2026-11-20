use chip_math::DVec3;

use crate::{
    kernel::CadError,
    mesh::{MeshFormat, MeshUnitKind, RawMeshData},
};

/// Parses a Wavefront `.obj` mesh into raw triangles (`f64` in source file units).
///
/// Supports `v`, `f` (`v`, `v/vt`, `v//vn`, `v/vt/vn`, negative relative indices), fan triangulation
/// for quads & n-gons, `o`/`g` object naming, and optional `# units: <mm|in|cm|m>` metadata comments.
pub(crate) fn parse_obj(source_name: &str, bytes: &[u8]) -> Result<RawMeshData, CadError> {
    if bytes.is_empty() {
        return Err(CadError::EmptyInput);
    }
    let text = std::str::from_utf8(bytes).map_err(|e| CadError::MeshParse {
        format: "OBJ",
        detail: format!("invalid UTF-8: {e}"),
    })?;

    let mut positions: Vec<DVec3> = Vec::new();
    let mut raw_triangles: Vec<[DVec3; 3]> = Vec::new();
    let mut detected_unit: Option<MeshUnitKind> = None;
    let mut object_name: Option<String> = None;
    let mut face_verts: Vec<DVec3> = Vec::with_capacity(8);

    for (line_no, raw_line) in text.lines().enumerate() {
        let line = raw_line.trim().trim_start_matches('\u{feff}');
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if detected_unit.is_none() {
                detected_unit = parse_obj_unit_comment(comment);
            }
            continue;
        }

        // Strip trailing inline `# ...` comments
        let code = match line.find('#') {
            Some(pos) => line[..pos].trim(),
            None => line,
        };
        if code.is_empty() {
            continue;
        }

        let mut parts = code.split_whitespace();
        let Some(tag) = parts.next() else {
            continue;
        };

        match tag {
            "v" => {
                let (Some(sx), Some(sy), Some(sz)) = (parts.next(), parts.next(), parts.next())
                else {
                    return Err(CadError::MeshParse {
                        format: "OBJ",
                        detail: format!("line {}: expected 3 coordinates after `v`", line_no + 1),
                    });
                };
                let x: f64 = sx.parse().map_err(|_| CadError::MeshParse {
                    format: "OBJ",
                    detail: format!("line {}: invalid vertex X '{sx}'", line_no + 1),
                })?;
                let y: f64 = sy.parse().map_err(|_| CadError::MeshParse {
                    format: "OBJ",
                    detail: format!("line {}: invalid vertex Y '{sy}'", line_no + 1),
                })?;
                let z: f64 = sz.parse().map_err(|_| CadError::MeshParse {
                    format: "OBJ",
                    detail: format!("line {}: invalid vertex Z '{sz}'", line_no + 1),
                })?;
                if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                    return Err(CadError::MeshParse {
                        format: "OBJ",
                        detail: format!("line {}: non-finite vertex coordinate", line_no + 1),
                    });
                }
                positions.push(DVec3::new(x, y, z));
            }
            "f" => {
                face_verts.clear();
                for token in parts {
                    let v_tok = token.split('/').next().unwrap_or_default();
                    let raw_idx: i64 = v_tok.parse().map_err(|_| CadError::MeshParse {
                        format: "OBJ",
                        detail: format!(
                            "line {}: invalid face vertex index '{token}'",
                            line_no + 1
                        ),
                    })?;
                    let resolved = resolve_obj_index(raw_idx, positions.len()).ok_or_else(|| {
                        CadError::MeshParse {
                            format: "OBJ",
                            detail: format!(
                                "line {}: face vertex index {raw_idx} out of range (have {} vertices)",
                                line_no + 1,
                                positions.len()
                            ),
                        }
                    })?;
                    face_verts.push(positions[resolved]);
                }
                if face_verts.len() >= 3 {
                    let v0 = face_verts[0];
                    for i in 1..(face_verts.len() - 1) {
                        raw_triangles.push([v0, face_verts[i], face_verts[i + 1]]);
                    }
                }
            }
            "o" | "g" if object_name.is_none() => {
                let rest = code[tag.len()..].trim();
                if !rest.is_empty() {
                    object_name = Some(rest.to_string());
                }
            }
            _ => {
                // Ignore `vt`, `vn`, `vp`, `mtllib`, `usemtl`, `s`, `l`, etc.
            }
        }
    }

    if raw_triangles.is_empty() {
        return Err(CadError::MeshParse {
            format: "OBJ",
            detail: "no valid triangular faces found in OBJ file".to_string(),
        });
    }

    let name = object_name.unwrap_or_else(|| {
        std::path::Path::new(source_name)
            .file_stem()
            .and_then(|s| s.to_str())
            .filter(|s| !s.trim().is_empty())
            .map(ToOwned::to_owned)
            .unwrap_or_else(|| "OBJ Mesh".to_string())
    });

    Ok(RawMeshData {
        name,
        format: MeshFormat::Obj,
        detected_unit,
        raw_triangles,
    })
}

fn resolve_obj_index(raw_idx: i64, vertex_count: usize) -> Option<usize> {
    if raw_idx > 0 {
        let idx = (raw_idx - 1) as usize;
        (idx < vertex_count).then_some(idx)
    } else if raw_idx < 0 {
        let back = (-raw_idx) as usize;
        vertex_count.checked_sub(back)
    } else {
        None
    }
}

fn parse_obj_unit_comment(comment: &str) -> Option<MeshUnitKind> {
    let lower = comment.trim().to_ascii_lowercase();
    for prefix in ["units:", "unit:", "units =", "unit ="] {
        if let Some(pos) = lower.find(prefix) {
            let after = lower[pos + prefix.len()..].trim();
            if let Some(first_word) = after.split(|c: char| !c.is_ascii_alphanumeric()).next()
                && let Some(unit) = MeshUnitKind::from_str_loose(first_word)
            {
                return Some(unit);
            }
        }
    }
    None
}
