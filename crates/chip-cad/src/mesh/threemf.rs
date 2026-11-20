use std::{
    collections::HashMap,
    io::Read as _,
    path::{Component, Path},
};

use chip_math::DVec3;
use zip::ZipArchive;

use crate::{
    kernel::CadError,
    mesh::{MeshFormat, MeshUnitKind, RawMeshData},
};

/// Maximum allowed total uncompressed bytes read from a 3MF OPC ZIP archive (`128 MiB`).
pub const MAX_MESH_UNPACK_BYTES: u64 = 128 * 1024 * 1024;
/// Maximum allowed ZIP entries inside a 3MF container.
pub const MAX_3MF_ARCHIVE_ENTRIES: usize = 1_024;
/// Maximum allowed instantiated triangles emitted from a 3MF `<build>` / `<components>` DAG.
pub const MAX_3MF_TRIANGLES: usize = 5_000_000;

const MAX_COMPONENT_RECURSION_DEPTH: usize = 32;

/// 3MF 3×4 affine matrix acting on row vectors `[x y z 1] * M`.
#[derive(Debug, Clone, Copy)]
struct ThreeMfMat {
    /// Columns: `[m00, m01, m02, m10, m11, m12, m20, m21, m22, m30, m31, m32]`
    m: [f64; 12],
}

impl ThreeMfMat {
    const IDENTITY: Self = Self {
        m: [
            1.0, 0.0, 0.0, // row 0
            0.0, 1.0, 0.0, // row 1
            0.0, 0.0, 1.0, // row 2
            0.0, 0.0, 0.0, // translation row 3
        ],
    };

    fn parse(raw: &str) -> Result<Self, CadError> {
        let vals: Vec<f64> = raw
            .split_whitespace()
            .map(|s| {
                s.parse::<f64>().map_err(|_| CadError::MeshParse {
                    format: "3MF",
                    detail: format!("invalid float '{s}' in 3MF transform matrix"),
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        if vals.len() != 12 || vals.iter().any(|v| !v.is_finite()) {
            return Err(CadError::MeshParse {
                format: "3MF",
                detail: format!(
                    "expected 12 finite floats in 3MF transform matrix, found {}",
                    vals.len()
                ),
            });
        }
        let mut m = [0.0f64; 12];
        m.copy_from_slice(&vals);
        Ok(Self { m })
    }

    /// Composes `self` followed by `parent` (row-vector convention: `(p * self) * parent`).
    fn then(&self, parent: &Self) -> Self {
        let a = &self.m;
        let b = &parent.m;
        let mul_row = |r: usize, c: usize| -> f64 {
            a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c]
        };
        let tx = a[9] * b[0] + a[10] * b[3] + a[11] * b[6] + b[9];
        let ty = a[9] * b[1] + a[10] * b[4] + a[11] * b[7] + b[10];
        let tz = a[9] * b[2] + a[10] * b[5] + a[11] * b[8] + b[11];
        Self {
            m: [
                mul_row(0, 0),
                mul_row(0, 1),
                mul_row(0, 2),
                mul_row(1, 0),
                mul_row(1, 1),
                mul_row(1, 2),
                mul_row(2, 0),
                mul_row(2, 1),
                mul_row(2, 2),
                tx,
                ty,
                tz,
            ],
        }
    }

    fn transform_point(&self, p: DVec3) -> DVec3 {
        let m = &self.m;
        DVec3::new(
            p.x * m[0] + p.y * m[3] + p.z * m[6] + m[9],
            p.x * m[1] + p.y * m[4] + p.z * m[7] + m[10],
            p.x * m[2] + p.y * m[5] + p.z * m[8] + m[11],
        )
    }

    fn determinant(&self) -> f64 {
        let m = &self.m;
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6])
    }
}

struct ThreeMfComponent {
    object_id: u32,
    transform: ThreeMfMat,
}

struct ThreeMfObject {
    name: Option<String>,
    vertices: Vec<DVec3>,
    triangles: Vec<[usize; 3]>,
    components: Vec<ThreeMfComponent>,
}

struct ThreeMfBuildItem {
    object_id: u32,
    transform: ThreeMfMat,
}

/// Parses a 3MF OPC ZIP archive into raw triangles (`f64` in declared 3MF model units).
pub(crate) fn parse_3mf(source_name: &str, bytes: &[u8]) -> Result<RawMeshData, CadError> {
    if bytes.is_empty() {
        return Err(CadError::EmptyInput);
    }

    let cursor = std::io::Cursor::new(bytes);
    let mut archive = ZipArchive::new(cursor).map_err(|e| CadError::MeshParse {
        format: "3MF",
        detail: format!("invalid ZIP container: {e}"),
    })?;

    // 1. Validate every entry path against zip-slip / symlinks and total unpack size ceiling.
    if archive.len() > MAX_3MF_ARCHIVE_ENTRIES {
        return Err(CadError::ArchiveTooLarge {
            limit_bytes: MAX_MESH_UNPACK_BYTES,
        });
    }
    let mut entry_names: Vec<String> = Vec::with_capacity(archive.len());
    let mut total_uncompressed: u64 = 0;
    for idx in 0..archive.len() {
        let entry = archive.by_index(idx).map_err(|e| CadError::MeshParse {
            format: "3MF",
            detail: format!("failed to inspect archive entry #{idx}: {e}"),
        })?;
        let raw_name = entry
            .name()
            .map_err(|e| CadError::MeshParse {
                format: "3MF",
                detail: format!("non-UTF-8 entry path in 3MF archive: {e}"),
            })?
            .into_owned();
        if entry.is_symlink() {
            return Err(CadError::UnsafeArchivePath {
                entry_path: raw_name,
            });
        }
        validate_safe_archive_path(&raw_name)?;
        total_uncompressed = total_uncompressed.saturating_add(entry.size());
        if total_uncompressed > MAX_MESH_UNPACK_BYTES {
            return Err(CadError::ArchiveTooLarge {
                limit_bytes: MAX_MESH_UNPACK_BYTES,
            });
        }
        entry_names.push(raw_name);
    }

    // 2. Discover primary 3D model path via `_rels/.rels`, falling back to `3D/3dmodel.model`.
    let mut actual_bytes_read: u64 = 0;
    let mut model_target: Option<String> = None;
    if let Some(rels_name) = entry_names
        .iter()
        .find(|n| n.eq_ignore_ascii_case("_rels/.rels"))
    {
        let rels_xml = read_archive_entry_utf8(&mut archive, rels_name, &mut actual_bytes_read)?;
        model_target = extract_3dmodel_target_from_rels(&rels_xml);
    }

    let resolved_model_entry = model_target
        .and_then(|target| {
            let trimmed = target.trim_start_matches('/');
            entry_names
                .iter()
                .find(|n| n.eq_ignore_ascii_case(trimmed))
                .cloned()
        })
        .or_else(|| {
            entry_names
                .iter()
                .find(|n| n.eq_ignore_ascii_case("3d/3dmodel.model"))
                .cloned()
        })
        .or_else(|| {
            entry_names
                .iter()
                .find(|n| n.to_ascii_lowercase().ends_with(".model"))
                .cloned()
        })
        .ok_or_else(|| CadError::MeshParse {
            format: "3MF",
            detail: "3MF archive does not contain a `.model` XML payload".to_string(),
        })?;

    let model_xml =
        read_archive_entry_utf8(&mut archive, &resolved_model_entry, &mut actual_bytes_read)?;
    parse_3mf_model_xml(source_name, &model_xml)
}

fn validate_safe_archive_path(raw: &str) -> Result<(), CadError> {
    if raw.contains('\\') || raw.starts_with('/') || raw.trim().is_empty() {
        return Err(CadError::UnsafeArchivePath {
            entry_path: raw.to_string(),
        });
    }
    for comp in Path::new(raw).components() {
        match comp {
            Component::Normal(seg) => {
                let s = seg.to_string_lossy();
                if s.is_empty() || s == "." || s == ".." {
                    return Err(CadError::UnsafeArchivePath {
                        entry_path: raw.to_string(),
                    });
                }
            }
            Component::CurDir
            | Component::ParentDir
            | Component::RootDir
            | Component::Prefix(_) => {
                return Err(CadError::UnsafeArchivePath {
                    entry_path: raw.to_string(),
                });
            }
        }
    }
    Ok(())
}

fn read_archive_entry_utf8(
    archive: &mut ZipArchive<std::io::Cursor<&[u8]>>,
    entry_name: &str,
    actual_bytes_read: &mut u64,
) -> Result<String, CadError> {
    let mut entry = archive
        .by_name(entry_name)
        .map_err(|e| CadError::MeshParse {
            format: "3MF",
            detail: format!("failed to open archive entry '{entry_name}': {e}"),
        })?;
    let remaining = MAX_MESH_UNPACK_BYTES.saturating_sub(*actual_bytes_read);
    let mut buf = Vec::with_capacity((entry.size().min(remaining) as usize).min(16 * 1024 * 1024));
    entry
        .by_ref()
        .take(remaining.saturating_add(1))
        .read_to_end(&mut buf)
        .map_err(CadError::Io)?;
    if buf.len() as u64 > remaining {
        return Err(CadError::ArchiveTooLarge {
            limit_bytes: MAX_MESH_UNPACK_BYTES,
        });
    }
    *actual_bytes_read = actual_bytes_read.saturating_add(buf.len() as u64);
    String::from_utf8(buf).map_err(|e| CadError::MeshParse {
        format: "3MF",
        detail: format!("entry '{entry_name}' is not valid UTF-8 XML: {e}"),
    })
}

/// Scans `<Relationship Target="..." Type="...3dmodel"/>` tags in `_rels/.rels`.
fn extract_3dmodel_target_from_rels(xml: &str) -> Option<String> {
    for tag in XmlTagIter::new(xml) {
        if tag.local_name.eq_ignore_ascii_case("relationship") && !tag.is_closing {
            let rel_type = tag.attr("type").unwrap_or_default();
            if rel_type.to_ascii_lowercase().contains("3dmodel")
                && let Some(target) = tag.attr("target")
            {
                return Some(target.to_string());
            }
        }
    }
    None
}

fn parse_3mf_model_xml(source_name: &str, xml: &str) -> Result<RawMeshData, CadError> {
    // 3MF spec default unit when `<model unit="...">` is omitted is `millimeter`.
    let mut detected_unit = MeshUnitKind::Millimeter;
    let mut objects: HashMap<u32, ThreeMfObject> = HashMap::new();
    let mut object_order: Vec<u32> = Vec::new();
    let mut build_items: Vec<ThreeMfBuildItem> = Vec::new();
    let mut current_obj_id: Option<u32> = None;

    for tag in XmlTagIter::new(xml) {
        let name = tag.local_name.as_str();
        if tag.is_closing {
            if name.eq_ignore_ascii_case("object") {
                current_obj_id = None;
            }
            continue;
        }

        if name.eq_ignore_ascii_case("model") {
            if let Some(u_str) = tag.attr("unit")
                && let Some(u) = MeshUnitKind::from_str_loose(u_str)
            {
                detected_unit = u;
            }
        } else if name.eq_ignore_ascii_case("object") {
            let id_str = tag.attr("id").ok_or_else(|| CadError::MeshParse {
                format: "3MF",
                detail: "<object> missing required `id` attribute".to_string(),
            })?;
            let id: u32 = id_str.parse().map_err(|_| CadError::MeshParse {
                format: "3MF",
                detail: format!("invalid <object id='{id_str}'>"),
            })?;
            let obj_name = tag.attr("name").map(ToOwned::to_owned);
            objects.insert(
                id,
                ThreeMfObject {
                    name: obj_name,
                    vertices: Vec::new(),
                    triangles: Vec::new(),
                    components: Vec::new(),
                },
            );
            object_order.push(id);
            if !tag.is_self_closing {
                current_obj_id = Some(id);
            }
        } else if name.eq_ignore_ascii_case("vertex") {
            if let Some(oid) = current_obj_id
                && let Some(obj) = objects.get_mut(&oid)
            {
                let x = parse_xml_f64(&tag, "x")?;
                let y = parse_xml_f64(&tag, "y")?;
                let z = parse_xml_f64(&tag, "z")?;
                obj.vertices.push(DVec3::new(x, y, z));
            }
        } else if name.eq_ignore_ascii_case("triangle") {
            if let Some(oid) = current_obj_id
                && let Some(obj) = objects.get_mut(&oid)
            {
                let v1 = parse_xml_usize(&tag, "v1")?;
                let v2 = parse_xml_usize(&tag, "v2")?;
                let v3 = parse_xml_usize(&tag, "v3")?;
                obj.triangles.push([v1, v2, v3]);
            }
        } else if name.eq_ignore_ascii_case("component") {
            if let Some(oid) = current_obj_id
                && let Some(obj) = objects.get_mut(&oid)
            {
                let ref_id_str = tag.attr("objectid").ok_or_else(|| CadError::MeshParse {
                    format: "3MF",
                    detail: "<component> missing required `objectid` attribute".to_string(),
                })?;
                let ref_id: u32 = ref_id_str.parse().map_err(|_| CadError::MeshParse {
                    format: "3MF",
                    detail: format!("invalid <component objectid='{ref_id_str}'>"),
                })?;
                let transform = match tag.attr("transform") {
                    Some(t_str) => ThreeMfMat::parse(t_str)?,
                    None => ThreeMfMat::IDENTITY,
                };
                obj.components.push(ThreeMfComponent {
                    object_id: ref_id,
                    transform,
                });
            }
        } else if name.eq_ignore_ascii_case("item")
            && let Some(ref_id_str) = tag.attr("objectid")
        {
            let ref_id: u32 = ref_id_str.parse().map_err(|_| CadError::MeshParse {
                format: "3MF",
                detail: format!("invalid <item objectid='{ref_id_str}'>"),
            })?;
            let transform = match tag.attr("transform") {
                Some(t_str) => ThreeMfMat::parse(t_str)?,
                None => ThreeMfMat::IDENTITY,
            };
            build_items.push(ThreeMfBuildItem {
                object_id: ref_id,
                transform,
            });
        }
    }

    let mut raw_triangles: Vec<[DVec3; 3]> = Vec::new();
    let mut first_obj_name: Option<String> = None;

    if build_items.is_empty() {
        // Fallback when `<build>` is empty: instantiate all top-level mesh objects at identity.
        for &oid in &object_order {
            if let Some(obj) = objects.get(&oid)
                && first_obj_name.is_none()
            {
                first_obj_name.clone_from(&obj.name);
            }
            emit_object_triangles(oid, &ThreeMfMat::IDENTITY, &objects, &mut raw_triangles, 0)?;
        }
    } else {
        for item in &build_items {
            if let Some(obj) = objects.get(&item.object_id)
                && first_obj_name.is_none()
            {
                first_obj_name.clone_from(&obj.name);
            }
            emit_object_triangles(
                item.object_id,
                &item.transform,
                &objects,
                &mut raw_triangles,
                0,
            )?;
        }
    }

    if raw_triangles.is_empty() {
        return Err(CadError::MeshParse {
            format: "3MF",
            detail: "3MF model contains zero triangles".to_string(),
        });
    }

    let name = first_obj_name.unwrap_or_else(|| {
        std::path::Path::new(source_name)
            .file_stem()
            .and_then(|s| s.to_str())
            .filter(|s| !s.trim().is_empty())
            .map(ToOwned::to_owned)
            .unwrap_or_else(|| "3MF Mesh".to_string())
    });

    Ok(RawMeshData {
        name,
        format: MeshFormat::ThreeMf,
        detected_unit: Some(detected_unit),
        raw_triangles,
    })
}

fn emit_object_triangles(
    object_id: u32,
    world_mat: &ThreeMfMat,
    objects: &HashMap<u32, ThreeMfObject>,
    out: &mut Vec<[DVec3; 3]>,
    depth: usize,
) -> Result<(), CadError> {
    if depth > MAX_COMPONENT_RECURSION_DEPTH {
        return Err(CadError::MeshParse {
            format: "3MF",
            detail: "cyclic or excessively deep 3MF <component> hierarchy".to_string(),
        });
    }
    let Some(obj) = objects.get(&object_id) else {
        return Err(CadError::MeshParse {
            format: "3MF",
            detail: format!("referenced objectid='{object_id}' not found in <resources>"),
        });
    };

    if out.len().saturating_add(obj.triangles.len()) > MAX_3MF_TRIANGLES {
        return Err(CadError::MeshParse {
            format: "3MF",
            detail: format!(
                "instantiated 3MF triangle count exceeds maximum allowed limit ({MAX_3MF_TRIANGLES})"
            ),
        });
    }

    let flip_winding = world_mat.determinant() < 0.0;
    for &[i0, i1, i2] in &obj.triangles {
        let (Some(&p0), Some(&p1), Some(&p2)) = (
            obj.vertices.get(i0),
            obj.vertices.get(i1),
            obj.vertices.get(i2),
        ) else {
            return Err(CadError::MeshParse {
                format: "3MF",
                detail: format!(
                    "triangle vertex index [{i0}, {i1}, {i2}] out of bounds ({} vertices) in object {object_id}",
                    obj.vertices.len()
                ),
            });
        };
        let v0 = world_mat.transform_point(p0);
        let v1 = world_mat.transform_point(p1);
        let v2 = world_mat.transform_point(p2);
        if flip_winding {
            out.push([v0, v2, v1]);
        } else {
            out.push([v0, v1, v2]);
        }
    }

    for comp in &obj.components {
        let combined = comp.transform.then(world_mat);
        emit_object_triangles(comp.object_id, &combined, objects, out, depth + 1)?;
    }
    Ok(())
}

fn parse_xml_f64(tag: &XmlTag, attr_name: &str) -> Result<f64, CadError> {
    let raw = tag.attr(attr_name).ok_or_else(|| CadError::MeshParse {
        format: "3MF",
        detail: format!(
            "<{}> missing required attribute `{attr_name}`",
            tag.local_name
        ),
    })?;
    let val: f64 = raw.parse().map_err(|_| CadError::MeshParse {
        format: "3MF",
        detail: format!(
            "invalid float `{attr_name}='{raw}'` in <{}>",
            tag.local_name
        ),
    })?;
    if !val.is_finite() {
        return Err(CadError::MeshParse {
            format: "3MF",
            detail: format!("non-finite float `{attr_name}='{raw}'`"),
        });
    }
    Ok(val)
}

fn parse_xml_usize(tag: &XmlTag, attr_name: &str) -> Result<usize, CadError> {
    let raw = tag.attr(attr_name).ok_or_else(|| CadError::MeshParse {
        format: "3MF",
        detail: format!(
            "<{}> missing required attribute `{attr_name}`",
            tag.local_name
        ),
    })?;
    raw.parse().map_err(|_| CadError::MeshParse {
        format: "3MF",
        detail: format!(
            "invalid integer `{attr_name}='{raw}'` in <{}>",
            tag.local_name
        ),
    })
}

/// Minimal, zero-dependency XML tag scanner for well-formed 3MF `.model` and `.rels` files.
struct XmlTag {
    local_name: String,
    is_closing: bool,
    is_self_closing: bool,
    attrs: Vec<(String, String)>,
}

impl XmlTag {
    fn attr(&self, name: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(name))
            .map(|(_, v)| v.as_str())
    }
}

struct XmlTagIter<'a> {
    src: &'a str,
    pos: usize,
}

impl<'a> XmlTagIter<'a> {
    fn new(src: &'a str) -> Self {
        Self { src, pos: 0 }
    }
}

impl Iterator for XmlTagIter<'_> {
    type Item = XmlTag;

    fn next(&mut self) -> Option<Self::Item> {
        let bytes = self.src.as_bytes();
        while self.pos < bytes.len() {
            let rel_lt = self.src[self.pos..].find('<')?;
            let start = self.pos + rel_lt;
            let rest = &self.src[start + 1..];

            // Skip comments `<!-- ... -->`
            if rest.starts_with("!--") {
                if let Some(end_c) = rest.find("-->") {
                    self.pos = start + 1 + end_c + 3;
                    continue;
                } else {
                    self.pos = bytes.len();
                    return None;
                }
            }
            // Skip processing instructions `<? ... ?>` and declarations `<! ... >`
            if rest.starts_with('?') || rest.starts_with('!') {
                if let Some(end_gt) = rest.find('>') {
                    self.pos = start + 1 + end_gt + 1;
                    continue;
                } else {
                    self.pos = bytes.len();
                    return None;
                }
            }

            // Locate closing `>` outside quoted attribute values
            let mut in_quote: Option<u8> = None;
            let mut end_rel: Option<usize> = None;
            for (i, &b) in rest.as_bytes().iter().enumerate() {
                if let Some(q) = in_quote {
                    if b == q {
                        in_quote = None;
                    }
                } else if b == b'"' || b == b'\'' {
                    in_quote = Some(b);
                } else if b == b'>' {
                    end_rel = Some(i);
                    break;
                }
            }
            let end_i = end_rel?;
            let inner = rest[..end_i].trim();
            self.pos = start + 1 + end_i + 1;
            if inner.is_empty() {
                continue;
            }

            let is_closing = inner.starts_with('/');
            let is_self_closing = !is_closing && inner.ends_with('/');
            let body = inner.trim_start_matches('/').trim_end_matches('/').trim();
            if body.is_empty() {
                continue;
            }

            let name_end = body
                .find(|c: char| c.is_ascii_whitespace())
                .unwrap_or(body.len());
            let full_name = &body[..name_end];
            let local_name = full_name
                .rsplit(':')
                .next()
                .unwrap_or(full_name)
                .to_string();

            let mut attrs = Vec::new();
            if !is_closing && name_end < body.len() {
                parse_xml_attrs(&body[name_end..], &mut attrs);
            }

            return Some(XmlTag {
                local_name,
                is_closing,
                is_self_closing,
                attrs,
            });
        }
        None
    }
}

fn parse_xml_attrs(mut s: &str, out: &mut Vec<(String, String)>) {
    loop {
        s = s.trim_start();
        if s.is_empty() {
            break;
        }
        let Some(eq_idx) = s.find('=') else {
            break;
        };
        let raw_key = s[..eq_idx].trim();
        let key = raw_key.rsplit(':').next().unwrap_or(raw_key);
        s = s[eq_idx + 1..].trim_start();
        let Some(quote) = s.chars().next().filter(|&c| c == '"' || c == '\'') else {
            break;
        };
        s = &s[1..];
        let Some(end_q) = s.find(quote) else {
            break;
        };
        let val = &s[..end_q];
        out.push((key.to_string(), val.to_string()));
        s = &s[end_q + 1..];
    }
}
