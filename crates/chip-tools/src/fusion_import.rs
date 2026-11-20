use std::{io::Read as _, path::Path};

use chip_math::{DisplayUnit, METERS_PER_FOOT, MM_PER_INCH};
use serde::Deserialize;
use thiserror::Error;

use crate::{
    feeds_speeds::{
        CuttingPreset, chipload_from_feed, feed_from_chipload, rpm_from_surface_speed,
        surface_speed_from_rpm,
    },
    model::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId},
    storage::ToolLibrary,
};

/// Maximum allowed Fusion 360 `.json` / `.tools` uncompressed payload (`32 MiB`).
pub const MAX_FUSION_JSON_BYTES: u64 = 32 * 1024 * 1024;
/// Maximum allowed ZIP entries inside a Fusion 360 `.tools` archive.
pub const MAX_FUSION_ARCHIVE_ENTRIES: usize = 256;

/// Errors reading or decoding a Fusion 360 `.json` or `.tools` (ZIP) tool library.
#[derive(Debug, Error)]
pub enum FusionImportError {
    #[error("I/O error reading Fusion 360 library '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to read `.tools` ZIP archive: {0}")]
    Zip(#[from] zip::result::ZipError),
    #[error("Fusion 360 library exceeds maximum allowed size ({limit_bytes} bytes)")]
    ArchiveTooLarge { limit_bytes: u64 },
    #[error("unsafe entry '{entry_path}' rejected in `.tools` archive")]
    UnsafeArchiveEntry { entry_path: String },
    #[error("`.tools` archive did not contain a JSON tool library entry")]
    MissingJsonInArchive,
    #[error("invalid Fusion 360 JSON syntax: {0}")]
    Json(#[from] serde_json::Error),
}

/// Summary and converted library returned by [`import_fusion_library_bytes`].
#[derive(Debug, Clone)]
pub struct FusionImportReport {
    pub library: ToolLibrary,
    pub imported_tool_count: usize,
    pub imported_preset_count: usize,
    /// Non-fatal warnings for unsupported exotic cutter types or malformed entries.
    pub warnings: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum FusionRoot {
    Wrapped {
        #[serde(default)]
        data: Vec<FusionToolRaw>,
    },
    BareArray(Vec<FusionToolRaw>),
}

#[derive(Debug, Deserialize)]
struct FusionToolRaw {
    #[serde(rename = "type", default)]
    tool_type: String,
    #[serde(default)]
    unit: String,
    #[serde(default)]
    description: String,
    #[serde(default)]
    vendor: String,
    #[serde(rename = "product-id", default)]
    product_id: String,
    #[serde(default)]
    guid: String,
    #[serde(rename = "geometry")]
    geom: Option<FusionGeomRaw>,
    #[serde(rename = "post-process")]
    post: Option<FusionPostRaw>,
    #[serde(rename = "start-values")]
    start_values: Option<FusionStartValuesRaw>,
}

#[derive(Debug, Default, Deserialize)]
struct FusionGeomRaw {
    /// Cutting diameter
    #[serde(rename = "DC", default)]
    dc: Option<f64>,
    /// Flute length
    #[serde(rename = "LCF", default)]
    lcf: Option<f64>,
    /// Overall length
    #[serde(rename = "OAL", default)]
    oal: Option<f64>,
    /// Body / stickout length below holder
    #[serde(rename = "LB", default)]
    lb: Option<f64>,
    /// Shaft / shank diameter
    #[serde(rename = "SFDM", default)]
    sfdm: Option<f64>,
    /// Number of flutes
    #[serde(rename = "NOF", default)]
    nof: Option<f64>,
    /// Corner radius
    #[serde(rename = "RE", default)]
    re: Option<f64>,
    /// Taper half-angle (degrees) in Fusion 360 chamfer/taper mills (`TA`)
    #[serde(rename = "TA", default)]
    ta: Option<f64>,
    /// Point / included angle (degrees) in Fusion 360 drills/spot drills (`SIG`)
    #[serde(rename = "SIG", default)]
    sig: Option<f64>,
    /// Tip diameter (`TPD`)
    #[serde(rename = "TPD", default)]
    tpd: Option<f64>,
    /// Outside diameter on face mills (`OD` / `DCON`)
    #[serde(rename = "OD", default)]
    od: Option<f64>,
}

#[derive(Debug, Default, Deserialize)]
struct FusionPostRaw {
    #[serde(default)]
    number: Option<u32>,
    #[serde(rename = "length-offset", default)]
    length_offset: Option<u32>,
    #[serde(rename = "diameter-offset", default)]
    diameter_offset: Option<u32>,
    #[serde(default)]
    clockwise: Option<bool>,
    #[serde(default)]
    coolant: Option<String>,
    #[serde(default)]
    comment: Option<String>,
}

#[derive(Debug, Default, Deserialize)]
struct FusionStartValuesRaw {
    #[serde(default)]
    presets: Vec<FusionPresetRaw>,
}

#[derive(Debug, Default, Deserialize)]
struct FusionPresetRaw {
    #[serde(default)]
    guid: String,
    #[serde(default)]
    name: String,
    #[serde(default)]
    description: String,
    /// Spindle RPM
    #[serde(default)]
    n: Option<f64>,
    /// Surface speed (`m/min` in metric tools, `ft/min` SFM in inch tools)
    #[serde(default)]
    v_c: Option<f64>,
    /// Cutting feedrate (`mm/min` or `in/min`)
    #[serde(default)]
    v_f: Option<f64>,
    /// Feed per tooth (`mm` or `in`)
    #[serde(default)]
    f_z: Option<f64>,
    #[serde(rename = "v_f_leadIn", default)]
    v_f_lead_in: Option<f64>,
    #[serde(rename = "v_f_ramp", default)]
    v_f_ramp: Option<f64>,
    #[serde(rename = "v_f_plunge", default)]
    v_f_plunge: Option<f64>,
    #[serde(default)]
    stepdown: Option<f64>,
    #[serde(default)]
    stepover: Option<f64>,
    #[serde(rename = "optimal-load", alias = "v_ae", default)]
    optimal_load: Option<f64>,
    #[serde(rename = "tool-coolant", default)]
    tool_coolant: Option<String>,
}

/// Imports a Fusion 360 `.json` or `.tools` (zipped JSON) file from disk.
pub fn import_fusion_library_path(
    path: &Path,
    library_name: Option<&str>,
) -> Result<FusionImportReport, FusionImportError> {
    let bytes = std::fs::read(path).map_err(|source| FusionImportError::Io {
        path: path.display().to_string(),
        source,
    })?;
    let fallback_name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Imported Fusion 360 Library");
    import_fusion_library_bytes(&bytes, library_name.unwrap_or(fallback_name))
}

/// Imports a Fusion 360 `.json` or `.tools` (ZIP) payload from memory.
pub fn import_fusion_library_bytes(
    bytes: &[u8],
    library_name: &str,
) -> Result<FusionImportReport, FusionImportError> {
    // Detect PK\x03\x04 ZIP header used by Fusion 360 `.tools` files
    let json_bytes: Vec<u8> = if bytes.starts_with(b"PK\x03\x04") {
        let cursor = std::io::Cursor::new(bytes);
        let mut archive = zip::ZipArchive::new(cursor)?;
        if archive.len() > MAX_FUSION_ARCHIVE_ENTRIES {
            return Err(FusionImportError::ArchiveTooLarge {
                limit_bytes: MAX_FUSION_JSON_BYTES,
            });
        }
        let mut extracted = None;
        for i in 0..archive.len() {
            let mut file = archive.by_index(i)?;
            let entry_name = file.name()?.into_owned();
            if file.is_symlink() {
                return Err(FusionImportError::UnsafeArchiveEntry {
                    entry_path: entry_name,
                });
            }
            if entry_name.ends_with(".json") {
                if file.size() > MAX_FUSION_JSON_BYTES {
                    return Err(FusionImportError::ArchiveTooLarge {
                        limit_bytes: MAX_FUSION_JSON_BYTES,
                    });
                }
                let mut buf = Vec::with_capacity((file.size() as usize).min(4 * 1024 * 1024));
                file.by_ref()
                    .take(MAX_FUSION_JSON_BYTES + 1)
                    .read_to_end(&mut buf)
                    .map_err(|source| FusionImportError::Io {
                        path: entry_name,
                        source,
                    })?;
                if buf.len() as u64 > MAX_FUSION_JSON_BYTES {
                    return Err(FusionImportError::ArchiveTooLarge {
                        limit_bytes: MAX_FUSION_JSON_BYTES,
                    });
                }
                extracted = Some(buf);
                break;
            }
        }
        extracted.ok_or(FusionImportError::MissingJsonInArchive)?
    } else {
        if bytes.len() as u64 > MAX_FUSION_JSON_BYTES {
            return Err(FusionImportError::ArchiveTooLarge {
                limit_bytes: MAX_FUSION_JSON_BYTES,
            });
        }
        bytes.to_vec()
    };

    let root: FusionRoot = serde_json::from_slice(&json_bytes)?;
    let raw_tools = match root {
        FusionRoot::Wrapped { data } => data,
        FusionRoot::BareArray(items) => items,
    };

    let mut tools = Vec::new();
    let mut warnings = Vec::new();
    let mut imported_preset_count = 0usize;

    for (idx, raw) in raw_tools.into_iter().enumerate() {
        let norm_type = raw.tool_type.trim().to_ascii_lowercase();
        let is_inches = raw.unit.trim().eq_ignore_ascii_case("inches")
            || raw.unit.trim().eq_ignore_ascii_case("inch")
            || raw.unit.trim().eq_ignore_ascii_case("in");
        let native_unit = if is_inches {
            DisplayUnit::Inch
        } else {
            DisplayUnit::Millimeter
        };
        let len_scale = if is_inches { MM_PER_INCH } else { 1.0 };
        let vc_scale = if is_inches { METERS_PER_FOOT } else { 1.0 };

        let g = raw.geom.unwrap_or_default();
        let post = raw.post.unwrap_or_default();

        let dc_mm = g.dc.unwrap_or(0.0) * len_scale;
        if !dc_mm.is_finite() || dc_mm <= 0.0 {
            warnings.push(format!(
                "Skipped Fusion tool #{idx} ('{}'): missing or non-positive cutting diameter DC.",
                raw.description
            ));
            continue;
        }

        let flutes = (g.nof.unwrap_or(2.0).round() as i64).max(1) as u32;
        let lcf_mm = (g.lcf.map(|v| v * len_scale).unwrap_or(dc_mm * 3.0)).max(0.5);
        let oal_mm = (g.oal.map(|v| v * len_scale).unwrap_or(dc_mm * 8.0)).max(lcf_mm);
        let sfdm_mm = (g.sfdm.map(|v| v * len_scale).unwrap_or(dc_mm)).max(0.5);
        let stickout_mm =
            g.lb.map(|v| v * len_scale)
                .unwrap_or(lcf_mm * 1.25)
                .max(lcf_mm)
                .min(oal_mm);

        let geometry = match norm_type.as_str() {
            "flat end mill" => CutterGeometry::FlatEndmill {
                diameter_mm: dc_mm,
                flute_length_mm: lcf_mm,
                overall_length_mm: oal_mm,
                shank_diameter_mm: sfdm_mm,
                flutes,
            },
            "bull nose end mill" => {
                let re_mm = (g.re.unwrap_or(0.0) * len_scale).clamp(0.01, 0.49 * dc_mm);
                CutterGeometry::BullNoseEndmill {
                    diameter_mm: dc_mm,
                    corner_radius_mm: re_mm,
                    flute_length_mm: lcf_mm,
                    overall_length_mm: oal_mm,
                    shank_diameter_mm: sfdm_mm,
                    flutes,
                }
            }
            "ball end mill" => CutterGeometry::BallEndmill {
                diameter_mm: dc_mm,
                flute_length_mm: lcf_mm,
                overall_length_mm: oal_mm,
                shank_diameter_mm: sfdm_mm,
                flutes,
            },
            "chamfer mill" | "spot drill" | "engraving" => {
                // Fusion `SIG` is full included angle; `TA` is taper half-angle
                let included_deg = g
                    .sig
                    .or_else(|| g.ta.map(|half| 2.0 * half))
                    .unwrap_or(90.0)
                    .clamp(5.0, 175.0);
                let tip_mm = (g.tpd.unwrap_or(0.0) * len_scale).clamp(0.0, 0.95 * dc_mm);
                CutterGeometry::ChamferMill {
                    diameter_mm: dc_mm,
                    tip_diameter_mm: tip_mm,
                    included_angle_deg: included_deg,
                    flute_length_mm: lcf_mm,
                    overall_length_mm: oal_mm,
                    shank_diameter_mm: sfdm_mm,
                    flutes,
                }
            }
            "drill" => {
                let point_deg = g.sig.unwrap_or(118.0).clamp(30.0, 175.0);
                CutterGeometry::TwistDrill {
                    diameter_mm: dc_mm,
                    point_angle_deg: point_deg,
                    flute_length_mm: lcf_mm,
                    overall_length_mm: oal_mm,
                    shank_diameter_mm: sfdm_mm,
                    flutes,
                }
            }
            "face mill" => {
                let outside_mm = g.od.map(|v| v * len_scale).unwrap_or(dc_mm).max(dc_mm);
                let re_mm = (g.re.unwrap_or(0.0) * len_scale).max(0.0);
                CutterGeometry::FaceMill {
                    cutting_diameter_mm: dc_mm,
                    outside_diameter_mm: outside_mm,
                    corner_radius_mm: re_mm,
                    flute_length_mm: lcf_mm,
                    overall_length_mm: oal_mm,
                    shank_diameter_mm: sfdm_mm,
                    flutes,
                }
            }
            unsupported => {
                warnings.push(format!(
                    "Skipped unsupported Fusion 360 tool type '{unsupported}' ('{}').",
                    raw.description
                ));
                continue;
            }
        };

        let tool_num = post.number.unwrap_or((idx as u32) + 1).max(1);
        let len_off = post.length_offset.unwrap_or(tool_num);
        let dia_off = post.diameter_offset.unwrap_or(tool_num);
        let spindle_dir = if post.clockwise.unwrap_or(true) {
            SpindleDirection::Cw
        } else {
            SpindleDirection::Ccw
        };
        let default_coolant = parse_fusion_coolant(post.coolant.as_deref()).unwrap_or_default();

        let name = if !raw.description.trim().is_empty() {
            raw.description.trim().to_string()
        } else if let Some(c) = post.comment.as_deref().filter(|s| !s.trim().is_empty()) {
            c.trim().to_string()
        } else {
            format!(
                "{} {}",
                native_unit.format_length(dc_mm),
                geometry.kind_label()
            )
        };

        let raw_id = if !raw.guid.trim().is_empty() {
            sanitize_slug(&raw.guid)
        } else {
            format!("fusion_t{tool_num}_{idx}")
        };

        let mut presets = Vec::new();
        if let Some(sv) = raw.start_values {
            for (p_idx, p) in sv.presets.into_iter().enumerate() {
                let rpm =
                    p.n.filter(|&v| v > 0.0)
                        .or_else(|| {
                            p.v_c
                                .and_then(|vc| rpm_from_surface_speed(vc * vc_scale, dc_mm))
                        })
                        .unwrap_or(8_000.0);

                let vc_m_min = p
                    .v_c
                    .filter(|&v| v > 0.0)
                    .map(|v| v * vc_scale)
                    .or_else(|| surface_speed_from_rpm(rpm, dc_mm))
                    .unwrap_or(150.0);

                let vf_mm_min = p
                    .v_f
                    .filter(|&v| v > 0.0)
                    .map(|v| v * len_scale)
                    .or_else(|| {
                        p.f_z
                            .filter(|&v| v > 0.0)
                            .map(|fz| feed_from_chipload(rpm, flutes, fz * len_scale))
                    })
                    .unwrap_or(600.0);

                let fz_mm = p
                    .f_z
                    .filter(|&v| v > 0.0)
                    .map(|v| v * len_scale)
                    .or_else(|| chipload_from_feed(rpm, flutes, vf_mm_min))
                    .unwrap_or(0.03);

                let preset_name = if !p.name.trim().is_empty() {
                    p.name.trim().to_string()
                } else {
                    format!("Preset {}", p_idx + 1)
                };
                let preset_id = if !p.guid.trim().is_empty() {
                    sanitize_slug(&p.guid)
                } else {
                    format!("preset_{}", p_idx + 1)
                };

                presets.push(CuttingPreset {
                    id: preset_id,
                    name: preset_name.clone(),
                    material: if p.description.trim().is_empty() {
                        preset_name
                    } else {
                        p.description.trim().to_string()
                    },
                    spindle_rpm: rpm,
                    surface_speed_m_min: vc_m_min,
                    cutting_feed_mm_min: vf_mm_min,
                    chipload_mm_tooth: fz_mm,
                    lead_feed_mm_min: p
                        .v_f_lead_in
                        .filter(|&v| v > 0.0)
                        .map(|v| v * len_scale)
                        .unwrap_or(0.8 * vf_mm_min),
                    ramp_feed_mm_min: p
                        .v_f_ramp
                        .filter(|&v| v > 0.0)
                        .map(|v| v * len_scale)
                        .unwrap_or(0.5 * vf_mm_min),
                    plunge_feed_mm_min: p
                        .v_f_plunge
                        .filter(|&v| v > 0.0)
                        .map(|v| v * len_scale)
                        .unwrap_or((vf_mm_min / 3.0).max(10.0)),
                    default_stepdown_mm: p
                        .stepdown
                        .filter(|&v| v > 0.0)
                        .map(|v| v * len_scale)
                        .unwrap_or(0.5 * dc_mm),
                    default_stepover_mm: p
                        .stepover
                        .filter(|&v| v > 0.0)
                        .map(|v| v * len_scale)
                        .unwrap_or(0.4 * dc_mm),
                    optimal_load_mm: p
                        .optimal_load
                        .filter(|&v| v.is_finite() && v > 0.0)
                        .map(|v| v * len_scale),
                    coolant_override: parse_fusion_coolant(p.tool_coolant.as_deref()),
                });
            }
        }

        imported_preset_count += presets.len();

        let candidate = Tool {
            id: ToolId(raw_id),
            tool_number: tool_num,
            length_offset_number: len_off,
            diameter_offset_number: dia_off,
            name,
            vendor: non_empty_opt(&raw.vendor),
            product_id: non_empty_opt(&raw.product_id),
            native_unit,
            spindle_direction: spindle_dir,
            default_coolant,
            stickout_length_mm: stickout_mm,
            geometry,
            presets,
        };

        match candidate.validate() {
            Ok(()) => tools.push(candidate),
            Err(err) => warnings.push(format!("Skipped invalid imported tool: {err}")),
        }
    }

    let imported_tool_count = tools.len();
    let mut library = ToolLibrary {
        schema_version: ToolLibrary::CURRENT_SCHEMA_VERSION,
        name: library_name.to_string(),
        tools,
    };
    library.sort_deterministically();

    Ok(FusionImportReport {
        library,
        imported_tool_count,
        imported_preset_count,
        warnings,
    })
}

fn non_empty_opt(s: &str) -> Option<String> {
    let trimmed = s.trim();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed.to_string())
    }
}

fn sanitize_slug(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' || ch == '-' {
            out.push(ch.to_ascii_lowercase());
        } else {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches('_');
    if trimmed.is_empty() {
        "tool".to_string()
    } else {
        trimmed.to_string()
    }
}

fn parse_fusion_coolant(s: Option<&str>) -> Option<CoolantMode> {
    let lower = s?.trim().to_ascii_lowercase();
    match lower.as_str() {
        "disabled" | "off" => Some(CoolantMode::Disabled),
        "flood" | "flood and mist" | "through tool" => Some(CoolantMode::Flood),
        "mist" => Some(CoolantMode::Mist),
        "air" | "air through tool" => Some(CoolantMode::AirBlast),
        _ => None,
    }
}
