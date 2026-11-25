//! Curated project-level workpiece material physics catalog, custom material overrides,
//! and deterministic cutter-preset matching (`chip-tools`).

use serde::{Deserialize, Serialize};

use crate::{
    feeds_speeds::CuttingPreset,
    model::{CoolantMode, Tool},
};

/// Canonical built-in workpiece material IDs.
pub const ALUMINUM_6061_T6_ID: &str = "aluminum-6061-t6";
pub const ALUMINUM_6063_T5_ID: &str = "aluminum-6063-t5";
pub const ALUMINUM_7075_T6_ID: &str = "aluminum-7075-t6";
pub const BRASS_C360_ID: &str = "brass-c360";
pub const ACETAL_DELRIN_ID: &str = "acetal-delrin";
pub const ACRYLIC_PMMA_ID: &str = "acrylic-pmma";
pub const HDPE_PLASTIC_ID: &str = "hdpe-plastic";
pub const HARDWOOD_PLYWOOD_ID: &str = "hardwood-plywood";
pub const MDF_ID: &str = "mdf";
pub const G10_CARBON_FIBER_ID: &str = "g10-carbon-fiber";
pub const MILD_STEEL_1018_ID: &str = "mild-steel-1018";

/// Default fallback workpiece material ID when a project does not specify one.
pub const DEFAULT_WORKPIECE_MATERIAL_ID: &str = ALUMINUM_6061_T6_ID;

/// Sentinel ID for user-defined custom workpiece materials.
pub const CUSTOM_WORKPIECE_MATERIAL_ID: &str = "custom";

/// High-level material family used to group the curated catalog and provide fallback
/// preset-matching semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialCategory {
    Aluminum,
    NonFerrousMetal,
    Steel,
    Plastic,
    Wood,
    Composite,
    Custom,
}

impl MaterialCategory {
    /// Display order of built-in material categories in UI selectors.
    pub const BUILTIN_ORDER: [Self; 6] = [
        Self::Aluminum,
        Self::NonFerrousMetal,
        Self::Plastic,
        Self::Wood,
        Self::Composite,
        Self::Steel,
    ];

    /// Human-readable category heading.
    pub const fn label(self) -> &'static str {
        match self {
            Self::Aluminum => "Aluminum",
            Self::NonFerrousMetal => "Non-Ferrous Metal",
            Self::Steel => "Steel",
            Self::Plastic => "Engineering Plastic",
            Self::Wood => "Wood & Sheet Goods",
            Self::Composite => "Composite / PCB",
            Self::Custom => "Custom",
        }
    }

    const fn fallback_tokens(self) -> &'static [&'static str] {
        match self {
            Self::Aluminum => &["aluminum", "aluminium", "alu"],
            Self::NonFerrousMetal => &["brass", "bronze", "copper", "non-ferrous"],
            Self::Steel => &["steel", "stainless", "ferrous", "iron"],
            Self::Plastic => &[
                "plastic", "acetal", "delrin", "acrylic", "pmma", "hdpe", "polymer", "nylon",
            ],
            Self::Wood => &[
                "wood",
                "plywood",
                "birch",
                "hardwood",
                "mdf",
                "tooling board",
            ],
            Self::Composite => &["composite", "fr4", "pcb", "g10", "carbon", "cfrp", "epoxy"],
            Self::Custom => &[],
        }
    }
}

/// Curated physical cutting-mechanics profile for a workpiece stock material.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkpieceMaterialProfile {
    /// Canonical stable slug (e.g., `"aluminum-6063-t5"` or `"custom"`).
    pub id: String,
    /// Human-readable material label.
    pub name: String,
    /// Material category for grouping and fallback preset matching.
    pub category: MaterialCategory,
    /// Concise machinist guidance notes (chip evacuation, welding risk, cutter choice).
    pub notes: String,
    /// Specific cutting energy `u_s` in `W*s/mm^3` (`= J/mm^3`).
    pub specific_cutting_energy_w_s_mm3: f64,
    /// Specific cutting force `K_c` in `N/mm^2` (`= MPa`) for radial/tangential force & bending stress.
    pub specific_cutting_force_n_mm2: f64,
    /// Radial-to-tangential cutting force ratio `k_r = F_r / F_t`.
    pub radial_force_ratio: f64,
    /// Minimum uncut chip thickness `h_min` (`mm`) below which rubbing/chip-welding occurs.
    pub min_chip_thickness_mm: f64,
    /// Recommended upper chipload sanity multiplier relative to tool diameter (`fz_max ~ factor * D`).
    pub max_recommended_chipload_factor: f64,
    /// Recommended carbide peripheral surface speed envelope `(min, max)` in `m/min`.
    pub recommended_sfm_range_m_min: (f64, f64),
    /// `true` for ductile/gummy alloys and thermoplastics prone to flute chip-welding on dry or thin cuts.
    pub gummy_chip_weld_risk: bool,
    /// Recommended coolant / chip-evacuation mode.
    pub preferred_coolant: CoolantMode,
    /// Deterministic fuzzy-matching tokens ordered from most specific to most general.
    #[serde(skip)]
    pub preset_match_keywords: &'static [&'static str],
}

/// Project-level workpiece material reference stored in `.chip.toml`.
///
/// References either a read-only built-in profile ID (e.g., `"aluminum-6063-t5"`) or `"custom"`
/// with optional physics overrides omitted from TOML when `None`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectMaterialSelection {
    pub material_id: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_specific_cutting_energy: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_specific_cutting_force: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_min_chip_thickness_mm: Option<f64>,
}

impl ProjectMaterialSelection {
    /// Selects a built-in material profile by its canonical ID with no custom overrides.
    pub fn builtin(material_id: impl Into<String>) -> Self {
        Self {
            material_id: material_id.into(),
            custom_name: None,
            custom_specific_cutting_energy: None,
            custom_specific_cutting_force: None,
            custom_min_chip_thickness_mm: None,
        }
    }

    /// Creates a custom workpiece material selection with explicit physics parameters.
    pub fn custom(
        name: impl Into<String>,
        specific_cutting_energy_w_s_mm3: f64,
        specific_cutting_force_n_mm2: f64,
        min_chip_thickness_mm: f64,
    ) -> Self {
        Self {
            material_id: CUSTOM_WORKPIECE_MATERIAL_ID.to_string(),
            custom_name: Some(name.into()),
            custom_specific_cutting_energy: Some(specific_cutting_energy_w_s_mm3),
            custom_specific_cutting_force: Some(specific_cutting_force_n_mm2),
            custom_min_chip_thickness_mm: Some(min_chip_thickness_mm),
        }
    }

    /// Returns `true` when this selection uses the `"custom"` profile ID.
    pub fn is_custom(&self) -> bool {
        self.material_id
            .trim()
            .eq_ignore_ascii_case(CUSTOM_WORKPIECE_MATERIAL_ID)
    }
}

/// Returns all 11 read-only built-in workpiece material profiles in deterministic canonical order.
#[allow(clippy::too_many_lines)]
pub fn builtin_workpiece_materials() -> Vec<WorkpieceMaterialProfile> {
    vec![
        WorkpieceMaterialProfile {
            id: ALUMINUM_6061_T6_ID.to_string(),
            name: "Aluminum 6061-T6".to_string(),
            category: MaterialCategory::Aluminum,
            notes: "General-purpose machinable aluminum alloy; clean chip formation with air blast or mist.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.75,
            specific_cutting_force_n_mm2: 750.0,
            radial_force_ratio: 0.40,
            min_chip_thickness_mm: 0.008,
            max_recommended_chipload_factor: 0.012,
            recommended_sfm_range_m_min: (120.0, 350.0),
            gummy_chip_weld_risk: true,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["6061", "aluminum", "aluminium", "alu"],
        },
        WorkpieceMaterialProfile {
            id: ALUMINUM_6063_T5_ID.to_string(),
            name: "Aluminum 6063-T5 Extrusion".to_string(),
            category: MaterialCategory::Aluminum,
            notes: "Architectural extrusion alloy; softer and gummier than 6061-T6 with high dry chip-welding risk. Prefer polished 1FL O-flute cutters, air blast/mist, and light interrupted-cut engagement.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.85,
            specific_cutting_force_n_mm2: 850.0,
            radial_force_ratio: 0.50,
            min_chip_thickness_mm: 0.010,
            max_recommended_chipload_factor: 0.010,
            recommended_sfm_range_m_min: (90.0, 260.0),
            gummy_chip_weld_risk: true,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["6063", "extrusion", "aluminum", "aluminium", "alu"],
        },
        WorkpieceMaterialProfile {
            id: ALUMINUM_7075_T6_ID.to_string(),
            name: "Aluminum 7075-T6".to_string(),
            category: MaterialCategory::Aluminum,
            notes: "High-strength aerospace aluminum; crisper chip breaking than 6061 with higher cutting forces.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.90,
            specific_cutting_force_n_mm2: 900.0,
            radial_force_ratio: 0.38,
            min_chip_thickness_mm: 0.007,
            max_recommended_chipload_factor: 0.012,
            recommended_sfm_range_m_min: (110.0, 320.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::Mist,
            preset_match_keywords: &["7075", "6061", "aluminum", "aluminium", "alu"],
        },
        WorkpieceMaterialProfile {
            id: BRASS_C360_ID.to_string(),
            name: "Brass C360 (Free-Machining)".to_string(),
            category: MaterialCategory::NonFerrousMetal,
            notes: "Free-machining leaded brass; short crisp chips and low welding risk, watch for grab on high-rake cutters.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.95,
            specific_cutting_force_n_mm2: 950.0,
            radial_force_ratio: 0.35,
            min_chip_thickness_mm: 0.006,
            max_recommended_chipload_factor: 0.012,
            recommended_sfm_range_m_min: (80.0, 220.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["brass", "c360", "bronze", "copper"],
        },
        WorkpieceMaterialProfile {
            id: ACETAL_DELRIN_ID.to_string(),
            name: "Acetal / Delrin (POM)".to_string(),
            category: MaterialCategory::Plastic,
            notes: "Rigid low-friction engineering thermoplastic; machines cleanly with sharp O-flute cutters and air blast.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.25,
            specific_cutting_force_n_mm2: 250.0,
            radial_force_ratio: 0.35,
            min_chip_thickness_mm: 0.012,
            max_recommended_chipload_factor: 0.020,
            recommended_sfm_range_m_min: (120.0, 350.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["delrin", "acetal", "pom", "plastic"],
        },
        WorkpieceMaterialProfile {
            id: ACRYLIC_PMMA_ID.to_string(),
            name: "Acrylic (PMMA)".to_string(),
            category: MaterialCategory::Plastic,
            notes: "Thermoplastic prone to melting and recutting chips when chipload is too thin; keep chip thickness above h_min with air blast.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.30,
            specific_cutting_force_n_mm2: 300.0,
            radial_force_ratio: 0.42,
            min_chip_thickness_mm: 0.015,
            max_recommended_chipload_factor: 0.020,
            recommended_sfm_range_m_min: (100.0, 300.0),
            gummy_chip_weld_risk: true,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["acrylic", "pmma", "plexiglass", "plastic"],
        },
        WorkpieceMaterialProfile {
            id: HDPE_PLASTIC_ID.to_string(),
            name: "HDPE / Engineering Plastic".to_string(),
            category: MaterialCategory::Plastic,
            notes: "Ductile thermoplastic prone to stringy chips and flute wrapping at thin chiploads; use 1FL O-flute with air blast.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.22,
            specific_cutting_force_n_mm2: 220.0,
            radial_force_ratio: 0.45,
            min_chip_thickness_mm: 0.015,
            max_recommended_chipload_factor: 0.025,
            recommended_sfm_range_m_min: (120.0, 360.0),
            gummy_chip_weld_risk: true,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["hdpe", "uhmw", "polyethylene", "plastic", "acetal"],
        },
        WorkpieceMaterialProfile {
            id: HARDWOOD_PLYWOOD_ID.to_string(),
            name: "Hardwood / Baltic Birch Plywood".to_string(),
            category: MaterialCategory::Wood,
            notes: "Hardwood and Baltic birch plywood; cut dry with dust extraction and keep chipload brisk to prevent scorching.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.15,
            specific_cutting_force_n_mm2: 150.0,
            radial_force_ratio: 0.35,
            min_chip_thickness_mm: 0.015,
            max_recommended_chipload_factor: 0.030,
            recommended_sfm_range_m_min: (150.0, 450.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::Disabled,
            preset_match_keywords: &["plywood", "birch", "hardwood", "wood"],
        },
        WorkpieceMaterialProfile {
            id: MDF_ID.to_string(),
            name: "MDF / Softwood".to_string(),
            category: MaterialCategory::Wood,
            notes: "Medium-density fiberboard and softwoods; low cutting force, cut dry with dust extraction.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.12,
            specific_cutting_force_n_mm2: 120.0,
            radial_force_ratio: 0.35,
            min_chip_thickness_mm: 0.018,
            max_recommended_chipload_factor: 0.035,
            recommended_sfm_range_m_min: (180.0, 500.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::Disabled,
            preset_match_keywords: &["mdf", "softwood", "wood", "plywood", "hardwood"],
        },
        WorkpieceMaterialProfile {
            id: G10_CARBON_FIBER_ID.to_string(),
            name: "G10 / FR4 / Carbon Fiber".to_string(),
            category: MaterialCategory::Composite,
            notes: "Abrasive thermoset composites and FR4 PCB; prefer diamond-cut/corn bits with vacuum dust extraction.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.55,
            specific_cutting_force_n_mm2: 550.0,
            radial_force_ratio: 0.45,
            min_chip_thickness_mm: 0.008,
            max_recommended_chipload_factor: 0.015,
            recommended_sfm_range_m_min: (60.0, 180.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::Disabled,
            preset_match_keywords: &["carbon", "cfrp", "fr4", "g10", "pcb", "composite"],
        },
        WorkpieceMaterialProfile {
            id: MILD_STEEL_1018_ID.to_string(),
            name: "Mild Steel 1018".to_string(),
            category: MaterialCategory::Steel,
            notes: "Low-carbon mild steel; high cutting forces requiring rigid machines, coated carbide, and flood/mist cooling.".to_string(),
            specific_cutting_energy_w_s_mm3: 2.60,
            specific_cutting_force_n_mm2: 2600.0,
            radial_force_ratio: 0.50,
            min_chip_thickness_mm: 0.008,
            max_recommended_chipload_factor: 0.008,
            recommended_sfm_range_m_min: (50.0, 140.0),
            gummy_chip_weld_risk: false,
            preferred_coolant: CoolantMode::Flood,
            preset_match_keywords: &["1018", "mild steel", "steel"],
        },
    ]
}

fn default_material_profile() -> WorkpieceMaterialProfile {
    let mut builtins = builtin_workpiece_materials();
    if let Some(pos) = builtins
        .iter()
        .position(|m| m.id == DEFAULT_WORKPIECE_MATERIAL_ID)
    {
        builtins.swap_remove(pos)
    } else {
        WorkpieceMaterialProfile {
            id: ALUMINUM_6061_T6_ID.to_string(),
            name: "Aluminum 6061-T6".to_string(),
            category: MaterialCategory::Aluminum,
            notes: "General-purpose machinable aluminum alloy.".to_string(),
            specific_cutting_energy_w_s_mm3: 0.75,
            specific_cutting_force_n_mm2: 750.0,
            radial_force_ratio: 0.40,
            min_chip_thickness_mm: 0.008,
            max_recommended_chipload_factor: 0.012,
            recommended_sfm_range_m_min: (120.0, 350.0),
            gummy_chip_weld_risk: true,
            preferred_coolant: CoolantMode::AirBlast,
            preset_match_keywords: &["6061", "aluminum", "aluminium", "alu"],
        }
    }
}

/// Resolves an optional [`ProjectMaterialSelection`] into an effective [`WorkpieceMaterialProfile`].
///
/// - `None` resolves to `aluminum-6061-t6` (backward-compatible default).
/// - `"custom"` resolves to a [`MaterialCategory::Custom`] profile seeded from default aluminum
///   physics and overridden by any finite positive custom fields.
/// - Unknown IDs fall back gracefully to `aluminum-6061-t6` (never panicking).
pub fn resolve_workpiece_material(
    sel: Option<&ProjectMaterialSelection>,
) -> WorkpieceMaterialProfile {
    let Some(sel) = sel else {
        return default_material_profile();
    };

    let trimmed_id = sel.material_id.trim();
    let mut profile = if trimmed_id.eq_ignore_ascii_case(CUSTOM_WORKPIECE_MATERIAL_ID) {
        let base = default_material_profile();
        WorkpieceMaterialProfile {
            id: CUSTOM_WORKPIECE_MATERIAL_ID.to_string(),
            name: "Custom Material".to_string(),
            category: MaterialCategory::Custom,
            notes: "User-defined custom workpiece material physics overrides.".to_string(),
            preset_match_keywords: &[],
            ..base
        }
    } else {
        builtin_workpiece_materials()
            .into_iter()
            .find(|m| m.id.eq_ignore_ascii_case(trimmed_id))
            .unwrap_or_else(default_material_profile)
    };

    if let Some(custom_name) = sel.custom_name.as_deref() {
        let trimmed = custom_name.trim();
        if !trimmed.is_empty() {
            profile.name = trimmed.to_string();
        }
    }
    if let Some(us) = sel.custom_specific_cutting_energy
        && us.is_finite()
        && us > 0.0
    {
        profile.specific_cutting_energy_w_s_mm3 = us;
    }
    if let Some(kc) = sel.custom_specific_cutting_force
        && kc.is_finite()
        && kc > 0.0
    {
        profile.specific_cutting_force_n_mm2 = kc;
    }
    if let Some(hmin) = sel.custom_min_chip_thickness_mm
        && hmin.is_finite()
        && hmin > 0.0
    {
        profile.min_chip_thickness_mm = hmin;
    }

    profile
}

fn score_preset_against_material(
    preset: &CuttingPreset,
    material: &WorkpieceMaterialProfile,
) -> u32 {
    let p_mat = preset.material.trim().to_ascii_lowercase();
    let p_name = preset.name.trim().to_ascii_lowercase();
    let p_id = preset.id.trim().to_ascii_lowercase();
    let m_id = material.id.trim().to_ascii_lowercase();
    let m_name = material.name.trim().to_ascii_lowercase();

    let mut score = 0u32;

    // Tier 1: Exact ID / exact material name / strong name containment
    if (!m_id.is_empty() && p_id == m_id)
        || (!m_name.is_empty() && (p_mat == m_name || p_name == m_name))
    {
        score = score.max(1200);
    } else if !p_mat.is_empty()
        && p_mat.len() >= 4
        && (m_name.starts_with(&p_mat) || p_mat.contains(&m_name))
    {
        score = score.max(1000);
    } else if !m_name.is_empty() && p_name.contains(&m_name) {
        score = score.max(950);
    }

    // Tier 2: Ordered material keyword specificity match
    let mut best_kw = 0u32;
    for (kw_idx, &kw) in material.preset_match_keywords.iter().enumerate() {
        let specificity = 600u32.saturating_sub((kw_idx as u32) * 90).max(120);
        let kw_score = if p_mat.contains(kw) {
            specificity + 60
        } else if p_name.contains(kw) {
            specificity + 30
        } else if p_id.contains(kw) {
            specificity
        } else {
            0
        };
        best_kw = best_kw.max(kw_score);
    }
    if material.category == MaterialCategory::Custom {
        for tok in m_name
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|t| t.len() >= 3 && *t != "custom" && *t != "material")
        {
            let kw_score = if p_mat.contains(tok) {
                560
            } else if p_name.contains(tok) {
                530
            } else if p_id.contains(tok) {
                500
            } else {
                0
            };
            best_kw = best_kw.max(kw_score);
        }
    }
    score = score.max(best_kw);

    // Tier 3: Category fallback tokens
    if score == 0 {
        for &tok in material.category.fallback_tokens() {
            if p_mat.contains(tok) {
                score = score.max(80);
            } else if p_name.contains(tok) {
                score = score.max(60);
            } else if p_id.contains(tok) {
                score = score.max(40);
            }
        }
    }

    score
}

/// Deterministically selects the best matching [`CuttingPreset`] on `tool` for `material`.
///
/// Precedence:
/// 1. Exact ID / material-name match
/// 2. Specific material keyword match (e.g., `"6063"` beats generic `"aluminum"`)
/// 3. Category-level fallback match
/// 4. First preset (`tool.presets.first()`) when no preset matches
pub fn match_best_preset_for_material<'a>(
    tool: &'a Tool,
    material: &WorkpieceMaterialProfile,
) -> Option<&'a CuttingPreset> {
    let mut best: Option<(u32, usize, &'a CuttingPreset)> = None;
    for (idx, preset) in tool.presets.iter().enumerate() {
        let s = score_preset_against_material(preset, material);
        let replace = match best {
            None => true,
            Some((best_s, best_idx, _)) => s > best_s || (s == best_s && idx < best_idx),
        };
        if replace {
            best = Some((s, idx, preset));
        }
    }
    best.map(|(_, _, p)| p)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::collections::BTreeSet;

    use crate::{
        builtin_carvera_air_machine, builtin_carvera_machine, builtin_carvera_z1_machine,
        default_starter_tool_library,
    };

    use super::*;

    #[test]
    fn builtin_materials_have_valid_physics_deterministic_order_and_6063_profile() {
        let mats = builtin_workpiece_materials();
        assert_eq!(mats.len(), 11);

        let expected_ids = vec![
            ALUMINUM_6061_T6_ID,
            ALUMINUM_6063_T5_ID,
            ALUMINUM_7075_T6_ID,
            BRASS_C360_ID,
            ACETAL_DELRIN_ID,
            ACRYLIC_PMMA_ID,
            HDPE_PLASTIC_ID,
            HARDWOOD_PLYWOOD_ID,
            MDF_ID,
            G10_CARBON_FIBER_ID,
            MILD_STEEL_1018_ID,
        ];
        let actual_ids: Vec<&str> = mats.iter().map(|m| m.id.as_str()).collect();
        assert_eq!(actual_ids, expected_ids);

        let mut seen = BTreeSet::new();
        for m in &mats {
            assert!(seen.insert(m.id.as_str()), "duplicate material id {}", m.id);
            assert!(!m.name.trim().is_empty());
            assert!(!m.notes.trim().is_empty());
            assert!(
                m.specific_cutting_energy_w_s_mm3.is_finite()
                    && m.specific_cutting_energy_w_s_mm3 > 0.0,
                "invalid u_s on {}",
                m.id
            );
            assert!(
                m.specific_cutting_force_n_mm2.is_finite() && m.specific_cutting_force_n_mm2 > 0.0,
                "invalid K_c on {}",
                m.id
            );
            assert!(
                m.radial_force_ratio.is_finite() && (0.20..=0.80).contains(&m.radial_force_ratio),
                "invalid k_r on {}",
                m.id
            );
            assert!(
                m.min_chip_thickness_mm.is_finite() && m.min_chip_thickness_mm > 0.0,
                "invalid h_min on {}",
                m.id
            );
            assert!(
                m.max_recommended_chipload_factor.is_finite()
                    && m.max_recommended_chipload_factor > 0.0,
                "invalid chipload factor on {}",
                m.id
            );
            let (vc_lo, vc_hi) = m.recommended_sfm_range_m_min;
            assert!(
                vc_lo.is_finite() && vc_hi.is_finite() && vc_lo > 0.0 && vc_lo < vc_hi,
                "invalid SFM range on {}",
                m.id
            );
            assert!(
                !m.preset_match_keywords.is_empty(),
                "keywords must be non-empty on {}",
                m.id
            );
        }

        // Verify Aluminum 6063-T5 Extrusion exact requirements
        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));
        assert_eq!(al6063.id, ALUMINUM_6063_T5_ID);
        assert_eq!(al6063.name, "Aluminum 6063-T5 Extrusion");
        assert_eq!(al6063.category, MaterialCategory::Aluminum);
        assert!(al6063.gummy_chip_weld_risk);
        assert_eq!(al6063.preferred_coolant, CoolantMode::AirBlast);
        assert!((al6063.specific_cutting_energy_w_s_mm3 - 0.85).abs() < 1e-9);
        assert!((al6063.specific_cutting_force_n_mm2 - 850.0).abs() < 1e-9);
        assert!((al6063.min_chip_thickness_mm - 0.010).abs() < 1e-9);

        // Unspecified & unknown selections fall back safely to Aluminum 6061-T6
        assert_eq!(resolve_workpiece_material(None).id, ALUMINUM_6061_T6_ID);
        assert_eq!(
            resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin("nonexistent-xyz")))
                .id,
            ALUMINUM_6061_T6_ID
        );

        // Custom material applies overrides and rejects NaN/non-positive overrides safely
        let custom = resolve_workpiece_material(Some(&ProjectMaterialSelection::custom(
            "Shop 6063 Billet",
            0.88,
            880.0,
            0.011,
        )));
        assert_eq!(custom.id, CUSTOM_WORKPIECE_MATERIAL_ID);
        assert_eq!(custom.category, MaterialCategory::Custom);
        assert_eq!(custom.name, "Shop 6063 Billet");
        assert!((custom.specific_cutting_energy_w_s_mm3 - 0.88).abs() < 1e-9);
        assert!((custom.specific_cutting_force_n_mm2 - 880.0).abs() < 1e-9);
        assert!((custom.min_chip_thickness_mm - 0.011).abs() < 1e-9);

        let mut bad_custom = ProjectMaterialSelection::custom("", f64::NAN, -10.0, 0.0);
        let resolved_bad = resolve_workpiece_material(Some(&bad_custom));
        assert_eq!(resolved_bad.name, "Custom Material");
        assert!(resolved_bad.specific_cutting_energy_w_s_mm3 > 0.0);
        assert!(resolved_bad.specific_cutting_force_n_mm2 > 0.0);
        assert!(resolved_bad.min_chip_thickness_mm > 0.0);
        bad_custom.custom_name = Some("Valid Name".to_string());
        assert_eq!(
            resolve_workpiece_material(Some(&bad_custom)).name,
            "Valid Name"
        );
    }

    #[test]
    fn match_best_preset_prefers_6063_on_carvera_1fl_oflutes_and_falls_back_cleanly() {
        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));
        let al6061 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6061_T6_ID,
        )));
        let wood = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            HARDWOOD_PLYWOOD_ID,
        )));
        let steel = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            MILD_STEEL_1018_ID,
        )));
        let delrin =
            resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(ACETAL_DELRIN_ID)));

        for machine in [
            builtin_carvera_machine(),
            builtin_carvera_air_machine(),
            builtin_carvera_z1_machine(),
        ] {
            // T1 (3.175x12 1FL) and T2 (3.175x25 1FL) must pick 6063 preset for 6063-T5 and 6061 preset for 6061-T6
            let t1 = machine.tools.iter().find(|t| t.tool_number == 1).unwrap();
            let t2 = machine.tools.iter().find(|t| t.tool_number == 2).unwrap();
            let t3 = machine.tools.iter().find(|t| t.tool_number == 3).unwrap();
            let t4 = machine.tools.iter().find(|t| t.tool_number == 4).unwrap();

            for t in [t1, t2, t3, t4] {
                let p6063 = match_best_preset_for_material(t, &al6063).unwrap();
                assert!(
                    p6063.material.contains("6063"),
                    "machine '{}' T{} must pick 6063 preset for aluminum-6063-t5, got '{}'",
                    machine.id,
                    t.tool_number,
                    p6063.name
                );
                assert_eq!(p6063.coolant_override, Some(CoolantMode::AirBlast));

                let p6061 = match_best_preset_for_material(t, &al6061).unwrap();
                assert!(
                    p6061.material.contains("6061"),
                    "machine '{}' T{} must keep picking 6061 preset for aluminum-6061-t6, got '{}'",
                    machine.id,
                    t.tool_number,
                    p6061.name
                );
            }

            // T2 6063-T5 preset must have safer reduced stepdown & stepover than its 6061 preset
            let t2_6061 = match_best_preset_for_material(t2, &al6061).unwrap();
            let t2_6063 = match_best_preset_for_material(t2, &al6063).unwrap();
            assert!(t2_6063.default_stepdown_mm < t2_6061.default_stepdown_mm);
            assert!(t2_6063.default_stepover_mm < t2_6061.default_stepover_mm);

            // T1 with Hardwood picks the wood preset
            let t1_wood = match_best_preset_for_material(t1, &wood).unwrap();
            assert!(t1_wood.id.ends_with("_wood"));
        }

        // Generic starter library T1 has no 6063 preset -> falls back to Aluminum 6061 preset
        let starter = default_starter_tool_library();
        let starter_t1 = &starter.tools[0];
        let matched_6063 = match_best_preset_for_material(starter_t1, &al6063).unwrap();
        assert_eq!(matched_6063.id, "al6061_rough");
        assert_eq!(
            match_best_preset_for_material(starter_t1, &wood)
                .unwrap()
                .id,
            "plywood"
        );
        assert_eq!(
            match_best_preset_for_material(starter_t1, &delrin)
                .unwrap()
                .id,
            "delrin"
        );
        assert_eq!(
            match_best_preset_for_material(starter_t1, &steel)
                .unwrap()
                .id,
            "steel1018"
        );
    }
}
