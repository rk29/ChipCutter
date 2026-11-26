//! Shared Avoid / Keep-Out Region resolution kernel (`chip-cam`).
//!
//! Resolves per-operation (`Operation::avoid`) and linked reusable project-level
//! (`ProjectDocument::avoid_regions`) keep-out regions—spanning 3D B-Rep faces/loops/holes,
//! 2D sketch regions/entities/layers, and Z-band-overlapping 3D workholding fixtures—into:
//! - 2D XY [`ResolvedAvoidRegion2D`] footprints (`nominal_region` + tool-center `effective_region`)
//!   for all 2.5D, 3-axis surface, undercut, and 3D deburr strategies (plus M86 viewport preview), and
//! - 1D rotary axial [`ResolvedAvoidAxialBands`] intervals `[u_lo, u_hi]` along the 4th-axis rotary
//!   axis for `Rotary2Linear` and `Rotary4AxisSim`.

use std::collections::HashSet;

use chip_bvh::TriMesh3D;
use chip_cad::{CadDocument, FaceId};
use chip_math::{DVec3, DisplayUnit, RigidFrame3};
use chip_sketch::SketchLayer;
use chip_tools::Tool;

use crate::{
    fixture_avoidance::ToolAssemblyProfile,
    offset2d::PocketRegion2D,
    params::{AvoidRegionConfig, SharedAvoidRegion},
    rest2d::RegionSet2D,
    rotary::{ResolvedRotaryConfig, project_selections_onto_rotary_axis},
    selection::{
        ExtractedFeature25D, GeometrySelectionItem, extract_rotary_selection_features,
        extract_selection_features_with_sketches,
    },
    setup::ResolvedSetup,
    surface3d::{SurfaceMesh3D, project_surface_z_slab_xy},
    wcs::CamError,
    workholding::ResolvedFixtureSolid,
};

/// Resolved 2D XY Avoid / Keep-Out footprints in the active Setup WCS frame.
#[derive(Debug, Clone, Default)]
pub struct ResolvedAvoidRegion2D {
    /// Raw union of all un-inflated keep-out footprints (selections + Z-overlapping fixtures),
    /// shared with 3D viewport preview rendering.
    pub nominal_region: RegionSet2D,
    /// Effective tool-center exclusion region after per-source inflation by
    /// `tool_r + eff_radial_clearance` (plus upper shank/holder tiers where fixtures rise above
    /// cutter flutes).
    pub effective_region: RegionSet2D,
    /// Non-fatal diagnostic warnings (e.g., missing shared region ID, unclosed open chain).
    pub warnings: Vec<String>,
}

impl ResolvedAvoidRegion2D {
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.effective_region.is_empty() && self.nominal_region.is_empty()
    }
}

/// Input bundle for [`resolve_avoid_region_2d`].
pub struct AvoidRegionInput<'a> {
    pub doc: &'a CadDocument,
    pub sketches: &'a [SketchLayer],
    pub setup: &'a ResolvedSetup,
    pub avoid: &'a AvoidRegionConfig,
    pub shared_regions: &'a [SharedAvoidRegion],
    pub fixtures: &'a [ResolvedFixtureSolid],
    pub tool: Option<&'a Tool>,
    pub tool_radius_mm: f64,
    pub z_bottom_wcs: f64,
    pub z_top_wcs: f64,
    pub display_unit: DisplayUnit,
}

/// Resolved 1D excluded axial intervals `[u_lo, u_hi]` (`mm` along the rotary axis) for
/// `Rotary2Linear` and `Rotary4AxisSim` (plus M86's `BoundaryPreviewOverlay::RotarySector`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ResolvedAvoidAxialBands {
    /// Raw un-inflated keep-out intervals `[u_lo, u_hi]` (`mm`), sorted and merged.
    pub nominal_bands: Vec<(f64, f64)>,
    /// Tool-center excluded intervals `[u_lo - (tool_r + axial_clearance), u_hi + (tool_r + axial_clearance)]`,
    /// sorted and merged.
    pub effective_bands: Vec<(f64, f64)>,
    /// Non-fatal diagnostic warnings.
    pub warnings: Vec<String>,
}

impl ResolvedAvoidAxialBands {
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.effective_bands.is_empty() && self.nominal_bands.is_empty()
    }
}

/// Input bundle for [`resolve_avoid_axial_bands`].
pub struct AvoidAxialBandsInput<'a> {
    pub doc: &'a CadDocument,
    pub sketches: &'a [SketchLayer],
    pub setup: &'a ResolvedSetup,
    pub rot: &'a ResolvedRotaryConfig,
    pub avoid: &'a AvoidRegionConfig,
    pub shared_regions: &'a [SharedAvoidRegion],
    pub fixtures: &'a [ResolvedFixtureSolid],
    pub tool: Option<&'a Tool>,
    pub tool_radius_mm: f64,
    pub r_floor: f64,
    pub r_outer: f64,
    pub display_unit: DisplayUnit,
}

struct AvoidSourceGroup<'a> {
    selections: &'a [GeometrySelectionItem],
    fixture_ids: &'a [String],
    include_all_fixtures: bool,
    radial_clearance_mm: f64,
    axial_clearance_mm: f64,
}

fn inflate_region(reg: &RegionSet2D, delta_mm: f64) -> RegionSet2D {
    if reg.is_empty() {
        return RegionSet2D::empty();
    }
    if delta_mm > 1e-5 {
        reg.dilate_outward(delta_mm)
    } else {
        reg.clone()
    }
}

fn finite_non_neg(v: f64) -> f64 {
    if v.is_finite() { v.max(0.0) } else { 0.0 }
}

/// Resolves all per-operation and linked non-suppressed [`SharedAvoidRegion`] entries into
/// 2D XY `nominal_region` and tool-center `effective_region` footprints in `input.setup`'s WCS.
pub fn resolve_avoid_region_2d(input: &AvoidRegionInput<'_>) -> ResolvedAvoidRegion2D {
    if input.avoid.is_empty() {
        return ResolvedAvoidRegion2D::default();
    }

    let mut eval_ctx = input.setup.eval_context(input.display_unit);
    if let Some(tool) = input.tool {
        tool.populate_eval_context(&mut eval_ctx);
    }

    let mut eval_avoid = input.avoid.clone();
    eval_avoid.reevaluate(&eval_ctx);

    let base_rad_clr = finite_non_neg(eval_avoid.radial_clearance.value_mm);
    let base_ax_clr = finite_non_neg(eval_avoid.axial_clearance.value_mm);
    let tool_r = input
        .tool
        .map(|t| 0.5 * t.geometry.cutting_diameter_mm().max(0.1))
        .unwrap_or_else(|| input.tool_radius_mm.max(0.0));
    let assembly = input.tool.map(ToolAssemblyProfile::from_tool);

    let mut warnings = Vec::new();
    let mut groups: Vec<AvoidSourceGroup<'_>> =
        Vec::with_capacity(1 + input.avoid.shared_region_ids.len());

    // 1. Operation-local source group
    if !input.avoid.selections.is_empty()
        || !input.avoid.fixture_ids.is_empty()
        || input.avoid.avoid_all_fixtures
    {
        groups.push(AvoidSourceGroup {
            selections: &input.avoid.selections,
            fixture_ids: &input.avoid.fixture_ids,
            include_all_fixtures: input.avoid.avoid_all_fixtures,
            radial_clearance_mm: base_rad_clr,
            axial_clearance_mm: base_ax_clr,
        });
    }

    // 2. Linked shared avoid regions (respecting `suppressed` and optional per-region clearances)
    let mut seen_shared: HashSet<&str> = HashSet::new();
    for sid in &input.avoid.shared_region_ids {
        if !seen_shared.insert(sid.as_str()) {
            continue;
        }
        let Some(shared) = input.shared_regions.iter().find(|r| r.id == *sid) else {
            warnings.push(format!(
                "Linked shared Avoid / Keep-Out region '{sid}' was not found; skipping."
            ));
            continue;
        };
        if shared.suppressed || shared.is_empty() {
            continue;
        }
        let mut eval_shared = shared.clone();
        eval_shared.reevaluate(&eval_ctx);
        let eff_rad = eval_shared
            .radial_clearance
            .as_ref()
            .map_or(base_rad_clr, |p| finite_non_neg(p.value_mm));
        let eff_ax = eval_shared
            .axial_clearance
            .as_ref()
            .map_or(base_ax_clr, |p| finite_non_neg(p.value_mm));
        groups.push(AvoidSourceGroup {
            selections: &shared.selections,
            fixture_ids: &shared.fixture_ids,
            include_all_fixtures: false,
            radial_clearance_mm: eff_rad,
            axial_clearance_mm: eff_ax,
        });
    }

    if groups.is_empty() {
        return ResolvedAvoidRegion2D {
            nominal_region: RegionSet2D::empty(),
            effective_region: RegionSet2D::empty(),
            warnings,
        };
    }

    let z_bottom = input.z_bottom_wcs.min(input.z_top_wcs);
    let z_top = input.z_top_wcs.max(input.z_bottom_wcs);
    let clearance_z = input.setup.clearance_z_wcs;

    let mut nominal_parts: Vec<RegionSet2D> = Vec::new();
    let mut effective_parts: Vec<RegionSet2D> = Vec::new();

    for group in groups {
        let slab_z_lo = z_bottom - group.axial_clearance_mm;
        let slab_z_hi = z_top.max(clearance_z) + group.axial_clearance_mm;

        // A. Selection keep-outs (3D B-Rep faces/loops/holes + 2D sketch regions/entities/layers)
        if !group.selections.is_empty() {
            let sel_nominal = extract_avoid_selections_2d(
                input.doc,
                input.sketches,
                input.setup,
                group.selections,
                slab_z_lo,
                slab_z_hi,
                &mut warnings,
            );
            if !sel_nominal.is_empty() {
                let eff = inflate_region(&sel_nominal, tool_r + group.radial_clearance_mm);
                if !eff.is_empty() {
                    effective_parts.push(eff);
                }
                nominal_parts.push(sel_nominal);
            }
        }

        // B. Fixture keep-outs (Z-band overlapping Setup fixtures)
        let selected_fixtures = select_fixtures_for_group(
            input.fixtures,
            group.include_all_fixtures,
            group.fixture_ids,
        );
        for fix in selected_fixtures {
            if !fix.aabb_wcs.is_valid() {
                continue;
            }
            if fix.is_spoilboard {
                let eff_top = fix.aabb_wcs.max.z - fix.allowed_overcut_mm.max(0.0);
                if eff_top <= z_bottom + 1e-4 {
                    continue;
                }
            } else if fix.aabb_wcs.max.z < slab_z_lo - 1e-4 || fix.aabb_wcs.min.z > slab_z_hi + 1e-4
            {
                continue;
            }

            let fix_surf = SurfaceMesh3D::from_tri_mesh(fix.mesh_wcs.clone());
            let span_xy = fix.aabb_wcs.size().x.max(fix.aabb_wcs.size().y).max(1.0);
            let cell_mm = (span_xy / 240.0).clamp(0.08, 0.25);

            // Project the fixture slab overlapping the operation's vertical Z band onto XY.
            let proj_z_lo = if fix.is_spoilboard {
                z_bottom - group.axial_clearance_mm
            } else {
                slab_z_lo
            };
            let proj_z_hi = slab_z_hi.max(fix.aabb_wcs.min.z + 0.1);
            let projected = project_surface_z_slab_xy(&fix_surf, proj_z_lo, proj_z_hi, cell_mm);
            let nominal_fix = if projected.is_empty() {
                fix.footprint_2d.clone()
            } else {
                projected
            };
            if nominal_fix.is_empty() {
                continue;
            }

            let flute_eff = inflate_region(&nominal_fix, tool_r + group.radial_clearance_mm);
            if !flute_eff.is_empty() {
                effective_parts.push(flute_eff);
            }
            nominal_parts.push(nominal_fix);

            // Protect wider non-cutting shank and holder/collet tiers whenever the fixture rises
            // above the cutter's flute or stickout height at `z_bottom` (prevents Rule 6 Shank/Holder
            // FixtureCollision on deep pockets/roughing next to tall clamps!).
            if let Some(asm) = assembly {
                if asm.shank_radius_mm > tool_r + 1e-3 {
                    let sh_lo = z_bottom + asm.flute_length_mm - group.axial_clearance_mm;
                    let sh_hi = z_bottom + asm.stickout_length_mm + group.axial_clearance_mm;
                    if fix.aabb_wcs.max.z >= sh_lo - 1e-4 && fix.aabb_wcs.min.z <= sh_hi + 1e-4 {
                        let sh_proj = project_surface_z_slab_xy(&fix_surf, sh_lo, sh_hi, cell_mm);
                        if !sh_proj.is_empty() {
                            let sh_eff = inflate_region(
                                &sh_proj,
                                asm.shank_radius_mm + group.radial_clearance_mm,
                            );
                            if !sh_eff.is_empty() {
                                effective_parts.push(sh_eff);
                            }
                        }
                    }
                }

                let nut_lo = z_bottom + asm.stickout_length_mm - group.axial_clearance_mm;
                if fix.aabb_wcs.max.z > nut_lo + 1e-4 {
                    let nut_hi = (z_bottom + asm.holder_cone_z0 + group.axial_clearance_mm)
                        .min(fix.aabb_wcs.max.z + group.axial_clearance_mm);
                    if nut_hi > nut_lo + 1e-4 {
                        let nut_proj =
                            project_surface_z_slab_xy(&fix_surf, nut_lo, nut_hi, cell_mm);
                        if !nut_proj.is_empty() {
                            let nut_eff = inflate_region(
                                &nut_proj,
                                asm.holder_nut_radius_mm + group.radial_clearance_mm,
                            );
                            if !nut_eff.is_empty() {
                                effective_parts.push(nut_eff);
                            }
                        }
                    }
                    let body_lo = z_bottom + asm.holder_cone_z0 - group.axial_clearance_mm;
                    if fix.aabb_wcs.max.z > body_lo + 1e-4 {
                        let body_proj = project_surface_z_slab_xy(
                            &fix_surf,
                            body_lo,
                            fix.aabb_wcs.max.z + group.axial_clearance_mm,
                            cell_mm,
                        );
                        if !body_proj.is_empty() {
                            let body_eff = inflate_region(
                                &body_proj,
                                asm.holder_body_radius_mm + group.radial_clearance_mm,
                            );
                            if !body_eff.is_empty() {
                                effective_parts.push(body_eff);
                            }
                        }
                    }
                }
            }
        }
    }

    ResolvedAvoidRegion2D {
        nominal_region: RegionSet2D::union_many(&mut nominal_parts),
        effective_region: RegionSet2D::union_many(&mut effective_parts),
        warnings,
    }
}

fn select_fixtures_for_group<'a>(
    fixtures: &'a [ResolvedFixtureSolid],
    include_all: bool,
    fixture_ids: &[String],
) -> Vec<&'a ResolvedFixtureSolid> {
    if fixtures.is_empty() {
        return Vec::new();
    }
    if include_all {
        return fixtures.iter().collect();
    }
    let mut seen: HashSet<&str> = HashSet::new();
    let mut out = Vec::new();
    for fid in fixture_ids {
        if !seen.insert(fid.as_str()) {
            continue;
        }
        if let Some(f) = fixtures.iter().find(|f| f.fixture_id == *fid) {
            out.push(f);
        }
    }
    out
}

fn extract_avoid_selections_2d(
    doc: &CadDocument,
    sketches: &[SketchLayer],
    setup: &ResolvedSetup,
    selections: &[GeometrySelectionItem],
    slab_z_lo: f64,
    slab_z_hi: f64,
    warnings: &mut Vec<String>,
) -> RegionSet2D {
    let mut sets = Vec::new();

    for item in selections {
        match extract_selection_features_with_sketches(
            doc,
            sketches,
            setup,
            std::slice::from_ref(item),
        ) {
            Ok(summary) => {
                for feat in &summary.features {
                    match feat {
                        ExtractedFeature25D::PlanarFace {
                            outer_boundary_wcs,
                            inner_islands_wcs,
                            ..
                        } => {
                            if outer_boundary_wcs.closed && !outer_boundary_wcs.segments.is_empty()
                            {
                                let outer = outer_boundary_wcs.ensure_orientation(true);
                                let islands = inner_islands_wcs
                                    .iter()
                                    .filter(|i| i.closed && !i.segments.is_empty())
                                    .map(|i| i.ensure_orientation(false))
                                    .collect();
                                let reg =
                                    RegionSet2D::from_single(PocketRegion2D::new(outer, islands));
                                if !reg.is_empty() {
                                    sets.push(reg);
                                }
                            }
                        }
                        ExtractedFeature25D::Contour { polyline_wcs, .. } => {
                            if polyline_wcs.closed && !polyline_wcs.segments.is_empty() {
                                let outer = polyline_wcs.ensure_orientation(true);
                                let reg = RegionSet2D::from_single(PocketRegion2D::new(
                                    outer,
                                    Vec::new(),
                                ));
                                if !reg.is_empty() {
                                    sets.push(reg);
                                }
                            } else {
                                warnings.push(
                                    "Avoid / Keep-Out ignored an open contour (use closed loops, faces, holes, or sketch regions)."
                                        .to_string(),
                                );
                            }
                        }
                        ExtractedFeature25D::CylindricalHole {
                            center_wcs,
                            diameter_mm,
                            ..
                        } => {
                            let r = 0.5 * diameter_mm.abs();
                            if r > 1e-4 {
                                let reg = RegionSet2D::from_circle(*center_wcs, r);
                                if !reg.is_empty() {
                                    sets.push(reg);
                                }
                            }
                        }
                        ExtractedFeature25D::ModeledChamfer { .. } => {}
                    }
                }
            }
            Err(
                CamError::FaceNotHorizontalInWcs { .. } | CamError::HoleNotAlignedWithWcsZ { .. },
            ) => {
                // Non-horizontal 3D B-Rep face (e.g. a shared keep-out face projected under an
                // `Indexed3Plus1` rotated frame): project its tessellated triangles onto virtual XY.
                if let GeometrySelectionItem::PlanarFace { binding }
                | GeometrySelectionItem::CylindricalHole { binding } = item
                {
                    let fid = FaceId(binding.last_index);
                    let projected =
                        project_cad_face_xy(doc, fid, &setup.frame, slab_z_lo, slab_z_hi);
                    if !projected.is_empty() {
                        sets.push(projected);
                    }
                }
            }
            Err(CamError::SketchNotAlignedWithSetupZ { sketch_name, .. }) => {
                warnings.push(format!(
                    "Avoid / Keep-Out sketch '{sketch_name}' is not parallel to the active Setup XY plane; skipping."
                ));
            }
            Err(err) => {
                warnings.push(format!(
                    "Avoid / Keep-Out selection could not be extracted: {err}"
                ));
            }
        }
    }

    RegionSet2D::union_many(&mut sets)
}

/// Projects a single B-Rep face's pick triangles transformed into `frame` onto XY across `[z_lo, z_hi]`.
fn project_cad_face_xy(
    doc: &CadDocument,
    face_id: FaceId,
    frame: &RigidFrame3,
    z_lo: f64,
    z_hi: f64,
) -> RegionSet2D {
    let tris: Vec<[DVec3; 3]> = doc
        .render_mesh
        .pick_triangles
        .iter()
        .filter(|t| t.face_id == face_id)
        .map(|t| {
            [
                frame.world_to_local_point(t.v0_mm),
                frame.world_to_local_point(t.v1_mm),
                frame.world_to_local_point(t.v2_mm),
            ]
        })
        .collect();
    if tris.is_empty() {
        return RegionSet2D::empty();
    }
    let surf = SurfaceMesh3D::from_raw_triangles(&tris);
    if !surf.aabb_wcs.is_valid() {
        return RegionSet2D::empty();
    }
    let span = surf.aabb_wcs.size().x.max(surf.aabb_wcs.size().y).max(1.0);
    let cell = (span / 160.0).clamp(0.08, 0.25);
    project_surface_z_slab_xy(&surf, z_lo, z_hi, cell)
}

/// Rigidly transforms a slice of [`ResolvedFixtureSolid`]s from `src_frame` into `dst_frame`
/// (rebuilding exact `f64` meshes, AABBs, and `FlatBvh` indexes for `Indexed3Plus1` virtual setups).
pub fn transform_fixtures_to_frame(
    fixtures: &[ResolvedFixtureSolid],
    src_frame: &RigidFrame3,
    dst_frame: &RigidFrame3,
) -> Vec<ResolvedFixtureSolid> {
    if fixtures.is_empty() {
        return Vec::new();
    }
    let same_frame = src_frame.origin_mm.distance_squared(dst_frame.origin_mm) <= 1e-18
        && src_frame.x_axis.distance_squared(dst_frame.x_axis) <= 1e-18
        && src_frame.y_axis.distance_squared(dst_frame.y_axis) <= 1e-18
        && src_frame.z_axis.distance_squared(dst_frame.z_axis) <= 1e-18;
    if same_frame {
        return fixtures.to_vec();
    }

    fixtures
        .iter()
        .map(|fix| {
            let new_verts: Vec<DVec3> = fix
                .mesh_wcs
                .vertices
                .iter()
                .map(|&v| dst_frame.world_to_local_point(src_frame.local_to_world_point(v)))
                .collect();
            let indices: Vec<[u32; 3]> = fix.mesh_wcs.triangles.iter().map(|t| t.indices).collect();
            let new_mesh = TriMesh3D::from_indexed(new_verts, &indices);
            let new_aabb = new_mesh.bounds;
            let new_surf = SurfaceMesh3D::from_tri_mesh(new_mesh.clone());
            let new_footprint = if new_aabb.is_valid() {
                let span = new_aabb.size().x.max(new_aabb.size().y).max(1.0);
                let cell = (span / 200.0).clamp(0.08, 0.30);
                project_surface_z_slab_xy(
                    &new_surf,
                    new_aabb.min.z - 0.5,
                    new_aabb.max.z + 0.5,
                    cell,
                )
            } else {
                RegionSet2D::empty()
            };
            ResolvedFixtureSolid {
                fixture_id: fix.fixture_id.clone(),
                name: fix.name.clone(),
                mesh_wcs: new_mesh,
                aabb_wcs: new_aabb,
                footprint_2d: new_footprint,
                clearance_margin_mm: fix.clearance_margin_mm,
                allowed_overcut_mm: fix.allowed_overcut_mm,
                is_spoilboard: fix.is_spoilboard,
                color_rgba: fix.color_rgba,
            }
        })
        .collect()
}

/// Resolves 1D excluded axial intervals `[u_lo, u_hi]` (`mm` along `input.rot.axis`) from
/// `input.avoid` + linked non-suppressed [`SharedAvoidRegion`]s + radial-overlapping fixtures.
pub fn resolve_avoid_axial_bands(input: &AvoidAxialBandsInput<'_>) -> ResolvedAvoidAxialBands {
    if input.avoid.is_empty() {
        return ResolvedAvoidAxialBands::default();
    }

    let mut eval_ctx = input.setup.eval_context(input.display_unit);
    if let Some(tool) = input.tool {
        tool.populate_eval_context(&mut eval_ctx);
    }

    let mut eval_avoid = input.avoid.clone();
    eval_avoid.reevaluate(&eval_ctx);

    let base_rad_clr = finite_non_neg(eval_avoid.radial_clearance.value_mm);
    let base_ax_clr = finite_non_neg(eval_avoid.axial_clearance.value_mm);
    let tool_r = input
        .tool
        .map(|t| 0.5 * t.geometry.cutting_diameter_mm().max(0.1))
        .unwrap_or_else(|| input.tool_radius_mm.max(0.0));

    let mut warnings = Vec::new();
    let mut groups: Vec<AvoidSourceGroup<'_>> =
        Vec::with_capacity(1 + input.avoid.shared_region_ids.len());

    if !input.avoid.selections.is_empty()
        || !input.avoid.fixture_ids.is_empty()
        || input.avoid.avoid_all_fixtures
    {
        groups.push(AvoidSourceGroup {
            selections: &input.avoid.selections,
            fixture_ids: &input.avoid.fixture_ids,
            include_all_fixtures: input.avoid.avoid_all_fixtures,
            radial_clearance_mm: base_rad_clr,
            axial_clearance_mm: base_ax_clr,
        });
    }

    let mut seen_shared: HashSet<&str> = HashSet::new();
    for sid in &input.avoid.shared_region_ids {
        if !seen_shared.insert(sid.as_str()) {
            continue;
        }
        let Some(shared) = input.shared_regions.iter().find(|r| r.id == *sid) else {
            warnings.push(format!(
                "Linked shared Avoid / Keep-Out region '{sid}' was not found; skipping."
            ));
            continue;
        };
        if shared.suppressed || shared.is_empty() {
            continue;
        }
        let mut eval_shared = shared.clone();
        eval_shared.reevaluate(&eval_ctx);
        let eff_rad = eval_shared
            .radial_clearance
            .as_ref()
            .map_or(base_rad_clr, |p| finite_non_neg(p.value_mm));
        let eff_ax = eval_shared
            .axial_clearance
            .as_ref()
            .map_or(base_ax_clr, |p| finite_non_neg(p.value_mm));
        groups.push(AvoidSourceGroup {
            selections: &shared.selections,
            fixture_ids: &shared.fixture_ids,
            include_all_fixtures: false,
            radial_clearance_mm: eff_rad,
            axial_clearance_mm: eff_ax,
        });
    }

    let (e1, e2, u_vec) = input.rot.axis.basis();
    let r_clear_lim = input
        .rot
        .clearance_radius_mm
        .max(input.r_outer + tool_r + 5.0);

    let mut nominal_raw: Vec<(f64, f64)> = Vec::new();
    let mut effective_raw: Vec<(f64, f64)> = Vec::new();

    for group in groups {
        let pad_u = tool_r + group.axial_clearance_mm;

        // A. Project selections onto rotary axis `u_vec`
        for item in group.selections {
            let extracted_opt = extract_rotary_selection_features(
                input.doc,
                input.sketches,
                input.setup,
                std::slice::from_ref(item),
            )
            .ok();
            if let Some((u0, u1)) = project_selections_onto_rotary_axis(
                Some(input.doc),
                &input.setup.frame,
                std::slice::from_ref(item),
                extracted_opt.as_ref(),
                input.rot,
            ) && u0.is_finite()
                && u1.is_finite()
            {
                let (lo, hi) = if u0 <= u1 { (u0, u1) } else { (u1, u0) };
                nominal_raw.push((lo, hi));
                effective_raw.push((lo - pad_u, hi + pad_u));
            }
        }

        // B. Project radial-overlapping fixtures onto rotary axis `u_vec`
        let selected_fixtures = select_fixtures_for_group(
            input.fixtures,
            group.include_all_fixtures,
            group.fixture_ids,
        );
        for fix in selected_fixtures {
            if !fix.aabb_wcs.is_valid() || fix.mesh_wcs.vertices.is_empty() {
                continue;
            }
            if fix.is_spoilboard
                && fix.aabb_wcs.max.z
                    <= input.rot.center_wcs.z - input.r_outer - group.radial_clearance_mm - 1e-4
            {
                continue;
            }
            let mut fu_min = f64::INFINITY;
            let mut fu_max = f64::NEG_INFINITY;
            let mut r_min = f64::INFINITY;
            let mut r_max = 0.0_f64;
            for &v in &fix.mesh_wcs.vertices {
                let u = v.dot(u_vec);
                let rel = v - input.rot.center_wcs;
                let r = rel.dot(e1).hypot(rel.dot(e2));
                fu_min = fu_min.min(u);
                fu_max = fu_max.max(u);
                r_min = r_min.min(r);
                r_max = r_max.max(r);
            }
            if !fu_min.is_finite() || !fu_max.is_finite() {
                continue;
            }
            // Check radial/vertical overlap with the rotary machining envelope
            let r_env_lo = (input.r_floor - group.radial_clearance_mm).max(0.0);
            let r_env_hi = r_clear_lim + group.radial_clearance_mm;
            let radially_disjoint = r_max < r_env_lo - 1e-4 || r_min > r_env_hi + 1e-4;
            if radially_disjoint {
                continue;
            }
            nominal_raw.push((fu_min, fu_max));
            effective_raw.push((fu_min - pad_u, fu_max + pad_u));
        }
    }

    ResolvedAvoidAxialBands {
        nominal_bands: merge_u_intervals_1d(nominal_raw),
        effective_bands: merge_u_intervals_1d(effective_raw),
        warnings,
    }
}

/// Sorts and merges overlapping 1D intervals `[lo, hi]`.
pub fn merge_u_intervals_1d(mut intervals: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    intervals.retain(|&(a, b)| a.is_finite() && b.is_finite() && b >= a - 1e-9);
    if intervals.is_empty() {
        return Vec::new();
    }
    for (a, b) in &mut intervals {
        if *a > *b {
            std::mem::swap(a, b);
        }
    }
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
    let mut merged: Vec<(f64, f64)> = Vec::with_capacity(intervals.len());
    for (a, b) in intervals {
        if let Some(last) = merged.last_mut()
            && a <= last.1 + 1e-6
        {
            last.1 = last.1.max(b);
        } else {
            merged.push((a, b));
        }
    }
    merged
}

/// Subtracts sorted/merged `excluded` 1D bands from `domain = (u_start, u_end)`, returning
/// surviving allowed intervals with width `> min_span_mm`.
pub fn subtract_u_intervals_1d(
    domain: (f64, f64),
    excluded: &[(f64, f64)],
    min_span_mm: f64,
) -> Vec<(f64, f64)> {
    let (dom_lo, dom_hi) = if domain.0 <= domain.1 {
        domain
    } else {
        (domain.1, domain.0)
    };
    if dom_hi - dom_lo <= min_span_mm {
        return Vec::new();
    }
    if excluded.is_empty() {
        return vec![(dom_lo, dom_hi)];
    }

    let merged = merge_u_intervals_1d(excluded.to_vec());
    let mut allowed = Vec::new();
    let mut cursor = dom_lo;
    for (ex_lo, ex_hi) in merged {
        if ex_hi <= cursor + 1e-9 {
            continue;
        }
        if ex_lo >= dom_hi - 1e-9 {
            break;
        }
        let seg_hi = ex_lo.min(dom_hi);
        if seg_hi - cursor > min_span_mm {
            allowed.push((cursor, seg_hi));
        }
        cursor = cursor.max(ex_hi);
        if cursor >= dom_hi - 1e-9 {
            break;
        }
    }
    if dom_hi - cursor > min_span_mm {
        allowed.push((cursor, dom_hi));
    }
    allowed
}

/// Clips an open or closed [`chip_math::ArcPolyline2D`] strictly outside `avoid`, using binary
/// boundary refinement (`< 0.001 mm`) at every entry/exit crossing so long linear or arc
/// segments never overshoot into the keep-out region.
pub fn clip_polyline_outside_avoid(
    poly: &chip_math::ArcPolyline2D,
    avoid: &RegionSet2D,
) -> Vec<chip_math::ArcPolyline2D> {
    use chip_math::{ArcPolyline2D, ArcSegment2D};

    if poly.segments.is_empty() {
        return Vec::new();
    }
    if avoid.is_empty() {
        return vec![poly.clone()];
    }

    let sub_seg = |seg: &ArcSegment2D, t0: f64, t1: f64| -> Option<ArcSegment2D> {
        let a = t0.clamp(0.0, 1.0);
        let b = t1.clamp(0.0, 1.0);
        if b - a <= 1e-9 {
            return None;
        }
        match *seg {
            ArcSegment2D::Line { start, end } => {
                let p0 = start.lerp(end, a);
                let p1 = start.lerp(end, b);
                (p0.distance_squared(p1) > 1e-10)
                    .then_some(ArcSegment2D::Line { start: p0, end: p1 })
            }
            ArcSegment2D::Arc {
                center,
                radius,
                ccw,
                ..
            } => {
                let (p0, _) = seg.eval_at(a)?;
                let (p1, _) = seg.eval_at(b)?;
                (p0.distance_squared(p1) > 1e-10).then_some(ArcSegment2D::Arc {
                    start: p0,
                    end: p1,
                    center,
                    radius,
                    ccw,
                })
            }
        }
    };

    let is_out = |seg: &ArcSegment2D, t: f64| -> bool {
        seg.eval_at(t)
            .is_some_and(|(p, _)| !avoid.contains_point(p))
    };

    let refine_crossing = |seg: &ArcSegment2D, t_out: f64, t_in: f64| -> f64 {
        let mut lo = t_out;
        let mut hi = t_in;
        for _ in 0..12 {
            let mid = 0.5 * (lo + hi);
            if is_out(seg, mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };

    let mut active_segs: Vec<ArcSegment2D> = Vec::new();
    for seg in &poly.segments {
        let len = seg.length();
        if len <= 1e-5 {
            continue;
        }
        let n_steps = ((len / 0.25).ceil() as usize).clamp(4, 512);
        let mut prev_t = 0.0_f64;
        let mut prev_out = is_out(seg, 0.0);
        let mut span_start: Option<f64> = prev_out.then_some(0.0);

        for k in 1..=n_steps {
            let cur_t = (k as f64) / (n_steps as f64);
            let cur_out = is_out(seg, cur_t);
            match (prev_out, cur_out) {
                (true, true) => {
                    // Check midpoint to avoid straddling a narrow keep-out corner
                    let mid_t = 0.5 * (prev_t + cur_t);
                    if !is_out(seg, mid_t) {
                        let t_exit = refine_crossing(seg, prev_t, mid_t);
                        if let Some(s0) = span_start.take()
                            && let Some(sub) = sub_seg(seg, s0, t_exit)
                        {
                            active_segs.push(sub);
                        }
                        let t_reenter = refine_crossing(seg, cur_t, mid_t);
                        span_start = Some(t_reenter);
                    }
                }
                (true, false) => {
                    let t_exit = refine_crossing(seg, prev_t, cur_t);
                    if let Some(s0) = span_start.take()
                        && let Some(sub) = sub_seg(seg, s0, t_exit)
                    {
                        active_segs.push(sub);
                    }
                }
                (false, true) => {
                    let t_enter = refine_crossing(seg, cur_t, prev_t);
                    span_start = Some(t_enter);
                }
                (false, false) => {}
            }
            prev_t = cur_t;
            prev_out = cur_out;
        }
        if let Some(s0) = span_start
            && let Some(sub) = sub_seg(seg, s0, 1.0)
        {
            active_segs.push(sub);
        }
    }

    if active_segs.is_empty() {
        return Vec::new();
    }

    let finalize = |segs: Vec<ArcSegment2D>| -> ArcPolyline2D {
        let closed = segs.len() >= 2
            && segs
                .first()
                .zip(segs.last())
                .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-3);
        ArcPolyline2D {
            segments: segs,
            closed,
        }
    };

    let mut chains: Vec<ArcPolyline2D> = Vec::new();
    let mut cur: Vec<ArcSegment2D> = Vec::new();
    for s in active_segs {
        if let Some(prev) = cur.last()
            && prev.end().distance(s.start()) > 1e-3
        {
            chains.push(finalize(std::mem::take(&mut cur)));
        }
        cur.push(s);
    }
    if !cur.is_empty() {
        chains.push(finalize(cur));
    }

    if poly.closed
        && chains.len() >= 2
        && let (Some(first_pt), Some(last_pt)) = (
            chains[0].segments.first().map(ArcSegment2D::start),
            chains
                .last()
                .and_then(|c| c.segments.last())
                .map(ArcSegment2D::end),
        )
        && first_pt.distance(last_pt) <= 1e-3
    {
        let first = chains.remove(0);
        if let Some(last) = chains.last_mut() {
            last.segments.extend(first.segments);
            last.closed = false;
        }
    }

    chains
}
