//! Cached 3D viewport cutting-area preview overlays (`BoundaryPreviewOverlay`) shared between
//! the `[Geometry]` tab live readouts and `wgpu` wireframe/translucent viewport rendering.
//!
//! Strictly invalidated only when active setup/operation index, CAD revision, setup stock/WCS,
//! operation selections/boundary/heights, or tool diameter change — performing **zero** per-frame
//! mesh projections or polygon allocations on idle 60 FPS frames.

use std::{
    hash::{Hash as _, Hasher as _},
    sync::Arc,
};

use chip_cad::CadDocument;
use chip_cam::{
    ExtractedFeature25D, ExtractedSelectionSummary, MachiningBoundaryMode, Operation,
    OperationKind, PocketBoundaryMode, PocketRegion2D, RegionSet2D, ResolvedHeights,
    ResolvedRotaryConfig, ResolvedRotaryWindow, ResolvedSetup, ResolvedStockGeometry,
    RotaryWindowInput, StockAxis, StrategyParams, SurfaceMesh3D, ToolContainmentMode,
    rect_region_2d, resolve_3d_boundary_config_detailed, resolve_rotary_window, stock_xy_region,
};
use chip_math::{Aabb3, DVec2, DVec3, DisplayUnit, EvalContext, RigidFrame3};
use chip_tools::Tool;

/// Resolved 3D viewport cutting-area preview geometry rendered when `OperationTab::Geometry` is active.
#[derive(Debug, Clone, PartialEq)]
pub enum BoundaryPreviewOverlay {
    /// 2.5D (`Facing`) & 3-Axis (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`,
    /// `Pencil3D`, `Undercut3D`, `Deburr3D`) XY machining containment preview.
    Planar2D {
        /// Nominal cutting boundary loops before tool-radius / offset dilation.
        nominal_region: RegionSet2D,
        /// Effective tool-center containment boundary after `tool_containment` + `boundary_offset`.
        effective_region: RegionSet2D,
        /// `true` when `effective_region` differs meaningfully from `nominal_region`.
        show_effective: bool,
        /// Upper Z reference plane in Setup WCS (`heights.top_z_wcs`).
        top_z_wcs: f64,
        /// Lower Z reference plane in Setup WCS (`heights.bottom_z_wcs`).
        bottom_z_wcs: f64,
        /// Any non-fatal boundary warnings surfaced by resolution.
        warnings: Vec<String>,
    },
    /// 4th-Axis Rotary (`Rotary2Linear`, `Rotary4AxisSim`) cylindrical cutting sector & chuck keep-out.
    RotarySector {
        rotary: ResolvedRotaryConfig,
        window: ResolvedRotaryWindow,
    },
    /// `Pocket2D` / `Adaptive2D` in `ClearOutsideSelectedToStock` mode: outer stock clearing envelope.
    StockOuterEnvelope {
        stock_region: RegionSet2D,
        top_z_wcs: f64,
        bottom_z_wcs: f64,
    },
}

/// Inputs required to evaluate or retrieve a cached [`BoundaryPreviewOverlay`].
pub struct BoundaryPreviewInput<'a> {
    pub active: bool,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub cad_rev: u64,
    pub frame_geo_sig: u64,
    pub display_unit: DisplayUnit,
    pub doc: Option<&'a CadDocument>,
    pub resolved_setup: &'a ResolvedSetup,
    pub op: &'a Operation,
    pub active_tool: Option<&'a Tool>,
    pub extracted: Option<&'a ExtractedSelectionSummary>,
}

/// Persistent per-app cache holding the latest [`SurfaceMesh3D`] and [`BoundaryPreviewOverlay`].
#[derive(Debug, Default)]
pub struct BoundaryPreviewCache {
    last_sig: Option<u64>,
    cached_surface_sig: Option<u64>,
    cached_surface: Option<Arc<SurfaceMesh3D>>,
    overlay: Option<Arc<BoundaryPreviewOverlay>>,
    recompute_count: u64,
}

impl BoundaryPreviewCache {
    /// Clears cached surface mesh and boundary preview (e.g. on project replacement).
    pub fn clear(&mut self) {
        self.last_sig = None;
        self.cached_surface_sig = None;
        self.cached_surface = None;
        self.overlay = None;
    }

    /// Number of times a non-trivial boundary preview was recomputed (for cache-invalidation tests).
    #[inline]
    pub const fn recompute_count(&self) -> u64 {
        self.recompute_count
    }

    /// Returns the most recently cached WCS [`SurfaceMesh3D`], if any.
    #[inline]
    pub fn cached_surface(&self) -> Option<&SurfaceMesh3D> {
        self.cached_surface.as_deref()
    }

    /// Ensures the WCS [`SurfaceMesh3D`] for `(cad_rev, frame)` is cached when `doc` has 3D geometry.
    pub fn ensure_surface(
        &mut self,
        doc: Option<&CadDocument>,
        cad_rev: u64,
        frame: &RigidFrame3,
    ) -> Option<&SurfaceMesh3D> {
        let Some(cad) = doc.filter(|d| d.has_3d_geometry()) else {
            self.cached_surface_sig = None;
            self.cached_surface = None;
            return None;
        };
        let surf_sig = compute_surface_signature(cad_rev, frame);
        if self.cached_surface_sig != Some(surf_sig) {
            let mesh = SurfaceMesh3D::from_cad_document(cad, &[], frame);
            self.cached_surface_sig = Some(surf_sig);
            self.cached_surface = Some(Arc::new(mesh));
        }
        self.cached_surface.as_deref()
    }

    /// Resolves (or returns the cached) [`BoundaryPreviewOverlay`] for `input`.
    ///
    /// Returns `None` immediately without work when `input.active == false` or the operation does
    /// not display a cutting-area overlay.
    pub fn get_or_compute(
        &mut self,
        input: &BoundaryPreviewInput<'_>,
    ) -> Option<Arc<BoundaryPreviewOverlay>> {
        if !input.active {
            return None;
        }
        let sig = compute_boundary_preview_signature(input);
        if self.last_sig == Some(sig) {
            return self.overlay.clone();
        }

        let needs_surface = needs_surface_mesh(input.op);
        let surface_opt = if needs_surface {
            self.ensure_surface(input.doc, input.cad_rev, &input.resolved_setup.frame)
                .map(|s| s as *const SurfaceMesh3D)
        } else {
            None
        };
        // Re-borrow immutably from `self.cached_surface` safely
        let surface_ref = if surface_opt.is_some() {
            self.cached_surface.as_deref()
        } else {
            None
        };

        let computed = compute_overlay(input, surface_ref).map(Arc::new);
        self.last_sig = Some(sig);
        self.overlay = computed.clone();
        self.recompute_count += 1;
        computed
    }
}

fn needs_surface_mesh(op: &Operation) -> bool {
    if op.kind.is_rotary_native() {
        return true;
    }
    if let Some(b) = op.machining_boundary() {
        return match op.kind {
            OperationKind::Facing => b.boundary_mode == MachiningBoundaryMode::ModelSilhouette,
            OperationKind::Adaptive3D
            | OperationKind::Waterline3D
            | OperationKind::Parallel3D
            | OperationKind::Scallop3D
            | OperationKind::Pencil3D
            | OperationKind::Undercut3D
            | OperationKind::Deburr3D => {
                matches!(
                    b.boundary_mode,
                    MachiningBoundaryMode::ModelSilhouette | MachiningBoundaryMode::SelectedRegions
                )
            }
            _ => false,
        };
    }
    false
}

fn compute_surface_signature(cad_rev: u64, frame: &RigidFrame3) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    cad_rev.hash(&mut h);
    for v in [frame.origin_mm, frame.x_axis, frame.y_axis, frame.z_axis] {
        v.x.to_bits().hash(&mut h);
        v.y.to_bits().hash(&mut h);
        v.z.to_bits().hash(&mut h);
    }
    h.finish()
}

fn compute_boundary_preview_signature(input: &BoundaryPreviewInput<'_>) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    input.setup_idx.hash(&mut h);
    input.op_idx.hash(&mut h);
    input.cad_rev.hash(&mut h);
    input.frame_geo_sig.hash(&mut h);
    input.op.id.hash(&mut h);
    std::mem::discriminant(&input.op.kind).hash(&mut h);
    format!("{:?}", input.display_unit).hash(&mut h);

    let tool_d = input
        .active_tool
        .map_or(0.0, |t| t.geometry.cutting_diameter_mm());
    tool_d.to_bits().hash(&mut h);

    format!("{:?}", input.op.heights).hash(&mut h);

    if let Some(b) = input.op.machining_boundary() {
        1_u8.hash(&mut h);
        std::mem::discriminant(&b.boundary_mode).hash(&mut h);
        std::mem::discriminant(&b.tool_containment).hash(&mut h);
        b.boundary_offset.value_mm.to_bits().hash(&mut h);
        b.boundary_offset.expr.hash(&mut h);
        b.silhouette_fill_holes.hash(&mut h);
    } else if let Some(rb) = input.op.rotary_axial_boundary() {
        2_u8.hash(&mut h);
        std::mem::discriminant(&rb.axial_boundary_mode).hash(&mut h);
        std::mem::discriminant(&rb.axial_containment).hash(&mut h);
        rb.axial_margin_start.value_mm.to_bits().hash(&mut h);
        rb.axial_margin_start.expr.hash(&mut h);
        rb.axial_margin_end.value_mm.to_bits().hash(&mut h);
        rb.axial_margin_end.expr.hash(&mut h);
        rb.explicit_u_start.value_mm.to_bits().hash(&mut h);
        rb.explicit_u_start.expr.hash(&mut h);
        rb.explicit_u_end.value_mm.to_bits().hash(&mut h);
        rb.explicit_u_end.expr.hash(&mut h);
        rb.start_angle_deg.to_bits().hash(&mut h);
        rb.end_angle_deg.to_bits().hash(&mut h);
    } else if let StrategyParams::Pocket2D(p) = input.op.resolved_strategy() {
        3_u8.hash(&mut h);
        std::mem::discriminant(&p.boundary_mode).hash(&mut h);
        p.treat_outer_boundary_as_open.hash(&mut h);
    } else {
        0_u8.hash(&mut h);
    }
    h.finish()
}

/// Resolves operation heights for preview using the active setup, tool, and extracted feature bounds.
pub fn resolve_preview_heights(
    op: &Operation,
    rs: &ResolvedSetup,
    active_tool: Option<&Tool>,
    extracted: Option<&ExtractedSelectionSummary>,
    display_unit: DisplayUnit,
) -> Option<ResolvedHeights> {
    let mut eval_ctx = rs.eval_context(display_unit);
    if let Some(tool) = active_tool {
        tool.populate_eval_context(&mut eval_ctx);
    }
    let (inf_top, inf_bot) = if let Some(sum) = extracted {
        eval_ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
        eval_ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
        (sum.inferred_top_z_wcs, sum.inferred_bottom_z_wcs)
    } else {
        (rs.stock.aabb_wcs().max.z, rs.model_aabb_wcs.max.z)
    };
    let mut h_eval = op.heights.clone();
    h_eval.reevaluate(&eval_ctx);
    ResolvedHeights::resolve(
        &h_eval,
        &rs.model_aabb_wcs,
        &rs.stock.aabb_wcs(),
        rs.clearance_z_wcs,
        rs.retract_z_wcs,
        inf_top,
        inf_bot,
    )
    .ok()
}

/// Evaluates `resolve_rotary_window` for a rotary operation, falling back to an 8-corner AABB
/// proxy mesh when no full [`SurfaceMesh3D`] is supplied (e.g. in standalone headless UI tests).
#[allow(clippy::too_many_arguments)]
pub fn resolve_rotary_window_for_preview(
    op: &Operation,
    rs: &ResolvedSetup,
    doc: Option<&CadDocument>,
    surface_mesh: Option<&SurfaceMesh3D>,
    extracted: Option<&ExtractedSelectionSummary>,
    active_tool: Option<&Tool>,
    resolved_heights: Option<&ResolvedHeights>,
    eval_ctx: &EvalContext,
) -> Option<Result<ResolvedRotaryWindow, String>> {
    let rot = rs.rotary?;
    let mut boundary = op.rotary_axial_boundary()?.clone();
    boundary.reevaluate(eval_ctx);

    let tool_d = active_tool
        .map(|t| t.geometry.cutting_diameter_mm())
        .unwrap_or(6.0);
    let stock_aabb = rs.stock.aabb_wcs();
    let top_z_wcs = resolved_heights.map_or(stock_aabb.max.z, |rh| rh.top_z_wcs);
    let bottom_z_wcs = resolved_heights.map_or(rot.center_wcs.z, |rh| rh.bottom_z_wcs);

    let fallback_mesh;
    let mesh_ref: &SurfaceMesh3D = if let Some(m) = surface_mesh {
        m
    } else {
        fallback_mesh = aabb_proxy_surface_mesh(&rs.model_aabb_wcs);
        &fallback_mesh
    };

    Some(
        resolve_rotary_window(&RotaryWindowInput {
            rot: &rot,
            frame: &rs.frame,
            doc,
            base_mesh: mesh_ref,
            selections: &op.selections,
            extracted_selection: extracted,
            boundary: &boundary,
            tool_diameter_mm: tool_d,
            top_z_wcs,
            bottom_z_wcs,
            op_name: &op.name,
        })
        .map_err(|e| e.to_string()),
    )
}

fn aabb_proxy_surface_mesh(aabb: &Aabb3) -> SurfaceMesh3D {
    if !aabb.is_valid() {
        return SurfaceMesh3D::from_indexed_triangles(Vec::new(), Vec::new(), Aabb3::EMPTY);
    }
    let mut verts = Vec::with_capacity(8);
    for x in [aabb.min.x, aabb.max.x] {
        for y in [aabb.min.y, aabb.max.y] {
            for z in [aabb.min.z, aabb.max.z] {
                verts.push(DVec3::new(x, y, z));
            }
        }
    }
    SurfaceMesh3D::from_indexed_triangles(verts, Vec::new(), *aabb)
}

fn compute_overlay(
    input: &BoundaryPreviewInput<'_>,
    surface_mesh: Option<&SurfaceMesh3D>,
) -> Option<BoundaryPreviewOverlay> {
    let rs = input.resolved_setup;
    let op = input.op;
    let mut eval_ctx = rs.eval_context(input.display_unit);
    if let Some(tool) = input.active_tool {
        tool.populate_eval_context(&mut eval_ctx);
    }
    if let Some(sum) = input.extracted {
        eval_ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
        eval_ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
    }

    let rh = resolve_preview_heights(
        op,
        rs,
        input.active_tool,
        input.extracted,
        input.display_unit,
    );
    let stock_aabb = rs.stock.aabb_wcs();
    let top_z_wcs = rh.as_ref().map_or(stock_aabb.max.z, |h| h.top_z_wcs);
    let bottom_z_wcs = rh.as_ref().map_or(stock_aabb.min.z, |h| h.bottom_z_wcs);

    // 1. 4th-Axis Rotary (`Rotary2Linear`, `Rotary4AxisSim`)
    if op.kind.is_rotary_native() {
        let rot = rs.rotary?;
        let win = resolve_rotary_window_for_preview(
            op,
            rs,
            input.doc,
            surface_mesh,
            input.extracted,
            input.active_tool,
            rh.as_ref(),
            &eval_ctx,
        )?
        .ok()?;
        return Some(BoundaryPreviewOverlay::RotarySector {
            rotary: rot,
            window: win,
        });
    }

    // 2. 2.5D Facing & 3-Axis / Undercut3D / Deburr3D Machining Boundary
    if let Some(boundary_raw) = op.machining_boundary() {
        let mut boundary = boundary_raw.clone();
        boundary.reevaluate(&eval_ctx);
        let tool_r = 0.5
            * input
                .active_tool
                .map_or(6.0, |t| t.geometry.cutting_diameter_mm());
        let empty_sel = ExtractedSelectionSummary::default();
        let sel_ref = input.extracted.unwrap_or(&empty_sel);

        let show_effective = boundary.tool_containment != ToolContainmentMode::ToolCenterOnBoundary
            || boundary.boundary_offset.value_mm.abs() > 1e-4;

        if op.kind == OperationKind::Facing {
            return Some(resolve_facing_preview(
                rs,
                surface_mesh,
                sel_ref,
                &boundary,
                tool_r,
                show_effective,
                top_z_wcs,
                bottom_z_wcs,
            ));
        }

        let empty_mesh;
        let surf_ref = if let Some(s) = surface_mesh {
            s
        } else {
            empty_mesh = aabb_proxy_surface_mesh(&rs.model_aabb_wcs);
            &empty_mesh
        };

        let resolved =
            resolve_3d_boundary_config_detailed(surf_ref, rs, sel_ref, &boundary, tool_r);
        return Some(BoundaryPreviewOverlay::Planar2D {
            nominal_region: resolved.nominal_region,
            effective_region: resolved.effective_region,
            show_effective,
            top_z_wcs,
            bottom_z_wcs,
            warnings: resolved.warnings,
        });
    }

    // 3. Pocket2D / Adaptive2D `ClearOutsideSelectedToStock`
    if let StrategyParams::Pocket2D(p) = op.resolved_strategy()
        && p.boundary_mode == PocketBoundaryMode::ClearOutsideSelectedToStock
    {
        return Some(BoundaryPreviewOverlay::StockOuterEnvelope {
            stock_region: stock_xy_region(rs),
            top_z_wcs,
            bottom_z_wcs,
        });
    }

    None
}

#[allow(clippy::too_many_arguments)]
fn resolve_facing_preview(
    rs: &ResolvedSetup,
    surface_mesh: Option<&SurfaceMesh3D>,
    selection: &ExtractedSelectionSummary,
    boundary: &chip_cam::MachiningBoundaryConfig,
    tool_r: f64,
    show_effective: bool,
    top_z_wcs: f64,
    bottom_z_wcs: f64,
) -> BoundaryPreviewOverlay {
    let mode = boundary.boundary_mode;
    let containment = boundary.tool_containment;
    let extra_offset_mm = boundary.boundary_offset.value_mm;
    let mut warnings = Vec::new();
    let nominal_region = match mode {
        MachiningBoundaryMode::StockBox => stock_xy_region(rs),
        MachiningBoundaryMode::ModelSilhouette => {
            if let Some(surf) = surface_mesh
                && !surf.triangles.is_empty()
                && surf.aabb_wcs.is_valid()
            {
                let mut cfg = boundary.clone();
                cfg.tool_containment = ToolContainmentMode::ToolCenterOnBoundary;
                cfg.boundary_offset = chip_math::ParamLength::from_mm(0.0);
                let res = resolve_3d_boundary_config_detailed(surf, rs, selection, &cfg, tool_r);
                warnings.extend(res.warnings);
                res.nominal_region
            } else {
                stock_xy_region(rs)
            }
        }
        MachiningBoundaryMode::SelectedRegions => {
            let mut outer_sets = Vec::new();
            let mut island_sets = Vec::new();
            for feat in &selection.features {
                match feat {
                    ExtractedFeature25D::PlanarFace {
                        outer_boundary_wcs,
                        raised_islands_wcs,
                        ..
                    } => {
                        if outer_boundary_wcs.closed && !outer_boundary_wcs.segments.is_empty() {
                            let outer = outer_boundary_wcs.ensure_orientation(true);
                            outer_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                                outer,
                                Vec::new(),
                            )));
                        }
                        for isl in raised_islands_wcs {
                            if isl.closed && !isl.segments.is_empty() {
                                let ccw = isl.ensure_orientation(true);
                                island_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                                    ccw,
                                    Vec::new(),
                                )));
                            }
                        }
                    }
                    ExtractedFeature25D::Contour { polyline_wcs, .. }
                        if polyline_wcs.closed && !polyline_wcs.segments.is_empty() =>
                    {
                        let outer = polyline_wcs.ensure_orientation(true);
                        outer_sets.push(RegionSet2D::from_single(PocketRegion2D::new(
                            outer,
                            Vec::new(),
                        )));
                    }
                    _ => {}
                }
            }
            if outer_sets.is_empty() {
                warnings.push(
                    "Machining boundary is set to 'Selected Regions' but no closed 2D contours or planar faces are selected; falling back to Stock Bounds."
                        .to_string(),
                );
                stock_xy_region(rs)
            } else {
                let outer_u = RegionSet2D::union_many(&mut outer_sets);
                let isl_u = RegionSet2D::union_many(&mut island_sets);
                if isl_u.is_empty() {
                    outer_u
                } else {
                    outer_u.difference(&isl_u)
                }
            }
        }
    };

    let tool_shift = match containment {
        ToolContainmentMode::ToolInsideBoundary => -tool_r,
        ToolContainmentMode::ToolCenterOnBoundary => 0.0,
        ToolContainmentMode::ToolOutsideBoundary => tool_r,
    };
    let delta = tool_shift
        + if extra_offset_mm.is_finite() {
            extra_offset_mm
        } else {
            0.0
        };

    let effective_region = if mode == MachiningBoundaryMode::StockBox {
        match rs.stock {
            ResolvedStockGeometry::Cylinder {
                axis: StockAxis::Z,
                center_wcs,
                radius_mm,
                ..
            } => {
                RegionSet2D::from_circle(DVec2::new(center_wcs.x, center_wcs.y), radius_mm + delta)
            }
            ResolvedStockGeometry::Box { aabb_wcs }
            | ResolvedStockGeometry::Cylinder { aabb_wcs, .. } => rect_region_2d(
                aabb_wcs.min.x - delta,
                aabb_wcs.min.y - delta,
                aabb_wcs.max.x + delta,
                aabb_wcs.max.y + delta,
            ),
        }
    } else if delta > 1e-5 {
        nominal_region.dilate_outward(delta)
    } else if delta < -1e-5 {
        nominal_region.erode_inward(-delta)
    } else {
        nominal_region.clone()
    };

    BoundaryPreviewOverlay::Planar2D {
        nominal_region,
        effective_region,
        show_effective,
        top_z_wcs,
        bottom_z_wcs,
        warnings,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        Operation, OperationKind, RotaryAxialBoundaryMode, RotaryAxisKind, RotaryDirection,
        RotaryDriveMode, RotaryUnwindMode, Setup, StockConfig, WorkOffsetSlot,
    };
    use chip_math::{Aabb3, DVec3, DisplayUnit, ParamLength};
    use chip_tools::default_starter_tool_library;

    use super::*;

    fn sample_box_setup() -> ResolvedSetup {
        let doc = CadDocument::empty();
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(120.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: Default::default(),
            z_align: Default::default(),
        };
        setup
            .resolve_with_sketches(&doc, &[], DisplayUnit::Millimeter)
            .unwrap()
    }

    fn sample_rotary_setup() -> ResolvedSetup {
        let mut rs = sample_box_setup();
        rs.model_aabb_wcs = Aabb3::new(
            DVec3::new(-30.0, -18.0, -18.0),
            DVec3::new(30.0, 18.0, 18.0),
        );
        rs.stock = ResolvedStockGeometry::Cylinder {
            axis: StockAxis::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 21.0,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-40.0, -21.0, -21.0),
                DVec3::new(40.0, 21.0, 21.0),
            ),
        };
        rs.rotary = Some(ResolvedRotaryConfig {
            mode: RotaryDriveMode::Simultaneous4Axis,
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::RightHandCcw,
            center_wcs: DVec3::ZERO,
            stock_outer_radius_mm: 21.0,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            clearance_radius_mm: 33.0,
            chuck_u_min: -60.0,
            chuck_u_max: -40.0,
            chuck_radius_mm: 32.5,
            chuck_end_positive: false,
            axis_letter: 'A',
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        });
        rs
    }

    #[test]
    fn cache_skips_recomputation_on_idle_frames_and_invalidates_on_boundary_or_tool_edit() {
        let rs = sample_box_setup();
        let tools = default_starter_tool_library();
        let tool6 = tools.tools.first().unwrap().clone();
        let mut tool10 = tool6.clone();
        if let chip_tools::CutterGeometry::FlatEndmill {
            ref mut diameter_mm,
            ..
        } = tool10.geometry
        {
            *diameter_mm = 10.0;
        }

        let mut op = Operation::new("op1", "3D Adaptive", OperationKind::Adaptive3D);
        op.machining_boundary_mut().unwrap().tool_containment =
            ToolContainmentMode::ToolCenterOnBoundary;
        let mut cache = BoundaryPreviewCache::default();

        // 1. Inactive tab -> returns None without recomputing
        let inactive = cache.get_or_compute(&BoundaryPreviewInput {
            active: false,
            setup_idx: 0,
            op_idx: 0,
            cad_rev: 1,
            frame_geo_sig: 42,
            display_unit: DisplayUnit::Millimeter,
            doc: None,
            resolved_setup: &rs,
            op: &op,
            active_tool: Some(&tool6),
            extracted: None,
        });
        assert!(inactive.is_none());
        assert_eq!(cache.recompute_count(), 0);

        // 2. First active frame -> computes once
        let first = cache
            .get_or_compute(&BoundaryPreviewInput {
                active: true,
                setup_idx: 0,
                op_idx: 0,
                cad_rev: 1,
                frame_geo_sig: 42,
                display_unit: DisplayUnit::Millimeter,
                doc: None,
                resolved_setup: &rs,
                op: &op,
                active_tool: Some(&tool6),
                extracted: None,
            })
            .expect("Adaptive3D must produce Planar2D overlay");
        assert_eq!(cache.recompute_count(), 1);
        assert!(matches!(*first, BoundaryPreviewOverlay::Planar2D { .. }));

        // 3. Repeated identical frames -> zero recomputation!
        for _ in 0..10 {
            let again = cache.get_or_compute(&BoundaryPreviewInput {
                active: true,
                setup_idx: 0,
                op_idx: 0,
                cad_rev: 1,
                frame_geo_sig: 42,
                display_unit: DisplayUnit::Millimeter,
                doc: None,
                resolved_setup: &rs,
                op: &op,
                active_tool: Some(&tool6),
                extracted: None,
            });
            assert!(again.is_some());
        }
        assert_eq!(
            cache.recompute_count(),
            1,
            "idle frames must never recompute boundary preview"
        );

        // 4. Editing containment mode invalidates cache and sets show_effective = true
        op.machining_boundary_mut().unwrap().tool_containment =
            ToolContainmentMode::ToolOutsideBoundary;
        let updated = cache
            .get_or_compute(&BoundaryPreviewInput {
                active: true,
                setup_idx: 0,
                op_idx: 0,
                cad_rev: 1,
                frame_geo_sig: 42,
                display_unit: DisplayUnit::Millimeter,
                doc: None,
                resolved_setup: &rs,
                op: &op,
                active_tool: Some(&tool6),
                extracted: None,
            })
            .unwrap();
        assert_eq!(cache.recompute_count(), 2);
        match updated.as_ref() {
            BoundaryPreviewOverlay::Planar2D {
                nominal_region,
                effective_region,
                show_effective,
                ..
            } => {
                assert!(*show_effective);
                assert!(
                    effective_region.total_area_mm2() > nominal_region.total_area_mm2() + 10.0,
                    "ToolOutsideBoundary must expand effective region"
                );
            }
            other => panic!("expected Planar2D, got {other:?}"),
        }

        // 5. Editing tool diameter invalidates cache
        let _ = cache.get_or_compute(&BoundaryPreviewInput {
            active: true,
            setup_idx: 0,
            op_idx: 0,
            cad_rev: 1,
            frame_geo_sig: 42,
            display_unit: DisplayUnit::Millimeter,
            doc: None,
            resolved_setup: &rs,
            op: &op,
            active_tool: Some(&tool10),
            extracted: None,
        });
        assert_eq!(cache.recompute_count(), 3);
    }

    #[test]
    fn resolves_rotary_sector_and_pocket_clear_outside_overlays() {
        let rs_rot = sample_rotary_setup();
        let tools = default_starter_tool_library();
        let tool6 = tools.tools.first().unwrap().clone();
        let mut rot_op = Operation::new("op_rot", "4-Axis Finish", OperationKind::Rotary4AxisSim);
        {
            let rb = rot_op.rotary_axial_boundary_mut().unwrap();
            rb.axial_boundary_mode = RotaryAxialBoundaryMode::StockBounds;
            rb.axial_margin_start = ParamLength::from_mm(-5.0); // extends past -40 toward chuck -> clamped!
            rb.start_angle_deg = -90.0;
            rb.end_angle_deg = 90.0;
        }
        let mut cache = BoundaryPreviewCache::default();
        let ov = cache
            .get_or_compute(&BoundaryPreviewInput {
                active: true,
                setup_idx: 0,
                op_idx: 0,
                cad_rev: 1,
                frame_geo_sig: 7,
                display_unit: DisplayUnit::Millimeter,
                doc: None,
                resolved_setup: &rs_rot,
                op: &rot_op,
                active_tool: Some(&tool6),
                extracted: None,
            })
            .expect("Rotary4AxisSim must produce RotarySector overlay");
        match ov.as_ref() {
            BoundaryPreviewOverlay::RotarySector { window, .. } => {
                assert!(window.clamped_by_chuck);
                assert!((window.start_angle_deg - (-90.0)).abs() < 1e-6);
                assert!((window.end_angle_deg - 90.0).abs() < 1e-6);
            }
            other => panic!("expected RotarySector, got {other:?}"),
        }

        // Pocket2D ClearOutsideSelectedToStock -> StockOuterEnvelope
        let rs_box = sample_box_setup();
        let mut pkt = Operation::new("op_pkt", "Pocket Boss", OperationKind::Pocket2D);
        if let StrategyParams::Pocket2D(p) = pkt.strategy_mut() {
            p.boundary_mode = PocketBoundaryMode::ClearOutsideSelectedToStock;
        }
        let ov_pkt = cache
            .get_or_compute(&BoundaryPreviewInput {
                active: true,
                setup_idx: 0,
                op_idx: 1,
                cad_rev: 1,
                frame_geo_sig: 8,
                display_unit: DisplayUnit::Millimeter,
                doc: None,
                resolved_setup: &rs_box,
                op: &pkt,
                active_tool: Some(&tool6),
                extracted: None,
            })
            .expect("Pocket2D ClearOutsideSelectedToStock must produce StockOuterEnvelope");
        assert!(matches!(
            ov_pkt.as_ref(),
            BoundaryPreviewOverlay::StockOuterEnvelope { .. }
        ));
    }
}
