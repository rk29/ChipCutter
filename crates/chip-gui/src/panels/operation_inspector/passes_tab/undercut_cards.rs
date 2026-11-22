use chip_cam::{
    DovetailMode, DovetailParams, SlotUndercutMode, SlotUndercutParams, SlotUndercutPreset,
    Undercut3DParams, UndercutEntryMode,
};
use chip_math::DisplayUnit;
use chip_sim::SimEngineKind;
use chip_tools::CutterGeometry;

use super::PassItemCtx;
use crate::widgets::{BannerKind, inline_banner, section_card};

pub(super) struct SlotUndercutUiCtx<'a> {
    pub p: &'a mut SlotUndercutParams,
    pub assigned_tool_geom: Option<&'a CutterGeometry>,
    pub resolved_depth_mm: Option<f64>,
    pub sim_engine: SimEngineKind,
    pub display_unit: DisplayUnit,
    pub item: PassItemCtx<'a>,
}

pub(super) struct DovetailUiCtx<'a> {
    pub p: &'a mut DovetailParams,
    pub assigned_tool_geom: Option<&'a CutterGeometry>,
    pub resolved_depth_mm: Option<f64>,
    pub sim_engine: SimEngineKind,
    pub display_unit: DisplayUnit,
    pub item: PassItemCtx<'a>,
}

pub(super) struct Undercut3DUiCtx<'a> {
    pub p: &'a mut Undercut3DParams,
    pub assigned_tool_geom: Option<&'a CutterGeometry>,
    pub sim_engine: SimEngineKind,
    pub display_unit: DisplayUnit,
    pub item: PassItemCtx<'a>,
}

/// Advisory banner shown on undercut operations when the 2.5D Heightmap engine is active.
fn maybe_push_tridexel_banner(banners: &mut Vec<(BannerKind, String)>, sim_engine: SimEngineKind) {
    if sim_engine != SimEngineKind::TriDexel {
        banners.push((
            BannerKind::Warn,
            "Simulation Engine is set to 2.5D Heightmap, which cannot represent undercut roofs. Switch Simulation Engine to Tri-Dexel (Volumetric) to verify carved undercut cavities."
                .to_string(),
        ));
    }
}

/// Evaluates immediate live throat, neck, thickness, and reach clearance banners for `SlotUndercut`.
pub fn collect_slot_undercut_banners(
    p: &SlotUndercutParams,
    assigned_tool_geom: Option<&CutterGeometry>,
    resolved_depth_mm: Option<f64>,
    sim_engine: SimEngineKind,
    display_unit: DisplayUnit,
) -> Vec<(BannerKind, String)> {
    let mut banners = Vec::new();
    maybe_push_tridexel_banner(&mut banners, sim_engine);

    let Some(geom) = assigned_tool_geom else {
        banners.push((
            BannerKind::Warn,
            "No slot/keyway cutter assigned. Select a Slot / T-Slot Mill (or Flat Endmill for straight keyways) in the Tool tab."
                .to_string(),
        ));
        return banners;
    };

    let slot_w = p.slot_width.value_mm;
    let throat_w = p.throat_width.value_mm;
    let slot_h = p.slot_height.value_mm;
    let needs_undercut = p.mode == SlotUndercutMode::TSlot || throat_w < slot_w - 1e-4;

    match *geom {
        CutterGeometry::SlotMill {
            diameter_mm,
            cutting_thickness_mm,
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            let mut has_error = false;
            if diameter_mm > slot_w + 1e-4 {
                has_error = true;
                banners.push((
                    BannerKind::Error,
                    format!(
                        "Slot mill diameter ({}) exceeds target slot width ({}).",
                        display_unit.format_length(diameter_mm),
                        display_unit.format_length(slot_w),
                    ),
                ));
            }
            if cutting_thickness_mm > slot_h + 1e-4 {
                has_error = true;
                banners.push((
                    BannerKind::Error,
                    format!(
                        "Slot mill cutting thickness ({}) exceeds slot height ({}).",
                        display_unit.format_length(cutting_thickness_mm),
                        display_unit.format_length(slot_h),
                    ),
                ));
            }
            if p.mode != SlotUndercutMode::SideGroove && needs_undercut {
                let max_lat = (0.5 * slot_w - 0.5 * diameter_mm).max(0.0);
                let neck_reach_r = max_lat + 0.5 * neck_diameter_mm;
                let half_throat = 0.5 * throat_w;
                if neck_diameter_mm >= throat_w - 1e-4 {
                    has_error = true;
                    banners.push((
                        BannerKind::Error,
                        format!(
                            "Relieved neck Ø{} does not clear throat width ({}). Widen throat or pick a slimmer-neck Slot Mill.",
                            display_unit.format_length(neck_diameter_mm),
                            display_unit.format_length(throat_w),
                        ),
                    ));
                } else if neck_reach_r > half_throat + 1e-4 {
                    has_error = true;
                    banners.push((
                        BannerKind::Error,
                        format!(
                            "At full lateral offset ({}), relieved neck reaches {} from centerline (> half-throat {}).",
                            display_unit.format_length(max_lat),
                            display_unit.format_length(neck_reach_r),
                            display_unit.format_length(half_throat),
                        ),
                    ));
                } else {
                    let wall_clr = half_throat - neck_reach_r;
                    let kind = if wall_clr < 0.25 {
                        BannerKind::Warn
                    } else {
                        BannerKind::Info
                    };
                    banners.push((
                        kind,
                        format!(
                            "Throat clearance OK: {} radial neck clearance at full undercut width (neck Ø{} in {} throat).",
                            display_unit.format_length(wall_clr),
                            display_unit.format_length(neck_diameter_mm),
                            display_unit.format_length(throat_w),
                        ),
                    ));
                }
            }
            if let Some(depth) = resolved_depth_mm
                && depth > neck_length_mm + 1e-4
            {
                has_error = true;
                banners.push((
                    BannerKind::Error,
                    format!(
                        "Total slot depth ({}) exceeds relieved neck length ({}); upper shank will crash into roof.",
                        display_unit.format_length(depth),
                        display_unit.format_length(neck_length_mm),
                    ),
                ));
            }
            if !has_error && (p.mode == SlotUndercutMode::SideGroove || !needs_undercut) {
                banners.push((
                    BannerKind::Info,
                    format!(
                        "Slot Mill (Ø{} × {} thick, neck Ø{}, reach {}): safe horizontal open-end entry/exit active.",
                        display_unit.format_length(diameter_mm),
                        display_unit.format_length(cutting_thickness_mm),
                        display_unit.format_length(neck_diameter_mm),
                        display_unit.format_length(neck_length_mm),
                    ),
                ));
            }
        }
        CutterGeometry::FlatEndmill { diameter_mm, .. } => {
            if needs_undercut {
                banners.push((
                    BannerKind::Error,
                    "T-Slot undercut requires a relieved-neck Slot / T-Slot Mill (assigned: Flat Endmill)."
                        .to_string(),
                ));
            } else if diameter_mm > slot_w + 1e-4 {
                banners.push((
                    BannerKind::Error,
                    format!(
                        "Flat Endmill Ø{} exceeds keyway width ({}).",
                        display_unit.format_length(diameter_mm),
                        display_unit.format_length(slot_w),
                    ),
                ));
            } else {
                banners.push((
                    BannerKind::Info,
                    format!(
                        "Flat Endmill Ø{} machining straight keyway ({} wide).",
                        display_unit.format_length(diameter_mm),
                        display_unit.format_length(slot_w),
                    ),
                ));
            }
        }
        _ => {
            banners.push((
                BannerKind::Error,
                format!(
                    "Assigned tool is a {} -- Slot / T-Slot requires a Slot / T-Slot Mill (or Flat Endmill for straight keyways).",
                    geom.kind_label()
                ),
            ));
        }
    }

    banners
}

/// Evaluates immediate live angle-match, throat, cutting-height, and reach banners for `Dovetail`.
pub fn collect_dovetail_banners(
    p: &DovetailParams,
    assigned_tool_geom: Option<&CutterGeometry>,
    resolved_depth_mm: Option<f64>,
    sim_engine: SimEngineKind,
    display_unit: DisplayUnit,
) -> Vec<(BannerKind, String)> {
    let mut banners = Vec::new();
    maybe_push_tridexel_banner(&mut banners, sim_engine);

    let Some(geom) = assigned_tool_geom else {
        banners.push((
            BannerKind::Warn,
            "No dovetail cutter assigned. Select a Dovetail Cutter matching the slot flank angle in the Tool tab."
                .to_string(),
        ));
        return banners;
    };

    let CutterGeometry::DovetailCutter {
        diameter_mm,
        side_angle_deg,
        cutting_height_mm,
        corner_radius_mm,
        neck_diameter_mm,
        neck_length_mm,
        ..
    } = *geom
    else {
        banners.push((
            BannerKind::Error,
            format!(
                "Assigned tool is a {} -- Dovetail requires a Dovetail Cutter.",
                geom.kind_label()
            ),
        ));
        return banners;
    };

    let mut has_error = false;
    if (side_angle_deg - p.side_angle_deg).abs() > 0.5 {
        has_error = true;
        banners.push((
            BannerKind::Error,
            format!(
                "Cutter angle ({side_angle_deg:.1}°) mismatches target dovetail flank angle ({:.1}°, max ±0.5°).",
                p.side_angle_deg
            ),
        ));
    }
    if cutting_height_mm + 1e-4 < p.dovetail_depth.value_mm {
        has_error = true;
        banners.push((
            BannerKind::Error,
            format!(
                "Dovetail cutter cutting height ({}) is shallower than target dovetail depth ({}).",
                display_unit.format_length(cutting_height_mm),
                display_unit.format_length(p.dovetail_depth.value_mm),
            ),
        ));
    }
    if let Some(depth) = resolved_depth_mm
        && depth > neck_length_mm + 1e-4
    {
        has_error = true;
        banners.push((
            BannerKind::Error,
            format!(
                "Total depth ({}) exceeds relieved neck length ({}).",
                display_unit.format_length(depth),
                display_unit.format_length(neck_length_mm),
            ),
        ));
    }

    if p.mode == DovetailMode::InternalSlot {
        let throat_w = p.top_throat_width.value_mm;
        let top_d = CutterGeometry::dovetail_top_diameter_mm(
            diameter_mm,
            side_angle_deg,
            p.dovetail_depth.value_mm.max(0.0),
            corner_radius_mm,
        );
        if neck_diameter_mm >= throat_w - 1e-4 {
            banners.push((
                BannerKind::Error,
                format!(
                    "Dovetail neck Ø{} must be strictly smaller than top throat width ({}).",
                    display_unit.format_length(neck_diameter_mm),
                    display_unit.format_length(throat_w),
                ),
            ));
        } else if top_d > throat_w + 1e-4 {
            banners.push((
                BannerKind::Error,
                format!(
                    "Cutter upper flank Ø{} at depth {} exceeds top throat width ({}).",
                    display_unit.format_length(top_d),
                    display_unit.format_length(p.dovetail_depth.value_mm),
                    display_unit.format_length(throat_w),
                ),
            ));
        } else if !has_error {
            let offset_final = (0.5 * throat_w - 0.5 * top_d).max(0.0);
            let neck_clr = 0.5 * throat_w - (offset_final + 0.5 * neck_diameter_mm);
            let kind = if neck_clr < 0.2 {
                BannerKind::Warn
            } else {
                BannerKind::Info
            };
            banners.push((
                kind,
                format!(
                    "{side_angle_deg:.0}° Dovetail match OK: {} radial neck clearance in {} throat (final lateral offset {}).",
                    display_unit.format_length(neck_clr.max(0.0)),
                    display_unit.format_length(throat_w),
                    display_unit.format_length(offset_final),
                ),
            ));
        }
    } else if !has_error {
        banners.push((
            BannerKind::Info,
            format!(
                "{side_angle_deg:.0}° External Dovetail Rail (Ø{} cutter, {} flank height): horizontal pull-away before +Z retract active.",
                display_unit.format_length(diameter_mm),
                display_unit.format_length(cutting_height_mm),
            ),
        ));
    }

    banners
}

/// Evaluates immediate live tool compatibility and neck-clearance banners for `Undercut3D`.
pub fn collect_undercut3d_banners(
    p: &Undercut3DParams,
    assigned_tool_geom: Option<&CutterGeometry>,
    sim_engine: SimEngineKind,
    display_unit: DisplayUnit,
) -> Vec<(BannerKind, String)> {
    let mut banners = Vec::new();
    maybe_push_tridexel_banner(&mut banners, sim_engine);

    let Some(geom) = assigned_tool_geom else {
        banners.push((
            BannerKind::Warn,
            "No Lollipop Endmill assigned. Select a relieved-neck Lollipop Endmill in the Tool tab."
                .to_string(),
        ));
        return banners;
    };

    let CutterGeometry::LollipopEndmill {
        diameter_mm,
        neck_diameter_mm,
        neck_length_mm,
        ..
    } = *geom
    else {
        banners.push((
            BannerKind::Error,
            format!(
                "Assigned tool is a {} -- 3D Lollipop Undercut requires a Lollipop Endmill.",
                geom.kind_label()
            ),
        ));
        return banners;
    };

    let radial_reach =
        (0.5 * (diameter_mm - neck_diameter_mm) - p.neck_clearance.value_mm).max(0.0);
    if radial_reach <= 0.05 {
        banners.push((
            BannerKind::Warn,
            format!(
                "Effective horizontal overhang reach is nearly zero ({}) with sphere Ø{}, neck Ø{}, and {} clearance. Reduce Neck Clearance or choose a slimmer-neck Lollipop.",
                display_unit.format_length(radial_reach),
                display_unit.format_length(diameter_mm),
                display_unit.format_length(neck_diameter_mm),
                display_unit.format_length(p.neck_clearance.value_mm),
            ),
        ));
    } else {
        banners.push((
            BannerKind::Info,
            format!(
                "Lollipop Ø{} / neck Ø{} (reach {}): max safe horizontal overhang depth {} with {} neck clearance.",
                display_unit.format_length(diameter_mm),
                display_unit.format_length(neck_diameter_mm),
                display_unit.format_length(neck_length_mm),
                display_unit.format_length(radial_reach),
                display_unit.format_length(p.neck_clearance.value_mm),
            ),
        ));
    }

    banners
}

pub(super) fn show_slot_undercut_params(ui: &mut egui::Ui, ctx: SlotUndercutUiCtx<'_>) {
    let SlotUndercutUiCtx {
        p,
        assigned_tool_geom,
        resolved_depth_mm,
        sim_engine,
        display_unit,
        mut item,
    } = ctx;
    let (setup_idx, op_idx) = item.ids();

    for (kind, msg) in collect_slot_undercut_banners(
        p,
        assigned_tool_geom,
        resolved_depth_mm,
        sim_engine,
        display_unit,
    ) {
        inline_banner(ui, kind, &msg);
        ui.add_space(4.0);
    }

    section_card(
        ui,
        ("passes_slot_undercut_card", setup_idx, op_idx),
        "Slot / Keyway / T-Slot Passes",
        true,
        |ui| {
            item.grid(ui, "passes_slot_undercut_grid", |f| {
                let prev_preset = p.preset;
                f.set_tip(
                    "Standard metric DIN 650 / DIN 6885 and imperial ASME T-slot & keyway dimension presets",
                );
                f.combo(
                    "Standard Preset",
                    &mut p.preset,
                    (&SlotUndercutPreset::ALL, SlotUndercutPreset::label),
                    "Apply Slot/T-Slot Standard Preset",
                );
                if p.preset != prev_preset && p.preset != SlotUndercutPreset::Custom {
                    p.apply_preset(p.preset);
                }

                f.set_tip(
                    "Choose between an undercut T-Slot, straight parallel Keyway, or open-sided Side Groove",
                );
                f.combo(
                    "Slot Mode",
                    &mut p.mode,
                    (&SlotUndercutMode::ALL, SlotUndercutMode::label),
                    "Edit Slot Undercut Mode",
                );

                f.set_tip("Total undercut head width measured wall-to-wall");
                f.len("Slot Width", &mut p.slot_width, "Edit Undercut Slot Width");

                if p.mode != SlotUndercutMode::SideGroove {
                    f.set_tip(
                        "Width of the pre-cleared upper vertical throat (must strictly clear the relieved neck at full lateral offset)",
                    );
                    f.len(
                        "Throat Width",
                        &mut p.throat_width,
                        "Edit Pre-Cleared Throat Width",
                    );
                }

                f.set_tip(
                    "Axial height of the undercut slot head (automatically split into overlapping Z passes when taller than cutter thickness)",
                );
                f.len(
                    "Slot Height",
                    &mut p.slot_height,
                    "Edit Undercut Slot Height",
                );

                f.set_tip("Progressive lateral roughing stepover per side from throat centerline outward");
                f.len(
                    "Radial Stepover",
                    &mut p.radial_stepover,
                    "Edit Slot Radial Stepover",
                );

                f.set_tip("Axial overlap stepdown when Slot Height exceeds the Slot Mill cutting thickness");
                f.len(
                    "Axial Stepdown",
                    &mut p.axial_stepdown,
                    "Edit Slot Axial Stepdown",
                );

                f.set_tip("Lateral stock reserved on both undercut walls for the final finishing pass");
                f.len(
                    "Finish Allowance",
                    &mut p.finish_allowance,
                    "Edit Slot Finish Allowance",
                );

                f.set_tip("Append an extra zero-stock spring pass on both walls to eliminate deflection");
                f.check(
                    "Spring Pass",
                    &mut p.spring_pass,
                    "Repeat Final Wall Pass",
                    "Toggle Slot Spring Pass",
                );

                f.set_tip(
                    "Safe entry/exit choreography guaranteeing zero vertical plunges or retracts beneath an undercut roof",
                );
                f.combo(
                    "Entry / Exit Mode",
                    &mut p.entry_mode,
                    (&UndercutEntryMode::ALL, UndercutEntryMode::label),
                    "Edit Slot Entry Mode",
                );

                if p.entry_mode != UndercutEntryMode::EntryBoreCenter {
                    f.set_tip(
                        "Horizontal extension past open slot end(s) so vertical plunge & retract occur strictly in open air",
                    );
                    f.len(
                        "Entry Extension",
                        &mut p.entry_extension,
                        "Edit Slot Entry Extension",
                    );
                }

                f.set_tip("Prefer Climb milling vs. Conventional milling along both left and right walls");
                f.check(
                    "Climb Direction",
                    &mut p.climb_direction,
                    "Climb Milling",
                    "Toggle Slot Cut Direction",
                );
            });
        },
    );
}

pub(super) fn show_dovetail_params(ui: &mut egui::Ui, ctx: DovetailUiCtx<'_>) {
    let DovetailUiCtx {
        p,
        assigned_tool_geom,
        resolved_depth_mm,
        sim_engine,
        display_unit,
        mut item,
    } = ctx;
    let (setup_idx, op_idx) = item.ids();

    for (kind, msg) in collect_dovetail_banners(
        p,
        assigned_tool_geom,
        resolved_depth_mm,
        sim_engine,
        display_unit,
    ) {
        inline_banner(ui, kind, &msg);
        ui.add_space(4.0);
    }

    section_card(
        ui,
        ("passes_dovetail_card", setup_idx, op_idx),
        "Dovetail Slot / Rail Passes",
        true,
        |ui| {
            item.grid(ui, "passes_dovetail_grid", |f| {
                f.set_tip(
                    "Choose Internal Dovetail Slot (female cavity inside a pre-cleared throat) or External Dovetail Rail (male guideway)",
                );
                f.combo(
                    "Dovetail Mode",
                    &mut p.mode,
                    (&DovetailMode::ALL, DovetailMode::label),
                    "Edit Dovetail Mode",
                );

                if p.mode == DovetailMode::InternalSlot {
                    f.set_tip(
                        "Width of the pre-cleared upper vertical throat (must clear the dovetail cutter upper diameter and relieved neck)",
                    );
                    f.len(
                        "Top Throat Width",
                        &mut p.top_throat_width,
                        "Edit Dovetail Top Throat Width",
                    );
                }

                f.set_tip(
                    "Axial height of the angled dovetail flank (must not exceed the cutter's Cutting Height)",
                );
                f.len(
                    "Dovetail Depth",
                    &mut p.dovetail_depth,
                    "Edit Dovetail Depth",
                );

                f.set_tip(
                    "Target dovetail flank angle measured from the horizontal floor (validated within ±0.5° of the assigned Dovetail Cutter)",
                );
                f.drag(
                    "Side Angle",
                    &mut p.side_angle_deg,
                    20.0..=80.0,
                    (0.5, "°"),
                    "Edit Dovetail Side Angle",
                );

                f.set_tip("Progressive lateral roughing stepover into the angled dovetail flank");
                f.len(
                    "Radial Stepover",
                    &mut p.radial_stepover,
                    "Edit Dovetail Radial Stepover",
                );

                f.set_tip("Lateral stock allowance reserved for the final angled flank finishing pass");
                f.len(
                    "Finish Allowance",
                    &mut p.finish_allowance,
                    "Edit Dovetail Finish Allowance",
                );

                f.set_tip("Append an extra zero-stock spring pass along the final angled flank");
                f.check(
                    "Spring Pass",
                    &mut p.spring_pass,
                    "Repeat Final Flank Pass",
                    "Toggle Dovetail Spring Pass",
                );

                if p.mode == DovetailMode::InternalSlot {
                    f.set_tip(
                        "Safe entry/exit choreography guaranteeing zero vertical plunges or retracts through the dovetail roof",
                    );
                    f.combo(
                        "Entry / Exit Mode",
                        &mut p.entry_mode,
                        (&UndercutEntryMode::ALL, UndercutEntryMode::label),
                        "Edit Dovetail Entry Mode",
                    );
                }

                if p.mode == DovetailMode::ExternalRail
                    || p.entry_mode != UndercutEntryMode::EntryBoreCenter
                {
                    f.set_tip(
                        "Horizontal extension past open slot ends (or outward pull-away from external rails) before +Z retract",
                    );
                    f.len(
                        "Entry Extension",
                        &mut p.entry_extension,
                        "Edit Dovetail Entry Extension",
                    );
                }

                f.set_tip("Prefer Climb milling vs. Conventional milling along dovetail flanks");
                f.check(
                    "Climb Direction",
                    &mut p.climb_direction,
                    "Climb Milling",
                    "Toggle Dovetail Cut Direction",
                );
            });
        },
    );
}

pub(super) fn show_undercut3d_params(ui: &mut egui::Ui, ctx: Undercut3DUiCtx<'_>) {
    let Undercut3DUiCtx {
        p,
        assigned_tool_geom,
        sim_engine,
        display_unit,
        mut item,
    } = ctx;
    let (setup_idx, op_idx) = item.ids();

    for (kind, msg) in collect_undercut3d_banners(p, assigned_tool_geom, sim_engine, display_unit) {
        inline_banner(ui, kind, &msg);
        ui.add_space(4.0);
    }

    section_card(
        ui,
        ("passes_undercut3d_card", setup_idx, op_idx),
        "3D Lollipop Undercut Passes",
        true,
        |ui| {
            item.grid(ui, "passes_undercut3d_grid", |f| {
                f.set_tip(
                    "Compute horizontal Z-level spacing automatically from Target Cusp height and Lollipop sphere radius",
                );
                f.check(
                    "Cusp-Based Step",
                    &mut p.use_cusp_height,
                    "Derive from Target Cusp",
                    "Toggle Undercut3D Cusp Stepdown",
                );

                if p.use_cusp_height {
                    f.set_tip("Maximum allowable spherical scallop / cusp height along overhanging walls");
                    f.len(
                        "Target Cusp",
                        &mut p.target_cusp,
                        "Edit Undercut3D Target Cusp",
                    );
                } else {
                    f.set_tip("Fixed vertical Z-slice spacing across overhanging downward-facing surfaces");
                    f.len(
                        "Z Stepdown",
                        &mut p.z_stepdown,
                        "Edit Undercut3D Z Stepdown",
                    );
                }

                f.set_tip(
                    "Minimum downward overhang slope below the horizontal equator (targets surfaces with n_z <= -sin(angle))",
                );
                f.drag(
                    "Min Overhang",
                    &mut p.min_overhang_angle_deg,
                    1.0..=80.0,
                    (0.5, "°"),
                    "Edit Undercut3D Min Overhang Angle",
                );

                f.set_tip(
                    "Minimum radial safety clearance enforced between the relieved neck/shank and overhanging ceilings above (deeper recesses are trimmed automatically)",
                );
                f.len(
                    "Neck Clearance",
                    &mut p.neck_clearance,
                    "Edit Undercut3D Neck Clearance",
                );

                f.set_tip("Uniform surface normal stock allowance left on overhanging 3D walls");
                f.len(
                    "Stock to Leave",
                    &mut p.stock_to_leave,
                    "Edit Undercut3D Stock to Leave",
                );

                f.set_tip(
                    "Minimum horizontal pull-away distance into open air away from overhanging ceilings before any +Z retract",
                );
                f.len(
                    "Lead-Out Pull",
                    &mut p.lead_out_distance,
                    "Edit Undercut3D Lead-Out Distance",
                );

                f.set_tip("Prefer Climb milling vs. Conventional milling along overhanging contour slices");
                f.check(
                    "Climb Direction",
                    &mut p.climb_direction,
                    "Climb Milling",
                    "Toggle Undercut3D Cut Direction",
                );
            });
        },
    );
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::ParamLength;
    use chip_tools::{starter_dovetail_60deg_12mm, starter_lollipop_6mm, starter_tslot_12mm};

    use super::*;

    #[test]
    fn undercut_banners_advise_tridexel_and_validate_clearances() {
        let slot_tool = starter_tslot_12mm();
        let dove_tool = starter_dovetail_60deg_12mm();
        let lolli_tool = starter_lollipop_6mm();

        let mut slot_p = SlotUndercutParams::default();
        slot_p.apply_preset(SlotUndercutPreset::TSlotM8);
        assert!((slot_p.slot_width.value_mm - 14.5).abs() < 1e-6);
        assert!((slot_p.throat_width.value_mm - 8.5).abs() < 1e-6);
        assert!((slot_p.slot_height.value_mm - 6.0).abs() < 1e-6);

        // Heightmap engine emits Tri-Dexel advisory banner; TriDexel engine does not!
        let b_hm = collect_slot_undercut_banners(
            &slot_p,
            Some(&slot_tool.geometry),
            Some(12.0),
            SimEngineKind::Heightmap,
            DisplayUnit::Millimeter,
        );
        assert!(
            b_hm.iter()
                .any(|(k, m)| *k == BannerKind::Warn && m.contains("Tri-Dexel"))
        );

        let b_td = collect_slot_undercut_banners(
            &slot_p,
            Some(&slot_tool.geometry),
            Some(12.0),
            SimEngineKind::TriDexel,
            DisplayUnit::Millimeter,
        );
        assert!(b_td.iter().all(|(k, _)| *k != BannerKind::Error));

        // Dovetail angle mismatch (45° target vs 60° cutter) triggers Error banner
        let bad_dove = DovetailParams {
            side_angle_deg: 45.0,
            ..DovetailParams::default()
        };
        let b_dove = collect_dovetail_banners(
            &bad_dove,
            Some(&dove_tool.geometry),
            Some(10.0),
            SimEngineKind::TriDexel,
            DisplayUnit::Millimeter,
        );
        assert!(
            b_dove
                .iter()
                .any(|(k, m)| *k == BannerKind::Error && m.contains("mismatches"))
        );

        // Lollipop Undercut3D valid tool reports Info banner with reach
        let u3d = Undercut3DParams {
            neck_clearance: ParamLength::from_mm(0.25),
            ..Undercut3DParams::default()
        };
        let b_u3d = collect_undercut3d_banners(
            &u3d,
            Some(&lolli_tool.geometry),
            SimEngineKind::TriDexel,
            DisplayUnit::Millimeter,
        );
        assert!(b_u3d.iter().any(|(k, _)| *k == BannerKind::Info));
    }
}
