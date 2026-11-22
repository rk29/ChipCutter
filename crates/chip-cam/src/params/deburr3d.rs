use chip_math::{EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

fn default_chamfer_width() -> ParamLength {
    ParamLength::from_mm(0.5)
}

fn default_tip_offset() -> ParamLength {
    ParamLength::from_mm(0.75)
}

fn default_wall_clearance() -> ParamLength {
    ParamLength::from_mm(0.15)
}

fn default_edge_extension() -> ParamLength {
    ParamLength::from_mm(0.0)
}

fn default_min_dihedral_angle_deg() -> f64 {
    15.0
}

fn default_true() -> bool {
    true
}

fn default_chord_tolerance_mm() -> f64 {
    0.005
}

fn default_multiple_passes() -> u32 {
    1
}

/// Strategy parameters for **3D Edge Deburring & Non-Planar Chamfering** (`OperationKind::Deburr3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Deburr3DParams {
    /// Target constant edge-break / chamfer width (`mm`) measured along the adjacent faces.
    #[serde(default = "default_chamfer_width")]
    pub chamfer_width: ParamLength,
    /// Axial clearance above the cutter tip (`mm`) keeping a pointed Chamfer Mill cutting on its
    /// stronger conical flank rather than the fragile dead-center tip.
    #[serde(default = "default_tip_offset")]
    pub tip_offset: ParamLength,
    /// Minimum safety clearance (`mm`) preserved from neighboring vertical walls, bosses, and
    /// inside corners when automatically trimming colliding ends of 3D edge chains.
    #[serde(default = "default_wall_clearance")]
    pub wall_clearance: ParamLength,
    /// Optional tangent extension (`mm`) added off the open ends of non-closed 3D edge chains
    /// before neighbor-wall collision trimming.
    #[serde(default = "default_edge_extension")]
    pub edge_extension: ParamLength,
    /// Automatically detect sharp convex 3D edges across the B-Rep solid (`true` overrides manual
    /// selections; auto-detection also activates automatically when no edges are selected).
    #[serde(default)]
    pub auto_detect_edges: bool,
    /// Minimum dihedral normal-change angle (degrees) for auto-detecting sharp edges, ignoring
    /// smooth tangent fillet seams and split-cylinder seams.
    #[serde(default = "default_min_dihedral_angle_deg")]
    pub min_dihedral_angle_deg: f64,
    /// Exclude pure vertical side-wall edges parallel to Setup WCS `±Z` during auto-detection.
    #[serde(default = "default_true")]
    pub exclude_vertical_edges: bool,
    /// Allow machining downward-facing underside overhang rim edges from underneath using a
    /// relieved-neck `LollipopEndmill`.
    #[serde(default)]
    pub allow_underside_back_chamfer: bool,
    /// Prefer climb milling (`true`) vs. conventional milling (`false`) along deburred edges.
    #[serde(default = "default_true")]
    pub climb_direction: bool,
    /// Maximum 3D chord-sagitta subdivision tolerance (`mm`) along curved non-planar edges.
    #[serde(default = "default_chord_tolerance_mm")]
    pub chord_tolerance_mm: f64,
    /// Number of progressive depth/width passes (`1` = single full-width pass; `> 1` steps cleanly
    /// up to `chamfer_width` for larger bevels).
    #[serde(default = "default_multiple_passes")]
    pub multiple_passes: u32,
    /// Append an extra finish spring pass at full `chamfer_width` to eliminate tool deflection.
    #[serde(default)]
    pub spring_pass: bool,
}

impl Default for Deburr3DParams {
    fn default() -> Self {
        Self {
            chamfer_width: default_chamfer_width(),
            tip_offset: default_tip_offset(),
            wall_clearance: default_wall_clearance(),
            edge_extension: default_edge_extension(),
            auto_detect_edges: false,
            min_dihedral_angle_deg: default_min_dihedral_angle_deg(),
            exclude_vertical_edges: true,
            allow_underside_back_chamfer: false,
            climb_direction: true,
            chord_tolerance_mm: default_chord_tolerance_mm(),
            multiple_passes: default_multiple_passes(),
            spring_pass: false,
        }
    }
}

impl Deburr3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.chamfer_width.reevaluate(ctx);
        let _ = self.tip_offset.reevaluate(ctx);
        let _ = self.wall_clearance.reevaluate(ctx);
        let _ = self.edge_extension.reevaluate(ctx);
    }
}
