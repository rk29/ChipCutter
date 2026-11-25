//! Fuzzy-searchable `Ctrl+K` Command Palette & `?` / `F1` Keyboard Shortcut Cheat-Sheet modal
//! (`08-ui-viewport.md` §Discoverability).

use egui::{CornerRadius, Key, Margin, Order, Stroke, StrokeKind, Vec2};

use crate::{
    panels::welcome_gallery::SampleProjectId,
    theme::ThemeTokens,
    widgets::{IconKind, inline_icon, static_card},
};

/// Executable workspace command identifiers dispatched by the `Ctrl+K` Command Palette.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandId {
    // Project & Import
    NewProject,
    OpenProject,
    SaveProject,
    OpenProjectProperties,
    ExportChipzBundle,
    ImportStepModel,
    ImportMesh3D,
    ReloadStepModel,
    ImportVector2D,
    OpenWelcomeScreen,
    OpenSampleGallery,

    // Authoring & Operations
    NewSetup,
    OpenFlipJigWizard,
    OpenSoftJawWizard,
    NewSketchSetupXy,
    NewSketchOn3dFace,
    AddOpAdaptive2D,
    AddOpFacing,
    AddOpContour2D,
    AddOpPocket2D,
    AddOpDrillBore,
    AddOpThreadMill,
    AddOpSlotUndercut,
    AddOpDovetail,
    AddOpChamferTrace,
    AddOpVCarve,
    AddOpAdaptive3D,
    AddOpWaterline3D,
    AddOpParallel3D,
    AddOpScallop3D,
    AddOpPencil3D,
    AddOpDeburr3D,
    AddOpUndercut3D,
    AddOpRotary2Linear,
    AddOpRotary4AxisSim,
    DuplicateOperation,
    MoveOperationUp,
    MoveOperationDown,
    DeleteSelected,
    Undo,
    Redo,

    // Execute, Simulation, & Post
    GenerateActiveSetup,
    GenerateAllSetups,
    RunSimulation,
    OpenGCodePreview,
    ExportGCode,
    OpenToolLibrary,
    OpenMachineManager,
    OpenDiagnosticsLog,

    // View, Camera, Theme, & Units
    ToggleMeasureTool,
    ToggleReachHeatmap,
    FitToViewport,
    ToggleOrthoPerspective,
    AlignViewSetupTop,
    ResetWorkspaceLayout,
    ToggleThemeDarkLight,
    SwitchUnitsMillimeter,
    SwitchUnitsInch,
    OpenShortcutCheatsheet,
    OpenSettings,
    OpenAboutModal,

    // Quick-Load Samples
    SampleCamTorture,
    SampleAdaptiveRest,
    SampleVCarveInlay,
    SampleEngravedNameplate,
    SamplePure2dPanel,
    SampleSculptedMeshDome,
    SampleReliefWaveMesh,
    SampleRotaryIndexedShaft,
    SampleRotaryWrappedEngraving,
    SampleRotaryHelicalFlute,
    SampleThreadMillManifold,
    SampleTwoSidedVise,
    SampleRevisionReloadV2,
}

pub struct PaletteEntry {
    pub id: CommandId,
    pub category: &'static str,
    pub label: &'static str,
    pub keywords: &'static str,
    pub shortcut: Option<&'static str>,
}

pub const PALETTE_COMMANDS: &[PaletteEntry] = &[
    PaletteEntry {
        id: CommandId::GenerateActiveSetup,
        category: "CAM",
        label: "Generate Dirty Toolpaths in Active Setup",
        keywords: "generate build compute toolpath cam",
        shortcut: Some("Ctrl+G"),
    },
    PaletteEntry {
        id: CommandId::GenerateAllSetups,
        category: "CAM",
        label: "Generate Toolpaths Across All Setups",
        keywords: "generate all setups toolpath",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::RunSimulation,
        category: "Simulate",
        label: "Run 3D Stock Simulation & 5-Rule Crash Verification",
        keywords: "simulate verify gouge crash dexel heightmap",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenGCodePreview,
        category: "Post",
        label: "Open G-Code Preview Pane",
        keywords: "post gcode nc ngc grbl linuxcnc preview",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ExportGCode,
        category: "Post",
        label: "Export Safety-Verified G-Code (.nc / .ngc)…",
        keywords: "export save post gcode nc ngc grbl linuxcnc",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenProject,
        category: "File",
        label: "Open Project or Portable Bundle (.chip.toml / .chipz)…",
        keywords: "open load file project bundle chipz toml",
        shortcut: Some("Ctrl+O"),
    },
    PaletteEntry {
        id: CommandId::SaveProject,
        category: "File",
        label: "Save Project (.chip.toml)",
        keywords: "save write project toml",
        shortcut: Some("Ctrl+S"),
    },
    PaletteEntry {
        id: CommandId::NewProject,
        category: "File",
        label: "New Blank Project",
        keywords: "new clear blank untitled project",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenProjectProperties,
        category: "Project",
        label: "Project Properties...",
        keywords: "project properties rename name units machine summary metadata",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ExportChipzBundle,
        category: "File",
        label: "Export Self-Contained Portable Bundle (.chipz)…",
        keywords: "bundle pack archive zip chipz portable",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ImportStepModel,
        category: "Import",
        label: "Import 3D STEP Solid (.step / .stp)…",
        keywords: "import step stp cad brep 3d solid model",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ImportMesh3D,
        category: "Import",
        label: "Import 3D Mesh (.stl / .obj / .3mf)…",
        keywords: "import mesh stl obj 3mf 3d surface relief sculpted",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ReloadStepModel,
        category: "Import",
        label: "Reload Active STEP Model & Associatively Re-Match",
        keywords: "reload refresh rematch step cad associative",
        shortcut: Some("Ctrl+R"),
    },
    PaletteEntry {
        id: CommandId::ImportVector2D,
        category: "Import",
        label: "Import 2D DXF / SVG Vector Drawing…",
        keywords: "import dxf svg vector 2d laser router artwork",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::NewSketchSetupXy,
        category: "Sketch",
        label: "New 2D Parametric Sketch on Active Setup XY",
        keywords: "sketch 2d draw rectangle circle text slot",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::NewSketchOn3dFace,
        category: "Sketch",
        label: "New Associative 2D Sketch on Planar 3D STEP Face…",
        keywords: "sketch 3d face planar associative",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::NewSetup,
        category: "Setup",
        label: "Add New Clamping Setup (G54–G59.3)",
        keywords: "setup wcs g54 g55 flip op2 stock",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenFlipJigWizard,
        category: "Workholding",
        label: "Two-Sided Flip-Jig Dowel-Pin Wizard…",
        keywords: "workholding flip jig dowel pins op2 symmetric g55",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenSoftJawWizard,
        category: "Workholding",
        label: "Auto Soft-Jaw Generator Wizard (CNC & 3D-Print STL)…",
        keywords: "workholding soft jaws vise pocket stl dogbone kurt milo carvera",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpAdaptive2D,
        category: "Operation",
        label: "Add 2D Adaptive / Trochoidal Clearing Operation",
        keywords: "add operation adaptive trochoidal hsm roughing",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpContour2D,
        category: "Operation",
        label: "Add 2D Contour / Profile Operation",
        keywords: "add operation contour profile tabs cutout",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpPocket2D,
        category: "Operation",
        label: "Add 2D Pocket Clearing Operation",
        keywords: "add operation pocket cavity island offset",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpFacing,
        category: "Operation",
        label: "Add Facing / Stock Surfacing Operation",
        keywords: "add operation face facing deck surface",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpDrillBore,
        category: "Operation",
        label: "Add Drilling / Helical Bore Milling Operation",
        keywords: "add operation drill peck g81 g83 helical bore hole",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpThreadMill,
        category: "Operation",
        label: "Add Helical Thread Milling Operation",
        keywords: "add operation thread mill helix helical tap m6 m8 m10 unc unf iso metric internal external",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpSlotUndercut,
        category: "Operation",
        label: "Add Slot / Keyway / T-Slot Undercut Operation",
        keywords: "add operation slot t-slot tslot keyway woodruff din650 din6885 undercut side groove",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpDovetail,
        category: "Operation",
        label: "Add Dovetail Slot / Rail Operation",
        keywords: "add operation dovetail slot rail guideway angled 60 45 undercut internal external",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpChamferTrace,
        category: "Operation",
        label: "Add Edge Chamfer / 2D Curve Trace Operation",
        keywords: "add operation chamfer bevel deburr trace engrave",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpVCarve,
        category: "Operation",
        label: "Add 3D Medial-Axis V-Carve / V-Inlay Operation",
        keywords: "add operation vcarve v-carve vbit inlay bevel sign",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpAdaptive3D,
        category: "Operation",
        label: "Add 3D Adaptive Roughing Operation",
        keywords: "add operation 3d adaptive roughing mesh surface trochoidal",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpWaterline3D,
        category: "Operation",
        label: "Add 3D Waterline / Z-Level Operation",
        keywords: "add operation 3d waterline z-level contour mesh surface steep",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpParallel3D,
        category: "Operation",
        label: "Add 3D Parallel / Raster Finishing Operation",
        keywords: "add operation 3d parallel raster scanline finishing mesh surface cusp",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpScallop3D,
        category: "Operation",
        label: "Add 3D Constant-Scallop Finishing Operation",
        keywords: "add operation 3d scallop constant cusp geodesic finishing mesh surface",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpPencil3D,
        category: "Operation",
        label: "Add 3D Pencil / Fillet Cleanup Operation",
        keywords: "add operation 3d pencil valley crease fillet cleanup corner finishing mesh",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpDeburr3D,
        category: "Operation",
        label: "Add 3D Edge Deburr / Chamfer Operation",
        keywords: "add operation 3d deburr chamfer bevel edge break lollipop back chamfer non-planar",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpUndercut3D,
        category: "Operation",
        label: "Add 3D Lollipop Undercut Finishing Operation",
        keywords: "add operation 3d undercut lollipop overhang concave groove neck clearance spherical",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpRotary2Linear,
        category: "Operation",
        label: "Add 4th-Axis Rotary Surfacing Operation",
        keywords: "add operation 4th axis rotary spiral axial parallel surfacing a-axis b-axis g93",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::AddOpRotary4AxisSim,
        category: "Operation",
        label: "Add Simultaneous 4-Axis (X+Y+Z+A) Operation",
        keywords: "add operation simultaneous 4-axis rotary helical flute off-centerline auto tilt g93",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::DuplicateOperation,
        category: "Edit",
        label: "Duplicate Focused Operation",
        keywords: "duplicate copy clone operation",
        shortcut: Some("Ctrl+D"),
    },
    PaletteEntry {
        id: CommandId::MoveOperationUp,
        category: "Edit",
        label: "Move Focused Operation Up",
        keywords: "reorder move up operation",
        shortcut: Some("Ctrl+Up"),
    },
    PaletteEntry {
        id: CommandId::MoveOperationDown,
        category: "Edit",
        label: "Move Focused Operation Down",
        keywords: "reorder move down operation",
        shortcut: Some("Ctrl+Down"),
    },
    PaletteEntry {
        id: CommandId::DeleteSelected,
        category: "Edit",
        label: "Delete Selected Operation or Sketch Entity",
        keywords: "delete remove operation entity",
        shortcut: Some("Del"),
    },
    PaletteEntry {
        id: CommandId::Undo,
        category: "Edit",
        label: "Undo Last Edit",
        keywords: "undo history revert",
        shortcut: Some("Ctrl+Z"),
    },
    PaletteEntry {
        id: CommandId::Redo,
        category: "Edit",
        label: "Redo Edit",
        keywords: "redo history forward",
        shortcut: Some("Ctrl+Y"),
    },
    PaletteEntry {
        id: CommandId::OpenToolLibrary,
        category: "Tools",
        label: "Manage Tools / Open Tool Library...",
        keywords: "manage tools cutter endmill vbit drill fusion library",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenMachineManager,
        category: "Tools",
        label: "Open Machine Manager (Milo, Carvera, Voron Cascade, Custom)…",
        keywords: "machine milo 1.5 2.0 carvera air z1 voron cascade generic definition envelope spindle clone export import",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ToggleMeasureTool,
        category: "View",
        label: "Toggle Click-to-Measure Tool",
        keywords: "measure distance diameter radius angle depth inspect ruler wcs",
        shortcut: Some("M"),
    },
    PaletteEntry {
        id: CommandId::ToggleReachHeatmap,
        category: "View",
        label: "Toggle Minimum-Radius & Flute/Stickout Reach Heatmap",
        keywords: "reach min radius heatmap fillet flute stickout corner machinability check",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::FitToViewport,
        category: "View",
        label: "Fit Scene to 3D Viewport",
        keywords: "fit zoom all camera center",
        shortcut: Some("F"),
    },
    PaletteEntry {
        id: CommandId::ToggleOrthoPerspective,
        category: "View",
        label: "Toggle Orthographic / Perspective Camera",
        keywords: "ortho perspective projection camera",
        shortcut: Some("P"),
    },
    PaletteEntry {
        id: CommandId::AlignViewSetupTop,
        category: "View",
        label: "Align Camera to Active Setup +Z Top",
        keywords: "top view align setup wcs z",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ToggleThemeDarkLight,
        category: "View",
        label: "Toggle Studio Dark / Daylight Light Theme",
        keywords: "theme dark light daylight color appearance",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SwitchUnitsMillimeter,
        category: "Units",
        label: "Set Display Units to Millimeters (mm)",
        keywords: "units metric millimeter mm",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SwitchUnitsInch,
        category: "Units",
        label: "Set Display Units to Inches (in)",
        keywords: "units imperial inch inches thou",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::ResetWorkspaceLayout,
        category: "View",
        label: "Reset Default Workspace Pane Layout",
        keywords: "reset layout panes tiles dock",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenDiagnosticsLog,
        category: "View",
        label: "Open Diagnostics & Event Log Tab",
        keywords: "diagnostics log errors warnings",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenSampleGallery,
        category: "Samples",
        label: "Browse Sample Projects Gallery…",
        keywords: "samples demos examples gallery",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenWelcomeScreen,
        category: "Help",
        label: "Open Welcome & Quick-Start Screen",
        keywords: "welcome start recent help",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::OpenSettings,
        category: "Settings",
        label: "Open Application Settings",
        keywords: "application settings preferences theme units simulation engine resolution mesh skinning mouse navigation",
        shortcut: Some("Ctrl+,"),
    },
    PaletteEntry {
        id: CommandId::OpenShortcutCheatsheet,
        category: "Help",
        label: "Keyboard Shortcut Cheat-Sheet",
        keywords: "shortcuts keys hotkeys help reference",
        shortcut: Some("F1 / ?"),
    },
    PaletteEntry {
        id: CommandId::OpenAboutModal,
        category: "Help",
        label: "About ChipCutter (Version & Build Info)…",
        keywords: "about version release build info license credits",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleCamTorture,
        category: "Samples",
        label: "Sample: Load CAM Torture Block (3D STEP)",
        keywords: "sample demo torture block step",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleAdaptiveRest,
        category: "Samples",
        label: "Sample: Load 2D Adaptive + Rest Machining",
        keywords: "sample demo adaptive rest trochoidal",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleVCarveInlay,
        category: "Samples",
        label: "Sample: Load V-Carve Sign & Mated V-Inlay",
        keywords: "sample demo vcarve inlay sign",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleEngravedNameplate,
        category: "Samples",
        label: "Sample: Load Engraved Nameplate (TrueType + Hershey)",
        keywords: "sample demo nameplate text font",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SamplePure2dPanel,
        category: "Samples",
        label: "Sample: Load Pure-2D CNC Router Panel (.dxf)",
        keywords: "sample demo pure 2d dxf router panel",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleSculptedMeshDome,
        category: "Samples",
        label: "Sample: Load Sculpted Dome (STL Mesh)",
        keywords: "sample demo stl mesh dome 3d surface sculpted",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleReliefWaveMesh,
        category: "Samples",
        label: "Sample: Load Relief Wave (OBJ Mesh)",
        keywords: "sample demo obj mesh relief wave 3d parallel scallop finishing",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleRotaryIndexedShaft,
        category: "Samples",
        label: "Sample: Load Rotary Indexed Shaft (4th-Axis A)",
        keywords: "sample demo 4th axis rotary shaft chuck a-axis g93 indexed 3+1 spiral",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleRotaryWrappedEngraving,
        category: "Samples",
        label: "Sample: Load Wrapped Rotary Engraving (Z+X+A)",
        keywords: "sample demo 4th axis rotary wrap cylinder engraving vcarve pocket grbl a-axis",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleRotaryHelicalFlute,
        category: "Samples",
        label: "Sample: Load Simultaneous 4-Axis Helical Flute (X+Y+Z+A)",
        keywords: "sample demo simultaneous 4-axis rotary helical flute auger tilt xyza g93",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleThreadMillManifold,
        category: "Samples",
        label: "Sample: Load Threaded Manifold Block (Helical Thread Milling)",
        keywords: "sample demo thread mill manifold m8 unc m12 internal external helix tap",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleTwoSidedVise,
        category: "Samples",
        label: "Sample: Load Two-Sided Vise & Soft-Jaw Bracket (G54 -> G55)",
        keywords: "sample demo workholding vise clamp soft jaws flip carryover two sided",
        shortcut: None,
    },
    PaletteEntry {
        id: CommandId::SampleRevisionReloadV2,
        category: "Samples",
        label: "Sample: Associatively Reload CAD Revision v2",
        keywords: "sample demo revision v2 reload associative",
        shortcut: None,
    },
];

impl CommandId {
    pub const fn as_sample(self) -> Option<SampleProjectId> {
        match self {
            Self::SampleCamTorture => Some(SampleProjectId::CamTortureBlock),
            Self::SampleAdaptiveRest => Some(SampleProjectId::AdaptiveRest),
            Self::SampleVCarveInlay => Some(SampleProjectId::VCarveSignInlay),
            Self::SampleEngravedNameplate => Some(SampleProjectId::EngravedNameplate),
            Self::SamplePure2dPanel => Some(SampleProjectId::Pure2dPanel),
            Self::SampleSculptedMeshDome => Some(SampleProjectId::SculptedMeshDome),
            Self::SampleReliefWaveMesh => Some(SampleProjectId::ReliefWaveMesh),
            Self::SampleRotaryIndexedShaft => Some(SampleProjectId::RotaryIndexedShaft),
            Self::SampleRotaryWrappedEngraving => Some(SampleProjectId::RotaryWrappedEngraving),
            Self::SampleRotaryHelicalFlute => Some(SampleProjectId::RotaryHelicalFlute),
            Self::SampleThreadMillManifold => Some(SampleProjectId::ThreadMillManifold),
            Self::SampleTwoSidedVise => Some(SampleProjectId::TwoSidedViseWorkholding),
            Self::SampleRevisionReloadV2 => Some(SampleProjectId::CadRevisionV2Reload),
            _ => None,
        }
    }
}

/// Computes a fuzzy subsequence + word-boundary score for `query` against `candidate`.
/// Returns `None` if any non-whitespace token in `query` fails to match as an ordered subsequence.
pub fn fuzzy_match_score(query: &str, candidate: &str) -> Option<i32> {
    let q_trim = query.trim();
    if q_trim.is_empty() {
        return Some(0);
    }
    let cand_bytes = candidate.to_ascii_lowercase().into_bytes();
    let mut total_score = 0_i32;

    for token in q_trim.split_whitespace() {
        let tok_bytes = token.to_ascii_lowercase().into_bytes();
        let token_score = score_single_token(&tok_bytes, &cand_bytes)?;
        total_score += token_score;
    }
    Some(total_score)
}

fn score_single_token(q: &[u8], cand: &[u8]) -> Option<i32> {
    if q.is_empty() {
        return Some(0);
    }
    // High bonus for contiguous substring match
    if let Some(pos) = cand.windows(q.len()).position(|w| w == q) {
        let boundary_bonus = if pos == 0 || is_word_sep(cand[pos - 1]) {
            60
        } else {
            25
        };
        return Some(100 + boundary_bonus + (q.len() as i32 * 8) - (pos as i32));
    }

    // Ordered subsequence fallback with word-boundary & adjacency bonuses
    let mut qi = 0;
    let mut prev_match: Option<usize> = None;
    let mut score = 0_i32;
    for (ci, &cb) in cand.iter().enumerate() {
        if cb == q[qi] {
            score += 10;
            if ci == 0 || is_word_sep(cand[ci - 1]) {
                score += 18;
            }
            if prev_match == Some(ci.saturating_sub(1)) {
                score += 14;
            }
            prev_match = Some(ci);
            qi += 1;
            if qi == q.len() {
                return Some(score - (ci as i32 / 3));
            }
        }
    }
    None
}

#[inline]
const fn is_word_sep(b: u8) -> bool {
    matches!(b, b' ' | b'-' | b'_' | b'/' | b'+' | b':' | b'(' | b'.')
}

#[derive(Debug, Default)]
pub struct CommandPaletteState {
    pub open: bool,
    pub query: String,
    pub selected_idx: usize,
    pub focus_input_next_frame: bool,
}

impl CommandPaletteState {
    pub fn open_fresh(&mut self) {
        self.open = true;
        self.query.clear();
        self.selected_idx = 0;
        self.focus_input_next_frame = true;
    }
}

#[derive(Debug, Default)]
pub struct ShortcutCheatsheetState {
    pub open: bool,
}

/// Renders the `Ctrl+K` fuzzy Command Palette overlay and returns the chosen [`CommandId`].
pub fn show_command_palette(
    ctx: &egui::Context,
    theme: crate::theme::AppTheme,
    state: &mut CommandPaletteState,
) -> Option<CommandId> {
    if !state.open {
        return None;
    }

    let tok = ThemeTokens::for_theme(theme);
    let mut chosen: Option<CommandId> = None;
    let mut should_close = ctx.input(|i| i.key_pressed(Key::Escape));

    let mut matches: Vec<(i32, usize, &'static PaletteEntry)> = PALETTE_COMMANDS
        .iter()
        .enumerate()
        .filter_map(|(ord, entry)| {
            let searchable = format!(
                "{} {} {} {}",
                entry.category,
                entry.label,
                entry.keywords,
                entry.shortcut.unwrap_or("")
            );
            fuzzy_match_score(&state.query, &searchable).map(|sc| (sc, ord, entry))
        })
        .collect();

    if !state.query.trim().is_empty() {
        matches.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
    }
    if matches.is_empty() {
        state.selected_idx = 0;
    } else if state.selected_idx >= matches.len() {
        state.selected_idx = matches.len() - 1;
    }

    let (nav_up, nav_down, accept_enter) = ctx.input(|i| {
        (
            i.key_pressed(Key::ArrowUp),
            i.key_pressed(Key::ArrowDown),
            i.key_pressed(Key::Enter),
        )
    });
    if !matches.is_empty() {
        if nav_up {
            state.selected_idx = state.selected_idx.saturating_sub(1);
        }
        if nav_down {
            state.selected_idx = (state.selected_idx + 1).min(matches.len() - 1);
        }
        if accept_enter {
            chosen = Some(matches[state.selected_idx].2.id);
            should_close = true;
        }
    }

    let screen = ctx.content_rect();
    let palette_w = (screen.width() - 48.0).clamp(360.0, 580.0);
    let anchor = egui::pos2(screen.center().x - 0.5 * palette_w, screen.top() + 72.0);

    let area_resp = egui::Area::new(egui::Id::unique("chipcutter_command_palette"))
        .order(Order::Foreground)
        .fixed_pos(anchor)
        .show(ctx, |ui| {
            egui::Frame::NONE
                .fill(tok.elevated_bg)
                .stroke(Stroke::new(1.2, tok.accent))
                .corner_radius(CornerRadius::same(8))
                .inner_margin(Margin::symmetric(10, 8))
                .show(ui, |ui| {
                    ui.set_min_width(palette_w);
                    ui.set_max_width(palette_w);

                    ui.horizontal(|ui| {
                        inline_icon(ui, IconKind::Search, tok.accent, 15.0);
                        let prev_q = state.query.clone();
                        let edit_resp = ui
                            .add(
                                egui::TextEdit::singleline(&mut state.query)
                                    .hint_text(
                                        "Type a command, operation, view, or sample (Esc to close)…",
                                    )
                                    .desired_width(palette_w - 42.0),
                            )
                            .on_hover_text("Type keywords to fuzzy-search commands, operations, and sample projects");
                        if state.focus_input_next_frame {
                            edit_resp.request_focus();
                            state.focus_input_next_frame = false;
                        }
                        if state.query != prev_q {
                            state.selected_idx = 0;
                        }
                    });
                    ui.separator();

                    if matches.is_empty() {
                        ui.weak("No matching commands.");
                    } else {
                        egui::ScrollArea::vertical()
                            .id_salt("cmd_palette_results")
                            .max_height(320.0)
                            .show(ui, |ui| {
                                for (row_i, (_, _, entry)) in matches.iter().enumerate() {
                                    let is_sel = row_i == state.selected_idx;
                                    let row_w = ui.available_width().max(200.0);
                                    let (rect, resp) = ui.allocate_exact_size(
                                        Vec2::new(row_w, 25.0),
                                        egui::Sense::click(),
                                    );
                                    if resp.hovered() {
                                        state.selected_idx = row_i;
                                    }
                                    if resp.clicked() {
                                        chosen = Some(entry.id);
                                        should_close = true;
                                    }
                                    if ui.is_rect_visible(rect) {
                                        if is_sel {
                                            ui.painter().rect(
                                                rect,
                                                CornerRadius::same(4),
                                                tok.accent.gamma_multiply(0.22),
                                                Stroke::new(1.0, tok.accent),
                                                StrokeKind::Inside,
                                            );
                                        }
                                        let cat_txt = format!("[{}]", entry.category);
                                        ui.painter().text(
                                            egui::pos2(rect.left() + 6.0, rect.center().y),
                                            egui::Align2::LEFT_CENTER,
                                            &cat_txt,
                                            egui::FontId::monospace(10.5),
                                            tok.info_fg,
                                        );
                                        ui.painter().text(
                                            egui::pos2(rect.left() + 82.0, rect.center().y),
                                            egui::Align2::LEFT_CENTER,
                                            entry.label,
                                            egui::FontId::proportional(12.5),
                                            tok.text_primary,
                                        );
                                        if let Some(sc) = entry.shortcut {
                                            ui.painter().text(
                                                egui::pos2(rect.right() - 8.0, rect.center().y),
                                                egui::Align2::RIGHT_CENTER,
                                                sc,
                                                egui::FontId::monospace(11.0),
                                                tok.text_muted,
                                            );
                                        }
                                    }
                                }
                            });
                    }
                });
        });

    let clicked_outside = ctx.input(|i| i.pointer.any_pressed()) && !area_resp.response.hovered();
    if should_close || clicked_outside {
        state.open = false;
    }

    chosen
}

/// Renders the `?` / `F1` **Keyboard Shortcut Cheat-Sheet** modal.
pub fn show_shortcut_cheatsheet_modal(ctx: &egui::Context, state: &mut ShortcutCheatsheetState) {
    if !state.open {
        return;
    }

    let mut keep_open = state.open;
    egui::Window::new("Keyboard Shortcuts & Mouse Reference")
        .id(egui::Id::unique("chipcutter_shortcut_cheatsheet"))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .default_width(640.0)
        .open(&mut keep_open)
        .show(ctx, |ui| {
            ui.set_min_width(620.0);
            ui.columns(2, |cols| {
                static_card(&mut cols[0], Some("Project & Workspace"), |ui| {
                    shortcut_table(
                        ui,
                        "sc_proj",
                        &[
                            ("Ctrl+K", "Open Command Palette"),
                            ("F1 / ?", "Keyboard Shortcut Cheat-Sheet"),
                            ("Ctrl+O", "Open .chip.toml / .chipz"),
                            ("Ctrl+S", "Save .chip.toml Project"),
                            ("Ctrl+R", "Reload Active STEP & Re-Match"),
                            ("Ctrl+Z / Ctrl+Y", "Undo / Redo (100 steps)"),
                            ("Esc", "Cancel Active Pick / Generation"),
                        ],
                    );
                });
                static_card(&mut cols[0], Some("CAM Browser & Operations"), |ui| {
                    shortcut_table(
                        ui,
                        "sc_ops",
                        &[
                            ("Ctrl+G", "Generate Dirty Toolpaths"),
                            ("Ctrl+D", "Duplicate Focused Operation"),
                            ("Ctrl+Up / Down", "Reorder Focused Operation"),
                            ("Drag Row", "Drag-and-Drop Operation Order"),
                            ("Del", "Delete Focused Op / Sketch Entity"),
                        ],
                    );
                });
                static_card(&mut cols[1], Some("3D Viewport & Simulation"), |ui| {
                    shortcut_table(
                        ui,
                        "sc_vp",
                        &[
                            ("M", "Toggle Click-to-Measure Tool"),
                            ("P", "Toggle Ortho / Perspective"),
                            ("F", "Fit Scene to Viewport"),
                            ("Space", "Play / Pause 3D Simulation"),
                            ("Scroll", "Zoom to Cursor"),
                            ("MMB / Shift+MMB", "Pan / Orbit (Fusion 360 preset)"),
                        ],
                    );
                });
                static_card(&mut cols[1], Some("Parametric Expression Inputs"), |ui| {
                    shortcut_table(
                        ui,
                        "sc_expr",
                        &[
                            ("Tab", "Autocomplete Scoped Variable"),
                            ("Up / Down", "Nudge Plain Number (±0.1 / 0.005)"),
                            ("Shift + Up/Down", "Coarse Nudge (10x step)"),
                            ("Alt + Up/Down", "Fine Micrometer Nudge (0.1x)"),
                            ("Drag [mm]/[in]", "Horizontal Scrub Numeric Value"),
                        ],
                    );
                });
            });
        });

    if ctx.input(|i| i.key_pressed(Key::Escape)) {
        keep_open = false;
    }
    state.open = keep_open;
}

fn shortcut_table(ui: &mut egui::Ui, id_salt: &'static str, rows: &[(&str, &str)]) {
    let tok = ThemeTokens::from_ui(ui);
    egui::Grid::new(id_salt)
        .num_columns(2)
        .min_col_width(108.0)
        .spacing([10.0, 5.0])
        .show(ui, |ui| {
            for &(key, desc) in rows {
                ui.colored_label(tok.info_fg, egui::RichText::new(key).monospace().size(11.5));
                ui.label(desc);
                ui.end_row();
            }
        });
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn fuzzy_matcher_ranks_exact_word_boundaries_above_subsequences_and_rejects_misses() {
        let s_exact =
            fuzzy_match_score("vcarve", "Add 3D Medial-Axis V-Carve Operation vcarve").unwrap();
        let s_subseq =
            fuzzy_match_score("vcarve", "View Camera Align Vertical Center").unwrap_or(-999);
        assert!(s_exact > s_subseq);

        assert!(fuzzy_match_score("ctrl+g", "Generate Dirty Toolpaths Ctrl+G").is_some());
        assert!(
            fuzzy_match_score("adap rest", "Sample: Load 2D Adaptive + Rest Machining").is_some()
        );
        assert!(fuzzy_match_score("zzzz_no_such_cmd", "Generate Dirty Toolpaths").is_none());
    }

    #[test]
    fn command_palette_matches_about_and_version_queries() {
        let entry = PALETTE_COMMANDS
            .iter()
            .find(|e| e.id == CommandId::OpenAboutModal)
            .expect("OpenAboutModal entry present");
        let searchable = format!("{} {} {}", entry.category, entry.label, entry.keywords);
        assert!(fuzzy_match_score("about", &searchable).is_some());
        assert!(fuzzy_match_score("version", &searchable).is_some());
    }

    #[test]
    fn command_palette_contains_and_matches_thread_mill_keywords() {
        let entry = PALETTE_COMMANDS
            .iter()
            .find(|e| e.id == CommandId::AddOpThreadMill)
            .expect("AddOpThreadMill entry present");
        let searchable = format!("{} {} {}", entry.category, entry.label, entry.keywords);
        for query in ["thread", "helix", "tap", "M6", "M8", "UNC"] {
            assert!(
                fuzzy_match_score(query, &searchable).is_some(),
                "expected CommandPalette ThreadMill entry to match query '{query}'"
            );
        }
    }

    #[test]
    fn command_palette_contains_and_matches_deburr3d_keywords() {
        let entry = PALETTE_COMMANDS
            .iter()
            .find(|e| e.id == CommandId::AddOpDeburr3D)
            .expect("AddOpDeburr3D entry present");
        let searchable = format!("{} {} {}", entry.category, entry.label, entry.keywords);
        for query in ["deburr", "3d deburr", "chamfer", "lollipop", "back chamfer"] {
            assert!(
                fuzzy_match_score(query, &searchable).is_some(),
                "expected CommandPalette Deburr3D entry to match query '{query}'"
            );
        }
    }

    #[test]
    fn command_palette_contains_and_matches_undercut_operations() {
        for (id, queries) in [
            (
                CommandId::AddOpSlotUndercut,
                &["t-slot", "keyway", "slot undercut"][..],
            ),
            (
                CommandId::AddOpDovetail,
                &["dovetail", "guideway", "rail"][..],
            ),
            (
                CommandId::AddOpUndercut3D,
                &["3d undercut", "lollipop", "overhang"][..],
            ),
        ] {
            let entry = PALETTE_COMMANDS
                .iter()
                .find(|e| e.id == id)
                .unwrap_or_else(|| panic!("{id:?} entry present"));
            let searchable = format!("{} {} {}", entry.category, entry.label, entry.keywords);
            for query in queries {
                assert!(
                    fuzzy_match_score(query, &searchable).is_some(),
                    "expected {id:?} to match query '{query}'"
                );
            }
        }
    }

    #[test]
    fn command_palette_matches_project_properties_manage_tools_and_application_settings() {
        for (id, queries) in [
            (
                CommandId::OpenProjectProperties,
                &["project properties", "rename project"][..],
            ),
            (
                CommandId::OpenToolLibrary,
                &["manage tools", "tool library"][..],
            ),
            (
                CommandId::OpenSettings,
                &["application settings", "preferences"][..],
            ),
        ] {
            let entry = PALETTE_COMMANDS
                .iter()
                .find(|e| e.id == id)
                .unwrap_or_else(|| panic!("{id:?} entry present"));
            let searchable = format!("{} {} {}", entry.category, entry.label, entry.keywords);
            for query in queries {
                assert!(
                    fuzzy_match_score(query, &searchable).is_some(),
                    "expected {id:?} to match query '{query}'"
                );
            }
        }
    }
}
