use thiserror::Error;

use crate::model::CadDocument;

/// Errors returned while loading or tessellating a CAD model.
#[derive(Debug, Error)]
pub enum CadError {
    #[error("STEP file is empty")]
    EmptyInput,
    #[error("failed to parse STEP exchange data: {0}")]
    StepParse(String),
    #[error("STEP file contains no solid or shell B-Rep geometry")]
    NoGeometry,
    #[error("failed to extract B-Rep topology for shell #{shell_id}: {detail}")]
    TopologyConversion { shell_id: u64, detail: String },
    #[error("tessellation produced zero renderable triangles")]
    EmptyTessellation,
    #[error("failed to parse triangle mesh ({format}): {detail}")]
    MeshParse {
        format: &'static str,
        detail: String,
    },
    #[error("unsupported mesh file format '{extension}' (expected .stl, .obj, or .3mf)")]
    UnsupportedMeshFormat { extension: String },
    #[error("unsafe archive entry path '{entry_path}' rejected in 3MF container")]
    UnsafeArchivePath { entry_path: String },
    #[error("3MF archive exceeds maximum allowed uncompressed size ({limit_bytes} bytes)")]
    ArchiveTooLarge { limit_bytes: u64 },
    #[error("invalid model transform: {0}")]
    InvalidTransform(String),
    #[error("I/O error reading CAD file: {0}")]
    Io(#[from] std::io::Error),
}

/// Abstract CAD / B-Rep kernel boundary (`chip-cad`).
///
/// Downstream crates (`chip-cam`, `chip-cli`, `chip-gui`) depend solely on this trait and
/// [`CadDocument`], keeping `monstertruck` fully encapsulated.
pub trait CadKernel: Send + Sync {
    fn load_step_bytes(
        &self,
        source_name: &str,
        step_bytes: &[u8],
        chord_tol_mm: f64,
    ) -> Result<CadDocument, CadError>;
}
