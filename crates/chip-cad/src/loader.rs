//! Unified CAD (STEP) and 3D triangle mesh (STL/OBJ/3MF) model loader (`chip-cad`).

use std::path::Path;

use chip_math::DEFAULT_TESS_CHORD_TOL_MM;

use crate::{
    CadDocument, CadError, CadKernel, MeshImportOptions, MeshUnitKind, ModelTransform,
    MonstertruckKernel, is_mesh_extension, load_mesh_cad_document,
};

/// Returns `true` if `path` has a supported 3D triangle mesh extension (`.stl`, `.obj`, `.3mf`).
pub fn is_mesh_path(path: &Path) -> bool {
    path.extension()
        .and_then(|s| s.to_str())
        .is_some_and(is_mesh_extension)
}

/// Unified CAD/Mesh loader from in-memory bytes, dispatching `.stl`/`.obj`/`.3mf` to
/// [`load_mesh_cad_document`] and STEP files to [`MonstertruckKernel::load_step_bytes`].
pub fn load_model_bytes_cad_doc(
    source_name: &str,
    bytes: &[u8],
    unit_override: Option<MeshUnitKind>,
) -> Result<CadDocument, CadError> {
    if is_mesh_path(Path::new(source_name)) {
        load_mesh_cad_document(
            source_name,
            bytes,
            &MeshImportOptions {
                unit_override,
                ..MeshImportOptions::default()
            },
        )
    } else {
        MonstertruckKernel.load_step_bytes(source_name, bytes, DEFAULT_TESS_CHORD_TOL_MM)
    }
}

/// Unified CAD/Mesh loader reading `path` from disk and dispatching STEP vs. STL/OBJ/3MF cleanly.
pub fn load_model_entry_cad_doc(
    path: &Path,
    unit_override: Option<MeshUnitKind>,
) -> Result<CadDocument, CadError> {
    let bytes = std::fs::read(path)?;
    let source_name = path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("model.step");
    load_model_bytes_cad_doc(source_name, &bytes, unit_override)
}

/// Unified CAD/Mesh loader reading `path` from disk and applying `transform` in exact `f64` mm.
pub fn load_model_entry_cad_doc_transformed(
    path: &Path,
    unit_override: Option<MeshUnitKind>,
    transform: &ModelTransform,
) -> Result<CadDocument, CadError> {
    transform.validate()?;
    let raw = load_model_entry_cad_doc(path, unit_override)?;
    if transform.is_identity() {
        Ok(raw)
    } else {
        Ok(raw.transformed(transform))
    }
}
