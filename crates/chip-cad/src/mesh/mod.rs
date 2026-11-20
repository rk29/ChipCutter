pub mod bvh;
pub mod fixtures;
pub mod obj;
pub mod stl;
pub mod threemf;

use std::collections::HashMap;

use chip_math::{Aabb3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    kernel::CadError,
    model::{BodyId, CadDocument, FaceId, FaceTriangle, RenderMesh, RenderVertex, StepUnitKind},
};

pub use bvh::{MeshBvh, MeshRayHit};
pub use fixtures::{
    sample_box_3mf_bytes, sample_helical_flute_shaft_stl_bytes, sample_relief_wave_obj_text,
    sample_rotary_shaft_stl_bytes, sample_sculpted_dome_stl_bytes,
};

/// Source file format of an imported triangle mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeshFormat {
    StlBinary,
    StlAscii,
    Obj,
    ThreeMf,
}

impl MeshFormat {
    pub const fn label(self) -> &'static str {
        match self {
            Self::StlBinary => "Binary STL",
            Self::StlAscii => "ASCII STL",
            Self::Obj => "Wavefront OBJ",
            Self::ThreeMf => "3MF (OPC Archive)",
        }
    }

    pub const fn is_unitless_by_spec(self) -> bool {
        matches!(self, Self::StlBinary | Self::StlAscii | Self::Obj)
    }
}

/// Length unit of coordinates inside an imported triangle mesh prior to `f64` mm normalization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum MeshUnitKind {
    #[default]
    Millimeter,
    Centimeter,
    Meter,
    Inch,
    Foot,
    Micron,
}

impl MeshUnitKind {
    pub const ALL: [Self; 6] = [
        Self::Millimeter,
        Self::Inch,
        Self::Centimeter,
        Self::Meter,
        Self::Foot,
        Self::Micron,
    ];

    pub const fn scale_to_mm(self) -> f64 {
        match self {
            Self::Millimeter => 1.0,
            Self::Centimeter => 10.0,
            Self::Meter => 1000.0,
            Self::Inch => 25.4,
            Self::Foot => 304.8,
            Self::Micron => 0.001,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Millimeter => "Millimeters (mm)",
            Self::Centimeter => "Centimeters (cm)",
            Self::Meter => "Meters (m)",
            Self::Inch => "Inches (in)",
            Self::Foot => "Feet (ft)",
            Self::Micron => "Microns (µm)",
        }
    }

    pub const fn short_label(self) -> &'static str {
        match self {
            Self::Millimeter => "mm",
            Self::Centimeter => "cm",
            Self::Meter => "m",
            Self::Inch => "in",
            Self::Foot => "ft",
            Self::Micron => "um",
        }
    }

    pub fn from_str_loose(raw: &str) -> Option<Self> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "mm" | "millimeter" | "millimeters" | "millimetre" | "millimetres" => {
                Some(Self::Millimeter)
            }
            "cm" | "centimeter" | "centimeters" | "centimetre" | "centimetres" => {
                Some(Self::Centimeter)
            }
            "m" | "meter" | "meters" | "metre" | "metres" => Some(Self::Meter),
            "in" | "inch" | "inches" => Some(Self::Inch),
            "ft" | "foot" | "feet" => Some(Self::Foot),
            "um" | "µm" | "micron" | "microns" => Some(Self::Micron),
            _ => None,
        }
    }

    pub fn to_step_unit_kind(self) -> StepUnitKind {
        match self {
            Self::Millimeter => StepUnitKind::Millimeter,
            Self::Centimeter => StepUnitKind::Centimeter,
            Self::Meter => StepUnitKind::Meter,
            Self::Inch => StepUnitKind::Inch,
            Self::Foot | Self::Micron => StepUnitKind::CustomScaleToMm(self.scale_to_mm()),
        }
    }
}

/// Options controlling triangle mesh import normalization and vertex welding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshImportOptions {
    /// Explicit unit override; if `None`, uses file-declared unit (e.g. 3MF `<model unit="...">`)
    /// or defaults to [`MeshUnitKind::Millimeter`].
    pub unit_override: Option<MeshUnitKind>,
    /// Spatial welding distance in millimeters for merging coincident vertices (`1e-4` default).
    pub weld_tolerance_mm: f64,
}

impl Default for MeshImportOptions {
    fn default() -> Self {
        Self {
            unit_override: None,
            weld_tolerance_mm: 1e-4,
        }
    }
}

/// Single canonical `f64` triangle in a [`TriangleMeshBody`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshTriangle {
    pub indices: [u32; 3],
    pub normal: DVec3,
    pub aabb: Aabb3,
}

/// Topology, welding, and watertightness diagnostics collected on mesh import.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshDiagnostics {
    pub raw_triangle_count: usize,
    pub valid_triangle_count: usize,
    pub degenerate_triangles_removed: usize,
    pub welded_vertex_count: usize,
    pub open_boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub is_watertight: bool,
    pub signed_volume_mm3: f64,
}

/// Imported triangle mesh body normalized to canonical `f64` millimeters.
#[derive(Debug, Clone, PartialEq)]
pub struct TriangleMeshBody {
    pub id: BodyId,
    pub name: String,
    pub format: MeshFormat,
    pub detected_unit: Option<MeshUnitKind>,
    pub applied_unit: MeshUnitKind,
    pub vertices_mm: Vec<DVec3>,
    pub triangles: Vec<MeshTriangle>,
    pub aabb_mm: Aabb3,
    pub diagnostics: MeshDiagnostics,
}

impl TriangleMeshBody {
    /// Builds a [`MeshBvh`] acceleration structure for this body.
    pub fn build_bvh(&self) -> MeshBvh {
        MeshBvh::build(&self.triangles)
    }
}

/// Raw parsed triangle list in source file units prior to scaling and welding.
pub(crate) struct RawMeshData {
    pub name: String,
    pub format: MeshFormat,
    pub detected_unit: Option<MeshUnitKind>,
    pub raw_triangles: Vec<[DVec3; 3]>,
}

/// Returns `true` if `ext` (without leading dot, case-insensitive) is a supported mesh format.
pub fn is_mesh_extension(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("stl")
        || ext.eq_ignore_ascii_case("obj")
        || ext.eq_ignore_ascii_case("3mf")
}

/// Inspects `source_name` extension and parses `bytes` into a canonical `f64` millimeter [`TriangleMeshBody`].
pub fn load_mesh_bytes(
    source_name: &str,
    bytes: &[u8],
    opts: &MeshImportOptions,
) -> Result<TriangleMeshBody, CadError> {
    if bytes.is_empty() {
        return Err(CadError::EmptyInput);
    }

    let ext = std::path::Path::new(source_name)
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();

    let raw = match ext.as_str() {
        "stl" => stl::parse_stl(source_name, bytes)?,
        "obj" => obj::parse_obj(source_name, bytes)?,
        "3mf" => threemf::parse_3mf(source_name, bytes)?,
        other => {
            return Err(CadError::UnsupportedMeshFormat {
                extension: other.to_string(),
            });
        }
    };

    build_triangle_mesh_body(raw, opts, BodyId(0))
}

/// Loads a triangle mesh file into a unified [`CadDocument`] ready for viewport rendering,
/// Setup stock/WCS resolution, and 3-axis Drop-Cutter / Push-Cutter queries.
pub fn load_mesh_cad_document(
    source_name: &str,
    bytes: &[u8],
    opts: &MeshImportOptions,
) -> Result<CadDocument, CadError> {
    let mesh = load_mesh_bytes(source_name, bytes, opts)?;
    Ok(CadDocument::from_mesh(source_name.to_string(), mesh))
}

/// Scales raw triangles into millimeters, welds vertices spatially, filters degenerates,
/// and computes manifold/watertight diagnostics.
pub(crate) fn build_triangle_mesh_body(
    raw: RawMeshData,
    opts: &MeshImportOptions,
    body_id: BodyId,
) -> Result<TriangleMeshBody, CadError> {
    let raw_triangle_count = raw.raw_triangles.len();
    if raw_triangle_count == 0 {
        return Err(CadError::NoGeometry);
    }

    let applied_unit = opts
        .unit_override
        .or(raw.detected_unit)
        .unwrap_or(MeshUnitKind::Millimeter);
    let scale = applied_unit.scale_to_mm();
    let weld_tol = if opts.weld_tolerance_mm.is_finite() && opts.weld_tolerance_mm > 0.0 {
        opts.weld_tolerance_mm
    } else {
        1e-4
    };
    let weld_tol_sq = weld_tol * weld_tol;
    let inv_cell = 1.0 / weld_tol;

    let mut vertices_mm: Vec<DVec3> = Vec::with_capacity(raw_triangle_count);
    let mut cell_map: HashMap<(i64, i64, i64), Vec<u32>> =
        HashMap::with_capacity(raw_triangle_count);

    let mut weld_vertex = |p: DVec3| -> (u32, DVec3) {
        let cx = (p.x * inv_cell).floor() as i64;
        let cy = (p.y * inv_cell).floor() as i64;
        let cz = (p.z * inv_cell).floor() as i64;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(bucket) = cell_map.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &idx in bucket {
                            let existing = vertices_mm[idx as usize];
                            if existing.distance_squared(p) <= weld_tol_sq {
                                return (idx, existing);
                            }
                        }
                    }
                }
            }
        }
        let new_idx = vertices_mm.len() as u32;
        vertices_mm.push(p);
        cell_map.entry((cx, cy, cz)).or_default().push(new_idx);
        (new_idx, p)
    };

    let mut triangles: Vec<MeshTriangle> = Vec::with_capacity(raw_triangle_count);
    let mut degenerate_triangles_removed = 0usize;
    let mut aabb_mm = Aabb3::EMPTY;
    let mut signed_volume_6x = 0.0f64;

    // Edge incidence counters:
    // undirected edge `(min(a,b), max(a,b))` -> total incident triangle count
    // directed half-edge `(a, b)` -> count (if > 1, non-manifold / duplicate winding)
    let mut undirected_edges: HashMap<(u32, u32), u32> =
        HashMap::with_capacity(raw_triangle_count * 2);
    let mut directed_edges: HashMap<(u32, u32), u32> =
        HashMap::with_capacity(raw_triangle_count * 3);

    for raw_tri in raw.raw_triangles {
        let p0 = raw_tri[0] * scale;
        let p1 = raw_tri[1] * scale;
        let p2 = raw_tri[2] * scale;
        if !p0.is_finite() || !p1.is_finite() || !p2.is_finite() {
            degenerate_triangles_removed += 1;
            continue;
        }
        let (i0, v0) = weld_vertex(p0);
        let (i1, v1) = weld_vertex(p1);
        let (i2, v2) = weld_vertex(p2);
        if i0 == i1 || i1 == i2 || i2 == i0 {
            degenerate_triangles_removed += 1;
            continue;
        }

        let cross = (v1 - v0).cross(v2 - v0);
        let area2 = cross.length();
        // Area < 1e-10 mm^2 => area2 < 2e-10
        if !area2.is_finite() || area2 < 2e-10 {
            degenerate_triangles_removed += 1;
            continue;
        }

        let normal = cross / area2;
        let tri_min = v0.min(v1).min(v2);
        let tri_max = v0.max(v1).max(v2);
        let tri_aabb = Aabb3::new(tri_min, tri_max);
        aabb_mm = aabb_mm.union(&tri_aabb);
        signed_volume_6x += v0.dot(v1.cross(v2));

        for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
            let undir = if a < b { (a, b) } else { (b, a) };
            *undirected_edges.entry(undir).or_insert(0) += 1;
            *directed_edges.entry((a, b)).or_insert(0) += 1;
        }

        triangles.push(MeshTriangle {
            indices: [i0, i1, i2],
            normal,
            aabb: tri_aabb,
        });
    }

    if triangles.is_empty() || !aabb_mm.is_valid() {
        return Err(CadError::EmptyTessellation);
    }

    let mut open_boundary_edges = 0usize;
    let mut non_manifold_edges = 0usize;
    for (&(a, b), &count) in &undirected_edges {
        if count == 1 {
            open_boundary_edges += 1;
        } else if count > 2 {
            non_manifold_edges += 1;
        } else {
            // count == 2: check consistent orientation (should have 1 of (a,b) and 1 of (b,a))
            let fwd = directed_edges.get(&(a, b)).copied().unwrap_or(0);
            let rev = directed_edges.get(&(b, a)).copied().unwrap_or(0);
            if fwd != 1 || rev != 1 {
                non_manifold_edges += 1;
            }
        }
    }

    let is_watertight = open_boundary_edges == 0 && non_manifold_edges == 0;
    let diagnostics = MeshDiagnostics {
        raw_triangle_count,
        valid_triangle_count: triangles.len(),
        degenerate_triangles_removed,
        welded_vertex_count: vertices_mm.len(),
        open_boundary_edges,
        non_manifold_edges,
        is_watertight,
        signed_volume_mm3: signed_volume_6x / 6.0,
    };

    Ok(TriangleMeshBody {
        id: body_id,
        name: raw.name,
        format: raw.format,
        detected_unit: raw.detected_unit,
        applied_unit,
        vertices_mm,
        triangles,
        aabb_mm,
        diagnostics,
    })
}

/// Builds a viewport [`RenderMesh`] for `mesh` with crease-angle vertex normal smoothing:
/// adjacent triangles whose dihedral angle is `< crease_angle_deg` (default `35°`) share
/// smooth averaged normals, while sharper edges stay crisp flat-shaded.
pub fn build_render_mesh_for_triangle_mesh(
    mesh: &TriangleMeshBody,
    crease_angle_deg: f64,
    face_id_for_mesh: FaceId,
) -> RenderMesh {
    let cos_crease = crease_angle_deg.to_radians().cos();

    // Map each welded vertex -> list of incident triangle indices
    let mut vert_tris: Vec<Vec<usize>> = vec![Vec::new(); mesh.vertices_mm.len()];
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        for &vi in &tri.indices {
            if let Some(list) = vert_tris.get_mut(vi as usize) {
                list.push(ti);
            }
        }
    }

    let tri_count = mesh.triangles.len();
    let mut vertices = Vec::with_capacity(tri_count * 3);
    let mut indices = Vec::with_capacity(tri_count * 3);
    let mut pick_triangles = Vec::with_capacity(tri_count);

    for tri in &mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = mesh.vertices_mm[i0 as usize];
        let v1 = mesh.vertices_mm[i1 as usize];
        let v2 = mesh.vertices_mm[i2 as usize];

        let base = vertices.len() as u32;
        for &vi in &[i0, i1, i2] {
            let pos = mesh.vertices_mm[vi as usize];
            let mut acc_n = DVec3::ZERO;
            if let Some(inc) = vert_tris.get(vi as usize) {
                for &other_ti in inc {
                    let other_n = mesh.triangles[other_ti].normal;
                    if tri.normal.dot(other_n) >= cos_crease {
                        acc_n += other_n;
                    }
                }
            }
            let smooth_n = if acc_n.length_squared() > 1e-12 {
                acc_n.normalize()
            } else {
                tri.normal
            };
            vertices.push(RenderVertex {
                position_mm: [pos.x as f32, pos.y as f32, pos.z as f32],
                normal: [smooth_n.x as f32, smooth_n.y as f32, smooth_n.z as f32],
                face_id: face_id_for_mesh.0,
            });
        }
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);

        pick_triangles.push(FaceTriangle {
            face_id: face_id_for_mesh,
            v0_mm: v0,
            v1_mm: v1,
            v2_mm: v2,
        });
    }

    RenderMesh {
        vertices,
        indices,
        pick_triangles,
        edge_polylines: Vec::new(),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{Ray3, approx_eq};

    use super::*;

    #[test]
    fn parses_binary_stl_including_solid_header_trap_and_ascii_stl() {
        // 1. Watertight Sculpted Dome binary STL
        let dome_bytes = sample_sculpted_dome_stl_bytes();
        let dome = load_mesh_bytes(
            "sculpted_dome.stl",
            &dome_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert_eq!(dome.format, MeshFormat::StlBinary);
        assert!(
            dome.diagnostics.is_watertight,
            "dome must be watertight: {:?}",
            dome.diagnostics
        );
        assert_eq!(dome.diagnostics.open_boundary_edges, 0);
        assert_eq!(dome.diagnostics.non_manifold_edges, 0);
        assert_eq!(dome.diagnostics.degenerate_triangles_removed, 0);
        let sz = dome.aabb_mm.size();
        assert!(approx_eq(sz.x, 70.0, 1e-3));
        assert!(approx_eq(sz.y, 70.0, 1e-3));
        assert!(approx_eq(sz.z, 33.0, 1e-3));
        // Expected volume = 70*70*8 + (2/3)*pi*25^3 = 39200 + 32724.92 = 71924.92 mm^3
        assert!(
            (dome.diagnostics.signed_volume_mm3 - 71_924.92).abs() < 350.0,
            "unexpected dome volume: {}",
            dome.diagnostics.signed_volume_mm3
        );

        // 2. Binary STL whose 80-byte header intentionally starts with `"solid "` (classic trap!)
        let mut trap_bytes = dome_bytes.clone();
        trap_bytes[..11].copy_from_slice(b"solid TRAP!");
        let trap = load_mesh_bytes("trap.stl", &trap_bytes, &MeshImportOptions::default()).unwrap();
        assert_eq!(trap.format, MeshFormat::StlBinary);
        assert_eq!(trap.triangles.len(), dome.triangles.len());

        // 3. ASCII STL with inch override
        let ascii_stl = r#"solid unit_tri
  facet normal 0 0 1
    outer loop
      vertex 0.0 0.0 0.0
      vertex 1.0 0.0 0.0
      vertex 0.0 1.0 0.0
    endloop
  endfacet
endsolid unit_tri
"#;
        let tri_in = load_mesh_bytes(
            "tri.stl",
            ascii_stl.as_bytes(),
            &MeshImportOptions {
                unit_override: Some(MeshUnitKind::Inch),
                ..MeshImportOptions::default()
            },
        )
        .unwrap();
        assert_eq!(tri_in.format, MeshFormat::StlAscii);
        assert_eq!(tri_in.triangles.len(), 1);
        assert!(approx_eq(tri_in.aabb_mm.max.x, 25.4, 1e-9));
        assert_eq!(tri_in.diagnostics.open_boundary_edges, 3);
        assert!(!tri_in.diagnostics.is_watertight);
    }

    #[test]
    fn parses_wavefront_obj_quads_negative_indices_and_relief_wave() {
        let obj_text = sample_relief_wave_obj_text();
        let relief = load_mesh_bytes(
            "relief_wave.obj",
            obj_text.as_bytes(),
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert_eq!(relief.format, MeshFormat::Obj);
        assert!(
            relief.diagnostics.is_watertight,
            "relief wave OBJ must be watertight: {:?}",
            relief.diagnostics
        );
        let sz = relief.aabb_mm.size();
        assert!(approx_eq(sz.x, 80.0, 1e-3));
        assert!(approx_eq(sz.y, 60.0, 1e-3));
        assert!(sz.z > 10.0 && sz.z <= 15.5);

        // Test negative indices + v/vt/vn syntax + n-gon fan triangulation
        let neg_obj = "\
# units: cm
o pentagon_plate
v 0 0 0
v 2 0 0
v 2 2 0
v 1 3 0
v 0 2 0
f -5/1/1 -4/2/1 -3/3/1 -2/4/1 -1/5/1
";
        let pent = load_mesh_bytes(
            "pent.obj",
            neg_obj.as_bytes(),
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert_eq!(pent.detected_unit, Some(MeshUnitKind::Centimeter));
        assert_eq!(pent.applied_unit, MeshUnitKind::Centimeter);
        assert_eq!(pent.triangles.len(), 3);
        assert!(approx_eq(pent.aabb_mm.max.x, 20.0, 1e-9));
        assert!(approx_eq(pent.aabb_mm.max.y, 30.0, 1e-9));
    }

    #[test]
    fn parses_3mf_units_transforms_and_rejects_zip_slip() {
        use std::io::Write as _;

        // 1. Inch 3MF with 3x4 translation transform `(1, 2, 0)` in inches!
        let bytes_in = sample_box_3mf_bytes(MeshUnitKind::Inch, Some("1 0 0 0 1 0 0 0 1 1 2 0"));
        let box_in =
            load_mesh_bytes("inch_box.3mf", &bytes_in, &MeshImportOptions::default()).unwrap();
        assert_eq!(box_in.format, MeshFormat::ThreeMf);
        assert_eq!(box_in.detected_unit, Some(MeshUnitKind::Inch));
        assert_eq!(box_in.applied_unit, MeshUnitKind::Inch);
        assert!(box_in.diagnostics.is_watertight);
        // Unit box [0..2, 0..1, 0..0.5] in shifted by (1, 2, 0) in => [25.4..76.2, 50.8..76.2, 0..12.7] mm
        assert!(approx_eq(box_in.aabb_mm.min.x, 25.4, 1e-6));
        assert!(approx_eq(box_in.aabb_mm.max.x, 76.2, 1e-6));
        assert!(approx_eq(box_in.aabb_mm.min.y, 50.8, 1e-6));
        assert!(approx_eq(box_in.aabb_mm.max.z, 12.7, 1e-6));

        // 2. Zip-slip malicious archive must be rejected cleanly
        let mut bad_zip = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut bad_zip);
            let opts = zip::write::SimpleFileOptions::default();
            zw.start_file("../escape.model", opts).unwrap();
            zw.write_all(b"<model/>").unwrap();
            zw.finish().unwrap();
        }
        let err = load_mesh_bytes(
            "evil.3mf",
            &bad_zip.into_inner(),
            &MeshImportOptions::default(),
        )
        .unwrap_err();
        assert!(matches!(err, CadError::UnsafeArchivePath { .. }));
    }

    #[test]
    fn mesh_bvh_xy_circle_z_slab_and_raycast_are_accurate() {
        let dome_bytes = sample_sculpted_dome_stl_bytes();
        let dome = load_mesh_bytes("dome.stl", &dome_bytes, &MeshImportOptions::default()).unwrap();
        let bvh = dome.build_bvh();

        // Downward ray at (0, 0, 100) must strike dome apex at Z = 33.0 mm
        let ray = Ray3::new(DVec3::new(0.0, 0.0, 100.0), DVec3::new(0.0, 0.0, -1.0)).unwrap();
        let hit = bvh
            .raycast(&dome.vertices_mm, &dome.triangles, &ray)
            .unwrap();
        assert!(approx_eq(hit.point_mm.z, 33.0, 1e-4));

        // XY circle query at center finds dome triangles; outside bounds finds none
        assert!(!bvh.query_xy_circle(0.0, 0.0, 5.0).is_empty());
        assert!(bvh.query_xy_circle(200.0, 200.0, 5.0).is_empty());

        // Z-slab query around Z=20 finds hemisphere slice triangles
        let slab = bvh.query_z_slab(19.5, 20.5);
        assert!(!slab.is_empty());
    }
}
