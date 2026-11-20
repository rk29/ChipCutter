use std::{f64::consts::PI, io::Write as _};

use chip_math::DVec3;
use zip::{CompressionMethod, DateTime, ZipWriter, write::SimpleFileOptions};

use crate::mesh::MeshUnitKind;

/// Generates a deterministic, 100% watertight binary STL of a **Sculpted Hemispherical Dome**
/// (`R = 25.0 mm` hemisphere centered at `(0, 0, 8.0)`) seamlessly stitched onto a `70 × 70 × 8 mm`
/// square base pedestal (`X, Y ∈ [-35, 35]`, `Z ∈ [0, 33]`).
///
/// Serves both as the Phase 5 `Sculpted Dome (STL Mesh)` sample part and as an analytical
/// ground-truth oracle for Drop-Cutter and Push-Cutter surface contact tests.
pub fn sample_sculpted_dome_stl_bytes() -> Vec<u8> {
    let n_theta: usize = 96; // multiple of 4 so 4 square corners are hit exactly
    let n_phi: usize = 32;
    let radius = 25.0f64;
    let base_h = 8.0f64;
    let half_w = 35.0f64;

    let mut tris: Vec<[DVec3; 3]> = Vec::with_capacity(n_theta * (2 * n_phi + 5));

    // Precompute latitude rings `j = 0..n_phi` (j=0 is equator at z=base_h, j=n_phi is apex)
    let mut rings: Vec<Vec<DVec3>> = Vec::with_capacity(n_phi);
    for j in 0..n_phi {
        let phi = (j as f64 / n_phi as f64) * (0.5 * PI);
        let r = radius * phi.cos();
        let z = base_h + radius * phi.sin();
        let mut ring = Vec::with_capacity(n_theta);
        for k in 0..n_theta {
            let theta = (k as f64 / n_theta as f64) * (2.0 * PI);
            ring.push(DVec3::new(r * theta.cos(), r * theta.sin(), z));
        }
        rings.push(ring);
    }
    let apex = DVec3::new(0.0, 0.0, base_h + radius);

    // 1. Hemisphere lateral quads + apex cap
    for j in 0..(n_phi - 1) {
        for k in 0..n_theta {
            let kn = (k + 1) % n_theta;
            let a = rings[j][k];
            let b = rings[j][kn];
            let c = rings[j + 1][kn];
            let d = rings[j + 1][k];
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    let top_ring = &rings[n_phi - 1];
    for k in 0..n_theta {
        let kn = (k + 1) % n_theta;
        tris.push([top_ring[k], top_ring[kn], apex]);
    }

    // 2. Square top/bottom border rings projected along exact rays `theta`
    let mut sq_top = Vec::with_capacity(n_theta);
    let mut sq_bot = Vec::with_capacity(n_theta);
    for k in 0..n_theta {
        let theta = (k as f64 / n_theta as f64) * (2.0 * PI);
        let c = theta.cos();
        let s = theta.sin();
        let denom = c.abs().max(s.abs());
        let scale = half_w / denom;
        let x = scale * c;
        let y = scale * s;
        sq_top.push(DVec3::new(x, y, base_h));
        sq_bot.push(DVec3::new(x, y, 0.0));
    }

    // 3. Flat pedestal top flange (`Z = base_h`, outward normal `+Z`)
    let equator = &rings[0];
    for k in 0..n_theta {
        let kn = (k + 1) % n_theta;
        let c0 = equator[k];
        let c1 = equator[kn];
        let s0 = sq_top[k];
        let s1 = sq_top[kn];
        tris.push([c0, s0, s1]);
        tris.push([c0, s1, c1]);
    }

    // 4. Vertical pedestal walls (`Z = 0..base_h`, outward horizontal normals)
    for k in 0..n_theta {
        let kn = (k + 1) % n_theta;
        let t0 = sq_top[k];
        let t1 = sq_top[kn];
        let b0 = sq_bot[k];
        let b1 = sq_bot[kn];
        tris.push([b0, b1, t1]);
        tris.push([b0, t1, t0]);
    }

    // 5. Flat pedestal bottom face (`Z = 0`, outward normal `-Z`)
    let bot_center = DVec3::ZERO;
    for k in 0..n_theta {
        let kn = (k + 1) % n_theta;
        tris.push([bot_center, sq_bot[kn], sq_bot[k]]);
    }

    encode_binary_stl("ChipCutter Sculpted Dome (R=25mm on 70x70x8mm base)", &tris)
}

/// Generates a deterministic, watertight Wavefront `.obj` sinusoidal wave relief
/// (`80 × 60 × 15 mm`, `X ∈ [0, 80]`, `Y ∈ [0, 60]`, `Z ∈ [0, 15]`) using quad faces.
pub fn sample_relief_wave_obj_text() -> String {
    let nx: usize = 32;
    let ny: usize = 24;
    let w = 80.0f64;
    let h = 60.0f64;
    let base_z = 9.0f64;
    let amp_z = 3.0f64;

    let mut out = String::new();
    out.push_str("# ChipCutter Sample Sinusoidal Relief Wave\n");
    out.push_str("# units: mm\n");
    out.push_str("o relief_wave\n");

    let top_idx = |i: usize, j: usize| -> usize { 1 + j * (nx + 1) + i };
    let bot_offset = (nx + 1) * (ny + 1);
    let bot_idx = |i: usize, j: usize| -> usize { 1 + bot_offset + j * (nx + 1) + i };

    // Top surface vertices
    for j in 0..=ny {
        let y = (j as f64 / ny as f64) * h;
        for i in 0..=nx {
            let x = (i as f64 / nx as f64) * w;
            let z = base_z
                + amp_z * ((x / w) * 2.0 * PI).sin()
                + (0.5 * amp_z) * ((y / h) * 2.0 * PI).cos();
            out.push_str(&format!("v {x:.6} {y:.6} {z:.6}\n"));
        }
    }
    // Bottom surface vertices (`z = 0`)
    for j in 0..=ny {
        let y = (j as f64 / ny as f64) * h;
        for i in 0..=nx {
            let x = (i as f64 / nx as f64) * w;
            out.push_str(&format!("v {x:.6} {y:.6} 0.000000\n"));
        }
    }

    // Top quads (+Z outward)
    for j in 0..ny {
        for i in 0..nx {
            out.push_str(&format!(
                "f {} {} {} {}\n",
                top_idx(i, j),
                top_idx(i + 1, j),
                top_idx(i + 1, j + 1),
                top_idx(i, j + 1)
            ));
        }
    }
    // Bottom quads (-Z outward)
    for j in 0..ny {
        for i in 0..nx {
            out.push_str(&format!(
                "f {} {} {} {}\n",
                bot_idx(i, j),
                bot_idx(i, j + 1),
                bot_idx(i + 1, j + 1),
                bot_idx(i + 1, j)
            ));
        }
    }
    // Front wall (`j = 0`, -Y outward)
    for i in 0..nx {
        out.push_str(&format!(
            "f {} {} {} {}\n",
            bot_idx(i, 0),
            bot_idx(i + 1, 0),
            top_idx(i + 1, 0),
            top_idx(i, 0)
        ));
    }
    // Back wall (`j = ny`, +Y outward)
    for i in 0..nx {
        out.push_str(&format!(
            "f {} {} {} {}\n",
            bot_idx(i + 1, ny),
            bot_idx(i, ny),
            top_idx(i, ny),
            top_idx(i + 1, ny)
        ));
    }
    // Left wall (`i = 0`, -X outward)
    for j in 0..ny {
        out.push_str(&format!(
            "f {} {} {} {}\n",
            bot_idx(0, j + 1),
            bot_idx(0, j),
            top_idx(0, j),
            top_idx(0, j + 1)
        ));
    }
    // Right wall (`i = nx`, +X outward)
    for j in 0..ny {
        out.push_str(&format!(
            "f {} {} {} {}\n",
            bot_idx(nx, j),
            bot_idx(nx, j + 1),
            top_idx(nx, j + 1),
            top_idx(nx, j)
        ));
    }

    out
}

/// Generates a minimal, valid, watertight 3MF OPC ZIP archive containing a `2 × 1 × 0.5` box
/// tagged with `unit` and optional 12-float 3×4 `<item transform="...">` string.
pub fn sample_box_3mf_bytes(unit: MeshUnitKind, transform_str: Option<&str>) -> Vec<u8> {
    let unit_attr = match unit {
        MeshUnitKind::Millimeter => "millimeter",
        MeshUnitKind::Centimeter => "centimeter",
        MeshUnitKind::Meter => "meter",
        MeshUnitKind::Inch => "inch",
        MeshUnitKind::Foot => "foot",
        MeshUnitKind::Micron => "micron",
    };

    let transform_attr = match transform_str {
        Some(t) => format!(r#" transform="{t}""#),
        None => String::new(),
    };

    let model_xml = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="{unit_attr}" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" name="Sample3mfBox" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="2" y="0" z="0"/>
          <vertex x="2" y="1" z="0"/>
          <vertex x="0" y="1" z="0"/>
          <vertex x="0" y="0" z="0.5"/>
          <vertex x="2" y="0" z="0.5"/>
          <vertex x="2" y="1" z="0.5"/>
          <vertex x="0" y="1" z="0.5"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="2" v3="1"/>
          <triangle v1="0" v2="3" v3="2"/>
          <triangle v1="4" v2="5" v3="6"/>
          <triangle v1="4" v2="6" v3="7"/>
          <triangle v1="0" v2="1" v3="5"/>
          <triangle v1="0" v2="5" v3="4"/>
          <triangle v1="1" v2="2" v3="6"/>
          <triangle v1="1" v2="6" v3="5"/>
          <triangle v1="2" v2="3" v3="7"/>
          <triangle v1="2" v2="7" v3="6"/>
          <triangle v1="3" v2="0" v3="4"/>
          <triangle v1="3" v2="4" v3="7"/>
        </triangles>
      </mesh>
    </object>
  </resources>
  <build>
    <item objectid="1"{transform_attr}/>
  </build>
</model>
"#
    );

    let rels_xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#;

    let content_types_xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"#;

    let mut cursor = std::io::Cursor::new(Vec::new());
    {
        let mut zw = ZipWriter::new(&mut cursor);
        let opts = SimpleFileOptions::default()
            .compression_method(CompressionMethod::Deflated)
            .last_modified_time(DateTime::default());
        let _ = zw.start_file("[Content_Types].xml", opts);
        let _ = zw.write_all(content_types_xml.as_bytes());
        let _ = zw.start_file("_rels/.rels", opts);
        let _ = zw.write_all(rels_xml.as_bytes());
        let _ = zw.start_file("3D/3dmodel.model", opts);
        let _ = zw.write_all(model_xml.as_bytes());
        let _ = zw.finish();
    }
    cursor.into_inner()
}

/// Generates a deterministic, 100% watertight binary STL of a **4th-Axis Rotary Shaft**
/// (`Ø40 mm × 80 mm` aligned along the WCS `X / A` axis, `X ∈ [-40, +40]`) combining:
/// - Left & right `R = 20 mm` cylindrical journals,
/// - A 4-flat indexed wrench section (`±15 mm` flats at `90°` intervals),
/// - A reduced turned central collar groove (`R = 14 mm`), and
/// - A sculpted 3-lobe cam section (`R = 17.5 + 2.0 * cos(3θ)`) for polar target-gouge testing.
pub fn sample_rotary_shaft_stl_bytes() -> Vec<u8> {
    let n_theta: usize = 96;
    let x_stations: &[f64] = &[
        -40.0, -26.0, -25.0, -6.0, -5.0, -2.5, 7.5, 10.0, 12.0, 28.0, 30.0, 40.0,
    ];

    let profile_radius = |x: f64, theta: f64| -> f64 {
        let r_outer = 20.0_f64;
        if x <= -26.0 || x >= 30.0 {
            r_outer
        } else if (-25.0..=-6.0).contains(&x) {
            let denom = theta.cos().abs().max(theta.sin().abs()).max(1e-6);
            (15.0 / denom).min(r_outer)
        } else if (-26.0..-25.0).contains(&x) {
            let t = x + 26.0;
            let denom = theta.cos().abs().max(theta.sin().abs()).max(1e-6);
            let r_sq = (15.0 / denom).min(r_outer);
            r_outer + (r_sq - r_outer) * t
        } else if (-6.0..-5.0).contains(&x) {
            let t = x + 6.0;
            let denom = theta.cos().abs().max(theta.sin().abs()).max(1e-6);
            let r_sq = (15.0 / denom).min(r_outer);
            r_sq + (r_outer - r_sq) * t
        } else if (-5.0..-2.5).contains(&x) {
            let t = (x + 5.0) / 2.5;
            r_outer + (14.0 - r_outer) * t
        } else if (-2.5..=7.5).contains(&x) {
            14.0
        } else if (7.5..10.0).contains(&x) {
            let t = (x - 7.5) / 2.5;
            14.0 + (r_outer - 14.0) * t
        } else if (10.0..12.0).contains(&x) {
            let t = (x - 10.0) / 2.0;
            let r_cam = 17.5 + 2.0 * (3.0 * theta).cos();
            r_outer + (r_cam - r_outer) * t
        } else if (12.0..=28.0).contains(&x) {
            17.5 + 2.0 * (3.0 * theta).cos()
        } else {
            let t = (x - 28.0) / 2.0;
            let r_cam = 17.5 + 2.0 * (3.0 * theta).cos();
            r_cam + (r_outer - r_cam) * t
        }
    };

    let mut rings: Vec<Vec<DVec3>> = Vec::with_capacity(x_stations.len());
    for &x in x_stations {
        let mut ring = Vec::with_capacity(n_theta);
        for k in 0..n_theta {
            let theta = (k as f64 / n_theta as f64) * (2.0 * PI);
            let r = profile_radius(x, theta);
            // Match `RotaryAxisKind::XAxis`: e1 = +Z, e2 = -Y -> p = (x, -r sin θ, r cos θ)
            ring.push(DVec3::new(x, -r * theta.sin(), r * theta.cos()));
        }
        rings.push(ring);
    }

    let mut tris: Vec<[DVec3; 3]> = Vec::with_capacity(n_theta * (2 * x_stations.len() + 2));

    // Lateral skin (outward radial normals)
    for s in 0..(rings.len() - 1) {
        for k in 0..n_theta {
            let kn = (k + 1) % n_theta;
            let a = rings[s][k];
            let b = rings[s + 1][k];
            let c = rings[s + 1][kn];
            let d = rings[s][kn];
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }

    // Left end cap (`X = -40`, outward normal `-X`)
    let left_center = DVec3::new(x_stations[0], 0.0, 0.0);
    let left_ring = &rings[0];
    for k in 0..n_theta {
        let kn = (k + 1) % n_theta;
        tris.push([left_center, left_ring[k], left_ring[kn]]);
    }

    // Right end cap (`X = +40`, outward normal `+X`)
    let right_center = DVec3::new(x_stations[x_stations.len() - 1], 0.0, 0.0);
    let right_ring = &rings[rings.len() - 1];
    for k in 0..n_theta {
        let kn = (k + 1) % n_theta;
        tris.push([right_center, right_ring[kn], right_ring[k]]);
    }

    encode_binary_stl("ChipCutter Rotary Indexed Shaft Fixture (mm)", &tris)
}

fn encode_binary_stl(header_text: &str, tris: &[[DVec3; 3]]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(84 + tris.len() * 50);
    let mut header = [0u8; 80];
    let hb = header_text.as_bytes();
    let copy_len = hb.len().min(80);
    header[..copy_len].copy_from_slice(&hb[..copy_len]);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for &[v0, v1, v2] in tris {
        let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        for p in [n, v0, v1, v2] {
            buf.extend_from_slice(&(p.x as f32).to_le_bytes());
            buf.extend_from_slice(&(p.y as f32).to_le_bytes());
            buf.extend_from_slice(&(p.z as f32).to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    }
    buf
}
