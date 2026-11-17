use glam::{DMat4, DVec3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Errors raised when constructing a right-handed orthonormal [`RigidFrame3`].
#[derive(Debug, Clone, PartialEq, Error)]
pub enum FrameError {
    #[error("frame input contains non-finite coordinates (NaN or Inf)")]
    NonFiniteInput,
    #[error("{axis} axis vector has near-zero length")]
    ZeroLengthAxis { axis: &'static str },
    #[error("Z axis and X reference axis are collinear (|cos θ| = {dot_abs:.6})")]
    CollinearAxes { dot_abs: f64 },
}

/// Canonical `f64` right-handed orthonormal `SE(3)` coordinate frame (`X × Y = Z`) in millimeters.
///
/// Used primarily to map between raw CAD model world coordinates and a Setup's
/// Work Coordinate System (`WCS`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RigidFrame3 {
    pub origin_mm: DVec3,
    pub x_axis: DVec3,
    pub y_axis: DVec3,
    pub z_axis: DVec3,
}

impl Default for RigidFrame3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl RigidFrame3 {
    /// Identity frame aligned with world XYZ at `(0, 0, 0)`.
    pub const IDENTITY: Self = Self {
        origin_mm: DVec3::ZERO,
        x_axis: DVec3::X,
        y_axis: DVec3::Y,
        z_axis: DVec3::Z,
    };

    /// Builds a right-handed orthonormal frame from a primary `Z` direction and an optional `X` hint.
    ///
    /// - `flip_z` negates `Z` before projecting `X`.
    /// - If `raw_x_hint` is `Some`, it is projected perpendicular to `Z` and rejected with
    ///   [`FrameError::CollinearAxes`] if nearly parallel to `Z`.
    /// - If `raw_x_hint` is `None`, a deterministic perpendicular axis is chosen automatically.
    /// - `flip_x` negates `X`, `Y` is derived as `Z × X`, and `clock_90_steps` rotates `(X, Y)`
    ///   counter-clockwise around `+Z` in exact `90°` increments.
    pub fn from_z_and_x_hint(
        origin_mm: DVec3,
        raw_z: DVec3,
        raw_x_hint: Option<DVec3>,
        flip_z: bool,
        flip_x: bool,
        clock_90_steps: u8,
    ) -> Result<Self, FrameError> {
        if !origin_mm.is_finite() || !raw_z.is_finite() {
            return Err(FrameError::NonFiniteInput);
        }
        let z_len = raw_z.length();
        if z_len < 1e-9 {
            return Err(FrameError::ZeroLengthAxis { axis: "Z" });
        }
        let mut z = raw_z / z_len;
        if flip_z {
            z = -z;
        }

        let mut x = match raw_x_hint {
            Some(hx) => {
                if !hx.is_finite() {
                    return Err(FrameError::NonFiniteInput);
                }
                let hx_len = hx.length();
                if hx_len < 1e-9 {
                    return Err(FrameError::ZeroLengthAxis { axis: "X" });
                }
                let hx_unit = hx / hx_len;
                let dot_abs = hx_unit.dot(z).abs();
                let proj = hx_unit - z * hx_unit.dot(z);
                let proj_len = proj.length();
                if proj_len < 1e-6 {
                    return Err(FrameError::CollinearAxes { dot_abs });
                }
                proj / proj_len
            }
            None => {
                // Choose deterministic world reference not nearly parallel to Z
                let fallback = if z.dot(DVec3::X).abs() < 0.9 {
                    DVec3::X
                } else {
                    DVec3::Y
                };
                let proj = fallback - z * fallback.dot(z);
                proj.normalize()
            }
        };

        if flip_x {
            x = -x;
        }

        let y = z.cross(x).normalize();
        // Re-orthogonalize X tightly against (Y, Z) for ~1e-16 accuracy
        let x = y.cross(z).normalize();

        let (x_rot, y_rot) = match clock_90_steps % 4 {
            0 => (x, y),
            1 => (y, -x),
            2 => (-x, -y),
            _ => (-y, x),
        };

        Ok(Self {
            origin_mm,
            x_axis: x_rot,
            y_axis: y_rot,
            z_axis: z,
        })
    }

    /// Returns a copy of this frame anchored at `origin_mm`.
    #[inline]
    pub fn with_origin(mut self, origin_mm: DVec3) -> Self {
        self.origin_mm = origin_mm;
        self
    }

    /// Transforms a world-space point (`mm`) into local frame coordinates (`mm`).
    #[inline]
    pub fn world_to_local_point(&self, p_world: DVec3) -> DVec3 {
        let d = p_world - self.origin_mm;
        DVec3::new(d.dot(self.x_axis), d.dot(self.y_axis), d.dot(self.z_axis))
    }

    /// Transforms a local-frame point (`mm`) into world-space coordinates (`mm`).
    #[inline]
    pub fn local_to_world_point(&self, p_local: DVec3) -> DVec3 {
        self.origin_mm + self.x_axis * p_local.x + self.y_axis * p_local.y + self.z_axis * p_local.z
    }

    /// Transforms a world-space direction/vector into local frame coordinates (no translation).
    #[inline]
    pub fn world_to_local_dir(&self, v_world: DVec3) -> DVec3 {
        DVec3::new(
            v_world.dot(self.x_axis),
            v_world.dot(self.y_axis),
            v_world.dot(self.z_axis),
        )
    }

    /// Transforms a local-frame direction/vector into world-space coordinates (no translation).
    #[inline]
    pub fn local_to_world_dir(&self, v_local: DVec3) -> DVec3 {
        self.x_axis * v_local.x + self.y_axis * v_local.y + self.z_axis * v_local.z
    }

    /// Homogeneous `4x4` matrix mapping local points to world points (`P_world = M * P_local`).
    #[inline]
    pub fn to_dmat4_local_to_world(&self) -> DMat4 {
        DMat4::from_cols(
            self.x_axis.extend(0.0),
            self.y_axis.extend(0.0),
            self.z_axis.extend(0.0),
            self.origin_mm.extend(1.0),
        )
    }

    /// Exact rigid-body `4x4` inverse mapping world points to local points (`P_local = M^-1 * P_world`).
    #[inline]
    pub fn to_dmat4_world_to_local(&self) -> DMat4 {
        let tx = -self.origin_mm.dot(self.x_axis);
        let ty = -self.origin_mm.dot(self.y_axis);
        let tz = -self.origin_mm.dot(self.z_axis);
        DMat4::from_cols_array(&[
            self.x_axis.x,
            self.y_axis.x,
            self.z_axis.x,
            0.0,
            self.x_axis.y,
            self.y_axis.y,
            self.z_axis.y,
            0.0,
            self.x_axis.z,
            self.y_axis.z,
            self.z_axis.z,
            0.0,
            tx,
            ty,
            tz,
            1.0,
        ])
    }

    /// Verifies that the axes form a right-handed orthonormal basis within `tol`.
    pub fn is_orthonormal_right_handed(&self, tol: f64) -> bool {
        let len_ok = (self.x_axis.length() - 1.0).abs() <= tol
            && (self.y_axis.length() - 1.0).abs() <= tol
            && (self.z_axis.length() - 1.0).abs() <= tol;
        let ortho_ok = self.x_axis.dot(self.y_axis).abs() <= tol
            && self.y_axis.dot(self.z_axis).abs() <= tol
            && self.z_axis.dot(self.x_axis).abs() <= tol;
        let rh_ok = (self.x_axis.cross(self.y_axis) - self.z_axis).length() <= tol;
        len_ok && ortho_ok && rh_ok
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn constructs_right_handed_orthonormal_frames_and_roundtrips_points() {
        let z_tilted = DVec3::new(0.0, 0.5, 0.866_025_403_784_438_6);
        let x_hint = DVec3::new(1.0, 0.2, 0.0);
        for clock in 0..4 {
            for flip_z in [false, true] {
                for flip_x in [false, true] {
                    let f = RigidFrame3::from_z_and_x_hint(
                        DVec3::new(12.5, -8.0, 4.25),
                        z_tilted,
                        Some(x_hint),
                        flip_z,
                        flip_x,
                        clock,
                    )
                    .unwrap();
                    assert!(f.is_orthonormal_right_handed(1e-12));

                    let p_world = DVec3::new(-33.2, 48.7, 19.1);
                    let p_local = f.world_to_local_point(p_world);
                    let roundtrip = f.local_to_world_point(p_local);
                    assert!((roundtrip - p_world).length() < 1e-11);

                    let m_l2w = f.to_dmat4_local_to_world();
                    let m_w2l = f.to_dmat4_world_to_local();
                    let p_mat = m_w2l.transform_point3(p_world);
                    assert!((p_mat - p_local).length() < 1e-11);
                    let rt_mat = m_l2w.transform_point3(p_mat);
                    assert!((rt_mat - p_world).length() < 1e-11);
                }
            }
        }
    }

    #[test]
    fn rejects_collinear_or_zero_axes() {
        let err = RigidFrame3::from_z_and_x_hint(
            DVec3::ZERO,
            DVec3::Z,
            Some(DVec3::new(0.0, 0.0, -3.0)),
            false,
            false,
            0,
        )
        .unwrap_err();
        assert!(matches!(err, FrameError::CollinearAxes { .. }));

        let zero_err =
            RigidFrame3::from_z_and_x_hint(DVec3::ZERO, DVec3::ZERO, None, false, false, 0)
                .unwrap_err();
        assert!(matches!(zero_err, FrameError::ZeroLengthAxis { axis: "Z" }));
    }
}
