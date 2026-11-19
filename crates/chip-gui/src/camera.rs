use chip_math::{Aabb3, DMat4, DVec3, DVec4, Ray3, RigidFrame3};
use glam::{Mat4, Vec3};
use serde::{Deserialize, Serialize};

/// Camera projection mode. Orthographic is the default for CAD/CAM inspection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProjectionMode {
    Orthographic {
        /// Visible world height in millimeters at the target plane.
        ortho_height_mm: f64,
    },
    Perspective {
        /// Vertical field-of-view in radians.
        fov_y_rad: f64,
    },
}

impl ProjectionMode {
    pub const fn is_orthographic(self) -> bool {
        matches!(self, Self::Orthographic { .. })
    }
}

/// Mouse navigation binding presets matching mainstream CAD/CAM packages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MousePreset {
    #[default]
    Fusion360,
    SolidWorks,
    FreeCadBlender,
}

impl MousePreset {
    pub const ALL: [Self; 3] = [Self::Fusion360, Self::SolidWorks, Self::FreeCadBlender];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Fusion360 => "Fusion 360 (Shift+MMB Orbit, MMB Pan)",
            Self::SolidWorks => "SolidWorks (MMB Orbit, Ctrl+MMB Pan)",
            Self::FreeCadBlender => "FreeCAD / Blender (MMB Orbit, Shift+MMB Pan)",
        }
    }

    pub const fn short_label(self) -> &'static str {
        match self {
            Self::Fusion360 => "Fusion 360",
            Self::SolidWorks => "SolidWorks",
            Self::FreeCadBlender => "FreeCAD/Blender",
        }
    }
}

/// Orbit constraint mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OrbitMode {
    /// Keeps `+Z` spindle axis upright on screen (default CAM turntable).
    #[default]
    TurntableZUp,
    /// Unconstrained 3D trackball rotation.
    FreeTrackball,
}

impl OrbitMode {
    pub const fn label(self) -> &'static str {
        match self {
            Self::TurntableZUp => "Turntable (+Z Up)",
            Self::FreeTrackball => "Free Trackball",
        }
    }
}

/// Standard orthographic/isometric CAD view orientations (`+Z` spindle up, `+X` right, `+Y` back).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StandardView {
    Isometric,
    Top,
    Bottom,
    Front,
    Back,
    Right,
    Left,
}

impl StandardView {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Isometric => "Iso",
            Self::Top => "Top (+Z)",
            Self::Bottom => "Bottom (-Z)",
            Self::Front => "Front (-Y)",
            Self::Back => "Back (+Y)",
            Self::Right => "Right (+X)",
            Self::Left => "Left (-X)",
        }
    }
}

/// Precision `f64` CAD/CAM camera producing `f32` GPU matrices and exact `f64` picking rays.
#[derive(Debug, Clone)]
pub struct CadCamera {
    pub target_mm: DVec3,
    pub distance_mm: f64,
    /// Azimuth around `+Z` in radians (`0` = looking from `+X`, `-π/2` = looking from `-Y` front).
    pub yaw_rad: f64,
    /// Elevation above XY plane in radians (`+π/2` = looking straight down from `+Z` Top).
    pub pitch_rad: f64,
    /// Free-trackball orientation quaternion (used when `orbit_mode == FreeTrackball`).
    pub trackball_rot: chip_math::DQuat,
    pub projection: ProjectionMode,
    pub mouse_preset: MousePreset,
    pub orbit_mode: OrbitMode,
    pub scene_radius_mm: f64,
    /// Orthonormal reference frame for Turntable orbit and standard views (defaults to world XYZ,
    /// or locks onto the active Setup's WCS `+Z` spindle axis!).
    pub turntable_frame: RigidFrame3,
}

impl Default for CadCamera {
    fn default() -> Self {
        let mut cam = Self {
            target_mm: DVec3::new(40.0, 30.0, 8.0),
            distance_mm: 180.0,
            yaw_rad: -std::f64::consts::FRAC_PI_4 * 1.35,
            pitch_rad: 0.62,
            trackball_rot: chip_math::DQuat::IDENTITY,
            projection: ProjectionMode::Orthographic {
                ortho_height_mm: 100.0,
            },
            mouse_preset: MousePreset::Fusion360,
            orbit_mode: OrbitMode::TurntableZUp,
            scene_radius_mm: 75.0,
            turntable_frame: RigidFrame3::IDENTITY,
        };
        cam.sync_trackball_from_angles();
        cam
    }
}

impl CadCamera {
    /// Camera eye position, unit forward (`target - eye`), unit right, and unit up vectors in `f64` mm.
    pub fn basis(&self) -> (DVec3, DVec3, DVec3, DVec3) {
        match self.orbit_mode {
            OrbitMode::TurntableZUp => {
                let cp = self.pitch_rad.cos();
                let sp = self.pitch_rad.sin();
                let cy = self.yaw_rad.cos();
                let sy = self.yaw_rad.sin();
                // Offset from target to eye in active `turntable_frame` coordinates
                let eye_dir_local = DVec3::new(cp * cy, cp * sy, sp);
                let eye_dir = self
                    .turntable_frame
                    .local_to_world_dir(eye_dir_local)
                    .normalize_or_zero();
                let forward = (-eye_dir).normalize_or_zero();
                // Horizontal tangent around turntable +Z so even at pitch = ±90° right vector never degenerates!
                let right_local = DVec3::new(-sy, cy, 0.0).normalize_or_zero();
                let right = self
                    .turntable_frame
                    .local_to_world_dir(right_local)
                    .normalize_or_zero();
                let up = right.cross(forward).normalize_or_zero();
                let eye = self.target_mm + eye_dir * self.distance_mm;
                (eye, forward, right, up)
            }
            OrbitMode::FreeTrackball => {
                let right = (self.trackball_rot * DVec3::X).normalize();
                let up = (self.trackball_rot * DVec3::Y).normalize();
                let back = (self.trackball_rot * DVec3::Z).normalize();
                let forward = -back;
                let eye = self.target_mm + back * self.distance_mm;
                (eye, forward, right, up)
            }
        }
    }

    fn sync_trackball_from_angles(&mut self) {
        let (_, forward, right, up) = {
            let mut tmp = self.clone();
            tmp.orbit_mode = OrbitMode::TurntableZUp;
            tmp.basis()
        };
        let back = -forward;
        let rot_mat = chip_math::DMat3::from_cols(right, up, back);
        self.trackball_rot = chip_math::DQuat::from_mat3(&rot_mat).normalize();
    }

    pub fn set_orbit_mode(&mut self, mode: OrbitMode) {
        if self.orbit_mode == mode {
            return;
        }
        if mode == OrbitMode::FreeTrackball {
            self.sync_trackball_from_angles();
        } else {
            let (eye, _, _, _) = self.basis();
            let offset_world = (eye - self.target_mm).normalize_or_zero();
            let offset = self.turntable_frame.world_to_local_dir(offset_world);
            self.pitch_rad = offset.z.clamp(-1.0, 1.0).asin();
            self.yaw_rad = offset.y.atan2(offset.x);
        }
        self.orbit_mode = mode;
    }

    /// Toggles between Orthographic and Perspective while matching apparent scale at `target_mm`.
    pub fn toggle_projection(&mut self) {
        self.projection = match self.projection {
            ProjectionMode::Orthographic { ortho_height_mm } => {
                let fov_y_rad = 38.0_f64.to_radians();
                self.distance_mm =
                    (ortho_height_mm / (2.0 * (0.5 * fov_y_rad).tan())).clamp(5.0, 1e6);
                ProjectionMode::Perspective { fov_y_rad }
            }
            ProjectionMode::Perspective { fov_y_rad } => {
                let ortho_height_mm =
                    (2.0 * self.distance_mm * (0.5 * fov_y_rad).tan()).clamp(1.0, 1e6);
                ProjectionMode::Orthographic { ortho_height_mm }
            }
        };
    }

    /// Snaps camera orientation to a standard CAD view without losing zoom or target.
    pub fn set_standard_view(&mut self, view: StandardView) {
        let eps = 1e-4;
        match view {
            StandardView::Isometric => {
                self.yaw_rad = -std::f64::consts::FRAC_PI_4 * 1.35;
                self.pitch_rad = 0.615_479_7;
            }
            StandardView::Top => {
                // +X right, +Y up on screen => yaw = -π/2, pitch = +π/2
                self.yaw_rad = -std::f64::consts::FRAC_PI_2;
                self.pitch_rad = std::f64::consts::FRAC_PI_2 - eps;
            }
            StandardView::Bottom => {
                self.yaw_rad = std::f64::consts::FRAC_PI_2;
                self.pitch_rad = -std::f64::consts::FRAC_PI_2 + eps;
            }
            StandardView::Front => {
                self.yaw_rad = -std::f64::consts::FRAC_PI_2;
                self.pitch_rad = 0.0;
            }
            StandardView::Back => {
                self.yaw_rad = std::f64::consts::FRAC_PI_2;
                self.pitch_rad = 0.0;
            }
            StandardView::Right => {
                self.yaw_rad = 0.0;
                self.pitch_rad = 0.0;
            }
            StandardView::Left => {
                self.yaw_rad = std::f64::consts::PI;
                self.pitch_rad = 0.0;
            }
        }
        self.sync_trackball_from_angles();
    }

    /// Frames a 3D bounding box comfortably inside the viewport.
    pub fn fit_aabb(&mut self, aabb: &Aabb3) {
        if !aabb.is_valid() {
            return;
        }
        self.target_mm = aabb.center();
        let radius = (0.5 * aabb.diagonal()).max(5.0);
        self.scene_radius_mm = radius;
        let padded_h = radius * 2.35;
        match &mut self.projection {
            ProjectionMode::Orthographic { ortho_height_mm } => {
                *ortho_height_mm = padded_h;
                self.distance_mm = radius * 3.5;
            }
            ProjectionMode::Perspective { fov_y_rad } => {
                self.distance_mm = padded_h / (2.0 * (0.5 * *fov_y_rad).tan());
            }
        }
    }

    /// Locks the turntable horizon to `wcs_frame` and aligns the camera looking straight down
    /// Setup `-Z` with `+X` right and `+Y` up (`08-ui-viewport.md`).
    pub fn align_to_setup_top(&mut self, wcs_frame: &RigidFrame3, bounds_wcs: &Aabb3) {
        self.turntable_frame = *wcs_frame;
        if bounds_wcs.is_valid() {
            self.target_mm = wcs_frame.local_to_world_point(bounds_wcs.center());
            let radius = (0.5 * bounds_wcs.diagonal()).max(5.0);
            self.scene_radius_mm = radius;
            let padded_h = radius * 2.4;
            match &mut self.projection {
                ProjectionMode::Orthographic { ortho_height_mm } => {
                    *ortho_height_mm = padded_h;
                    self.distance_mm = radius * 3.5;
                }
                ProjectionMode::Perspective { fov_y_rad } => {
                    self.distance_mm = padded_h / (2.0 * (0.5 * *fov_y_rad).tan());
                }
            }
        }
        self.set_standard_view(StandardView::Top);
    }

    /// Switches to Orthographic projection and looks straight down `-Z` of `sketch_frame`,
    /// framing `bounds_2d` (with comfortable padding).
    pub fn align_to_sketch_plane(
        &mut self,
        sketch_frame: &RigidFrame3,
        bounds_2d: &chip_math::Aabb2,
    ) {
        self.turntable_frame = *sketch_frame;
        let (center_2d, diag_mm) = if bounds_2d.is_valid() && bounds_2d.size().length() > 1.0 {
            (bounds_2d.center(), bounds_2d.size().length())
        } else {
            (chip_math::DVec2::ZERO, 120.0)
        };
        self.target_mm =
            sketch_frame.local_to_world_point(DVec3::new(center_2d.x, center_2d.y, 0.0));
        let radius = (0.5 * diag_mm).max(15.0);
        self.scene_radius_mm = radius;
        self.distance_mm = radius * 3.5;
        self.projection = ProjectionMode::Orthographic {
            ortho_height_mm: radius * 2.35,
        };
        self.set_standard_view(StandardView::Top);
    }

    /// Projects a 3D world-space point (`mm`) into 2D `egui` screen coordinates inside `viewport_rect`.
    /// Returns `None` if behind the camera near clip plane.
    pub fn project_world_to_screen(
        &self,
        world_pt_mm: DVec3,
        viewport_rect: egui::Rect,
    ) -> Option<egui::Pos2> {
        if viewport_rect.width() < 2.0 || viewport_rect.height() < 2.0 {
            return None;
        }
        let aspect = (viewport_rect.width() as f64) / (viewport_rect.height() as f64);
        let (view_d, proj_d) = self.view_proj_f64(aspect);
        let clip = (proj_d * view_d) * world_pt_mm.extend(1.0);
        if clip.w <= 1e-9 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        if ndc.z < -0.1 || ndc.z > 1.05 {
            return None;
        }
        let sx = viewport_rect.left() + ((ndc.x + 1.0) * 0.5) as f32 * viewport_rect.width();
        let sy = viewport_rect.top() + ((1.0 - ndc.y) * 0.5) as f32 * viewport_rect.height();
        Some(egui::pos2(sx, sy))
    }

    pub fn orbit_pixels(&mut self, delta_px: egui::Vec2) {
        let speed = 0.0065;
        match self.orbit_mode {
            OrbitMode::TurntableZUp => {
                self.yaw_rad -= (delta_px.x as f64) * speed;
                let limit = std::f64::consts::FRAC_PI_2 - 1e-4;
                self.pitch_rad =
                    (self.pitch_rad + (delta_px.y as f64) * speed).clamp(-limit, limit);
            }
            OrbitMode::FreeTrackball => {
                let (_, _, right, up) = self.basis();
                let q_yaw = chip_math::DQuat::from_axis_angle(up, -(delta_px.x as f64) * speed);
                let q_pitch =
                    chip_math::DQuat::from_axis_angle(right, -(delta_px.y as f64) * speed);
                self.trackball_rot = (q_yaw * q_pitch * self.trackball_rot).normalize();
            }
        }
    }

    pub fn pan_pixels(&mut self, delta_px: egui::Vec2, viewport_height_px: f32) {
        let mm_per_px = self.mm_per_pixel(viewport_height_px);
        let (_, _, right, up) = self.basis();
        self.target_mm -= right * (delta_px.x as f64 * mm_per_px);
        self.target_mm += up * (delta_px.y as f64 * mm_per_px);
    }

    /// Zooms smoothly around the world point under `cursor_pos`!
    pub fn zoom_at_cursor(
        &mut self,
        zoom_factor: f64,
        cursor_pos: Option<egui::Pos2>,
        viewport_rect: egui::Rect,
    ) {
        let factor = zoom_factor.clamp(0.2, 5.0);
        if (factor - 1.0).abs() < 1e-6 {
            return;
        }

        // Measure offset on target plane before zoom so cursor world point stays pinned
        let pre_anchor = cursor_pos.and_then(|p| self.point_on_target_plane(p, viewport_rect));

        match &mut self.projection {
            ProjectionMode::Orthographic { ortho_height_mm } => {
                *ortho_height_mm = (*ortho_height_mm / factor).clamp(0.5, 1e6);
            }
            ProjectionMode::Perspective { .. } => {
                self.distance_mm = (self.distance_mm / factor).clamp(1.0, 1e6);
            }
        }

        if let (Some(p_screen), Some(world_before)) = (cursor_pos, pre_anchor)
            && let Some(world_after) = self.point_on_target_plane(p_screen, viewport_rect)
        {
            self.target_mm += world_before - world_after;
        }
    }

    fn point_on_target_plane(
        &self,
        screen_pos: egui::Pos2,
        viewport_rect: egui::Rect,
    ) -> Option<DVec3> {
        let ray = self.screen_ray(screen_pos, viewport_rect)?;
        let (_, forward, _, _) = self.basis();
        let denom = ray.dir.dot(forward);
        if denom.abs() < 1e-9 {
            return None;
        }
        let t = (self.target_mm - ray.origin).dot(forward) / denom;
        Some(ray.at(t))
    }

    /// World-space millimeters per vertical pixel at the target plane.
    pub fn mm_per_pixel(&self, viewport_height_px: f32) -> f64 {
        let h_px = (viewport_height_px as f64).max(1.0);
        let world_h = match self.projection {
            ProjectionMode::Orthographic { ortho_height_mm } => ortho_height_mm,
            ProjectionMode::Perspective { fov_y_rad } => {
                2.0 * self.distance_mm * (0.5 * fov_y_rad).tan()
            }
        };
        world_h / h_px
    }

    /// Exact `f64` view and `[0, 1]` WebGPU depth projection matrices.
    pub fn view_proj_f64(&self, aspect: f64) -> (DMat4, DMat4) {
        let aspect = aspect.max(0.05);
        let (eye, forward, right, up) = self.basis();
        let view = DMat4::look_to_rh(eye, forward, up);
        let _ = right;

        let depth_span = (self.distance_mm + self.scene_radius_mm * 6.0).max(500.0);
        let proj = match self.projection {
            ProjectionMode::Orthographic { ortho_height_mm } => {
                let half_h = 0.5 * ortho_height_mm;
                let half_w = half_h * aspect;
                let near = (self.distance_mm - depth_span).min(-depth_span);
                let far = self.distance_mm + depth_span;
                DMat4::orthographic_rh(-half_w, half_w, -half_h, half_h, near, far)
            }
            ProjectionMode::Perspective { fov_y_rad } => {
                let near = (self.distance_mm * 0.01).clamp(0.1, 10.0);
                let far = (self.distance_mm + depth_span * 2.0).max(near + 100.0);
                DMat4::perspective_rh(fov_y_rad, aspect, near, far)
            }
        };
        (view, proj)
    }

    /// `f32` matrices for `wgpu` uniform upload: `(view_proj, inv_view_proj, eye_pos, view)`.
    pub fn gpu_matrices(&self, width_px: u32, height_px: u32) -> (Mat4, Mat4, Vec3, Mat4) {
        let aspect = (width_px.max(1) as f64) / (height_px.max(1) as f64);
        let (view_d, proj_d) = self.view_proj_f64(aspect);
        let vp_d = proj_d * view_d;
        let inv_vp_d = vp_d.inverse();
        let (eye_d, _, _, _) = self.basis();

        let vp = dmat4_to_mat4(vp_d);
        let inv_vp = dmat4_to_mat4(inv_vp_d);
        let view = dmat4_to_mat4(view_d);
        let eye = Vec3::new(eye_d.x as f32, eye_d.y as f32, eye_d.z as f32);
        (vp, inv_vp, eye, view)
    }

    /// Unprojects a 2D `egui` cursor coordinate inside `viewport_rect` into an exact `f64` [`Ray3`].
    pub fn screen_ray(&self, screen_pos: egui::Pos2, viewport_rect: egui::Rect) -> Option<Ray3> {
        if viewport_rect.width() < 2.0 || viewport_rect.height() < 2.0 {
            return None;
        }
        let ndc_x =
            ((screen_pos.x - viewport_rect.left()) / viewport_rect.width()) as f64 * 2.0 - 1.0;
        let ndc_y =
            1.0 - ((screen_pos.y - viewport_rect.top()) / viewport_rect.height()) as f64 * 2.0;

        let aspect = (viewport_rect.width() as f64) / (viewport_rect.height() as f64);
        let (view_d, proj_d) = self.view_proj_f64(aspect);
        let inv_vp = (proj_d * view_d).inverse();

        let near_h = inv_vp * DVec4::new(ndc_x, ndc_y, 0.0, 1.0);
        let far_h = inv_vp * DVec4::new(ndc_x, ndc_y, 1.0, 1.0);
        if near_h.w.abs() < 1e-12 || far_h.w.abs() < 1e-12 {
            return None;
        }
        let near_pt = near_h.truncate() / near_h.w;
        let far_pt = far_h.truncate() / far_h.w;

        match self.projection {
            ProjectionMode::Orthographic { .. } => {
                // Start ortho picking ray well in front of the scene along camera forward so clipped near planes never miss tall bosses
                let (_, forward, _, _) = self.basis();
                let mid = 0.5 * (near_pt + far_pt);
                let origin = mid - forward * (self.distance_mm + self.scene_radius_mm * 4.0);
                Ray3::new(origin, forward)
            }
            ProjectionMode::Perspective { .. } => {
                let (eye, _, _, _) = self.basis();
                Ray3::new(eye, far_pt - near_pt)
            }
        }
    }

    /// Applies mouse orbit, pan, and scroll-zoom interactions for the active [`MousePreset`].
    pub fn handle_viewport_input(&mut self, ui: &egui::Ui, response: &egui::Response) {
        let rect = response.rect;
        if !response.hovered() && !response.dragged() {
            return;
        }

        let (pointer_delta, middle_down, right_down, modifiers, scroll_y, hover_pos) =
            ui.input(|i| {
                (
                    i.pointer.delta(),
                    i.pointer.button_down(egui::PointerButton::Middle),
                    i.pointer.button_down(egui::PointerButton::Secondary),
                    i.modifiers,
                    i.smooth_scroll_delta.y,
                    i.pointer.hover_pos(),
                )
            });

        if pointer_delta.length_sq() > 0.0 && (middle_down || right_down) {
            let (do_orbit, do_pan) = match self.mouse_preset {
                MousePreset::Fusion360 => {
                    let orbit = (middle_down && modifiers.shift) || (right_down && modifiers.shift);
                    let pan = middle_down && !modifiers.shift;
                    (orbit, pan)
                }
                MousePreset::SolidWorks => {
                    let pan =
                        middle_down && (modifiers.ctrl || modifiers.command || modifiers.shift);
                    let orbit = (middle_down && !pan) || right_down;
                    (orbit, pan)
                }
                MousePreset::FreeCadBlender => {
                    let pan = (middle_down && modifiers.shift) || (right_down && modifiers.shift);
                    let orbit = middle_down && !modifiers.shift;
                    (orbit, pan)
                }
            };

            if do_orbit {
                self.orbit_pixels(pointer_delta);
            } else if do_pan {
                self.pan_pixels(pointer_delta, rect.height());
            }
        }

        if scroll_y.abs() > 0.1 && response.hovered() {
            let factor = (scroll_y as f64 * 0.0022).exp();
            self.zoom_at_cursor(factor, hover_pos, rect);
        }
    }
}

fn dmat4_to_mat4(m: DMat4) -> Mat4 {
    let cols = m.to_cols_array();
    let mut out = [0.0_f32; 16];
    for (dst, src) in out.iter_mut().zip(cols.iter()) {
        *dst = *src as f32;
    }
    Mat4::from_cols_array(&out)
}
