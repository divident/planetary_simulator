//! World-to-screen mapping, panning, zooming and frame tracking.
//!
//! World space is y-up and metric; screen space is y-down and in pixels.

use crate::orbit::{conic_path, Shape};
use crate::physics::System;
use crate::vec2::{v2, V2};

#[derive(Clone, Copy, PartialEq)]
pub enum Follow {
    Free,
    Barycenter,
    Body(usize),
}

impl Follow {
    pub fn label(self, sys: &System) -> &'static str {
        match self {
            Follow::Free => "free",
            Follow::Barycenter => "barycenter",
            Follow::Body(i) => sys.bodies[i].name,
        }
    }

    pub fn next(self) -> Follow {
        match self {
            Follow::Free => Follow::Barycenter,
            Follow::Barycenter => Follow::Body(0),
            Follow::Body(0) => Follow::Body(1),
            _ => Follow::Free,
        }
    }
}

pub struct Camera {
    pub center: V2,
    /// Screen pixels per world unit.
    pub zoom: f64,
    pub follow: Follow,
    /// Pixels along the left and right edges hidden behind HUD panels. `fit`
    /// centers within what is left so the orbit is not framed under a panel.
    pub inset: (f32, f32),
    screen: (f32, f32),
}

impl Camera {
    pub fn new() -> Self {
        Camera {
            center: V2::ZERO,
            zoom: 1.0,
            follow: Follow::Free,
            inset: (0.0, 0.0),
            screen: (1280.0, 800.0),
        }
    }

    pub fn set_screen(&mut self, w: f32, h: f32) {
        self.screen = (w, h);
    }

    pub fn world_to_screen(&self, p: V2) -> (f32, f32) {
        let d = (p - self.center) * self.zoom;
        (
            self.screen.0 * 0.5 + d.x as f32,
            self.screen.1 * 0.5 - d.y as f32,
        )
    }

    pub fn screen_to_world(&self, x: f32, y: f32) -> V2 {
        v2(
            (x - self.screen.0 * 0.5) as f64 / self.zoom + self.center.x,
            (self.screen.1 * 0.5 - y) as f64 / self.zoom + self.center.y,
        )
    }

    /// Scale world lengths (radii, vector lengths) into pixels.
    pub fn scale(&self, l: f64) -> f32 {
        (l * self.zoom) as f32
    }

    /// Zoom about a fixed screen point, so the world point under the cursor stays put.
    pub fn zoom_at(&mut self, factor: f64, sx: f32, sy: f32) {
        let before = self.screen_to_world(sx, sy);
        self.zoom = (self.zoom * factor).clamp(0.02, 60.0);
        let after = self.screen_to_world(sx, sy);
        self.center += before - after;
    }

    pub fn pan_pixels(&mut self, dx: f32, dy: f32) {
        self.center += v2(-dx as f64 / self.zoom, dy as f64 / self.zoom);
        self.follow = Follow::Free;
    }

    /// Ease the center toward the tracked target. Framerate-independent
    /// exponential smoothing keeps fast orbits from jittering the view.
    pub fn track(&mut self, sys: &System, frame_time: f32) {
        let target = match self.follow {
            Follow::Free => return,
            Follow::Barycenter => sys.center_of_mass(),
            Follow::Body(i) => sys.bodies[i].pos,
        };
        let k = 1.0 - (-12.0 * frame_time as f64).exp();
        self.center += (target - self.center) * k;
    }

    /// Frame the whole trajectory inside the un-occluded part of the window.
    ///
    /// For a bound orbit this bounds the predicted conic, which is what the view
    /// needs to hold for the entire period; the current separation alone would
    /// let a body swing off screen later. Unbound and degenerate radial orbits
    /// have no finite extent, so those fall back to the bodies themselves.
    pub fn fit(&mut self, sys: &System) {
        // A fitted view is a static one: leaving a tracking target engaged would
        // pull the camera straight back onto it and undo the framing.
        self.follow = Follow::Free;
        let el = crate::orbit::elements(sys);
        let com = sys.center_of_mass();
        let m = sys.total_mass();
        let shares = [-sys.bodies[1].mass / m, sys.bodies[0].mass / m];

        let mut pts: Vec<V2> = Vec::new();
        if el.shape == Shape::Elliptic {
            for p in conic_path(&el, 96) {
                for share in shares {
                    pts.push(com + p * share);
                }
            }
        }
        if pts.is_empty() {
            pts.extend(sys.bodies.iter().map(|b| b.pos));
        }

        let pad = sys.bodies[0].radius.max(sys.bodies[1].radius) + 30.0;
        let (mut lo, mut hi) = (pts[0], pts[0]);
        for p in &pts {
            lo = v2(lo.x.min(p.x), lo.y.min(p.y));
            hi = v2(hi.x.max(p.x), hi.y.max(p.y));
        }
        lo -= v2(pad, pad);
        hi += v2(pad, pad);

        let (bw, bh) = ((hi.x - lo.x).max(1e-6), (hi.y - lo.y).max(1e-6));
        let avail_w = (self.screen.0 - self.inset.0 - self.inset.1).max(120.0) as f64;
        let avail_h = self.screen.1 as f64 - 40.0;
        self.zoom = (avail_w / bw).min(avail_h / bh).clamp(0.02, 60.0);

        // Place the box center at the middle of the visible strip rather than
        // the middle of the window.
        let strip_center = (self.inset.0 + self.screen.0 - self.inset.1) * 0.5;
        let dx = (strip_center - self.screen.0 * 0.5) as f64 / self.zoom;
        self.center = (lo + hi) * 0.5 - v2(dx, 0.0);
    }
}
