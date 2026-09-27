//! All drawing. Nothing here mutates simulation state.

use macroquad::prelude::*;

use crate::camera::Camera;
use crate::orbit::{conic_path, Elements};
use crate::physics::System;
use crate::vec2::V2;

pub const BG: Color = Color::new(0.031, 0.035, 0.058, 1.0);
const INK: Color = Color::new(0.90, 0.93, 1.00, 1.0);
const DIM: Color = Color::new(0.55, 0.60, 0.74, 1.0);
const ACCENT: Color = Color::new(0.45, 0.85, 1.00, 1.0);
const WARN: Color = Color::new(1.00, 0.52, 0.42, 1.0);
const PANEL: Color = Color::new(0.055, 0.065, 0.105, 0.88);
const PANEL_EDGE: Color = Color::new(0.20, 0.24, 0.36, 0.9);

/// Panel geometry, shared with the camera so `fit` knows what is occluded.
pub const PANEL_MARGIN: f32 = 16.0;
pub const PANEL_W: f32 = 268.0;
pub const HELP_W: f32 = 342.0;

fn rgb(c: [f32; 3], a: f32) -> Color {
    Color::new(c[0], c[1], c[2], a)
}

// ---------------------------------------------------------------- starfield

pub struct Stars {
    /// Normalized position, radius, brightness.
    pts: Vec<(f32, f32, f32, f32)>,
}

impl Stars {
    pub fn new(n: usize) -> Self {
        // Deterministic LCG so the sky is identical on every run.
        let mut s: u64 = 0x5eed_1337;
        let mut next = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 33) as f32) / (u32::MAX as f32 / 2.0)
        };
        let pts = (0..n)
            .map(|_| {
                let b: f32 = next();
                (next(), next(), 0.4 + b * 1.3, 0.10 + b * 0.55)
            })
            .collect();
        Stars { pts }
    }

    /// Drawn in screen space with a slow parallax offset, so panning feels like
    /// motion through a field rather than across a static texture.
    pub fn draw(&self, cam: &Camera) {
        let (w, h) = (screen_width(), screen_height());
        let ox = (-cam.center.x * cam.zoom * 0.04) as f32;
        let oy = (cam.center.y * cam.zoom * 0.04) as f32;
        for &(nx, ny, r, b) in &self.pts {
            let x = (nx * w + ox).rem_euclid(w);
            let y = (ny * h + oy).rem_euclid(h);
            draw_circle(x, y, r, Color::new(0.75, 0.82, 1.0, b));
        }
    }
}

// -------------------------------------------------------------------- scene

pub fn draw_trails(cam: &Camera, sys: &System) {
    for b in &sys.bodies {
        let n = b.trail.len();
        if n < 2 {
            continue;
        }
        let mut prev = cam.world_to_screen(b.trail[0]);
        for (i, &p) in b.trail.iter().enumerate().skip(1) {
            let cur = cam.world_to_screen(p);
            // Fade from transparent at the tail to solid at the head.
            let t = i as f32 / n as f32;
            draw_line(prev.0, prev.1, cur.0, cur.1, 1.0 + t * 1.4, rgb(b.color, t.powf(1.4) * 0.9));
            prev = cur;
        }
    }
}

/// The analytic orbit each body will follow if nothing changes: the relative
/// conic scaled by the other body's mass fraction, about the barycenter.
pub fn draw_predicted_orbits(cam: &Camera, sys: &System, el: &Elements) {
    let path = conic_path(el, 260);
    if path.len() < 2 {
        return;
    }
    let com = sys.center_of_mass();
    let m = sys.total_mass();
    let shares = [-sys.bodies[1].mass / m, sys.bodies[0].mass / m];

    for (i, share) in shares.iter().enumerate() {
        let col = rgb(sys.bodies[i].color, 0.22);
        let mut prev = cam.world_to_screen(com + path[0] * *share);
        for p in &path[1..] {
            let cur = cam.world_to_screen(com + *p * *share);
            draw_line(prev.0, prev.1, cur.0, cur.1, 1.0, col);
            prev = cur;
        }
    }
}

pub fn draw_barycenter(cam: &Camera, sys: &System) {
    let (x, y) = cam.world_to_screen(sys.center_of_mass());
    let c = Color::new(1.0, 1.0, 1.0, 0.5);
    draw_line(x - 7.0, y, x + 7.0, y, 1.0, c);
    draw_line(x, y - 7.0, x, y + 7.0, 1.0, c);
    draw_circle_lines(x, y, 4.0, 1.0, Color::new(1.0, 1.0, 1.0, 0.28));
}

pub fn draw_bodies(cam: &Camera, sys: &System, labels: bool) {
    for b in &sys.bodies {
        let (x, y) = cam.world_to_screen(b.pos);
        let r = cam.scale(b.radius).max(2.5);

        // Concentric discs, widest first. Many layers at a low alpha each sum
        // into a smooth falloff; a few at high alpha would show as hard rings.
        const HALO_LAYERS: i32 = 14;
        for i in (1..=HALO_LAYERS).rev() {
            let f = i as f32 / HALO_LAYERS as f32;
            draw_circle(x, y, r * (1.0 + f * 1.6), rgb(b.color, 0.028));
        }
        draw_circle(x, y, r, rgb(b.color, 1.0));
        // Off-center highlight reads as a lit sphere rather than a flat disc.
        draw_circle(x - r * 0.28, y - r * 0.28, r * 0.42, Color::new(1.0, 1.0, 1.0, 0.22));

        if labels {
            draw_text(b.name, x + r + 8.0, y - r - 4.0, 16.0, rgb(b.color, 0.85));
        }
    }
}

fn arrow(x: f32, y: f32, dx: f32, dy: f32, col: Color) {
    let len = (dx * dx + dy * dy).sqrt();
    if len < 3.0 {
        return;
    }
    let (ex, ey) = (x + dx, y + dy);
    draw_line(x, y, ex, ey, 1.6, col);
    let (ux, uy) = (dx / len, dy / len);
    let head = (len * 0.22).clamp(5.0, 12.0);
    // Two barbs, each the tip direction rotated by +/- 30 degrees.
    for s in [1.0f32, -1.0] {
        let (c, sn) = ((0.5236f32).cos(), (0.5236f32).sin() * s);
        let (bx, by) = (-(ux * c - uy * sn), -(ux * sn + uy * c));
        draw_line(ex, ey, ex + bx * head, ey + by * head, 1.6, col);
    }
}

pub fn draw_vectors(cam: &Camera, sys: &System) {
    // Velocities share one scale so the two arrows stay comparable; the scale is
    // normalized to the faster body so it is always visible but never huge.
    let vmax = sys.bodies.iter().map(|b| b.vel.len()).fold(1e-9, f64::max);
    let vscale = 70.0 / vmax;
    let amax = sys.bodies.iter().map(|b| b.acc.len()).fold(1e-9, f64::max);
    let ascale = 48.0 / amax;

    for b in &sys.bodies {
        let (x, y) = cam.world_to_screen(b.pos);
        let v = b.vel * vscale;
        arrow(x, y, v.x as f32, -v.y as f32, rgb(b.color, 0.95));
        let a = b.acc * ascale;
        arrow(x, y, a.x as f32, -a.y as f32, Color::new(1.0, 0.55, 0.75, 0.8));
    }
}

/// Faint line joining the two bodies, annotated with the separation.
pub fn draw_separation(cam: &Camera, sys: &System) {
    let a = cam.world_to_screen(sys.bodies[0].pos);
    let b = cam.world_to_screen(sys.bodies[1].pos);
    draw_line(a.0, a.1, b.0, b.1, 1.0, Color::new(1.0, 1.0, 1.0, 0.13));
    let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
    let txt = format!("r = {:.1}", sys.rel_pos().len());
    draw_text(&txt, mid.0 + 6.0, mid.1 - 6.0, 15.0, Color::new(1.0, 1.0, 1.0, 0.30));
}

/// Arrow marking the drag-to-set-velocity gesture in progress.
pub fn draw_drag(cam: &Camera, sys: &System, idx: usize, from: V2, to: V2) {
    let a = cam.world_to_screen(from);
    let b = cam.world_to_screen(to);
    arrow(a.0, a.1, b.0 - a.0, b.1 - a.1, Color::new(1.0, 1.0, 1.0, 0.85));
    let speed = (to - from).len() * crate::VEL_DRAG_SCALE;
    let txt = format!("{} -> v = {:.1}", sys.bodies[idx].name, speed);
    draw_text(&txt, b.0 + 8.0, b.1, 16.0, INK);
}

// ---------------------------------------------------------------------- HUD

fn panel(x: f32, y: f32, w: f32, h: f32) {
    draw_rectangle(x, y, w, h, PANEL);
    draw_rectangle_lines(x, y, w, h, 1.0, PANEL_EDGE);
}

/// A left-aligned key with a right-aligned value, the standard HUD row.
fn row(x: f32, y: f32, w: f32, key: &str, val: &str, col: Color) {
    draw_text(key, x, y, 16.0, DIM);
    let m = measure_text(val, None, 16, 1.0);
    draw_text(val, x + w - m.width, y, 16.0, col);
}

fn section(x: f32, y: f32, title: &str) {
    draw_text(title, x, y, 14.0, Color::new(0.42, 0.47, 0.62, 1.0));
}

pub struct HudState<'a> {
    pub sys: &'a System,
    pub el: &'a Elements,
    pub cam: &'a Camera,
    pub preset: &'a str,
    pub blurb: &'a str,
    pub paused: bool,
    pub time_scale: f64,
    pub dt: f64,
    pub substeps: usize,
    pub fps: i32,
}

pub fn draw_hud(s: &HudState) {
    let (x, w) = (PANEL_MARGIN, PANEL_W);
    let pad = 14.0;
    let h = 372.0;
    panel(x, 16.0, w, h);

    let cx = x + pad;
    let cw = w - pad * 2.0;
    let mut y = 42.0;

    draw_text(s.preset, cx, y, 20.0, INK);
    y += 18.0;
    draw_text(s.blurb, cx, y, 14.0, DIM);
    y += 26.0;

    section(cx, y, "STATE");
    y += 18.0;
    let status = if s.paused { "paused" } else { "running" };
    row(cx, y, cw, "status", status, if s.paused { WARN } else { ACCENT });
    y += 19.0;
    row(cx, y, cw, "sim time", &format!("{:.2} s", s.sys.time), INK);
    y += 19.0;
    row(cx, y, cw, "time scale", &format!("{:.2}x", s.time_scale), INK);
    y += 19.0;
    row(cx, y, cw, "step dt", &format!("{:.4} s", s.dt), INK);
    y += 19.0;
    row(cx, y, cw, "steps/frame", &format!("{}", s.substeps), INK);
    y += 26.0;

    section(cx, y, "ORBIT");
    y += 18.0;
    row(cx, y, cw, "type", s.el.shape.label(), ACCENT);
    y += 19.0;
    row(cx, y, cw, "eccentricity", &format!("{:.4}", s.el.ecc), INK);
    y += 19.0;
    let a = if s.el.semi_major.is_finite() {
        format!("{:.1}", s.el.semi_major)
    } else {
        "inf".to_string()
    };
    row(cx, y, cw, "semi-major a", &a, INK);
    y += 19.0;
    row(cx, y, cw, "separation r", &format!("{:.1}", s.el.separation), INK);
    y += 19.0;
    row(cx, y, cw, "periapsis", &format!("{:.1}", s.el.periapsis), INK);
    y += 19.0;
    match s.el.apoapsis {
        Some(ap) => row(cx, y, cw, "apoapsis", &format!("{:.1}", ap), INK),
        None => row(cx, y, cw, "apoapsis", "-", DIM),
    }
    y += 19.0;
    match s.el.period {
        Some(p) => row(cx, y, cw, "period T", &format!("{:.2} s", p), INK),
        None => row(cx, y, cw, "period T", "-", DIM),
    }
    y += 26.0;

    section(cx, y, "CONSERVED");
    y += 18.0;
    row(cx, y, cw, "energy E", &format!("{:.4e}", s.sys.total_energy()), INK);
    y += 19.0;
    let drift = s.sys.energy_drift();
    // Verlet keeps this bounded and tiny; a growing value means dt is too large.
    let dcol = if drift.abs() > 1e-3 { WARN } else { ACCENT };
    row(cx, y, cw, "drift", &format!("{:+.3e}", drift), dcol);
    y += 19.0;
    row(cx, y, cw, "ang. momentum", &format!("{:.4e}", s.sys.angular_momentum()), INK);

    if s.sys.contact() {
        let m = measure_text("BODIES OVERLAPPING", None, 16, 1.0);
        draw_text(
            "BODIES OVERLAPPING",
            (screen_width() - m.width) * 0.5,
            34.0,
            16.0,
            WARN,
        );
    }

    draw_scale_bar(s.cam);
    draw_status_line(s);
}

fn draw_scale_bar(cam: &Camera) {
    // Snap the bar to a 1/2/5 x 10^n world length near 130 px.
    let target = 130.0 / cam.zoom;
    let mag = 10f64.powf(target.log10().floor());
    let unit = [1.0, 2.0, 5.0, 10.0]
        .iter()
        .map(|m| m * mag)
        .min_by(|a, b| (a - target).abs().partial_cmp(&(b - target).abs()).unwrap())
        .unwrap();
    let px = (unit * cam.zoom) as f32;

    let (x, y) = (screen_width() - px - 26.0, screen_height() - 34.0);
    draw_line(x, y, x + px, y, 1.5, DIM);
    draw_line(x, y - 4.0, x, y + 4.0, 1.5, DIM);
    draw_line(x + px, y - 4.0, x + px, y + 4.0, 1.5, DIM);
    let txt = format!("{} units", unit);
    let m = measure_text(&txt, None, 14, 1.0);
    draw_text(&txt, x + (px - m.width) * 0.5, y - 9.0, 14.0, DIM);
}

fn draw_status_line(s: &HudState) {
    let txt = format!(
        "{} fps   |   zoom {:.2}x   |   camera: {}   |   H for help",
        s.fps,
        s.cam.zoom,
        s.cam.follow.label(s.sys)
    );
    draw_text(&txt, 18.0, screen_height() - 16.0, 15.0, DIM);
}

const HELP: &[(&str, &str)] = &[
    ("Space", "pause / resume"),
    ("N", "single step while paused"),
    ("R", "reset current preset"),
    ("1 - 6", "load preset"),
    (",  .", "time scale down / up"),
    ("[  ]", "integrator dt down / up"),
    ("Wheel", "zoom at cursor"),
    ("Drag LMB", "pan the view"),
    ("Drag RMB", "pull a body to set its velocity"),
    ("C", "cycle camera target"),
    ("F", "fit both bodies in view"),
    ("T", "trails"),
    ("V", "velocity and acceleration vectors"),
    ("O", "predicted orbit paths"),
    ("B", "barycenter and separation"),
    ("L", "body labels"),
    ("H", "this panel"),
];

pub fn draw_help() {
    let (w, h) = (HELP_W, HELP.len() as f32 * 21.0 + 60.0);
    let x = screen_width() - w - PANEL_MARGIN;
    let y = PANEL_MARGIN;
    panel(x, y, w, h);
    draw_text("CONTROLS", x + 14.0, y + 30.0, 16.0, INK);
    let mut ty = y + 56.0;
    for (k, d) in HELP {
        draw_text(k, x + 14.0, ty, 15.0, ACCENT);
        draw_text(d, x + 108.0, ty, 15.0, DIM);
        ty += 21.0;
    }
}
