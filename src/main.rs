//! Two-body gravitational simulation with real-time visualization.
//!
//! Physics runs in f64 on a fixed timestep decoupled from the frame rate, so the
//! trajectory is identical whether the window renders at 30 or 240 fps.

mod camera;
mod orbit;
mod physics;
mod presets;
mod render;
mod vec2;

use macroquad::prelude::*;

use camera::Camera;
use physics::System;
use presets::PRESETS;
use render::{HudState, Stars};
use vec2::V2;

/// World units of drag per unit of resulting velocity, for the right-drag gesture.
pub const VEL_DRAG_SCALE: f64 = 0.15;

/// Default integrator step. Small enough that even the eccentric presets keep
/// energy drift near the f64 noise floor.
const DEFAULT_DT: f64 = 0.004;
/// Ceiling on physics steps per frame, so a huge time scale degrades into
/// slow motion rather than freezing the window.
const MAX_SUBSTEPS: usize = 6000;

/// Command line: `--capture <path>` renders a few seconds, writes a PNG and
/// exits; `--preset <n>` picks the starting configuration.
struct Args {
    capture: Option<String>,
    preset: usize,
}

fn parse_args() -> Args {
    let mut a = Args { capture: None, preset: 0 };
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--capture" if i + 1 < argv.len() => {
                a.capture = Some(argv[i + 1].clone());
                i += 1;
            }
            "--preset" if i + 1 < argv.len() => {
                a.preset = argv[i + 1].parse::<usize>().unwrap_or(1).saturating_sub(1);
                i += 1;
            }
            _ => {}
        }
        i += 1;
    }
    a.preset = a.preset.min(PRESETS.len() - 1);
    a
}

struct App {
    sys: System,
    cam: Camera,
    stars: Stars,
    preset: usize,
    paused: bool,
    time_scale: f64,
    dt: f64,
    accumulator: f64,
    substeps: usize,
    /// Right-drag in progress: (body index, cursor position in world space).
    drag: Option<(usize, V2)>,
    last_mouse: Vec2,
    show_trails: bool,
    show_vectors: bool,
    show_orbits: bool,
    show_barycenter: bool,
    show_labels: bool,
    show_help: bool,
}

impl App {
    fn new() -> Self {
        let mut cam = Camera::new();
        cam.set_screen(screen_width(), screen_height());
        let sys = PRESETS[0].build();
        cam.fit(&sys);
        App {
            sys,
            cam,
            stars: Stars::new(420),
            preset: 0,
            paused: false,
            time_scale: 5.0,
            dt: DEFAULT_DT,
            accumulator: 0.0,
            substeps: 0,
            drag: None,
            last_mouse: Vec2::ZERO,
            show_trails: true,
            show_vectors: false,
            show_orbits: true,
            show_barycenter: true,
            show_labels: true,
            show_help: true,
        }
    }

    fn load(&mut self, idx: usize) {
        self.preset = idx;
        self.sys = PRESETS[idx].build();
        self.accumulator = 0.0;
        self.sync_insets();
        self.cam.fit(&self.sys);
    }

    /// Tell the camera which screen edges the HUD covers.
    fn sync_insets(&mut self) {
        self.cam.set_screen(screen_width(), screen_height());
        let left = render::PANEL_MARGIN * 2.0 + render::PANEL_W;
        let right = if self.show_help {
            render::PANEL_MARGIN * 2.0 + render::HELP_W
        } else {
            render::PANEL_MARGIN * 2.0
        };
        self.cam.inset = (left, right);
    }

    /// Index of the body under a screen point, if any. The hit radius has a
    /// pixel floor so small bodies stay grabbable when zoomed out.
    fn pick(&self, mx: f32, my: f32) -> Option<usize> {
        self.sys
            .bodies
            .iter()
            .enumerate()
            .filter_map(|(i, b)| {
                let (x, y) = self.cam.world_to_screen(b.pos);
                let d = ((x - mx).powi(2) + (y - my).powi(2)).sqrt();
                let hit = self.cam.scale(b.radius).max(14.0);
                (d <= hit).then_some((i, d))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(i, _)| i)
    }

    fn input(&mut self) {
        let (mx, my) = mouse_position();
        let mouse = vec2(mx, my);

        for (i, key) in [
            KeyCode::Key1,
            KeyCode::Key2,
            KeyCode::Key3,
            KeyCode::Key4,
            KeyCode::Key5,
            KeyCode::Key6,
        ]
        .iter()
        .enumerate()
        {
            if i < PRESETS.len() && is_key_pressed(*key) {
                self.load(i);
            }
        }

        if is_key_pressed(KeyCode::R) {
            let p = self.preset;
            self.load(p);
        }
        if is_key_pressed(KeyCode::Space) {
            self.paused = !self.paused;
        }
        if is_key_pressed(KeyCode::N) && self.paused {
            self.sys.advance(self.dt);
        }
        if is_key_pressed(KeyCode::Period) {
            self.time_scale = (self.time_scale * 1.25).min(64.0);
        }
        if is_key_pressed(KeyCode::Comma) {
            self.time_scale = (self.time_scale / 1.25).max(0.05);
        }
        if is_key_pressed(KeyCode::RightBracket) {
            self.dt = (self.dt * 1.5).min(0.1);
        }
        if is_key_pressed(KeyCode::LeftBracket) {
            self.dt = (self.dt / 1.5).max(1e-5);
        }
        if is_key_pressed(KeyCode::C) {
            self.cam.follow = self.cam.follow.next();
        }
        if is_key_pressed(KeyCode::F) {
            self.sync_insets();
            self.cam.fit(&self.sys);
        }
        if is_key_pressed(KeyCode::T) {
            self.show_trails = !self.show_trails;
            if !self.show_trails {
                self.sys.clear_trails();
            }
        }
        if is_key_pressed(KeyCode::V) {
            self.show_vectors = !self.show_vectors;
        }
        if is_key_pressed(KeyCode::O) {
            self.show_orbits = !self.show_orbits;
        }
        if is_key_pressed(KeyCode::B) {
            self.show_barycenter = !self.show_barycenter;
        }
        if is_key_pressed(KeyCode::L) {
            self.show_labels = !self.show_labels;
        }
        if is_key_pressed(KeyCode::H) {
            self.show_help = !self.show_help;
        }

        let (_, wheel) = mouse_wheel();
        if wheel != 0.0 {
            self.cam.zoom_at(1.14f64.powf(wheel.signum() as f64), mx, my);
        }

        // Left drag pans; the first frame of the press only seeds the anchor.
        if is_mouse_button_down(MouseButton::Left) && !is_mouse_button_pressed(MouseButton::Left) {
            let d = mouse - self.last_mouse;
            if d.length_squared() > 0.0 {
                self.cam.pan_pixels(d.x, d.y);
            }
        }

        // Right drag from a body sets that body's velocity on release.
        if is_mouse_button_pressed(MouseButton::Right) {
            if let Some(i) = self.pick(mx, my) {
                self.drag = Some((i, self.cam.screen_to_world(mx, my)));
            }
        }
        if let Some((i, _)) = self.drag {
            if is_mouse_button_down(MouseButton::Right) {
                self.drag = Some((i, self.cam.screen_to_world(mx, my)));
            } else {
                let (_, to) = self.drag.take().unwrap();
                let from = self.sys.bodies[i].pos;
                self.sys.bodies[i].vel = (to - from) * VEL_DRAG_SCALE;
                self.sys.clear_trails();
                self.sys.rebase_energy();
            }
        }

        self.last_mouse = mouse;
    }

    fn update(&mut self, frame_time: f32) {
        self.substeps = 0;
        if !self.paused {
            // Clamp the frame delta so a stall (window drag, first frame) cannot
            // queue up an unbounded backlog of physics steps.
            self.accumulator += (frame_time.min(0.05) as f64) * self.time_scale;
            while self.accumulator >= self.dt && self.substeps < MAX_SUBSTEPS {
                self.sys.advance(self.dt);
                self.accumulator -= self.dt;
                self.substeps += 1;
            }
            if self.substeps == MAX_SUBSTEPS {
                self.accumulator = 0.0; // drop the backlog rather than lag behind
            }
        }
        self.sync_insets();
        self.cam.track(&self.sys, frame_time);
    }

    fn draw(&self) {
        clear_background(render::BG);
        self.stars.draw(&self.cam);

        let el = orbit::elements(&self.sys);

        if self.show_orbits {
            render::draw_predicted_orbits(&self.cam, &self.sys, &el);
        }
        if self.show_trails {
            render::draw_trails(&self.cam, &self.sys);
        }
        if self.show_barycenter {
            render::draw_separation(&self.cam, &self.sys);
            render::draw_barycenter(&self.cam, &self.sys);
        }
        render::draw_bodies(&self.cam, &self.sys, self.show_labels);
        if self.show_vectors {
            render::draw_vectors(&self.cam, &self.sys);
        }
        if let Some((i, to)) = self.drag {
            render::draw_drag(&self.cam, &self.sys, i, self.sys.bodies[i].pos, to);
        }

        render::draw_hud(&HudState {
            sys: &self.sys,
            el: &el,
            cam: &self.cam,
            preset: PRESETS[self.preset].name,
            blurb: PRESETS[self.preset].blurb,
            paused: self.paused,
            time_scale: self.time_scale,
            dt: self.dt,
            substeps: self.substeps,
            fps: get_fps(),
        });
        if self.show_help {
            render::draw_help();
        }
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Two-Body Gravity".to_owned(),
        window_width: 1280,
        window_height: 800,
        high_dpi: true,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let args = parse_args();
    let mut app = App::new();
    if args.preset != 0 {
        app.load(args.preset);
    }
    let mut frame: u32 = 0;

    loop {
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }
        app.input();
        app.update(get_frame_time());
        app.draw();

        // The framebuffer must be read before the swap, while this frame's
        // draw calls are still the front of the pipeline.
        if is_key_pressed(KeyCode::P) {
            save_png(&format!("two-body-{frame}.png"));
        }
        // Give the sim time to draw a trail before grabbing the framebuffer.
        if let Some(path) = &args.capture {
            if frame >= 900 {
                save_png(path);
                break;
            }
        }

        next_frame().await;
        frame += 1;
    }
}

fn save_png(path: &str) {
    let img = get_screen_data();
    img.export_png(path);
    println!("wrote {path} ({}x{})", img.width(), img.height());
}
