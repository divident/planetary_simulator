//! Two-body gravitational dynamics.
//!
//! Integration uses velocity Verlet, which is symplectic: unlike explicit Euler
//! or RK4 it does not secularly drain or pump orbital energy, so a closed orbit
//! stays closed for millions of steps. Energy oscillates within a bounded band
//! instead of drifting, which is what the HUD's drift readout shows.

use crate::vec2::{v2, V2};
use std::collections::VecDeque;

/// Maximum trail samples retained per body.
const TRAIL_CAP: usize = 3000;
/// A trail sample is recorded only after the body has moved this far, so the
/// trail length is independent of the frame rate and the time scale.
const TRAIL_MIN_STEP: f64 = 1.2;

#[derive(Clone)]
pub struct Body {
    pub pos: V2,
    pub vel: V2,
    pub acc: V2,
    pub mass: f64,
    /// Drawing radius in world units. Also used for the contact test.
    pub radius: f64,
    pub color: [f32; 3],
    pub name: &'static str,
    pub trail: VecDeque<V2>,
}

impl Body {
    pub fn new(pos: V2, vel: V2, mass: f64, color: [f32; 3], name: &'static str) -> Self {
        Body {
            pos,
            vel,
            acc: V2::ZERO,
            mass,
            // Constant-density scaling: radius grows as the cube root of mass.
            radius: 2.6 * mass.cbrt(),
            color,
            name,
            trail: VecDeque::new(),
        }
    }

    fn record_trail(&mut self) {
        let far_enough = match self.trail.back() {
            Some(&last) => (self.pos - last).len_sq() >= TRAIL_MIN_STEP * TRAIL_MIN_STEP,
            None => true,
        };
        if far_enough {
            self.trail.push_back(self.pos);
            while self.trail.len() > TRAIL_CAP {
                self.trail.pop_front();
            }
        }
    }

    pub fn clear_trail(&mut self) {
        self.trail.clear();
    }
}

pub struct System {
    pub bodies: [Body; 2],
    pub g: f64,
    /// Plummer softening length. Keeps the 1/r^2 force finite if the bodies
    /// pass through each other, which no fixed step size could otherwise survive.
    pub softening: f64,
    pub time: f64,
    pub steps: u64,
    /// Total energy at setup, the reference for the drift readout.
    pub e0: f64,
}

impl System {
    pub fn new(b0: Body, b1: Body, g: f64) -> Self {
        let mut s = System {
            bodies: [b0, b1],
            g,
            softening: 0.5,
            time: 0.0,
            steps: 0,
            e0: 0.0,
        };
        s.compute_accelerations();
        s.e0 = s.total_energy();
        s
    }

    /// Separation vector from body 0 to body 1.
    pub fn rel_pos(&self) -> V2 {
        self.bodies[1].pos - self.bodies[0].pos
    }

    pub fn rel_vel(&self) -> V2 {
        self.bodies[1].vel - self.bodies[0].vel
    }

    pub fn total_mass(&self) -> f64 {
        self.bodies[0].mass + self.bodies[1].mass
    }

    pub fn center_of_mass(&self) -> V2 {
        (self.bodies[0].pos * self.bodies[0].mass + self.bodies[1].pos * self.bodies[1].mass)
            / self.total_mass()
    }

    pub fn momentum(&self) -> V2 {
        self.bodies[0].vel * self.bodies[0].mass + self.bodies[1].vel * self.bodies[1].mass
    }

    pub fn kinetic_energy(&self) -> f64 {
        self.bodies
            .iter()
            .map(|b| 0.5 * b.mass * b.vel.len_sq())
            .sum()
    }

    pub fn potential_energy(&self) -> f64 {
        let r = (self.rel_pos().len_sq() + self.softening * self.softening).sqrt();
        -self.g * self.bodies[0].mass * self.bodies[1].mass / r
    }

    pub fn total_energy(&self) -> f64 {
        self.kinetic_energy() + self.potential_energy()
    }

    /// Energy error relative to setup, as a fraction. Meaningless if the initial
    /// energy is near zero (a parabolic orbit), so the caller should guard.
    pub fn energy_drift(&self) -> f64 {
        if self.e0.abs() < 1e-12 {
            0.0
        } else {
            (self.total_energy() - self.e0) / self.e0.abs()
        }
    }

    /// Angular momentum of the system about the center of mass.
    pub fn angular_momentum(&self) -> f64 {
        let com = self.center_of_mass();
        let vcom = self.momentum() / self.total_mass();
        self.bodies
            .iter()
            .map(|b| b.mass * (b.pos - com).cross(b.vel - vcom))
            .sum()
    }

    pub fn contact(&self) -> bool {
        self.rel_pos().len() < self.bodies[0].radius + self.bodies[1].radius
    }

    fn compute_accelerations(&mut self) {
        let d = self.rel_pos();
        // Plummer softening: |r| -> sqrt(r^2 + eps^2) in the denominator.
        let r2 = d.len_sq() + self.softening * self.softening;
        let inv_r3 = 1.0 / (r2 * r2.sqrt());
        let (m0, m1) = (self.bodies[0].mass, self.bodies[1].mass);
        self.bodies[0].acc = d * (self.g * m1 * inv_r3);
        self.bodies[1].acc = d * (-self.g * m0 * inv_r3);
    }

    /// One velocity Verlet step: drift on the old acceleration, recompute, then
    /// kick with the average of old and new.
    pub fn step(&mut self, dt: f64) {
        for b in &mut self.bodies {
            b.pos += b.vel * dt + b.acc * (0.5 * dt * dt);
        }
        let old = [self.bodies[0].acc, self.bodies[1].acc];
        self.compute_accelerations();
        for (i, b) in self.bodies.iter_mut().enumerate() {
            b.vel += (old[i] + b.acc) * (0.5 * dt);
        }
        self.time += dt;
        self.steps += 1;
    }

    /// Advance by `dt` and refresh the trails. Trails are sampled once per call
    /// rather than once per step so that fast-forwarding does not flood them.
    pub fn advance(&mut self, dt: f64) {
        self.step(dt);
        for b in &mut self.bodies {
            b.record_trail();
        }
    }

    /// Recenter so the center of mass sits at the origin with zero net momentum.
    /// Removes the uniform drift a hand-set velocity introduces.
    pub fn zero_momentum(&mut self) {
        let com = self.center_of_mass();
        let vcom = self.momentum() / self.total_mass();
        for b in &mut self.bodies {
            b.pos -= com;
            b.vel -= vcom;
            b.clear_trail();
        }
        self.compute_accelerations();
        self.e0 = self.total_energy();
    }

    /// Re-baseline the energy reference, e.g. after the user edits a velocity.
    pub fn rebase_energy(&mut self) {
        self.compute_accelerations();
        self.e0 = self.total_energy();
    }

    pub fn clear_trails(&mut self) {
        for b in &mut self.bodies {
            b.clear_trail();
        }
    }
}

/// Build a two-body pair on a Keplerian orbit, expressed in the center-of-mass
/// frame so the pair has no net drift.
///
/// The bodies start at apoapsis (or, for `e >= 1`, at the given separation) with
/// the relative velocity from the vis-viva equation `v^2 = mu (2/r - 1/a)`.
pub fn kepler_pair(
    g: f64,
    m0: f64,
    m1: f64,
    semi_major: f64,
    ecc: f64,
    c0: [f32; 3],
    c1: [f32; 3],
    n0: &'static str,
    n1: &'static str,
) -> (Body, Body) {
    let m = m0 + m1;
    let mu = g * m;
    let r = semi_major * (1.0 + ecc);
    let speed = (mu * (2.0 / r - 1.0 / semi_major)).max(0.0).sqrt();

    // Separation along +x, relative velocity perpendicular to it.
    let r_rel = v2(r, 0.0);
    let v_rel = v2(0.0, speed);

    // Split about the barycenter: each body's share is the *other* body's mass fraction.
    let b0 = Body::new(-r_rel * (m1 / m), -v_rel * (m1 / m), m0, c0, n0);
    let b1 = Body::new(r_rel * (m0 / m), v_rel * (m0 / m), m1, c1, n1);
    (b0, b1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orbit;

    const G: f64 = 200.0;

    fn run(sys: &mut System, duration: f64, dt: f64) {
        let n = (duration / dt).round() as u64;
        for _ in 0..n {
            sys.step(dt);
        }
    }

    #[test]
    fn circular_orbit_keeps_its_radius() {
        let (a, b) = kepler_pair(G, 1000.0, 8.0, 220.0, 0.0, [0.0; 3], [0.0; 3], "A", "B");
        let mut sys = System::new(a, b, G);
        sys.softening = 0.0;
        let r0 = sys.rel_pos().len();
        for _ in 0..200_000 {
            sys.step(0.004);
            let r = sys.rel_pos().len();
            assert!((r - r0).abs() / r0 < 1e-3, "radius wandered: {r} vs {r0}");
        }
    }

    #[test]
    fn verlet_does_not_drain_energy() {
        // An eccentric orbit is the hard case: the periapsis passage is where a
        // non-symplectic integrator bleeds energy fastest.
        let (a, b) = kepler_pair(G, 900.0, 60.0, 200.0, 0.8, [0.0; 3], [0.0; 3], "A", "B");
        let mut sys = System::new(a, b, G);
        run(&mut sys, 400.0, 0.002);
        assert!(
            sys.energy_drift().abs() < 1e-4,
            "energy drift {:e} after 400 s",
            sys.energy_drift()
        );
    }

    #[test]
    fn body_returns_to_start_after_one_period() {
        let (a, b) = kepler_pair(G, 900.0, 60.0, 220.0, 0.5, [0.0; 3], [0.0; 3], "A", "B");
        let mut sys = System::new(a, b, G);
        sys.softening = 0.0;
        let period = orbit::elements(&sys).period.expect("bound orbit");
        let start = sys.bodies[1].pos;
        run(&mut sys, period, 1e-4);
        let err = (sys.bodies[1].pos - start).len() / start.len();
        assert!(err < 1e-3, "closure error {err} after T = {period}");
    }

    #[test]
    fn barycenter_and_momentum_stay_put() {
        let (a, b) = kepler_pair(G, 400.0, 400.0, 200.0, 0.4, [0.0; 3], [0.0; 3], "A", "B");
        let mut sys = System::new(a, b, G);
        sys.zero_momentum();
        run(&mut sys, 300.0, 0.004);
        assert!(sys.momentum().len() < 1e-9, "momentum {:?}", sys.momentum());
        assert!(sys.center_of_mass().len() < 1e-6);
    }

    #[test]
    fn elements_recover_the_construction_parameters() {
        for &e in &[0.0, 0.3, 0.55, 0.9] {
            let (a, b) = kepler_pair(G, 900.0, 60.0, 210.0, e, [0.0; 3], [0.0; 3], "A", "B");
            let mut sys = System::new(a, b, G);
            sys.softening = 0.0;
            let el = orbit::elements(&sys);
            assert!((el.ecc - e).abs() < 1e-9, "ecc {} != {e}", el.ecc);
            assert!((el.semi_major - 210.0).abs() < 1e-6, "a = {}", el.semi_major);
            // Construction starts at apoapsis.
            assert!((el.separation - 210.0 * (1.0 + e)).abs() < 1e-6);
        }
    }

    #[test]
    fn elements_classify_an_unbound_orbit() {
        let star = Body::new(v2(0.0, 0.0), v2(0.0, 0.0), 1200.0, [0.0; 3], "Star");
        let rogue = Body::new(v2(-780.0, 250.0), v2(34.0, 0.0), 6.0, [0.0; 3], "Rogue");
        let sys = System::new(star, rogue, G);
        let el = orbit::elements(&sys);
        assert_eq!(el.shape, orbit::Shape::Hyperbolic);
        assert!(el.ecc > 1.0);
        assert!(el.period.is_none());
    }

    #[test]
    fn softening_survives_a_head_on_collapse() {
        // Zero angular momentum: the bodies pass through each other, which is a
        // singularity without softening.
        let a = Body::new(v2(-260.0, 0.0), v2(0.0, 0.0), 500.0, [0.0; 3], "A");
        let b = Body::new(v2(260.0, 0.0), v2(0.0, 0.0), 300.0, [0.0; 3], "B");
        let mut sys = System::new(a, b, G);
        run(&mut sys, 200.0, 0.001);
        assert!(sys.bodies.iter().all(|b| b.pos.len().is_finite()));
        assert!(sys.total_energy().is_finite());
    }
}
