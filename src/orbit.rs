//! Keplerian orbital elements derived from the instantaneous state vector.
//!
//! The two-body problem reduces to one body of reduced mass orbiting a fixed
//! center with gravitational parameter `mu = G(m0 + m1)`. Every element below
//! describes that *relative* orbit; each real body traces the same conic scaled
//! by the other body's mass fraction.

use crate::physics::System;
use crate::vec2::V2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    Elliptic,
    Parabolic,
    Hyperbolic,
}

impl Shape {
    pub fn label(self) -> &'static str {
        match self {
            Shape::Elliptic => "elliptic (bound)",
            Shape::Parabolic => "parabolic (marginal)",
            Shape::Hyperbolic => "hyperbolic (escape)",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Elements {
    /// Semi-major axis. Negative for a hyperbola.
    pub semi_major: f64,
    pub ecc: f64,
    /// Specific angular momentum of the relative orbit.
    pub ang_mom: f64,
    /// Angle of the periapsis direction, radians.
    pub arg_periapsis: f64,
    pub periapsis: f64,
    /// `None` unless the orbit is bound.
    pub apoapsis: Option<f64>,
    pub period: Option<f64>,
    pub separation: f64,
    pub shape: Shape,
}

pub fn elements(sys: &System) -> Elements {
    let mu = sys.g * sys.total_mass();
    let r_vec = sys.rel_pos();
    let v_vec = sys.rel_vel();
    let r = r_vec.len();
    let v2 = v_vec.len_sq();

    let energy = 0.5 * v2 - mu / r;
    let ang_mom = r_vec.cross(v_vec);

    // Laplace-Runge-Lenz eccentricity vector; points at periapsis.
    let e_vec = (r_vec * (v2 - mu / r) - v_vec * r_vec.dot(v_vec)) / mu;
    let ecc = e_vec.len();

    // Classify on energy, not eccentricity. A radial free-fall has e == 1 by
    // construction yet is plainly bound; its energy correctly reports it as a
    // degenerate ellipse with zero periapsis.
    let scale = mu / r;
    let shape = if energy.abs() < 1e-9 * scale {
        Shape::Parabolic
    } else if energy < 0.0 {
        Shape::Elliptic
    } else {
        Shape::Hyperbolic
    };

    let semi_major = if shape == Shape::Parabolic {
        f64::INFINITY
    } else {
        -mu / (2.0 * energy)
    };

    let periapsis = if shape == Shape::Parabolic {
        ang_mom * ang_mom / (2.0 * mu)
    } else {
        semi_major * (1.0 - ecc)
    };

    let (apoapsis, period) = if shape == Shape::Elliptic {
        (
            Some(semi_major * (1.0 + ecc)),
            Some(std::f64::consts::TAU * (semi_major.powi(3) / mu).sqrt()),
        )
    } else {
        (None, None)
    };

    Elements {
        semi_major,
        ecc,
        ang_mom,
        arg_periapsis: e_vec.y.atan2(e_vec.x),
        periapsis,
        apoapsis,
        period,
        separation: r,
        shape,
    }
}

/// Sample the predicted relative orbit as a polyline, in the frame where body 0
/// sits at the origin.
///
/// Uses the conic in polar form, `r = p / (1 + e cos v)`. For unbound orbits the
/// true anomaly is clamped short of the asymptote, where `r` diverges.
pub fn conic_path(el: &Elements, samples: usize) -> Vec<V2> {
    // Semi-latus rectum, valid for every conic including the parabola.
    let p = if el.shape == Shape::Parabolic {
        2.0 * el.periapsis
    } else {
        el.semi_major * (1.0 - el.ecc * el.ecc)
    };
    if !p.is_finite() || p <= 0.0 {
        return Vec::new();
    }

    // Orbits are traced in the direction of the angular momentum.
    let sign = if el.ang_mom >= 0.0 { 1.0 } else { -1.0 };

    let max_nu = if el.ecc < 1.0 {
        std::f64::consts::PI
    } else {
        // Asymptote at cos(nu) = -1/e; stop just inside it.
        (-1.0 / el.ecc).acos() * 0.985
    };

    let mut out = Vec::with_capacity(samples);
    for i in 0..samples {
        let t = i as f64 / (samples - 1) as f64;
        // Sweep symmetrically about periapsis so both branches are drawn.
        let nu = (t * 2.0 - 1.0) * max_nu;
        let denom = 1.0 + el.ecc * nu.cos();
        if denom <= 1e-6 {
            continue;
        }
        let r = p / denom;
        let ang = el.arg_periapsis + sign * nu;
        out.push(V2 {
            x: r * ang.cos(),
            y: r * ang.sin(),
        });
    }
    out
}
