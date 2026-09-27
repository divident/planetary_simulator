//! Starting configurations.
//!
//! Units are arbitrary but chosen so that a typical orbit spans a few hundred
//! world units and completes in a few tens of simulated seconds at `G = 200`.

use crate::physics::{kepler_pair, Body, System};
use crate::vec2::v2;

pub const G: f64 = 200.0;

const AMBER: [f32; 3] = [1.00, 0.78, 0.29];
const CYAN: [f32; 3] = [0.42, 0.82, 1.00];
const CORAL: [f32; 3] = [1.00, 0.45, 0.42];
const VIOLET: [f32; 3] = [0.70, 0.55, 1.00];
const MINT: [f32; 3] = [0.45, 0.95, 0.72];

pub struct Preset {
    pub name: &'static str,
    pub blurb: &'static str,
    build: fn() -> System,
}

impl Preset {
    pub fn build(&self) -> System {
        let mut s = (self.build)();
        s.zero_momentum();
        s
    }
}

pub const PRESETS: &[Preset] = &[
    Preset {
        name: "Circular orbit",
        blurb: "e = 0, light satellite",
        build: || {
            let (a, b) = kepler_pair(G, 1000.0, 8.0, 220.0, 0.0, AMBER, CYAN, "Star", "Planet");
            System::new(a, b, G)
        },
    },
    Preset {
        name: "Elliptical orbit",
        blurb: "e = 0.55, visible barycenter wobble",
        build: || {
            let (a, b) = kepler_pair(G, 900.0, 60.0, 200.0, 0.55, AMBER, CORAL, "Star", "Planet");
            System::new(a, b, G)
        },
    },
    Preset {
        name: "Comet",
        blurb: "e = 0.9, fast periapsis passage",
        build: || {
            let (a, b) = kepler_pair(G, 1400.0, 4.0, 260.0, 0.9, AMBER, MINT, "Star", "Comet");
            System::new(a, b, G)
        },
    },
    Preset {
        name: "Equal-mass binary",
        blurb: "identical masses about a shared barycenter",
        build: || {
            let (a, b) = kepler_pair(G, 400.0, 400.0, 200.0, 0.4, VIOLET, CYAN, "Alpha", "Beta");
            System::new(a, b, G)
        },
    },
    Preset {
        name: "Hyperbolic flyby",
        blurb: "unbound, deflected once and gone",
        build: || {
            let star = Body::new(v2(0.0, 0.0), v2(0.0, 0.0), 1200.0, AMBER, "Star");
            // Fast enough at this range to exceed escape velocity, offset in y
            // to give the encounter a non-zero impact parameter.
            let rogue = Body::new(v2(-780.0, 250.0), v2(34.0, 0.0), 6.0, CORAL, "Rogue");
            System::new(star, rogue, G)
        },
    },
    Preset {
        name: "Radial infall",
        blurb: "zero angular momentum, straight-line collapse",
        build: || {
            let a = Body::new(v2(-260.0, 0.0), v2(0.0, 0.0), 500.0, VIOLET, "Alpha");
            let b = Body::new(v2(260.0, 0.0), v2(0.0, 0.0), 300.0, MINT, "Beta");
            System::new(a, b, G)
        },
    },
];
