//! Minimal f64 2D vector. The physics runs in double precision; macroquad's
//! `Vec2` is f32 and is only used at the drawing boundary.

use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct V2 {
    pub x: f64,
    pub y: f64,
}

pub const fn v2(x: f64, y: f64) -> V2 {
    V2 { x, y }
}

impl V2 {
    pub const ZERO: V2 = v2(0.0, 0.0);

    pub fn len_sq(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn len(self) -> f64 {
        self.len_sq().sqrt()
    }

    pub fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    /// z-component of the 3D cross product; the specific angular momentum in 2D.
    pub fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }

}

impl Add for V2 {
    type Output = V2;
    fn add(self, o: V2) -> V2 {
        v2(self.x + o.x, self.y + o.y)
    }
}

impl Sub for V2 {
    type Output = V2;
    fn sub(self, o: V2) -> V2 {
        v2(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f64> for V2 {
    type Output = V2;
    fn mul(self, s: f64) -> V2 {
        v2(self.x * s, self.y * s)
    }
}

impl Div<f64> for V2 {
    type Output = V2;
    fn div(self, s: f64) -> V2 {
        v2(self.x / s, self.y / s)
    }
}

impl Neg for V2 {
    type Output = V2;
    fn neg(self) -> V2 {
        v2(-self.x, -self.y)
    }
}

impl AddAssign for V2 {
    fn add_assign(&mut self, o: V2) {
        *self = *self + o;
    }
}

impl SubAssign for V2 {
    fn sub_assign(&mut self, o: V2) {
        *self = *self - o;
    }
}
