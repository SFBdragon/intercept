//! Presolve convex 2D shape collision library.

pub mod narrow;
pub mod broad;


#[cfg(feature = "f64")]
type Vec2 = glam::DVec2;
#[cfg(feature = "f64")]
type Fp = f64;
#[cfg(not(feature = "f64"))]
type Vec2 = glam::Vec2;
#[cfg(not(feature = "f64"))]
type Fp = f32;


mod prelude {
    pub use self::{broad::*, narrow::*};

    pub mod narrow {
        pub use crate::narrow::{Intersect, Circle, Aabb, Poly, Shape};
        pub use crate::narrow::swept::{self, Body, BodySweepData};
    }

    pub mod broad {
        pub use crate::broad::{Plane, Collider, reacter::{Reacter, PhaseReacter, DeflectReacter, SlideReacter}};
    }
}

#[cfg(test)]
mod tests {
    //use super::*;
    //use super::prelude::*;

    
}