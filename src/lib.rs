//! Presolve convex 2D shape collision library.

pub mod broad;
pub mod narrow;

pub use narrow::swept;
pub use narrow::swept::{Body, BodySweepData};
pub use narrow::{Aabb, Circle, Intersect, Poly, Shape};

#[cfg(test)]
mod tests {}
