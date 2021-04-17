//! Presolve convex 2D shape collision library.

pub mod inters;
pub mod swept;

pub use inters::{Circle, Aabb, Poly, Shape, Intersect};
pub use swept::{Body, BodySweptData, body_sweep, shape_sweep};

#[cfg(test)]
mod tests {

}