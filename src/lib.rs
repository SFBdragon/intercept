//! Presolve convex 2D shape collision library.

pub mod inters;
pub mod swept;

pub use inters::{Circle, AABB, Poly, Shape, Intersect};
pub use swept::{Body, BodySweptData, body_body_swept, shape_shape_sweep};

#[cfg(test)]
mod tests {

}