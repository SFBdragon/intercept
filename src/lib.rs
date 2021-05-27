//! Presolve convex 2D shape collision library.

pub mod inters;
pub mod broad;

pub use inters::{Circle, Aabb, Poly, Shape, Intersect};
pub use inters::swept::{Body, BodySweptData, body_sweep};

#[cfg(test)]
mod tests {
   
}