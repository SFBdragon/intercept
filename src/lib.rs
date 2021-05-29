//! Presolve convex 2D shape collision library.

pub mod inters;
pub mod broad;

pub use inters::{Circle, Aabb, Poly, Shape, Intersect};
pub use inters::swept;
pub use inters::swept::{Body, BodySweptData};

#[cfg(test)]
mod tests {
   
}