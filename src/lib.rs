//! Presolve convex 2D shape collision library.

pub mod narrow;
pub mod broad;

pub use narrow::{Circle, Aabb, Poly, Shape, Intersect};
pub use narrow::swept;
pub use narrow::swept::{Body, BodySweptData};

#[cfg(test)]
mod tests {
   
}