use cgmath::Vector2;
use super::inters::Shape;
use super::utils::*;

pub struct StaticBody {
   /// Posistion
   pub pos: Vector2<f64>,
   /// Compositing shapes
   pub shapes: Vec<Shape>,
}

impl StaticBody {
   pub fn new(shapes: Vec<Shape>) -> StaticBody {
      StaticBody { pos: v2f64_zero(), shapes }
   }
}

pub struct Body {
   /// Posistion
   pub pos: Vector2<f64>,
   /// Compositing shapes
   pub shapes: Vec<Shape>,
   /// Velocity
   pub vel: Vector2<f64>,
}

impl Body {
   pub fn new(shapes: Vec<Shape>) -> Body {
      Body { pos: v2f64_zero(), vel: v2f64_zero(), shapes }
   }
}