use cgmath::{ElementWise, InnerSpace, Vector2};

use crate::BodySweepData;

pub trait Reacter {
   /// Return the velocity to resume after a collision.
   fn react(&mut self, vel: Vector2<f64>, self_id: usize, other_id: usize, data: BodySweepData, epsilon: f64) -> Vector2<f64>;
}

/// React by maintaining a coefficient of the previous velocity, phasing through static colliders.
pub struct PhaseReacter {
   /// `0.0` = halt, `1.0` = maintain velocity, `-1.0` = reverse velocity, etc.
   coefficient: f64,
}
impl Reacter for PhaseReacter {
   fn react(&mut self, vel: Vector2<f64>, _: usize, _: usize, _: BodySweepData, _: f64) -> Vector2<f64> {
      vel.mul_element_wise(self.coefficient)
   }
}

/// React by reflecting a coefficient of the velocity across the normal of collision.
pub struct DeflectReacter {
   coefficient: f64,
}
impl Reacter for DeflectReacter {
   fn react(&mut self, vel: Vector2<f64>, _: usize, _: usize, data: BodySweepData, _: f64) -> Vector2<f64> {
      // i = r by normal: r=d−2(d⋅n)n
      (vel - data.norm.mul_element_wise(2.0 * vel.dot(data.norm))).mul_element_wise(self.coefficient)
   }
}


/// React by maintaining previous velocity in the direction perpendicular to the normal.
pub struct SlideReacter {
   norm_from_constricting: Vector2<f64>,
   coefficient: f64,
}
impl Reacter for SlideReacter {
   fn react(&mut self, vel: Vector2<f64>, _: usize, _: usize, data: BodySweepData, epsilon: f64) -> Vector2<f64> {
      // check if the collision is instantanious, accounting for epsilon adjustment
      let is_instant = vel.mul_element_wise(data.travel).dot(data.norm) > epsilon * -2.0;
      // Update to current norm of constriction
      let noc = self.norm_from_constricting;
      if is_instant {
         self.norm_from_constricting = noc;
      } else {
         self.norm_from_constricting = cgmath::vec2(0.0, 0.0);
      }
      // Stop if both this and the previous collisions constrict movement totally, or if velocity is reverse-parallel to the normal.
      if (is_instant && vel.dot(noc) > 0.0) || vel.normalize().dot(data.norm) <= 0.99 {
         self.norm_from_constricting = data.norm;
         return cgmath::vec2(0.0, 0.0)
      }
      let perp = cgmath::vec2(data.norm.y, -data.norm.x);
      perp.mul_element_wise(vel.dot(perp) * self.coefficient)
   }
}
