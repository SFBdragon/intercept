use cgmath::Vector2;

use crate::BodySweepData;

pub trait Reacter {
   /// Return the velocity to resume after a collision.
   fn react(&self, vel: Vector2<f64>, self_id: usize, other_id: usize, data: BodySweepData, epsilon: f64) -> Vector2<f64>;
}
/// React by maintaining previous velocity in the direction perpendicular to the normal.
pub struct SlideReacter {
   norm_from_constricting: Vector2<f64>,
   vel_of_constricting: Vector2<f64>,
}
impl Reacter for SlideReacter {
   fn react(&self, vel: Vector2<f64>, self_id: usize, other_id: usize, data: BodySweepData, epsilon: f64) -> Vector2<f64> {
      panic!()
   }
}



// todo: create halt responder, deflect responder, etc.


/* fn slide_response(b: &mut Body, resti: f64, data: &BodySweptData) {
   // 'slide' along the normal of collision, maintaining velocity along the normal (speed is reduced)
   let perp_norm = cgmath::vec2(data.norm.y, -data.norm.x);
   let new_vel = perp_norm.mul_element_wise(b.vel.dot(perp_norm) * resti);
   b.pos = b.vel.mul_element_wise(data.travel) + new_vel * (1.0 - data.travel); // use body.translate
   b.vel = new_vel;
}
fn deflect_response(b: &mut Body, resti: f64, data: &BodySweptData) {
   // i = r by normal: r=d−2(d⋅n)n
   let new_vel = (b.vel - data.norm.mul_element_wise(b.vel.dot(data.norm) * 2.0)).mul_element_wise(resti);
   b.pos = b.vel.mul_element_wise(data.travel) + new_vel * (1.0 - data.travel); // use body.translate
   b.vel = new_vel;
} */
