
use super::{Cosmos, CollData};
use cgmath::{Vector2};

pub trait Responder {
   /// Return the velocity to resume after a collision. Collisions with triggers void this call.
   fn response(&self, cosmos: &mut Cosmos, vel: Vector2<f64>, self_id: usize, other_id: usize, data: CollData) -> Vector2<f64>;
   /// Execute logic immediately after a collision/trigger has occured.
   fn trigger(&self, cosmos: &mut Cosmos, self_id: usize, other_id: usize, data: CollData);
}


// todo: create slide responder, deflect responder, phase responder, physics responder, etc.


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
