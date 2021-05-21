use crate::{inters::swept::{Body, BodySweptData}, inters};
use cgmath::{ElementWise, InnerSpace, Vector2};

/// Describes the behaviour of `Body`s after collision.
#[derive(Debug, Clone, Copy)]
pub enum Response {
   Halt,
   Phase(f64),
   Deflect(f64),
   Slide(f64),
}


fn slide_response(b1: &mut Body, b2: &mut Body, data: &BodySweptData, offset_epsilon: f64) {
   // 'slide' along the normal of collision, maintaining velocity along the normal (speed is reduced)
   let epsilon = data.norm.mul_element_wise(offset_epsilon); // offset slightly away from the Body to avoid tunneling
   let perp_norm = cgmath::vec2(data.norm.y, -data.norm.x);
   let new_vel = perp_norm.mul_element_wise(b1.vel.dot(perp_norm));
   b1.pos = b1.vel * data.travel + new_vel * (1.0 - data.travel) + epsilon;
   b1.vel = new_vel;

   let new_vel = perp_norm.mul_element_wise(b2.vel.dot(perp_norm));
   b2.pos = b2.vel * data.travel + new_vel * (1.0 - data.travel) - epsilon;
   b2.vel = new_vel;
}

fn deflect_response(b1: &mut Body, b2: &mut Body, data: &BodySweptData, offset_epsilon: f64) {
   // i = r by normal: r=d−2(d⋅n)n
   let epsilon = data.norm.mul_element_wise(offset_epsilon); // offset slightly away from the Body to avoid tunneling
   let new_vel = b1.vel - data.norm.mul_element_wise(b1.vel.dot(data.norm) * 2.0);
   b1.pos = b1.vel * data.travel + new_vel * (1.0 - data.travel) + epsilon;
   b1.vel = new_vel;
   
   let new_vel = b2.vel - data.norm.mul_element_wise(b2.vel.dot(data.norm) * 2.0);
   b2.pos = b2.vel * data.travel + new_vel * (1.0 - data.travel) - epsilon;
   b2.vel = new_vel;
}


pub fn resolve(bodies: &mut [Body], t: f64, offset_epsilon: f64) { 

   // get a list of all bodies
   // check each body against every other potentially colliding body
      // find the soonest to collide body over the provided timestep
         // resolve the collision if any, if so, repeat the process until no collision takes place

   // this causes potential issues with 'ghost' collisions if done in the wrong order, and if the objects are going fast enough
}
