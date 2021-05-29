use crate::inters::swept::{Body, BodySweptData};
use cgmath::{ElementWise, InnerSpace, Vector2};

/// Describes the behaviour of `Body`s after collision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Response {
   Phase = 0,
   Deflect = 1,
   Slide = 2,
}

struct ResData {
   /// Whether to check other collisions against this `Body`. Will collide with others.
   pub collidable: bool,
   /// The coefficient of restitution for physics, or a multiplier for simple collision response.
   pub restitution: f64,
   /// Describes the behaviour of `Body`s that collide with this `Body`.
   pub response: Response,
}

pub struct Cosmos {
   table: Vec<(Body, ResData, String)>,
   pub gravity: Vector2<f64>,
   // ...
}


fn phase_response(b: &mut Body, resti: f64, data: &BodySweptData) {
   let new_vel = b.vel.mul_element_wise(resti);
   b.pos = b.pos + b.vel.mul_element_wise(data.travel) + new_vel.mul_element_wise(1.0 - data.travel);
   b.vel = new_vel;
}
fn slide_response(b: &mut Body, resti: f64, data: &BodySweptData) {
   // 'slide' along the normal of collision, maintaining velocity along the normal (speed is reduced)
   let perp_norm = cgmath::vec2(data.norm.y, -data.norm.x);
   let new_vel = perp_norm.mul_element_wise(b.vel.dot(perp_norm) * resti);
   b.pos = b.vel.mul_element_wise(data.travel) + new_vel * (1.0 - data.travel);
   b.vel = new_vel;
}
fn deflect_response(b: &mut Body, resti: f64, data: &BodySweptData) {
   // i = r by normal: r=d−2(d⋅n)n
   let new_vel = (b.vel - data.norm.mul_element_wise(b.vel.dot(data.norm) * 2.0)).mul_element_wise(resti);
   b.pos = b.vel.mul_element_wise(data.travel) + new_vel * (1.0 - data.travel);
   b.vel = new_vel;
}

fn response() {
   /*

   type ResponseFn = fn(&mut Body, f64, &BodySweptData);
   const RESPONSE_JUMP_TABLE: [ResponseFn; 3] = [phase_response, slide_response, deflect_response];

      // this causes potential issues with 'ghost' collisions if done in the wrong order, and if the objects are going fast enough
         // todo: check if this needs to be overhauled for the above reason
         let epsilon = data.norm.mul_element_wise(offset_epsilon); // offset slightly away from the Body to avoid tunneling
         RESPONSE_JUMP_TABLE[bodies[index].response as usize](bodies[i], bodies[index].restitution, &data);
         RESPONSE_JUMP_TABLE[bodies[i].response as usize](bodies[index], bodies[i].restitution, &data);
         bodies[i].pos = bodies[i].pos + epsilon;
         bodies[index].pos = bodies[index].pos - epsilon;
      } else {
         bodies[i].pos = bodies[i].pos + bodies[i].vel;
 */
}
