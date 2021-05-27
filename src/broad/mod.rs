//use std::sync::Mutex;

use crate::{body_sweep, inters::swept::{Body, BodySweptData}};


pub fn step(bodies: &mut [&mut Body], t: f64, offset_epsilon: f64) {

   for i in 0..(bodies.len() - 1) {
      let mut data = BodySweptData { b1_shape: usize::MAX, b2_shape: usize::MAX, travel: f64::MAX, norm: cgmath::vec2(0.0, 0.0) };
      let mut index = usize::MAX;
      for j in i..bodies.len() {
         if let Some(bsd) = body_sweep(bodies[i], bodies[j], t) {
            if bsd.travel < data.travel {
               data = bsd;
               index = j;
            }
         }
      }
      // call things
   }
}

