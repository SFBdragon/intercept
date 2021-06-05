pub mod reacter;

use std::collections::HashMap;

use crate::{broad::reacter::Reacter, narrow::swept::{Body, BodySweepData}};
use fnv::{FnvBuildHasher, FnvHashMap, FnvHashSet};
use indexmap::IndexMap;
use cgmath::ElementWise;


pub enum ColliderType {
   /// Boolean indicates whether reacters should have this static Collider's velocity added to theirs as a result of the collision.
   Static(bool),
   Reacter(Box<dyn Reacter>),
}

/// Colliders are units managed by a Plane. Colliders can be classified into two types: static, and reacting. Defined by whether `reacter.is_some() == true`. 
/// Changes in position are managed by the Plane, changes in velocity are managed by reactions/user. 
/// Mutating body's position is allowed, but may cause tunneling, as pre-solving is solely used, create Reacters with care.
/// This layout of responsibility is designed to be ideal for games such as platformers that usually would implement custom behaviours for all moving entities, instead of physical.
/// If you would prefer, a physically based simulation broadphase is implemented under *crate::broad::phys::Cosmos*. <br>
/// *Static-Static*   : No behaviour <br>
/// *Static-Reacting* : Reacter `.reacter.react(..)`, Static `.trigger(..)` <br>
/// *Reacting-Reacting* : Reacter `.trigger(..)`, Reacter `.trigger(..)`
pub struct Collider {
   pub body: Body,
   broad_y: (f64, f64),

   /// The Reacter handles the collision of this Collider and one that does not react.
   pub reacter: ColliderType,
   /// The trigger is invoked for reacters against reacters, and for static colliders against reacters only.
   pub trigger: Box<dyn Fn(usize, usize, BodySweepData, f64)>,
}



pub struct Plane { // todo: document
   pub id_counter: usize,
   pub table: FnvHashMap<usize, Collider>,

   adds: Option<usize>, // id of first of add candidates
   rems: FnvHashSet<usize>, // ids of all removal candidates

   pub is_be_updated: bool,
   pub is_be_sorted: bool,

   // the x axis is chosen to be the discrimination axis due to (handling a row of columns most efficiently): 
   //    1. it is the most 'primitave' axis
   //    2. screens/viewports are usually wider than they are tall
   //    3. game worlds are usually wider than they are tall
   //    4. Character/humanoid hitboxes are usually taller than they are wide, and next to each other
   pub be_list: Vec<(bool, usize, f64)>,
}

impl Plane {

   pub fn get_be_list_indecies(&self, id: usize) -> (usize, usize) {
      //! Binary searches be_list for the b & e values belonging to the id provided. Returns indecies of `(b, e)`. Ensure be_list is sorted.
      panic!() // todo: implement
   }
   pub fn get_be_list_position(&self, x: f64) -> usize {
      //! Binary searches be_list for the index of- or the index after the given x value. Ensure be_list is sorted.
      panic!() // todo: implement
   }


   pub fn update(&mut self) { // todo: possibly merge with sort()
      //! Updates be_list's b & e values. Update `Body`s prior. Does not process adds and removals; use incorporate() instead for that.
      for v in self.be_list.iter_mut() {
         if let Some(c) = self.table.get(&v.1) {
            match v.0 {
               false => v.2 = f64::min(c.body.aabb.min.x, c.body.aabb.min.x + c.body.vel.x), // b value
               true => v.2 = f64::max(c.body.aabb.max.x, c.body.aabb.max.x + c.body.vel.x), // e value
            }
         }
      }
      self.is_be_updated = true;
   }

   pub fn sort(&mut self) {
      // todo: should sorting unupdated be allowed?
      if !self.is_be_updated { println!("WARNING: Cosmos::sort(&mut self) called on a potentially out-of-date be_list.") }

      // sort b & e values
      // insertion sort due to its adaptivity & efficiency: sort is expected to be mostly sorted due to temporal cohesion
      let len = self.be_list.len();
      insertion_be(&mut self.be_list, 0, len - 1);
      self.is_be_sorted = true;
   }

   pub fn incorporate(&mut self) {
      //! Merges/Culls Collider additions and removals for be_list if/as necessary. Updates and sorts the list if not already done.

      // Ensure be_list merges validly
      if self.is_be_updated { self.update(); }
      if self.is_be_sorted { self.sort(); }

      // Create list for added b & e values. This may be a candidate for parallelization?
      if let Some(id) = self.adds {
         // create new b & e values
         let add_len = (self.id_counter - id) * 2; // gives the quantity of bodies added * 2
         let mut add = Vec::with_capacity(add_len);
         for i in id..self.id_counter {
            if let Some(c) = self.table.get(&i) {
               add.push((false, i, f64::min(c.body.aabb.min.x, c.body.aabb.min.x + c.body.vel.x)));
               add.push((false, i, f64::max(c.body.aabb.max.x, c.body.aabb.max.x + c.body.vel.x)));
            }
         }
         // sort new b & e values
         // there are no gaurantees as to order, thus a quicksort(-insertion hybrid) is used
         quicksort_be(&mut add, 0, add_len - 1);

         let mut new_be_list = Vec::with_capacity(self.be_list.len() + add_len - self.rems.len());
         let mut old_index = 0;
         if self.rems.len() > 0 { // If removals as well as additions must be accounted for.
            let mut add_index = 0;
            for _ in 0..new_be_list.len() {
               if self.be_list[old_index].2 > add[add_index].2 {
                  new_be_list.push(add[add_index]);
                  add_index = add_index + 1;
               } else {
                  if !self.rems.contains(&self.be_list[old_index].1) { new_be_list.push(self.be_list[old_index]); }
                  old_index = old_index + 1;
               }
            }
         } else { // If only additions must be accounted for.
            let mut add_index = 0;
            for _ in 0..new_be_list.len() {
               if self.be_list[old_index].2 > add[add_index].2 {
                  new_be_list.push(add[add_index]);
                  add_index = add_index + 1;
               } else {
                  new_be_list.push(self.be_list[old_index]);
                  old_index = old_index + 1;
               }
            }
         }
      } else if self.rems.len() > 0 { // If only removals are in order. Else, no action required.
         let mut new_be_list = Vec::with_capacity(self.be_list.len() - self.rems.len());
         let mut old_index = 0;
         for _ in 0..new_be_list.len() {
            if !self.rems.contains(&self.be_list[old_index].1) { new_be_list.push(self.be_list[old_index]); }
            old_index = old_index + 1;
         }
         self.be_list = new_be_list;
      }

      self.adds = None;
      self.rems = FnvHashSet::default();
   }

   fn reposition_val(&mut self, index: usize, x: f64) {
      let old_val = self.be_list[index];
      if x > old_val.2 {
         let mut j = index + 1;
         while j < self.be_list.len() && self.be_list[j].2 < x {
            self.be_list[j-1] = self.be_list[j];
            j = j + 1;
         }
         self.be_list[j - 1] = (old_val.0, old_val.1, x);
      } else {
         let mut j = index - 1;
         while j >= 0 && self.be_list[j].2 > x {
            self.be_list[j+1] = self.be_list[j];
            j = j - 1;
         }
         self.be_list[j + 1] = (old_val.0, old_val.1, x);
      }
   }

   pub fn sweep_and_prune(&mut self, t: f64, epsilon: f64) { // todo: implement carries, recur, etc
      //! Performs a [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) broadphase algorithm implementation.

      let mut active = IndexMap::with_hasher(FnvBuildHasher::default()); // id, b be index
      let mut candidates = HashMap::with_hasher(FnvBuildHasher::default()); // id, (other id, other be b, bsd)
      let mut remainders = IndexMap::with_hasher(FnvBuildHasher::default()); // id, (be val indecies, x belist pos's)
      for (be_index, val) in self.be_list.iter().enumerate() {
         let id1 = val.1;
         match val.0 {
            false => { let _ = active.insert(id1, be_index); }, // send b val to active
            true => {
               let val1_be_b = active.remove(&id1).expect("No active list entry for e value.");
               // Check if there are any potential colliding Colliders according to Sweep And Prune.
               if active.len() == 0 { continue; }
               // Retrieve the collider.
               let c1 = self.table.get(&id1).expect("Table does not contain Id of val in be_list");
               // Retrieve a previous collision candidate if any, else init to invalid data.
               let mut was_candidate: bool;
               let (mut id2, mut val2_be_b, mut data) = 
                  if let Some(c) = candidates.get(&id1) { was_candidate = true; *c 
                  } else { was_candidate = false; (usize::MAX, usize::MAX, BodySweepData::new_invalid()) };

               // Find the closest collision, if any.
               for (v2_id, v2_be_b) in active.iter() {
                  let c2 = self.table.get(&id2).expect("Table does not contain Id of active collider.");
                  if c1.broad_y.0 <= c2.broad_y.1 && c1.broad_y.1 >= c2.broad_y.0 { // y broad box check
                     if let Some(bsd) = crate::narrow::swept::body_sweep(&c1.body, &c2.body, t) {
                        if bsd.travel < data.travel {
                           was_candidate = false;
                           data = bsd;
                           id2 = *v2_id;
                           val2_be_b = *v2_be_b;
                        }
                     }
                  }
               }

               // If a collision candidate has been found, 
               if id2 != usize::MAX {
                  // Retrieve all necessary data from c2.
                  let c2 = self.table.get(&id2).expect("Collider of id2 does not exist.");
                  let (c2_is_static, c2_vel_add) = if let ColliderType::Static(va) = c2.reacter { 
                     (true, if va { c2.body.vel } else { cgmath::vec2(0.0, 0.0) } ) 
                  } else { (false, cgmath::vec2(0.0, 0.0)) };

                  // Modify c1 in response to the collision.
                  let c1 = if let Some(c) = self.table.get_mut(&id1) { c } else { panic!(); };
                  let c1_is_static = if let ColliderType::Static(_) = (*c1).reacter { true } else { false };

                  if !c2_is_static {
                     (c1.trigger)(id2, id1, data, epsilon);
                  } else if c1_is_static {
                     c1.body.translate(c1.body.vel.mul_element_wise(t * data.travel) + data.norm.mul_element_wise(epsilon));
                     let vel = if let ColliderType::Reacter(r) = &c1.reacter 
                        { r.react(c1.body.vel, id1, id2, data, epsilon) } else { panic!() };
                     c1.body.vel += vel + c2_vel_add;
                     let broad = c1.body.get_broad();
                     c1.broad_y = (broad.min.y, broad.max.y);
                     remainders.insert(id1, ((val1_be_b, be_index), (broad.min.y, broad.max.y)));
                  }

                  if !was_candidate { // Only insert if the Collider has yet to be processed.
                     candidates.insert(id2, (id1, val2_be_b, data.invert()));
                  }
               }
            }
         }
      }

      // Resort the collided objects
      for (_, (indecies, broad_x) ) in remainders.iter() {
         self.reposition_val(indecies.0, broad_x.0);
         self.reposition_val(indecies.1, broad_x.1);
      }
      active.clear();
      candidates.clear();

      // todo: complete for remainders
      for (be_index, val) in self.be_list.iter().enumerate() {
         let id1 = val.1;
         match val.0 {
            false => { let _ = active.insert(id1, be_index); }, // send b val to active
            true => {

               let val1_be_b = active.remove(&id1).expect("No active list entry for e value.");
               // Check if there are any potential colliding Colliders according to Sweep And Prune.
               if active.len() == 0 { continue; }
               // Retrieve the collider.
               let c1 = self.table.get(&id1).expect("Table does not contain Id of val in be_list");
               // Retrieve a previous collision candidate if any, else init to invalid data.
               let mut was_candidate: bool;
               let (mut id2, mut val2_be_b, mut data) = 
                  if let Some(c) = candidates.get(&id1) { was_candidate = true; *c 
                  } else { was_candidate = false; (usize::MAX, usize::MAX, BodySweepData::new_invalid()) };

               // Find the closest collision, if any.
               for (v2_id, v2_be_b) in active.iter() {
                  let c2 = self.table.get(&id2).expect("Table does not contain Id of active collider.");
                  if c1.broad_y.0 <= c2.broad_y.1 && c1.broad_y.1 >= c2.broad_y.0 { // y broad box check
                     if let Some(bsd) = crate::narrow::swept::body_sweep(&c1.body, &c2.body, t) {
                        if bsd.travel < data.travel {
                           was_candidate = false;
                           data = bsd;
                           id2 = *v2_id;
                           val2_be_b = *v2_be_b;
                        }
                     }
                  }
               }

               // If a collision candidate has been found, 
               if id2 != usize::MAX {
                  // Retrieve all necessary data from c2.
                  let c2 = self.table.get(&id2).expect("Collider of id2 does not exist.");
                  let (c2_is_static, c2_vel_add) = if let ColliderType::Static(va) = c2.reacter { 
                     (true, if va { c2.body.vel } else { cgmath::vec2(0.0, 0.0) } ) 
                  } else { (false, cgmath::vec2(0.0, 0.0)) };

                  // Modify c1 in response to the collision.
                  let c1 = if let Some(c) = self.table.get_mut(&id1) { c } else { panic!(); };
                  let c1_is_static = if let ColliderType::Static(_) = (*c1).reacter { true } else { false };

                  if !c2_is_static {
                     (c1.trigger)(id2, id1, data, epsilon);
                  } else if c1_is_static {
                     c1.body.translate(c1.body.vel.mul_element_wise(t * data.travel) + data.norm.mul_element_wise(epsilon));
                     let vel = if let ColliderType::Reacter(r) = &c1.reacter 
                        { r.react(c1.body.vel, id1, id2, data, epsilon) } else { panic!() };
                     c1.body.vel += vel + c2_vel_add;
                     let broad = c1.body.get_broad();
                     c1.broad_y = (broad.min.y, broad.max.y);
                     remainders.insert(id1, ((val1_be_b, be_index), (broad.min.y, broad.max.y)));
                  }

                  if !was_candidate { // Only insert if the Collider has yet to be processed.
                     candidates.insert(id2, (id1, val2_be_b, data.invert()));
                  }
               }
            }
         }
      }
      
      // todo: update all to completion
   }
}


// ---------- Sorting Algorithms ---------- //

// code based off of https://en.wikipedia.org/wiki/Insertion_sort#Algorithm
#[inline]
fn insertion_be(a: &mut Vec<(bool, usize, f64)>, lo: usize, hi: usize) {
   for i in (lo + 1)..hi {
      let val = a[i];
      let mut j = i;
      while j != 0 && a[j].2 > val.2 {
         a[j] = a[j-1];
         j = j - 1;
      }
      a[j] = val;
   }
}
// code based off of https://en.wikipedia.org/wiki/Quicksort#Hoare_partition_scheme
fn quicksort_be(a: &mut Vec<(bool, usize, f64)>, lo: usize, hi: usize) {
   fn partition(a: &mut Vec<(bool, usize, f64)>, lo: usize, hi: usize) -> usize {
      let pivot = a[(lo + hi) / 2];
      let mut i = lo;
      let mut j = hi;
      loop {
         while a[i].2 < pivot.2 { i = i + 1; }
         while a[j].2 > pivot.2 { j = j - 1; }
         if i >= j {
            return j
         }
         let t = a[i];
         a[i] = a[j];
         a[j] = t;
      }
   }
   
   if lo < hi {
      if hi - lo >= 12 {
         let p = partition(a, lo, hi);
         quicksort_be(a, lo, p);
         quicksort_be(a, p + 1, hi);
      } else {
         // https://en.wikipedia.org/wiki/Quicksort#Hoare_partition_scheme, under 'Optimizations', 2nd
         insertion_be(a, lo, hi);
      }
   }
}
