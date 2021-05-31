pub mod response;

use std::any::Any;
use crate::narrow::swept::{Body, BodySweptData};
use self::response::Responder;
use cgmath::Vector2;
use fnv::{FnvHashMap, FnvHashSet};
use indexmap::IndexSet;


#[derive(Debug, Clone, Copy)]
pub struct CollData {
   /// The index of this Collider's colliding Shape.
   pub self_shape: usize,
   /// The index of the other Collider's colliding Shape.
   pub other_shape: usize,
   /// The fraction of this Body's velocity until collision.
   pub travel: f64,
   /// The normal from the other collider.
   pub norm: Vector2<f64>,
}
#[inline]
fn swept_to_coll_data(data: BodySweptData) -> (CollData, CollData) {(
   CollData { self_shape: data.b1_shape, other_shape: data.b2_shape, travel: data.travel, norm: data.norm }, // fixme: data.travel does not take into account substepping carry?
   CollData { self_shape: data.b2_shape, other_shape: data.b1_shape, travel: data.travel, norm: -data.norm }
)}

struct Collider { // todo: finalize
   /// The body.
   pub body: Body,

   /// Toggles whether to not invoke a collision response from colliding Colliders.
   pub is_trigger: bool,
   /// Handles collisions and triggers.
   pub responder: Box<dyn Responder>,

   /// Arbitrary data.
   pub user_data: Box<dyn Any>,
}

pub struct Cosmos { // todo: document
   id_counter: usize,
   table: FnvHashMap<usize, Collider>,

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

impl Cosmos {

   pub fn get_be_list_indecies(&self, id: usize) -> (usize, usize) {
      //! Binary searches be_list for the b & e values belonging to the id provided. Returns indecies of `(b, e)`. Ensure be_list is sorted.
      panic!() // todo: implement
   }
   pub fn get_be_list_position(&self, x: f64) -> usize {
      //! Binary searches be_list for the index of- or the index before the given x value. Ensure be_list is sorted.
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

   pub fn step(&mut self, t: f64) {
      // todo: remove or ?:self.sweep_and_prune();
   }

   fn sweep_and_prune(&mut self, t: f64) { // todo: implement carries, recur, etc
      //! Performs a [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) broadphase algorithm implementation.
      
      let mut set = IndexSet::<usize>::new();
      for v in self.be_list.iter() {
         match v.0 {
            false => { let _ = set.insert(v.1); }, // send b val to active
            true => {
               set.remove(&v.1);
               if set.len() > 0 {
                  if let Some(c1) = self.table.get(&v.1) {
                     let b1b_min = f64::min(c1.body.aabb.min.y, c1.body.aabb.min.y + c1.body.vel.y);
                     let b1b_max = f64::max(c1.body.aabb.max.y, c1.body.aabb.max.y + c1.body.vel.y);

                     let mut data = BodySweptData { b1_shape: usize::MAX, b2_shape: usize::MAX, travel: f64::INFINITY, norm: cgmath::vec2(0.0, 0.0) };
                     let mut second = 0;
                     for u in set.iter() {
                        if let Some(c2) = self.table.get(u) {
                           let b2b_min = f64::min(c2.body.aabb.min.y, c2.body.aabb.min.y + c2.body.vel.y);
                           let b2b_max = f64::max(c2.body.aabb.max.y, c2.body.aabb.max.y + c2.body.vel.y);

                           // the final portion of the broad bounding box bounds check
                           if b1b_min <= b2b_max && b1b_max >= b2b_min {
                              if let Some(bsd) = crate::narrow::swept::body_sweep(&c1.body, &c2.body, t) {
                                 if bsd.travel < data.travel {
                                    data = bsd;
                                    second = *u;
                                 }
                              }
                           }
                        }
                     }

                     // todo:
                     if data.b1_shape != usize::MAX {
                        // update positions

                        // (get mut c1).responder.response(cosmos, vel, self_id, other_id, data) // if other !trigger
                        // (get mut c2 from second).responder.response(cosmos, vel, self_id, other_id, data) // if other !trigger

                        // trigger c1 & c2

                        // put in a queue of stuff that needs to be checked again
                     } else {
                        // put into a queue for stuff considered uncollided
                     }
                  } else {
                     panic!();
                  }
               }
            }
         }
      }
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
