pub mod cosmos;
pub mod phys;

use crate::{Body, BodySweptData, swept};
use std::{collections::HashSet, ops::Range, thread};
use indexmap::IndexSet;


type NoCollResponse = fn(usize);
type CollResponse = fn(usize, usize, &BodySweptData);
type BodyIndex<'a> = fn(usize) -> &'a Body;

#[inline] // this should be inlined where used to optimise away all(?) the function calls
fn sweep_and_prune(index: fn(usize) -> &'static Body, ncr: NoCollResponse, cr: CollResponse, mut sort: Vec<(bool, usize, f64)>, t: f64,
   adds: Range<usize>, rems: HashSet<usize>) -> Vec<(bool, usize, f64)> {
   //! Incorporates body adds and removals, and performs a [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) broadphase algorithm implementation.

   // the x axis is chosen to be the discrimination axis due to (handling a row of columns most efficiently): 
   //    1. it is the most 'primitave' axis
   //    2. screens/viewports are usually wider than they are tall
   //    3. game worlds are usually wider than they are tall
   //    4. Character/humanoid hitboxes are usually taller than they are wide, and next to each other

   let len = sort.len();
   let update = move |mut sort: Vec<(bool, usize, f64)>| { 
      // update b & e values
      for v in sort.iter_mut() {
         let b = index(v.1);
         v.2 = match v.0 { // todo: check perf against inlign assign and if statement?
            false => f64::min(b.aabb.min.x, b.aabb.min.x + b.vel.x), // b value
            true => f64::max(b.aabb.max.x, b.aabb.max.x + b.vel.x), // e value
         };
      }

      // sort b & e values
      // insertion sort due to its adaptivity & efficiency: sort is expected to be mostly sorted due to temporal cohesion
      insertion_be(&mut sort, 0, len - 1);
      sort
   };

   sort = if adds.len() > 0 {
      let new_len = len - rems.len() * 2 + adds.len() * 2;

      // --- create added, sorted, be values --- //
      // create all the new b & e values necessary
      let add_len = adds.len() * 2;
      let mut add = Vec::with_capacity(add_len);
      for i in adds {
         let b = index(i);
         add.push((false, i, f64::min(b.aabb.min.x, b.aabb.min.x + b.vel.x)));
         add.push((false, i, f64::max(b.aabb.max.x, b.aabb.max.x + b.vel.x)));
      }
      // sort new b & e values
      // there are no gaurantees as to order, thus a quicksort(-insertion hybrid) is used
      quicksort_be(&mut add, 0, add_len - 1);

      // --- update and sort old sort data --- //
      let old = update(sort);

      // --- merge old and new data ---//
      let mut new_sort = Vec::with_capacity(new_len);
      let mut old_index = 0;
      let mut add_index = 0;

      for _ in 0..new_len {
         if old[old_index].2 > add[add_index].2 {
            new_sort.push(add[add_index]);
            add_index = add_index + 1;
         } else {
            if !rems.contains(&old[old_index].1) { new_sort.push(old[old_index]); }
            old_index = old_index + 1;
         }
      }

      new_sort
   } else if rems.len() > 0 {
      let new_len = len - rems.len() * 2;
      let mut new_sort = Vec::with_capacity(new_len);
      let mut index = 0;

      for _ in 0..new_len {
         if !rems.contains(&sort[index].1) { new_sort.push(sort[index]); }
         index = index + 1;
      }

      update(new_sort)
   } else {
      update(sort)
   };
   
   let mut set = IndexSet::<usize>::new();
   for v in sort.iter() {
      match v.0 {
         false => { let _ =  set.insert(v.1); }, // send b val to active
         true => {
            set.remove(&v.1);
            if set.len() > 0 {
               let b1 = index(v.1);
               let b1b_min = f64::min(b1.aabb.min.y, b1.aabb.min.y + b1.vel.y);
               let b1b_max = f64::max(b1.aabb.max.y, b1.aabb.max.y + b1.vel.y);

               let mut data = BodySweptData { b1_shape: usize::MAX, b2_shape: usize::MAX, travel: f64::INFINITY, norm: cgmath::vec2(0.0, 0.0) };
               let mut second = 0;
               for u in set.iter() {
                  let b2 = index(*u);
                  let b2b_min = f64::min(b2.aabb.min.y, b2.aabb.min.y + b2.vel.y);
                  let b2b_max = f64::max(b2.aabb.max.y, b2.aabb.max.y + b2.vel.y);

                  // the final portion of the broad bounding box bounds check
                  if b1b_min <= b2b_max && b1b_max >= b2b_min {
                     if let Some(bsd) = swept::body_sweep(b1, b2, t) {
                        if bsd.travel < data.travel {
                           data = bsd;
                           second = *u;
                        }
                     }
                  }
               }

               if data.b1_shape != usize::MAX {
                  cr(v.1, second, &data);
               } else {
                  ncr(v.1);
               }
            }
         }
      }
   }

   return sort
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
