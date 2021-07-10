pub mod reacter;

use crate::{*, prelude::narrow::*};
use self::reacter::Reacter;
use fnv::{FnvBuildHasher, FnvHashMap, FnvHashSet};
use indexmap::IndexMap;
use std::{cmp::Ordering, fmt::Debug, mem::ManuallyDrop};


union ColliderTypeUnion {
    /// Boolean indicates whether reacters should have this static Collider's velocity added to theirs as a result of the collision.
    stat: bool,
    reacter: ManuallyDrop<Box<dyn Reacter>>,
}

/// Colliders are units managed by a Plane. Colliders can be classified into two types: static, and reacting.
/// Changes in position are managed by the Plane, changes in velocity are managed by reactions/user.
/// Mutating body's position is allowed, but may cause tunneling, as pre-solving is solely used: create Reacters with care.
/// This layout of responsibility is designed to be ideal for games such as platformers that usually would implement custom behaviours for most moving entities, instead of physical.
/// *Static-Static*   : No behaviour <br>
/// *Static-Reacting* : Reacter `.reacter.react(..)`, Static `.trigger(..)` <br>
/// *Reacting-Reacting* : Reacter `.trigger(..)`, Reacter `.trigger(..)`
pub struct Collider {
    pub body: Body,
    pub trigger: Box<dyn Fn(usize, usize, Fp, BodySweepData, Fp)>,

    is_static: bool,
    ctu: ColliderTypeUnion,

    broad_y: (Fp, Fp),
    remainder: Fp,
}

impl Debug for Collider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Collider")
            .field("is_static", &self.is_static)
            .field("body", &self.body)
            .finish()
    }
}
impl Drop for Collider {
    fn drop(&mut self) {
        if !self.is_static {
            unsafe { ManuallyDrop::drop(&mut self.ctu.reacter) }
        }
    }
}
impl Collider {
    pub fn new_reacter(body: Body, reacter: Box<dyn Reacter>) -> Collider {
        let broad = body.get_broad();
        Collider {
            body,
            trigger: Box::new(|_, _, _, _, _| {}),

            is_static: false,
            ctu: ColliderTypeUnion {
                reacter: ManuallyDrop::new(reacter),
            },
            broad_y: (broad.min.y, broad.max.y),
            remainder: 1.0,
        }
    }
    pub fn new_reacter_trigger(
        body: Body,
        trigger: Box<dyn Fn(usize, usize, Fp, BodySweepData, Fp)>,
        reacter: Box<dyn Reacter>,
    ) -> Collider {
        let broad = body.get_broad();
        Collider {
            body,
            trigger,

            is_static: false,
            ctu: ColliderTypeUnion {
                reacter: ManuallyDrop::new(reacter),
            },
            broad_y: (broad.min.y, broad.max.y),
            remainder: 1.0,
        }
    }
    /// Knock defines whether dynamic colliders should have this static collider's velocity added to them on collision.
    /// This is usually recommended to be enabled for collisions to appear more sensible.
    pub fn new_static(body: Body, knock: bool) -> Collider {
        let broad = body.get_broad();
        Collider {
            body,
            trigger: Box::new(|_, _, _, _, _| {}),

            is_static: true,
            ctu: ColliderTypeUnion { stat: knock },
            broad_y: (broad.min.y, broad.max.y),
            remainder: 1.0,
        }
    }
    /// Knock defines whether dynamic colliders should have this static collider's velocity added to them on collision.
    /// This is usually recommended to be enabled for collisions to appear more sensible.
    pub fn new_static_trigger(
        body: Body,
        trigger: Box<dyn Fn(usize, usize, Fp, BodySweepData, Fp)>,
        knock: bool,
    ) -> Collider {
        let broad = body.get_broad();
        Collider {
            body,
            trigger,
            is_static: true,
            ctu: ColliderTypeUnion { stat: knock },
            broad_y: (broad.min.y, broad.max.y),
            remainder: 1.0
        }
    }
}

#[derive(Debug)]
pub struct Plane {
    id_counter: usize,
    table: FnvHashMap<usize, Collider>,
    adds: Option<usize>,     // id of first of add candidates
    rems: FnvHashSet<usize>, // ids of all removal candidates

    require_recalc: bool,
    require_resort: bool,

    // the x axis is chosen to be the discrimination axis due to (handling a row of columns most efficiently):
    //    1. it is the most 'primitave' axis
    //    2. screens/viewports are usually wider than they are tall
    //    3. game worlds are usually wider than they are tall
    //    4. Character/humanoid hitboxes are usually taller than they are wide, and next to each other
    be_list: Vec<(bool, usize, Fp)>,
}

impl Default for Plane {
    fn default() -> Self {
        Plane::new()
    }
}
impl Plane {
    pub fn new() -> Plane {
        Plane {
            id_counter: 0,
            table: FnvHashMap::default(),
            adds: None,
            rems: FnvHashSet::default(),
            require_recalc: true,
            require_resort: true,
            be_list: Vec::new(),
        }
    }

    #[inline]
    pub unsafe fn supress_recalc(&mut self) {
        //! Unflags the broadphase for requiring recalculation.
        //! # Safety
        //! Doing so while having mutated any collider's velocity, position, or shapes may lead to unexpected/incorrect
        //! behaviour when the broadphase is invoked/updated. Do so only when no such mutations have occured.
        self.require_recalc = false;
    }
    #[inline]
    pub unsafe fn supress_resort(&mut self) {
        //! Unflags the broadphase for requiring re-sorting.
        //! # Safety
        //! Doing so while having mutated any collider's velocity, position, or shapes beyond an arbitrary threshhold
        //! may lead to unexpected/incorrect behaviour when the broadphase is invoked/updated. Do so only when no such mutations have occured.
        self.require_recalc = false;
    }

    #[inline]
    pub fn get_collider(&self, id: usize) -> &Collider {
        //! # Panics
        //! Panics if a collider with the specified ID does not exist.
        self.table
            .get(&id)
            .expect("Plane does not contain the collider with the specified ID.")
    }
    #[inline]
    pub fn get_collider_mut(&mut self, id: usize) -> &mut Collider {
        //! Flags broadphase as requiring recalculation.
        //! # Panics
        //! Panics if a collider with the specified ID does not exist.
        self.require_recalc = true;
        self.table
            .get_mut(&id)
            .expect("Plane does not contain the collider with the specified ID.")
    }
    #[inline]
    pub unsafe fn get_collider_mut_silent(&mut self, id: usize) -> &mut Collider {
        //! This version does not flag the broadphase as requiring recalculation.
        //! # Safety
        //! Do not mutate the collider's velocity, position, or shapes, else unexpected/incorrect behaviour when the broadphase is invoked may occur.
        //! # Panics
        //! Panics if a collider with the specified ID does not exist.

        self.table
            .get_mut(&id)
            .expect("Plane does not contain the collider with the specified ID.")
    }

    pub fn add_collider(&mut self, c: Collider) -> usize {
        //! Queues a collider for incorporation into the broadphase.
        self.id_counter += 1;
        self.table.insert(self.id_counter, c);
        if self.adds.is_none() {
            self.adds = Some(self.id_counter);
        }
        self.id_counter
    }
    pub fn remove_collider(&mut self, id: usize) {
        //! Queues a collider for removal from the broadphase.
        //! # Panics
        //! Panics if a collider with the specified ID does not exist. 
        //! This may be due to the ID being incorrect, the collider having been removed, or the collider has not been incorporated into the broadphase yet.
        //! Incorporation is performed by `Plane::update(&mut self)`.

        self.rems.insert(id);
        if self.table.remove(&id).is_none() {
            panic!("Plane does not contain the collider with the specified ID."); 
        };
    }

    pub fn line_query(&self, a: Vec2, b: Vec2, callback: &dyn Fn(usize, &Collider, Fp) -> bool) {
        //! Invokes the callback for each Collider the aabb intersects with until no more intersections or the callback `(id, collider, coeff a->b)` returns false. <br>
        //! Requires that the broadphase be unflagged for recalculation and re-sorting.

        assert!(!self.require_recalc && !self.require_resort, "Ensure b & e values have been recalculated and re-sorted.");

        let aabb = Aabb::new_safe(a.x, a.y, b.x, b.y);
        let mut active = FnvHashSet::default();
        for val in self.be_list.iter() {
            if val.2 < aabb.min.x {
                match val.0 {
                    false => { active.insert(val.1); }
                    true => { active.remove(&val.1); }
                }
            } else if val.2 > aabb.max.x {
                active.insert(val.1);
            } else {
                break;
            }
        }

        for id in active {
            let c = self.get_collider(id);
            if c.body.aabb.aabb_test(&aabb) {
                if let Some(r) = c.body.line_query(a, b) {
                    if !callback(id, c, r) {
                        break;
                    }
                }
            }
        }
    }
    pub fn aabb_query(&self, aabb: Aabb, callback: &dyn Fn(usize, &Collider) -> bool) {
        //! Invokes the callback for each Collider the aabb intersects with until no more intersections or the callback `(id, collider)` returns false. <br>
        //! Requires that the broadphase be unflagged for recalculation and resorting.
        assert!(!self.require_recalc && !self.require_resort, "Ensure b & e values have been recalculated and re-sorted.");

        let mut active = FnvHashSet::default();
        for val in self.be_list.iter() {
            if val.2 < aabb.min.x {
                match val.0 {
                    false => {
                        let _ = active.insert(val.1);
                    }
                    true => {
                        let _ = active.remove(&val.1);
                    }
                }
            } else if val.2 > aabb.max.x {
                active.insert(val.1);
            } else {
                break;
            }
        }

        for id in active {
            let c = self.get_collider(id);
            if c.body.aabb.aabb_test(&aabb) && !callback(id, c) {
                return;
            }
        }
    }

    pub fn recalc_be_vals(&mut self) {
        //! Update `Body`s prior. Does not process Collider adds and removals; use `self.incorporate(..)` instead. <br>
        //! Flags the broadphase for requiring re-sorting. Unflags the broadphase for requiring recalculation.
        for v in self.be_list.iter_mut() {
            if let Some(c) = self.table.get(&v.1) {
                match v.0 {
                    false => v.2 = Fp::min(c.body.aabb.min.x, c.body.aabb.min.x + c.body.vel.x), // b value
                    true => v.2 = Fp::max(c.body.aabb.max.x, c.body.aabb.max.x + c.body.vel.x), // e value
                }
            }
        }
        self.require_recalc = false;
        self.require_resort = true;
    }
    pub fn sort_be_vals(&mut self) {
        //! Ensure broadphase is not flagged for recalculation. <br>
        //! Unflags the broadphase for requiring resorting.
        assert!(!self.require_recalc, "Broadphase is flagged for requiring recalculation, will not sort out-of-date data.");

        // sort b & e values
        // insertion sort due to its adaptivity & efficiency: sort is expected to be mostly sorted due to temporal cohesion
        let len = self.be_list.len();
        insertion_be(&mut self.be_list, 0, len - 1);
        self.require_resort = false;
    }
    pub fn update(&mut self) {
        //! Merges additions and culls removals from the braodphase. <br>
        //! Recalculates and re-sorts broadphase first, if flagged as required.

        // Ensure be_list merges validly
        if self.require_recalc {
            self.recalc_be_vals();
        }
        if self.require_resort {
            self.sort_be_vals();
        }

        // Create list for added b & e values. This may be a candidate for parallelization?
        if let Some(id) = self.adds {
            // create new b & e values
            let add_len = (self.id_counter - id) * 2; // gives the quantity of bodies added * 2
            let mut add = Vec::with_capacity(add_len);
            for i in id..self.id_counter {
                if let Some(c) = self.table.get(&i) {
                    add.push((
                        false,
                        i,
                        Fp::min(c.body.aabb.min.x, c.body.aabb.min.x + c.body.vel.x),
                    ));
                    add.push((
                        false,
                        i,
                        Fp::max(c.body.aabb.max.x, c.body.aabb.max.x + c.body.vel.x),
                    ));
                }
            }
            // sort new b & e values
            // there are no gaurantees as to order, thus pattern-attacking quicksort is ideal
            self.be_list.sort_unstable_by(|x, y| -> Ordering {
                if x.2 > y.2 {
                    Ordering::Less
                } else if x.2 < y.2 {
                    Ordering::Greater
                } else {
                    Ordering::Equal
                }
            });

            let mut new_be_list =
                Vec::with_capacity(self.be_list.len() + add_len - self.rems.len());
            let mut old_index = 0;
            let mut add_index = 0;
            if !self.rems.is_empty() {
                // If removals as well as additions must be accounted for.
                for _ in 0..new_be_list.len() {
                    if self.be_list[old_index].2 > add[add_index].2 {
                        new_be_list.push(add[add_index]);
                        add_index += 1;
                    } else {
                        if !self.rems.contains(&self.be_list[old_index].1) {
                            new_be_list.push(self.be_list[old_index]);
                        }
                        old_index += 1;
                    }
                }
            } else {
                // If only additions must be accounted for.
                for _ in 0..new_be_list.len() {
                    if self.be_list[old_index].2 > add[add_index].2 {
                        new_be_list.push(add[add_index]);
                        add_index += 1;
                    } else {
                        new_be_list.push(self.be_list[old_index]);
                        old_index += 1;
                    }
                }
            }
        } else if !self.rems.is_empty() {
            // If only removals are in order. Else, no action required.
            let mut new_be_list = Vec::with_capacity(self.be_list.len() - self.rems.len());
            for be_val in self.be_list.iter() {
                if !self.rems.contains(&be_val.1) {
                    new_be_list.push(*be_val);
                }
            }
            self.be_list = new_be_list;
        }

        self.adds = None;
        self.rems = FnvHashSet::default();
    }

    fn reposition_val(&mut self, index: usize, x: Fp) {
        let old_val = self.be_list[index];
        let mut j = index;
        if x > old_val.2 {
            while j <= self.be_list.len() && self.be_list[j + 1].2 < x {
                self.be_list[j] = self.be_list[j + 1];
                j += 1;
            }
        } else {
            while j > 0 && self.be_list[j - 1].2 > x {
                self.be_list[j] = self.be_list[j - 1];
                j -= 1;
            }
        }
        self.be_list[j] = (old_val.0, old_val.1, x);
    }
    pub fn sweep_and_prune(&mut self, t: Fp, epsilon: Fp) {
        //! Performs a [sweep and prune](https://en.wikipedia.org/wiki/Sweep_and_prune) broadphase algorithm implementation. 

        if self.require_recalc || self.require_resort || self.adds.is_some() || !self.rems.is_empty() {
            self.update();
        }
        
        let mut active = IndexMap::with_hasher(FnvBuildHasher::default()); // id, b be index
        let mut candidates = FnvHashMap::default(); // id, (other id, other be b, bsd)
        let mut to_sort = IndexMap::with_hasher(FnvBuildHasher::default()); // id, (be val indecies, x belist pos's)       
        for (be_index, val) in self.be_list.iter().enumerate() {
            let id1 = val.1;
            match val.0 {
                false => {
                    let _ = active.insert(id1, be_index);
                } // send b val to active
                true => {
                    let val1_be_b = active
                        .remove(&id1)
                        .expect("No active list entry for e value.");
                    // Check if there are any potential colliding Colliders according to Sweep And Prune.
                    if active.is_empty() {
                        continue;
                    }
                    // Retrieve the collider.
                    let c1 = self
                        .table
                        .get(&id1)
                        .expect("Table does not contain Id of val in be_list");
                    // Retrieve a previous collision candidate if any, else init to invalid data.
                    let (mut was_candidate, (mut id2, mut val2_be_b, mut data)) =
                        if let Some(c) = candidates.remove(&id1) {
                            (true, c)
                        } else {
                            (false, (usize::MAX, usize::MAX, BodySweepData::new_invalid()))
                        };

                    // Find the closest collision, if any.
                    for (v2_id, v2_be_b) in active.iter() {
                        let c2 = self
                            .table
                            .get(&id2)
                            .expect("Table does not contain Id of active collider.");
                        if c1.broad_y.0 <= c2.broad_y.1
                            && c1.broad_y.1 >= c2.broad_y.0
                            && !(c1.is_static && c2.is_static)
                        {
                            // y broad box & static-static check
                            if let Some(bsd) =
                                crate::narrow::swept::body_sweep(&c1.body, &c2.body, t)
                            {
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
                        let c2 = self
                            .table
                            .get(&id2)
                            .expect("Collider of id2 does not exist.");
                        let (c2_is_static, c2_vel_add) = if c2.is_static {
                            (true, if unsafe { c2.ctu.stat } {
                                    c2.body.vel
                                } else {
                                    Vec2::ZERO
                                },
                            )
                        } else {
                            (false, Vec2::ZERO)
                        };

                        // Modify c1 in response to the collision.
                        let c1 = self.table.get_mut(&id1).expect("");
                        if !c2_is_static {
                            (c1.trigger)(id1, id2, data.travel * t, data, epsilon);
                        } else if c1.is_static {
                            c1.remainder = 1.0 - data.travel; // c1.remainder == 1.0
                            c1.body.translate(c1.body.vel * (t * data.travel) + data.norm * epsilon);
                            let vel = unsafe {
                                (*c1.ctu.reacter).react(c1.body.vel, id1, id2, data, epsilon)
                            };
                            c1.body.vel += vel + c2_vel_add;
                            let broad = c1.body.get_broad();
                            c1.broad_y = (broad.min.y, broad.max.y);
                            to_sort.insert(id1, ((val1_be_b, be_index), (broad.min.y, broad.max.y)));
                        }

                        if !was_candidate {
                            // Only insert if the Collider has yet to be processed.
                            candidates.insert(id2, (id1, val2_be_b, data.invert()));
                        }
                    }
                }
            }
        }

        while !to_sort.is_empty() {
            // Resort the collided objects
            for (_, (indecies, broad_x)) in to_sort.iter() {
                self.reposition_val(indecies.0, broad_x.0);
                self.reposition_val(indecies.1, broad_x.1);
            }
            active.clear();
            candidates.clear();

            for (be_index, val) in self.be_list.iter().enumerate() {
                let id1 = val.1;
                match val.0 {
                    false => {
                        let _ = active.insert(id1, be_index); // send b val to active
                    }
                    true => {
                        let val1_be_b = active
                            .remove(&id1)
                            .expect("No active list entry for e value.");
                        // Check if there are any potential colliding Colliders according to Sweep And Prune.
                        if active.is_empty() {
                            continue;
                        }

                        // Retrieve a previous collision candidate if any, else init to invalid data.
                        let prev_sort = to_sort.remove(&id1).is_some();
                        let (mut was_candidate, (mut id2, mut val2_be_b, mut data)) =
                            if let Some(c) = candidates.remove(&id1) {
                                (true, c)
                            } else {
                                (false, (usize::MAX, usize::MAX, BodySweepData::new_invalid()))
                            };

                        if !prev_sort && !was_candidate {
                            continue;
                        }

                        let table_raw: *mut FnvHashMap<usize, Collider> = &mut self.table;
                        // Retrieve the collider.
                        let c1 = unsafe {
                            (*table_raw)
                                .get_mut(&id1)
                                .expect("Table does not contain Id of val in be_list")
                        };
                        let mut rem_diff = Fp::NAN;

                        // Find the closest collision, if any.
                        for (v2_id, v2_be_b) in active.iter() {
                            if prev_sort || to_sort.contains_key(v2_id) {
                                // don't check colliders if niether were sort candidates
                                let c2 = unsafe {
                                    (*table_raw)
                                        .get_mut(&id2)
                                        .expect("Table does not contain Id of val in active")
                                };
                                if c1.broad_y.0 <= c2.broad_y.1
                                    && c1.broad_y.1 >= c2.broad_y.0
                                    && !(c1.is_static && c2.is_static)
                                {
                                    // y broad box check
                                    // Adjust for largest remainder
                                    let c1c2_rem_diff = (c2.remainder - c1.remainder) * t;
                                    if c1c2_rem_diff < 0.0 {
                                        let offset = c2.body.vel * c1c2_rem_diff;
                                        c2.body.translate(offset);
                                        if let Some(bsd) = swept::body_sweep(&c1.body, &c2.body, t * c1.remainder) {
                                            if bsd.travel < data.travel {
                                                was_candidate = false;
                                                data = bsd;
                                                id2 = *v2_id;
                                                val2_be_b = *v2_be_b;
                                                rem_diff = c1c2_rem_diff;
                                            }
                                        }
                                        c2.body.translate(-offset);
                                    } else {
                                        let offset = c1.body.vel * -c1c2_rem_diff;
                                        c1.body.translate(offset);
                                        if let Some(bsd) = swept::body_sweep(&c1.body, &c2.body, t * c2.remainder) {
                                            if bsd.travel - c1c2_rem_diff < data.travel {
                                                was_candidate = false;
                                                data = bsd;
                                                data.travel -= c1c2_rem_diff;
                                                id2 = *v2_id;
                                                val2_be_b = *v2_be_b;
                                                rem_diff = c1c2_rem_diff;
                                            }
                                        }
                                        c1.body.translate(-offset);
                                    }
                                }
                            }
                        }

                        // If a collision candidate has been found,
                        if id2 != usize::MAX {
                            // Retrieve all necessary data from c2.
                            let c2 = self
                                .table
                                .get(&id2)
                                .expect("Collider of id2 does not exist.");
                            let (c2_is_static, c2_vel_add) = if c2.is_static {
                                (true, if unsafe { c2.ctu.stat } { 
                                    c2.body.vel 
                                } else { 
                                    Vec2::ZERO
                                })
                            } else {
                                (false, Vec2::ZERO)
                            };

                            // Modify c1 in response to the collision.
                            if !c2_is_static {
                                (c1.trigger)(id1, id2, data.travel * t, data, epsilon);
                            } else if c1.is_static {
                                c1.remainder -= data.travel;
                                c1.body.translate(c1.body.vel * (t * data.travel) + data.norm * epsilon);
                                let vel = unsafe {
                                    (*c1.ctu.reacter).react(c1.body.vel, id1, id2, data, epsilon)
                                };
                                c1.body.vel += vel + c2_vel_add;
                                let broad = c1.body.get_broad();
                                c1.broad_y = (broad.min.y, broad.max.y);
                                to_sort.insert(id1, ((val1_be_b, be_index), (broad.min.y, broad.max.y)));
                            }

                            if !was_candidate {
                                // Only insert if the Collider has yet to be processed.
                                data.travel += rem_diff;
                                candidates.insert(id2, (id1, val2_be_b, data.invert()));
                            }
                        }
                    }
                }
            }
        }

        // finally, update all clear paths
        for (_, c) in self.table.iter_mut() {
            c.body.translate(c.body.vel * c.remainder);
            c.remainder = 1.0;
        }

        self.require_recalc = false;
        self.require_resort = false;
    }

/* pub fn get_be_list_indecies(&self, id: usize, minx: Fp, maxx: Fp) -> (usize, usize) {
        //! Binary searches be_list for the b & e values belonging to the id provided.
        //! Returns indecies of `(b, e)`, or `(usize::MAX, usize::MAX)` if minx and maxx aren't found. <br>
        //! Ensure be_list is sorted.
        let len = self.be_list.len();
        let mut b = usize::MAX;
        for i in self.get_be_list_leftmost(minx)..len {
            if self.be_list[i].1 == id {
            b = i;
            break;
            }
        }
        let mut e = usize::MAX;
        for i in self.get_be_list_leftmost(maxx)..len {
            if self.be_list[i].1 == id {
            e = i;
            break;
            }
        }
        (b, e)
    }
    pub fn get_be_list_leftmost(&self, x: Fp) -> usize {
        //! Binary searches be_list for the leftmost valid position for x. <br>
        //! Ensure be_list is sorted.
        let mut lo = 0;
        let mut hi = self.be_list.len();
        while lo < hi {
            let half = (lo + hi) / 2;
            if self.be_list[half].2 < x {
            lo = half + 1;
            } else {
            hi = half;
            }
        }
        lo
    }*/
}

// ---------- Sorting Algorithms ---------- //

// code based off of https://en.wikipedia.org/wiki/Insertion_sort#Algorithm
#[inline]
fn insertion_be(a: &mut Vec<(bool, usize, Fp)>, lo: usize, hi: usize) {
    for i in (lo + 1)..hi {
        let val = a[i];
        let mut j = i;
        while j != 0 && a[j - 1].2 > val.2 {
            a[j] = a[j - 1];
            j -= 1;
        }
        a[j] = val;
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn broad_intgr_test() {
        let mut plane = Plane::default();
        let col1 = Collider::new_static(
            Body::new(vec![Aabb::new(-0.5, -0.5, 0.5, 0.5).into()], Vec2::splat(0.5), Vec2::X, false), 
            true);

        plane.add_collider(col1);

        todo!();
    }
}
