use std::{fmt::{Debug, Formatter}, mem::ManuallyDrop};
use cgmath::{ElementWise, InnerSpace, Vector2};

pub mod swept;

// ---------- Point & Line ---------- //

#[inline]
pub fn colinearity_test(a: Vector2<f64>, b: Vector2<f64>, c: Vector2<f64>) -> bool {
   //! Returns whether all three points are colinear.
   let v = a - b;
   let w = b - c;
   v.x * w.y - v.y * w.x == 0.0 && v.dot(w) > 0.0
}
#[inline]
pub fn point_normal_test(loc: Vector2<f64>, a: Vector2<f64>, normal: Vector2<f64>) -> bool {
   //! Returns whether the point is toward the opposite direction of the normal from the vertex `a`.
   normal.dot(loc - a) <= 0.0
}

#[inline]
pub fn seg_seg_test(a1: Vector2<f64>, a2: Vector2<f64>, b1: Vector2<f64>, b2: Vector2<f64>) -> bool {
   //! Returns whether a line segment-line segment intersection occurs.
   let da = a2 - a1;
   let db = b2 - b1;

   let dot = da.x * db.y - db.x * da.y;

   // ensures result is consistent with line_line_query()
   // prevents rust from doing unpck optimisations that decrease perf
   if dot == 0.0 { return false }
   let dd = dot * dot;

   let nd1 = a1 - b1;
   let tdd = (da.x * nd1.y - da.y * nd1.x) * dot;
   let udd = (db.x * nd1.y - db.y * nd1.x) * dot;
   udd >= 0.0 && udd <= dd && tdd >= 0.0 && tdd <= dd
}
#[inline]
pub fn seg_seg_query(a1: Vector2<f64>, a2: Vector2<f64>, b1: Vector2<f64>, b2: Vector2<f64>) -> Option<f64> {
   //! Returns the coefficient distance along line segment `a` an intersection occurs.
   let da = a2 - a1;
   let db = b2 - b1;

   let dot = da.x * db.y - db.x * da.y;
   if dot == 0.0 { return None } // guard against colinearity
   let dd = dot * dot;

   let nd1 = a1 - b1;
   let tdd = (db.x * nd1.y - db.y * nd1.x) * dot;
   if tdd < 0.0 || tdd > dd { return None } // seg a guard

   let udd = (da.x * nd1.y - da.y * nd1.x) * dot;
   if udd < 0.0 || udd > dd { return None } // seg b guard

   Some(tdd / dd)
}
#[inline]
pub fn line_seg_query(o: Vector2<f64>, n: Vector2<f64>, b1: Vector2<f64>, b2: Vector2<f64>) -> Option<f64> { 
   //! Returns the distance along line from `o` an intersection occurs.
   let db = b2 - b1;

   let dot = n.x * db.y - db.x * n.y;
   if dot == 0.0 { return None } // guard against colinearity

   let nd1 = o - b1;
   let udd = (o.x * nd1.y - o.y * nd1.x) * dot;
   if udd < 0.0 || udd > dot * dot { return None } // seg b guard
   
   Some((db.x * nd1.y - db.y * nd1.x) / dot)
}
#[inline]
pub fn seg_line_query(a1: Vector2<f64>, a2: Vector2<f64>, o: Vector2<f64>, n: Vector2<f64>) -> Option<f64> { 
   //! Returns the coefficient distance along line segment `a` an intersection occurs.
   let da = a2 - a1;
   let dot = da.x * n.y - n.x * da.y;
   if dot == 0.0 { return None } // guard against colinearity

   let nd1 = a1 - o;
   let t = (n.x * nd1.y - n.y * nd1.x) / dot;
   if t < 0.0 || t > 1.0 { None } else { Some(t) }
}
#[inline]
pub fn line_line_query(o1: Vector2<f64>, n1: Vector2<f64>, o2: Vector2<f64>, n2: Vector2<f64>) -> Option<f64> { 
   //! Returns the distance along line `1` from `o1` an intersection occurs.
   let dot = n1.x * n2.y - n2.x * n1.y;
   if dot == 0.0 { return None } // guard against colinearity
   Some((n2.x * (o1.y - o2.y) - n2.y * (o1.x - o2.x)) / dot)
}

#[derive(Clone, Copy, Debug)]
pub struct Circle {
   pub rad: f64,
   pub pos: Vector2<f64>,
}
impl Circle {
   #[inline]
   pub fn new(rad: f64, posx: f64, posy: f64) -> Circle {
      Circle {
         rad: f64::abs(rad),
         pos: cgmath::vec2(posx, posy),
      }
   }
   #[inline]
   pub fn translate(self, offset: Vector2<f64>) -> Circle {
      Circle { pos: self.pos + offset, rad: self.rad }
   }

   #[inline]
   pub fn bounding_test(self, aabb: &Aabb) -> bool {
      //! Performs an AABB test between `aabb` and the circle's bounding box.
      aabb.min.x <= self.rad + self.pos.x && aabb.max.x >= self.pos.x - self.rad 
      && aabb.min.y <= self.pos.y + self.rad && aabb.max.y >= self.pos.y - self.rad
   }
}

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
   pub min: Vector2<f64>,
   pub max: Vector2<f64>,
}
impl Aabb {
   #[inline]
   pub fn new(minx: f64, miny: f64, maxx: f64, maxy: f64) -> Aabb {
      assert_eq!(minx < maxx, true);
      assert_eq!(miny < maxy, true);

      Aabb { 
         min: cgmath::vec2(minx, miny), 
         max: cgmath::vec2(maxx, maxy) 
      }
   }
   pub fn new_safe(ax: f64, by: f64, cx: f64, dy: f64) -> Aabb {
      //! Orders minimum and maximum values.
      if ax < cx {
         if by < dy {
            Aabb::new(ax, by, cx, dy)
         } else {
            Aabb::new(ax, dy, cx, by)
         }
      } else {
         if by < dy {
            Aabb::new(cx, by, ax, dy)
         } else {
            Aabb::new(cx, dy, ax, by)
         }
      }
   }

   #[inline]
   pub fn minx_maxy(self) -> Vector2<f64> {
      cgmath::vec2(self.min.x, self.max.y)
   }
   #[inline]
   pub fn maxx_miny(self) -> Vector2<f64> {
      cgmath::vec2(self.max.x, self.min.y)
   }
   #[inline]
   pub fn translate(self, offset: Vector2<f64>) -> Aabb {
      Aabb { min: self.min + offset, max: self.max + offset }
   }
   
   #[inline]
   pub fn broaden(&self, dir: Vector2<f64>) -> Aabb {
      Aabb { min: cgmath::vec2(f64::min(self.min.x, self.min.x + dir.x), f64::min(self.min.y, self.min.y + dir.y)),
         max: cgmath::vec2(f64::max(self.max.x, self.max.x + dir.x), f64::max(self.max.y, self.max.y + dir.y)) }
   }
}

/// A 2D convex polygon, vertices arranged clockwise - tailed with a duplicate of the first, with unit-length normals - without duplication. 
#[derive(Debug, Clone)]
pub struct Poly {
   pub aabb: Aabb,
   /// First vertex's duplicate tails. `verts.len() - 1 == norms.len()`
   pub verts: Vec<Vector2<f64>>,
   /// Length equals actual vertex count. `verts.len() - 1 == norms.len()`
   pub norms: Vec<Vector2<f64>>,
}
impl Poly {
   /// `verts` must form a convex polygon. `verts` must not contain duplicate or interior vertices.
   pub fn new(verts: &[Vector2<f64>]) -> Poly {
      let len = verts.len();
      let mut index = 0;

      let mut topmost_y = f64::MIN;
      for i in 0..len {
         if verts[i].y > topmost_y {
            index = i;
            topmost_y = verts[i].y;
         }
      }

      let mut order = vec![];
      order.extend(0..len);
      order[0] = index;
      order[index] = 0;

      // giftwrap
      let mut a = Vector2::<f64>::unit_x();
      for v in 0..(len - 1) {
         let vert = verts[order[v]];
         let mut best = f64::MIN;
         for i in (v + 1)..len {
            let dot = a.dot((verts[order[i]] - vert).normalize()) as f64;
            if dot > best {
               index = i;
               best = dot;
            }
         }
         a = verts[order[index]] - vert;
         let t = order[v + 1];
         order[v + 1] = order[index];
         order[index] = t;
      }

      let mut ordered_and_duped = Vec::with_capacity(len + 1);
      for i in 0..len { ordered_and_duped.push(verts[order[i]]); }
      ordered_and_duped.push(verts[order[0]]);
      Poly::new_from_wound(ordered_and_duped)
   }
   /// `verts` must form a convex polygon, be wrapped clockwise, and must contain a duplicate of the first trailing vertex.
   pub fn new_from_wound(verts: Vec<Vector2<f64>>) -> Poly {
      let len = verts.len() - 1; // verts contains a duplicate trailing vertex
      let mut norms = Vec::with_capacity(len);
      let (mut ix, mut iy, mut ax, mut ay) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
      for i in 0..len {
         norms.push(cgmath::vec2(-(verts[i+1].y - verts[i].y), verts[i+1].x - verts[i].x).normalize());
         
         if verts[i].x < ix { ix = verts[i].x; }
         if verts[i].x > ax { ax = verts[i].x; }
         if verts[i].y < iy { iy = verts[i].y; }
         if verts[i].y > ay { ay = verts[i].y; }
      }

      Poly { aabb: Aabb::new(ix, iy, ax, ay), verts, norms }
   }

   #[inline]
   pub fn translate(mut self, offset: Vector2<f64>) -> Poly {
      self.aabb = self.aabb.translate(offset);
      for i in 0..self.verts.len() {
         self.verts[i] = self.verts[i] + offset;
      }
      self
   }
}


// ---------- Shape-Shape intersection tests ---------- //

fn aabb_circle_test(&Aabb { min, max }: &Aabb, &Circle { rad, pos }: &Circle) -> bool {
   let rad2 = rad + rad;
   if min.x <= rad2 + pos.x && max.x >= pos.x - rad2 && min.y <= pos.y + rad2 && max.y >= pos.y - rad2 {
      if pos.x < min.x {
         if pos.y < min.y {
            return rad * rad >= (min - pos).magnitude2()
         } else if pos.y > max.y {
            let dx = pos.x - min.x;
            let dy = pos.y - max.y;
            return rad * rad >= dx * dx + dy * dy
         }
      } else if pos.x > max.x {
         if pos.y < min.y {
            let dx = pos.x - max.x;
            let dy = pos.y - min.y;
            return rad * rad >= dx * dx + dy * dy
         } else if pos.y > max.y {
            return rad * rad >= (pos - max).magnitude2()
         }
      }
      true
   } else {
      false
   }
}
fn poly_aabb_test(poly: &Poly, aabb: &Aabb) -> bool {
   if poly.aabb.aabb_test(aabb) {
      // given the aabbs intersect, the polygon must have the seperating axis
      for i in 0..poly.norms.len() {
         let n = poly.norms[i];
         let closest = 
            if n.x >= 0.0 {
               if n.y >= 0.0 {
                  aabb.min
               } else {
                  aabb.minx_maxy()
               }
            } else {
               if n.y >= 0.0 {
                  aabb.maxx_miny()
               } else {
                  aabb.max
               }
            };
         if n.dot(closest - poly.verts[i]) > 0.0 {
            return false // valid seperating axis found
         }
      }
      true
   } else {
      false
   }
}
fn poly_circle_test(poly: &Poly, circle: &Circle) -> bool {
   if !circle.bounding_test(&poly.aabb) { return false }

   let len = poly.norms.len();
   let mut found_separating_axis = false;
   if (0..len).any(|i| circle.point_test(poly.verts[i])) { return true } // vert tests
   for i in 0..len { // pseudo SAT tests
      let v = poly.verts[i];
      let vc = circle.pos - v;
      let dot = poly.norms[i].dot(vc) as f64; // >0: SA, <=0: no SA
      if dot <= 0.0 {
         continue; // axis does not seperate center
      } else {
         found_separating_axis = true;
         if dot <= circle.rad { // if extended axis encompases center
            let x = (poly.verts[i+1] - v).dot(vc) as f64;
            if 0.0 <= x && x <= 1.0 {
               return true // circle definitely touches line
            } else {
               continue;
            }
         } else {
            return false // circle center is outside extended poly
         }
      }
   }
   !found_separating_axis
}


// ---------- Intersect ---------- //

pub trait Intersect {
   fn get_bounding_box(&self) -> Aabb;

   fn point_test(&self, point: Vector2<f64>) -> bool; // point test
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool; // line intersection
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<f64>; // line entrypoint distance coeff

   fn circle_test(&self, circle: &Circle) -> bool; // circle intersection
   fn aabb_test(&self, aabb: &Aabb) -> bool; // aabb intersection
   fn poly_test(&self, poly: &Poly) -> bool; // poly intersection
}

impl Intersect for Circle {
   #[inline]
   fn get_bounding_box(&self) -> Aabb {
      Aabb { min: self.pos.sub_element_wise(self.rad), max: self.pos.add_element_wise(self.rad) }
   }

   #[inline]
   fn point_test(&self, a: Vector2<f64>) -> bool {
      //! Return whether a circle-point intersect occurs.
      self.rad * self.rad <= (self.pos.x - a.x) * (self.pos.x - a.x) + (self.pos.y - a.y) * (self.pos.y - a.y)
   }
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      //! Returns whether a circle-line intersection occurs.
      let dx = b.x - a.x;
      let dy = b.y - a.y;
      let mag = dx * dx + dy * dy;
      let u = (self.pos.x - a.x) * dx + (self.pos.y - a.y) * dy;
      if u < 0.0 || u > mag { return false }
      let u = u / mag;

      let distx = self.pos.x - (a.x + u * dx);
      let disty = self.pos.y - (b.y + u * dy);
      self.rad * self.rad >= distx * distx + disty * disty
   }
   #[inline]
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<f64> {
      //! Returns the coefficient distance along line segment `a->b` an intersection occurs. Does not return tangents.
      let diff = a - self.pos;
      let ab = b - a;
      let inv_size = 1.0 / (ab.magnitude2() as f64).sqrt();
      let unit = ab.mul_element_wise(inv_size); // normalization
      
      let dot = unit.dot(diff) as f64;
      let x = self.rad * self.rad + dot * dot - diff.magnitude2() as f64;
      if x > 0.0 {
         let d = -(dot + x.sqrt()) * inv_size;
         if d >= 0.0 && d <= 1.0 {
            return Some(d)
         }
      }
      None
   }
   
   #[inline]
   fn circle_test(&self, c: &Circle) -> bool {
      (self.rad - c.rad) * (self.rad - c.rad) <= (self.pos - c.pos).magnitude2()
   }
   #[inline]
   fn aabb_test(&self, aabb: &Aabb) -> bool {
       aabb_circle_test(aabb, self)
   }
   #[inline]
   fn poly_test(&self, poly: &Poly) -> bool {
      poly_circle_test(poly, self)
   }
}
impl Intersect for Aabb {
   #[inline]
   fn get_bounding_box(&self) -> Aabb {
      *self
   }

   #[inline]
   fn point_test(&self, point: Vector2<f64>) -> bool {
      //! Returns whether an AABB-point intersection occurs.
      point.x >= self.min.x && point.x <= self.max.x && point.y >= self.min.y && point.y <= self.max.y
   }
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      //! Returns whether an AABB-line intersection occurs.
      // SAT tests (https://www.gamedev.net/forums/topic/338987-aabb---line-segment-intersection-test/)
      let halfab = (b - a).mul_element_wise(0.5);  
      let halfaabb = (self.max - self.min).mul_element_wise(0.5);    
      let halfdiff = a + halfab - (self.min + self.max).mul_element_wise(0.5);
      let abs_hd_x = f64::abs(halfab.x);
      let abs_hd_y = f64::abs(halfab.y);
      !( f64::abs(halfdiff.x) > halfaabb.x + abs_hd_x
      || f64::abs(halfdiff.y) > halfaabb.y + abs_hd_y
      || f64::abs(halfab.x * halfdiff.y - halfab.y * halfdiff.x) > halfaabb.x * abs_hd_y + halfaabb.y * abs_hd_x + 0.00001 )
   }
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<f64> {
      //! Returns the coefficient distance along line segment `a->b` an intersection occurs.
      let ab = b - a;
      if a.x < self.min.x { // l/r tests
         if b.x > self.min.x {
            let t = (self.min.x - a.x) / ab.x;
            if t >= 0.0 || t <= 1.0 {
               return Some(t)
            }
         }
      } else if a.x > self.max.x {
         if b.x < self.max.x {
            let t = (self.max.x - a.x) /  ab.x;
            if t >= 0.0 || t <= 1.0 {
               return Some(t)
            }
         }
      }
      if a.y < self.min.y { // t/d tests
         if b.y > self.min.y {
            let t = (self.min.y - a.y) / ab.y;
            if t >= 0.0 || t <= 1.0 {
               return Some(t)
            }
         }
      } else if a.y > self.max.y {
         if b.y < self.max.y {
            let t = (self.max.y - a.y) / ab.y;
            if t >= 0.0 || t <= 1.0 {
               return Some(t)
            }
         }
      }
      None
   }

   #[inline]
   fn circle_test(&self, circle: &Circle) -> bool {
       aabb_circle_test(self, circle)
   }
   #[inline]
   fn aabb_test(&self, other: &Aabb) -> bool {
      self.min.x <= other.max.x && self.max.x >= other.min.x && self.min.y <= other.max.y && self.max.y >= other.min.y
   }
   #[inline]
   fn poly_test(&self, poly: &Poly) -> bool {
      poly_aabb_test(poly, self)
   }
}
impl Intersect for Poly {
   #[inline]
   fn get_bounding_box(&self) -> Aabb {
      self.aabb
   }

   fn point_test(&self, loc: Vector2<f64>) -> bool {
      (0..self.norms.len()).all(|i| point_normal_test(loc, self.verts[i], self.norms[i]))
   }
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      //! Returns whether a line-polygon intersection occurs.
      let len = self.norms.len();
      (0..len).any(|i| seg_seg_test(self.verts[i], self.verts[i+1], a, b))
   }
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<f64> {
      //! Returns the coefficient distance along line segment `a->b` an intersection occurs.
      let len = self.norms.len();
      let line = b - a;
      for i in 0..len {
         if point_normal_test(a, self.verts[i], self.norms[i]) && self.norms[i].dot(line) < 0.0 {
            if let Some(t) = seg_seg_query(a, b, self.verts[i], self.verts[i+1]) {
               return Some(t)
            }
         }
      }
      None
   }

   #[inline]
   fn circle_test(&self, circle: &Circle) -> bool {
      poly_circle_test(self, circle)
   }
   #[inline]
   fn aabb_test(&self, aabb: &Aabb) -> bool {
      poly_aabb_test(self, aabb)
   }
   fn poly_test(&self, other: &Poly) -> bool {
      //! Returns whether an intersection occurs between the two polygons.
      let len1 = self.norms.len();
      let len2 = other.norms.len();
      'axes1: for i in 0..len1 { // seperating axis algorithm
         let n = self.norms[i];
         let max = n.dot(self.verts[i]) as f64;
         for v in 0..len2 {
            if max >= n.dot(other.verts[v]) {
               continue 'axes1; // invalid axis if it does not seperate
            }
         }
         return false // valid axis found
      }
      'axes2: for i in 0..len2 { // rewind logo problem necessitates both polys be verified
         let n = other.norms[i];
         let max = n.dot(other.verts[i]) as f64;
         for v in 0..len1 {
            if max >= n.dot(self.verts[v]) {
               continue 'axes2; // invalid axis if it does not seperate
            }
         }
         return false // valid axis found
      }
      true
   }
}

// ---------- Shape ---------- //

union ShapeUnion {
   circle: Circle, // 24 bytes
   aabb: Aabb, // 32 bytes
   poly: ManuallyDrop<Poly>, // 80 bytes
}

pub struct Shape {
   id: u8,
   shape: ShapeUnion,
}
impl Shape {
   pub const CIRCLE_ID: u8 = 0u8;
   pub const AABB_ID: u8 = 1u8;
   pub const POLY_ID: u8 = 2u8;
   
   pub fn circle(circle: Circle) -> Shape {
      Shape { id: Shape::CIRCLE_ID, shape: ShapeUnion { circle } }
   }
   pub fn aabb(aabb: Aabb) -> Shape {
      Shape { id: Shape::AABB_ID, shape: ShapeUnion { aabb } }
   }
   pub fn poly(poly: Poly) -> Shape {
      Shape { id: Shape::POLY_ID, shape: ShapeUnion { poly: ManuallyDrop::new(poly) } }
   }

   pub fn get_circle(&self) -> Option<&Circle> {
      //! Safe alternative to `expect_circle`.
      if self.id == Shape::CIRCLE_ID { unsafe { Some(&self.shape.circle) } } else { None }
   }
   pub fn get_aabb(&self) -> Option<&Aabb> {
      //! Safe alternative to `expect_aabb`.
      if self.id == Shape::AABB_ID { unsafe { Some(&self.shape.aabb) } } else { None }
   }
   pub fn get_poly(&self) -> Option<&Poly> {
      //! Safe alternative to `expect_aabb`.
      if self.id == Shape::POLY_ID { unsafe { Some(&self.shape.poly) } } else { None }
   }
   pub fn get_circle_mut(&mut self) -> Option<&mut Circle> {
      //! Safe alternative to `expect_circle`.
      if self.id == Shape::CIRCLE_ID { unsafe { Some(&mut self.shape.circle) } } else { None }
   }
   pub fn get_aabb_mut(&mut self) -> Option<&mut Aabb> {
      //! Safe alternative to `expect_aabb`.
      if self.id == Shape::AABB_ID { unsafe { Some(&mut self.shape.aabb) } } else { None }
   }
   pub fn get_poly_mut(&mut self) -> Option<&mut Poly> {
      //! Safe alternative to `expect_aabb`.
      if self.id == Shape::POLY_ID { unsafe { Some(&mut self.shape.poly) } } else { None }
   }

   pub fn get_id(&self) -> u8 {
      //! Returns the internal shape id. Compare/Index to CIRCLE_ID, AABB_ID, POLY_ID.
      self.id
   }
   pub unsafe fn expect_circle(&self) -> &Circle {
      //! Match/Index against `get_id` to ensure safety.
      &self.shape.circle
   }
   pub unsafe fn expect_aabb(&self) -> &Aabb {
      //! Match/Index against `get_id` to ensure safety.
      &self.shape.aabb
   }
   pub unsafe fn expect_poly(&self) -> &Poly {
      //! Match/Index against `get_id` to ensure safety.
      &self.shape.poly
   }
   pub unsafe fn expect_circle_mut(&mut self) -> &mut Circle {
      //! Match/Index against `get_id` to ensure safety.
      &mut self.shape.circle
   }
   pub unsafe fn expect_aabb_mut(&mut self) -> &mut Aabb {
      //! Match/Index against `get_id` to ensure safety.
      &mut self.shape.aabb
   }
   pub unsafe fn expect_poly_mut(&mut self) -> &mut Poly {
      //! Match/Index against `get_id` to ensure safety.
      &mut self.shape.poly
   }

   pub fn shape_test(&self, other: &Shape) -> bool {
      // manually implemented jump table optimization
      // Safety: Shape id verified by jump table indexed by id
      fn circle_circle_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.circle.circle_test(&su2.circle) } }
      fn circle_aabb_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.circle.aabb_test(&su2.aabb) } }
      fn circle_poly_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.circle.poly_test(&su2.poly) } }
      fn aabb_circle_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.aabb.circle_test(&su2.circle) } }
      fn aabb_aabb_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.aabb.aabb_test(&su2.aabb) } }
      fn aabb_poly_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.aabb.poly_test(&su2.poly) } }
      fn poly_circle_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.poly.circle_test(&su2.circle) } }
      fn poly_aabb_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.poly.aabb_test(&su2.aabb) } }
      fn poly_poly_inter(su1: &ShapeUnion, su2: &ShapeUnion) -> bool { unsafe { su1.poly.poly_test(&su2.poly) } }

      type ShapeTestHeader = fn(&ShapeUnion, &ShapeUnion) -> bool;
      const SHAPE_JUMP_TABLE: [[ShapeTestHeader; 3]; 3] = [
         [circle_circle_inter, circle_aabb_inter, circle_poly_inter],
         [aabb_circle_inter, aabb_aabb_inter, aabb_poly_inter],
         [poly_circle_inter, poly_aabb_inter, poly_poly_inter]];
      SHAPE_JUMP_TABLE[self.id as usize][other.id as usize](&self.shape, &other.shape)
   }
}

/// A `match`-like macro for conveniently accessing the underlying `Circle`/`Aabb`/`Poly` of a `Shape`. 
/// 
/// # Examples
///
/// ```
/// let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0);
/// let shape = Shape::aabb(aabb);
///
/// match_shape!(&shape => {
///    Circle(c) => panic!(),
///    Aabb(a) => assert_eq(*a, aabb),
///    Poly(p) => panic!()
/// });
/// ```
/// Or, for mutating a `Shape`: 
/// ```
/// let circle = Circle::new(0.5, 1.0, 1.0);
/// let shape = Shape::circle(circle);
///
/// match_shape!(&mut shape => {
///    Circle(c) => { c.rad = 1.0; assert_eq!(c, Circle::new(1.0, 1.0, 1.0)); },
///    Aabb(a) => panic!(),
///    Poly(p) => panic!()
/// });
/// ```
macro_rules! shape_match {
   (&mut $shape:expr => { Circle($c:ident) => $ce:expr, Aabb($a:ident) => $ae:expr, Poly($p:ident) => $pe:expr } ) => {
      match (&$shape).get_id() {
         0u8 => { let $c: &mut Circle; unsafe { $c = $shape.expect_circle_mut(); } $ce },
         1u8 => { let $a: &mut Aabb; unsafe { $a = $shape.expect_aabb_mut(); } $ae },
         _ => { let $p: &mut Poly; unsafe { $p = $shape.expect_poly_mut(); } $pe },
      }
   };
   ($shape:expr => { Circle($c:ident) => $ce:expr, Aabb($a:ident) => $ae:expr, Poly($p:ident) => $pe:expr } ) => {
      match (&$shape).get_id() {
         0u8 => { let $c: &Circle; unsafe { $c = $shape.expect_circle(); } $ce },
         1u8 => { let $a: &Aabb; unsafe { $a = $shape.expect_aabb(); } $ae },
         _ => { let $p: &Poly; unsafe { $p = $shape.expect_poly(); } $pe },
      }
   };
}

impl Drop for Shape {
   fn drop(&mut self) {
      if self.id == 2u8 {
         // Safety: Just confirmed that ShapeUnion is poly by associated id
         unsafe {
            ManuallyDrop::drop(&mut self.shape.poly);
         }
      }
   }
}
impl Clone for Shape {
   fn clone(&self) -> Shape {
      shape_match!(&self => {
         Circle(c) => Shape { id: Shape::CIRCLE_ID, shape: ShapeUnion { circle: *c } },
         Aabb(a) => Shape { id: Shape::AABB_ID, shape: ShapeUnion { aabb: *a } },
         Poly(p) => Shape { id: Shape::POLY_ID, shape: ShapeUnion { poly: ManuallyDrop::new(p.clone()) } }
      })
   }
}
impl Debug for Shape {
   fn fmt(&self, f: &mut Formatter) -> std::fmt::Result {
      f.debug_struct("Shape")
         .field("id", &self.id)
         .field("shape", shape_match!(self => { Circle(c) => c, Aabb(a) => a, Poly(p) => p }))
         .finish()
   }
}
impl Intersect for Shape {
   fn get_bounding_box(&self) -> Aabb {
      shape_match!(&self => {
         Circle(c) => c.get_bounding_box(),
         Aabb(a) => *a,
         Poly(p) => p.aabb
      })
   }

   fn point_test(&self, loc: Vector2<f64>) -> bool {
      shape_match!(&self => {
         Circle(c) => c.point_test(loc),
         Aabb(a) => a.point_test(loc),
         Poly(p) => p.point_test(loc)
      })
   }
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      //! Returns whether a line-shape intersection occurs.
      shape_match!(&self => {
         Circle(c) => c.line_test(a, b),
         Aabb(aabb) => aabb.line_test(a, b),
         Poly(p) => p.line_test(a, b)
      })
   }
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<f64> {
      //! Returns the coefficient distance along line segment `a->b` an intersection occurs.
      shape_match!(&self => {
         Circle(c) => c.line_query(a, b),
         Aabb(aabb) => aabb.line_query(a, b),
         Poly(p) => p.line_query(a, b)
      })
   }
   
   fn circle_test(&self, circle: &Circle) -> bool {
      shape_match!(&self => {
         Circle(c) => c.circle_test(circle),
         Aabb(a) => a.circle_test(circle),
         Poly(p) => p.circle_test(circle)
      })
   }
   fn aabb_test(&self, aabb: &Aabb) -> bool {
      shape_match!(&self => {
         Circle(c) => c.aabb_test(aabb),
         Aabb(a) => a.aabb_test(aabb),
         Poly(p) => p.aabb_test(aabb)
      })
   }
   fn poly_test(&self, poly: &Poly) -> bool {
      shape_match!(&self => {
         Circle(c) => c.poly_test(poly),
         Aabb(a) => a.poly_test(poly),
         Poly(p) => p.poly_test(poly)
      })
   }
}


#[cfg(test)]
mod tests {
   use cgmath::{vec2};
   use super::*;

   #[test]
   fn line_seg_test() {
      assert_eq!(seg_seg_query(vec2(0.0, 0.0), vec2(3.0, 1.0), vec2(2.0, 1.0), vec2(2.0, -4.0)), Some(2.0 / 3.0));
      assert_eq!(seg_seg_query(vec2(3.0, 1.0), vec2(0.0, 0.0), vec2(2.0, 1.0), vec2(2.0, -4.0)), Some(1.0 / 3.0));
   }

   #[test]
   fn circle_seg_test() {
      let c = Circle::new(0.5, 0.5, 0.5);
      
      assert_eq!(c.line_query(vec2(0.0, 1.0), vec2(3.0, 1.0)).is_some(), false);
      assert_eq!(c.line_query(vec2(3.0, 1.0), vec2(0.0, 1.0)).is_some(), false);
      assert_eq!(c.line_query(vec2(0.0, 0.0), vec2(-1.0, -1.0)).is_some(), false);
      
      assert_eq!(c.line_query(vec2(-1.0, 0.5), vec2(3.0, 0.5)), Some(0.25));
      assert_eq!(c.line_query(vec2(0.0, 0.0), vec2(3.0, 1.0)).is_some(), true);
      assert_eq!(c.line_query(vec2(3.0, 1.0), vec2(0.0, 0.0)).is_some(), true);
      assert_eq!(c.line_query(vec2(0.0, 0.9), vec2(3.0, 1.0)).is_some(), true);
   }

   #[test]
   fn test_poly_poly() {
      let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]).translate(vec2(0.5, 0.5));
      let p2 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]).translate(vec2(0.8, 0.8));

      assert_eq!(p1.poly_test(&p2), true);

      let p1 = p1.translate(vec2(0.5, 0.5));
      let p2 = p2.translate(vec2(0.8, 0.8));

      assert_eq!(p1.poly_test(&p2), false);

      // poly swept sat confirmation
      let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)])
      .translate(vec2(0.5, 0.5)).translate(vec2(1.0, 1.0));
      let p2 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)])
      .translate(vec2(2.0, 1.0)).translate(vec2(0.1, -0.2));

      assert_eq!(p1.poly_test(&p2), false);
   }
}
