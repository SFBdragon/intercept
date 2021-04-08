use cgmath::{ElementWise, InnerSpace, Vector2};

macro_rules! retifsome {
   ($e:expr) => {
      if let Some(r) = $e {
         return Some(r);
      }
   };
}


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
pub fn line_line_test(a1: Vector2<f64>, a2: Vector2<f64>, b1: Vector2<f64>, b2: Vector2<f64>) -> bool {
   //! Returns whether a line-line intersection occurs.
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
pub fn line_line_query(a1: Vector2<f64>, a2: Vector2<f64>, b1: Vector2<f64>, b2: Vector2<f64>) -> Option<Vector2<f64>> { 
   //! Optionally returns the line-line intersection point.
   let da = a2 - a1;
   let db = b2 - b1;

   let dot = da.x * db.y - db.x * da.y;
   if dot == 0.0 { return None } // guard against colinearity
   let dd = dot * dot;

   let nd1 = a1 - b1;
   let tdd = (da.x * nd1.y - da.y * nd1.x) * dot;
   if tdd < 0.0 || tdd > dd { return None }

   let udd = (db.x * nd1.y - db.y * nd1.x) * dot;
   if udd < 0.0 || udd > dd { return None }

   let t = tdd / dd;
   Some(cgmath::vec2(a1.x + t * da.x, a1.y + t * da.y))
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
   pub fn bounding_test(self, aabb: &AABB) -> bool {
      //! Performs an AABB test between `aabb` and the circle's bounding box.
      aabb.min.x <= self.rad + self.pos.x && aabb.max.x >= self.pos.x - self.rad 
      && aabb.min.y <= self.pos.y + self.rad && aabb.max.y >= self.pos.y - self.rad
   }
}

#[derive(Debug, Clone, Copy)]
pub struct AABB {
   pub min: Vector2<f64>,
   pub max: Vector2<f64>,
}
impl AABB {
   #[inline]
   pub fn new(minx: f64, miny: f64, maxx: f64, maxy: f64) -> AABB {
      assert_eq!(minx < maxx, true);
      assert_eq!(miny < maxy, true);

      AABB { 
         min: cgmath::vec2(minx, miny), 
         max: cgmath::vec2(maxx, maxy) 
      }
   }
   pub fn new_safe(ax: f64, by: f64, cx: f64, dy: f64) -> AABB {
      //! Orders minimum and maximum values.
      if ax < cx {
         if by < dy {
            AABB::new(ax, by, cx, dy)
         } else {
            AABB::new(ax, dy, cx, by)
         }
      } else {
         if by < dy {
            AABB::new(cx, by, ax, dy)
         } else {
            AABB::new(cx, dy, ax, by)
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
   pub fn translate(self, offset: Vector2<f64>) -> AABB {
      AABB { min: self.min + offset, max: self.max + offset }
   }
   
   #[inline]
   pub fn closest_vert(self, norm: Vector2<f64>) -> Vector2<f64> {
      //! Returns the vertex of the AABB that projects the furthest back from the normal (normalization not required).
      if norm.x >= 0.0 {
         if norm.y >= 0.0 {
            self.min
         } else {
            self.minx_maxy()
         }
      } else {
         if norm.y >= 0.0 {
            self.maxx_miny()
         } else {
            self.max
         }
      }
   }
}

/// A 2D convex polygon, vertices arranged clockwise - tailed with a duplicate of the first, with unit-length normals - without duplication. 
#[derive(Debug, Clone)]
pub struct Poly {
   pub aabb: AABB,
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

      let mut order = <Vec<usize>>::with_capacity(len);
      for i in 0..len { order.push(i); }
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
         a = verts[order[index]] - a;
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

      Poly { aabb: AABB::new(ix, iy, ax, ay), verts, norms }
   }

   pub fn translate(&mut self, offset: Vector2<f64>) {
      self.aabb = self.aabb.translate(offset);
      for i in 0..self.verts.len() {
         self.verts[i] = self.verts[i] + offset;
      }
   }
}


// ---------- Shape-Shape intersection tests ---------- //

fn aabb_circle_test(&AABB { min, max }: &AABB, &Circle { rad, pos }: &Circle) -> bool {
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
fn poly_aabb_test(poly: &Poly, aabb: &AABB) -> bool {
   if poly.aabb.aabb_test(aabb) {
      // given the aabbs intersect, the polygon must have the seperating axis
      for i in 0..poly.norms.len() {
         let n = poly.norms[i];
         if n.dot(aabb.closest_vert(n) - poly.verts[i]) > 0.0 {
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
   let mut found_sa = false;
   if (0..len).any(|i| circle.point_test(poly.verts[i])) { return true } // vert tests
   for i in 0..len { // pseudo SAT tests
      let n = poly.norms[i];
      let dot = n.dot(circle.pos - poly.verts[i]) as f64; // >0: SAT, <=0: NOSAT
      if dot <= 0.0 {
         continue; // axis does not seperate center
      } else {
         found_sa = true;
         if dot <= circle.rad { // if extended axis encompases center
            let x = (poly.verts[i+1] - poly.verts[i]).dot(circle.pos) as f64;
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
   !found_sa
}


// ---------- Intersect ---------- //

pub trait Intersect {
   fn get_aabb(&self) -> AABB;

   fn point_test(&self, point: Vector2<f64>) -> bool; // point test
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool; // line intersection
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>>; // line entrypoint

   fn circle_test(&self, circle: &Circle) -> bool; // circle intersection
   fn aabb_test(&self, aabb: &AABB) -> bool; // aabb intersection
   fn poly_test(&self, poly: &Poly) -> bool; // poly intersection
}

impl Intersect for Circle {
   #[inline]
   fn get_aabb(&self) -> AABB {
      AABB { min: self.pos.sub_element_wise(self.rad), max: self.pos.add_element_wise(self.rad) }
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
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
      //! Optionally returns entrypoint of `a`->`b` through `self`.
      let diff = a - self.pos;
      let unit = (b - a).normalize();
      let dot = unit.dot(diff) as f64;
      let x = dot * dot - (diff.magnitude2() - self.rad * self.rad);

      if x < 0.0 {
         None
      } else if x == 0.0 {
         Some(a + unit.mul_element_wise(-dot))
      } else {
         let d = -dot - x.sqrt();
         if d <= 1.0 && d >= 0.0 {
            Some(a + unit.mul_element_wise(d))
         } else {
            None
         }
      }
   }
   
   #[inline]
   fn circle_test(&self, c: &Circle) -> bool {
      (self.rad - c.rad) * (self.rad - c.rad) <= (self.pos - c.pos).magnitude2()
   }
   #[inline]
   fn aabb_test(&self, aabb: &AABB) -> bool {
       aabb_circle_test(aabb, self)
   }
   #[inline]
   fn poly_test(&self, poly: &Poly) -> bool {
      poly_circle_test(poly, self)
   }
}

impl Intersect for AABB {
   #[inline]
   fn get_aabb(&self) -> AABB {
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
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
      //! Optionally returns entrypoint of `a`->`b` through `self`.
      if a.x < self.min.x { // l/r tests
         retifsome!(line_line_query(a, b, self.min, self.minx_maxy()));
      } else if a.x > self.max.x {
         retifsome!(line_line_query(a, b, self.min, self.minx_maxy()));
      }
      if a.y < self.min.y { // t/d tests
         retifsome!(line_line_query(a, b, self.min, self.minx_maxy()));
      } else if a.y > self.max.y {
         retifsome!(line_line_query(a, b, self.min, self.minx_maxy()));
      }
      None
   }

   #[inline]
   fn circle_test(&self, circle: &Circle) -> bool {
       aabb_circle_test(self, circle)
   }
   #[inline]
   fn aabb_test(&self, other: &AABB) -> bool {
      self.min.x <= other.max.x && self.max.x >= other.min.x && self.min.y <= other.max.y && self.max.y >= other.min.y
   }
   #[inline]
   fn poly_test(&self, poly: &Poly) -> bool {
      poly_aabb_test(poly, self)
   }
}

impl Intersect for Poly {
   #[inline]
   fn get_aabb(&self) -> AABB {
      self.aabb
   }

   fn point_test(&self, loc: Vector2<f64>) -> bool {
      (0..self.norms.len()).all(|i| point_normal_test(loc, self.verts[i], self.norms[i]))
   }
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      //! Returns whether a line-polygon intersection occurs.
      let len = self.norms.len();
      (0..len).any(|i| line_line_test(self.verts[i], self.verts[i+1], a, b))
   }
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
      //! Optionally returns entrypoint of `a`->`b` through `self`.
      let len = self.norms.len();
      let line = b - a;
      for i in 0..len {
         if point_normal_test(a, self.verts[i], self.norms[i]) && self.norms[i].dot(line) < 0.0 {
            retifsome!(line_line_query(a, b, self.verts[i], self.verts[i+1]));
         }
      }
      None
   }

   #[inline]
   fn circle_test(&self, circle: &Circle) -> bool {
      poly_circle_test(self, circle)
   }
   #[inline]
   fn aabb_test(&self, aabb: &AABB) -> bool {
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

#[derive(Debug, Clone)]
pub enum Shape {
   Circle(Circle), // 24 bytes
   Aabb(AABB), // 32 bytes
   Poly(Poly), // 80 bytes
}
impl Shape {
   pub fn shape_test(&self, other: &Shape) -> bool {
      match self {
         Shape::Aabb(aabb) => {
            match other {
               Shape::Aabb(aabb2) => aabb.aabb_test(aabb2),
               Shape::Circle(c) => aabb.circle_test(c),
               Shape::Poly(poly) => aabb.poly_test(poly),
            }
         },
         Shape::Circle(c) => {
            match other {
               Shape::Aabb(aabb) => c.aabb_test(aabb),
               Shape::Circle(c2) => c.circle_test(c2),
               Shape::Poly(poly) => c.poly_test(poly),
            }
         },
         Shape::Poly(poly) => {
            match other {
               Shape::Aabb(aabb) => poly.aabb_test(aabb),
               Shape::Circle(c) => poly.circle_test(c),
               Shape::Poly(poly2) => poly.poly_test(poly2),
            }
         },
      }
   }
}
impl Intersect for Shape {
   fn get_aabb(&self) -> AABB {
      match self {
         Shape::Aabb(aabb) => aabb.get_aabb(),
         Shape::Circle(c) => c.get_aabb(),
         Shape::Poly(poly) => poly.get_aabb(),
      }
   }

   fn point_test(&self, loc: Vector2<f64>) -> bool {
      match self {
         Shape::Aabb(aabb) => aabb.point_test(loc),
         Shape::Circle(c) => c.rad * c.rad >= (loc - c.pos).magnitude2(),
         Shape::Poly(poly) => poly.point_test(loc),
      }
   }
   /// Returns whether a line-shape intersection occurs.
   fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      match self {
         Shape::Aabb(aabb) => aabb.line_test(a, b),
         Shape::Circle(c) => c.line_test(a, b),
         Shape::Poly(poly) => poly.line_test(a, b),
      }
   }
   /// Optionally returns entrypoint of `a`->`b` through `self`.
   fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
      match self {
         Shape::Aabb (aabb) => aabb.line_query(a, b),
         Shape::Circle(c) => c.line_query(a, b),
         Shape::Poly(poly) => poly.line_query(a, b),
      }
   }
   
   fn circle_test(&self, circle: &Circle) -> bool {
      match self {
         Shape::Aabb(aabb) => aabb.circle_test(circle),
         Shape::Circle(c) => c.circle_test(circle),
         Shape::Poly(poly) => poly.circle_test(circle),
      }
   }
   fn aabb_test(&self, aabb: &AABB) -> bool {
      match self {
         Shape::Aabb(aabb2) => aabb2.aabb_test(aabb),
         Shape::Circle(c)  => c.aabb_test(aabb),
         Shape::Poly(poly) => poly.aabb_test(aabb),
      }
   }
   fn poly_test(&self, poly: &Poly) -> bool {
      match self {
         Shape::Aabb(aabb) => aabb.poly_test(poly),
         Shape::Circle(c)  => c.poly_test(poly),
         Shape::Poly(poly2) => poly2.poly_test(poly),
      }
   }
}