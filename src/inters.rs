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
pub fn point_colinearity(a: Vector2<f64>, b: Vector2<f64>, c: Vector2<f64>) -> bool {
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


// Circle

#[derive(Clone, Copy, Debug)]
pub struct Circle {
   pub rad: f64,
   pub pos: Vector2<f64>,
}

impl Circle {
   #[inline]
   pub fn new(rad: f64, pos: Vector2<f64>) -> Circle {
      Circle {
         rad: f64::abs(rad),
         pos
      }
   }
   #[inline]
   pub fn new_raw(rad: f64, posx: f64, posy: f64) -> Circle {
      Circle {
         rad: f64::abs(rad),
         pos: cgmath::vec2(posx, posy),
      }
   }

   #[inline]
   pub fn point_test(self, a: Vector2<f64>) -> bool {
      //! Return whether a circle-point intersect occurs.
      self.rad * self.rad <= (self.pos.x - a.x) * (self.pos.x - a.x) + (self.pos.y - a.y) * (self.pos.y - a.y)
   }
   pub fn line_test(self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
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
   pub fn line_query(self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
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
   pub fn circle_test(self, c: Circle) -> bool {
      //! Return whether a circle-circle intersect occurs.
      (self.rad - c.rad) * (self.rad - c.rad) <= (self.pos - c.pos).magnitude2()
   }
}


// ---------- AABB ---------- //

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
   pub fn point_test(self, point: Vector2<f64>) -> bool {
      //! Returns whether an AABB-point intersection occurs.
      point.x >= self.min.x && point.x <= self.max.x && point.y >= self.min.y && point.y <= self.max.y
   }
   pub fn line_test(self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
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
   pub fn line_query(self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
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
   pub fn aabb_test(self, other: AABB) -> bool {
      //! Returns whether an AABB-AABB intersection occurs.
      self.min.x <= other.max.x && self.max.x >= other.min.x && self.min.y <= other.max.y && self.max.y >= other.min.y
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
}


// ---------- Poly ---------- //

#[derive(Debug, Clone)]
pub struct Poly {
   pub verts: Vec<Vector2<f64>>,
   pub norms: Vec<Vector2<f64>>,
   pub center: Vector2<f64>
}

impl Poly {
   /// `verts` must form a convex polygon.
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

      let mut ordered = Vec::with_capacity(len);
      for i in 0..len { ordered.push(verts[order[i]]); }
      Poly::new_from_wound(ordered)
   }
   /// `verts` must form a convex polygon and must be pregiftwrapped clockwise.
   pub fn new_from_wound(verts: Vec<Vector2<f64>>) -> Poly {
      let len = verts.len();
      let mut norms = Vec::with_capacity(len);
      let mut center = cgmath::vec2(0.0, 0.0);
      for i in 0..len {
         norms.push(cgmath::vec2(-(verts[(i+1) % len].y - verts[i].y), verts[(i+1) % len].x - verts[i].x).normalize());
         center = center + verts[i];
      }

      Poly { verts, norms, center: center.div_element_wise(len as f64) }
   }
   
   pub fn point_test(&self, loc: Vector2<f64>) -> bool {
      (0..self.verts.len()).all(|i| point_normal_test(loc, self.verts[i], self.norms[i]))
   }
   /// Returns whether an intersection occurs between the two polygons.
   pub fn poly_test(&self, other: &Poly) -> bool {
      let len1 = self.verts.len();
      let len2 = other.verts.len();
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

   /// Returns whether a line-polygon intersection occurs.
   pub fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      let len = self.verts.len();
      (0..len-1).any(|i| line_line_test(self.verts[i], self.verts[i+1], a, b)) 
         || line_line_test(self.verts[len-1], self.verts[0], a, b)
   }
   /// Optionally returns entrypoint of `a`->`b` through `self`.
   pub fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
      let len = self.verts.len();
      for i in 0..len {
         if point_normal_test(a, self.verts[i], self.norms[i]) {
            retifsome!(line_line_query(a, b, self.verts[i], self.verts[(i+1)%len]));
         }
      }
      None
   }
   
   pub fn translate(&mut self, offset: Vector2<f64>) {
      for i in 0..self.verts.len() {
         self.verts[i] = self.verts[i] + offset;
      }
   }
}


// ---------- shape-shape inters ---------- //

pub fn aabb_circle_test(AABB { min, max }: AABB, Circle { rad, pos }: Circle) -> bool {
   if min.x <= rad + pos.x && max.x >= pos.x - rad && min.y <= pos.y + rad && max.y >= pos.y - rad {
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
pub fn poly_aabb_test(poly: &Poly, poly_aabb: AABB, aabb: AABB) -> bool {
   if aabb.aabb_test(poly_aabb) {
      let aabb_verts = [aabb.min, aabb.max, aabb.minx_maxy(), aabb.maxx_miny()];
      'axes1: for i in 0..poly.verts.len() { // seperating axis algorithm
         let n = poly.norms[i];
         let max = n.dot(poly.verts[i]) as f64;
         for v in aabb_verts.iter() {
            if max >= n.dot(*v) {
               continue 'axes1; // invalid axis if it does not seperate
            }
         }
         return false // valid axis found
      }
      true // given the aabbs intersect, the polygon must have the seperating axis
   } else {
      false
   }
}
pub fn poly_circle_test(poly: &Poly, circle: Circle) -> bool {
   let len = poly.verts.len();

   // point tests
   if (0..len).any(|i| circle.point_test(poly.verts[i])) { 
      return true
   }
   // line tests
   if (0..(len-1)).any(|i| circle.line_test(poly.verts[i], poly.verts[i+1])) || circle.line_test(poly.verts[len-1], poly.verts[0]) { 
      return true
   }
   // check if circle center is within poly
   (0..len).all(|i| point_normal_test(circle.pos, poly.verts[i], poly.norms[i]))
}


// ---------- Shape ---------- //

#[derive(Debug, Clone)]
pub enum ShapeType {
   Aabb { aabb: AABB }, // 32 bytes
   Circle { circle: Circle }, // 24 bytes
   /// Clockwise wound verticies
   Polygon { poly: Poly }, // 32 bytes
}

#[derive(Debug, Clone)]
pub struct Shape {
   pub kind: ShapeType,
   pub aabb: AABB,
}


impl Shape {
   #[inline]
   pub fn new_aabb(aabb: AABB) -> Shape {
      Shape { kind: ShapeType::Aabb { aabb }, aabb }
   }
   #[inline]
   pub fn new_circle(rad: f64, pos: Vector2<f64>) -> Shape {
      Shape { kind: ShapeType::Circle { circle: Circle::new(rad, pos) }, aabb: AABB::new(pos.x - rad, pos.y - rad, pos.x + rad, pos.y + rad) }
   }
   pub fn new_poly(poly: Poly) -> Shape {
      let (mut ix, mut iy, mut ax, mut ay) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
      for i in 0..poly.verts.len() { // calculate aabb
         if poly.verts[i].x < ix { ix = poly.verts[i].x; }
         if poly.verts[i].x > ax { ax = poly.verts[i].y; }
         if poly.verts[i].y < iy { iy = poly.verts[i].x; }
         if poly.verts[i].y > ay { ay = poly.verts[i].y; }
      }
      Shape { kind: ShapeType::Polygon { poly }, aabb: AABB::new(ix, iy, ax, ay) }
   }

   pub fn point_test(&self, loc: Vector2<f64>) -> bool {
      match &self.kind {
         &ShapeType::Aabb { .. } => self.aabb.point_test(loc),
         &ShapeType::Circle { circle: Circle { rad, pos } } => rad * rad >= (loc - pos).magnitude2(),
         ShapeType::Polygon { poly } => poly.point_test(loc),
      }
   }
   /// Returns whether a line-shape intersection occurs.
   pub fn line_test(&self, a: Vector2<f64>, b: Vector2<f64>) -> bool {
      match &self.kind {
         &ShapeType::Aabb { .. } => self.aabb.line_test(a, b),
         &ShapeType::Circle { circle } => circle.line_test(a, b),
         ShapeType::Polygon { poly } => poly.line_test(a, b), // self.aabb.aabb_test(AABB::new_safe(a.x, a.y, b.x, b.y)) && ..
      }
   }
   /// Optionally returns entrypoint of `a`->`b` through `self`.
   pub fn line_query(&self, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
      match &self.kind {
         &ShapeType::Aabb { .. } => self.aabb.line_query(a, b),
         &ShapeType::Circle { circle } => circle.line_query(a, b),
         ShapeType::Polygon { poly } => poly.line_query(a, b), // self.aabb.aabb_test(AABB::new_safe(a.x, a.y, b.x, b.y)) && ..
      }
   }
   pub fn aabb_test(&self, aabb: AABB) -> bool {
      match &self.kind {
         &ShapeType::Aabb { .. } => self.aabb.aabb_test(aabb),
         &ShapeType::Circle { circle } => aabb_circle_test(aabb, circle),
         ShapeType::Polygon { poly } => poly_aabb_test(poly, self.aabb, aabb),
      }
   }
   pub fn circle_test(&self, circle: Circle) -> bool {
      match &self.kind {
         &ShapeType::Aabb { .. } => aabb_circle_test(self.aabb, circle),
         &ShapeType::Circle { circle: c } => circle.circle_test(c),
         ShapeType::Polygon { poly } => poly_circle_test(poly, circle),
      }
   }
   pub fn shape_test(&self, other: &Shape) -> bool {
      match &self.kind {
         &ShapeType::Aabb { aabb: shape } => {
            match &other.kind {
               &ShapeType::Aabb { .. } => shape.aabb_test(self.aabb),
               &ShapeType::Circle { circle } => aabb_circle_test(shape, circle),
               ShapeType::Polygon { poly } => poly_aabb_test(poly, self.aabb, shape),
            }
         },
         &ShapeType::Circle { circle: shape } => {
            match &other.kind {
               &ShapeType::Aabb { .. } => aabb_circle_test(self.aabb, shape),
               &ShapeType::Circle { circle } => shape.circle_test(circle),
               ShapeType::Polygon { poly } => poly_circle_test(poly, shape),
            }
         },
         ShapeType::Polygon { poly: shape } => {
            match &other.kind {
               &ShapeType::Aabb { .. } => poly_aabb_test(shape, self.aabb, other.aabb),
               &ShapeType::Circle { circle } => poly_circle_test(shape, circle),
               ShapeType::Polygon { poly } => shape.poly_test(poly),
            }
         },
      }
   }
}


   /* /// Returns circle-line test result data.   
   /// Tangential ray returns `RayResult::EntryExit(pos, pos)`.
   pub fn ray_result(self, a: Vector2<f64>, b: Vector2<f64>) -> RayResult {
      let diff = a - self.pos;
      let unit = (b - a).normalize();
      let dot = unit.dot(diff);
      let x = dot * dot - (diff.magnitude2() - self.rad * self.rad);

      if x < 0.0 {
         RayResult::None
      } else if x == 0.0 {
         // one solution exists: d (len) = -dot
         let tangent = a + unit.mul_element_wise(-dot);
         RayResult::EntryExit(tangent, tangent)
      } else {
         // two solutions exist: d (len) = -dot +/- sqrt(x)  [+/- : exit/entry]
         let sqrtx = x.sqrt();
         let d = -dot - sqrtx;

         if d > 1.0 || d < -1.0 {
            RayResult::None
         } else {
            if self.point_query(b) {
               if d < 0.0 {
                  RayResult::Inside
               } else {
                  RayResult::Entry(a + unit.mul_element_wise(d))
               }
            } else {
               let dn = -dot + sqrtx;
               if d < 0.0 {
                  RayResult::Exit(a + unit.mul_element_wise(dn))
               } else {
                  RayResult::EntryExit(a + unit.mul_element_wise(d), a + unit.mul_element_wise(dn))
               }
            }
         }
      }
   } */