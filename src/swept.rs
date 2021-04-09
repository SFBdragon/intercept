use cgmath::{ElementWise, InnerSpace, Vector2};
use crate::inters::{Shape, Intersect, AABB, Circle, Poly};

macro_rules! inv_norm {
   ($e:expr) => {
      if let Some(r) = $e {
         Some((-r.0, r.1))
      } else {
         None
      }
   };
}

#[derive(Debug, Clone)]
pub struct Body {
   /// Posistion
   pub pos: Vector2<f64>,
   /// Compositing shapes
   pub shapes: Vec<Shape>,
   /// Bounding box
   pub aabb: AABB,
   /// Velocity
   pub vel: Vector2<f64>,
}
impl Body {
   pub fn new(shapes: Vec<Shape>) -> Body {
      let (mut ix, mut iy, mut ax, mut ay) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
      for s in shapes.iter() {
         let aabb = s.get_aabb();
         if aabb.min.x < ix { ix = aabb.min.x; }
         if aabb.max.x > ax { ax = aabb.max.x; }
         if aabb.min.y < iy { iy = aabb.min.y; }
         if aabb.max.y > ay { ay = aabb.max.y; }
      }
      Body { pos: cgmath::vec2(0.0, 0.0), vel: cgmath::vec2(0.0, 0.0), aabb: AABB::new(ix, iy, ax, ay), shapes }
   }
}


// ---------- Approximate Sweep Bounding Tests ---------- //

#[inline]
fn aabb_circle_approx_sweep(vel: Vector2<f64>, unit_vel: Vector2<f64>, aabb: &AABB, o: Vector2<f64>, rad: f64) -> bool {
   let (c, l, r) = if vel.x >= 0.0 {
      if vel.y >= 0.0 {
         (aabb.max, aabb.maxx_miny(), aabb.minx_maxy())
      } else {
         (aabb.maxx_miny(), aabb.max, aabb.min)
      }
   } else {
      if vel.y >= 0.0 {
         (aabb.minx_maxy(), aabb.min, aabb.max)
      } else {
         (aabb.min, aabb.minx_maxy(), aabb.maxx_miny())
      }
   };
   unit_vel.dot(o - c - vel) < rad
   && (l - o).perp_dot(unit_vel) < rad
   && (r - o).perp_dot(unit_vel) > -rad
}
#[inline]
fn aabb_aabb_approx_sweep(vel_f1t2: Vector2<f64>, aabb1: &AABB, aabb2: &AABB, diff_p1p2: Vector2<f64>) -> bool {
   let (c1c2, l1r2, r1l2) = if vel_f1t2.x >= 0.0 {
      if vel_f1t2.y >= 0.0 {
         (aabb2.min - aabb1.max, aabb2.maxx_miny() - aabb1.minx_maxy(), aabb2.minx_maxy() - aabb1.maxx_miny())
      } else {
         (aabb2.minx_maxy() - aabb1.maxx_miny(), aabb2.max - aabb1.max, aabb2.min - aabb1.min)
      }
   } else {
      if vel_f1t2.y >= 0.0 {
         (aabb2.minx_maxy() - aabb1.maxx_miny(), aabb2.min - aabb1.min, aabb2.max - aabb1.max)
      } else {
         (aabb2.max - aabb1.min, aabb2.minx_maxy() - aabb1.maxx_miny(), aabb2.maxx_miny() - aabb1.minx_maxy())
      }
   };
   (c1c2 + diff_p1p2 - vel_f1t2).dot(vel_f1t2) < 0.0
   && (r1l2 + diff_p1p2).perp_dot(vel_f1t2) < 0.0
   && (l1r2 + diff_p1p2).perp_dot(vel_f1t2) > 0.0
}


// ---------- Sweep ---------- //

fn aabb_aabb_swept(b1: &Body, a1: &AABB, b2: &Body, a2: &AABB, t: f64) -> Option<(Vector2<f64>, f64)> { // ~12ns
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   let p1p2 = b2.pos - b1.pos;
   let (dx_entry, dx_exit, dy_entry, dy_exit): (f64, f64, f64, f64);
   if rv.x >= 0.0 {
      dx_entry = p1p2.x + a2.min.x - a1.max.x;
      dx_exit = p1p2.x + a2.max.x - a1.min.x;
   } else {
      dx_entry = a1.min.x - p1p2.x - a2.max.x;
      dx_exit = a1.max.x - p1p2.x - a2.min.x;
   }
   if rv.y >= 0.0 {
      dy_entry = p1p2.y + a2.min.y - a1.max.y;
      dy_exit = p1p2.y + a2.max.y - a1.min.y;
   } else {
      dy_entry = a1.min.y - p1p2.y - a2.max.y;
      dy_exit = a1.max.y - p1p2.y - a2.min.y;
   }
   
   let (tx_entry, tx_exit, ty_entry, ty_exit): (f64, f64, f64, f64);
   if rv.x == 0.0 {
      tx_entry = f64::MIN;
      tx_exit = f64::MAX;
   } else {
      tx_entry = dx_entry / rv.x;
      tx_exit = dx_exit / rv.x;
   }
   if rv.y == 0.0 {
      ty_entry = f64::MIN;
      ty_exit = f64::MAX;
   } else {
      ty_entry = dy_entry / rv.y;
      ty_exit = dy_exit / rv.y;
   }
   
   if (tx_entry < 0.0 && ty_entry < 0.0) || tx_entry > 1.0 || ty_entry > 1.0 {
      return None
   }

   let entry = f64::max(tx_entry, ty_entry);
   let exit = f64::min(tx_exit, ty_exit);
   if entry > exit {
      None
   } else {
      if tx_entry > ty_entry {
         if tx_entry > 0.0 {
            Some((cgmath::vec2(1.0, 0.0), entry))
         } else {
            Some((cgmath::vec2(-1.0, 0.0), entry))
         }
      } else {
         if ty_entry > 0.0 {
            Some((cgmath::vec2(0.0, 1.0), entry))
         } else {
            Some((cgmath::vec2(0.0, -1.0), entry))
         }
      }
   }
}
fn circle_circle_swept(b1: &Body, c1: &Circle, b2: &Body, c2: &Circle, t: f64) -> Option<(Vector2<f64>, f64)> { // ~31ns
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   let rv_mag = rv.magnitude() as f64;
   let unit = rv.div_element_wise(rv_mag);
   let srad = c1.rad + c2.rad;

   /*let lat_diff = c1c2.perp_dot(unit).abs();
   if lat_diff > srad { return None } // check if circles are too far apart along vel
   let d = c1c2.dot(unit) - (srad * srad - lat_diff * lat_diff).sqrt();
   let td = d / rv.dot(unit) as f64;
   if td >= 1.0 || td < 0.0 {
      None
   } else {
      Some(((rv.mul_element_wise(td) - c1c2).div_element_wise(srad), td))
   }*/
   let c2c1 = (b1.pos + c1.pos) - (b2.pos + c2.pos);
   let dot = unit.dot(c2c1) as f64;
   let x = srad * srad + dot * dot - c2c1.magnitude2() as f64;
   let d = -(dot + x.sqrt()) / rv_mag;
   if x > 0.0 && d >= 0.0 && d <= 1.0 {
      Some(((c2c1 + rv.mul_element_wise(d)).div_element_wise(srad), d))
   } else {
      None
   }
}
fn poly_poly_swept(b1: &Body, p1: &Poly, b2: &Body, p2: &Poly, t: f64) -> Option<(Vector2<f64>, f64)> {
   let dpos = b2.pos - b1.pos;
   let rv2 = (b1.vel - b2.vel).mul_element_wise(t);
   /* if !aabb_aabb_approx_sweep(rv2, &p1.aabb, &p2.aabb, dpos) {
      println!("aabb approx tripped");
      return None
   } */
   
   let mut immenence2 = f64::MAX; // distance to collision squared
   let mut imminent_norm = cgmath::vec2(0.0, 0.0); // norm of collision
   
   let rv1 = -rv2;
   'p1f: for i in 0..p1.norms.len() {
      let n = p1.norms[i];
      if n.dot(rv1) > 0.0 {
         // skip if normal faces a similar direction to velocity (including perp)
         continue;
      }

      let vprojd = n.dot(p1.verts[i] - dpos) as f64; // seperating axis dot
      let mut cprojs = f64::MAX; // closest dot
      let mut cprojv = usize::MAX; // closest vert index

      // SAT, store closest vert
      for v in 0..p2.norms.len() {
         let proj = n.dot(p2.verts[v]) as f64;
         if proj < vprojd { // invalid seperating axis
            continue 'p1f;
         } else if proj < cprojs { // closer vert found
            cprojs = proj;
            cprojv = v;
         }
      }

      // get the time between collision of vert and line, if any
      let cv = p2.verts[cprojv] + dpos;
      if let Some(v) = super::inters::line_line_query(cv, cv + rv1, p1.verts[i], p1.verts[i+1]) {
         let dist2 = (v - cv).magnitude2() as f64;
         if dist2 < immenence2 {
            immenence2 = dist2;
            imminent_norm = n;
         }
      }
   }

   'p2f: for i in 0..p2.norms.len() {
      let n = p2.norms[i];
      if n.dot(rv2) > 0.0 {
         // skip if normal faces a similar direction to velocity (including perp)
         continue;
      }

      let vprojd = n.dot(p2.verts[i] + dpos) as f64; // seperating axis dot
      let mut cprojs = f64::MAX; // closest dot
      let mut cprojv = usize::MAX; // closest vert index

      // SAT, store closest vert
      for v in 0..p1.norms.len() {
         let proj = n.dot(p1.verts[v]) as f64;
         if proj < vprojd { // invalid seperating axis
            continue 'p2f;
         } else if proj < cprojs { // closer vert found
            cprojs = proj;
            cprojv = v;
         }
      }

      // get the time between collision of vert and line, if any
      let cv = p1.verts[cprojv] - dpos;
      if let Some(v) = super::inters::line_line_query(cv, cv + rv2, p2.verts[i], p2.verts[i+1]) {
         let dist2 = (v - cv).magnitude2() as f64;
         if dist2 < immenence2 {
            immenence2 = dist2;
            imminent_norm = -n;
         }
      }
   }

   if immenence2 != f64::MAX {
      Some((imminent_norm, immenence2.sqrt()))
   } else {
      println!("return tripped");
      None
   }
}

fn circle_aabb_swept(b1: &Body, circle: &Circle, b2: &Body, aabb: &AABB, t: f64) -> Option<(Vector2<f64>, f64)> { // ~34ns
   // Checks against expanded AABB, if it's a corner case, circle test to get collision point, if any
   let p1p2 = b2.pos - b1.pos;
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   let seg_origin = circle.pos - p1p2; // relative to aabb
   let seg_end = seg_origin + rv; // relative to aabb
   let aabb_width = aabb.max.x - aabb.min.x;
   let aabb_height = aabb.max.y - aabb.min.y;
   if rv.x > 0.0 && seg_origin.x < aabb.min.x - circle.rad && seg_end.x > aabb.min.x - circle.rad { // LEFT
      let diffx = seg_origin.x - (aabb.min.x - circle.rad);
      let diffy = seg_origin.y - aabb.min.y;
      let differentiator = (rv.x * diffy - rv.y * diffx) / rv.x;
      if differentiator >= -circle.rad && differentiator < aabb_height + circle.rad {
         if differentiator < 0.0 {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.min }).line_query(seg_origin, seg_end) {
               return Some(((result2 - aabb.min).div_element_wise(circle.rad), (result2.x - seg_origin.x) / rv.x))
            } else {
               None
            }
         } else if differentiator > aabb_height {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.minx_maxy() }).line_query(seg_origin, seg_end) {
               Some(((result2 - aabb.minx_maxy()).div_element_wise(circle.rad), (result2.x - seg_origin.x) / rv.x))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(-1.0, 0.0), -diffx / rv.x))
         }
      }
   } else if rv.x < 0.0 && seg_origin.x > aabb.max.x + circle.rad && seg_end.x < aabb.max.x + circle.rad { // RIGHT
      let diffx = seg_origin.x - (aabb.max.x + circle.rad);
      let diffy = seg_origin.y - aabb.max.y;
      let differentiator = (rv.x * diffy - rv.y * diffx) / -rv.x;
      if differentiator >= -circle.rad && differentiator < aabb_height + circle.rad {
         if differentiator < 0.0 {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.max }).line_query(seg_origin, seg_end) {
               Some(((result2 - aabb.max).div_element_wise(circle.rad), (result2.x - seg_origin.x) / rv.x))
            } else {
               None
            }
         } else if differentiator > aabb_height {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.maxx_miny() }).line_query(seg_origin, seg_end) {
               return Some(((result2 - aabb.maxx_miny()).div_element_wise(circle.rad), (result2.x - seg_origin.x) / rv.x))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(1.0, 0.0), diffx / -rv.x))
         }
      }
   }
   if rv.y > 0.0 && seg_origin.y < aabb.min.y - circle.rad && seg_end.y > aabb.min.y - circle.rad { // BOTTOM
      let diffx = seg_origin.x - aabb.min.x;
      let diffy = seg_origin.y - (aabb.min.y - circle.rad);
      let differentiator = (rv.x * diffy - rv.y * diffx) / -rv.y;
      if differentiator >= -circle.rad && differentiator < aabb_width + circle.rad {
         if differentiator < 0.0 {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.min }).line_query(seg_origin, seg_end) {
               return Some(((result2 - aabb.min).div_element_wise(circle.rad), (result2.y - seg_origin.y) / rv.y))
            } else {
               None
            }
         } else if differentiator > aabb_width {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.maxx_miny() }).line_query(seg_origin, seg_end) {

               Some(((result2 - aabb.maxx_miny()).div_element_wise(circle.rad), (result2.y - seg_origin.y) / rv.y))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(0.0, -1.0), diffy / -rv.y))
         }
      }
   } else if rv.y < 0.0 && seg_origin.y > aabb.max.y + circle.rad && seg_end.y < aabb.max.y + circle.rad { // TOP
      let diffx = seg_origin.x - aabb.min.x;
      let diffy = seg_origin.y - (aabb.min.y + circle.rad);
      let differentiator = (rv.x * diffy - rv.y * diffx) / rv.y;
      if differentiator >= -circle.rad && differentiator < aabb_width + circle.rad {
         if differentiator < 0.0 {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.max }).line_query(seg_origin, seg_end) {
               Some(((result2 - aabb.max).div_element_wise(circle.rad), (result2.y - seg_origin.y) / rv.y))
            } else {
               None
            }
         } else if differentiator > aabb_width {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.minx_maxy() }).line_query(seg_origin, seg_end) {
               return Some(((result2 - aabb.minx_maxy()).div_element_wise(circle.rad), (result2.y - seg_origin.y) / rv.y))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(0.0, 1.0), -diffy / rv.y))
         }
      }
   }
   None
}
fn aabb_poly_swept(b1: &Body, aabb: &AABB, b2: &Body, poly: &Poly, t: f64) -> Option<(Vector2<f64>, f64)> {
   let rv1 = (b1.vel - b2.vel).mul_element_wise(t); // relative veocity to b1/a1
   let dpos = b2.pos - b1.pos; // offset of b2 from b1
   if !aabb_aabb_approx_sweep(rv1, &aabb, &poly.aabb, dpos) {
      return None
   }

   let mut immenence2 = f64::MAX; // distance to collision squared
   let mut imminent_norm = cgmath::vec2(0.0, 0.0); // norm of collision
   
   let len = poly.norms.len();
   for i in 0..len {
      let n = poly.norms[i];
      if n.dot(rv1) > 0.0 { // check that norm and vel face opposite each other
         continue;
      }

      let closest = aabb.closest_vert(n) - dpos;
      if n.dot(closest) < n.dot(poly.verts[i]) { // check whether this is a seperating axis
         continue;
      }
      
      // get the time between collision of vert and line, if any
      if let Some(v) = super::inters::line_line_query(closest, closest + rv1, poly.verts[i], poly.verts[(i+1)%len]) {
         let dist2 = (v - closest).magnitude2() as f64;
         if dist2 < immenence2 {
            immenence2 = dist2;
            imminent_norm = n;
         }
      }
   }

   let rv2 = -rv1;
   if rv2.x > 0.0 { // LEFT
      let mut closest = cgmath::vec2(f64::MIN, 0.0);
      for v in poly.verts.iter() {
         if v.x > closest.x {
            closest = *v;
         }
      }

      if closest.x <= aabb.min.x - dpos.x {
         // get the time between collision of vert and line, if any
         if let Some(v) = super::inters::line_line_query(closest, closest + rv2, aabb.min, aabb.minx_maxy()) {
            let dist2 = (v - closest).magnitude2() as f64;
            if dist2 < immenence2 {
               immenence2 = dist2;
               imminent_norm = cgmath::vec2(1.0, 0.0); // reversed
            }
         }
      }
   } else if rv2.x < 0.0 { // RIGHT
      let mut closest = cgmath::vec2(f64::MAX, 0.0);
      for v in poly.verts.iter() {
         if v.x < closest.x {
            closest = *v;
         }
      }

      if closest.x >= aabb.max.x - dpos.x {
         // get the time between collision of vert and line, if any
         if let Some(v) = super::inters::line_line_query(closest, closest + rv2, aabb.maxx_miny(), aabb.max) {
            let dist2 = (v - closest).magnitude2() as f64;
            if dist2 < immenence2 {
               immenence2 = dist2;
               imminent_norm = cgmath::vec2(-1.0, 0.0); // reversed
            }
         }
      }
   }
   if rv2.y > 0.0 { // BOTTOM
      let mut closest = cgmath::vec2(0.0, f64::MIN);
      for v in poly.verts.iter() {
         if v.y > closest.y {
            closest = *v;
         }
      }

      if closest.y <= aabb.min.y - dpos.x {
         // get the time between collision of vert and line, if any
         if let Some(v) = super::inters::line_line_query(closest, closest + rv2, aabb.min, aabb.maxx_miny()) {
            let dist2 = (v - closest).magnitude2() as f64;
            if dist2 < immenence2 {
               immenence2 = dist2;
               imminent_norm = cgmath::vec2(0.0, 1.0); // reversed
            }
         }
      }
   } else if rv2.y < 0.0 { // TOP
      let mut closest = cgmath::vec2(0.0, f64::MAX);
      for v in poly.verts.iter() { // note: first is not guaranteed to be topmost
         if v.y < closest.y {
            closest = *v;
         }
      }

      if closest.y >= aabb.min.y - dpos.x {
         // get the time between collision of vert and line, if any
         if let Some(v) = super::inters::line_line_query(closest, closest + rv2, aabb.minx_maxy(), aabb.max) {
            let dist2 = (v - closest).magnitude2() as f64;
            if dist2 < immenence2 {
               immenence2 = dist2;
               imminent_norm = cgmath::vec2(0.0, -1.0); // reversed
            }
         }
      }
   }

   if immenence2 != f64::MAX {
      Some((imminent_norm, immenence2.sqrt()))
   } else {
      None
   }
}
fn circle_poly_swept(b1: &Body, circle: &Circle, b2: &Body, poly: &Poly, t: f64) -> Option<(Vector2<f64>, f64)> {
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   let rv_mag = rv.magnitude() as f64;
   let unit_rv = rv.div_element_wise(rv_mag);
   let o = b1.pos - b2.pos + circle.pos;
   /*if !aabb_circle_approx_sweep(rv, unit_rv, &poly.aabb, o, circle.rad) {
      return None
   }*/
   
   let len = poly.norms.len();
   let b = o + rv;
   let cr2 = circle.rad * circle.rad;
   for i in 0..len {
      let n = poly.norms[i];
      let v = poly.verts[i];
      let vo = o - v;
      let ndotva = n.dot(vo) as f64;
      if ndotva >= 0.0 {
         if n.dot(b - v) >= 0.0 { // fallshort/tangential
            return None
         } else { // guarantees ray-seg collision, and that n.dot(rv) != 0.0
            let diff = poly.verts[i+1] - v;
            let t = vo.perp_dot(rv) / diff.perp_dot(rv);
            if t < 0.0 {
               // modified circle collision algorithm
               // note: this will never re-test `v` due to the circle-fallshort check
               let udotvo = unit_rv.dot(vo) as f64;
               let x = udotvo * udotvo + cr2 - vo.magnitude2() as f64;
               if x > 0.0 { // tangential and passing lines are not collisions
                  let d = -udotvo - x.sqrt();
                  return if d < rv_mag { // impale/fallshort : success/guaranteed failure
                     Some(((unit_rv.mul_element_wise(d) - vo).div_element_wise(circle.rad), d))
                  } else {
                     None
                  }
               }
            } else if t <= 1.0 {
               return Some((n, vo.perp_dot(diff) / diff.perp_dot(rv)))
            } else {
               let v1o = o - poly.verts[i+1];
               let udotv1o = unit_rv.dot(v1o) as f64;
               let x = udotv1o * udotv1o + cr2 - v1o.magnitude2() as f64;
               if x > 0.0 {
                  let d = -udotv1o - x.sqrt();
                  return if d < rv_mag {
                     Some(((unit_rv.mul_element_wise(d) - v1o).div_element_wise(circle.rad), d))
                  } else {
                     None
                  }
               }
            }
         }
      }
   }
   None // circle is inside poly, no collision occurs
}

pub trait Sweep {
   fn get_broad_aabb(&self, b: &Body, t: f64) -> AABB;

   fn circle_sweep(&self, b1: &Body, circle: &Circle, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)>;
   fn aabb_sweep(&self, b1: &Body, aabb: &AABB, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)>;
   fn poly_sweep(&self, b1: &Body, poly: &Poly, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)>;

   fn shape_sweep(&self, b1: &Body, shape: &Shape, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)>;
}

impl Sweep for Circle {
   fn get_broad_aabb(&self, b: &Body, t: f64) -> AABB {
      let vel = b.vel.mul_element_wise(t);
      if vel.x >= 0.0 {
         if vel.y >= 0.0 {
            AABB { min: self.pos.sub_element_wise(self.rad), max: self.pos.add_element_wise(self.rad) + vel }
         } else {
            AABB::new(self.pos.x - self.rad, self.pos.y - self.rad + vel.y, self.pos.x + self.rad + vel.x, self.pos.y + self.rad)
         }
      } else {
         if vel.y >= 0.0 {
            AABB::new(self.pos.x - self.rad + vel.x, self.pos.y - self.rad, self.pos.x + self.rad, self.pos.y + self.rad + vel.y)
         } else {
            AABB { min: self.pos.sub_element_wise(self.rad) + vel, max: self.pos.add_element_wise(self.rad) }
         }
      }
   }

   #[inline]
   fn circle_sweep(&self, b1: &Body, circle: &Circle, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      circle_circle_swept(b1, self, b2, circle, t)
   }
   #[inline]
   fn aabb_sweep(&self, b1: &Body, aabb: &AABB, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      circle_aabb_swept(b1, self, b2, aabb, t)
   }
   #[inline]
   fn poly_sweep(&self, b1: &Body, poly: &Poly, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      circle_poly_swept(b1, self, b2, poly, t)
   }

   fn shape_sweep(&self, b1: &Body, shape: &Shape, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match shape {
         Shape::Circle(c) => self.circle_sweep(b1, c, b2, t),
         Shape::Aabb(aabb) => self.aabb_sweep(b1, aabb, b2, t),
         Shape::Poly(poly) => self.poly_sweep(b1, poly, b2, t),
      }
   }
}
impl Sweep for AABB {
   fn get_broad_aabb(&self, b: &Body, t: f64) -> AABB {
      let vel = b.vel.mul_element_wise(t);
      if vel.x >= 0.0 {
         if vel.y >= 0.0 {
            AABB { min: self.min, max: self.max + vel }
         } else {
            AABB::new(self.min.x, self.min.y + vel.y, self.max.x + vel.x, self.max.y)
         }
      } else {
         if vel.y >= 0.0 {
            AABB::new(self.min.x + vel.x, self.min.y, self.max.x, self.max.y + vel.y)
         } else {
            AABB { min: self.min + vel, max: self.max }
         }
      }
   }

   #[inline]
   fn circle_sweep(&self, b1: &Body, circle: &Circle, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      inv_norm!(circle_aabb_swept(b2, circle, b1, self, t))
   }
   #[inline]
   fn aabb_sweep(&self, b1: &Body, aabb: &AABB, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      aabb_aabb_swept(b1, self, b2, aabb, t)
   }
   #[inline]
   fn poly_sweep(&self, b1: &Body, poly: &Poly, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      aabb_poly_swept(b1, self, b2, poly, t)
   }

   fn shape_sweep(&self, b1: &Body, shape: &Shape, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match shape {
         Shape::Circle(c) => self.circle_sweep(b1, c, b2, t),
         Shape::Aabb(aabb) => self.aabb_sweep(b1, aabb, b2, t),
         Shape::Poly(poly) => self.poly_sweep(b1, poly, b2, t),
      }
   }
}
impl Sweep for Poly {
   fn get_broad_aabb(&self, b: &Body, t: f64) -> AABB {
      let vel = b.vel.mul_element_wise(t);
      if vel.x >= 0.0 {
         if vel.y >= 0.0 {
            AABB { min: self.aabb.min, max: self.aabb.max + vel }
         } else {
            AABB::new(self.aabb.min.x, self.aabb.min.y + vel.y, self.aabb.max.x + vel.x, self.aabb.max.y)
         }
      } else {
         if vel.y >= 0.0 {
            AABB::new(self.aabb.min.x + vel.x, self.aabb.min.y, self.aabb.max.x, self.aabb.max.y + vel.y)
         } else {
            AABB { min: self.aabb.min + vel, max: self.aabb.max }
         }
      }
   }

   #[inline]
   fn circle_sweep(&self, b1: &Body, circle: &Circle, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      inv_norm!(circle_poly_swept(b2, circle, b1, self, t))
   }
   #[inline]
   fn aabb_sweep(&self, b1: &Body, aabb: &AABB, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      inv_norm!(aabb_poly_swept(b2, aabb, b1, self, t))
   }
   #[inline]
   fn poly_sweep(&self, b1: &Body, poly: &Poly, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      poly_poly_swept(b1, self, b2, poly, t)
   }

   fn shape_sweep(&self, b1: &Body, shape: &Shape, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match shape {
         Shape::Circle(c) => self.circle_sweep(b1, c, b2, t),
         Shape::Aabb(aabb) => self.aabb_sweep(b1, aabb, b2, t),
         Shape::Poly(poly) => self.poly_sweep(b1, poly, b2, t),
      }
   }
}
impl Sweep for Shape {
   #[inline]
   fn get_broad_aabb(&self, b: &Body, t: f64) -> AABB {
      match self {
         Shape::Circle(c) => c.get_broad_aabb(b, t),
         Shape::Aabb(aabb) => aabb.get_broad_aabb(b, t),
         Shape::Poly(poly) => poly.get_broad_aabb(b, t),
      }
   }

   #[inline]
   fn circle_sweep(&self, b1: &Body, circle: &Circle, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match self {
         Shape::Circle(c) => c.circle_sweep(b1, circle, b2, t),
         Shape::Aabb(aabb) => aabb.circle_sweep(b1, circle, b2, t),
         Shape::Poly(poly) => poly.circle_sweep(b1, circle, b2, t),
      }
   }
   #[inline]
   fn aabb_sweep(&self, b1: &Body, aabb: &AABB, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match self {
         Shape::Circle(c) => c.aabb_sweep(b1, aabb, b2, t),
         Shape::Aabb(aabb) => aabb.aabb_sweep(b1, aabb, b2, t),
         Shape::Poly(poly) => poly.aabb_sweep(b1, aabb, b2, t),
      }
   }
   #[inline]
   fn poly_sweep(&self, b1: &Body, poly: &Poly, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match self {
         Shape::Circle(c) => c.poly_sweep(b1, poly, b2, t),
         Shape::Aabb(aabb) => aabb.poly_sweep(b1, poly, b2, t),
         Shape::Poly(poly) => poly.poly_sweep(b1, poly, b2, t),
      }
   }

   #[inline]
   fn shape_sweep(&self, b1: &Body, shape: &Shape, b2: &Body, t: f64) -> Option<(Vector2<f64>, f64)> {
      match shape {
         Shape::Circle(c) => self.circle_sweep(b1, c, b2, t),
         Shape::Aabb(aabb) => self.aabb_sweep(b1, aabb, b2, t),
         Shape::Poly(poly) => self.poly_sweep(b1, poly, b2, t),
      }
   }
}

#[derive(Debug, Clone)]
pub struct SweptDynStatData {
   /// Index of the colliding shape of body.
   pub b1_shape: usize,
   /// Index of the colliding shape of stat.
   pub b2_shape: usize,
   /// Collision normal from shape of b2.
   pub norm: Vector2<f64>,
   /// Fraction of bodys' velocity until collision.
   pub travel: f64,
}

pub fn body_body_swept(b1: &Body, b2: &Body, t: f64) -> Option<SweptDynStatData> {
   let mut result: Option<SweptDynStatData> = None;
   let mut travel = f64::MAX;
   for (x, s1) in b1.shapes.iter().enumerate() {
      for (y, s2) in b2.shapes.iter().enumerate() {
         if s1.get_broad_aabb(b1, t).aabb_test(&s2.get_broad_aabb(b2, t)) {
            if let Some((n, t)) = s1.shape_sweep(b1, s2, b2, t) {
               if t < travel {
                  travel = t;
                  result = Some(SweptDynStatData {
                     b1_shape: x,
                     b2_shape: y,
                     norm: n,
                     travel: t,
                  })
               }
            }
         }
      }
   }
   result
}

#[cfg(test)]
mod tests {
   use cgmath::vec2;
   use super::*;

   #[test]
   fn aabb_circle_approx_sweep_test() {
      let aabb = AABB::new(0.0, 0.0, 1.0, 1.0);
      let circle = Circle::new(1.0, 2.0, 2.0);
      assert_eq!(aabb_circle_approx_sweep(vec2(0.0, 1.0), vec2(0.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), false);
      assert_eq!(aabb_circle_approx_sweep(vec2(-1.0, 1.0), vec2(-1.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), false);

      
      assert_eq!(aabb_circle_approx_sweep(vec2(1.0, 1.0), vec2(1.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), true);
      assert_eq!(aabb_circle_approx_sweep(vec2(-1.0, -1.0), vec2(-1.0, -1.0).normalize(), &aabb, circle.pos, circle.rad), true); // does not detect backward passes
      assert_eq!(aabb_circle_approx_sweep(vec2(-5.0, 0.0), vec2(-1.0, 0.0).normalize(), &aabb, circle.pos - vec2(4.0, 2.1), circle.rad), true);
   }

   #[test]
   fn aabb_aabb_approx_sweep_test() {
      let aabb1 = AABB::new(0.0, 0.0, 1.0, 1.0);
      let aabb2 = AABB::new(2.0, 1.0, 3.0, 2.0);

      assert_eq!(aabb_aabb_approx_sweep(vec2(2.0, 0.0), &aabb1, &aabb2, vec2(0.0, 0.0)), false);
      assert_eq!(aabb_aabb_approx_sweep(vec2(1.0, 0.0), &aabb1, &aabb2.translate(vec2(0.0, -1.0)), vec2(0.0, 0.0)), false);
      assert_eq!(aabb_aabb_approx_sweep(vec2(2.0, 0.0), &aabb1, &aabb2.translate(vec2(0.0, 1.0)), vec2(0.0, 0.0)), false);
      assert_eq!(aabb_aabb_approx_sweep(vec2(2.0, 1.0), &aabb1, &aabb2, vec2(3.0, 0.0)), false);
      
      assert_eq!(aabb_aabb_approx_sweep(vec2(2.0, 1.0), &aabb1, &aabb2, vec2(0.0, 0.0)), true);
      assert_eq!(aabb_aabb_approx_sweep(vec2(-2.0, -1.0), &aabb1, &aabb2, vec2(0.0, 0.0)), true); // does not detect backward passes
      assert_eq!(aabb_aabb_approx_sweep(vec2(2.0, 1.1), &aabb1, &aabb2.translate(vec2(0.0, 1.0)), vec2(0.0, 0.0)), true);
   }

   #[test]
   fn aabb_aabb_swept_test() {
      let aabb1 = AABB::new(0.0, 0.0, 1.0, 1.0);
      let b1 = Body { aabb: aabb1, shapes: vec![Shape::Aabb(aabb1)], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0) };
      let aabb2 = AABB::new(2.0, 1.0, 3.0, 2.0);
      let b2 = Body { aabb: aabb2, shapes: vec![Shape::Aabb(aabb2)], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0) };

      assert_eq!(aabb_aabb_swept(&b1, &aabb1, &b2, &aabb2, 1.0).is_some(), true);
      assert_eq!(aabb_aabb_swept(&b1, &aabb1, &b2, &aabb2, 100.0).is_some(), true);
      assert_eq!(aabb_aabb_swept(&b1, &aabb1, &b2, &aabb2, -1.0).is_some(), false);
   }

   #[test]
   fn circle_circle_swept_test() {
      let c1 = Circle::new(0.5, 0.0, 0.0);
      let b1 = Body { aabb: c1.get_aabb(), shapes: vec![Shape::Circle(c1)], pos: vec2(1.0, 1.0), vel: vec2(1.0, 1.0) };
      let c2 = Circle::new(0.1, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0) };

      println!("{:?}", circle_circle_swept(&b1, &c1, &b2, &c2, 1.0));
      assert_eq!(circle_circle_swept(&b1, &c1, &b2, &c2, 1.0).is_some(), true);
      assert_eq!(circle_circle_swept(&b1, &c1, &b2, &c2, 0.5).is_some(), false);
      assert_eq!(circle_circle_swept(&b1, &c1, &b2, &c2, -1.0).is_some(), false);

      let c1 = Circle::new(0.5, 0.0, 0.0);
      let b1 = Body { aabb: c1.get_aabb(), shapes: vec![Shape::Circle(c1)], pos: vec2(2.0, 1.0), vel: vec2(1.0, 1.0) };
      let c2 = Circle::new(0.1, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0) };

      assert_eq!(circle_circle_swept(&b1, &c1, &b2, &c2, 1.0).is_some(), false);
      assert_eq!(circle_circle_swept(&b1, &c1, &b2, &c2, 0.5).is_some(), false);
      assert_eq!(circle_circle_swept(&b1, &c1, &b2, &c2, -1.0).is_some(), false);
   }

   #[test]
   fn aabb_circle_swept() {
      let a1 = AABB::new(-0.5, -0.5, 0.5, 0.5);
      let b1 = Body { aabb: a1, shapes: vec![Shape::Aabb(a1)], pos: vec2(1.0, 1.0), vel: vec2(2.0, 2.0) };
      let c2 = Circle::new(0.5, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0) };

      println!("{:?}", a1.circle_sweep(&b1, &c2, &b2, 1.0));
      assert_eq!(a1.circle_sweep(&b1, &c2, &b2, 1.0).is_some(), true);

      let a1 = AABB::new(0.0, 0.0, 1.0, 1.0);
      let b1 = Body { aabb: a1, shapes: vec![Shape::Aabb(a1)], pos: vec2(0.0, 0.0), vel: vec2(0.0, 0.0) };
      let c2 = Circle::new(0.5, 2.0, 2.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(1.0, 1.4), vel: vec2(-2.0, -2.0) };

      println!("{:?}", a1.circle_sweep(&b1, &c2, &b2, 1.0));
      assert_eq!(a1.circle_sweep(&b1, &c2, &b2, 1.0).is_some(), true);
   }

   #[test]
   fn poly_body_swept_test() {
      let poly1 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]);
      let b1 = Body { aabb: poly1.aabb, shapes: vec![Shape::Poly(poly1.clone())], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0) };
      let poly2 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]).translate(vec2(2.0, 1.0));
      let b2 = Body { aabb: poly2.aabb, shapes: vec![Shape::Poly(poly2.clone())], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0) };

      println!("{:?}", b1);
      println!("{:?}", b2);
      assert_eq!(poly_poly_swept(&b1, &poly1, &b2, &poly2, 1.0).is_some(), true);
      assert_eq!(body_body_swept(&b1, &b2, 1.0).is_some(), true);
   }

   // todo: swept tests
}
