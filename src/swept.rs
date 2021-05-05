use cgmath::{ElementWise, InnerSpace, Vector2};
use crate::inters::{Shape, Intersect, Aabb, Circle, Poly};


#[derive(Debug, Clone)]
pub struct Body {
   /// Posistion
   pub pos: Vector2<f64>,
   /// Compositing shapes
   pub shapes: Vec<Shape>,
   /// Bounding box
   pub aabb: Aabb,
   /// Velocity
   pub vel: Vector2<f64>,
   /// Whether the object is *relatively* fast moving.
   pub bullet: bool,

   // todo: collision resolution type ? phase/none/static, freeze, bounce, slide
}
impl Body {
   pub fn new(shapes: Vec<Shape>, pos: Vector2<f64>, vel: Vector2<f64>, is_bullet: bool) -> Body {
      let (mut ix, mut iy, mut ax, mut ay) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
      for s in shapes.iter() {
         let aabb = s.get_aabb();
         if aabb.min.x < ix { ix = aabb.min.x; }
         if aabb.max.x > ax { ax = aabb.max.x; }
         if aabb.min.y < iy { iy = aabb.min.y; }
         if aabb.max.y > ay { ay = aabb.max.y; }
      }
      Body { pos, vel, aabb: Aabb::new(ix, iy, ax, ay).translate(pos), shapes, bullet: is_bullet }
   }

   pub fn get_broad_aabb(&self) -> Aabb {
      self.aabb.translate(self.pos).broaden(self.vel)
   }
   pub fn get_shape_broad_aabb(&self, shape: usize) -> Aabb {
      self.shapes[shape].get_aabb().translate(self.pos).broaden(self.vel)
   }

   pub fn translate(&mut self, offset: Vector2<f64>) {
      self.pos = self.pos + offset;
      self.aabb = self.aabb.translate(offset);
   }
}


// ---------- Sweep Helper Functions ---------- //

#[inline(always)]
fn aabb_circle_approx_sweep(vel: Vector2<f64>, unit_vel: Vector2<f64>, aabb: &Aabb, o: Vector2<f64>, rad: f64) -> bool { // 4ns
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
#[inline(always)]
fn aabb_aabb_approx_sweep(vel_f1t2: Vector2<f64>, aabb1: &Aabb, aabb2: &Aabb, diff_p1p2: Vector2<f64>) -> bool { // 5ns
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
#[inline(always)]
fn line_query_mod(rv: Vector2<f64>, d_v: Vector2<f64>, d_o: Vector2<f64>) -> Option<f64> {
   //! Optimised line-line query function, assuming non-colinearity and a valid seperating axis of d_v for rv, or manual checking
   let dot = rv.x * d_v.y - d_v.x * rv.y;
   let u = (rv.x * d_o.y - rv.y * d_o.x) / dot;
   if u < 0.0 || u > 1.0 { return None }
   Some((d_v.x * d_o.y - d_v.y * d_o.x) / dot)
}


// ---------- Sweep ---------- //
// note: perf timings are for my machine specifically, as reference for a low-end, modern processor (Ryzen 3200G, stock clock speed)

fn aabb_aabb_sweep(b1: &Body, a1: &Aabb, b2: &Body, a2: &Aabb, t: f64) -> Option<(f64, Vector2<f64>)> { // ~12ns
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   if rv.x == 0.0 && rv.y == 0.0 { return None }
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
            Some((entry, cgmath::vec2(1.0, 0.0)))
         } else {
            Some((entry, cgmath::vec2(-1.0, 0.0)))
         }
      } else {
         if ty_entry > 0.0 {
            Some((entry, cgmath::vec2(0.0, 1.0)))
         } else {
            Some((entry, cgmath::vec2(0.0, -1.0)))
         }
      }
   }
}
fn circle_circle_sweep(b1: &Body, c1: &Circle, b2: &Body, c2: &Circle, t: f64) -> Option<(f64, Vector2<f64>)> { // ~11ns(f) - ~32ns(s)
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   if rv.x == 0.0 && rv.y == 0.0 { return None }
   let rv_mag = rv.magnitude() as f64;
   let unit = rv.div_element_wise(rv_mag);
   let srad = c1.rad + c2.rad;

   let c2c1 = (b1.pos + c1.pos) - (b2.pos + c2.pos);
   let dot = unit.dot(c2c1) as f64;
   let x = srad * srad + dot * dot - c2c1.magnitude2() as f64;
   if x > 0.0 {
      let d = -(dot + x.sqrt()) / rv_mag;
      if d >= 0.0 && d <= 1.0 {
         return Some((d, (c2c1 + rv.mul_element_wise(d)).div_element_wise(srad)))
      }
   }
   None
}
fn poly_poly_sweep(b1: &Body, p1: &Poly, b2: &Body, p2: &Poly, t: f64) -> Option<(f64, Vector2<f64>)> { // (6-4) ~80ns
   let rv1 = (b1.vel - b2.vel).mul_element_wise(t);
   if rv1.x == 0.0 && rv1.y == 0.0 { return None }
   let dpos = b2.pos - b1.pos;
   
   // bullets are swept-checked (extra ~5ns)
   if (b1.bullet || b2.bullet) && !aabb_aabb_approx_sweep(rv1, &p1.aabb, &p2.aabb, dpos) { return None }
   
   let rv2 = -rv1;
   let mut immenence = f64::MAX; // time to collision
   let mut imminent_norm = cgmath::vec2(0.0, 0.0); // norm of collision from 2
   
   for p1vi in 0..p1.norms.len() {
      let n = p1.norms[p1vi];
      if n.dot(rv2) >= 0.0 { continue; }
      let v = -p1.verts[p1vi];

      let diff_verts = p1.verts[p1vi+1] + v;
      let dot = rv2.perp_dot(diff_verts);
      let dd = dot * dot;
      for p2vi in 0..p2.norms.len() {
         let p2v = p2.verts[p2vi] + dpos + v;
         let desc = rv2.perp_dot(p2v) * dot;
         if desc >= 0.0 && desc <= dd { 
            let t = diff_verts.perp_dot(p2v) * dot;
            if t >= 0.0 && t <= dd && t < immenence * dd {
               immenence = t / dd;
               imminent_norm = n;
            }
         }
      }
   }
   for p2vi in 0..p2.norms.len() {
      let n = p2.norms[p2vi];
      if n.dot(rv1) >= 0.0 { continue; }
      let v = -p2.verts[p2vi];

      let diff_verts = p2.verts[p2vi+1] + v;
      let dot = rv1.perp_dot(diff_verts);
      let dd = dot * dot;
      for p1vi in 0..p1.norms.len() {
         let vc = p1.verts[p1vi] + dpos + v;
         let desc = rv1.perp_dot(vc) * dot;
         if desc >= 0.0 && desc <= dd { 
            let t = diff_verts.perp_dot(vc) * dot;
            if t >= 0.0 && t <= dd && t < immenence * dd {
               immenence = t / dd;
               imminent_norm = -n;
            }
         }
      }
   }

   if immenence < 1.0 {
      Some((immenence, imminent_norm))
   } else {
      None
   }
}

fn circle_aabb_sweep(b1: &Body, circle: &Circle, b2: &Body, aabb: &Aabb, t: f64) -> Option<(f64, Vector2<f64>)> { // ~13ns(!c) - ~34ns
   // Checks against expanded AABB, if it's a corner case, circle test to get collision point, if any (bullet checks won't be worth it as the norm of vel isn't precalculated)
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   if rv.x == 0.0 && rv.y == 0.0 { return None }
   let p1p2 = b2.pos - b1.pos;
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
               return Some(((result2.x - seg_origin.x) / rv.x, (result2 - aabb.min).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else if differentiator > aabb_height {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.minx_maxy() }).line_query(seg_origin, seg_end) {
               Some(((result2.x - seg_origin.x) / rv.x, (result2 - aabb.minx_maxy()).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else {
            return Some((-diffx / rv.x, cgmath::vec2(-1.0, 0.0)))
         }
      }
   } else if rv.x < 0.0 && seg_origin.x > aabb.max.x + circle.rad && seg_end.x < aabb.max.x + circle.rad { // RIGHT
      let diffx = seg_origin.x - (aabb.max.x + circle.rad);
      let diffy = seg_origin.y - aabb.max.y;
      let differentiator = (rv.x * diffy - rv.y * diffx) / -rv.x;
      if differentiator >= -circle.rad && differentiator < aabb_height + circle.rad {
         if differentiator < 0.0 {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.max }).line_query(seg_origin, seg_end) {
               Some(((result2.x - seg_origin.x) / rv.x, (result2 - aabb.max).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else if differentiator > aabb_height {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.maxx_miny() }).line_query(seg_origin, seg_end) {
               return Some(((result2.x - seg_origin.x) / rv.x, (result2 - aabb.maxx_miny()).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else {
            return Some((diffx / -rv.x, cgmath::vec2(1.0, 0.0)))
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
               return Some(((result2.y - seg_origin.y) / rv.y, (result2 - aabb.min).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else if differentiator > aabb_width {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.maxx_miny() }).line_query(seg_origin, seg_end) {

               Some(((result2.y - seg_origin.y) / rv.y, (result2 - aabb.maxx_miny()).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else {
            return Some((diffy / -rv.y, cgmath::vec2(0.0, -1.0)))
         }
      }
   } else if rv.y < 0.0 && seg_origin.y > aabb.max.y + circle.rad && seg_end.y < aabb.max.y + circle.rad { // TOP
      let diffx = seg_origin.x - aabb.min.x;
      let diffy = seg_origin.y - (aabb.min.y + circle.rad);
      let differentiator = (rv.x * diffy - rv.y * diffx) / rv.y;
      if differentiator >= -circle.rad && differentiator < aabb_width + circle.rad {
         if differentiator < 0.0 {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.max }).line_query(seg_origin, seg_end) {
               Some(((result2.y - seg_origin.y) / rv.y, (result2 - aabb.max).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else if differentiator > aabb_width {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.minx_maxy() }).line_query(seg_origin, seg_end) {
               return Some(((result2.y - seg_origin.y) / rv.y, (result2 - aabb.minx_maxy()).div_element_wise(circle.rad)))
            } else {
               None
            }
         } else {
            return Some((-diffy / rv.y, cgmath::vec2(0.0, 1.0)))
         }
      }
   }
   None
}
fn aabb_poly_sweep(b1: &Body, aabb: &Aabb, b2: &Body, poly: &Poly, t: f64) -> Option<(f64, Vector2<f64>)> {
   let rv1 = (b1.vel - b2.vel).mul_element_wise(t);
   if rv1.x == 0.0 && rv1.y == 0.0 { return None }
   let aabb = aabb.translate(b1.pos - b2.pos);

   // bullets should be swept-checked (extra ~5ns)
   if (b1.bullet || b2.bullet) && !aabb_aabb_approx_sweep(rv1, &aabb, &poly.aabb, cgmath::vec2(0.0, 0.0)) { return None }

   let mut immenence = f64::MAX; // time until collision
   let mut imminent_norm = cgmath::vec2(0.0, 0.0); // norm of collision

   for p2vi in 0..poly.norms.len() {
      let n = poly.norms[p2vi];
      if n.dot(rv1) >= 0.0 { continue; }
      let v = poly.verts[p2vi];
      let diff = poly.verts[p2vi+1] - v;
      if n.x >= 0.0 {
         if n.y >= 0.0 {
            if let Some(t) = line_query_mod(rv1, diff, aabb.min - v) {
               if t >= 0.0 && t < immenence {
                  immenence = t;
                  imminent_norm = n;
               }
            }
         } 
         if n.y <= 0.0 {
            if let Some(t) = line_query_mod(rv1, diff, aabb.minx_maxy() - v) {
               if t >= 0.0 && t < immenence {
                  immenence = t;
                  imminent_norm = n;
               }
            }
         }
      } 
      if n.x <= 0.0 {
         if n.y >= 0.0 {
            if let Some(t) = line_query_mod(rv1, diff, aabb.maxx_miny() - v) {
               if t >= 0.0 && t < immenence {
                  immenence = t;
                  imminent_norm = n;
               }
            }
         }
         if n.y <= 0.0 {
            if let Some(t) = line_query_mod(rv1, diff, aabb.max - v) {
               if t >= 0.0 && t < immenence {
                  immenence = t;
                  imminent_norm = n;
               }
            }
         }
      }
   }

   let rv2 = -rv1;
   if rv2.x > 0.0 { // LEFT
      for p2vi in 0..poly.norms.len() {
         let vo = poly.verts[p2vi] - aabb.min;
         if vo.x < 0.0 && vo.x + rv2.x > 0.0 {
            let t = -vo.y / rv2.x;
            if t >= 0.0 && t < immenence {
               immenence = t;
               imminent_norm = poly.norms[p2vi];
            }
         }
      }
   } else if rv2.x < 0.0 { // RIGHT
      for p2vi in 0..poly.norms.len() {
         let vo = poly.verts[p2vi] - aabb.max;
         if vo.x > 0.0 && vo.x + rv2.x < 0.0 {
            let t = -vo.y / rv2.x;
            if t >= 0.0 && t < immenence {
               immenence = t;
               imminent_norm = poly.norms[p2vi];
            }
         }
      }
   }
   if rv2.y > 0.0 { // BOTTOM
      for p2vi in 0..poly.norms.len() {
         let vo = poly.verts[p2vi] - aabb.min;
         if vo.y < 0.0 && vo.y + rv2.y > 0.0 {
            let t = -vo.x / rv2.y;
            if t >= 0.0 && t < immenence {
               immenence = t;
               imminent_norm = poly.norms[p2vi];
            }
         }
      }
   } else if rv2.y < 0.0 { // TOP
      for p2vi in 0..poly.norms.len() {
         let vo = poly.verts[p2vi] - aabb.min;
         if vo.y < 0.0 && vo.y + rv2.y > 0.0 {
            let t = -vo.x / rv2.y;
            if t >= 0.0 && t < immenence {
               immenence = t;
               imminent_norm = poly.norms[p2vi];
            }
         }
      }
   }

   if immenence < 1.0 {
      Some((immenence, imminent_norm))
   } else {
      None
   }
}
fn circle_poly_sweep(b1: &Body, circle: &Circle, b2: &Body, poly: &Poly, t: f64) -> Option<(f64, Vector2<f64>)> { // 18(s)-36(s)ns (n=6)
   let rv = (b1.vel - b2.vel).mul_element_wise(t);
   if rv.x == 0.0 && rv.y == 0.0 { return None }
   let rv_mag = rv.magnitude() as f64;
   let unit_rv = rv.div_element_wise(rv_mag);
   let o = b1.pos - b2.pos + circle.pos;

   // the expensive operations must be performed regardless, thus bullets should be swept-checked (extra ~4ns)
   if (b1.bullet || b2.bullet) && !aabb_circle_approx_sweep(rv, unit_rv, &poly.aabb, o, circle.rad) {
      return None
   }
   
   let mut closest_vert = usize::MAX;
   let mut closest_dist = f64::MAX;
   let b = o + rv;
   let cr2 = circle.rad * circle.rad;
   for i in 0..poly.norms.len() {
      let n = poly.norms[i];
      let v = poly.verts[i];
      let vo = o - v;

      let vo_mag2 = vo.magnitude2() as f64;
      if vo_mag2 < closest_dist {
         closest_dist = vo_mag2;
         closest_vert = i;
      }

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
               let x = udotvo * udotvo + cr2 - vo_mag2 as f64;
               if x > 0.0 { // tangential and passing lines are not collisions
                  let d = -udotvo - x.sqrt();
                  return if d < rv_mag { // impale/fallshort : success/guaranteed failure
                     Some((d, (unit_rv.mul_element_wise(d) - vo).div_element_wise(circle.rad)))
                  } else {
                     None
                  }
               }
            } else if t <= 1.0 {
               return Some((vo.perp_dot(diff) / diff.perp_dot(rv), n))
            } else {
               let v1o = o - poly.verts[i+1];
               let udotv1o = unit_rv.dot(v1o) as f64;
               let x = udotv1o * udotv1o + cr2 - v1o.magnitude2() as f64;
               if x > 0.0 {
                  let d = -udotv1o - x.sqrt();
                  return if d < rv_mag {
                     Some((d, (unit_rv.mul_element_wise(d) - v1o).div_element_wise(circle.rad)))
                  } else {
                     None
                  }
               }
            }
         }
      }
   }
   // perform a circle check to ensure against corner deadzones
   let vo = o - poly.verts[closest_vert];
   let udotvo = (-unit_rv).dot(vo) as f64;
   let x = udotvo * udotvo + cr2 - vo.magnitude2() as f64;
   if x > 0.0 { // tangential and passing lines are not collisions
      let d = -udotvo - x.sqrt();
      return if d < rv_mag { // impale/fallshort : success/guaranteed failure
         Some((d, ((-unit_rv).mul_element_wise(d) - vo).div_element_wise(circle.rad)))
      } else {
         None
      }
   }
   None // circle is inside poly, no collision occurs
}


#[derive(Debug, Clone)]
pub struct BodySweptData {
   /// Index of the colliding shape of body.
   pub b1_shape: usize,
   /// Index of the colliding shape of stat.
   pub b2_shape: usize,
   /// Fraction of bodys' velocity until collision.
   pub travel: f64,
   /// Collision normal from shape of b2.
   pub norm: Vector2<f64>,
}

macro_rules! inv_norm {
   ($e:expr) => {
      if let Some(r) = $e {
         Some((r.0, -r.1))
      } else {
         None
      }
   };
}
pub fn shape_sweep(b1: &Body, s1: usize, b2: &Body, s2: usize, t: f64) -> Option<(f64, Vector2<f64>)> {
   let s1s = &b1.shapes[s1];
   let s2s = &b2.shapes[s2];
   match s1s {
      Shape::Circle(c) => match s2s {
         Shape::Circle(c2) => circle_circle_sweep(b1, c, b2, c2, t),
         Shape::Aabb(aabb) => circle_aabb_sweep(b1, c, b2, aabb, t),
         Shape::Poly(poly) => circle_poly_sweep(b1, c, b2, poly, t),
      },
      Shape::Aabb(aabb) => match s2s {
         Shape::Circle(c) => inv_norm!(circle_aabb_sweep(b2, c, b1, aabb, t)),
         Shape::Aabb(aabb2) => aabb_aabb_sweep(b1, aabb, b2, aabb2, t),
         Shape::Poly(poly) => aabb_poly_sweep(b1, aabb, b2, poly, t),
      },
      Shape::Poly(poly) => match s2s {
         Shape::Circle(c) => inv_norm!(circle_poly_sweep(b2, c, b1, poly, t)),
         Shape::Aabb(aabb) => inv_norm!(aabb_poly_sweep(b2, aabb, b1, poly, t)),
         Shape::Poly(poly2) => poly_poly_sweep(b1, poly, b2, poly2, t),
      },
   }
}
pub fn body_sweep(b1: &Body, b2: &Body, t: f64) -> Option<BodySweptData> {
   if b1.get_broad_aabb().aabb_test(&b2.get_broad_aabb()) {
      if b1.shapes.len() == 1 && b2.shapes.len() == 1 { // single shape optimisation
         if let Some((t, n)) = shape_sweep(b1, 0, b2, 0, t) {
            Some(BodySweptData {
               b1_shape: 0,
               b2_shape: 0,
               travel: t,
               norm: n,
            })
         } else {
            None
         }
      } else {
         let mut result: Option<BodySweptData> = None;
         let mut travel = f64::MAX;
         let b1_aabb = &b1.get_broad_aabb();
         let b2_aabb = &b2.get_broad_aabb();

         let mut f2 = Vec::new();
         f2.extend((0..b2.shapes.len()).filter(|y| b2.get_shape_broad_aabb(*y).aabb_test(b1_aabb)));

         for x in (0..b1.shapes.len()).filter(|x| b1.get_shape_broad_aabb(*x).aabb_test(b2_aabb)) {
            for y in f2.iter() {
               if let Some((t, n)) = shape_sweep(b1, x, b2, *y, t) {
                  if t < travel {
                     travel = t;
                     result = Some(BodySweptData {
                        b1_shape: x,
                        b2_shape: *y,
                        norm: n,
                        travel: t,
                     })
                  }
               }
            }
         }
         result
      }
   } else {
      None
   }
}

#[cfg(test)]
mod tests {
   use cgmath::vec2;
   use super::*;

   #[test]
   fn aabb_circle_approx_sweep_test() {
      let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0);
      let circle = Circle::new(1.0, 2.0, 2.0);
      assert_eq!(aabb_circle_approx_sweep(vec2(0.0, 1.0), vec2(0.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), false);
      assert_eq!(aabb_circle_approx_sweep(vec2(-1.0, 1.0), vec2(-1.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), false);

      
      assert_eq!(aabb_circle_approx_sweep(vec2(1.0, 1.0), vec2(1.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), true);
      assert_eq!(aabb_circle_approx_sweep(vec2(-1.0, -1.0), vec2(-1.0, -1.0).normalize(), &aabb, circle.pos, circle.rad), true); // does not detect backward passes
      assert_eq!(aabb_circle_approx_sweep(vec2(-5.0, 0.0), vec2(-1.0, 0.0).normalize(), &aabb, circle.pos - vec2(4.0, 2.1), circle.rad), true);
   }

   #[test]
   fn aabb_aabb_approx_sweep_test() {
      let aabb1 = Aabb::new(0.0, 0.0, 1.0, 1.0);
      let aabb2 = Aabb::new(2.0, 1.0, 3.0, 2.0);

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
      let aabb1 = Aabb::new(0.0, 0.0, 1.0, 1.0);
      let b1 = Body { aabb: aabb1, shapes: vec![Shape::Aabb(aabb1)], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0), bullet: false };
      let aabb2 = Aabb::new(2.0, 1.0, 3.0, 2.0);
      let b2 = Body { aabb: aabb2, shapes: vec![Shape::Aabb(aabb2)], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0), bullet: false };

      assert_eq!(aabb_aabb_sweep(&b1, &aabb1, &b2, &aabb2, 1.0).is_some(), true);
      assert_eq!(aabb_aabb_sweep(&b1, &aabb1, &b2, &aabb2, 100.0).is_some(), true);
      assert_eq!(aabb_aabb_sweep(&b1, &aabb1, &b2, &aabb2, -1.0).is_some(), false);
   }

   #[test]
   fn circle_circle_swept_test() {
      let c1 = Circle::new(0.5, 0.0, 0.0);
      let b1 = Body { aabb: c1.get_aabb(), shapes: vec![Shape::Circle(c1)], pos: vec2(1.0, 1.0), vel: vec2(1.0, 1.0), bullet: false };
      let c2 = Circle::new(0.1, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0), bullet: false };

      println!("{:?}", circle_circle_sweep(&b1, &c1, &b2, &c2, 1.0));
      assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 1.0).is_some(), true);
      println!("{:?}", body_sweep(&b1, &b2, 1.0));
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
      assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 0.5).is_some(), false);
      assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, -1.0).is_some(), false);

      let c1 = Circle::new(0.5, 0.0, 0.0);
      let b1 = Body { aabb: c1.get_aabb(), shapes: vec![Shape::Circle(c1)], pos: vec2(2.0, 1.0), vel: vec2(1.0, 1.0), bullet: false };
      let c2 = Circle::new(0.1, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0), bullet: false };

      assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 1.0).is_some(), false);
      assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 0.5).is_some(), false);
      assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, -1.0).is_some(), false);
   }

   #[test]
   fn aabb_circle_swept() {
      let a1 = Aabb::new(-0.5, -0.5, 0.5, 0.5);
      let b1 = Body { aabb: a1, shapes: vec![Shape::Aabb(a1)], pos: vec2(1.0, 1.0), vel: vec2(2.0, 2.0), bullet: false };
      let c2 = Circle::new(0.5, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0), bullet: false };

      println!("{:?}", body_sweep(&b1, &b2, 1.0));
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

      let a1 = Aabb::new(0.0, 0.0, 1.0, 1.0);
      let b1 = Body { aabb: a1, shapes: vec![Shape::Aabb(a1)], pos: vec2(0.0, 0.0), vel: vec2(0.0, 0.0), bullet: false };
      let c2 = Circle::new(0.5, 2.0, 2.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(1.0, 1.4), vel: vec2(-2.0, -2.0), bullet: false };

      println!("{:?}", body_sweep(&b1, &b2, 1.0));
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
   }

   #[test]
   fn poly_circle_swept_test() { 
      // copy of circle tests, both succeed
      let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]);
      let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::Poly(p1.clone())], pos: vec2(1.0, 1.0), vel: vec2(1.0, 1.0), bullet: false };
      let c2 = Circle::new(0.1, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0), bullet: false };

      println!("{:?}", body_sweep(&b1, &b2, 1.0));
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

      let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]).translate(vec2(0.5, 0.5));
      let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::Poly(p1.clone())], pos: vec2(2.0, 1.0), vel: vec2(1.0, 1.0), bullet: false };
      let c2 = Circle::new(0.1, 2.0, 4.0);
      let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(0.2, -0.5), vel: vec2(0.0, -2.0), bullet: false };

      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), false);
      assert_eq!(body_sweep(&b1, &b2, 0.5).is_some(), false);
      assert_eq!(body_sweep(&b1, &b2, -1.0).is_some(), false);
   }

   #[test]
   fn poly_aabb_swept_test() {
      let poly = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]).translate(vec2(0.5, 0.5));
      let b1 = Body { aabb: poly.aabb, shapes: vec![Shape::Poly(poly)], pos: vec2(0.2, 0.2), vel: vec2(1.0, 0.1), bullet: false };
      let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0).translate(vec2(3.0, 0.0));
      let b2 = Body { aabb: aabb, shapes: vec![Shape::Aabb(aabb)], pos: vec2(0.1, -0.2), vel: vec2(-4.0, 0.0), bullet: false };

      dbg!(b2.clone());
      println!("{:?}", body_sweep(&b1, &b2, 0.3));
      println!("{:?}", body_sweep(&b1, &b2, 1.0));
      println!("{:?}", body_sweep(&b1, &b2, 4.0));
      assert_eq!(body_sweep(&b1, &b2, 0.3).is_some(), false);
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
      assert_eq!(body_sweep(&b1, &b2, 5.0).is_some(), true);

      let poly = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]);
      let b1 = Body { aabb: poly.aabb, shapes: vec![Shape::Poly(poly)], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0), bullet: false };
      let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0).translate(vec2(2.0, 1.0));
      let b2 = Body { aabb: aabb, shapes: vec![Shape::Aabb(aabb)], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0), bullet: false };
      
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
   }

   #[test]
   fn test_poly_sweep() {
      let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]).translate(vec2(0.5, 0.5));
      let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::Poly(p1)], pos: vec2(0.2, 0.2), vel: vec2(1.0, 0.1), bullet: false };
      let p2 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]).translate(vec2(3.0, 0.0));
      let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::Poly(p2)], pos: vec2(0.1, -0.2), vel: vec2(-4.0, 0.0), bullet: false };

      println!("{:?}", body_sweep(&b1, &b2, 0.3));
      println!("{:?}", body_sweep(&b1, &b2, 1.0));
      println!("{:?}", body_sweep(&b1, &b2, 4.0));
      assert_eq!(body_sweep(&b1, &b2, 0.3).is_some(), false);
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
      assert_eq!(body_sweep(&b1, &b2, 5.0).is_some(), true);

      let p1 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]);
      let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::Poly(p1)], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0), bullet: false };
      let p2 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]).translate(vec2(2.0, 1.0));
      let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::Poly(p2)], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0), bullet: false };
      
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

      let p1 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]);
      let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::Poly(p1)], pos: vec2(0.0, 0.0), vel: vec2(2.0, 1.0), bullet: false };
      let p2 = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]).translate(vec2(1.0, 0.0));
      let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::Poly(p2)], pos: vec2(1.0, -0.5), vel: vec2(-2.0, 0.0), bullet: false };
      
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
   }
}
