use cgmath::{ElementWise, InnerSpace, Vector2};
use crate::inters::{Shape, Intersect, AABB, Circle, Poly};

// ---------- Sweep ---------- //

fn aabb_aabb_swept(b1: &Body, a1: &AABB, b2: &Body, a2: &AABB) -> Option<(Vector2<f64>, f64)> {
   let rv = b1.vel - b2.vel;
   let dpos = b2.pos - b1.pos;
   let (dx_entry, dx_exit, dy_entry, dy_exit): (f64, f64, f64, f64);
   if rv.x >= 0.0 {
      dx_entry = dpos.x + a2.max.x - a1.min.x;
      dx_exit = dpos.x + a2.min.x - a1.max.x;
   } else {
      dx_entry = a1.max.x - dpos.x - a2.min.x;
      dx_exit = a1.min.x - dpos.x - a2.max.x;
   }
   if rv.y >= 0.0 {
      dy_entry = dpos.y + a2.max.y - a1.min.y;
      dy_exit = dpos.y + a2.min.y - a1.max.y;
   } else {
      dy_entry = a1.max.y - dpos.y - a2.min.y;
      dy_exit = a1.min.y - dpos.y - a2.max.y;
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
fn circle_circle_swept(b1: &Body, c1: &Circle, b2: &Body, c2: &Circle) -> Option<(Vector2<f64>, f64)> {
   let rv = b1.vel - b2.vel;
   let unit = rv.normalize();
   let dpos = b2.pos - b1.pos;
   let srad = c1.rad + c2.rad;

   // calculate the perpendicular (to velocity) difference in position of the circles
   let lat_diff = (unit.perp_dot(c1.pos) - unit.perp_dot(c2.pos + dpos)).abs();
   if lat_diff > srad { // check if circles are on a collision course
      return None
   }
   // calculate the difference (against velocity) in position of the circles
   let d = (unit.dot(c2.pos + dpos) - unit.dot(c1.pos)) + (srad * srad - lat_diff * lat_diff).sqrt();
   let t = d / unit.dot(rv) as f64;
   if t >= 1.0 { // if t == 1, no resolution need take place
      None
   } else {
      Some((-dpos.normalize(), t))
   }
}
fn poly_poly_swept(b1: &Body, p1: &Poly, b2: &Body, p2: &Poly) -> Option<(Vector2<f64>, f64)> {
   let mut immenence2 = f64::MAX; // distance to collision squared
   let mut imminent_norm: Vector2<f64>; // norm of collision
   
   let dpos = b2.pos - b1.pos;
   let p1l = p1.norms.len();
   let p2l = p2.norms.len();

   let rv1 = b2.vel - b1.vel;
   'p1f: for i in 0..p1l {
      let n = p1.norms[i];
      if n.dot(rv1) > 0.0 {
         // skip if normal faces a similar direction to velocity (including perp)
         continue;
      }

      let vprojd = n.dot(p1.verts[i] - dpos) as f64; // seperating axis dot
      let mut cprojs = f64::MAX; // closest dot
      let mut cprojv: usize; // closest vert index

      // SAT, store closest vert
      for v in 0..p2l {
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
      if let Some(v) = super::inters::line_line_query(cv, cv + rv1, p1.verts[i], p1.verts[(i+1)%p1l]) {
         let dist2 = (v - cv).magnitude2() as f64;
         if dist2 < immenence2 {
            immenence2 = dist2;
            imminent_norm = n;
         }
      }
   }

   let rv2 = b1.vel - b2.vel;
   'p2f: for i in 0..p2l {
      let n = p2.norms[i];
      if n.dot(rv2) > 0.0 {
         // skip if normal faces a similar direction to velocity (including perp)
         continue;
      }

      let vprojd = n.dot(p2.verts[i] + dpos) as f64; // seperating axis dot
      let mut cprojs = f64::MAX; // closest dot
      let mut cprojv: usize; // closest vert index

      // SAT, store closest vert
      for v in 0..p1l {
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
      if let Some(v) = super::inters::line_line_query(cv, cv + rv2, p2.verts[i], p2.verts[(i+1)%p2l]) {
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
      None
   }
}

fn aabb_circle_swept(b1: &Body, aabb: &AABB, b2: &Body, circle: &Circle) -> Option<(Vector2<f64>, f64)> {
   // Checks against expanded AABB, if it's a corner case, circle test to get collision point if any
   let rvb2 = b2.vel - b1.vel;
   if rvb2.x > 0.0 { // LEFT
      if let Some(result) = super::inters::line_line_query(circle.pos, circle.pos + rvb2, 
         aabb.min.sub_element_wise(circle.rad), cgmath::vec2(aabb.min.x - circle.rad, aabb.max.y + circle.rad)) {
         if result.y < aabb.min.y {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.min }).line_query(circle.pos, circle.pos + rvb2) {
               return Some(((aabb.min - result2).normalize(), (result2.x - circle.pos.x) / rvb2.x))
            } else {
               None
            }
         } else if result.y > aabb.max.y {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.minx_maxy() }).line_query(circle.pos, circle.pos + rvb2) {
               Some(((result2 - aabb.minx_maxy()).normalize(), (result2.x - circle.pos.x) / rvb2.x))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(1.0, 0.0), (result.x - circle.pos.x) / rvb2.x))
         }
      }
   } else if rvb2.x < 0.0 { // RIGHT
      if let Some(result) = super::inters::line_line_query(circle.pos, circle.pos + rvb2, 
         aabb.max.add_element_wise(circle.rad), cgmath::vec2(aabb.max.x + circle.rad, aabb.min.y - circle.rad)) {
         if result.y < aabb.min.y {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.maxx_miny() }).line_query(circle.pos, circle.pos + rvb2) {
               return Some(((aabb.maxx_miny() - result2).normalize(), (result2.x - circle.pos.x) / rvb2.x))
            } else {
               None
            }
         } else if result.y > aabb.max.y {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.max }).line_query(circle.pos, circle.pos + rvb2) {
               Some(((result2 - aabb.max).normalize(), (result2.x - circle.pos.x) / rvb2.x))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(-1.0, 0.0), (result.x - circle.pos.x) / rvb2.x))
         }
      }
   }
   if rvb2.y > 0.0 { // BOTTOM
      if let Some(result) = super::inters::line_line_query(circle.pos, circle.pos + rvb2, 
         aabb.min.sub_element_wise(circle.rad), cgmath::vec2(aabb.max.x + circle.rad, aabb.min.y - circle.rad)) {
         if result.x < aabb.min.x {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.min }).line_query( circle.pos, circle.pos + rvb2) {
               return Some(((aabb.min - result2).normalize(), (result2.y - circle.pos.y) / rvb2.y))
            } else {
               None
            }
         } else if result.x > aabb.max.x {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.maxx_miny() }).line_query(circle.pos, circle.pos + rvb2) {
               Some(((result2 - aabb.maxx_miny()).normalize(), (result2.y - circle.pos.y) / rvb2.y))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(0.0, 1.0), (result.y - circle.pos.y) / rvb2.y))
         }
      }
   } else if rvb2.x < 0.0 { // TOP
      if let Some(result) = super::inters::line_line_query(circle.pos, circle.pos + rvb2, 
         aabb.max.add_element_wise(circle.rad), cgmath::vec2(aabb.min.x - circle.rad, aabb.max.y + circle.rad)) {
         if result.x < aabb.min.x {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.minx_maxy() }).line_query(circle.pos, circle.pos + rvb2) {
               return Some(((aabb.minx_maxy() - result2).normalize(), (result2.y - circle.pos.y) / rvb2.y))
            } else {
               None
            }
         } else if result.x > aabb.max.x {
            return if let Some(result2) = (Circle { rad: circle.rad, pos: aabb.max }).line_query(circle.pos, circle.pos + rvb2) {
               Some(((result2 - aabb.max).normalize(), (result2.y - circle.pos.y) / rvb2.y))
            } else {
               None
            }
         } else {
            return Some((cgmath::vec2(0.0, -1.0), (result.y - circle.pos.y) / rvb2.y))
         }
      }
   }
   None
}
fn aabb_poly_swept(b1: &Body, aabb: &AABB, b2: &Body, poly: &Poly) -> Option<(Vector2<f64>, f64)> {
   let mut immenence2 = f64::MAX; // distance to collision squared
   let mut imminent_norm: Vector2<f64>; // norm of collision
   
   let rv1 = b1.vel - b2.vel; // relative veocity to b1/a1
   let dpos = b2.pos - b1.pos; // offset of b2 from b1
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
      let closest = cgmath::vec2(f64::MIN, 0.0);
      for v in poly.verts {
         if v.x > closest.x {
            closest = v;
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
      let closest = cgmath::vec2(f64::MAX, 0.0);
      for v in poly.verts {
         if v.x < closest.x {
            closest = v;
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
      let closest = cgmath::vec2(0.0, f64::MIN);
      for v in poly.verts { // given that the polys can be user-giftwrapped, first is not guaranteed to be topmost 
         if v.y > closest.y {
            closest = v;
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
      let closest = cgmath::vec2(0.0, f64::MAX);
      for v in poly.verts { // note: first is not guaranteed to be topmost
         if v.y < closest.y {
            closest = v;
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
fn circle_poly_swept(b1: &Body, circle: &Circle, b2: &Body, poly: &Poly) -> Option<(Vector2<f64>, f64)> {
   // perform a reasonability lateral and vertical bounding check to help guarantee a collision?

   // expand polygon with circle corners, ray test all valid normals (line adj v) ?
   // SAT test lines with extensions, using eliminate and checking criteria

   // if n.dot(a) > n.dot(vi) && n.dot(b) < n.dot(vi)
   //  diff.dot(intersection) =  diff.cross(verts[i] - a) / diff.dot(norms[i]);
   // if diffdot > 1.0 do circle test against vi+1
   // if diffdot < 0.0 do circle test against vi
   // both successes return, !failures do not quit!

   panic!();
}

pub trait Sweep {
   fn get_broad_aabb(&self, b: &Body); // todo: maybe change to compound vel?

   fn circle_sweep(&self, b1: &Body, circle: &Circle, b2: &Body);
   fn aabb_sweep(&self, b1: &Body, aabb: &AABB, b2: &Body);
   fn poly_sweep(&self, b1: &Body, poly: &Poly, b2: &Body);

   // fn shape_sweep(&self, b1: &Body, shape: &Shape, b2: &Body);
}

// todo: create impl for aabb/circle/poly /shape (then inline into body-body?)


// ---------- Bodies ---------- //

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
      for s in shapes { // todo: impl shape/trait aabb fetch
         if verts[i].x < ix { ix = verts[i].x; }
         if verts[i].x > ax { ax = verts[i].y; }
         if verts[i].y < iy { iy = verts[i].x; }
         if verts[i].y > ay { ay = verts[i].y; }
      }
      panic!(); // todo aabb create
      Body { pos: v2f64_zero(), vel: v2f64_zero(), aabb, shapes }
   }
}

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

pub fn body_body_swept(b1: &Body, b2: &Body) -> Option<SweptDynStatData> {
   let mut result: Option<SweptDynStatData> = None;
   let mut travel = f64::MAX;

   let v2 = b1.vel.magnitude2() as f64;
   for (x, s1) in b1.shapes.iter().enumerate() {
      for (y, s2) in b2.shapes.iter().enumerate() {
         let rev_norm = false;
         if let Some((mut n, t)) = match s1 {
            Shape::Aabb(aabb) => match s2 {
               Shape::Aabb(aabb2) => aabb_aabb_swept(b1, aabb, b2, aabb2),
               Shape::Circle(c) => aabb_circle_swept(b1, aabb, b2, c),
               Shape::Poly(poly) => aabb_poly_swept(b1, aabb, b2, poly),
            },
            Shape::Circle(c) => match s2 {
               Shape::Aabb(aabb) => { rev_norm = true; aabb_circle_swept(b2, aabb, b1, c) }
               Shape::Circle(c2) => circle_circle_swept(b1, c, b2, c2),
               Shape::Poly(poly) => circle_poly_swept(b1, c, b2, poly),
            },
            Shape::Poly(poly) => match s2 {
               Shape::Aabb(aabb) => { rev_norm = true; aabb_poly_swept(b2, aabb, b1, poly) }
               Shape::Circle(c) => { rev_norm = true; circle_poly_swept(b2, c, b1, poly) }
               Shape::Poly(poly2) => poly_poly_swept(b1, poly, b2, poly2),
            }
         } {
            if rev_norm {
               n = -n;
            }
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
   result
}
