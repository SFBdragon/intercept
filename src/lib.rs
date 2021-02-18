use cgmath::Vector2;

/// Checks whether line segments a and b intersect, and if so, returns the `Vector<f64>` at which they do
pub fn intersects(a1: Vector2<f64>, a2: Vector2<f64>, b1: Vector2<f64>, b2: Vector2<f64>) -> Option<Vector2<f64>> { 
   let da = a2 - a1;
   let db = b2 - b1;

   let dot = da.x * db.y - da.y * db.x;
   if dot == 0.0 { return None; } // guard against colinearity

   let s = (da.x * (a1.y - b1.y) - da.y * (a1.x - b1.x)) / dot;
   if s < 0.0 || s > 1.0 { return None; }

   let t = (db.x * (a1.y - b1.y) - db.y * (a1.x - b1.x)) / dot;
   if t < 0.0 || t > 1.0 { return None; }

   Some(cgmath::vec2(a1.x + (t * da.x), a1.y + (t * da.y)))
}

pub struct AABB {
   pub min: Vector2<f64>,
   pub max: Vector2<f64>,
}

impl AABB {
   pub fn from_f64s(minx: f64, miny: f64, maxx: f64, maxy: f64) -> AABB {
      AABB { min: cgmath::vec2(minx, miny), max: cgmath::vec2(maxx, maxy) }
   }

   pub fn intersects(&self, other: &AABB) -> bool {
      self.min.x > other.max.x && self.max < other.min
   }
}



pub enum ShapeType {
   Aabb,
   Circle { radius: f64, loc: Vector2<f64> },
   /// Clockwise wound verticies
   Polygon { count: usize, verts: [Vector2<f64>; 8], normals: [Vector2<f64>; 8] }, // get normal as (-y, x), point outward for clockwise wind
   // Chain { count: usize, verts: [Vector2<f64>; 8] },
}

pub struct Shape {
   pub kind: ShapeType,
   pub aabb: AABB,
}



impl Shape {
   pub fn make_poly_from_wound(count: usize, verts: [Vector2<f64>; 8]) -> Shape {
      assert_eq!(count > 8, false);
   
      // calculate normals, assuming a clockwise wind
      let xt = count - 1;
      let mut normals = [cgmath::vec2(0.0, 0.0); 8];
      for i in 0..xt { normals[i] = cgmath::InnerSpace::normalize(verts[i] - verts[i+1]); }
      normals[xt] = cgmath::InnerSpace::normalize(verts[xt] - verts[0]);
   
      // calculate aabb
      let (mut ix, mut iy, mut ax, mut ay) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
      for i in 0..count {
         if verts[i].x < ix { ix = verts[i].x; }
         if verts[i].x > ax { ax = verts[i].y; }
         if verts[i].y < iy { iy = verts[i].x; }
         if verts[i].y > ay { ay = verts[i].y; }
      }
      
      Shape { kind: ShapeType::Polygon { count, verts, normals }, aabb: AABB::from_f64s(ix, iy, ax, ay) }
   }

   pub fn contains(&self, loc: Vector2<f64>) -> bool {
      match self.kind {
         ShapeType::Aabb => loc.x >= self.aabb.min.x && loc.x <= self.aabb.max.x && loc.y >= self.aabb.min.y && loc.y <= self.aabb.max.y, // aabb test
         ShapeType::Circle{ radius, loc: l } => {
            let dx = loc.x - l.x;
            let dy = loc.x - l.y;
            radius * radius >= dx * dx + dy * dy // pyth
         }
         ShapeType::Polygon { count, verts, normals } => {
            for i in 0..(count - 1) {
               // if the sign of normal dot loc < 0, normal is pointing away from the point
               if normals[i].x * (loc.x - verts[i].x) + normals[i].y * (loc.y - verts[i].y) < 0.0 {
                  return false
               }
            }
            if normals[count - 1].x * (loc.x - verts[count - 1].x) + normals[count - 1].y * (loc.y - verts[count - 1].y) < 0.0 {
               return false
            }
            true
         }
      }
   }
}



#[cfg(test)]
mod tests {
   use cgmath::vec2;
   use super::*;

   #[test]
   fn sect() {
      assert_eq!(intersects(vec2(0.0, 0.0), vec2(2.0, 3.0), vec2(0.0, 3.0), vec2(2.0, 0.0)), Some(vec2(1.0, 1.5)));
      assert_eq!(intersects(vec2(0.0, 3.0), vec2(2.0, 0.0), vec2(-1.0, -1.5), vec2(1.0, 0.0)), None);
   }
   #[test]
   fn aabb_contains() {
      assert_eq!(Shape::contains(&(Shape { kind: ShapeType::Aabb, aabb: AABB { min: vec2(0.0, 0.0), max: vec2(2.0, 4.0) }}), vec2(1.0, 1.0)), true);
      assert_eq!(Shape::contains(&(Shape { kind: ShapeType::Aabb, aabb: AABB { min: vec2(-2.0, -2.0), max: vec2(2.0, 4.0) }}), vec2(0.0, 5.0)), false);
   }
   #[test]
   fn circle_contains() {
      assert_eq!(Shape::contains(&(Shape { kind: ShapeType::Circle { radius: 2.0, loc: vec2(1.0, 1.0)}, aabb: AABB { min: vec2(0.0, 0.0), max: vec2(2.0, 4.0) }}), vec2(1.0, 1.0)), true);
      assert_eq!(Shape::contains(&(Shape { kind: ShapeType::Circle { radius: 2.0, loc: vec2(1.0, 1.0)}, aabb: AABB { min: vec2(0.0, 0.0), max: vec2(2.0, 4.0) }}), vec2(3.5, 3.5)), false);
   }
}