//! Presolve convex 2D shape collision library.

pub mod inters;
pub mod swept;
pub mod body;
pub mod utils;



#[cfg(test)]
mod tests {
   use cgmath::vec2;
   use super::inters::*;

   // ---------------- collide tests --------------- //

   #[test]
   fn aabb_aabb() {
      let a = AABB::new(0.0, 0.0, 1.0, 1.0);
      
      assert_eq!(a.aabb_test(AABB::new(0.5, 0.5, 1.5, 0.9)), true);
      assert_eq!(a.aabb_test(AABB::new(1.0, 0.5, 1.5, 0.9)), true);
      assert_eq!(a.aabb_test(AABB::new(-2.0, -2.0, -0.1, 3.0)), false);
   }
   #[test]
   fn aabb_line_test() {
      let aabb = AABB::new(0.0, 0.0, 1.0, 1.0);

      assert_eq!(aabb.line_query(vec2(-2.0, 1.5), vec2(2.0, -0.5)), Some(vec2(0.0, 0.5)));
      assert_eq!(aabb.line_test(vec2(-2.0, 1.5), vec2(2.0, -0.5)), true);
      assert_eq!(aabb.line_test(vec2(0.5, 0.5), vec2(2.0, -0.5)), true);
      assert_eq!(aabb.line_test(vec2(-2.0, 1.5), vec2(0.5, 0.5)), true);
      assert_eq!(aabb.line_test(vec2(0.0, 0.5), vec2(2.0, -0.5)), true);
      assert_eq!(aabb.line_test(vec2(0.0, -0.5), vec2(2.0, -0.5)), false);
      assert_eq!(aabb.line_test(vec2(0.0, 1.5), vec2(2.0, 1.5)), false);
      assert_eq!(aabb.line_test(vec2(-0.1, 0.5), vec2(-1.0, -0.5)), false);
   }

   #[test]
   fn aabb_contains() {
      let s = Shape::new_aabb(AABB::new(-1.0, -1.0, 1.0, 1.0));
      assert_eq!(s.point_test(vec2(0.0, 0.0)), true);
      assert_eq!(s.point_test(vec2(1.0, 1.0)), true);
      assert_eq!(s.point_test(vec2(1.1, 1.1)), false);
   }
   #[test]
   fn circle_contains() {
      let s = Shape::new_circle(2.0, vec2(1.0, 1.0));
      assert_eq!(s.point_test(vec2(1.0, 1.0)), true);
      assert_eq!(s.point_test(vec2(3.5, 3.5)), false);
   }
   #[test]
   fn poly_contains() {
      let shape = Shape::new_poly(Poly::new(&[vec2(1.0, -0.5), vec2(0.0, 0.0), vec2(1.0, 2.5), vec2(2.3, 0.5)]));
      
      assert_eq!(shape.point_test(vec2(1.0, 1.0)), true); // inside
      assert_eq!(shape.point_test(vec2(1.0, 2.5)), true); // on vert
      assert_eq!(shape.point_test(vec2(-1.0, 0.0)), false); // outside
   }

   #[test]
   fn shape_aabb_test() {
      let bb = AABB::new(0.0, 0.0, 1.0, 1.0);
      let saabb = Shape::new_aabb(AABB::new(1.0, 1.0, 2.0, 2.0));
      let scircle = Shape::new_circle(0.5, vec2(1.5, 0.5));
      let spoly = Shape::new_poly(Poly::new(&[vec2(1.0, 1.0), vec2(2.0, 0.6), vec2(0.6, 2.0)]));

      assert_eq!(saabb.aabb_test(bb), true);
      assert_eq!(saabb.aabb_test(bb.translate(vec2(-0.1, -0.1))), false);
      assert_eq!(scircle.aabb_test(bb), true);
      assert_eq!(scircle.aabb_test(bb.translate(vec2(-0.1, -0.1))), false);
      assert_eq!(spoly.aabb_test(bb.translate(vec2(-0.1, -0.1))), false);
   }

   #[test]
   fn line_circle_test() {
      let c = Circle::new_raw(0.5, 0.5, 0.5);
      assert_eq!(c.line_query(vec2(-1.0, 1.0), vec2(1.5, 1.0)), Some(vec2(0.5, 1.0)));
      
      assert_eq!(c.line_query(vec2(-1.0, 0.5), vec2(1.5, 3.5)), None);
      assert_eq!(c.line_query(vec2(-1.0, 0.5), vec2(0.5, 0.5)), Some(vec2(0.0, 0.5)));
      assert_eq!(c.line_query(vec2(0.5, 0.5), vec2(1.5, 0.5)), None);
   }
}