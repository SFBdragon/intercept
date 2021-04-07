use criterion::{black_box, criterion_group, criterion_main, Criterion};
use intercept::inters::*;
use cgmath::{Vector2};

macro_rules! retifsome {
   ($e:expr) => {
      if let Some(r) = $e {
         return Some(r);
      }
   };
}

fn line_query(aabb: AABB, a: Vector2<f64>, b: Vector2<f64>) -> Option<Vector2<f64>> {
   if a.x < aabb.min.x { // l/r tests
      retifsome!(line_line_query(a, b, aabb.min, aabb.minx_maxy()));
   } else if a.x > aabb.max.x {
      retifsome!(line_line_query(a, b, aabb.min, aabb.minx_maxy()));
   }
   if a.y < aabb.min.y { // t/d tests
      retifsome!(line_line_query(a, b, aabb.min, aabb.minx_maxy()));
   } else if a.y > aabb.max.y {
      retifsome!(line_line_query(a, b, aabb.min, aabb.minx_maxy()));
   }
   None
}

fn criterion_benchmark(c: &mut Criterion) {
   /*c.bench_function("sat line test", |b| b.iter(|| line_test(
      black_box(AABB::new(0.0, 0.0, 1.0, 1.0)), 
      black_box(cgmath::vec2(-0.5, -0.5)), 
      black_box(cgmath::vec2(0.5, -0.5)))));*/
   c.bench_function("old line aabb test", |b| b.iter(|| line_query(
      black_box(AABB::new(0.0, 0.0, 1.0, 1.0)), 
      black_box(cgmath::vec2(-0.5, -0.5)), 
      black_box(cgmath::vec2(0.5, 0.5)))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);