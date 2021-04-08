use criterion::{black_box, criterion_group, criterion_main, Criterion};
use intercept::inters::*;
use cgmath::{vec2, Vector2, InnerSpace};

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
         (aabb2.max - aabb1.min, aabb2.minx_maxy() - aabb1.minx_maxy(), aabb2.maxx_miny() - aabb1.maxx_miny())
      }
   };
   (c1c2 + diff_p1p2 - vel_f1t2).dot(vel_f1t2) < 0.0
   && (r1l2 + diff_p1p2).perp_dot(vel_f1t2) < 0.0
   && (l1r2 + diff_p1p2).perp_dot(vel_f1t2) > 0.0
}

fn criterion_benchmark(c: &mut Criterion) {
   c.bench_function("aabb swept approx", |b| b.iter(|| aabb_aabb_approx_sweep(
      black_box(vec2(2.0, 1.0)),black_box( &AABB::new(0.0, 0.0, 1.0, 1.0)), 
      black_box(&AABB::new(2.0, 1.0, 3.0, 2.0)), 
      black_box(vec2(0.0, 0.0)))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);