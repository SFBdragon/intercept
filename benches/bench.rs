use criterion::{black_box, criterion_group, criterion_main, Criterion};
use intercept::inters::*;
use intercept::swept::*;
use cgmath::{vec2};

fn criterion_benchmark(c: &mut Criterion) {
   let a1 = AABB::new(0.0, 0.0, 1.0, 1.0);
   let b1 = Body { aabb: a1, shapes: vec![Shape::Aabb(a1)], pos: vec2(0.0, 0.0), vel: vec2(0.0, 0.0) };
   let c2 = Circle::new(0.5, 2.0, 2.0);
   let b2 = Body { aabb: c2.get_aabb(), shapes: vec![Shape::Circle(c2)], pos: vec2(1.0, 1.4), vel: vec2(-2.0, -2.0) };

   c.bench_function("aabb circle swept", |b| b.iter(|| a1.circle_sweep(black_box(&b1), black_box(&c2), 
   black_box(&b2), black_box(1.0))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);