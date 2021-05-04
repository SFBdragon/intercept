use criterion::{black_box, criterion_group, criterion_main, Criterion};
use cgmath::*;
use intercept::inters::*;
use intercept::swept::*;

fn criterion_benchmark(c: &mut Criterion) {
   let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]).translate(vec2(0.5, 0.5));
   let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::Poly(p1)], pos: vec2(0.0, 0.0), vel: vec2(1.0, 0.0), bullet: false };
   let p2 = Poly::new(&[vec2(0.1, 0.1), vec2(1.1, 1.1), vec2(1.0, 0.0), vec2(0.0, 1.0)]).translate(vec2(1.5, 3.0));
   let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::Poly(p2)], pos: vec2(0.1, -0.2), vel: vec2(0.0, -4.0), bullet: false };

   assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

   c.bench_function("poly swept test", |b| b.iter(|| body_sweep(black_box(&b1), black_box(&b2), black_box(1.0))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);