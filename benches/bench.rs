use criterion::{black_box, criterion_group, criterion_main, Criterion};
use cgmath::*;
use intercept::inters::*;
use intercept::swept::*;

fn criterion_benchmark(c: &mut Criterion) {
   let poly = Poly::new(&[vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]);
      let b1 = Body { aabb: poly.aabb, shapes: vec![Shape::Poly(poly)], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0), bullet: false };
      let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0).translate(vec2(2.0, 1.0));
      let b2 = Body { aabb: aabb, shapes: vec![Shape::Aabb(aabb)], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0), bullet: false };
      
      assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

   c.bench_function("aabb poly swept test", |b| b.iter(|| body_sweep(black_box(&b1), black_box(&b2), black_box(1.0))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);