use criterion::{black_box, criterion_group, criterion_main, Criterion};
use cgmath::*;
use intercept::inters::{*, swept::*};
use intercept::broadphase::*;

fn criterion_benchmark(c: &mut Criterion) {
   let p1 = Poly::new(&[vec2(0.0, 0.5), vec2(0.5, 0.2), vec2(0.5, -0.2), vec2(0.0, -0.5), vec2(-0.5, -0.2), vec2(-0.5, 0.2)]).translate(vec2(0.5, 0.5));
   let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::new_poly(p1.clone())], pos: vec2(0.0, 0.0), vel: vec2(1.0, 1.0), bullet: false, collidable: true, response: Response::Slide(1.0) };
   let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0).translate(vec2(2.0, 1.0));
   let b2 = Body { aabb: aabb, shapes: vec![Shape::new_aabb(aabb)], pos: vec2(0.2, -0.5), vel: vec2(-1.0, 0.0), bullet: false, collidable: true, response: Response::Slide(1.0) };
   
   assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

   //c.bench_function("aabb poly swept test", |b| b.iter(|| body_sweep(black_box(&b1), black_box(&b2), black_box(1.0))));
   let shape = Shape::new_poly(p1.clone());
   c.bench_function("clone test", |b| b.iter(|| Shape::clone(black_box(&shape))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);