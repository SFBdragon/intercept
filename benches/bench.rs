use std::thread;

use criterion::{criterion_group, criterion_main, Criterion};
use criterion::black_box;
/*use intercept::inters::{*, swept::*};
use intercept::broadphase::*;
use cgmath::{ElementWise, InnerSpace, Vector2, vec2};*/


fn criterion_benchmark(c: &mut Criterion) {
   let x = || black_box(1) + black_box(2);
   c.bench_function("sa", |b| b.iter(|| { 
      let v = black_box(Vec::<usize>::with_capacity(black_box(100000)));
   }));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);