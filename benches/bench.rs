use criterion::{criterion_group, criterion_main, Criterion};
/*use criterion::black_box;
use intercept::inters::{*, swept::*};
use intercept::broadphase::*;
use cgmath::{ElementWise, InnerSpace, Vector2, vec2};*/

fn criterion_benchmark(c: &mut Criterion) {
   c.bench_function("circle circle swept test", |b| b.iter(|| ()));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);