//use criterion::black_box;
use criterion::{criterion_group, criterion_main, Criterion};


fn criterion_benchmark(c: &mut Criterion) {
    let _ = c;
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
