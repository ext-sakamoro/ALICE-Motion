# Contributing to ALICE-Motion

## Build

```bash
cargo build
cargo build --no-default-features   # no_std check
```

## Test

```bash
cargo test
```

## Lint

```bash
cargo clippy -- -W clippy::all
cargo fmt -- --check
cargo doc --no-deps 2>&1 | grep warning
```

## Design Constraints

- **no_std core**: all modules must compile without `std`. Use fixed-size arrays, not `Vec`.
- **Zero allocation**: hot paths must not allocate. `NurbsCurve` uses `[Vec3; 16]`, not heap.
- **Compact**: `CubicBezier` = 48 bytes, `Vec3` = 12 bytes. Keep structures cache-friendly.
- **Fast sqrt**: use bit-manipulation approximation + Newton-Raphson, not `f32::sqrt()`.
- **Reciprocal multiplication**: pre-compute `1.0 / x` and multiply, avoid division in loops.
