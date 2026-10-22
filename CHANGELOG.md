# Changelog

All notable changes to ALICE-Motion will be documented in this file.

## [Unreleased]

### Fixed
- licence の file を Cargo.toml の宣言 `MIT OR Apache-2.0` に合わせた: LICENSE-MIT と LICENSE-APACHE (Apache-2.0 の全文) を置き、README と TRADEMARK_NOTICE もそれに合わせた

## [0.1.0] - 2026-02-23

### Added
- `Vec3` — 12-byte 3D vector with dot, cross, lerp, normalize, fast sqrt
- `CubicBezier` — cubic Bezier segment with de Casteljau split, arc length
- `BezierSpline` — chain of up to 8 cubic segments
- `NurbsCurve` — NURBS evaluation with Cox-de Boor basis, weighted points, builder API
- `TrapezoidalProfile` — trapezoidal velocity profile (accel-cruise-decel)
- `SCurveProfile` — S-curve velocity profile with smoothstep approximation
- `MotionPlan` — combines path + velocity profile for JIT position/velocity/acceleration queries
- `no_std` compatible core (zero heap allocation)
- 84 unit tests covering all modules
