# ALICE-Motion

**Procedural Motion Control — Don't send waypoints, send the trajectory equation**

> "A robot arm doesn't need 10,000 coordinates. It needs one curve."

```
Traditional:  1000-point trajectory = 12,000 bytes (xyz x f32 x 1000)
ALICE-Motion: Same trajectory       = 48 bytes (NURBS degree + knots + control points)
```

## The Problem

ALICE-Print sends G-code to 3D printers. ALICE-Edge compresses sensor data. But when a drone follows a flight path, a robot arm picks up an object, or an AGV navigates a warehouse — the controller receives **arrays of waypoints** (thousands of xyz coordinates) at high frequency.

This is raw data masquerading as instructions.

## The Solution

Instead of transmitting coordinate arrays, transmit **the mathematical description of the trajectory**:

- **Path** = NURBS/Bezier curve coefficients (compact parametric form)
- **Timing** = velocity profile as a timing law (trapezoidal or S-curve approximation)

The actuator JIT-evaluates the curve at arbitrary precision, interpolating positions at its own control frequency — decoupled from the sender's update rate.

## Architecture

```
+-------------------------------------------------------------+
|                        ALICE-Motion                         |
+-------------------------------------------------------------+
|                                                             |
|  +-----------------+   +------------------+                |
|  | Trajectory      |-->| Time Profiler    |                |
|  | (NURBS/Bezier)  |   | (Trapezoidal /   |                |
|  |                 |   |  S-Curve approx) |                |
|  +-----------------+   +------------------+                |
|          |                      |                          |
|          +----------+-----------+                          |
|                     |                                      |
|              +--------------+                              |
|              |  MotionPlan  |                              |
|              | .position(t) |                              |
|              | .velocity(t) |                              |
|              | .acceleration(t)                            |
|              | .duration()  |                              |
|              +--------------+                              |
|                                                             |
+-------------------------------------------------------------+
```

## Trajectory Representations

### NURBS Curve (Primary)

Non-Uniform Rational B-Spline — the industry standard for smooth curves with exact conics.

```
+---------------------------------------------+
|  NurbsCurve                                  |
|  |- degree: u8          (typically 2-3)     |
|  |- n_points: usize     (up to 16 points)   |
|  |- knots: [f32; 24]    (knot vector)       |
|  |- points: [Vec3; 16]  (control points)    |
|  |- weights: [f32; 16]  (rationality)       |
|                                               |
|  Builder API:                                 |
|    NurbsCurve::builder(degree)                |
|      .add_point(Vec3)                         |
|      .add_weighted_point(Vec3, weight)        |
|      .build()                                 |
+---------------------------------------------+
```

### Cubic Bezier (Lightweight)

Cubic Bezier segments chained end-to-end via `BezierSpline`. Simpler than NURBS, sufficient for most paths.

```
CubicBezier: 4 control points x 3 axes x f32 = 48 bytes per segment
BezierSpline: up to 8 chained segments (384 bytes max)

Constructors:
  CubicBezier::new(p0, p1, p2, p3)
  CubicBezier::from_endpoints(start, end)   // auto control points
```

Both curve types expose:
- `.position(t)` — point on curve at parameter t in [0, 1]
- `.velocity(t)` — first derivative (tangent direction)
- `.arc_length(subdivisions)` — approximate arc length by chord summation

`CubicBezier` additionally exposes `.acceleration(t)` and `.split(t)` (de Casteljau subdivision).

## Velocity Profiles

Two profiles are implemented. Both map a scalar distance along the path to a time coordinate.

| Profile | Parameters | Acceleration | Notes |
|---------|-----------|--------------|-------|
| `TrapezoidalProfile` | v_max, a_max, distance | Piecewise constant | Accelerate -> cruise -> decelerate. Falls back to triangle profile when distance is too short to reach v_max. |
| `SCurveProfile` | v_max, a_max, j_max, distance | Smoothed via smoothstep | Simplified approximation using the cubic smoothstep function `x * x * (3 - 2x)`. Velocity and acceleration are computed by finite differences. Not a full 7-phase jerk-continuous profile. |

Both implement the `VelocityProfile` trait:

```rust
pub trait VelocityProfile {
    fn position_at(&self, t: f32) -> f32;
    fn velocity_at(&self, t: f32) -> f32;
    fn acceleration_at(&self, t: f32) -> f32;
    fn duration(&self) -> f32;
}
```

## API Design

```rust
use alice_motion::{
    Vec3, CubicBezier, NurbsCurve,
    planner::MotionPlan,
};

// --- Option 1: Bezier path + trapezoidal profile ---

let curve = CubicBezier::from_endpoints(
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(0.5, 0.4, 0.0),
);

let plan = MotionPlan::bezier_trapezoidal(
    curve,
    0.5,   // v_max (units/s)
    2.0,   // a_max (units/s^2)
);

// Evaluate at any time t (JIT interpolation)
let pos = plan.position(0.5);
let vel = plan.velocity(0.5);
let acc = plan.acceleration(0.5);
let dur = plan.duration();

// --- Option 2: NURBS path + S-curve approximation ---

let curve = NurbsCurve::builder(3)
    .add_point(Vec3::new(0.0, 0.0, 0.0))
    .add_point(Vec3::new(0.1, 0.0, 0.05))
    .add_point(Vec3::new(0.3, 0.2, 0.05))
    .add_point(Vec3::new(0.5, 0.4, 0.0))
    .build();

let plan = MotionPlan::nurbs_scurve(
    curve,
    0.5,    // v_max (units/s)
    2.0,    // a_max (units/s^2)
    10.0,   // j_max (units/s^3)
);

// Use in a control loop
let dt = 0.001; // 1 kHz
let mut t = 0.0f32;
while t <= plan.duration() {
    let setpoint = plan.position(t);
    // send setpoint to actuator ...
    t += dt;
}
```

No serialization (`to_bytes` / `from_bytes`) is implemented. `MotionPlan` is an in-process structure only.

## Ecosystem Integration (Planned)

The feature flags `physics`, `edge`, and `ros2` are defined but have no active dependencies. All ecosystem crate integrations are planned for future implementation.

```
ALICE-Edge (sensor) ----> ALICE-Motion (trajectory) ----> Actuator
                                  |
                          ALICE-Physics (Planned)
                       (collision avoidance / SDF)
```

| Feature | Status | Description |
|---------|--------|-------------|
| `physics` | Planned | ALICE-Physics collision avoidance bridge |
| `edge` | Planned | ALICE-Edge sensor feedback bridge |
| `ros2` | Planned | ROS 2 message type bridge |

## Feature Flags

| Feature | Dependencies | Description |
|---------|-------------|-------------|
| *(default)* | None | Core trajectory math, no_std, zero alloc |
| `std` | std | Enables std-dependent features |
| `physics` | *(none yet)* | Reserved for ALICE-Physics integration (Planned) |
| `edge` | *(none yet)* | Reserved for ALICE-Edge integration (Planned) |
| `ros2` | *(none yet)* | Reserved for ROS 2 bridge (Planned) |

## Tests

84 tests across all modules, runnable with `cargo test`:

| Module | Tests |
|--------|-------|
| `vec3` | 22 |
| `bezier` | 17 |
| `nurbs` | 15 |
| `profile` | 19 |
| `planner` | 11 |
| **Total** | **84** |

## License

MIT

## Author

Moroya Sakamoto

---

*"The trajectory is the equation, not the points on it."*
