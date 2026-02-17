# ALICE-Motion

**Procedural Motion Control — Don't send waypoints, send the trajectory equation**

> "A robot arm doesn't need 10,000 coordinates. It needs one curve."

```
Traditional:  1000-point trajectory = 12,000 bytes (xyz × f32 × 1000)
ALICE-Motion: Same trajectory       = 48 bytes (NURBS degree + knots + control points)
```

## The Problem

ALICE-Print sends G-code to 3D printers. ALICE-Edge compresses sensor data. But when a drone follows a flight path, a robot arm picks up an object, or an AGV navigates a warehouse — the controller receives **arrays of waypoints** (thousands of xyz coordinates) at high frequency.

This is raw data masquerading as instructions.

## The Solution

Instead of transmitting coordinate arrays, transmit **the mathematical description of the trajectory**:

- **Path** = NURBS/Bezier curve coefficients (compact parametric form)
- **Timing** = velocity profile as a differential equation (trapezoidal/S-curve/quintic)
- **Constraints** = max acceleration, jerk limits, obstacle avoidance zones as SDF

The actuator JIT-evaluates the curve at arbitrary precision, interpolating positions at its own control frequency — decoupled from the sender's update rate.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ALICE-Motion                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────────────┐  │
│  │ Trajectory     │──▶│ Time Profiler │──▶│ Actuator Interface    │  │
│  │ Compiler       │   │ (velocity/    │   │ (step/servo/BLDC)     │  │
│  │ NURBS/Bezier   │   │  accel/jerk)  │   │                       │  │
│  └───────────────┘   └───────────────┘   └───────────────────────┘  │
│          ▲                    ▲                      │               │
│          │                    │                      ▼               │
│  ┌───────────────┐   ┌───────────────┐   ┌───────────────────────┐  │
│  │ Path Planner  │   │ Constraint    │   │ Feedback Controller   │  │
│  │ (A* on SDF    │   │ Solver        │   │ (PID / model-based)   │  │
│  │  obstacle map)│   │ (jerk ≤ J_max)│   │                       │  │
│  └───────────────┘   └───────────────┘   └───────────────────────┘  │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

## Trajectory Representations

### NURBS Curve (Primary)

Non-Uniform Rational B-Spline — the industry standard for smooth curves with exact conics.

```
┌─────────────────────────────────────────────┐
│  NurbsTrajectory                             │
│  ├─ degree: u8          (typically 3-5)     │
│  ├─ n_control: u8       (number of points)  │
│  ├─ knots: [f32; n+p+1] (knot vector)      │
│  ├─ points: [Vec3; n]   (control points)    │
│  └─ weights: [f32; n]   (rationality)       │
│                                               │
│  Total: ~48 bytes for a typical 6-DOF move  │
│  vs 12,000 bytes as waypoint array           │
│  Ratio: 250x compression                     │
└─────────────────────────────────────────────┘
```

### Bezier Spline (Lightweight)

Cubic Bezier segments chained end-to-end. Simpler than NURBS, sufficient for most paths.

```
Segment: 4 control points × 3 axes × f32 = 48 bytes
Typical path: 3-5 segments = 144-240 bytes
```

### Quintic Polynomial (Time-Optimal)

5th-degree polynomial in time — guarantees continuous position, velocity, acceleration, and jerk.

```
q(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
Coefficients: 6 × f32 per axis × 3 axes = 72 bytes per segment
```

## Velocity Profiles

| Profile | Params | Continuity | Use Case |
|---------|--------|-----------|----------|
| Trapezoidal | v_max, a_max | C⁰ (velocity discontinuity) | Simple pick-and-place |
| S-Curve | v_max, a_max, j_max | C¹ (smooth acceleration) | Industrial robot arms |
| Quintic | boundary conditions | C² (smooth jerk) | Precision assembly |
| Time-Optimal | torque limits per joint | C¹ + torque-bounded | Racing drones |

## Packet Format

```
┌──────────────────────────────────────────────┐
│  MotionPacket (variable, typically 48-256 B) │
│  ├─ magic: [u8; 4]    = "AMOT"             │
│  ├─ type: u8           = NURBS/Bezier/Poly  │
│  ├─ axes: u8           = 3 (xyz) or 6 (xyzrpy) │
│  ├─ duration_ms: u32   = total move time    │
│  ├─ profile: u8        = Trap/S-Curve/Quint │
│  ├─ constraints: [f32; 3] = v/a/j limits    │
│  └─ curve_data: [u8; N]  = coefficients     │
└──────────────────────────────────────────────┘
```

### Size Comparison

| Motion | Waypoint Array | ALICE-Motion | Ratio |
|--------|---------------|-------------|-------|
| Linear move (1m) | 4,000 B (1000 pts) | **24 bytes** (start + end + profile) | **167x** |
| Arc move (90°) | 12,000 B (1000 pts) | **48 bytes** (NURBS arc) | **250x** |
| Complex path (S-curve) | 48,000 B (4000 pts) | **192 bytes** (4 NURBS segments) | **250x** |
| 6-DOF robot motion | 96,000 B (4000 × 6) | **384 bytes** (6-axis NURBS) | **250x** |
| Drone flight plan (10 min) | 7.2 MB (100Hz × 6DOF) | **2 KB** (spline + waypoints) | **3,600x** |

## API Design

```rust
use alice_motion::{
    Trajectory, NurbsCurve, VelocityProfile,
    MotionPlanner, Actuator,
};

// Define trajectory (48 bytes)
let trajectory = NurbsCurve::new()
    .degree(3)
    .add_point([0.0, 0.0, 0.0])    // start
    .add_point([0.1, 0.0, 0.05])   // lift
    .add_point([0.3, 0.2, 0.05])   // cruise
    .add_point([0.5, 0.4, 0.0])    // place
    .build();

// Define velocity profile
let profile = VelocityProfile::s_curve(
    v_max: 0.5,      // m/s
    a_max: 2.0,       // m/s²
    j_max: 10.0,      // m/s³
);

// Compile motion plan
let plan = MotionPlanner::compile(&trajectory, &profile)?;

// Evaluate at any time t (JIT interpolation)
let pos = plan.position(0.5);  // halfway through
let vel = plan.velocity(0.5);
let acc = plan.acceleration(0.5);

// Serialize for transmission (48 bytes)
let packet = plan.to_bytes();

// On actuator side: reconstruct and execute
let plan = MotionPlan::from_bytes(&packet)?;
for t in (0.0..plan.duration()).step_by(0.001) { // 1kHz control loop
    let setpoint = plan.position(t);
    actuator.move_to(setpoint);
}
```

## Ecosystem Integration

```
ALICE-Edge (sensor) ───▶ ALICE-Motion (trajectory) ───▶ Actuator
      │                         ▲                          │
      │                         │                          │
      │                  ALICE-Physics                     │
      │              (collision avoidance)                  │
      │                         ▲                          │
      ▼                         │                          ▼
ALICE-SDF (obstacle map) ──────┘                   ALICE-Sync
                                                (multi-robot sync)
```

| Bridge | Direction | Data |
|--------|-----------|------|
| Edge → Motion | Sensor feedback → PID correction | Position/force readings |
| Physics → Motion | Collision-free path planning | SDF obstacle field |
| Motion → Sync | Multi-robot trajectory coordination | Curve coefficients |
| Motion → Print | G-code as degenerate case (linear segments) | MotionPacket |

## Target Platforms

| Platform | DOF | Control Rate | Latency |
|----------|-----|-------------|---------|
| Raspberry Pi 5 | 6+ DOF | 10 kHz | < 100 µs |
| STM32H7 (Cortex-M7) | 6 DOF | 20 kHz | < 50 µs |
| ESP32-S3 | 3 DOF | 5 kHz | < 200 µs |
| x86_64 (simulation) | 32+ DOF | 100 kHz | < 10 µs |

## Feature Flags

| Feature | Dependencies | Description |
|---------|-------------|-------------|
| *(default)* | None | Core trajectory math, no_std, zero alloc |
| `std` | std | File I/O, dynamic allocation |
| `physics` | alice-physics | ALICE-Physics collision avoidance |
| `edge` | alice-edge | ALICE-Edge sensor feedback bridge |
| `ros2` | std | ROS 2 message type bridge (future) |

## License

MIT

## Author

Moroya Sakamoto

---

*"The trajectory is the equation, not the points on it."*
