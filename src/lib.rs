//! # ALICE-Motion
//!
//! **Procedural Motion Control — Don't send waypoints, send the trajectory equation.**
//!
//! ALICE-Motion provides compact trajectory representation using parametric curves.
//! A full 6-DOF move fits in 48 bytes, evaluated on-the-fly at any control frequency.
//!
//! ## Modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`vec3`] | 3D vector — 12-byte `Copy` type with dot, cross, lerp, fast sqrt |
//! | [`bezier`] | Cubic Bezier curves — de Casteljau split, arc length, spline chains (max 8 segments) |
//! | [`nurbs`] | NURBS curves — Cox-de Boor basis, weighted control points, builder API |
//! | [`profile`] | Velocity profiles — trapezoidal and S-curve timing laws |
//! | [`planner`] | Motion planner — combines path (Bezier/NURBS) with velocity profile for JIT evaluation |
//!
//! ## Cargo Features
//!
//! | Feature | Default | Description |
//! |---------|---------|-------------|
//! | `std` | no | Standard library support |
//! | `ffi` | no | C FFI bindings (cdylib) for Unity / UE5 |
//! | `physics` | no | ALICE-Physics deterministic integration (future) |
//! | `edge` | no | ALICE-Edge sensor feedback bridge (future) |
//! | `ros2` | no | ROS 2 message bridge (future) |
//!
//! ## Quick Start
//!
//! ```
//! use alice_motion::{CubicBezier, MotionPlan, Vec3};
//!
//! // Define a straight-line trajectory from origin to (10, 0, 0)
//! let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
//!
//! // Pair with a trapezoidal velocity profile: v_max=2.0, a_max=4.0
//! let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);
//!
//! // JIT-evaluate position at any time
//! let pos = plan.position(plan.duration() / 2.0);
//! assert!(pos.x > 0.0);
//! assert!(plan.speed(plan.duration() / 2.0) > 0.1);
//! ```
//!
//! ## Binary Sizes
//!
//! | Structure | Size |
//! |-----------|------|
//! | `Vec3` | 12 bytes |
//! | `CubicBezier` | 48 bytes (4 control points) |
//! | `TrapezoidalProfile` | 28 bytes |
//! | `SCurveProfile` | 24 bytes |
//!
//! Author: Moroya Sakamoto

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::similar_names,
    clippy::many_single_char_names,
    clippy::module_name_repetitions,
    clippy::inline_always,
    clippy::suboptimal_flops
)]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod bezier;
#[cfg(feature = "ffi")]
pub mod ffi;
pub mod nurbs;
pub mod planner;
pub mod profile;
pub mod vec3;

pub use bezier::CubicBezier;
pub use nurbs::NurbsCurve;
pub use planner::MotionPlan;
pub use profile::{SCurveProfile, TrapezoidalProfile, VelocityProfile};
pub use vec3::Vec3;
