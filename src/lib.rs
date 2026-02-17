//! ALICE-Motion — Procedural Motion Control
//!
//! Don't send waypoints, send the trajectory equation.
//!
//! Compact trajectory representation using parametric curves:
//! - NURBS/Bezier curve evaluation (48 bytes for a 6-DOF move)
//! - Velocity profiles (trapezoidal, S-curve, quintic)
//! - JIT interpolation at arbitrary control frequency
//! - no_std, zero-alloc core
//!
//! Author: Moroya Sakamoto

#![cfg_attr(not(feature = "std"), no_std)]

pub mod vec3;
pub mod bezier;
pub mod nurbs;
pub mod profile;
pub mod planner;

pub use vec3::Vec3;
pub use bezier::CubicBezier;
pub use nurbs::NurbsCurve;
pub use profile::{VelocityProfile, TrapezoidalProfile, SCurveProfile};
pub use planner::MotionPlan;
