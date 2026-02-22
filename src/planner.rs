//! Motion planner — combines trajectory + velocity profile
//!
//! Compiles a path curve with a timing law into a MotionPlan
//! that can be JIT-evaluated at any time t.
//!
//! Author: Moroya Sakamoto

use crate::bezier::CubicBezier;
use crate::nurbs::NurbsCurve;
use crate::profile::{SCurveProfile, TrapezoidalProfile, VelocityProfile};
use crate::vec3::Vec3;

/// Path representation
#[allow(clippy::large_enum_variant)]
pub enum Path {
    Bezier(CubicBezier),
    Nurbs(NurbsCurve),
}

/// Motion plan — path + timing
pub struct MotionPlan {
    path: Path,
    profile: ProfileKind,
}

enum ProfileKind {
    Trapezoidal(TrapezoidalProfile),
    SCurve(SCurveProfile),
}

impl MotionPlan {
    /// Create from Bezier + trapezoidal profile
    pub fn bezier_trapezoidal(curve: CubicBezier, v_max: f32, a_max: f32) -> Self {
        let distance = curve.arc_length(64);
        Self {
            path: Path::Bezier(curve),
            profile: ProfileKind::Trapezoidal(TrapezoidalProfile::new(v_max, a_max, distance)),
        }
    }

    /// Create from NURBS + S-curve profile
    pub fn nurbs_scurve(curve: NurbsCurve, v_max: f32, a_max: f32, j_max: f32) -> Self {
        let distance = curve.arc_length(64);
        Self {
            path: Path::Nurbs(curve),
            profile: ProfileKind::SCurve(SCurveProfile::new(v_max, a_max, j_max, distance)),
        }
    }

    /// Position at time t (seconds)
    pub fn position(&self, t: f32) -> Vec3 {
        let s = self.path_parameter(t);
        match &self.path {
            Path::Bezier(b) => b.position(s),
            Path::Nurbs(n) => n.position(s),
        }
    }

    /// Velocity vector at time t
    pub fn velocity(&self, t: f32) -> Vec3 {
        const H: f32 = 0.0005;
        const INV_2H: f32 = 1.0 / (2.0 * 0.0005);
        const INV_H: f32 = 1.0 / 0.0005;
        let p0 = self.position((t - H).max(0.0));
        let p1 = self.position((t + H).min(self.duration()));
        let inv_dt = if t < H || t > self.duration() - H {
            INV_H
        } else {
            INV_2H
        };
        (p1 - p0) * inv_dt
    }

    /// Acceleration vector at time t
    pub fn acceleration(&self, t: f32) -> Vec3 {
        const H: f32 = 0.001;
        const INV_2H: f32 = 1.0 / (2.0 * 0.001);
        const INV_H: f32 = 1.0 / 0.001;
        let v0 = self.velocity((t - H).max(0.0));
        let v1 = self.velocity((t + H).min(self.duration()));
        let inv_dt = if t < H || t > self.duration() - H {
            INV_H
        } else {
            INV_2H
        };
        (v1 - v0) * inv_dt
    }

    /// Total motion duration
    pub fn duration(&self) -> f32 {
        match &self.profile {
            ProfileKind::Trapezoidal(p) => p.duration(),
            ProfileKind::SCurve(p) => p.duration(),
        }
    }

    /// Path parameter s ∈ [0, 1] at time t
    #[inline(always)]
    fn path_parameter(&self, t: f32) -> f32 {
        let dist = match &self.profile {
            ProfileKind::Trapezoidal(p) => {
                let total = p.distance;
                if total < 1e-10 {
                    return 0.0;
                }
                let inv_total = 1.0 / total;
                p.position_at(t) * inv_total
            }
            ProfileKind::SCurve(p) => {
                let total = p.distance;
                if total < 1e-10 {
                    return 0.0;
                }
                let inv_total = 1.0 / total;
                p.position_at(t) * inv_total
            }
        };
        dist.clamp(0.0, 1.0)
    }

    /// Speed (scalar velocity magnitude) at time t
    pub fn speed(&self, t: f32) -> f32 {
        self.velocity(t).length()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bezier_plan() {
        let curve =
            CubicBezier::from_endpoints(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);

        let start = plan.position(0.0);
        let end = plan.position(plan.duration());

        assert!(start.distance(Vec3::ZERO) < 0.5);
        assert!((end.x - 10.0).abs() < 1.0, "end={end:?}");
    }

    #[test]
    fn test_nurbs_plan() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(3.0, 5.0, 0.0))
            .add_point(Vec3::new(7.0, 5.0, 0.0))
            .add_point(Vec3::new(10.0, 0.0, 0.0))
            .build();

        let plan = MotionPlan::nurbs_scurve(curve, 2.0, 4.0, 20.0);
        assert!(plan.duration() > 0.0);

        let start = plan.position(0.0);
        assert!(start.distance(Vec3::ZERO) < 1.0);
    }

    #[test]
    fn test_plan_velocity() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);

        // Mid-motion should have positive speed
        let mid = plan.duration() / 2.0;
        let speed = plan.speed(mid);
        assert!(speed > 0.1, "should be moving at midpoint, speed={speed}");
    }

    #[test]
    fn test_plan_duration_positive() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 0.5, 1.0);
        assert!(plan.duration() > 0.0);
    }

    // --- Additional tests ---

    #[test]
    fn test_plan_position_finite_everywhere() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(5.0, 3.0, 1.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 1.0, 2.0);
        let n = 20;
        for i in 0..=n {
            let t = plan.duration() * i as f32 / n as f32;
            let p = plan.position(t);
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "position must be finite at t={t}"
            );
        }
    }

    #[test]
    fn test_plan_speed_nonnegative() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(8.0, 0.0, 0.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);
        let n = 20;
        for i in 0..=n {
            let t = plan.duration() * i as f32 / n as f32;
            let speed = plan.speed(t);
            assert!(speed >= 0.0, "speed must be >= 0 at t={t}, got {speed}");
        }
    }

    #[test]
    fn test_plan_acceleration_finite() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 1.0, 2.0);
        let mid = plan.duration() / 2.0;
        let a = plan.acceleration(mid);
        assert!(
            a.x.is_finite() && a.y.is_finite() && a.z.is_finite(),
            "acceleration must be finite"
        );
    }

    #[test]
    fn test_nurbs_plan_velocity_finite() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(2.0, 3.0, 0.0))
            .add_point(Vec3::new(5.0, 3.0, 0.0))
            .add_point(Vec3::new(7.0, 0.0, 0.0))
            .build();
        let plan = MotionPlan::nurbs_scurve(curve, 1.5, 3.0, 15.0);
        let n = 10;
        for i in 0..=n {
            let t = plan.duration() * i as f32 / n as f32;
            let v = plan.velocity(t);
            assert!(v.x.is_finite(), "velocity.x must be finite at t={t}");
        }
    }

    #[test]
    fn test_plan_speed_at_start_near_zero() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);
        // Speed at t=0 should be 0 (starts from rest in trapezoidal profile)
        let s = plan.speed(0.0);
        assert!(s < 0.5, "speed at start should be near zero, got {s}");
    }

    #[test]
    fn test_plan_speed_at_end_near_zero() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);
        let dur = plan.duration();
        let s = plan.speed(dur);
        assert!(s < 0.5, "speed at end should be near zero, got {s}");
    }

    #[test]
    fn test_plan_nurbs_scurve_duration_depends_on_speed() {
        let curve_a = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 0.0, 0.0))
            .add_point(Vec3::new(2.0, 0.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        let curve_b = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 0.0, 0.0))
            .add_point(Vec3::new(2.0, 0.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        // Faster v_max → shorter duration
        let plan_slow = MotionPlan::nurbs_scurve(curve_a, 0.5, 1.0, 5.0);
        let plan_fast = MotionPlan::nurbs_scurve(curve_b, 5.0, 10.0, 50.0);
        assert!(
            plan_fast.duration() < plan_slow.duration(),
            "faster plan should be shorter: {} vs {}",
            plan_fast.duration(),
            plan_slow.duration()
        );
    }
}
