//! Motion planner — combines trajectory + velocity profile
//!
//! Compiles a path curve with a timing law into a MotionPlan
//! that can be JIT-evaluated at any time t.
//!
//! Author: Moroya Sakamoto

use crate::bezier::CubicBezier;
use crate::nurbs::NurbsCurve;
use crate::profile::{VelocityProfile, TrapezoidalProfile, SCurveProfile};
use crate::vec3::Vec3;

/// Path representation
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
    pub fn bezier_trapezoidal(
        curve: CubicBezier,
        v_max: f32,
        a_max: f32,
    ) -> Self {
        let distance = curve.arc_length(64);
        Self {
            path: Path::Bezier(curve),
            profile: ProfileKind::Trapezoidal(TrapezoidalProfile::new(v_max, a_max, distance)),
        }
    }

    /// Create from NURBS + S-curve profile
    pub fn nurbs_scurve(
        curve: NurbsCurve,
        v_max: f32,
        a_max: f32,
        j_max: f32,
    ) -> Self {
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
        let h = 0.0005;
        let p0 = self.position((t - h).max(0.0));
        let p1 = self.position((t + h).min(self.duration()));
        let dt = if t < h { h } else if t > self.duration() - h { h } else { 2.0 * h };
        (p1 - p0) * (1.0 / dt)
    }

    /// Acceleration vector at time t
    pub fn acceleration(&self, t: f32) -> Vec3 {
        let h = 0.001;
        let v0 = self.velocity((t - h).max(0.0));
        let v1 = self.velocity((t + h).min(self.duration()));
        let dt = if t < h { h } else if t > self.duration() - h { h } else { 2.0 * h };
        (v1 - v0) * (1.0 / dt)
    }

    /// Total motion duration
    pub fn duration(&self) -> f32 {
        match &self.profile {
            ProfileKind::Trapezoidal(p) => p.duration(),
            ProfileKind::SCurve(p) => p.duration(),
        }
    }

    /// Path parameter s ∈ [0, 1] at time t
    fn path_parameter(&self, t: f32) -> f32 {
        let dist = match &self.profile {
            ProfileKind::Trapezoidal(p) => {
                let total = p.distance;
                if total < 1e-10 { return 0.0; }
                p.position_at(t) / total
            }
            ProfileKind::SCurve(p) => {
                let total = p.distance;
                if total < 1e-10 { return 0.0; }
                p.position_at(t) / total
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
        let curve = CubicBezier::from_endpoints(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
        );
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
        let curve = CubicBezier::from_endpoints(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
        );
        let plan = MotionPlan::bezier_trapezoidal(curve, 2.0, 4.0);

        // Mid-motion should have positive speed
        let mid = plan.duration() / 2.0;
        let speed = plan.speed(mid);
        assert!(speed > 0.1, "should be moving at midpoint, speed={speed}");
    }

    #[test]
    fn test_plan_duration_positive() {
        let curve = CubicBezier::from_endpoints(
            Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0),
        );
        let plan = MotionPlan::bezier_trapezoidal(curve, 0.5, 1.0);
        assert!(plan.duration() > 0.0);
    }
}
