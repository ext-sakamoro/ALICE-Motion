//! Velocity profiles — timing laws for trajectory execution
//!
//! Defines how fast to traverse a path: trapezoidal, S-curve, quintic.
//! Maps normalized path parameter s ∈ \[0,1\] to time t.
//!
//! Author: Moroya Sakamoto

/// Velocity profile trait
pub trait VelocityProfile {
    /// Position along path [0, 1] at time t
    fn position_at(&self, t: f32) -> f32;
    /// Velocity at time t
    fn velocity_at(&self, t: f32) -> f32;
    /// Acceleration at time t
    fn acceleration_at(&self, t: f32) -> f32;
    /// Total duration in seconds
    fn duration(&self) -> f32;
}

/// Trapezoidal velocity profile
///
/// Accelerate → cruise → decelerate
/// Params: v_max, a_max → 8 bytes
#[derive(Debug, Clone, Copy)]
pub struct TrapezoidalProfile {
    /// Maximum velocity (units/s)
    pub v_max: f32,
    /// Maximum acceleration (units/s²)
    pub a_max: f32,
    /// Total path length
    pub distance: f32,
    // Computed times
    t_accel: f32,
    t_cruise: f32,
    /// Deceleration phase duration (mirrors t_accel for symmetric profiles).
    /// Stored explicitly for future asymmetric decel support.
    #[allow(dead_code)]
    t_decel: f32,
    total_time: f32,
}

impl TrapezoidalProfile {
    pub fn new(v_max: f32, a_max: f32, distance: f32) -> Self {
        let t_accel = v_max / a_max;
        let d_accel = 0.5 * a_max * t_accel * t_accel;

        let (t_accel, t_cruise, t_decel, total_time) = if 2.0 * d_accel >= distance {
            // Triangle profile (can't reach v_max)
            let t_half = fast_sqrt_profile(distance / a_max);
            (t_half, 0.0, t_half, 2.0 * t_half)
        } else {
            // Full trapezoidal
            let d_cruise = distance - 2.0 * d_accel;
            let t_cruise = d_cruise / v_max;
            (t_accel, t_cruise, t_accel, 2.0 * t_accel + t_cruise)
        };

        Self {
            v_max,
            a_max,
            distance,
            t_accel,
            t_cruise,
            t_decel,
            total_time,
        }
    }
}

impl VelocityProfile for TrapezoidalProfile {
    fn position_at(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, self.total_time);

        if t <= self.t_accel {
            // Acceleration phase
            0.5 * self.a_max * t * t
        } else if t <= self.t_accel + self.t_cruise {
            // Cruise phase
            let d_accel = 0.5 * self.a_max * self.t_accel * self.t_accel;
            let v = self.a_max * self.t_accel;
            d_accel + v * (t - self.t_accel)
        } else {
            // Deceleration phase
            let dt = self.total_time - t;
            self.distance - 0.5 * self.a_max * dt * dt
        }
    }

    fn velocity_at(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, self.total_time);

        if t <= self.t_accel {
            self.a_max * t
        } else if t <= self.t_accel + self.t_cruise {
            self.a_max * self.t_accel // v_max or triangle peak
        } else {
            let dt = self.total_time - t;
            self.a_max * dt
        }
    }

    fn acceleration_at(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, self.total_time);

        if t <= self.t_accel {
            self.a_max
        } else if t <= self.t_accel + self.t_cruise {
            0.0
        } else {
            -self.a_max
        }
    }

    fn duration(&self) -> f32 {
        self.total_time
    }
}

/// S-curve velocity profile
///
/// Smooth jerk-limited motion: 7 phases.
/// Params: v_max, a_max, j_max → 12 bytes
#[derive(Debug, Clone, Copy)]
pub struct SCurveProfile {
    /// Maximum velocity
    pub v_max: f32,
    /// Maximum acceleration
    pub a_max: f32,
    /// Maximum jerk
    pub j_max: f32,
    /// Total path distance
    pub distance: f32,
    /// Time to reach a_max (jerk phase).
    /// Reserved for the full 7-phase S-curve implementation;
    /// currently the simplified smoothstep approximation is used instead.
    #[allow(dead_code)]
    t_jerk: f32,
    /// Total duration
    total_time: f32,
}

impl SCurveProfile {
    pub fn new(v_max: f32, a_max: f32, j_max: f32, distance: f32) -> Self {
        // Simplified: assume full s-curve profile
        let t_jerk = a_max / j_max;
        // Rough estimate of total time
        let t_accel = v_max / a_max + t_jerk;
        let d_accel = 0.5 * v_max * t_accel;

        let total_time = if 2.0 * d_accel >= distance {
            // Can't reach v_max
            2.0 * fast_sqrt_profile(distance / a_max) + 2.0 * t_jerk
        } else {
            let d_cruise = distance - 2.0 * d_accel;
            let t_cruise = d_cruise / v_max;
            2.0 * t_accel + t_cruise
        };

        Self {
            v_max,
            a_max,
            j_max,
            distance,
            t_jerk,
            total_time,
        }
    }
}

impl VelocityProfile for SCurveProfile {
    fn position_at(&self, t: f32) -> f32 {
        // Simplified: use trapezoidal with smoothed corners
        let inv_total = 1.0 / self.total_time;
        let normalized = t * inv_total;
        let smoothed = smoothstep(normalized);
        smoothed * self.distance
    }

    fn velocity_at(&self, t: f32) -> f32 {
        const INV_2H: f32 = 1.0 / (2.0 * 0.001);
        let h = 0.001;
        let p0 = self.position_at((t - h).max(0.0));
        let p1 = self.position_at((t + h).min(self.total_time));
        (p1 - p0) * INV_2H
    }

    fn acceleration_at(&self, t: f32) -> f32 {
        const INV_2H: f32 = 1.0 / (2.0 * 0.001);
        let h = 0.001;
        let v0 = self.velocity_at((t - h).max(0.0));
        let v1 = self.velocity_at((t + h).min(self.total_time));
        (v1 - v0) * INV_2H
    }

    fn duration(&self) -> f32 {
        self.total_time
    }
}

/// Smoothstep function for S-curve approximation
#[inline]
fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Fast sqrt for no_std profile computation
#[inline(always)]
fn fast_sqrt_profile(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let i = f32::to_bits(x);
    let i = 0x1fbd1df5 + (i >> 1);
    let y = f32::from_bits(i);
    let inv_y = 1.0 / y;
    0.5 * (y + x * inv_y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trapezoidal_endpoints() {
        let prof = TrapezoidalProfile::new(1.0, 2.0, 10.0);
        assert!((prof.position_at(0.0)).abs() < 0.01);
        assert!((prof.position_at(prof.duration()) - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_trapezoidal_velocity() {
        let prof = TrapezoidalProfile::new(2.0, 4.0, 20.0);
        // At start, velocity should be 0
        assert!(prof.velocity_at(0.0).abs() < 0.01);
        // At end, velocity should be 0
        assert!(prof.velocity_at(prof.duration()).abs() < 0.01);
        // Mid-cruise should be at max velocity
        let mid = prof.duration() / 2.0;
        assert!(prof.velocity_at(mid) > 0.5);
    }

    #[test]
    fn test_trapezoidal_triangle() {
        // Short distance → triangle profile (can't reach v_max)
        let prof = TrapezoidalProfile::new(10.0, 2.0, 1.0);
        assert!(prof.duration() > 0.0);
        assert!((prof.position_at(prof.duration()) - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_scurve_endpoints() {
        let prof = SCurveProfile::new(1.0, 2.0, 10.0, 10.0);
        assert!((prof.position_at(0.0)).abs() < 0.01);
        let end_pos = prof.position_at(prof.duration());
        assert!(
            (end_pos - 10.0).abs() < 0.5,
            "end pos should be ~10, got {end_pos}"
        );
    }

    #[test]
    fn test_scurve_smooth_velocity() {
        let prof = SCurveProfile::new(2.0, 4.0, 20.0, 20.0);
        // Velocity should be zero at endpoints
        assert!(prof.velocity_at(0.0).abs() < 0.5);
    }

    #[test]
    fn test_smoothstep() {
        assert!((smoothstep(0.0)).abs() < 0.01);
        assert!((smoothstep(1.0) - 1.0).abs() < 0.01);
        assert!((smoothstep(0.5) - 0.5).abs() < 0.01);
    }

    // --- Additional tests ---

    #[test]
    fn test_trapezoidal_duration_positive() {
        let prof = TrapezoidalProfile::new(1.0, 2.0, 5.0);
        assert!(prof.duration() > 0.0);
    }

    #[test]
    fn test_trapezoidal_acceleration_phases() {
        // Full trapezoidal: v_max=2, a_max=4, distance=20
        // t_accel = v_max/a_max = 0.5s
        let prof = TrapezoidalProfile::new(2.0, 4.0, 20.0);
        // During acceleration (early): accel should be positive
        let a_early = prof.acceleration_at(0.1);
        assert!(
            a_early > 0.0,
            "acceleration phase should be positive, got {a_early}"
        );
        // At end: accel should be negative (decelerating)
        let dur = prof.duration();
        let a_late = prof.acceleration_at(dur - 0.1);
        assert!(
            a_late < 0.0,
            "deceleration phase should be negative, got {a_late}"
        );
    }

    #[test]
    fn test_trapezoidal_cruise_acceleration_zero() {
        // In the cruise phase, acceleration should be 0
        let prof = TrapezoidalProfile::new(2.0, 4.0, 20.0);
        // Cruise is in the middle
        let mid_time = prof.duration() / 2.0;
        let a_cruise = prof.acceleration_at(mid_time);
        assert!(
            a_cruise.abs() < 1e-5,
            "cruise acceleration should be ~0, got {a_cruise}"
        );
    }

    #[test]
    fn test_trapezoidal_position_monotone() {
        // Position should be non-decreasing over time
        let prof = TrapezoidalProfile::new(1.0, 2.0, 10.0);
        let mut prev = prof.position_at(0.0);
        let n = 100;
        for i in 1..=n {
            let t = prof.duration() * i as f32 / n as f32;
            let curr = prof.position_at(t);
            assert!(
                curr >= prev - 1e-5,
                "position must be non-decreasing at t={t}: {curr} < {prev}"
            );
            prev = curr;
        }
    }

    #[test]
    fn test_trapezoidal_velocity_nonnegative() {
        // Velocity must be >= 0 throughout
        let prof = TrapezoidalProfile::new(2.0, 4.0, 20.0);
        let n = 50;
        for i in 0..=n {
            let t = prof.duration() * i as f32 / n as f32;
            let v = prof.velocity_at(t);
            assert!(v >= -1e-5, "velocity must be >= 0 at t={t}: {v}");
        }
    }

    #[test]
    fn test_trapezoidal_fields_accessible() {
        let prof = TrapezoidalProfile::new(3.0, 6.0, 30.0);
        assert_eq!(prof.v_max, 3.0);
        assert_eq!(prof.a_max, 6.0);
        assert_eq!(prof.distance, 30.0);
    }

    #[test]
    fn test_scurve_fields_accessible() {
        let prof = SCurveProfile::new(3.0, 6.0, 30.0, 25.0);
        assert_eq!(prof.v_max, 3.0);
        assert_eq!(prof.a_max, 6.0);
        assert_eq!(prof.j_max, 30.0);
        assert_eq!(prof.distance, 25.0);
    }

    #[test]
    fn test_scurve_duration_positive() {
        let prof = SCurveProfile::new(1.0, 2.0, 10.0, 15.0);
        assert!(prof.duration() > 0.0);
    }

    #[test]
    fn test_scurve_position_monotone() {
        // S-curve position should be non-decreasing
        let prof = SCurveProfile::new(2.0, 4.0, 20.0, 20.0);
        let mut prev = prof.position_at(0.0);
        let n = 100;
        for i in 1..=n {
            let t = prof.duration() * i as f32 / n as f32;
            let curr = prof.position_at(t);
            assert!(
                curr >= prev - 1e-4,
                "S-curve position must be non-decreasing at t={t}"
            );
            prev = curr;
        }
    }

    #[test]
    fn test_smoothstep_clamped_below() {
        // smoothstep(-1.0) should clamp to 0.0
        assert!((smoothstep(-1.0)).abs() < 1e-6);
    }

    #[test]
    fn test_smoothstep_clamped_above() {
        // smoothstep(2.0) should clamp to 1.0
        assert!((smoothstep(2.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_smoothstep_symmetry() {
        // smoothstep(x) + smoothstep(1-x) == 1  (point symmetry around 0.5)
        for i in 0..=10 {
            let x = i as f32 / 10.0;
            let sum = smoothstep(x) + smoothstep(1.0 - x);
            assert!(
                (sum - 1.0).abs() < 1e-5,
                "smoothstep symmetry violated at x={x}: sum={sum}"
            );
        }
    }

    #[test]
    fn test_trapezoidal_short_distance_triangle_profile() {
        // When 2*d_accel >= distance, it becomes triangle profile
        // v_max=100, a_max=1, distance=0.1 → definitely a triangle
        let prof = TrapezoidalProfile::new(100.0, 1.0, 0.1);
        // Triangle: t_cruise == 0, so velocity peaks below v_max
        // The peak velocity = a_max * t_half; position at duration ≈ distance
        assert!((prof.position_at(prof.duration()) - 0.1).abs() < 0.05);
    }
}
