//! NURBS curve evaluation — Non-Uniform Rational B-Spline
//!
//! Industry standard for smooth curves with exact conics.
//! Compact: 6-DOF trajectory in ~48 bytes.
//!
//! Author: Moroya Sakamoto

use crate::vec3::Vec3;

/// Maximum control points
const MAX_CONTROL_POINTS: usize = 16;
/// Maximum knots (n + p + 1 ≤ 24)
const MAX_KNOTS: usize = 24;

/// NURBS curve
///
/// C(u) = Σ Nᵢ,ₚ(u) wᵢ Pᵢ / Σ Nᵢ,ₚ(u) wᵢ
///
/// Stored in compact fixed-size arrays for no_std.
#[derive(Debug, Clone)]
pub struct NurbsCurve {
    /// Curve degree (typically 3)
    pub degree: u8,
    /// Number of control points
    n_points: usize,
    /// Control points
    points: [Vec3; MAX_CONTROL_POINTS],
    /// Weights (1.0 for non-rational B-spline)
    weights: [f32; MAX_CONTROL_POINTS],
    /// Knot vector
    knots: [f32; MAX_KNOTS],
    /// Number of knots
    n_knots: usize,
}

impl NurbsCurve {
    /// Create a NURBS curve builder
    pub fn builder(degree: u8) -> NurbsBuilder {
        NurbsBuilder {
            degree,
            points: [Vec3::ZERO; MAX_CONTROL_POINTS],
            weights: [1.0; MAX_CONTROL_POINTS],
            n_points: 0,
        }
    }

    /// Evaluate position at parameter u ∈ [0, 1]
    pub fn position(&self, u: f32) -> Vec3 {
        let p = self.degree as usize;

        // Handle endpoints exactly
        if u <= 0.0 {
            return self.points[0];
        }
        if u >= 1.0 {
            return self.points[self.n_points - 1];
        }

        let u_mapped = self.knots[p]
            + u * (self.knots[self.n_knots - p - 1] - self.knots[p]);

        let mut numerator = Vec3::ZERO;
        let mut denominator = 0.0f32;

        for i in 0..self.n_points {
            let basis = self.basis_function(i, p, u_mapped);
            let w = self.weights[i] * basis;
            numerator = numerator + self.points[i] * w;
            denominator += w;
        }

        if denominator.abs() < 1e-10 {
            Vec3::ZERO
        } else {
            let inv_denom = 1.0 / denominator;
            numerator * inv_denom
        }
    }

    /// Evaluate first derivative (velocity) using finite differences
    pub fn velocity(&self, u: f32) -> Vec3 {
        const INV_2H: f32 = 1.0 / (2.0 * 0.001);
        let h = 0.001;
        let p0 = self.position((u - h).max(0.0));
        let p1 = self.position((u + h).min(1.0));
        (p1 - p0) * INV_2H
    }

    /// Number of control points
    pub fn point_count(&self) -> usize {
        self.n_points
    }

    /// Approximate arc length
    pub fn arc_length(&self, subdivisions: usize) -> f32 {
        let mut length = 0.0f32;
        let mut prev = self.position(0.0);
        let inv_sub = 1.0 / subdivisions as f32;
        for i in 1..=subdivisions {
            let t = i as f32 * inv_sub;
            let curr = self.position(t);
            length += prev.distance(curr);
            prev = curr;
        }
        length
    }

    /// Cox-de Boor basis function Nᵢ,ₚ(u)
    fn basis_function(&self, i: usize, p: usize, u: f32) -> f32 {
        if p == 0 {
            if u >= self.knots[i] && u < self.knots[i + 1] {
                return 1.0;
            }
            // Handle last knot span
            if i + 1 == self.n_knots - 1 && u == self.knots[i + 1] {
                return 1.0;
            }
            return 0.0;
        }

        let mut result = 0.0f32;

        let denom1 = self.knots[i + p] - self.knots[i];
        if denom1.abs() > 1e-10 {
            let inv_d1 = 1.0 / denom1;
            result += (u - self.knots[i]) * inv_d1 * self.basis_function(i, p - 1, u);
        }

        let denom2 = self.knots[i + p + 1] - self.knots[i + 1];
        if denom2.abs() > 1e-10 {
            let inv_d2 = 1.0 / denom2;
            result += (self.knots[i + p + 1] - u) * inv_d2 * self.basis_function(i + 1, p - 1, u);
        }

        result
    }
}

/// Builder for NURBS curves
pub struct NurbsBuilder {
    degree: u8,
    points: [Vec3; MAX_CONTROL_POINTS],
    weights: [f32; MAX_CONTROL_POINTS],
    n_points: usize,
}

impl NurbsBuilder {
    /// Add a control point with weight 1.0
    pub fn add_point(mut self, point: Vec3) -> Self {
        if self.n_points < MAX_CONTROL_POINTS {
            self.points[self.n_points] = point;
            self.weights[self.n_points] = 1.0;
            self.n_points += 1;
        }
        self
    }

    /// Add a weighted control point
    pub fn add_weighted_point(mut self, point: Vec3, weight: f32) -> Self {
        if self.n_points < MAX_CONTROL_POINTS {
            self.points[self.n_points] = point;
            self.weights[self.n_points] = weight;
            self.n_points += 1;
        }
        self
    }

    /// Build the NURBS curve with uniform knot vector
    pub fn build(self) -> NurbsCurve {
        let p = self.degree as usize;
        let n = self.n_points;
        let n_knots = n + p + 1;

        // Generate clamped uniform knot vector
        let mut knots = [0.0f32; MAX_KNOTS];
        for i in 0..n_knots {
            if i <= p {
                knots[i] = 0.0;
            } else if i >= n {
                knots[i] = 1.0;
            } else {
                knots[i] = (i - p) as f32 / (n - p) as f32;
            }
        }

        NurbsCurve {
            degree: self.degree,
            n_points: n,
            points: self.points,
            weights: self.weights,
            knots,
            n_knots,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nurbs_endpoints() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 1.0, 0.0))
            .add_point(Vec3::new(2.0, 1.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();

        let start = curve.position(0.0);
        let end = curve.position(1.0);
        assert!(start.distance(Vec3::ZERO) < 0.1, "start={start:?}");
        assert!((end.x - 3.0).abs() < 0.1, "end={end:?}");
    }

    #[test]
    fn test_nurbs_midpoint() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 2.0, 0.0))
            .add_point(Vec3::new(3.0, 2.0, 0.0))
            .add_point(Vec3::new(4.0, 0.0, 0.0))
            .build();

        let mid = curve.position(0.5);
        // Should be elevated (y > 0) at midpoint
        assert!(mid.y > 0.5, "midpoint should be elevated, got {mid:?}");
    }

    #[test]
    fn test_nurbs_velocity() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 0.0, 0.0))
            .add_point(Vec3::new(2.0, 0.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();

        let vel = curve.velocity(0.5);
        assert!(vel.x > 0.0, "should move in +x, got {vel:?}");
    }

    #[test]
    fn test_nurbs_arc_length() {
        // Degree 2 with 3 points for a simple curve
        let curve = NurbsCurve::builder(2)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(5.0, 0.0, 0.0))
            .add_point(Vec3::new(10.0, 0.0, 0.0))
            .build();

        let len = curve.arc_length(100);
        assert!((len - 10.0).abs() < 1.0, "straight line should be ~10, got {len}");
    }

    #[test]
    fn test_nurbs_weighted() {
        // Circle arc using weights
        let curve = NurbsCurve::builder(2)
            .add_weighted_point(Vec3::new(1.0, 0.0, 0.0), 1.0)
            .add_weighted_point(Vec3::new(1.0, 1.0, 0.0), 0.707)
            .add_weighted_point(Vec3::new(0.0, 1.0, 0.0), 1.0)
            .build();

        let mid = curve.position(0.5);
        // For a quarter circle, midpoint should be at ~45° → (cos45, sin45) ≈ (0.707, 0.707)
        let r = fast_sqrt_test(mid.x * mid.x + mid.y * mid.y);
        assert!((r - 1.0).abs() < 0.2, "weighted curve should approximate circle, r={r}");
    }

    fn fast_sqrt_test(x: f32) -> f32 {
        if x <= 0.0 { return 0.0; }
        let i = f32::to_bits(x);
        let i = 0x1fbd1df5 + (i >> 1);
        let y = f32::from_bits(i);
        0.5 * (y + x / y)
    }

    // --- Additional tests ---

    #[test]
    fn test_nurbs_degree_field() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::ZERO)
            .add_point(Vec3::new(1.0, 0.0, 0.0))
            .add_point(Vec3::new(2.0, 0.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        assert_eq!(curve.degree, 3);
    }

    #[test]
    fn test_nurbs_point_count() {
        let curve = NurbsCurve::builder(2)
            .add_point(Vec3::ZERO)
            .add_point(Vec3::new(1.0, 0.0, 0.0))
            .add_point(Vec3::new(2.0, 0.0, 0.0))
            .build();
        assert_eq!(curve.point_count(), 3);
    }

    #[test]
    fn test_nurbs_position_within_bounds() {
        // All sampled positions must have x in [0, 3] for these control points
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 1.0, 0.0))
            .add_point(Vec3::new(2.0, 1.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        for i in 0..=20 {
            let u = i as f32 / 20.0;
            let p = curve.position(u);
            assert!(p.x >= -0.01 && p.x <= 3.01, "x out of range at u={u}: {}", p.x);
        }
    }

    #[test]
    fn test_nurbs_degree2_three_points() {
        // Minimum case: degree-2 with exactly 3 points
        let curve = NurbsCurve::builder(2)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 1.0, 0.0))
            .add_point(Vec3::new(2.0, 0.0, 0.0))
            .build();
        assert_eq!(curve.point_count(), 3);
        let start = curve.position(0.0);
        let end = curve.position(1.0);
        assert!(start.distance(Vec3::ZERO) < 0.1);
        assert!((end.x - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_nurbs_velocity_finite() {
        // Velocity must be finite everywhere (no NaN/Inf)
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 2.0, 0.0))
            .add_point(Vec3::new(2.0, 2.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        for i in 0..=10 {
            let u = i as f32 / 10.0;
            let v = curve.velocity(u);
            assert!(v.x.is_finite(), "velocity.x must be finite at u={u}");
            assert!(v.y.is_finite(), "velocity.y must be finite at u={u}");
            assert!(v.z.is_finite(), "velocity.z must be finite at u={u}");
        }
    }

    #[test]
    fn test_nurbs_arc_length_increases_with_points() {
        // More spread-out points → longer arc length
        let short_curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(0.5, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 0.0, 0.0))
            .add_point(Vec3::new(1.5, 0.0, 0.0))
            .build();
        let long_curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(5.0, 0.0, 0.0))
            .add_point(Vec3::new(10.0, 0.0, 0.0))
            .add_point(Vec3::new(15.0, 0.0, 0.0))
            .build();
        let short_len = short_curve.arc_length(50);
        let long_len = long_curve.arc_length(50);
        assert!(long_len > short_len, "longer span must yield longer arc, {} vs {}", long_len, short_len);
    }

    #[test]
    fn test_nurbs_builder_overflow_silently_truncates() {
        // Adding more than MAX_CONTROL_POINTS (16) should not panic
        let mut builder = NurbsCurve::builder(3);
        // Add 20 points — only 16 should be accepted
        for i in 0..20 {
            builder = builder.add_point(Vec3::new(i as f32, 0.0, 0.0));
        }
        let curve = builder.build();
        assert!(curve.point_count() <= MAX_CONTROL_POINTS);
    }

    #[test]
    fn test_nurbs_weighted_point_count() {
        let curve = NurbsCurve::builder(2)
            .add_weighted_point(Vec3::new(0.0, 0.0, 0.0), 1.0)
            .add_weighted_point(Vec3::new(1.0, 1.0, 0.0), 2.0)
            .add_weighted_point(Vec3::new(2.0, 0.0, 0.0), 1.0)
            .build();
        assert_eq!(curve.point_count(), 3);
    }

    // FNV-1a content hash over a sampled NURBS trajectory
    fn fnv1a(data: &[u8]) -> u64 {
        let mut h: u64 = 0xcbf29ce484222325;
        for &b in data {
            h ^= b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
        h
    }

    fn hash_nurbs_samples(curve: &NurbsCurve, n: usize) -> u64 {
        let mut bytes = [0u8; 12]; // 3 x f32
        let mut h: u64 = 0xcbf29ce484222325;
        for i in 0..=n {
            let u = i as f32 / n as f32;
            let p = curve.position(u);
            let px = p.x.to_bits().to_le_bytes();
            let py = p.y.to_bits().to_le_bytes();
            let pz = p.z.to_bits().to_le_bytes();
            bytes[0..4].copy_from_slice(&px);
            bytes[4..8].copy_from_slice(&py);
            bytes[8..12].copy_from_slice(&pz);
            let next = fnv1a(&bytes);
            h ^= next;
            h = h.wrapping_mul(0x100000001b3);
        }
        h
    }

    #[test]
    fn test_nurbs_sample_hash_nonzero() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 2.0, 0.0))
            .add_point(Vec3::new(2.0, 2.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        let h = hash_nurbs_samples(&curve, 16);
        assert_ne!(h, 0, "content hash must not be zero");
    }

    #[test]
    fn test_nurbs_sample_hash_deterministic() {
        let curve = NurbsCurve::builder(3)
            .add_point(Vec3::new(0.0, 0.0, 0.0))
            .add_point(Vec3::new(1.0, 2.0, 0.0))
            .add_point(Vec3::new(2.0, 2.0, 0.0))
            .add_point(Vec3::new(3.0, 0.0, 0.0))
            .build();
        let h1 = hash_nurbs_samples(&curve, 16);
        let h2 = hash_nurbs_samples(&curve, 16);
        assert_eq!(h1, h2, "content hash must be deterministic");
    }
}
