//! Cubic Bezier curve — 4 control points, 48 bytes per segment
//!
//! Evaluates position, velocity (tangent), and acceleration at parameter t.
//! Chaining segments produces smooth paths for robot arms and drones.
//!
//! Author: Moroya Sakamoto

use crate::vec3::Vec3;

/// Cubic Bezier curve segment — 4 control points × 12 bytes = 48 bytes
#[derive(Debug, Clone, Copy)]
pub struct CubicBezier {
    pub p0: Vec3,
    pub p1: Vec3,
    pub p2: Vec3,
    pub p3: Vec3,
}

impl CubicBezier {
    pub fn new(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3) -> Self {
        Self { p0, p1, p2, p3 }
    }

    /// Create from start/end points with automatic control points
    pub fn from_endpoints(start: Vec3, end: Vec3) -> Self {
        let dir = end - start;
        Self {
            p0: start,
            p1: start + dir * 0.333,
            p2: start + dir * 0.667,
            p3: end,
        }
    }

    /// Evaluate position at parameter t ∈ [0, 1]
    ///
    /// B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
    #[inline]
    pub fn position(&self, t: f32) -> Vec3 {
        let u = 1.0 - t;
        let u2 = u * u;
        let u3 = u2 * u;
        let t2 = t * t;
        let t3 = t2 * t;

        self.p0 * u3 + self.p1 * (3.0 * u2 * t) + self.p2 * (3.0 * u * t2) + self.p3 * t3
    }

    /// First derivative (velocity/tangent) at parameter t
    ///
    /// B'(t) = 3(1-t)²(P₁-P₀) + 6(1-t)t(P₂-P₁) + 3t²(P₃-P₂)
    #[inline]
    pub fn velocity(&self, t: f32) -> Vec3 {
        let u = 1.0 - t;
        let d01 = self.p1 - self.p0;
        let d12 = self.p2 - self.p1;
        let d23 = self.p3 - self.p2;

        d01 * (3.0 * u * u) + d12 * (6.0 * u * t) + d23 * (3.0 * t * t)
    }

    /// Second derivative (acceleration) at parameter t
    ///
    /// B''(t) = 6(1-t)(P₂-2P₁+P₀) + 6t(P₃-2P₂+P₁)
    #[inline]
    pub fn acceleration(&self, t: f32) -> Vec3 {
        let u = 1.0 - t;
        let a = self.p2 - self.p1 * 2.0 + self.p0;
        let b = self.p3 - self.p2 * 2.0 + self.p1;
        a * (6.0 * u) + b * (6.0 * t)
    }

    /// Approximate arc length using subdivision
    pub fn arc_length(&self, subdivisions: usize) -> f32 {
        let mut length = 0.0f32;
        let mut prev = self.position(0.0);
        for i in 1..=subdivisions {
            let t = i as f32 / subdivisions as f32;
            let curr = self.position(t);
            length += prev.distance(curr);
            prev = curr;
        }
        length
    }

    /// Split curve at parameter t (de Casteljau algorithm)
    pub fn split(&self, t: f32) -> (CubicBezier, CubicBezier) {
        let p01 = self.p0.lerp(self.p1, t);
        let p12 = self.p1.lerp(self.p2, t);
        let p23 = self.p2.lerp(self.p3, t);
        let p012 = p01.lerp(p12, t);
        let p123 = p12.lerp(p23, t);
        let p0123 = p012.lerp(p123, t);

        (
            CubicBezier::new(self.p0, p01, p012, p0123),
            CubicBezier::new(p0123, p123, p23, self.p3),
        )
    }

    /// Serialized size
    pub const fn size_bytes() -> usize {
        48 // 4 points × 3 floats × 4 bytes
    }
}

/// Bezier spline — chain of cubic segments with C1 continuity
pub struct BezierSpline {
    segments: [CubicBezier; 8], // Max 8 segments
    count: usize,
}

impl BezierSpline {
    pub fn new() -> Self {
        Self {
            segments: [CubicBezier::new(Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO); 8],
            count: 0,
        }
    }

    /// Add a segment (must connect to previous endpoint)
    pub fn add_segment(&mut self, segment: CubicBezier) -> bool {
        if self.count >= 8 {
            return false;
        }
        self.segments[self.count] = segment;
        self.count += 1;
        true
    }

    /// Evaluate position at global parameter t ∈ [0, 1]
    pub fn position(&self, t: f32) -> Vec3 {
        if self.count == 0 {
            return Vec3::ZERO;
        }
        let scaled = t * self.count as f32;
        let seg_idx = (scaled as usize).min(self.count - 1);
        let local_t = scaled - seg_idx as f32;
        self.segments[seg_idx].position(local_t.clamp(0.0, 1.0))
    }

    /// Evaluate velocity at global parameter t
    pub fn velocity(&self, t: f32) -> Vec3 {
        if self.count == 0 {
            return Vec3::ZERO;
        }
        let scaled = t * self.count as f32;
        let seg_idx = (scaled as usize).min(self.count - 1);
        let local_t = scaled - seg_idx as f32;
        self.segments[seg_idx].velocity(local_t.clamp(0.0, 1.0))
    }

    /// Total arc length
    pub fn arc_length(&self) -> f32 {
        let mut total = 0.0f32;
        for i in 0..self.count {
            total += self.segments[i].arc_length(32);
        }
        total
    }

    /// Number of segments
    pub fn segment_count(&self) -> usize {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bezier_endpoints() {
        let b = CubicBezier::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(2.0, 1.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
        );
        let p0 = b.position(0.0);
        let p1 = b.position(1.0);
        assert!((p0.x).abs() < 0.01);
        assert!((p1.x - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_bezier_midpoint() {
        let b = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let mid = b.position(0.5);
        assert!((mid.x - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_bezier_velocity() {
        let b = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let vel = b.velocity(0.5);
        assert!(vel.x > 0.0, "should move in +x");
    }

    #[test]
    fn test_bezier_split() {
        let b = CubicBezier::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 2.0, 0.0),
            Vec3::new(3.0, 2.0, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
        );
        let (left, right) = b.split(0.5);
        // Split point should match original at t=0.5
        let orig_mid = b.position(0.5);
        let left_end = left.position(1.0);
        let right_start = right.position(0.0);
        assert!(orig_mid.distance(left_end) < 0.01);
        assert!(orig_mid.distance(right_start) < 0.01);
    }

    #[test]
    fn test_bezier_arc_length() {
        // Straight line should have arc length ≈ distance
        let b = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let len = b.arc_length(100);
        assert!((len - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_spline() {
        let mut spline = BezierSpline::new();
        spline.add_segment(CubicBezier::from_endpoints(
            Vec3::ZERO,
            Vec3::new(5.0, 0.0, 0.0),
        ));
        spline.add_segment(CubicBezier::from_endpoints(
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(10.0, 5.0, 0.0),
        ));
        assert_eq!(spline.segment_count(), 2);
        let start = spline.position(0.0);
        let end = spline.position(1.0);
        assert!(start.distance(Vec3::ZERO) < 0.1);
        assert!((end.x - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_size() {
        assert_eq!(CubicBezier::size_bytes(), 48);
    }
}
