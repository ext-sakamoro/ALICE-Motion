//! 3D vector type — minimal, `no_std`, Copy
//!
//! Author: Moroya Sakamoto

/// 3D vector — 12 bytes
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline]
    #[must_use] 
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline]
    #[must_use] 
    pub fn from_array(a: [f32; 3]) -> Self {
        Self {
            x: a[0],
            y: a[1],
            z: a[2],
        }
    }

    #[inline]
    #[must_use] 
    pub fn to_array(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    #[inline(always)]
    #[must_use] 
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    #[must_use] 
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline(always)]
    #[must_use] 
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    #[inline(always)]
    #[must_use] 
    pub fn length(self) -> f32 {
        fast_sqrt(self.length_squared())
    }

    #[inline(always)]
    #[must_use] 
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len < 1e-10 {
            Self::ZERO
        } else {
            self * (1.0 / len)
        }
    }

    /// Linear interpolation
    #[inline]
    #[must_use] 
    pub fn lerp(self, other: Self, t: f32) -> Self {
        self * (1.0 - t) + other * t
    }

    /// Distance to another point
    #[inline]
    #[must_use] 
    pub fn distance(self, other: Self) -> f32 {
        (self - other).length()
    }

    /// Distance squared
    #[inline]
    #[must_use] 
    pub fn distance_squared(self, other: Self) -> f32 {
        (self - other).length_squared()
    }
}

/// Fast sqrt approximation (Quake-style + Newton-Raphson)
#[inline(always)]
fn fast_sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let i = f32::to_bits(x);
    let i = 0x1fbd_1df5 + (i >> 1); // Initial approximation
    let y = f32::from_bits(i);
    // One Newton-Raphson step: 0.5 * (y + x/y) = 0.5 * y + 0.5 * x * inv_y
    let inv_y = 1.0 / y;
    0.5 * (y + x * inv_y)
}

impl core::ops::Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl core::ops::Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl core::ops::Mul<f32> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl core::ops::Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dot_product() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert!((a.dot(b)).abs() < 1e-6);
        assert!((a.dot(a) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_cross_product() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(y);
        assert!((z.z - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_length() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert!((v.length() - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_lerp() {
        let a = Vec3::ZERO;
        let b = Vec3::new(10.0, 0.0, 0.0);
        let mid = a.lerp(b, 0.5);
        assert!((mid.x - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_normalize() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        let n = v.normalize();
        assert!((n.length() - 1.0).abs() < 0.05);
    }

    // --- Additional tests ---

    #[test]
    fn test_zero_constant() {
        let z = Vec3::ZERO;
        assert_eq!(z.x, 0.0);
        assert_eq!(z.y, 0.0);
        assert_eq!(z.z, 0.0);
    }

    #[test]
    fn test_from_array_roundtrip() {
        let arr = [1.5f32, 2.5, 3.5];
        let v = Vec3::from_array(arr);
        let out = v.to_array();
        assert_eq!(out, arr);
    }

    #[test]
    fn test_add() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        let c = a + b;
        assert_eq!(c.x, 5.0);
        assert_eq!(c.y, 7.0);
        assert_eq!(c.z, 9.0);
    }

    #[test]
    fn test_sub() {
        let a = Vec3::new(5.0, 7.0, 9.0);
        let b = Vec3::new(1.0, 2.0, 3.0);
        let c = a - b;
        assert_eq!(c.x, 4.0);
        assert_eq!(c.y, 5.0);
        assert_eq!(c.z, 6.0);
    }

    #[test]
    fn test_mul_scalar() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let scaled = v * 3.0;
        assert_eq!(scaled.x, 3.0);
        assert_eq!(scaled.y, 6.0);
        assert_eq!(scaled.z, 9.0);
    }

    #[test]
    fn test_neg() {
        let v = Vec3::new(1.0, -2.0, 3.0);
        let n = -v;
        assert_eq!(n.x, -1.0);
        assert_eq!(n.y, 2.0);
        assert_eq!(n.z, -3.0);
    }

    #[test]
    fn test_length_squared() {
        let v = Vec3::new(1.0, 2.0, 2.0);
        // 1 + 4 + 4 = 9
        assert!((v.length_squared() - 9.0).abs() < 1e-6);
    }

    #[test]
    fn test_distance() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(3.0, 4.0, 0.0);
        assert!((a.distance(b) - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_distance_squared() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 1.0, 1.0);
        assert!((a.distance_squared(b) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_zero_vector() {
        // Normalizing the zero vector should return ZERO, not NaN
        let v = Vec3::ZERO;
        let n = v.normalize();
        assert_eq!(n.x, 0.0);
        assert_eq!(n.y, 0.0);
        assert_eq!(n.z, 0.0);
    }

    #[test]
    fn test_cross_anticommutative() {
        // a x b == -(b x a)
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        let ab = a.cross(b);
        let ba = b.cross(a);
        assert!((ab.x + ba.x).abs() < 1e-5);
        assert!((ab.y + ba.y).abs() < 1e-5);
        assert!((ab.z + ba.z).abs() < 1e-5);
    }

    #[test]
    fn test_lerp_at_zero() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(10.0, 20.0, 30.0);
        let result = a.lerp(b, 0.0);
        assert!((result.x - a.x).abs() < 1e-6);
        assert!((result.y - a.y).abs() < 1e-6);
        assert!((result.z - a.z).abs() < 1e-6);
    }

    #[test]
    fn test_lerp_at_one() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(10.0, 20.0, 30.0);
        let result = a.lerp(b, 1.0);
        assert!((result.x - b.x).abs() < 1e-5);
        assert!((result.y - b.y).abs() < 1e-5);
        assert!((result.z - b.z).abs() < 1e-5);
    }

    #[test]
    fn test_dot_self_equals_length_squared() {
        let v = Vec3::new(2.0, 3.0, 6.0);
        let dot_self = v.dot(v);
        let len_sq = v.length_squared();
        assert!((dot_self - len_sq).abs() < 1e-6);
    }

    // FNV-1a hash verification: hash a Vec3's bit-representation
    fn fnv1a_vec3(v: Vec3) -> u64 {
        let bytes = [
            v.x.to_bits().to_le_bytes(),
            v.y.to_bits().to_le_bytes(),
            v.z.to_bits().to_le_bytes(),
        ];
        let mut h: u64 = 0xcbf29ce484222325;
        for chunk in &bytes {
            for &b in chunk {
                h ^= b as u64;
                h = h.wrapping_mul(0x100000001b3);
            }
        }
        h
    }

    #[test]
    fn test_fnv1a_hash_nonzero() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let h = fnv1a_vec3(v);
        assert_ne!(h, 0, "FNV-1a hash must not be zero for non-trivial input");
    }

    #[test]
    fn test_fnv1a_hash_deterministic() {
        let v = Vec3::new(3.14, 2.71, 1.41);
        let h1 = fnv1a_vec3(v);
        let h2 = fnv1a_vec3(v);
        assert_eq!(h1, h2, "FNV-1a hash must be deterministic");
    }

    #[test]
    fn test_fnv1a_hash_differs_per_vector() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert_ne!(fnv1a_vec3(a), fnv1a_vec3(b));
    }
}
