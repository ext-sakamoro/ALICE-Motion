//! 軌道と障害物の衝突検出。
//!
//! 軌道を離散サンプリングして障害物 (球, AABB) との交差判定を行う。
//! パス計画段階での衝突チェックと回避に使用。

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

use crate::bezier::CubicBezier;
use crate::vec3::Vec3;

/// 球障害物。
#[derive(Debug, Clone, Copy)]
pub struct SphereObstacle {
    /// 中心位置。
    pub center: Vec3,
    /// 半径。
    pub radius: f32,
}

impl SphereObstacle {
    /// 新しい球障害物を作成。
    #[must_use]
    pub const fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }

    /// 点が球の内部にあるか。
    #[must_use]
    pub fn contains(&self, point: Vec3) -> bool {
        (point - self.center).length_squared() <= self.radius * self.radius
    }

    /// 点と球表面との距離（負=内部）。
    #[must_use]
    pub fn signed_distance(&self, point: Vec3) -> f32 {
        (point - self.center).length() - self.radius
    }
}

/// AABB (軸並行バウンディングボックス) 障害物。
#[derive(Debug, Clone, Copy)]
pub struct AabbObstacle {
    /// 最小角。
    pub min: Vec3,
    /// 最大角。
    pub max: Vec3,
}

impl AabbObstacle {
    /// 新しい AABB 障害物を作成。
    #[must_use]
    pub const fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// 点が AABB 内部にあるか。
    #[must_use]
    pub fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// 点と AABB の符号付き距離（近似, 負=内部）。
    #[must_use]
    pub fn signed_distance(&self, point: Vec3) -> f32 {
        // 最近接点までの距離
        let cx = (point.x - self.min.x).max(0.0).min(self.max.x - self.min.x);
        let cy = (point.y - self.min.y).max(0.0).min(self.max.y - self.min.y);
        let cz = (point.z - self.min.z).max(0.0).min(self.max.z - self.min.z);
        let closest = Vec3::new(self.min.x + cx, self.min.y + cy, self.min.z + cz);
        let dist = (point - closest).length();
        if self.contains(point) {
            -dist.max(1e-8)
        } else {
            dist
        }
    }
}

/// 障害物の列挙型。
#[derive(Debug, Clone, Copy)]
pub enum Obstacle {
    /// 球。
    Sphere(SphereObstacle),
    /// AABB。
    Aabb(AabbObstacle),
}

impl Obstacle {
    /// 点が障害物内にあるか。
    #[must_use]
    pub fn contains(&self, point: Vec3) -> bool {
        match self {
            Self::Sphere(s) => s.contains(point),
            Self::Aabb(a) => a.contains(point),
        }
    }
}

/// 衝突チェック結果。
#[derive(Debug, Clone, Copy)]
pub struct CollisionHit {
    /// 衝突したパラメータ t ∈ [0, 1]。
    pub parameter: f32,
    /// 衝突位置。
    pub position: Vec3,
    /// 衝突した障害物のインデックス。
    pub obstacle_index: usize,
}

/// Bezier 軌道と障害物群の衝突検出。
///
/// 軌道を `samples` 個のポイントでサンプリングし、
/// 各ポイントで全障害物との交差をチェック。
///
/// 最初に見つかった衝突を返す。
#[must_use]
pub fn check_collision(
    curve: &CubicBezier,
    obstacles: &[Obstacle],
    samples: u32,
) -> Option<CollisionHit> {
    if obstacles.is_empty() || samples == 0 {
        return None;
    }
    let rcp = 1.0 / samples as f32;
    for i in 0..=samples {
        let t = i as f32 * rcp;
        let pos = curve.position(t);
        for (idx, obs) in obstacles.iter().enumerate() {
            if obs.contains(pos) {
                return Some(CollisionHit {
                    parameter: t,
                    position: pos,
                    obstacle_index: idx,
                });
            }
        }
    }
    None
}

/// 全衝突ポイントを返す（最初だけでなく全て）。
#[must_use]
pub fn check_all_collisions(
    curve: &CubicBezier,
    obstacles: &[Obstacle],
    samples: u32,
) -> Vec<CollisionHit> {
    let mut hits = Vec::new();
    if obstacles.is_empty() || samples == 0 {
        return hits;
    }
    let rcp = 1.0 / samples as f32;
    for i in 0..=samples {
        let t = i as f32 * rcp;
        let pos = curve.position(t);
        for (idx, obs) in obstacles.iter().enumerate() {
            if obs.contains(pos) {
                hits.push(CollisionHit {
                    parameter: t,
                    position: pos,
                    obstacle_index: idx,
                });
            }
        }
    }
    hits
}

/// 軌道がクリア（衝突なし）かどうかを高速チェック。
#[must_use]
pub fn is_path_clear(curve: &CubicBezier, obstacles: &[Obstacle], samples: u32) -> bool {
    check_collision(curve, obstacles, samples).is_none()
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(feature = "std"))]
    use alloc::vec;

    #[test]
    fn sphere_contains() {
        let s = SphereObstacle::new(Vec3::ZERO, 1.0);
        assert!(s.contains(Vec3::ZERO));
        assert!(s.contains(Vec3::new(0.5, 0.0, 0.0)));
        assert!(!s.contains(Vec3::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn sphere_signed_distance() {
        let s = SphereObstacle::new(Vec3::ZERO, 1.0);
        assert!((s.signed_distance(Vec3::ZERO) - (-1.0)).abs() < 0.05);
        assert!((s.signed_distance(Vec3::new(2.0, 0.0, 0.0)) - 1.0).abs() < 0.05);
    }

    #[test]
    fn aabb_contains() {
        let a = AabbObstacle::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(a.contains(Vec3::ZERO));
        assert!(a.contains(Vec3::new(1.0, 1.0, 1.0)));
        assert!(!a.contains(Vec3::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn aabb_signed_distance_outside() {
        let a = AabbObstacle::new(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0));
        let d = a.signed_distance(Vec3::new(2.0, 0.5, 0.5));
        assert!(d > 0.0);
    }

    #[test]
    fn aabb_signed_distance_inside() {
        let a = AabbObstacle::new(Vec3::ZERO, Vec3::new(2.0, 2.0, 2.0));
        let d = a.signed_distance(Vec3::new(1.0, 1.0, 1.0));
        assert!(d < 0.0);
    }

    #[test]
    fn obstacle_enum_contains() {
        let obs = Obstacle::Sphere(SphereObstacle::new(Vec3::ZERO, 1.0));
        assert!(obs.contains(Vec3::ZERO));
        assert!(!obs.contains(Vec3::new(5.0, 0.0, 0.0)));
    }

    #[test]
    fn collision_straight_through_sphere() {
        // (0,0,0) → (10,0,0) の直線が (5,0,0) 半径1の球を通過
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let obs = vec![Obstacle::Sphere(SphereObstacle::new(
            Vec3::new(5.0, 0.0, 0.0),
            1.0,
        ))];
        let hit = check_collision(&curve, &obs, 100);
        assert!(hit.is_some());
        let h = hit.unwrap();
        assert_eq!(h.obstacle_index, 0);
        assert!(h.parameter > 0.3 && h.parameter < 0.7);
    }

    #[test]
    fn collision_miss() {
        // (0,0,0) → (10,0,0) の直線が (5,5,0) 半径1の球を通過しない
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let obs = vec![Obstacle::Sphere(SphereObstacle::new(
            Vec3::new(5.0, 5.0, 0.0),
            1.0,
        ))];
        assert!(is_path_clear(&curve, &obs, 100));
    }

    #[test]
    fn collision_with_aabb() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let obs = vec![Obstacle::Aabb(AabbObstacle::new(
            Vec3::new(4.0, -1.0, -1.0),
            Vec3::new(6.0, 1.0, 1.0),
        ))];
        assert!(!is_path_clear(&curve, &obs, 100));
    }

    #[test]
    fn all_collisions_count() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let obs = vec![Obstacle::Sphere(SphereObstacle::new(
            Vec3::new(5.0, 0.0, 0.0),
            2.0,
        ))];
        let hits = check_all_collisions(&curve, &obs, 100);
        // 半径2の球を直径4分通過 → 複数サンプルがヒット
        assert!(hits.len() > 1);
    }

    #[test]
    fn empty_obstacles() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        assert!(is_path_clear(&curve, &[], 100));
    }

    #[test]
    fn zero_samples() {
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        let obs = vec![Obstacle::Sphere(SphereObstacle::new(Vec3::ZERO, 100.0))];
        assert!(check_collision(&curve, &obs, 0).is_none());
    }
}
