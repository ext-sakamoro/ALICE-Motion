//! リアルタイム軌道修正 — 走行中の軌道再計画とブレンド。
//!
//! 実行中の軌道を中断して新軌道にスムーズに切り替える。
//! 時間ベースのブレンド補間で急激な速度変化を防止。

use crate::bezier::CubicBezier;
use crate::vec3::Vec3;

/// ブレンドモード。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlendMode {
    /// 線形補間 (lerp)。
    Linear,
    /// smoothstep 補間（滑らかな遷移）。
    Smoothstep,
    /// 即時切替（ブレンドなし）。
    Immediate,
}

/// 軌道ブレンダー設定。
#[derive(Debug, Clone, Copy)]
pub struct BlendConfig {
    /// ブレンド時間（秒）。
    pub blend_duration: f32,
    /// ブレンドモード。
    pub mode: BlendMode,
}

impl BlendConfig {
    /// デフォルト設定 (0.5秒, Smoothstep)。
    #[must_use]
    pub const fn new(blend_duration: f32) -> Self {
        Self {
            blend_duration,
            mode: BlendMode::Smoothstep,
        }
    }
}

/// 軌道スナップショット (再計画時の現在状態)。
#[derive(Debug, Clone, Copy)]
pub struct TrajectorySnapshot {
    /// 現在位置。
    pub position: Vec3,
    /// 現在速度。
    pub velocity: Vec3,
    /// スナップショット時刻。
    pub time: f32,
}

/// 再計画された軌道。
///
/// 旧軌道と新軌道をブレンドして滑らかに遷移する。
#[derive(Debug, Clone)]
pub struct ReplanTrajectory {
    /// 旧軌道のスナップショット。
    snapshot: TrajectorySnapshot,
    /// 新しい目標軌道 (Bezier)。
    new_curve: CubicBezier,
    /// 新軌道の総時間。
    new_duration: f32,
    /// ブレンド設定。
    blend: BlendConfig,
}

impl ReplanTrajectory {
    /// 再計画軌道を作成。
    ///
    /// `snapshot` は再計画時点の状態。
    /// `new_curve` は新しい目標軌道。
    #[must_use]
    pub const fn new(
        snapshot: TrajectorySnapshot,
        new_curve: CubicBezier,
        new_duration: f32,
        blend: BlendConfig,
    ) -> Self {
        Self {
            snapshot,
            new_curve,
            new_duration: new_duration.max(0.001),
            blend,
        }
    }

    /// 再計画後の位置を評価。
    ///
    /// `t` は再計画時点からの経過時間。
    #[must_use]
    pub fn position(&self, t: f32) -> Vec3 {
        let t_clamped = t.clamp(0.0, self.new_duration);
        let param = t_clamped / self.new_duration;
        let new_pos = self.new_curve.position(param);

        if self.blend.mode == BlendMode::Immediate || t >= self.blend.blend_duration {
            return new_pos;
        }

        // 旧軌道の外挿位置
        let old_pos = self.snapshot.position + self.snapshot.velocity * t;

        let alpha = blend_factor(t, self.blend.blend_duration, self.blend.mode);
        old_pos.lerp(new_pos, alpha)
    }

    /// 新軌道の全体時間。
    #[must_use]
    pub const fn duration(&self) -> f32 {
        self.new_duration
    }

    /// ブレンドが完了したか (ブレンド時間を超過)。
    #[must_use]
    pub fn is_blend_complete(&self, t: f32) -> bool {
        self.blend.mode == BlendMode::Immediate || t >= self.blend.blend_duration
    }
}

/// 現在位置から目標への再計画 Bezier を自動生成。
///
/// 現在の速度方向を考慮して制御点を配置し、
/// 滑らかな遷移カーブを作る。
#[must_use]
pub fn create_replan_curve(snapshot: &TrajectorySnapshot, target: Vec3) -> CubicBezier {
    let p0 = snapshot.position;
    let p3 = target;
    let vel_len = snapshot.velocity.length();

    // 速度方向に沿って第1制御点を配置
    let tangent_scale = if vel_len > 1e-6 { vel_len * 0.33 } else { 0.33 };
    let vel_dir = if vel_len > 1e-6 {
        snapshot.velocity * (1.0 / vel_len)
    } else {
        let diff = p3 - p0;
        let diff_len = diff.length();
        if diff_len > 1e-6 {
            diff * (1.0 / diff_len)
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        }
    };

    let p1 = p0 + vel_dir * tangent_scale;
    // 第2制御点は目標付近
    let to_target = p3 - p0;
    let p2 = p3 - to_target * 0.33;

    CubicBezier::new(p0, p1, p2, p3)
}

/// ブレンド係数を計算 (0→1)。
fn blend_factor(t: f32, duration: f32, mode: BlendMode) -> f32 {
    if duration < 1e-8 {
        return 1.0;
    }
    let ratio = (t / duration).clamp(0.0, 1.0);
    match mode {
        BlendMode::Linear => ratio,
        BlendMode::Smoothstep => ratio * ratio * (3.0 - 2.0 * ratio),
        BlendMode::Immediate => 1.0,
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blend_mode_eq() {
        assert_eq!(BlendMode::Linear, BlendMode::Linear);
        assert_ne!(BlendMode::Linear, BlendMode::Smoothstep);
    }

    #[test]
    fn blend_config_default() {
        let cfg = BlendConfig::new(0.5);
        assert!((cfg.blend_duration - 0.5).abs() < 1e-6);
        assert_eq!(cfg.mode, BlendMode::Smoothstep);
    }

    #[test]
    fn blend_factor_linear() {
        assert!((blend_factor(0.0, 1.0, BlendMode::Linear)).abs() < 1e-6);
        assert!((blend_factor(0.5, 1.0, BlendMode::Linear) - 0.5).abs() < 1e-6);
        assert!((blend_factor(1.0, 1.0, BlendMode::Linear) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn blend_factor_smoothstep() {
        assert!((blend_factor(0.0, 1.0, BlendMode::Smoothstep)).abs() < 1e-6);
        assert!((blend_factor(0.5, 1.0, BlendMode::Smoothstep) - 0.5).abs() < 1e-6);
        assert!((blend_factor(1.0, 1.0, BlendMode::Smoothstep) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn blend_factor_immediate() {
        assert!((blend_factor(0.0, 1.0, BlendMode::Immediate) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn replan_immediate_mode() {
        let snap = TrajectorySnapshot {
            position: Vec3::ZERO,
            velocity: Vec3::new(1.0, 0.0, 0.0),
            time: 0.0,
        };
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0));
        let blend = BlendConfig {
            blend_duration: 0.0,
            mode: BlendMode::Immediate,
        };
        let traj = ReplanTrajectory::new(snap, curve, 2.0, blend);
        assert!(traj.is_blend_complete(0.0));
    }

    #[test]
    fn replan_position_at_start() {
        let snap = TrajectorySnapshot {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            time: 0.0,
        };
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
        let blend = BlendConfig::new(0.5);
        let traj = ReplanTrajectory::new(snap, curve, 2.0, blend);
        let pos = traj.position(0.0);
        assert!((pos - Vec3::ZERO).length() < 0.1);
    }

    #[test]
    fn replan_position_at_end() {
        let snap = TrajectorySnapshot {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            time: 0.0,
        };
        let target = Vec3::new(10.0, 0.0, 0.0);
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, target);
        let blend = BlendConfig::new(0.5);
        let traj = ReplanTrajectory::new(snap, curve, 2.0, blend);
        let pos = traj.position(2.0);
        assert!((pos - target).length() < 0.1);
    }

    #[test]
    fn replan_blend_complete() {
        let snap = TrajectorySnapshot {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            time: 0.0,
        };
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(5.0, 0.0, 0.0));
        let blend = BlendConfig::new(0.5);
        let traj = ReplanTrajectory::new(snap, curve, 2.0, blend);
        assert!(!traj.is_blend_complete(0.3));
        assert!(traj.is_blend_complete(0.5));
    }

    #[test]
    fn create_replan_curve_basic() {
        let snap = TrajectorySnapshot {
            position: Vec3::new(1.0, 0.0, 0.0),
            velocity: Vec3::new(2.0, 0.0, 0.0),
            time: 0.5,
        };
        let target = Vec3::new(5.0, 0.0, 0.0);
        let curve = create_replan_curve(&snap, target);
        // 始点はスナップショット位置
        let start = curve.position(0.0);
        assert!((start - snap.position).length() < 1e-5);
        // 終点は目標
        let end = curve.position(1.0);
        assert!((end - target).length() < 1e-5);
    }

    #[test]
    fn replan_duration() {
        let snap = TrajectorySnapshot {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            time: 0.0,
        };
        let curve = CubicBezier::from_endpoints(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0));
        let traj = ReplanTrajectory::new(snap, curve, 3.0, BlendConfig::new(1.0));
        assert!((traj.duration() - 3.0).abs() < 1e-6);
    }
}
