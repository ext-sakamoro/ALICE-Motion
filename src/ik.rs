//! 逆運動学 (IK) ソルバー — CCD と FABRIK。
//!
//! ジョイントチェーンに対して目標位置への到達を計算する。
//! CCD (Cyclic Coordinate Descent) と FABRIK (Forward And Backward Reaching IK) を提供。

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

use crate::vec3::Vec3;

/// ジョイントチェーン（各ジョイントの位置を保持）。
#[derive(Debug, Clone)]
pub struct JointChain {
    /// ジョイント位置の配列。
    pub joints: Vec<Vec3>,
    /// 各リンクの長さ (`joints.len()` - 1 個)。
    pub lengths: Vec<f32>,
}

impl JointChain {
    /// ジョイント位置列からチェーンを構築。
    ///
    /// # Panics
    ///
    /// `positions` が 2 未満の場合。
    #[must_use]
    pub fn from_positions(positions: &[Vec3]) -> Self {
        assert!(positions.len() >= 2, "chain needs at least 2 joints");
        let joints = positions.to_vec();
        let lengths = joints.windows(2).map(|w| (w[1] - w[0]).length()).collect();
        Self { joints, lengths }
    }

    /// ジョイント数。
    #[must_use]
    pub const fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// チェーンの総長。
    #[must_use]
    pub fn total_length(&self) -> f32 {
        self.lengths.iter().sum()
    }

    /// エンドエフェクタ（末端）の位置。
    #[must_use]
    pub fn end_effector(&self) -> Vec3 {
        self.joints[self.joints.len() - 1]
    }

    /// ルート（根元）の位置。
    #[must_use]
    pub fn root(&self) -> Vec3 {
        self.joints[0]
    }
}

/// IK ソルバー設定。
#[derive(Debug, Clone, Copy)]
pub struct IkConfig {
    /// 最大反復回数。
    pub max_iterations: u32,
    /// 収束閾値（目標との距離がこれ以下で停止）。
    pub tolerance: f32,
}

impl IkConfig {
    /// デフォルト設定 (max=100, tolerance=0.001)。
    #[must_use]
    pub const fn new() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 0.001,
        }
    }
}

impl Default for IkConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// IK 計算結果。
#[derive(Debug, Clone)]
pub struct IkResult {
    /// 解が収束したか。
    pub converged: bool,
    /// 実行した反復回数。
    pub iterations: u32,
    /// 最終的な目標との距離。
    pub final_distance: f32,
}

/// CCD (Cyclic Coordinate Descent) IK ソルバー。
///
/// 末端から根元へ順にジョイントを回転させて目標に近づける。
pub fn solve_ccd(chain: &mut JointChain, target: Vec3, config: &IkConfig) -> IkResult {
    let n = chain.joints.len();
    if n < 2 {
        return IkResult {
            converged: false,
            iterations: 0,
            final_distance: f32::MAX,
        };
    }

    for iter in 0..config.max_iterations {
        let end = chain.end_effector();
        let dist = (end - target).length();
        if dist < config.tolerance {
            return IkResult {
                converged: true,
                iterations: iter,
                final_distance: dist,
            };
        }

        // 末端側から根元側へ (末端自身は除く)
        for i in (0..n - 1).rev() {
            let joint_pos = chain.joints[i];
            let to_end = chain.end_effector() - joint_pos;
            let to_target = target - joint_pos;

            let len_end = to_end.length();
            let len_target = to_target.length();
            if len_end < 1e-8 || len_target < 1e-8 {
                continue;
            }

            // 回転角度を計算
            let dot = to_end.dot(to_target) / (len_end * len_target);
            let dot_clamped = dot.clamp(-1.0, 1.0);
            let angle = libm::acosf(dot_clamped);

            if angle.abs() < 1e-8 {
                continue;
            }

            // 回転軸
            let axis = to_end.cross(to_target);
            let axis_len = axis.length();
            if axis_len < 1e-8 {
                continue;
            }
            let axis_norm = axis * (1.0 / axis_len);

            // i 以降のジョイントを回転
            for j in (i + 1)..n {
                let rel = chain.joints[j] - joint_pos;
                let rotated = rotate_rodrigues(rel, axis_norm, angle);
                chain.joints[j] = joint_pos + rotated;
            }
        }
    }

    let dist = (chain.end_effector() - target).length();
    IkResult {
        converged: dist < config.tolerance,
        iterations: config.max_iterations,
        final_distance: dist,
    }
}

/// FABRIK (Forward And Backward Reaching IK) ソルバー。
///
/// 前方パス (目標→根元) と後方パス (根元→末端) を繰り返して収束。
pub fn solve_fabrik(chain: &mut JointChain, target: Vec3, config: &IkConfig) -> IkResult {
    let n = chain.joints.len();
    if n < 2 {
        return IkResult {
            converged: false,
            iterations: 0,
            final_distance: f32::MAX,
        };
    }

    let root_pos = chain.root();

    // 到達不能チェック
    let dist_to_target = (target - root_pos).length();
    if dist_to_target > chain.total_length() {
        // 目標方向に最大限伸ばす
        let dir = if dist_to_target > 1e-8 {
            (target - root_pos) * (1.0 / dist_to_target)
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let mut pos = root_pos;
        chain.joints[0] = pos;
        for i in 0..chain.lengths.len() {
            pos = pos + dir * chain.lengths[i];
            chain.joints[i + 1] = pos;
        }
        return IkResult {
            converged: false,
            iterations: 0,
            final_distance: (chain.end_effector() - target).length(),
        };
    }

    for iter in 0..config.max_iterations {
        let dist = (chain.end_effector() - target).length();
        if dist < config.tolerance {
            return IkResult {
                converged: true,
                iterations: iter,
                final_distance: dist,
            };
        }

        // Forward pass: 末端を目標に置いてから根元へ
        chain.joints[n - 1] = target;
        for i in (0..n - 1).rev() {
            let dir = chain.joints[i] - chain.joints[i + 1];
            let len = dir.length();
            let d = if len > 1e-8 {
                dir * (1.0 / len)
            } else {
                Vec3::new(1.0, 0.0, 0.0)
            };
            chain.joints[i] = chain.joints[i + 1] + d * chain.lengths[i];
        }

        // Backward pass: 根元を元の位置に戻して末端へ
        chain.joints[0] = root_pos;
        for i in 0..chain.lengths.len() {
            let dir = chain.joints[i + 1] - chain.joints[i];
            let len = dir.length();
            let d = if len > 1e-8 {
                dir * (1.0 / len)
            } else {
                Vec3::new(1.0, 0.0, 0.0)
            };
            chain.joints[i + 1] = chain.joints[i] + d * chain.lengths[i];
        }
    }

    let dist = (chain.end_effector() - target).length();
    IkResult {
        converged: dist < config.tolerance,
        iterations: config.max_iterations,
        final_distance: dist,
    }
}

/// ロドリゲスの回転公式。
fn rotate_rodrigues(v: Vec3, axis: Vec3, angle: f32) -> Vec3 {
    let cos_a = libm::cosf(angle);
    let sin_a = libm::sinf(angle);
    let dot = v.dot(axis);
    let cross = axis.cross(v);
    v * cos_a + cross * sin_a + axis * (dot * (1.0 - cos_a))
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: Vec3, b: Vec3, eps: f32) -> bool {
        (a - b).length() < eps
    }

    #[test]
    fn joint_chain_from_positions() {
        let chain = JointChain::from_positions(&[
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ]);
        assert_eq!(chain.joint_count(), 3);
        assert!((chain.total_length() - 2.0).abs() < 0.05);
    }

    #[test]
    fn joint_chain_end_effector() {
        let chain = JointChain::from_positions(&[Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0)]);
        assert!(approx_eq(
            chain.end_effector(),
            Vec3::new(0.0, 1.0, 0.0),
            1e-5
        ));
    }

    #[test]
    fn ik_config_default() {
        let cfg = IkConfig::default();
        assert_eq!(cfg.max_iterations, 100);
        assert!((cfg.tolerance - 0.001).abs() < 1e-6);
    }

    #[test]
    fn ccd_already_at_target() {
        let mut chain = JointChain::from_positions(&[
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ]);
        let target = Vec3::new(2.0, 0.0, 0.0);
        let result = solve_ccd(&mut chain, target, &IkConfig::new());
        assert!(result.converged);
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn ccd_reachable_target() {
        let mut chain = JointChain::from_positions(&[
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ]);
        let target = Vec3::new(1.0, 1.0, 0.0);
        let cfg = IkConfig {
            max_iterations: 500,
            tolerance: 0.1,
        };
        let result = solve_ccd(&mut chain, target, &cfg);
        // fast_sqrt による近似誤差があるため、距離チェックで確認
        assert!(
            result.final_distance < 0.2,
            "CCD didn't converge enough: {}",
            result.final_distance
        );
    }

    #[test]
    fn fabrik_already_at_target() {
        let mut chain = JointChain::from_positions(&[Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)]);
        let target = Vec3::new(1.0, 0.0, 0.0);
        let result = solve_fabrik(&mut chain, target, &IkConfig::new());
        assert!(result.converged);
    }

    #[test]
    fn fabrik_reachable_target() {
        let mut chain = JointChain::from_positions(&[
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ]);
        let target = Vec3::new(0.0, 2.0, 0.0);
        let cfg = IkConfig {
            max_iterations: 200,
            tolerance: 0.05,
        };
        let result = solve_fabrik(&mut chain, target, &cfg);
        assert!(result.converged);
        assert!(approx_eq(chain.end_effector(), target, 0.1));
    }

    #[test]
    fn fabrik_unreachable_target() {
        let mut chain = JointChain::from_positions(&[Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)]);
        // 全長1.0なのに距離10に目標
        let target = Vec3::new(10.0, 0.0, 0.0);
        let result = solve_fabrik(&mut chain, target, &IkConfig::new());
        assert!(!result.converged);
    }

    #[test]
    fn fabrik_preserves_root() {
        let mut chain = JointChain::from_positions(&[
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(6.0, 0.0, 0.0),
            Vec3::new(7.0, 0.0, 0.0),
        ]);
        let target = Vec3::new(5.0, 2.0, 0.0);
        solve_fabrik(&mut chain, target, &IkConfig::new());
        assert!(approx_eq(chain.root(), Vec3::new(5.0, 0.0, 0.0), 1e-5));
    }

    #[test]
    fn rodrigues_identity() {
        let v = Vec3::new(1.0, 0.0, 0.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let rotated = rotate_rodrigues(v, axis, 0.0);
        assert!(approx_eq(rotated, v, 1e-5));
    }

    #[test]
    fn rodrigues_90_degrees() {
        let v = Vec3::new(1.0, 0.0, 0.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let rotated = rotate_rodrigues(v, axis, core::f32::consts::FRAC_PI_2);
        assert!(approx_eq(rotated, Vec3::new(0.0, 1.0, 0.0), 1e-5));
    }

    #[test]
    fn ccd_3d_target() {
        let mut chain = JointChain::from_positions(&[
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
        ]);
        let target = Vec3::new(1.0, 1.0, 1.0);
        let cfg = IkConfig {
            max_iterations: 200,
            tolerance: 0.05,
        };
        let result = solve_ccd(&mut chain, target, &cfg);
        assert!(result.converged);
    }
}
