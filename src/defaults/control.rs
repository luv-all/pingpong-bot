//! 스윙·관절 추종 휴리스틱.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlParams {
    pub min_swing_secs: f64,
    pub swing_commit_max_secs: f64,
    pub swing_follow_through_secs: f64,
    pub swing_commit_max_ball_y_frac: f64,
    pub ekf_meas_jump_m: f64,
    pub max_joint_accel: f64,
    /// 관절별 토크 상한 [N·m]. yaw 듀얼 MX-64면 `[12, 6, 6, 6]`.
    pub max_joint_torques: [f64; 4],
    pub joint_inertia: f64,
    pub racket_open_pitch: f64,
}

impl ControlParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.min_swing_secs > 0.0, "min_swing_secs > 0");
        ensure!(
            self.swing_commit_max_secs >= self.min_swing_secs,
            "swing_commit_max_secs >= min_swing_secs"
        );
        ensure!(self.swing_follow_through_secs >= 0.0, "follow_through >= 0");
        ensure!(
            (0.0..=1.0).contains(&self.swing_commit_max_ball_y_frac),
            "swing_commit_max_ball_y_frac in 0..=1"
        );
        ensure!(self.ekf_meas_jump_m > 0.0, "ekf_meas_jump_m > 0");
        ensure!(self.max_joint_accel > 0.0, "max_joint_accel > 0");
        ensure!(
            self.max_joint_torques.iter().all(|&t| t > 0.0),
            "max_joint_torques > 0"
        );
        ensure!(self.joint_inertia > 0.0, "joint_inertia > 0");
        return Ok(());
    }
}

pub fn control() -> ControlParams {
    return ControlParams {
        min_swing_secs: 0.08,
        swing_commit_max_secs: 0.35,
        swing_follow_through_secs: 0.06,
        swing_commit_max_ball_y_frac: 0.55,
        ekf_meas_jump_m: 0.6,
        max_joint_accel: 400.0,
        // yaw 듀얼 MX-64 stall≈6 → 12; 나머지 단일. I는 α≈τ/I가 스윙 가능하도록.
        max_joint_torques: [12.0, 6.0, 6.0, 6.0],
        joint_inertia: 0.015,
        racket_open_pitch: 0.45,
    };
}
