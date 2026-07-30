//! 스윙·관절 추종 휴리스틱.

use anyhow::{Result, ensure};

use crate::defaults::dxl_limits::joint_torque_limits_4dof_array;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlParams {
    /// 스윙 커밋 최소 소요시간 [s] — 이보다 임팩트까지 남은 시간이 짧으면
    /// 스윙을 포기한다.
    ///
    /// **2026-07-30 재조정.** WP2a는 단일 고정 임팩트 목표 스윕으로 실행
    /// 가능 하한을 0.24s로 추정했으나, 그 값을 그대로 넣자 실제 eval+랜덤
    /// 67샷 그리드(`diag_swing_commit_rate_across_shot_grid`) 커밋률이
    /// 0%로 붕괴했다 — 원인은 이 상수가 아니라 `try_auto_swing`의 포기
    /// 판정이 **가장 먼저 지나가는 평면**(로봇에서 먼 y_max, tti가 가장
    /// 작음)의 tti만 보고 있던 별개 버그였다(`world.rs`, `soonest_tti`→
    /// `latest_tti`로 수정 — 실제로는 "모든 평면이 다 늦었는가"를 봐야
    /// 하므로 `min`이 아니라 `max`가 맞다). 그 버그를 고친 뒤 같은 67샷
    /// 그리드로 이 값을 직접 스윕한 결과 **0.08~0.20s 전 구간이 커밋률
    /// 손실 없이 동일(75%, 50/67)**했고 0.24s부터 1샷 손실(73%)이
    /// 나타났다 — 그래서 0.20을 채택했다(옛 하한 0.24보다 살짝 낮춰
    /// 여유를 둠). 단일 목표 스윕(0.24) 자체는 틀리지 않았지만, 실제
    /// 그리드 검증 없이 곧바로 반영하면 위 버그 같은 상호작용을 놓친다 —
    /// 이 상수를 다시 바꿀 때는 반드시 `diag_swing_commit_rate_across_shot_grid`로
    /// 재검증할 것.
    pub min_swing_secs: f64,
    /// 스윙 커밋 최대 소요시간 [s] — 이보다 임팩트까지 남은 시간이 길면
    /// 아직 커밋하지 않고 대기한다(예측이 더 안정화되길 기다림).
    ///
    /// WP2a 실측: 옛값 0.35는 과보수적이었다 — 같은 단일목표 스윕에서
    /// 0.60s(테스트 상한)까지 토크 여유가 단조 개선(τ 0.94x→0.52x)되며
    /// 계속 실행 가능했다. **2026-07-30 확인**: 이 값은 67샷 그리드
    /// 커밋률에는 0.35~0.60 전 구간에서 영향이 없었다(항상 75%) — 즉
    /// 커밋률을 깎지 않으면서 스윙 품질(토크 여유)만 얻는 순수 이득이라
    /// 0.60 그대로 채택. 0.60은 테스트한 범위의 상한일 뿐 증명된 천장이
    /// 아니다 — 더 늘릴 근거를 찾으면 재조정 대상.
    pub swing_commit_max_secs: f64,
    pub swing_follow_through_secs: f64,
    pub swing_commit_max_ball_y_frac: f64,
    pub ekf_meas_jump_m: f64,
    pub max_joint_accel: f64,
    /// 관절별 토크 상한 [N·m] — [`joint_torque_limits_4dof_array`] SSOT
    /// (stall×derate, yaw 듀얼 포함).
    pub max_joint_torques: [f64; 4],
    pub joint_inertia: f64,
    pub racket_open_pitch: f64,
    /// Real: 미사용(실기는 항상 Position Mode + max PWM/Current Limit).
    /// Sim: RNEA로 다물체 `motor_max_force` 상한 갱신.
    pub torque_feedforward: bool,
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

impl Default for ControlParams {
    fn default() -> Self {
        return Self {
            min_swing_secs: 0.20,
            swing_commit_max_secs: 0.60,
            swing_follow_through_secs: 0.06,
            swing_commit_max_ball_y_frac: 0.55,
            ekf_meas_jump_m: 0.6,
            max_joint_accel: 400.0,
            max_joint_torques: joint_torque_limits_4dof_array(),
            joint_inertia: 0.015,
            racket_open_pitch: 0.45,
            torque_feedforward: true,
        };
    }
}
