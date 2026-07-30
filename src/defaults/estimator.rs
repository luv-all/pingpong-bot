//! 궤적 추정(EKF·탄도) 휴리스틱.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorParams {
    pub min_lead: f64,
    pub max_lead: f64,
    pub integrate_dt: f64,
    pub min_approach_speed_y: f64,
    pub min_strike_clearance: f64,
    pub q_pos: f64,
    pub q_vel: f64,
    pub r_meas: f64,
    /// 측정 게이트 임계 — 마할라노비스 d² (자유도 3).
    ///
    /// `d² = yᵀ(P_pp + R)⁻¹y`, y는 예측 대비 잔차. 임계를 넘으면 그 측정은
    /// 무시한다. 공분산으로 정규화하므로 필터가 확신할수록 게이트가 좁아지고
    /// 몇 프레임 놓쳐 P가 부풀면 저절로 넓어진다 — 거리 임계와 달리 공 속도별
    /// 재튜닝이 필요없다. 11.34 = χ²(3) 99% 분위 (≈3σ).
    pub gate_chi2: f64,
    /// 연속 거부 한도. 넘으면 "튄 게 아니라 필터가 틀렸다"로 보고 트랙을
    /// 버린 뒤 그 측정으로 재시드한다. 120 fps에서 5 ≈ 40 ms.
    pub gate_reject_limit: u32,
    /// 측정 공백 상한 [s]. 넘으면 하드 리셋 (세션 공백·프레임 드롭).
    pub stale_gap_secs: f64,
}

impl EstimatorParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.min_lead > 0.0, "min_lead > 0");
        ensure!(self.max_lead >= self.min_lead, "max_lead >= min_lead");
        ensure!(self.integrate_dt > 0.0, "integrate_dt > 0");
        ensure!(self.min_approach_speed_y > 0.0, "min_approach_speed_y > 0");
        ensure!(
            self.min_strike_clearance >= 0.0,
            "min_strike_clearance >= 0"
        );
        ensure!(self.q_pos >= 0.0, "q_pos >= 0");
        ensure!(self.q_vel >= 0.0, "q_vel >= 0");
        ensure!(self.r_meas > 0.0, "r_meas > 0");
        ensure!(self.gate_chi2 > 0.0, "gate_chi2 > 0");
        ensure!(self.gate_reject_limit >= 1, "gate_reject_limit >= 1");
        ensure!(self.stale_gap_secs > 0.0, "stale_gap_secs > 0");
        return Ok(());
    }
}

impl Default for EstimatorParams {
    fn default() -> Self {
        return Self {
            min_lead: 0.05,
            max_lead: 1.2,
            integrate_dt: 0.001,
            min_approach_speed_y: 0.8,
            min_strike_clearance: 0.05,
            q_pos: 1.0e-4,
            q_vel: 1.0e-2,
            r_meas: 0.0009,
            gate_chi2: 11.34,
            gate_reject_limit: 5,
            stale_gap_secs: 0.5,
        };
    }
}
