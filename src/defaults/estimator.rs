//! 궤적 추정(EKF·탄도) 휴리스틱.

use anyhow::{Result, ensure};

use crate::defaults::PhysicsParams;
use crate::estimator::Ekf;

/// 임베드 `[physics]` 기본값으로 빈 필터.
impl Default for Ekf {
    fn default() -> Self {
        return Self::with_physics(PhysicsParams::default());
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorParams {
    pub min_lead: f64,
    pub max_lead: f64,
    pub integrate_dt: f64,
    pub min_approach_speed_y: f64,
    pub min_strike_clearance: f64,
    pub q_pos: f64,
    pub q_vel: f64,
    /// 측정 분산 [m²] — σ 3.0 cm.
    ///
    /// **고주파 측정 노이즈만으로 정하면 안 된다.** 클립 9개에서 2계 차분으로 잰 순수
    /// 관측 노이즈는 σ 0.9~1.7 cm이고, 그 값(`r_meas` 1.7e-4)을 넣어 봤더니 예측이 오히려
    /// 나빠졌다 (fly_08 커밋 창 최대 오차 12 → 40 cm).
    ///
    /// 이유: R이 작아지면 필터가 과신해 공분산이 빨리 줄고, `max_impact_sigma` 게이트가
    /// 아직 부정확한 예측까지 통과시킨다. 반면 예측 오차에는 2계 차분이 상쇄해 못 보는
    /// **매끄러운 성분**이 들어간다 — 항력 0·스핀 0 가정, 바람, 캘리브 편향. R은 그 몫까지
    /// 흡수해야 σ가 실제 오차를 제대로 예측한다.
    ///
    /// 보정도(커밋 창 안 `실제 오차 / 필터가 말한 σ`, 1이면 정직):
    ///
    /// | r_meas | fly_01 | fly_05 | fly_08 | fly_09 |
    /// |--------|--------|--------|--------|--------|
    /// | 1.7e-4 | 1.61   | 2.63   | 2.81   | 1.16   | ← 2배 과신
    /// | **9e-4** | **0.94** | **1.20** | **0.74** | **0.53** | ← 채택
    /// | 1.6e-3 | 0.71   | 0.91   | 0.56   | 0.40   | ← 보수적
    ///
    /// 정확도/기회 수 절충은 R이 아니라 `max_impact_sigma`로 조절한다 — R은 정직해야 한다.
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
    /// 커밋을 허용할 **예측 도달점 불확실성** 상한 [m].
    ///
    /// `σ_impact ≈ hypot(σ_p, σ_v × 리드타임)`. 속도는 측정되지 않고 위치 차분에서 나오므로
    /// 시드 직후 σ_v가 1~2 m/s다 — 그 상태의 예측은 미터 단위로 틀린다. 필터는 그걸 공분산으로
    /// 이미 알고 있으니, 확신이 설 때까지 기다렸다 커밋한다.
    ///
    /// 클립 3개 실측(`diag_clip_prediction`, 커밋 창 안):
    ///
    /// | 임계 | fly_03 평균/최대 | fly_04 평균/최대 | 가장 이른 리드 |
    /// |------|-----------------|-----------------|---------------|
    /// | 없음 | 39 / 153 cm     | 69 / 245 cm     | —             |
    /// | 0.30 | 19 / 54         | 23 / 49         | 0.37 s        |
    /// | 0.20 | 13 / 25         | 23 / 49         | 0.34 s        |
    /// | 0.15 | 11 / 13         | 16 / 23         | 0.32 s        |
    /// | 0.10 | 9.5 / 13        | 9.5 / 10        | 0.24 s        |
    ///
    /// 0.15 채택 — 최대 오차를 13~23 cm로 누르면서 리드 0.32 s가 남아 스윙 시간이 충분하다.
    /// 0.10은 리드 0.24 s까지 밀려 빠듯하고, 0.20은 fly_04의 49 cm를 못 막는다.
    ///
    /// **대가**: 검출이 나쁜 샷은 확신에 도달하지 못해 아예 안 친다 (fly_02는 0.20 이하에서
    /// 커밋 창에 남는 예측이 0회). 엉뚱한 데를 치는 것보다는 낫지만 공짜가 아니다 —
    /// 검출률이 오르면 이 값을 다시 조일 수 있다.
    pub max_impact_sigma: f64,
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
        ensure!(self.max_impact_sigma > 0.0, "max_impact_sigma > 0");
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
            max_impact_sigma: 0.15,
        };
    }
}
