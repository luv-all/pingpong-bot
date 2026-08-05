//! 타점 정책 — "언제 칠 만한가". 필터 튜닝은 [`crate::vision::ekf`] 상수로 옮겼다.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorParams {
    pub min_lead: f64,
    pub max_lead: f64,
    pub integrate_dt: f64,
    pub min_approach_speed_y: f64,
    pub min_strike_clearance: f64,
    /// 커밋을 허용할 **예측 도달점 불확실성** 상한 [m].
    ///
    /// `σ_impact ≈ hypot(σ_p, σ_v × 리드타임)`. 속도는 측정되지 않고 위치 차분에서 나오므로
    /// 시드 직후 σ_v가 1~2 m/s다 — 그 상태의 예측은 미터 단위로 틀린다. 필터는 그걸 공분산으로
    /// 이미 알고 있으니, 확신이 설 때까지 기다렸다 커밋한다.
    ///
    /// 클립 3개 실측(커밋 창 안):
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
    /// **대가**: 검출이 나쁜 샷은 확신에 도달하지 못해 아예 안 친다. 엉뚱한 데를 치는 것보다는
    /// 낫지만 공짜가 아니다 — 검출률이 오르면 이 값을 다시 조일 수 있다.
    ///
    /// 이 표는 구 파이프라인(3D 관측 EKF) 기준이다. 픽셀 관측으로 바꾼 뒤 재측정해야 한다.
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
            max_impact_sigma: 0.15,
        };
    }
}
