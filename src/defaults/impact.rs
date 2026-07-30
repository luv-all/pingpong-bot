//! 랠리 임팩트·리턴 휴리스틱.

use anyhow::{Result, ensure};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpactParams {
    pub net_clearance: f64,
    pub rally_time_to_bounce: f64,
    pub racket_effective_restitution: f64,
    /// Rapier 라켓 collider 접선 마찰.
    pub racket_friction: f64,
    pub max_return_speed: f64,
    /// 랠리 리턴 바운드 목표의 y — `table::LENGTH_Y`에 대한 비율
    /// (WP3, 2026-07-30). 상대 코트는 `(0.5, 1.0]` — 0.5보다 크면 네트
    /// 너머, 1.0이면 상대 엔드라인. 낮출수록 목표까지 거리가 짧아져
    /// 필요 출사속도 `|v_out|`가 줄고, 그 결과 `peak_joint_speed_ratio`도
    /// 낮아진다(`docs/wp3-rally-target-distance.md` 참고).
    pub rally_target_y_frac: f64,
}

impl ImpactParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.net_clearance >= 0.0, "net_clearance >= 0");
        ensure!(self.rally_time_to_bounce > 0.0, "rally_time_to_bounce > 0");
        ensure!(
            self.racket_effective_restitution > 0.0,
            "racket_effective_restitution > 0"
        );
        ensure!(
            (0.0..=1.0).contains(&self.racket_friction),
            "racket_friction in 0..=1"
        );
        ensure!(self.max_return_speed > 0.0, "max_return_speed > 0");
        ensure!(
            (0.5..=1.0).contains(&self.rally_target_y_frac),
            "rally_target_y_frac in 0.5..=1.0 (상대 코트 안)"
        );
        return Ok(());
    }
}

impl Default for ImpactParams {
    fn default() -> Self {
        return Self {
            net_clearance: 0.08,
            rally_time_to_bounce: 0.55,
            racket_effective_restitution: 0.55,
            racket_friction: 0.5,
            max_return_speed: 6.0,
            rally_target_y_frac: 0.75,
        };
    }
}
