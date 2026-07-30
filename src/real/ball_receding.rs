//! 추정 공 y가 로봇에서 멀어지는지(증가) 히스테리시스로 본다.
//!
//! 로봇은 y≈0, 상대/급구는 y→LENGTH_Y. y 증가 = 새 급구 루프 후보.
//! 노이즈로 EKF가 매 프레임 리셋되지 않게 Δy·연속 샘플을 요구한다.

/// 한 샘플에서 인정할 최소 y 증가 [m].
pub const MIN_DELTA_Y: f64 = 0.05;
/// `MIN_DELTA_Y` 이상 증가가 연속으로 이만큼 나와야 확정.
pub const MIN_SAMPLES: u32 = 3;

/// 공 y 증가(로봇에서 멀어짐) 검출기.
#[derive(Debug, Clone)]
pub struct BallReceding {
    min_delta_y: f64,
    min_samples: u32,
    last_y: Option<f64>,
    streak: u32,
}

impl BallReceding {
    pub fn new(min_delta_y: f64, min_samples: u32) -> Self {
        return Self {
            min_delta_y,
            min_samples,
            last_y: None,
            streak: 0,
        };
    }

    pub fn reset(&mut self) {
        self.last_y = None;
        self.streak = 0;
    }

    /// `true` = 새 루프 신호. 호출 측에서 EKF 리셋 후 이 검출기도 `reset`할 것.
    pub fn observe(&mut self, ball_y: f64) -> bool {
        let Some(prev) = self.last_y else {
            self.last_y = Some(ball_y);
            return false;
        };
        self.last_y = Some(ball_y);
        if ball_y - prev >= self.min_delta_y {
            self.streak = self.streak.saturating_add(1);
        } else {
            self.streak = 0;
        }
        if self.streak >= self.min_samples {
            self.streak = 0;
            return true;
        }
        return false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ignores_small_noise() {
        let mut d = BallReceding::new(0.05, 3);
        assert!(!d.observe(1.0));
        assert!(!d.observe(1.02));
        assert!(!d.observe(1.04));
        assert!(!d.observe(1.06));
    }

    #[test]
    fn fires_after_sustained_increase() {
        let mut d = BallReceding::new(0.05, 3);
        assert!(!d.observe(0.5));
        assert!(!d.observe(0.56));
        assert!(!d.observe(0.62));
        assert!(d.observe(0.68));
    }

    #[test]
    fn decreasing_y_clears_streak() {
        let mut d = BallReceding::new(0.05, 3);
        assert!(!d.observe(0.5));
        assert!(!d.observe(0.56));
        assert!(!d.observe(0.50));
        assert!(!d.observe(0.56));
        assert!(!d.observe(0.62));
        assert!(d.observe(0.68));
    }
}
