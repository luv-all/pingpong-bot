//! 주기 로그 스로틀.
//!
//! 카메라 2대 × 120 fps면 틱당 로그는 초당 수백 줄이 된다. 진척 로그는 이걸로 묶는다.

use std::time::{Duration, Instant};

/// 고정 주기로 한 번씩만 통과시킨다. 첫 호출은 항상 통과.
pub struct Throttle {
    period: Duration,
    last: Option<Instant>,
}

impl Throttle {
    pub fn new(period: Duration) -> Self {
        return Self { period, last: None };
    }

    /// 주기가 지났으면 `true`를 주고 타이머를 리셋한다.
    pub fn ready(&mut self) -> bool {
        let now = Instant::now();
        if self
            .last
            .is_some_and(|last| now.duration_since(last) < self.period)
        {
            return false;
        }
        self.last = Some(now);
        return true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_call_always_passes() {
        let mut throttle = Throttle::new(Duration::from_secs(60));
        assert!(throttle.ready());
    }

    #[test]
    fn blocks_until_the_period_elapses() {
        let mut throttle = Throttle::new(Duration::from_secs(60));
        assert!(throttle.ready());
        assert!(!throttle.ready());
        assert!(!throttle.ready());
    }

    #[test]
    fn passes_again_once_the_period_is_zero() {
        let mut throttle = Throttle::new(Duration::ZERO);
        assert!(throttle.ready());
        assert!(throttle.ready(), "주기 0이면 매번 통과");
    }
}
