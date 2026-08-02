//! 기구학으로 나가는 계약. 비전이 내보내는 타입은 [`Trajectory`] 하나다.

use std::ops::Deref;
use std::time::{Duration, Instant};

use crate::{Point3, Vector3};

/// 한 시점의 공 상태 — `[x y z vx vy vz t]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    /// [`Trajectory::origin`] 기준 경과. 벽시계는 `origin + t`.
    pub t: Duration,
    pub position: Point3,
    pub velocity: Vector3,
    /// 축별 1σ [m]. 스칼라로 합치면 잘 관측되는 축이 나머지를 가린다.
    pub sigma_position: Vector3,
    /// 축별 1σ [m/s].
    pub sigma_velocity: Vector3,
    /// 각속도 [rad/s]. 아직 추정하지 않아 항상 `None`.
    pub spin: Option<Vector3>,
}

/// 시간순 상태 나열.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Track(pub Vec<State>);

impl Deref for Track {
    type Target = [State];

    fn deref(&self) -> &[State] {
        return &self.0;
    }
}

impl Track {
    /// `t` 시점 (표본 사이 선형 보간). 궤적 밖이면 `None`.
    pub fn at_time(&self, t: Duration) -> Option<State> {
        if t < self.first()?.t || t > self.last()?.t {
            return None;
        }
        // `first.t <= t`라 첫 표본이 술어를 통과한다 — index는 항상 1 이상.
        let index = self.partition_point(|s| s.t <= t);
        let Some(later) = self.get(index) else {
            return self.last().copied(); // t == last.t
        };
        let earlier = &self[index - 1];
        let since = |x: Duration| x.saturating_sub(earlier.t).as_secs_f64();
        return Some(lerp(earlier, later, since(t), since(later.t)));
    }

    /// `y` 평면을 로봇 쪽으로 지나는 첫 상태 — 타점 후보.
    pub fn at_plane(&self, y: f64) -> Option<State> {
        let pair = self
            .windows(2)
            .find(|w| w[0].position.y >= y && w[1].position.y < y)?;
        let (a, b) = (&pair[0], &pair[1]);
        return Some(lerp(a, b, a.position.y - y, a.position.y - b.position.y));
    }
}

/// 공 하나의 궤적.
///
/// 둘은 이어붙는 게 아니라 겹친다. `measured`는 공 시작부터, `predicted`는 트리거부터,
/// 둘 다 로봇까지다. 겹치는 구간의 차이가 예측 오차다.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    /// "같은 공인가"의 근거.
    pub seq: u64,
    /// `t = 0`의 벽시계.
    pub origin: Instant,
    /// 검출이 실패한 프레임엔 점이 없다.
    pub measured: Track,
    /// 트리거 순간에 한 번 만들고 고정한다.
    pub predicted: Track,
}

/// 가중치는 `num / den`. σ도 같이 섞어야 표본 사이에서 확신이 커지지 않는다.
fn lerp(a: &State, b: &State, num: f64, den: f64) -> State {
    if den <= f64::EPSILON {
        return *b;
    }
    let w = num / den;
    let mix = |x: Vector3, y: Vector3| x + (y - x) * w;
    return State {
        t: a.t + b.t.saturating_sub(a.t).mul_f64(w),
        position: a.position.lerp(&b.position, w),
        velocity: mix(a.velocity, b.velocity),
        sigma_position: mix(a.sigma_position, b.sigma_position),
        sigma_velocity: mix(a.sigma_velocity, b.sigma_velocity),
        spin: match (a.spin, b.spin) {
            (Some(x), Some(y)) => Some(mix(x, y)),
            _ => None,
        },
    };
}

#[cfg(test)]
#[path = "contract_tests.rs"]
mod tests;
