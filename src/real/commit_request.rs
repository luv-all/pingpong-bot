//! 추정 워커 → 제어 워커 메시지.

use std::time::Instant;

use pingpong_bot::robot::motion::{HitPlane, Prediction};
use pingpong_bot::vision::Trajectory;

/// "지금 이 궤적으로 스윙을 계획해 보라"는 요청.
///
/// 비전이 내놓는 계약([`Trajectory`])을 **그대로** 실어 나른다. 예전에는 추정 워커가 접수
/// 평면마다 잘라 [`Prediction`] 목록으로 바꿔 보냈는데, 그러면 비전이 로봇의 도달 범위를
/// 알아야 한다 — 도달 범위가 바뀔 때마다 비전 코드가 따라 바뀌는 층위 위반이다.
///
/// 자르는 일은 [`Self::predictions`]가 한다. 그건 플래너가 아직 `Trajectory` 를 모르기
/// 때문에 있는 어댑터고, 플래너가 배우면 사라진다.
pub struct CommitRequest {
    pub trajectory: Trajectory,
    /// 예측을 만든 시각. 제어 워커가 계획을 시작할 때까지 흐른 만큼 낡는다 —
    /// 오래된 요청은 계획하지 말고 버려야 한다 ([`Self::age_secs`]).
    pub at: Instant,
}

impl CommitRequest {
    /// 요청이 만들어진 뒤 흐른 시간 [s].
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }

    /// 마지막으로 **본** 공 y [m] — 로그용.
    ///
    /// [`Self::at`] 시각으로 보간하지 않는다. 표본 사이를 이으면 관측이 아니라 모델을 찍게
    /// 되는데, "요청할 때 공이 어디였나"를 로그로 되짚을 땐 실제로 본 값이어야 한다.
    /// 보간이 필요한 곳(`at_plane`·`at_time`)은 계약이 이미 해 준다.
    pub fn ball_y(&self) -> f64 {
        return self
            .trajectory
            .measured
            .last()
            .map_or(f64::NAN, |state| state.position.y);
    }

    /// 접수 평면마다 잘라 플래너가 아는 모양으로.
    pub fn predictions(&self, planes: &[HitPlane]) -> Vec<Prediction> {
        return predictions_at(&self.trajectory, planes);
    }
}

/// 예측 궤적을 접수 평면마다 잘라 [`Prediction`]으로.
///
/// 플래너가 `Trajectory` 를 배우면 통째로 사라질 어댑터다. 그때까지는 **소비자 옆**에
/// 둔다 — 비전 쪽에 두면 비전이 접수 평면을 아는 셈이 된다.
///
/// `time_to_impact_secs` 는 궤적의 **마지막 관측 시각** 기준이다. 그래야 제어측이 재는
/// 리드타임과 뜻이 같다.
pub(super) fn predictions_at(trajectory: &Trajectory, planes: &[HitPlane]) -> Vec<Prediction> {
    let Some(now) = trajectory.measured.last().map(|state| state.t) else {
        return Vec::new();
    };
    return planes
        .iter()
        .filter_map(|plane| {
            let state = trajectory.predicted.at_plane(plane.y)?;
            return Some(Prediction {
                time_to_impact_secs: state.t.saturating_sub(now).as_secs_f64(),
                impact_position: state.position,
                incoming_velocity: state.velocity,
            });
        })
        .collect();
}
