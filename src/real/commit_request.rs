//! 추정 워커 → 제어 워커 메시지.

use std::time::Instant;

use pingpong_bot::robot::motion::{HitPlane, Prediction};
use pingpong_bot::vision::Trajectory;

/// "지금 이 궤적으로 스윙을 계획해 보라"는 요청. 비전 계약([`Trajectory`])을 그대로 나른다.
pub struct CommitRequest {
    pub trajectory: Trajectory,
    /// 예측을 만든 시각. 계획을 시작할 때까지 흐른 만큼 낡는다 ([`Self::age_secs`]).
    pub at: Instant,
}

impl CommitRequest {
    /// 요청이 만들어진 뒤 흐른 시간 [s].
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }

    /// 마지막으로 **본** 공 y [m] — 로그용. 보간하지 않는다 (관측을 찍는 값이다).
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

/// 예측 궤적을 접수 평면마다 잘라 [`Prediction`]으로. `time_to_impact_secs` 는 궤적의
/// 마지막 관측 시각 기준이다.
///
/// TODO(제어): 플래너가 [`Trajectory`] 를 직접 받으면 이 함수를 지운다. 계약이 더 많이 준다 —
/// 5 ms 간격 예측 궤적 전체, 축별 σ, `measured`(지금까지 실제 경로), `seq`(같은 공인가).
/// 지금은 평면 몇 개의 점으로 줄여 넘기느라 그게 다 버려진다.
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
