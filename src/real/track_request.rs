//! 추정 워커 → 제어 워커 **선추종** 메시지.

use std::time::Instant;

use pingpong_bot::robot::motion::{HitPlane, Prediction};
use pingpong_bot::vision::Trajectory;

use super::commit_request::predictions_at;

/// "아직 칠 때는 아니지만, 궤적이 이러니 미리 옮겨 두라"는 요청.
///
/// [`super::CommitRequest`]와 채널을 나눈 이유: 커밋 채널은 bounded(1) drop-on-full이라
/// 선추종이 거기 끼면 정작 커밋 요청이 밀려 버려진다. 선추종은 놓쳐도 다음 프레임에 다시
/// 오지만 커밋은 그 공에 한 번뿐이다.
pub struct TrackRequest {
    pub trajectory: Trajectory,
    /// 예측을 만든 시각.
    pub at: Instant,
}

impl TrackRequest {
    pub fn age_secs(&self) -> f64 {
        return self.at.elapsed().as_secs_f64();
    }

    /// 접수 평면마다 잘라 플래너가 아는 모양으로. [`super::CommitRequest::predictions`]와 같다.
    pub fn predictions(&self, planes: &[HitPlane]) -> Vec<Prediction> {
        return predictions_at(&self.trajectory, planes);
    }
}
