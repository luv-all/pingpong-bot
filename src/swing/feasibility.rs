//! 마운트 튜닝용 임팩트 실현 가능성.

use crate::estimator::Prediction;
use crate::robot::{Arm, RobotPose};

use super::impact_candidate::best_impact_candidate;

/// 특정 임팩트 예측을 이 팔이 얼마나 여유 있게 실행할 수 있는지 - 마운트
/// 위치(높이·테이블과의 거리) 튜닝, 벤치마크 등 외부 연구용 공개 API.
///
/// `plan_swing`이 실제로 쓰는 것과 같은 다중 IK 시드 탐색([`best_impact_candidate`])
/// 결과를 그대로 노출한다. IK/속도 역산 자체가 실패하면 `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feasibility {
    /// 관절 중 필요 속도/한계 비율이 가장 큰 값. 1.0 이하면 실기 관절속도
    /// 한계 안에서 실행 가능, 클수록 특이점 근접(비현실적 소요속도).
    pub peak_joint_speed_ratio: f64,
    /// 레일 필요속도/한계 비율. 레일이 없는 팔이면 0.0.
    pub peak_rail_speed_ratio: f64,
}

/// [`Feasibility`] 계산 - 마운트 위치 스윕(`tools/mount_search` 등) 전용
/// 공개 API. `plan_swing`/`plan_bang_bang_swing`과 같은 다중 IK 시드 탐색을
/// 재사용하되, quintic/토크 궤적 생성 없이 "이 임팩트를 낼 수 있는가"만
/// 본다 - 마운트 후보를 대량으로 스윕할 때 매번 전체 궤적을 만들 필요는
/// 없어서 훨씬 가볍다.
pub fn feasibility(arm: &Arm, prediction: &Prediction, start: &RobotPose) -> Option<Feasibility> {
    let candidate = best_impact_candidate(arm, prediction, start).ok()?;
    let peak_rail_speed_ratio = arm
        .rail
        .as_ref()
        .map_or(0.0, |rail| candidate.rail_velocity.abs() / rail.max_speed);
    return Some(Feasibility {
        peak_joint_speed_ratio: candidate.peak_joint_speed_ratio,
        peak_rail_speed_ratio,
    });
}
