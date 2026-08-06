//! 임팩트 IK·목표 속도 역산 결과.

use nalgebra::Vector3;

use crate::error::DomainError;
use crate::robot::motion::Prediction;
use crate::robot::{self, Arm};

use super::impact_candidate::{ImpactCandidate, best_impact_candidate};

/// IK 목표 관절속도가 한계의 이 배수를 넘으면 특이점 근처로 본다.
pub(crate) const NEAR_SINGULARITY_SPEED_RATIO: f64 = 2.5;

/// 임팩트 IK·목표 속도 역산 결과. `plan_swing`(quintic)과 `plan_bang_bang_swing`
/// (순수 토크 적분, `motion::bang_bang`)이 같은 임팩트 설정을 공유한다 —
/// 갈라지는 지점은 이 목표를 어떤 궤적 "모양"에 넣느냐뿐이다.
pub(crate) struct ImpactTarget {
    pub(crate) pose: robot::Pose,
    pub(crate) joint_velocities: Vec<f64>,
    pub(crate) rail_velocity: f64,
    pub(crate) racket_velocity: Vector3<f64>,
}

pub(crate) fn solve_impact_target(
    arm: &Arm,
    prediction: &Prediction,
    start: &robot::Pose,
) -> Result<ImpactTarget, DomainError> {
    let candidate =
        best_impact_candidate(arm, prediction, start).map_err(DomainError::InfeasibleSwing)?;
    return Ok(impact_target_from_candidate(arm, candidate));
}

/// 이미 풀어 둔 IK 후보를 임팩트 목표로 바꾼다 — 근특이점 사전 축소만 적용.
///
/// [`solve_impact_target`]에서 갈라낸 이유: `plan_best_swing`(WP2b 복합 랭킹)이
/// 후보를 채점하려고 [`best_impact_candidate`]를 이미 한 번 부르는데, 채택된
/// 후보에 대해 그걸 또 풀면 IK를 두 번 도는 셈이 된다. 채점 결과를 그대로
/// 넘겨 재사용한다.
pub(crate) fn impact_target_from_candidate(arm: &Arm, candidate: ImpactCandidate) -> ImpactTarget {
    if candidate.peak_joint_speed_ratio > NEAR_SINGULARITY_SPEED_RATIO {
        let (joint_index, required_speed) = candidate
            .joint_velocities
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v.abs()))
            .fold(
                (0, 0.0_f64),
                |acc, cur| if cur.1 > acc.1 { cur } else { acc },
            );
        // 예전엔 여기서 NearSingularity로 하드 거절했다. 실기 관절속도(~5.18)
        // + 현재 마운트/슈터에서는 거의 모든 샷이 걸려 **스윙이 한 번도
        // commit되지 않았다**(시뮬 로그: streak→tti 포기). 목표 관절속도만
        // 한계로 스케일해 약한 스윙이라도 나가게 한다 — fit_end_velocity가
        // quintic peak도 추가로 깎는다.
        let scale = 1.0 / candidate.peak_joint_speed_ratio;
        tracing::warn!(
            joint_index,
            required_speed,
            speed_limit = arm.max_joint_speed,
            scale,
            "impact 관절속도가 한계 초과 — 끝속도를 {scale:.2}×로 스케일 (약한 스윙)"
        );
        let joint_velocities: Vec<f64> = candidate
            .joint_velocities
            .iter()
            .map(|v| v * scale)
            .collect();
        return ImpactTarget {
            pose: candidate.pose,
            joint_velocities,
            rail_velocity: candidate.rail_velocity * scale,
            racket_velocity: candidate.racket_velocity * scale,
        };
    }

    return ImpactTarget {
        pose: candidate.pose,
        joint_velocities: candidate.joint_velocities,
        rail_velocity: candidate.rail_velocity,
        racket_velocity: candidate.racket_velocity,
    };
}
