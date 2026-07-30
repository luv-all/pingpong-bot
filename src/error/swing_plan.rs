//! 스윙 궤적을 만들거나 실행할 수 없는 이유.

use thiserror::Error;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum SwingPlanError {
    /// 역기구학 해가 없음 - 목표 위치가 팔 도달 범위 밖
    #[error(
        "역기구학 해 없음 - 목표 위치 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m 가 도달 범위 밖"
    )]
    InverseKinematicsNoSolution {
        target_x: f64,
        target_y: f64,
        target_z: f64,
    },
    /// 위치에는 닿지만 그 지점에서 라켓 면을 요구 법선으로 세울 관절 조합이 없음.
    ///
    /// [`Self::InverseKinematicsNoSolution`]과 반드시 구분해야 한다 — 실측
    /// (fly_05·fly_07, 2026-07-31)에서 실패 표적 4개 중 3개는 **위치 IK는 성공**했다.
    /// 하나로 뭉뚱그리면 "팔이 짧다/레일이 짧다"는 엉뚱한 결론으로 샌다. 실제 대책은
    /// 마운트·리턴 법선 쪽이다.
    #[error(
        "라켓 면 방향 불가 - 위치 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m 에는 닿지만 \
         요구 법선 ({normal_x:.2}, {normal_y:.2}, {normal_z:.2}) 을 만들 관절 조합이 없음"
    )]
    RacketOrientationUnreachable {
        target_x: f64,
        target_y: f64,
        target_z: f64,
        normal_x: f64,
        normal_y: f64,
        normal_z: f64,
    },
    /// 임팩트 시각까지 남은 시간이 최소 스윙 소요 시간보다 짧음
    #[error("임팩트까지 {time_to_impact_secs:.3}s 남음 - 최소 스윙 {min_swing_secs:.3}s 필요")]
    InsufficientTime {
        time_to_impact_secs: f64,
        min_swing_secs: f64,
    },
    /// 임팩트 모델상 원하는 리턴 속도를 만들 수 없음
    #[error(
        "목표 리턴 속도 불가 - 입사 [{:.2}, {:.2}, {:.2}] -> 목표 [{:.2}, {:.2}, {:.2}] m/s",
        .incoming_velocity[0],
        .incoming_velocity[1],
        .incoming_velocity[2],
        .outgoing_velocity[0],
        .outgoing_velocity[1],
        .outgoing_velocity[2]
    )]
    ReturnVelocityUnreachable {
        incoming_velocity: [f64; 3],
        outgoing_velocity: [f64; 3],
    },
    /// 목표 라켓속도를 관절속도로 역산한 결과가 특이점 근처처럼 관절
    /// 속도 한계를 크게 벗어남.
    #[error(
        "특이점 근처 IK 해 - 관절 {joint_index} 필요속도 {required_speed:.2} rad/s \
         가 한계 {speed_limit:.2} rad/s를 크게 초과"
    )]
    NearSingularity {
        joint_index: usize,
        required_speed: f64,
        speed_limit: f64,
    },
    /// 임팩트 자세·목표속도 자체는 도달 가능한데, 거기까지 잇는 quintic
    /// 궤적(+ 팔로스루)이 중간에 관절 각도/속도 한계를 벗어남.
    #[error(
        "임팩트 자세는 도달 가능하나 quintic 궤적이 중간에 [{violated}] \
         한계를 벗어남 (레일 끝 x={rail_end_x:.3} m)"
    )]
    TrajectoryExceedsLimits {
        rail_end_x: f64,
        /// 실제로 위반한 한계 이름 (관절 속도/각가속도/각도 범위, 레일 속도/범위).
        violated: &'static str,
    },
    /// 궤적이 관절 각도/속도 한계는 지키지만 **토크** 한계를 넘음.
    #[error(
        "임팩트 자세는 도달 가능하나 궤적이 토크 한계를 초과 \
         (최악 관절 이용률 {utilization_pct:.0}%, 레일 끝 x={rail_end_x:.3} m)",
        utilization_pct = .utilization * 100.0
    )]
    TrajectoryExceedsTorque { rail_end_x: f64, utilization: f64 },
    /// 임팩트/궤적 자세가 테이블을 관통
    #[error("테이블 관통 {depth:.3}m - 목표 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m")]
    TablePenetration {
        target_x: f64,
        target_y: f64,
        target_z: f64,
        depth: f64,
    },
    /// 관절 속도·가속·토크 또는 레일 한계를 만족하는 궤적 없음
    #[error("관절/토크/레일 한계 - 목표 ({target_x:.3}, {target_y:.3}, {target_z:.3}) m")]
    JointOrTorqueLimit {
        target_x: f64,
        target_y: f64,
        target_z: f64,
    },
}

impl SwingPlanError {
    /// 재시도해도 안전 스윙이 안 나오는 하드 실패 (도달·리턴 불가).
    ///
    /// `InsufficientTime`만 false — 공이 더 가까워질 때까지 기다릴 수 있다.
    pub fn is_hard_unreachable(&self) -> bool {
        return !matches!(self, Self::InsufficientTime { .. });
    }

    /// 디버그 마커용 목표 좌표 (있으면).
    pub fn target_xyz(&self) -> Option<[f64; 3]> {
        return match self {
            Self::InverseKinematicsNoSolution {
                target_x,
                target_y,
                target_z,
            }
            | Self::RacketOrientationUnreachable {
                target_x,
                target_y,
                target_z,
                ..
            }
            | Self::TablePenetration {
                target_x,
                target_y,
                target_z,
                ..
            }
            | Self::JointOrTorqueLimit {
                target_x,
                target_y,
                target_z,
            } => Some([*target_x, *target_y, *target_z]),
            Self::InsufficientTime { .. }
            | Self::ReturnVelocityUnreachable { .. }
            | Self::NearSingularity { .. }
            | Self::TrajectoryExceedsLimits { .. }
            | Self::TrajectoryExceedsTorque { .. } => None,
        };
    }
}
