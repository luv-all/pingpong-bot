//! 재생 중인 스윙 궤적 래퍼.

use crate::robot::Joints;
use crate::swing;

/// 재생 중인 스윙 궤적 - quintic(`plan_swing`)과 순수 토크 bang-bang
/// (`plan_bang_bang_swing`)을 같은 재생 루프(`advance_swing`)로 다루기 위한
/// 얇은 래퍼. GUI 토글로 어느 쪽을 커밋할지 고르지만, 재생 쪽 코드는
/// 궤적 "모양"을 몰라도 되게 한다.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum PlaybackTrajectory {
    Quintic(swing::Trajectory),
    BangBang(swing::bang_bang::Trajectory),
}

impl PlaybackTrajectory {
    pub(super) fn duration_secs(&self) -> f64 {
        return match self {
            Self::Quintic(trajectory) => trajectory.duration_secs,
            Self::BangBang(trajectory) => trajectory.duration_secs(),
        };
    }

    pub(super) fn sample_at(&self, t: f64) -> Joints {
        return match self {
            Self::Quintic(trajectory) => trajectory.sample_at(t),
            Self::BangBang(trajectory) => trajectory.sample_at(t),
        };
    }

    pub(super) fn sample_rail_at(&self, t: f64) -> f64 {
        return match self {
            Self::Quintic(trajectory) => trajectory.sample_rail_at(t),
            Self::BangBang(trajectory) => trajectory.sample_rail_at(t),
        };
    }

    pub(super) fn sample_velocity_at(&self, t: f64) -> Vec<f64> {
        return match self {
            Self::Quintic(trajectory) => trajectory.sample_velocity_at(t),
            Self::BangBang(trajectory) => trajectory.sample_velocity_at(t),
        };
    }

    pub(super) fn follow_through_rail_x(&self) -> f64 {
        return match self {
            Self::Quintic(trajectory) => trajectory.follow_through_rail_x,
            Self::BangBang(trajectory) => trajectory.follow_through_rail_x(),
        };
    }
}
