//! 스윙 재생 상태.

use super::playback_trajectory::PlaybackTrajectory;

#[derive(Debug, Clone, PartialEq)]
pub(super) struct SwingPlayback {
    pub(super) trajectory: PlaybackTrajectory,
    pub(super) elapsed: f64,
    /// `advance_swing_torque_limited`용 관절 각속도 [rad/s].
    pub(super) joint_vel: Vec<f64>,
}
