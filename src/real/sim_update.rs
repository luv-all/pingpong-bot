//! 추정·제어 워커 → sim 자식 창 메시지.
//!
//! kiss3d(sim 창)와 OpenCV highgui(프리뷰) **둘 다 메인 스레드를 요구**해서 한 프로세스에
//! 같이 못 띄운다. `tools/verify_stereo`와 같은 방식으로 자기 자신을 `--sim-child`로 띄우고
//! 한 줄 JSON으로 먹인다.

use pingpong_bot::Point3;
use pingpong_bot::robot;
use pingpong_bot::robot::motion;
use serde::{Deserialize, Serialize};

/// 직렬화용 3D 점.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Xyz {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<Point3> for Xyz {
    fn from(point: Point3) -> Self {
        return Self {
            x: point.x,
            y: point.y,
            z: point.z,
        };
    }
}

impl From<Xyz> for Point3 {
    fn from(value: Xyz) -> Self {
        return Point3::new(value.x, value.y, value.z);
    }
}

/// 로봇 포즈 (레일 x + 관절각 [rad]).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseMsg {
    pub rail_x: f64,
    pub joints: Vec<f64>,
}

impl From<&robot::Pose> for PoseMsg {
    fn from(pose: &robot::Pose) -> Self {
        return Self {
            rail_x: pose.rail_x,
            joints: pose.joints.values.clone(),
        };
    }
}

impl From<&PoseMsg> for robot::Pose {
    fn from(msg: &PoseMsg) -> Self {
        return robot::Pose::new(msg.rail_x, robot::Joints::from_slice(&msg.joints));
    }
}

/// 커밋된 스윙 — sim 창이 그대로 재생한다.
///
/// `motion::Trajectory` 전체를 직렬화하는 대신 재생에 필요한 knot만 옮긴다
/// (도메인 타입에 serde를 얹지 않으려는 것 — 숫자 SSOT는 `defaults`다).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwingMsg {
    pub start: PoseMsg,
    pub impact: PoseMsg,
    pub follow_through: PoseMsg,
    pub start_velocity: Vec<f64>,
    pub impact_velocity: Vec<f64>,
    pub follow_through_velocity: Vec<f64>,
    /// 임팩트 knot 각가속도 — sim 재생이 원본 궤적과 같으려면 같이 옮겨야 한다.
    pub impact_acceleration: Vec<f64>,
    pub impact_time_secs: f64,
    pub duration_secs: f64,
    pub rail_start_velocity: f64,
    pub rail_end_velocity: f64,
    pub follow_through_rail_velocity: f64,
}

impl SwingMsg {
    pub fn from_trajectory(trajectory: &motion::Trajectory) -> Self {
        return Self {
            start: PoseMsg {
                rail_x: trajectory.rail.start,
                joints: trajectory.start.values.clone(),
            },
            impact: PoseMsg {
                rail_x: trajectory.rail.end,
                joints: trajectory.end.values.clone(),
            },
            follow_through: PoseMsg {
                rail_x: trajectory.follow_through_rail_x,
                joints: trajectory.follow_through.values.clone(),
            },
            start_velocity: trajectory.start_velocity.clone(),
            impact_velocity: trajectory.end_velocity.clone(),
            follow_through_velocity: trajectory.follow_through_velocity.clone(),
            impact_acceleration: trajectory.impact_acceleration.clone(),
            impact_time_secs: trajectory.impact_time_secs,
            duration_secs: trajectory.duration_secs,
            rail_start_velocity: trajectory.rail.start_velocity,
            rail_end_velocity: trajectory.rail.end_velocity,
            follow_through_rail_velocity: trajectory.follow_through_rail_velocity,
        };
    }

    pub fn to_trajectory(&self) -> motion::Trajectory {
        return motion::Trajectory::with_follow_through(
            robot::Joints::from_slice(&self.start.joints),
            robot::Joints::from_slice(&self.impact.joints),
            robot::Joints::from_slice(&self.follow_through.joints),
            self.start_velocity.clone(),
            self.impact_velocity.clone(),
            self.follow_through_velocity.clone(),
            self.impact_acceleration.clone(),
            self.impact_time_secs,
            self.duration_secs,
            motion::Rail {
                start: self.start.rail_x,
                end: self.impact.rail_x,
                start_velocity: self.rail_start_velocity,
                end_velocity: self.rail_end_velocity,
            },
            self.follow_through.rail_x,
            self.follow_through_rail_velocity,
        );
    }
}

/// sim 창 한 프레임 갱신. 필드는 전부 선택적 — 준 것만 바꾼다.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimUpdate {
    /// EKF가 추정한 공 위치 — 주황 공.
    #[serde(default)]
    pub ball: Option<Point3>,
    /// 예측 도달 위치 — 반투명 고스트 공.
    #[serde(default)]
    pub impact: Option<Point3>,
    /// 실기에서 읽은 로봇 포즈.
    #[serde(default)]
    pub pose: Option<PoseMsg>,
    /// 커밋된 스윙 — 받으면 sim이 재생한다.
    #[serde(default)]
    pub swing: Option<SwingMsg>,
}

impl SimUpdate {
    pub fn to_line(&self) -> String {
        return serde_json::to_string(self).unwrap_or_else(|_| "{}".to_owned());
    }

    pub fn parse_line(text: &str) -> Result<Self, serde_json::Error> {
        if text == "hide" || text == "null" {
            return Ok(Self::default());
        }
        return serde_json::from_str(text);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trajectory() -> motion::Trajectory {
        return motion::Trajectory::with_follow_through(
            robot::Joints::from_slice(&[0.0, 0.1, 0.2, 0.3]),
            robot::Joints::from_slice(&[0.4, 0.5, 0.6, 0.7]),
            robot::Joints::from_slice(&[0.44, 0.55, 0.66, 0.77]),
            vec![0.0; 4],
            vec![1.0, 1.1, 1.2, 1.3],
            vec![0.0; 4],
            vec![0.5, 0.6, 0.7, 0.8],
            0.30,
            0.36,
            motion::Rail {
                start: 0.2,
                end: 0.5,
                start_velocity: 0.0,
                end_velocity: 0.4,
            },
            0.52,
            0.0,
        );
    }

    #[test]
    fn swing_round_trips_through_json() {
        let original = trajectory();
        let line = SimUpdate {
            swing: Some(SwingMsg::from_trajectory(&original)),
            ..SimUpdate::default()
        }
        .to_line();

        let back = SimUpdate::parse_line(&line)
            .expect("parse")
            .swing
            .expect("swing")
            .to_trajectory();

        assert_eq!(back, original, "재생용 궤적이 원본과 같아야 한다");
    }

    #[test]
    fn ball_and_impact_round_trip() {
        let line = SimUpdate {
            ball: Some(Point3::new(0.7, 1.4, 0.95)),
            impact: Some(Point3::new(0.68, 0.2, 0.86)),
            ..SimUpdate::default()
        }
        .to_line();

        let back = SimUpdate::parse_line(&line).expect("parse");
        assert!((back.ball.expect("ball").x - 0.7).abs() < 1e-9);
        assert!((back.impact.expect("impact").y - 0.2).abs() < 1e-9);
    }

    #[test]
    fn hide_clears_everything() {
        let back = SimUpdate::parse_line("hide").expect("parse");
        assert!(back.ball.is_none() && back.impact.is_none() && back.swing.is_none());
    }
}
