//! 추정·제어 워커 → sim 자식 창 메시지.
//!
//! kiss3d(sim 창)와 OpenCV highgui(프리뷰) **둘 다 메인 스레드를 요구**해서 한 프로세스에
//! 같이 못 띄운다. `tools/verify_stereo`와 같은 방식으로 자기 자신을 `--sim-child`로 띄우고
//! 한 줄 JSON으로 먹인다.

use pingpong_bot::Point3;
use pingpong_bot::robot;
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

/// sim 창 한 프레임 갱신. 필드는 전부 선택적 — 준 것만 바꾼다.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimUpdate {
    /// EKF가 추정한 공 위치 — 주황 공.
    #[serde(default)]
    pub ball: Option<Point3>,
    /// 현재 선택한 제어 목표 위치 — 하늘색 공.
    #[serde(default)]
    pub target: Option<Point3>,
    /// 실기에서 읽은 로봇 포즈.
    #[serde(default)]
    pub pose: Option<PoseMsg>,
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

    #[test]
    fn ball_and_target_round_trip() {
        let line = SimUpdate {
            ball: Some(Point3::new(0.7, 1.4, 0.95)),
            target: Some(Point3::new(0.68, 0.2, 0.86)),
            ..SimUpdate::default()
        }
        .to_line();

        let back = SimUpdate::parse_line(&line).expect("parse");
        assert!((back.ball.expect("ball").x - 0.7).abs() < 1e-9);
        assert!((back.target.expect("target").y - 0.2).abs() < 1e-9);
    }

    #[test]
    fn hide_clears_everything() {
        let back = SimUpdate::parse_line("hide").expect("parse");
        assert!(back.ball.is_none() && back.target.is_none() && back.pose.is_none());
    }
}
