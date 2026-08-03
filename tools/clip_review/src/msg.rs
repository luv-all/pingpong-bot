//! 부모(OpenCV) → sim 자식 stdin 프로토콜 — 한 줄 JSON.
//!
//! 점 하나가 아니라 **궤적 두 개**를 옮긴다. 그래야 sim 창에서 실제와 예측이
//! 얼마나 벌어지는지가 한눈에 보인다.

use pingpong_bot::Point3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Xyz {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<Point3> for Xyz {
    fn from(p: Point3) -> Self {
        return Self {
            x: p.x,
            y: p.y,
            z: p.z,
        };
    }
}

impl From<Xyz> for Point3 {
    fn from(v: Xyz) -> Self {
        return Point3::new(v.x, v.y, v.z);
    }
}

/// 한 프레임의 씬 상태.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SceneMsg {
    /// EKF 추정 위치 — 주황 공.
    #[serde(default)]
    pub ekf: Option<Xyz>,
    /// 이 프레임의 삼각측량 — 반투명 공.
    #[serde(default)]
    pub raw: Option<Xyz>,
    /// 실제 궤적, 현재 프레임까지 — 초록, 굵게.
    #[serde(default)]
    pub observed: Vec<Xyz>,
    /// 실제 궤적, 현재 프레임 이후 — 죽인 초록. pass 1이 클립을 통째로 훑어 이미 안다.
    #[serde(default)]
    pub observed_future: Vec<Xyz>,
    /// EKF 가 보정한 궤적 — 하늘색. 초록(생 삼각측량)과 나란히 봐야 필터가 무엇을 폈는지 안다.
    #[serde(default)]
    pub filtered: Vec<Xyz>,
    /// 커밋 순간에 얼린 예측 — 자홍, 굵게. 이게 "예측이 맞았나"의 대상이다.
    #[serde(default)]
    pub committed: Vec<Xyz>,
}

impl SceneMsg {
    pub fn to_line(&self) -> String {
        return serde_json::to_string(self).unwrap_or_else(|_| "hide".to_owned());
    }

    /// `hide`/`null`은 전부 지움.
    pub fn parse_line(text: &str) -> Result<Self, serde_json::Error> {
        if text == "hide" || text == "null" {
            return Ok(Self::default());
        }
        return serde_json::from_str(text);
    }

    pub fn points(list: &[Xyz]) -> Vec<Point3> {
        return list.iter().copied().map(Into::into).collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trails_round_trip() {
        let msg = SceneMsg {
            ekf: Some(Point3::new(0.7, 2.0, 0.95).into()),
            raw: Some(Point3::new(0.71, 1.99, 0.94).into()),
            observed: vec![
                Point3::new(0.0, 2.5, 1.0).into(),
                Point3::new(0.1, 2.0, 1.0).into(),
            ],
            observed_future: vec![Point3::new(0.2, 1.0, 0.9).into()],
            filtered: vec![Point3::new(0.11, 1.99, 1.0).into()],
            committed: vec![
                Point3::new(0.1, 2.0, 1.0).into(),
                Point3::new(0.2, 1.5, 0.9).into(),
            ],
        };
        let back = SceneMsg::parse_line(&msg.to_line()).expect("parse");
        assert_eq!(back.observed.len(), 2);
        assert_eq!(back.committed.len(), 2);
        assert_eq!(back.filtered.len(), 1);
        assert_eq!(back.observed_future.len(), 1);
        assert!((Point3::from(back.ekf.expect("ekf")).y - 2.0).abs() < 1e-9);
    }

    /// 검출 실패 프레임 — 필터는 예측으로 살아 있고 고스트만 사라진다.
    #[test]
    fn ekf_only_frame_hides_the_ghost() {
        let msg = SceneMsg {
            ekf: Some(Point3::new(0.7, 1.8, 0.9).into()),
            ..SceneMsg::default()
        };
        let back = SceneMsg::parse_line(&msg.to_line()).expect("parse");
        assert!(back.raw.is_none());
        assert!(back.ekf.is_some());
    }

    #[test]
    fn hide_clears_everything() {
        let back = SceneMsg::parse_line("hide").expect("parse");
        assert!(back.ekf.is_none() && back.raw.is_none());
        assert!(back.observed.is_empty() && back.committed.is_empty());
        assert!(back.filtered.is_empty());
        assert!(back.observed_future.is_empty());
    }
}
