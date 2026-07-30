//! 부모(OpenCV) → sim 자식 stdin 프로토콜 — 한 줄 JSON.
//!
//! 생 삼각측량과 EKF 출력을 같이 보내 sim 창에서 겹쳐 본다.
//! 빈 프레임은 `{"raw":null,"ekf":null}` 또는 그냥 `hide`.

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

/// 한 프레임의 공 상태.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct BallMsg {
    /// 생 삼각측량 — 반투명 고스트 공.
    #[serde(default)]
    pub raw: Option<Xyz>,
    /// EKF 출력 — 주황 공.
    #[serde(default)]
    pub ekf: Option<Xyz>,
}

impl BallMsg {
    pub fn hidden() -> Self {
        return Self::default();
    }

    pub fn to_line(self) -> String {
        return serde_json::to_string(&self).unwrap_or_else(|_| "hide".to_string());
    }

    /// 자식이 받은 한 줄을 파싱한다. `hide`/`null`은 둘 다 숨김.
    pub fn parse_line(text: &str) -> Result<Self, serde_json::Error> {
        if text == "hide" || text == "null" {
            return Ok(Self::hidden());
        }
        return serde_json::from_str(text);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_both_points() {
        let msg = BallMsg {
            raw: Some(Point3::new(0.7, 2.0, 0.95).into()),
            ekf: Some(Point3::new(0.71, 1.99, 0.94).into()),
        };
        let back = BallMsg::parse_line(&msg.to_line()).expect("parse");
        assert!((Point3::from(back.raw.expect("raw")).x - 0.7).abs() < 1e-9);
        assert!((Point3::from(back.ekf.expect("ekf")).y - 1.99).abs() < 1e-9);
    }

    #[test]
    fn ekf_only_frame_hides_ghost() {
        // 검출 실패 프레임 — 필터는 예측으로 살아 있다
        let msg = BallMsg {
            raw: None,
            ekf: Some(Point3::new(0.7, 1.8, 0.9).into()),
        };
        let back = BallMsg::parse_line(&msg.to_line()).expect("parse");
        assert!(back.raw.is_none());
        assert!(back.ekf.is_some());
    }

    #[test]
    fn hide_clears_both() {
        let back = BallMsg::parse_line("hide").expect("parse");
        assert!(back.raw.is_none() && back.ekf.is_none());
    }
}
