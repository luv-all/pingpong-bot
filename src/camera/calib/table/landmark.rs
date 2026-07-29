//! 탁구대 규격 랜드마크 (solvePnP 외참용 SSOT).
//!
//! 원점 = 로봇 쪽 꼭짓점(바닥 아님, **테이블 면** `SURFACE_Z`).
//! +X 너비, +Y 길이, +Z up — [`crate::constants::table`].

use crate::Point3;

/// 고정 월드 랜드마크 하나.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landmark {
    /// 짧은 영문 id (로그·UI)
    pub id: &'static str,
    /// 클릭 안내 문구 (ASCII — Hershey)
    pub prompt: &'static str,
    /// 월드 좌표 [m]
    pub world: Point3,
}
