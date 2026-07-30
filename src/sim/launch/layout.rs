//! 슈터 설치 위치 (월드 좌표, Z-up).

use crate::constants::table;

/// 슈터 설치 위치 (월드 좌표, Z-up).
pub struct Layout;

impl Layout {
    /// 로봇은 y≈0, 슈터는 테이블 +y 끝(상대편).
    pub const MOUNT_X: f64 = table::WIDTH_X * 0.5;
    /// 마운트 기준 발사구 전방 돌출 [m] (탄도 SSOT)
    pub const BARREL_FORWARD_M: f64 = 0.22;
    /// 뷰어 직육면체 전체 크기 [m] (충돌 없음 — 표시 전용)
    pub const VISUAL_SIZE_X: f64 = 0.10;
    pub const VISUAL_SIZE_Y: f64 = 0.18;
    pub const VISUAL_SIZE_Z: f64 = 0.14;
    /// 슈터 마운트 y [m] — 본체는 테이블 밖, 발사구는 끝선(LENGTH_Y).
    pub const MOUNT_Y: f64 = table::LENGTH_Y + Self::BARREL_FORWARD_M;
    /// 슈터 마운트 기준 높이 [m] (테이블 면 → 중심). 탄도 SSOT.
    pub const BODY_HEIGHT: f64 = 0.45;
}
