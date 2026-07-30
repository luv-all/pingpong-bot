//! 평가 존.

/// 평가 존 — 로봇이 테이블(+y)을 바라볼 때 기준.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Zone {
    /// 로봇 기준 오른쪽 (+x)
    Right,
    Center,
    Left,
}
impl Zone {
    pub const ALL: [Self; 3] = [Self::Right, Self::Center, Self::Left];

    /// 블록 모드 발사 순서: 왼 → 중 → 오.
    pub const BLOCK_ORDER: [Self; 3] = [Self::Left, Self::Center, Self::Right];

    pub fn label(self) -> &'static str {
        return match self {
            Self::Right => "Right",
            Self::Center => "Center",
            Self::Left => "Left",
        };
    }

    pub(crate) fn zone_index(self) -> usize {
        return match self {
            Self::Right => 0,
            Self::Center => 1,
            Self::Left => 2,
        };
    }

    /// 슈터 `lateral_offset_m` [m] — 존 표시·레거시용. 발사 yaw는 [`LaunchParams`].
    pub fn lateral_m(self) -> f64 {
        return match self {
            Self::Right => 0.35,
            Self::Center => 0.0,
            Self::Left => -0.35,
        };
    }

    /// 좌·우 대칭 yaw [deg]. Right=+, Left=−, Center=0.
    pub fn yaw_deg(self, side_yaw_deg: f64) -> f64 {
        return match self {
            Self::Right => side_yaw_deg.abs(),
            Self::Left => -side_yaw_deg.abs(),
            Self::Center => 0.0,
        };
    }
}
