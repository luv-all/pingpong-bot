//! 리니어모터를 받치는 철제 프로파일.

use crate::constants::table;

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·윗면 기준 설치 치수).
///
/// - 기준면: 탁구대 로봇쪽 끝면 `y = 0`, 윗면 `z = SURFACE_Z`
/// - `behind_table_end` / `above_table` 은 양수 설치 거리
/// - [`Self::mount_y`] / [`Self::mount_z`] 는 sim 월드 좌표
///
/// 숫자는 [`crate::defaults::rail_frame`] 에서만 둔다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailFrame {
    /// 탁구대 끝면 기준 뒤쪽 거리 [m] (−Y)
    pub behind_table_end: f64,
    /// 탁구대 윗면 기준 위쪽 거리 [m] (+Z)
    pub above_table: f64,
}

impl RailFrame {
    /// base_link / 레일 마운트 y [m].
    pub fn mount_y(self) -> f64 {
        return -self.behind_table_end;
    }

    /// base_link / 레일 마운트 z [m].
    pub fn mount_z(self) -> f64 {
        return table::SURFACE_Z + self.above_table;
    }

    /// x=0 에서의 마운트 위치 `[x, y, z]`.
    pub fn mount_xyz0(self) -> [f64; 3] {
        return [0.0, self.mount_y(), self.mount_z()];
    }
}
