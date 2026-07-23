//! 탁구대 옆 리니어 모터 (X축 슬라이드) · 철제 프로파일 프레임.

use crate::constants::table;
use crate::Point3;

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

/// 탁구대 한쪽 변에 설치된 X축 리니어 레일.
///
/// 팔 베이스는 레일 위에서 좌우로 이동하고, 팔은 주로 Y/Z 평면에서 접수한다.
/// y/z는 [`RailFrame`] 에서 오고, 값은 `crate::defaults::primitive_4dof` 빌더 체인에서 명시한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearRail {
    /// 레일 위 베이스 고정 y [m] ([`RailFrame::mount_y`])
    pub mount_y: f64,
    /// 레일 위 베이스 z [m] ([`RailFrame::mount_z`])
    pub mount_z: f64,
    /// 이동 가능한 최소 x [m]
    pub x_min: f64,
    /// 이동 가능한 최대 x [m]
    pub x_max: f64,
    /// 최대 이동 속도 [m/s]
    pub max_speed: f64,
}

impl LinearRail {
    /// x를 레일 범위 안으로 제한한다.
    pub fn clamp_x(self, x: f64) -> f64 {
        return x.clamp(self.x_min, self.x_max);
    }

    /// 레일 위 베이스 원점 (월드).
    pub fn mount_point(self, rail_x: f64) -> Point3 {
        return Point3::new(self.clamp_x(rail_x), self.mount_y, self.mount_z);
    }

    /// 대기 위치 x - 레일 원점(로봇 쪽 끝).
    pub fn home_x(self) -> f64 {
        return self.x_min;
    }

    /// 레일 중앙 x.
    pub fn default_x(self) -> f64 {
        return (self.x_min + self.x_max) * 0.5;
    }
}
