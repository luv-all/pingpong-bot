//! 탁구대 옆 리니어 모터 (X축 슬라이드).

use crate::types::{Point3, World};

/// 탁구대 한쪽 변에 설치된 X축 리니어 레일.
///
/// 팔 베이스는 레일 위에서 좌우로 이동하고, 팔은 주로 Y·Z 평면에서 접수한다.
/// 값은 `Arm::competition()` 빌더 체인에서 명시한다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearRail {
    /// 레일 위 베이스 고정 y [m] (탁구대 로봇 쪽 끝 근처)
    pub mount_y: f64,
    /// 레일 위 베이스 z [m] — 테이블 면과 같거나 약간 위
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
    pub fn mount_point(self, rail_x: f64) -> Point3<World> {
        return Point3::new(self.clamp_x(rail_x), self.mount_y, self.mount_z);
    }

    /// 대기 위치 x — 레일 원점(로봇 쪽 끝).
    pub fn home_x(self) -> f64 {
        return self.x_min;
    }

    /// 레일 중앙 x.
    pub fn default_x(self) -> f64 {
        return (self.x_min + self.x_max) * 0.5;
    }
}
