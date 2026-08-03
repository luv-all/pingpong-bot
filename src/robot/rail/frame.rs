//! 리니어모터를 받치는 철제 프로파일.

use crate::constants::geometry::RAIL_THICKNESS;

/// 리니어모터를 받치는 철제 프로파일 (실측 설치 위치).
///
/// 필드는 둘 다 **월드 좌표** — 원점은 탁구대 로봇쪽 꼭짓점(바닥). 예전에는
/// y만 "끝면 기준 뒤쪽 거리"(뒤로 갈수록 양수)여서 z와 부호 관례가 어긋났다.
/// 줄자로 재는 값이 바닥·끝면 기준 좌표 그 자체이고, GUI 슬라이더도 두 축을
/// 같은 좌표계로 보여줘야 읽기 쉽다.
///
/// 두께는 [`RAIL_THICKNESS`] 상수 — 프로파일이 이미 제작돼 있어 **못 바꾼다**.
/// 실물에서 조정 가능한 축만 필드로 둔다.
///
/// 숫자는 [`crate::defaults::rail_frame`] 에서만 둔다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailFrame {
    /// 레일 마운트 y [m] — 탁구대 끝면(y=0) 기준, 테이블 밖이면 음수.
    /// 실물에서는 레일을 밀면 되는 조정.
    pub mount_y: f64,
    /// 바닥(z=0) → 레일 프로파일 하단 [m]. 실물에서는 지지 높이 조정.
    pub rail_bottom_z: f64,
}

impl RailFrame {
    /// 현장에서 재는 치수로 레일 프레임을 만든다.
    pub fn from_table_distance(table_distance_m: f64, rail_bottom_z: f64) -> Self {
        return Self {
            mount_y: -table_distance_m,
            rail_bottom_z,
        };
    }

    /// 탁구대 로봇 쪽 끝선에서 레일까지의 거리 [m].
    /// 레일이 테이블 밖(-Y)에 있으면 양수다.
    pub fn table_distance_m(self) -> f64 {
        return -self.mount_y;
    }

    /// 줄자로 재는 양수 거리를 월드 Y 좌표로 변환한다.
    pub fn set_table_distance_m(&mut self, distance_m: f64) {
        self.mount_y = -distance_m;
    }

    /// base_link / 레일 마운트 y [m].
    pub fn mount_y(self) -> f64 {
        return self.mount_y;
    }

    /// base_link / 레일 마운트 z [m] — 프로파일 하단 + 두께 (베이스는 윗면에 얹힌다).
    pub fn mount_z(self) -> f64 {
        return self.rail_bottom_z + RAIL_THICKNESS;
    }

    /// x=0 에서의 마운트 위치 `[x, y, z]`.
    pub fn mount_xyz0(self) -> [f64; 3] {
        return [0.0, self.mount_y(), self.mount_z()];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mount_z_is_profile_bottom_plus_thickness() {
        let frame = RailFrame {
            mount_y: -0.10,
            rail_bottom_z: 0.88,
        };
        assert!((frame.mount_z() - (0.88 + RAIL_THICKNESS)).abs() < 1e-12);
        assert!((frame.mount_y() - -0.10).abs() < 1e-12);
    }

    /// 두께는 고정이므로 하단을 Δ만큼 올리면 마운트도 정확히 Δ만큼 올라간다.
    #[test]
    fn raising_the_profile_bottom_raises_the_mount_by_the_same_amount() {
        let low = RailFrame {
            mount_y: -0.10,
            rail_bottom_z: 0.88,
        };
        let high = RailFrame {
            rail_bottom_z: 0.95,
            ..low
        };
        assert!((high.mount_z() - low.mount_z() - 0.07).abs() < 1e-12);
    }

    #[test]
    fn table_distance_uses_positive_distance_outside_the_end_line() {
        let mut frame = RailFrame::from_table_distance(0.10, 0.88);
        assert!((frame.table_distance_m() - 0.10).abs() < 1e-12);
        frame.set_table_distance_m(0.24);
        assert!((frame.mount_y() + 0.24).abs() < 1e-12);
    }
}
