//! 실기 수동 테스트 컨트롤 — reset / wait / zone 버튼이 이 타입으로 들어온다.
//!
//! 슈터(발사기)가 좌/센터/우로 공을 쏘는 무랠리 테스트 프로토콜에서, 운영자가
//! `--preview` 창의 키 입력으로 로봇의 준비 자세와 내부 상태를 직접 통제한다.

use pingpong_bot::robot::LinearRail;

/// 슈터가 겨누는 존 — 이 존의 레일 x가 다음 준비 자세 목표가 된다.
///
/// `LinearRail::x_min/x_max`는 실기에서 확인한 공통 안전 범위
/// (0.0100~1.3395 m)다.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestZone {
    Left,
    Center,
    Right,
}

impl TestZone {
    /// 이 존의 시작·타격 후 대기 레일 x [m].
    /// 안전 제어 범위 기준 16% / 50% / 83%를 사용한다.
    pub fn rail_x(self, rail: LinearRail) -> f64 {
        let fraction = match self {
            Self::Left => 0.16,
            Self::Center => 0.50,
            Self::Right => 0.83,
        };
        return rail.x_min + (rail.x_max - rail.x_min) * fraction;
    }

    /// 프리뷰 패널 표시용 라벨.
    pub fn label(self) -> &'static str {
        return match self {
            Self::Left => "LEFT",
            Self::Center => "CENTER",
            Self::Right => "RIGHT",
        };
    }

    /// 안전 제어 범위 기준 이 모드가 받는 x 구간 [m].
    /// 경계 공을 놓치지 않도록 0~45%, 20~60%, 55~100%로 서로 겹친다.
    pub fn bounds(self, rail: LinearRail) -> (f64, f64) {
        let span = rail.x_max - rail.x_min;
        let (min_fraction, max_fraction) = match self {
            Self::Left => (0.0, 0.45),
            Self::Center => (0.20, 0.60),
            Self::Right => (0.55, 1.0),
        };
        return (
            rail.x_min + span * min_fraction,
            rail.x_min + span * max_fraction,
        );
    }

    /// 해당 모드의 겹친 제어 구간에 들어오는지 판정한다.
    pub fn contains_x(self, rail: LinearRail, x: f64) -> bool {
        let (min, max) = self.bounds(rail);
        return x >= min && x <= max;
    }
}

/// `--preview` 창 키 입력이 `control_worker`로 보내는 수동 컨트롤.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestControl {
    /// 즉시 적용 — 하드웨어가 움직이는 중이어도 멈추고 준비 자세로 복귀한다.
    ResetPosition,
    /// 다음 idle 시점에 적용 — 존 변경 없이 latch·상태만 정리하고 현재
    /// home 레일 x로 복귀한다.
    Wait,
    /// 다음 idle 시점에 적용 — home 레일 x를 이 존으로 바꾸고 `Wait`과
    /// 동일하게 정리한다.
    SetZone(TestZone),
    /// 다음 idle 시점에 중앙 준비 자세로 복귀하고 전체 구간의 공을 받는다.
    DefaultMode,
}

impl TestControl {
    /// highgui 키코드 → 수동 컨트롤. 매핑에 없는 키는 `None`(무시).
    ///
    /// `1`/`2`/`3` = 0~45% / 20~60% / 55~100% 존, `4` = 전체 구간 기본 모드.
    /// `w` = wait(모드 유지), `r` = 즉시 리셋.
    /// `q`/ESC는 프리뷰 창 자체가 Quit으로 소비하므로 여기 없다.
    pub fn from_key(key: i32) -> Option<Self> {
        return match key {
            k if k == i32::from(b'1') => Some(Self::SetZone(TestZone::Left)),
            k if k == i32::from(b'2') => Some(Self::SetZone(TestZone::Center)),
            k if k == i32::from(b'3') => Some(Self::SetZone(TestZone::Right)),
            k if k == i32::from(b'4') => Some(Self::DefaultMode),
            k if k == i32::from(b'w') || k == i32::from(b'W') => Some(Self::Wait),
            k if k == i32::from(b'r') || k == i32::from(b'R') => Some(Self::ResetPosition),
            _ => None,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::robot::LinearRail;

    fn test_rail() -> LinearRail {
        return LinearRail {
            mount_y: 1.0,
            mount_z: 0.2,
            x_min: pingpong_bot::defaults::RAIL_X_MIN_M,
            x_max: pingpong_bot::defaults::RAIL_X_MAX_M,
            default_x: pingpong_bot::defaults::RAIL_READY_X_M,
            max_speed: 1.0,
        };
    }

    #[test]
    fn zone_home_positions_are_16_50_83_percent_of_safe_range() {
        let rail = test_rail();
        let span = rail.x_max - rail.x_min;
        assert!((TestZone::Left.rail_x(rail) - (rail.x_min + span * 0.16)).abs() < 1e-12);
        assert!((TestZone::Center.rail_x(rail) - (rail.x_min + span * 0.50)).abs() < 1e-12);
        assert!((TestZone::Right.rail_x(rail) - (rail.x_min + span * 0.83)).abs() < 1e-12);
    }

    #[test]
    fn zones_use_requested_overlapping_ranges() {
        let rail = test_rail();
        let at = |fraction: f64| rail.x_min + (rail.x_max - rail.x_min) * fraction;

        assert!(TestZone::Left.contains_x(rail, at(0.0)));
        assert!(TestZone::Left.contains_x(rail, at(0.45)));
        assert!(!TestZone::Left.contains_x(rail, at(0.451)));
        assert!(TestZone::Center.contains_x(rail, at(0.20)));
        assert!(TestZone::Center.contains_x(rail, at(0.60)));
        assert!(!TestZone::Center.contains_x(rail, at(0.601)));
        assert!(TestZone::Right.contains_x(rail, at(0.55)));
        assert!(TestZone::Right.contains_x(rail, at(1.0)));
    }

    #[test]
    fn zone_label_is_upper_case_name() {
        assert_eq!(TestZone::Left.label(), "LEFT");
        assert_eq!(TestZone::Center.label(), "CENTER");
        assert_eq!(TestZone::Right.label(), "RIGHT");
    }

    #[test]
    fn digit_keys_map_to_set_zone() {
        assert_eq!(
            TestControl::from_key(i32::from(b'1')),
            Some(TestControl::SetZone(TestZone::Left))
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'2')),
            Some(TestControl::SetZone(TestZone::Center))
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'3')),
            Some(TestControl::SetZone(TestZone::Right))
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'4')),
            Some(TestControl::DefaultMode)
        );
    }

    #[test]
    fn w_and_r_map_to_wait_and_reset_case_insensitively() {
        assert_eq!(
            TestControl::from_key(i32::from(b'w')),
            Some(TestControl::Wait)
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'W')),
            Some(TestControl::Wait)
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'r')),
            Some(TestControl::ResetPosition)
        );
        assert_eq!(
            TestControl::from_key(i32::from(b'R')),
            Some(TestControl::ResetPosition)
        );
    }

    #[test]
    fn unmapped_keys_are_ignored() {
        assert_eq!(TestControl::from_key(i32::from(b'x')), None);
        assert_eq!(TestControl::from_key(27), None);
        assert_eq!(TestControl::from_key(-1), None);
    }
}
