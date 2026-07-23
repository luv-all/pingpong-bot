//! 앱 기본 배선 SSOT.
//!
//! 도메인 타입에는 프리셋을 두지 않는다. 숫자·조립은 여기만.
//! 규격·치수(ITTF, CAD, G)는 [`crate::constants`].
//!
//! 패턴: `Type` + 동명 팩토리 (`physics()`, `control()`, …).
//!
//! 이름 규칙:
//! - `*Params` — 숫자·휴리스틱 가방 (`PhysicsParams`, `ScorerParams`, …)
//! - `*Config` — 드라이버/버스 배선 (`DynamixelConfig`, `RailConfig`)
//!
//! | 모듈 | 팩토리 |
//! |------|--------|
//! | [`physics`] | [`physics`] |
//! | [`control`] | [`control`] |
//! | [`impact`] | [`impact`] |
//! | [`estimator`] | [`estimator`] |
//! | [`robot`] | [`robot`] / [`primitive_4dof`] / [`shared_robot`] / [`rail_frame`] / [`urdf_4dof`] |
//! | [`vision`] | [`detector`] / [`scorer`] / [`colormask`] / [`roi`] |
//! | [`hardware`] | [`dynamixel`] / [`rail`] |
//! | [`planner`] | [`intercept`] |
//!
//! 활성 로봇을 바꾸려면 [`robot`] 본문만 고친다.

mod control;
mod estimator;
mod hardware;
mod impact;
mod physics;
mod planner;
mod robot;
mod vision;

pub use control::{ControlParams, control};
pub use estimator::{EstimatorParams, estimator};
pub use hardware::{dynamixel, rail};
pub use impact::{ImpactParams, impact};
pub use physics::{PhysicsParams, physics};
pub use planner::intercept;
pub use robot::{primitive_4dof, rail_frame, robot, shared_robot, urdf_4dof, urdf_test};
pub use vision::{colormask, detector, roi, scorer};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_validate() {
        physics().validate().unwrap();
        control().validate().unwrap();
        impact().validate().unwrap();
        estimator().validate().unwrap();
        intercept().validate().unwrap();
        scorer().validate().unwrap();
        colormask().validate().unwrap();
        roi().validate().unwrap();
        dynamixel().validate().unwrap();
        rail().validate().unwrap();
        assert!((control().max_joint_torques[0] - 12.0).abs() < 1e-12);
        assert!((impact().max_return_speed - 6.0).abs() < 1e-12);
    }

    #[test]
    fn shared_robot_is_4dof() {
        assert_eq!(shared_robot().arm.joint_count(), 4);
    }
}
