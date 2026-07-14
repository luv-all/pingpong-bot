//! # pingpong-domain
//!
//! 추정·제어·로봇 수학. 비전(캘리브·DLT·캡처)은 `pingpong_infra::vision`.
//!
//! 상수는 `constants::table::...` 경로를 쓴다. 루트에 펼치면
//! estimator / impact 같은 모듈 이름과 겹친다.

pub mod constants;
pub mod error;
pub mod estimator;
pub mod physics_config;
pub mod planner;
pub mod ports;
pub mod robot;
pub mod types;

/// 레거시 경로: `pingpong_domain::ballistics::*`
pub mod ballistics {
    pub use crate::estimator::ballistics::*;
}

pub use error::{DomainError, HwError, HwFailDetail, ObservationError, SwingPlanError};
pub use estimator::{
    BallEkf, drag_from_trajectory, friction_from_tangential_speeds, physics_coeffs_toml,
    predict_hit_plane, restitution_from_bounce_heights, restitution_from_normal_speeds,
};
pub use physics_config::{
    load_physics_from_config, merge_physics_into_config, PhysicsConfig, PhysicsParams,
};
pub use planner::{
    OrientedBox, accel, ball_past_midcourt_for_commit, clamp_above_table, in_swing_commit_window,
    loft_return_velocity, plan_swing, required_racket_velocity, robot_obbs, table_penetration,
    verify_impact_model,
};
pub use ports::{Clock, Estimator, Hardware, Telemetry};
pub use robot::rail::LinearRail;
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, RacketPose, RobotState, SUPPORTED_FK_JOINTS,
};
pub use types::*;
