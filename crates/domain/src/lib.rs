//! # pingpong-domain
//!
//! 순수 도메인 크레이트. OpenCV·모터 SDK 등 인프라 의존성 없이
//! 타입, 포트(trait), 물리/EKF 인터페이스만 정의한다.
//! 모든 다른 crate는 이 crate를 향해 의존한다 (헥사고날 아키텍처 코어).
//!
//! 상수는 [`constants`] 경로를 쓴다 (`constants::table::…`). 루트에 펼치지 않는다 —
//! `estimator` / `impact` 등 역할 모듈과 이름이 겹친다.

pub mod camera;
pub mod constants;
pub mod detector;
pub mod error;
pub mod estimator;
pub mod planner;
pub mod ports;
pub mod robot;
pub mod triangulator;
pub mod types;

/// 레거시 경로: `pingpong_domain::ballistics::*`
pub mod ballistics {
    pub use crate::estimator::ballistics::*;
}

pub use error::{DomainError, HwError, HwFailDetail, ObservationError, SwingPlanError};
pub use estimator::{BallEkf, PassThroughEstimator, predict_hit_plane};
pub use planner::{
    OrientedBox, RacketImpactTarget, accel, clamp_above_table, cooperative_return_velocity,
    ball_past_midcourt_for_commit, in_swing_commit_window, loft_return_velocity, plan_contact_swing,
    plan_swing,
    required_racket_velocity, robot_obbs, robot_obbs_all, table_penetration, verify_impact_model,
    verify_torque_limits,
};
pub use ports::{CameraSource, Clock, Detector, Estimator, Hardware, Telemetry};
pub use robot::rail::LinearRail;
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, RacketPose, RobotState, SUPPORTED_FK_JOINTS,
};
pub use triangulator::{dlt_triangulate, sample_at, triangulate_projections, triangulate_synced};
pub use types::*;
