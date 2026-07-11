//! # pingpong-domain
//!
//! 순수 도메인 크레이트. OpenCV·모터 SDK 등 인프라 의존성 없이
//! 타입, 포트(trait), 물리/EKF 인터페이스만 정의한다.
//! 모든 다른 crate는 이 crate를 향해 의존한다 (헥사고날 아키텍처 코어).

pub mod ballistics;
pub mod collision;
pub mod constants;
pub mod ekf;
pub mod error;
pub mod estimator;
pub mod impact;
pub mod physics;
pub mod ports;
pub mod rail;
pub mod robot;
pub mod trajectory;
pub mod triangulation;
pub mod types;

pub use ballistics::predict_hit_plane;
pub use collision::{
    OrientedBox, clamp_above_table, robot_obbs, robot_obbs_all, table_penetration,
};
pub use constants::{
    BALL_RADIUS, COOPERATIVE_RETURN_SCALE, DEFAULT_DRAG, DEFAULT_RESTITUTION, G, G_Z, INTEGRATE_DT,
    JOINT_INERTIA, LINK_FOREARM, LINK_UPPER, LOFT_TIME_TO_NET, MAX_JOINT_ACCEL, MAX_JOINT_SPEED,
    MAX_JOINT_TORQUE, MAX_LEAD, MAX_RETURN_SPEED, MIN_LEAD, MIN_SWING_SECS, NET_CLEARANCE, Q_POS,
    Q_VEL, R_MEAS, RACKET_OPEN_PITCH, RAIL_MAX_SPEED, SWING_COMMIT_MAX_SECS, SWING_DURATION_SECS,
    TABLE_BOUNCE_RESTITUTION,
    TABLE_DEFAULT_HIT_PLANE_Y, TABLE_HALF_THICKNESS, TABLE_LENGTH_Y, TABLE_NET_HEIGHT,
    TABLE_SURFACE_Z, TABLE_WIDTH_X, arm, ball, control, estimator as estimator_constants,
    geometry, impact as impact_constants, physics as physics_constants, table,
};
pub use ekf::BallEkf;
pub use error::{DomainError, HwError, HwFailDetail, ObservationError, SwingPlanError};
pub use estimator::PassThroughEstimator;
pub use impact::{
    RacketImpactTarget, cooperative_return_velocity, loft_return_velocity,
    required_racket_velocity, verify_impact_model,
};
pub use physics::{
    accel, in_swing_commit_window, plan_contact_swing, plan_swing, verify_torque_limits,
};
pub use ports::{CameraSource, Clock, Detector, Estimator, Hardware, Telemetry};
pub use rail::LinearRail;
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, RacketPose, RobotState, SUPPORTED_FK_JOINTS,
};
pub use triangulation::{dlt_triangulate, sample_at, triangulate_projections, triangulate_synced};
pub use types::*;
