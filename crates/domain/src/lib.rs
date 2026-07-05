//! # pingpong-domain
//!
//! 순수 도메인 크레이트. OpenCV·모터 SDK 등 인프라 의존성 없이
//! 타입, 포트(trait), 물리/EKF 인터페이스만 정의한다.
//! 모든 다른 crate는 이 crate를 향해 의존한다 (헥사고날 아키텍처 코어).

pub mod constants;
pub mod error;
pub mod estimator;
pub mod physics;
pub mod ports;
pub mod robot;
pub mod triangulation;
pub mod types;

pub use constants::{
    ball, table, BALL_RADIUS, TABLE_HALF_THICKNESS, TABLE_LENGTH_Y, TABLE_NET_HEIGHT,
    TABLE_SURFACE_Z, TABLE_WIDTH_X, TABLE_DEFAULT_HIT_PLANE_Y,
};
pub use error::{DomainError, HwError, HwFailDetail, ObservationError, SwingPlanError};
pub use estimator::PassThroughEstimator;
pub use physics::{accel, plan_swing, G};
pub use ports::{
    CameraSource, Clock, Detector, Estimator, Hardware, Telemetry,
};
pub use robot::{
    Arm, ArmBuildError, ArmBuilder, JointLimit, RacketPose, RobotState, SUPPORTED_FK_JOINTS,
};
pub use triangulation::{sample_at, triangulate_synced};
pub use types::*;
