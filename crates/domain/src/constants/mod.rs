//! 물리·규격 상수 — sim·real·제어가 공통으로 쓴다 (ITTF 등).
//!
//! 튜닝·측정 숫자도 여기 SSOT. 알고리즘 모듈에 `const`를 두지 않는다.
//! infra(Rapier·OpenCV)는 필요 시 `f32`로 캐스트만 한다.

pub mod arm;
pub mod ball;
pub mod control;
pub mod estimator;
pub mod geometry;
pub mod impact;
pub mod physics;
pub mod table;

pub use arm::{
    ARM_POSITION_LINKS, BASE_Y, ELBOW_DEFAULT, ELBOW_MAX, ELBOW_MIN, LINK_FOREARM, LINK_UPPER,
    LINK_WRIST_STUB, MAX_JOINT_SPEED, RAIL_MAX_SPEED, SHOULDER_DEFAULT, SHOULDER_MAX, SHOULDER_MIN,
    SUPPORTED_FK_JOINTS, WRIST_MAX, WRIST_MIN, YAW_DEFAULT, YAW_MAX, YAW_MIN,
};
pub use ball::RADIUS as BALL_RADIUS;
pub use ball::{RESTITUTION as DEFAULT_RESTITUTION, TABLE_BOUNCE_RESTITUTION};
pub use control::{
    JOINT_INERTIA, MAX_JOINT_ACCEL, MAX_JOINT_TORQUE, MIN_SWING_SECS, RACKET_OPEN_PITCH,
    SWING_COMMIT_MAX_SECS, SWING_DURATION_SECS,
};
pub use estimator::{INTEGRATE_DT, MAX_LEAD, MIN_LEAD, Q_POS, Q_VEL, R_MEAS};
pub use impact::{COOPERATIVE_RETURN_SCALE, LOFT_TIME_TO_NET, MAX_RETURN_SPEED, NET_CLEARANCE};
pub use physics::{DEFAULT_DRAG, G, G_Z};
pub use table::{
    DEFAULT_HIT_PLANE_Y as TABLE_DEFAULT_HIT_PLANE_Y, HALF_THICKNESS as TABLE_HALF_THICKNESS,
    LENGTH_Y as TABLE_LENGTH_Y, NET_HEIGHT as TABLE_NET_HEIGHT, SURFACE_Z as TABLE_SURFACE_Z,
    WIDTH_X as TABLE_WIDTH_X,
};
