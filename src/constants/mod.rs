//! 물리/규격 상수. sim, real, 제어가 같이 쓴다 (ITTF 등).
//!
//! 튜닝/측정 숫자도 여기만 둔다. 알고리즘 모듈에 const 를 흩뿌리지 말 것.

pub mod arm;
pub mod ball;
pub mod control;
pub mod estimator;
pub mod geometry;
pub mod impact;
pub mod physics;
pub mod table;

pub use arm::{BASE_Y, MAX_JOINT_SPEED, RAIL_MAX_SPEED};
pub use ball::RADIUS as BALL_RADIUS;
pub use ball::{
    RESTITUTION as DEFAULT_RESTITUTION, TABLE_BOUNCE_FRICTION, TABLE_BOUNCE_RESTITUTION,
};
pub use control::{
    EKF_MEAS_JUMP_M, JOINT_INERTIA, MAX_JOINT_ACCEL, MAX_JOINT_TORQUE, MIN_SWING_SECS,
    RACKET_OPEN_PITCH, SWING_COMMIT_MAX_BALL_Y_FRAC, SWING_COMMIT_MAX_SECS, SWING_FOLLOW_THROUGH_SECS,
};
pub use estimator::{INTEGRATE_DT, MAX_LEAD, MIN_LEAD, Q_POS, Q_VEL, R_MEAS};
pub use impact::{
    MAX_RETURN_SPEED, NET_CLEARANCE, RACKET_EFFECTIVE_RESTITUTION, RALLY_TIME_TO_BOUNCE,
};
pub use physics::{DEFAULT_DRAG, G, G_Z};
