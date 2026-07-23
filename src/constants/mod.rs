//! 물리/규격 상수. sim, real, 제어가 같이 쓴다 (ITTF·CAD·중력).
//!
//! 휴리스틱·측정값(스윙 창, e/μ/drag, EKF 잡음 등)은 `src/entry/` /
//! [`crate::tunables`]가 SSOT다. 머신 포트만 `config/local.toml`.

pub mod ball;
pub mod geometry;
pub mod physics;
pub mod table;

pub use ball::RADIUS as BALL_RADIUS;
pub use physics::{G, G_Z};
