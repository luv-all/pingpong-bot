//! 물리/규격 상수. sim, real, 제어가 같이 쓴다 (ITTF·CAD·중력).
//!
//! 휴리스틱·측정값(스윙 창, e/μ/drag, EKF 잡음 등)은 [`crate::defaults`]가 SSOT다.

pub mod ball;
pub mod geometry;
pub mod physics;
pub mod table;
pub mod viewer;

pub use ball::RADIUS as BALL_RADIUS;
pub use physics::{G, G_Z};
