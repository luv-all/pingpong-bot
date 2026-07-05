//! 물리·규격 상수 — sim·real·제어가 공통으로 쓴다 (ITTF 등).
//!
//! infra(Rapier·OpenCV)는 필요 시 `f32`로 캐스트만 한다.

pub mod ball;
pub mod table;

pub use ball::RADIUS as BALL_RADIUS;
pub use table::{
    HALF_THICKNESS as TABLE_HALF_THICKNESS, LENGTH_Y as TABLE_LENGTH_Y,
    NET_HEIGHT as TABLE_NET_HEIGHT, SURFACE_Z as TABLE_SURFACE_Z, WIDTH_X as TABLE_WIDTH_X,
    DEFAULT_HIT_PLANE_Y as TABLE_DEFAULT_HIT_PLANE_Y,
};
