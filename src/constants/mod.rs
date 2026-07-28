//! 물리/규격·하드웨어 datasheet 상수. sim, real, 제어가 같이 쓴다.
//!
//! - **여기**: ITTF·CAD·G·캠 datasheet·DXL stall/RPM — 부품/규격이 바뀌지 않는 한 고정
//! - **[`crate::defaults`]**: 휴리스틱·측정값·벤치 배선·스트림 요청 기본

pub mod ball;
pub mod camera;
pub mod dynamixel;
pub mod geometry;
pub mod physics;
pub mod table;
pub mod viewer;

pub use ball::RADIUS as BALL_RADIUS;
pub use camera::TABLE_LANDMARK_COUNT;
pub use physics::{G, G_Z};
