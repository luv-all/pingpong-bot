//! Rapier3d 디지털 트윈.
//!
//! - [`physics`]: 탁구대·피더·로봇 라켓·공
//! - [`launch`]: sim 피더 발사 파라미터 SSOT
//! - [`session`]: 물리 스레드 + 공유 월드
//! - [`gui`]: kiss3d 3D + egui (feature `gui`)
//! - [`eval`]: 프로토콜 채점 모드 (sim 전용 벤치마크)

pub mod eval;
pub mod gui;
pub mod launch;
pub mod physics;
pub mod session;
