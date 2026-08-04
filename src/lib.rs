//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 — vision → robot::motion 파이프라인.
//!
//! 도메인 타입은 모듈 경로로 쓴다 (`camera::Id`, `detector::Observation`).
//! 루트 `pub use`로 짧은 이름을 펼치지 않는다.

pub mod camera;
pub mod constants;
pub mod defaults;
pub mod error;
pub mod hardware;
pub mod physics;
pub mod robot;
pub mod sim;
pub mod telemetry;
pub mod vision;

/// 월드 좌표 점 [m] — `nalgebra::Point3<f64>`.
pub type Point3 = nalgebra::Point3<f64>;

/// 월드 벡터 [m], [m/s] 등 — `nalgebra::Vector3<f64>`.
///
/// [`Point3`]와 같은 이유로 별칭이다. 이 저장소엔 rapier·kiss3d의 f32 벡터가 같이
/// 돌아다녀서 f64 세계를 이름으로 못 박아 두는 편이 안전하다. 같은 타입이라 기존
/// `nalgebra::Vector3<f64>` 표기와 공존한다 — 강제 마이그레이션은 없다.
pub type Vector3 = nalgebra::Vector3<f64>;
