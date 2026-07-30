//! pingpong-bot 공용 라이브러리.
//!
//! 경연용 단일 애플리케이션 — detector → estimator → planner 파이프라인.
//!
//! 도메인 타입은 모듈 경로로 쓴다 (`camera::Id`, `detector::Observation`).
//! 루트 `pub use`로 짧은 이름을 펼치지 않는다.

pub mod camera;
pub mod constants;
pub mod defaults;
pub mod detector;
pub mod error;
pub mod estimator;
pub mod eval;
pub mod hardware;
pub mod logging;
pub mod pipeline;
pub mod planner;
pub mod robot;
pub mod sim;
pub mod swing;
pub mod telemetry;

/// 월드 좌표 점 [m] — `nalgebra::Point3<f64>`.
pub type Point3 = nalgebra::Point3<f64>;
