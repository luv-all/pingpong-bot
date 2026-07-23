//! 앱·테스트 진입점 조립 — **배선 SSOT**.
//!
//! 도메인 타입(`Arm`, `Scorer`, …)에는 프리셋을 두지 않는다.
//! competition 숫자·DSL 조립은 여기(와 하위 모듈)에만 둔다.

mod competition;

pub use competition::{
    competition_arm, competition_detector, competition_dynamixel, competition_intercept,
    competition_physics, competition_tunables, install_competition_tunables,
};
