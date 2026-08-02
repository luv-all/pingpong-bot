//! 공 물리 — 탄도 · 반발 · 접수 평면 · 물리 상수 역산.
//!
//! 추정 자체는 [`crate::vision`]으로 옮겼다. 여기 남은 것은 sim 과 real 이 공유하는
//! 운동학 커널과 그 위의 값 타입들이다.

mod ballistics;
mod bounce;
mod decision;
mod hit_plane;
mod impact;
mod kinematics;
mod measure;
mod prediction;

pub use ballistics::semi_implicit_euler;
pub use decision::{Decision, WaitReason, decide};
pub use hit_plane::HitPlane;
pub use impact::Impact;
pub use kinematics::Kinematics;
pub use measure::{BounceEvent, PhysicsIdentify, RollEvent, TrajAnalysis, TrajPoint};
pub use prediction::Prediction;
