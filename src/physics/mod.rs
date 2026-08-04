//! 공 물리 — 탄도·공기력·바운스, 그리고 궤적에서 그 상수를 역산하는 계측.
//!
//! 비전과 sim 이 **같은 커널**을 쓴다. 예측이 sim 과 다른 물리를 풀면 sim 에서 되는 게
//! 실기에서 안 된다 — 실제로 그런 적이 있다 (테이블을 무한 평면으로 보던 바운스).
//!
//! 추정은 여기 없다. 그건 [`crate::vision`] 이다.

mod ballistics;
pub(crate) mod bounce;
mod kinematics;
mod measure;

pub use ballistics::semi_implicit_euler;
pub use kinematics::Kinematics;
pub use measure::{BounceEvent, PhysicsIdentify, RollEvent, TrajAnalysis, TrajPoint};
