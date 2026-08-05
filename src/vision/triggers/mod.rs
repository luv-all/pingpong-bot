//! [`Trigger`](super::Trigger) 구현들.

mod bounce;
mod combine;
mod plane;
mod sigma;

pub use bounce::FirstBounce;
pub use combine::{All, Any};
pub use plane::PlaneCrossing;
pub use sigma::SigmaThreshold;

use crate::Vector3;
use crate::constants::table;
use crate::defaults::EstimatorParams;

/// 본선 트리거 — 필터가 좁혀졌거나, 늦어도 네트를 넘으면.
///
/// [`Any`]인 이유는 둘 중 하나만 쓰면 하나를 포기해야 해서다. σ만 보면 검출이 나쁜 샷에서
/// 영영 안 걸리고, 평면만 보면 이미 확신이 선 샷도 네트까지 기다린다.
///
/// 실기와 클립 도구가 **같은 걸** 써야 한다. 도구가 더 늦게 거는 트리거를 쓰면 도구가 재는
/// 리드타임이 실기보다 짧아져, 실기에서 쓸 수 있는 구간을 도구가 못 본다.
pub fn primary() -> Box<dyn super::Trigger> {
    let params = EstimatorParams::default();
    let sigma = params.max_impact_sigma;
    return Box::new(Any(vec![
        Box::new(SigmaThreshold {
            position: Vector3::repeat(sigma),
            // 속도 σ는 리드타임을 곱해 도달점 오차가 되므로 같은 예산을 최대 리드로 나눈다.
            velocity: Vector3::repeat(sigma / params.max_lead),
        }),
        Box::new(PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    ]));
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
