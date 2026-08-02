use crate::vision::contract::State;
use crate::vision::trigger::Trigger;

/// 전부 만족할 때. 빈 목록은 **발동하지 않는다** — 아무 조건도 안 걸었는데 즉시 참이 되면
/// 안 된다.
pub struct All(pub Vec<Box<dyn Trigger>>);

impl Trigger for All {
    fn name(&self) -> &'static str {
        return "all";
    }

    fn ready(&self, measured: &[State]) -> bool {
        return !self.0.is_empty() && self.0.iter().all(|t| t.ready(measured));
    }
}

/// 하나라도 만족할 때.
///
/// 실전에서 쓸 건 이쪽이다 — 앞의 조건이 **빠르면 빠르게**, 뒤의 조건이 **늦어도 반드시**를
/// 보장한다. 하나만 쓰면 둘 중 하나를 포기해야 한다.
///
/// ```ignore
/// Any(vec![
///     Box::new(SigmaThreshold { position: .., velocity: .. }),
///     Box::new(PlaneCrossing { y: table::LENGTH_Y * 0.5 }),
/// ])
/// ```
pub struct Any(pub Vec<Box<dyn Trigger>>);

impl Trigger for Any {
    fn name(&self) -> &'static str {
        return "any";
    }

    fn ready(&self, measured: &[State]) -> bool {
        return self.0.iter().any(|t| t.ready(measured));
    }
}
