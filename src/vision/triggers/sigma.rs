use crate::Vector3;
use crate::vision::contract::State;
use crate::vision::trigger::Trigger;

/// 필터가 충분히 좁혔다. **축별로 전부** 넘어야 한다.
///
/// 스칼라 하나로 재면 잘 관측되는 y축이 값을 지배해서, x축 속도가 아직 쓰레기인 채로
/// 통과한다.
pub struct SigmaThreshold {
    pub position: Vector3,
    pub velocity: Vector3,
}

impl Trigger for SigmaThreshold {
    fn name(&self) -> &'static str {
        return "sigma";
    }

    fn ready(&self, measured: &[State]) -> bool {
        let Some(last) = measured.last() else {
            return false;
        };
        return last.sigma_position < self.position && last.sigma_velocity < self.velocity;
    }
}
