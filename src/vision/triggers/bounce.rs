use crate::vision::trigger::{Evidence, Trigger};

/// 첫 바운스가 지났다 — `vz` 부호가 한 번이라도 뒤집혔는가.
///
/// 바운스는 미지수를 하나 없애 주지만 늦다.
pub struct FirstBounce;

impl Trigger for FirstBounce {
    fn name(&self) -> &'static str {
        return "bounce";
    }

    fn ready(&self, evidence: &Evidence) -> bool {
        return evidence
            .windows(2)
            .any(|w| w[0].velocity.z < 0.0 && w[1].velocity.z > 0.0);
    }
}
