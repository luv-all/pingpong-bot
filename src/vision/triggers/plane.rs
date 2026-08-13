use crate::vision::trigger::{Evidence, Trigger};

pub struct PlaneCrossing {
    pub y: f64,
}

impl Trigger for PlaneCrossing {
    fn name(&self) -> &'static str {
        return "plane";
    }

    fn ready(&self, evidence: &Evidence) -> bool {
        let Some(last) = evidence.last() else {
            return false;
        };
        // velocity.y < 0 을 같이 보는 이유: 없으면 로봇 뒤로 지나간 공이나 되돌아가는 공에도 참이 된다.
        return last.position.y < self.y && last.velocity.y < 0.0;
    }
}
