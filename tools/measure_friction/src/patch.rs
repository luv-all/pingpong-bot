//! PhysicsParams 붙이기용 측정 패치.

#[derive(Default)]
pub struct Patch {
    pub restitution: Option<f64>,
    pub friction: Option<f64>,
    pub drag: Option<f64>,
}

impl Patch {
    pub fn is_empty(&self) -> bool {
        return self.restitution.is_none() && self.friction.is_none() && self.drag.is_none();
    }
}
