//! 슈터 settings R/W.

use std::sync::{Arc, Mutex};

use crate::Point3;
use crate::shooter;
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

/// 슈터 settings R/W (+ 선택적 월드 position read).
#[derive(Clone)]
pub struct Handle {
    controls: Arc<Mutex<SimRuntimeControls>>,
    world: Option<Arc<Mutex<SimWorld>>>,
}

impl Handle {
    pub fn new(
        controls: Arc<Mutex<SimRuntimeControls>>,
        world: Option<Arc<Mutex<SimWorld>>>,
    ) -> Self {
        return Self { controls, world };
    }

    pub fn settings(&self) -> shooter::Settings {
        return self.controls.lock().expect("controls").shooter.clone();
    }

    pub fn set_settings(&self, settings: shooter::Settings) {
        self.controls.lock().expect("controls").shooter = settings;
    }

    pub fn request_shoot(&self) {
        self.controls.lock().expect("controls").request_shoot();
    }

    pub fn request_park(&self) {
        self.controls.lock().expect("controls").request_park();
    }

    /// 월드가 있으면 슈터 비주얼 위치 [m].
    pub fn position(&self) -> Option<Point3> {
        let world = self.world.as_ref()?;
        let world = world.lock().ok()?;
        let (pos, _rot) = world.shooter_pose();
        return Some(Point3::new(pos.x as f64, pos.y as f64, pos.z as f64));
    }

    pub fn controls(&self) -> Arc<Mutex<SimRuntimeControls>> {
        return Arc::clone(&self.controls);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shooter_handle_settings_rw() {
        let controls = Arc::new(Mutex::new(SimRuntimeControls::default()));
        let shooter = Handle::new(Arc::clone(&controls), None);
        let mut s = shooter.settings();
        s.speed_mps = 9.0;
        shooter.set_settings(s.clone());
        assert!((shooter.settings().speed_mps - 9.0).abs() < 1e-9);
    }
}
