//! sim 궤적 추정 — Rapier 진실 상태를 domain ballistics / EKF에 넣는다.
//!
//! 자동 스윙(`predict_impact`)은 진실 탄도를 쓰고, 파이프라인 Estimator는
//! 같은 상태를 EKF에 주입해 hit-plane 예측을 검증한다.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::ballistics::predict_hit_plane;
use crate::{BallEkf, Estimator, HitPlane, Point3, Prediction};
use nalgebra::Vector3;

use crate::sim::shooter::BallState;
use crate::sim::world::SimWorld;

/// Rapier 월드 스냅샷으로 접수 평면 교차를 예측한다 (물리 스텝·자동 스윙 공용).
pub fn predict_impact(world: &SimWorld, plane: HitPlane) -> Option<Prediction> {
    let snap = snapshot_from_world(world)?;
    return predict_hit_plane(snap.position, snap.velocity, plane, 0.0);
}

/// Rapier 월드에서 공 상태를 읽어 EKF에 주입한 뒤 `predict_to`한다.
pub struct SimBallEstimator {
    world: Arc<Mutex<SimWorld>>,
    ekf: BallEkf,
}

#[derive(Debug, Clone, Copy)]
struct BallSnapshot {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
}

impl SimBallEstimator {
    pub fn new(world: Arc<Mutex<SimWorld>>) -> Self {
        return Self {
            world,
            ekf: BallEkf::new(0.0),
        };
    }

    fn publish_debug_prediction(&self, prediction: Option<Prediction>) {
        if let Ok(mut world) = self.world.lock() {
            world.set_debug_prediction(prediction);
        }
    }
}

fn snapshot_from_world(world: &SimWorld) -> Option<BallSnapshot> {
    if world.ball_state != BallState::InFlight {
        return None;
    }
    let pos = world.ball_position();
    let vel = world.ball_velocity();
    return Some(BallSnapshot {
        position: Vector3::new(f64::from(pos.x), f64::from(pos.y), f64::from(pos.z)),
        velocity: Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z)),
    });
}

impl Estimator for SimBallEstimator {
    fn update(&mut self, _position: Point3, timestamp: Instant) {
        let snapshot = self
            .world
            .lock()
            .ok()
            .and_then(|world| snapshot_from_world(&world));
        let Some(snap) = snapshot else {
            self.publish_debug_prediction(None);
            return;
        };
        // 진실 위치·속도로 EKF를 리셋해 파이프라인 예측이 스윙과 맞게 유지
        self.ekf.set_state(snap.position, snap.velocity, timestamp);
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        let prediction = self.ekf.predict_to(plane).or_else(|| {
            let world = self.world.lock().ok()?;
            return predict_impact(&world, plane);
        })?;
        self.publish_debug_prediction(Some(prediction.clone()));
        return Some(prediction);
    }
}

#[cfg(test)]
mod tests {
    use crate::HitPlane;
    use crate::constants::table;

    use super::*;
    use crate::sim::BallShooterSettings;

    fn launch_snapshot() -> BallSnapshot {
        let settings = BallShooterSettings::default();
        let muzzle = settings.muzzle_position();
        let vel = settings.launch_velocity();
        return BallSnapshot {
            position: Vector3::new(
                f64::from(muzzle.x),
                f64::from(muzzle.y),
                f64::from(muzzle.z),
            ),
            velocity: Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z)),
        };
    }

    #[test]
    fn default_shot_impact_near_table_height_at_default_plane() {
        let snap = launch_snapshot();
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let pred =
            predict_hit_plane(snap.position, snap.velocity, plane, 0.0).expect("슈터 기본 샷 예측");
        assert!(
            (pred.impact_position.v.y - plane.y).abs() < 1e-5,
            "y={}",
            pred.impact_position.v.y
        );
        assert!(
            pred.impact_position.v.z > table::SURFACE_Z
                && pred.impact_position.v.z < table::SURFACE_Z + 0.30,
            "z={} — 짧은 팔 접수면(y={})에서 테이블~어깨 높이여야 함",
            pred.impact_position.v.z,
            plane.y
        );
        assert!(pred.impact_position.v.x > 0.2 && pred.impact_position.v.x < 1.3);
    }
}
