//! sim 궤적 추정 — Rapier 진실 상태를 domain ballistics / EKF에 넣는다.
//!
//! 자동 스윙(`predict_impact`)은 진실 탄도(+스핀 Magnus)를 쓰고, 파이프라인
//! Estimator는 같은 상태를 EKF에 주입해 hit-plane 예측을 검증한다.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::estimator::ballistics::predict_hit_plane;
use crate::{BallEkf, Estimator, HitPlane, Point3, Prediction};
use nalgebra::Vector3;

use crate::sim::shooter::BallState;
use crate::sim::world::SimWorld;

/// Rapier 월드 스냅샷으로 접수 평면 교차를 예측한다 (물리 스텝·자동 스윙 공용).
pub fn predict_impact(world: &SimWorld, plane: HitPlane) -> Option<Prediction> {
    let snap = snapshot_from_world(world)?;
    return predict_hit_plane(
        snap.position,
        snap.velocity,
        snap.omega,
        plane,
        &world.physics,
    );
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
    omega: Vector3<f64>,
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
    let omega = world.ball_angular_velocity();
    return Some(BallSnapshot {
        position: Vector3::new(f64::from(pos.x), f64::from(pos.y), f64::from(pos.z)),
        velocity: Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z)),
        omega: Vector3::new(f64::from(omega.x), f64::from(omega.y), f64::from(omega.z)),
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
        // sim 진실 경로: ω 포함 ballistics를 우선 (EKF는 아직 스핀 상태 없음).
        let prediction = self
            .world
            .lock()
            .ok()
            .and_then(|world| predict_impact(&world, plane))
            .or_else(|| self.ekf.predict_to(plane))?;
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
        let omega = settings.launch_angular_velocity();
        return BallSnapshot {
            position: Vector3::new(
                f64::from(muzzle.x),
                f64::from(muzzle.y),
                f64::from(muzzle.z),
            ),
            velocity: Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z)),
            omega: Vector3::new(f64::from(omega.x), f64::from(omega.y), f64::from(omega.z)),
        };
    }

    #[test]
    fn default_shot_impact_near_table_height_at_default_plane() {
        let snap = launch_snapshot();
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let pred = predict_hit_plane(
            snap.position,
            snap.velocity,
            snap.omega,
            plane,
            &crate::defaults::physics(),
        )
        .expect("슈터 기본 샷 예측");
        assert!(
            (pred.impact_position.coords.y - plane.y).abs() < 1e-5,
            "y={}",
            pred.impact_position.coords.y
        );
        assert!(
            pred.impact_position.coords.z > table::SURFACE_Z
                && pred.impact_position.coords.z < table::SURFACE_Z + 0.45,
            "z={} — 짧은 팔 접수면(y={})에서 테이블~어깨 높이여야 함",
            pred.impact_position.coords.z,
            plane.y
        );
        assert!(pred.impact_position.coords.x > 0.2 && pred.impact_position.coords.x < 1.3);
    }

    /// Rapier 진실 궤적이 hit-plane을 지날 때 Z가 직전 예측과 ≤6cm.
    ///
    /// 발사 직후 한 번만 예측하면 테이블 바운스 후 Rapier가 만든 ω·Magnus를
    /// 반영하지 못해 어긋난다. 매 스텝 현재 (p,v,ω)로 재예측해 비교한다.
    #[test]
    fn rapier_hit_plane_z_matches_predict_within_5cm() {
        let mut world = SimWorld::new(crate::defaults::primitive_4dof().expect("4dof"));
        world.set_use_ground_truth(false);
        world.shoot_ball(&BallShooterSettings::default());

        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let plane_y = plane.y as f32;

        let mut prev = world.ball_position();
        let mut last_pred_z = None;
        for _ in 0..5_000 {
            if let Some(pred) = predict_impact(&world, plane) {
                last_pred_z = Some(pred.impact_position.coords.z);
            }
            world.step(1.0 / 1000.0, None);
            let pos = world.ball_position();
            if prev.y > plane_y && pos.y <= plane_y {
                let pred_z = last_pred_z.expect("hit-plane 직전 예측이 있어야 함");
                let denom = pos.y - prev.y;
                let frac = if denom.abs() < 1e-8 {
                    0.0
                } else {
                    (plane_y - prev.y) / denom
                };
                let rapier_z = f64::from(prev.z + (pos.z - prev.z) * frac);
                assert!(
                    (rapier_z - pred_z).abs() <= 0.06,
                    "Rapier z={rapier_z:.4} predict z={pred_z:.4} |Δ|={:.4}m (>6cm)",
                    (rapier_z - pred_z).abs()
                );
                return;
            }
            prev = pos;
        }
        panic!("공이 hit-plane y를 지나가지 않음");
    }

    #[test]
    fn low_pitch_shot_rejected_by_net_gate() {
        let mut settings = BallShooterSettings::default();
        // 네트 아래로 스치는 낮은 pitch — 접수 예측이 나오면 안 됨.
        settings.pitch_deg = -25.0;
        settings.height_offset_m = 0.0;
        settings.speed_mps = 4.0;
        let muzzle = settings.muzzle_position();
        let vel = settings.launch_velocity();
        let position = Vector3::new(
            f64::from(muzzle.x),
            f64::from(muzzle.y),
            f64::from(muzzle.z),
        );
        let velocity = Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z));
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        assert!(
            predict_hit_plane(
                position,
                velocity,
                Vector3::zeros(),
                plane,
                &crate::defaults::physics(),
            )
            .is_none(),
            "네트 미달 낮은 pitch 샷은 예측 None이어야 함"
        );
    }
}
