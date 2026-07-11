//! sim 전용 궤적 추정 — Rapier 월드의 공 위치·속도로 접수 평면 교차를 예측한다.
//!
//! 2단계 EKF 본체 전까지 sim에서 IK·스윙 파이프라인을 검증하는 오라클 추정기.

use std::sync::{Arc, Mutex};

use nalgebra::Vector3;
use pingpong_domain::constants::{ball, table};
use pingpong_domain::physics::{G, MIN_SWING_SECS};
use pingpong_domain::{Estimator, HitPlane, Point3, Prediction, World};

use super::shooter::BallState;
use super::world::SimWorld;

/// Rapier와 동일한 ball·table restitution (world.rs 콜라이더 설정).
const BALL_RESTITUTION: f64 = 0.88;

/// Rapier 월드 스냅샷으로 접수 평면 교차를 예측한다 (물리 스텝·추정기 공용).
pub fn predict_impact(world: &SimWorld, plane: HitPlane) -> Option<Prediction> {
    let snap = snapshot_from_world(world)?;
    return integrated_impact(snap, plane);
}

/// Rapier 월드에서 공 상태를 읽어 `predict_to`를 계산한다.
pub struct SimBallEstimator {
    world: Arc<Mutex<SimWorld>>,
}

#[derive(Debug, Clone, Copy)]
struct BallSnapshot {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
}

impl SimBallEstimator {
    pub fn new(world: Arc<Mutex<SimWorld>>) -> Self {
        return Self { world };
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

/// Rapier와 같은 중력·테이블 바운스로 접수 평면 교차 시각을 적분한다.
fn integrated_impact(snap: BallSnapshot, plane: HitPlane) -> Option<Prediction> {
    let vy = snap.velocity.y;
    if vy >= -0.05 {
        return None;
    }

    const MIN_LEAD: f64 = 0.05;
    const MAX_LEAD: f64 = 1.2;
    const DT: f64 = 0.001;

    // 이미 평면을 지난 공 — 짧은 리드로 현재 궤적 위 intercept
    if snap.position.y <= plane.y + 1e-3 {
        let t = MIN_SWING_SECS.max(MIN_LEAD);
        if t > MAX_LEAD {
            return None;
        }
        return Some(short_lead_prediction(snap, t));
    }

    let g = G.z;
    let floor_z = table::SURFACE_Z + ball::RADIUS;
    let mut pos = snap.position;
    let mut vel = snap.velocity;
    let mut t = 0.0;

    while t < MAX_LEAD {
        let prev_y = pos.y;
        vel.z += g * DT;
        pos += vel * DT;
        t += DT;

        if pos.z <= floor_z && vel.z < 0.0 {
            pos.z = floor_z;
            vel.z = -vel.z * BALL_RESTITUTION;
        }

        if prev_y > plane.y && pos.y <= plane.y {
            let frac = (plane.y - prev_y) / (pos.y - prev_y);
            let t_cross = t - DT + DT * frac;
            if t_cross <= MIN_LEAD || t_cross > MAX_LEAD {
                return None;
            }
            let mut impact = pos;
            impact.y = plane.y;
            if impact.z < floor_z {
                impact.z = floor_z;
            }
            if impact.z > table::SURFACE_Z + 1.2 {
                return None;
            }
            return Some(Prediction {
                time_to_impact_secs: t_cross,
                impact_position: Point3::<World>::from_vector(impact),
                incoming_velocity: vel,
            });
        }
    }

    return None;
}

fn short_lead_prediction(snap: BallSnapshot, t: f64) -> Prediction {
    let g = G.z;
    let floor_z = table::SURFACE_Z + ball::RADIUS;
    let mut impact = snap.position + snap.velocity * t + Vector3::new(0.0, 0.0, 0.5 * g * t * t);
    if impact.z < floor_z {
        impact.z = floor_z;
    }
    return Prediction {
        time_to_impact_secs: t,
        impact_position: Point3::<World>::from_vector(impact),
        incoming_velocity: snap.velocity,
    };
}

impl Estimator for SimBallEstimator {
    fn update(&mut self, _observation: pingpong_domain::BallObservation) {
        // predict_to가 월드에서 직접 읽는다. 주차·시야 밖이면 디버그 마커 제거.
        let snapshot = self
            .world
            .lock()
            .ok()
            .and_then(|world| snapshot_from_world(&world));
        if snapshot.is_none() {
            self.publish_debug_prediction(None);
        }
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        let world = self.world.lock().ok()?;
        let prediction = predict_impact(&world, plane)?;
        drop(world);
        self.publish_debug_prediction(Some(prediction.clone()));
        return Some(prediction);
    }
}

#[cfg(test)]
mod tests {
    use pingpong_domain::HitPlane;
    use pingpong_domain::constants::table;

    use super::*;
    use crate::sim::shooter::BallShooterSettings;

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
        let pred = integrated_impact(snap, plane).expect("슈터 기본 샷 예측");
        assert!(
            (pred.impact_position.v.y - plane.y).abs() < 1e-6,
            "y={}",
            pred.impact_position.v.y
        );
        assert!(
            pred.impact_position.v.z > table::SURFACE_Z
                && pred.impact_position.v.z < table::SURFACE_Z + 0.25,
            "z={} — 짧은 팔 접수면(y={})에서 테이블~어깨 높이여야 함",
            pred.impact_position.v.z,
            plane.y
        );
        assert!(pred.impact_position.v.x > 0.2 && pred.impact_position.v.x < 1.3);
    }
}
