//! 공 탄도·바운스 예측의 공개 진입점.

use nalgebra::Vector3;

use crate::Point3;
use crate::constants::{ball, table};
use crate::defaults::{EstimatorParams, PhysicsParams};
use crate::estimator::{BallTrajectory, HitPlane, Prediction, TrajectorySample};

use super::{ballistics, bounce};

pub struct Kinematics;

impl Kinematics {
    /// 비행 중 공기력만 (중력 제외) [m/s^2] — Rapier 외력·예측기 공유 SSOT.
    pub fn aero_accel(
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        drag_coefficient: f64,
        magnus_coefficient: f64,
    ) -> Vector3<f64> {
        return ballistics::aero_accel(velocity, omega, drag_coefficient, magnus_coefficient);
    }

    pub fn clears_net(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        physics: &PhysicsParams,
    ) -> bool {
        return ballistics::clears_net_gate(position, velocity, omega, physics);
    }

    pub fn predict_to(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        plane: HitPlane,
        physics: &PhysicsParams,
    ) -> Option<Prediction> {
        return ballistics::predict_hit_plane(position, velocity, omega, plane, physics);
    }

    /// 현재 상태 다음 `integrate_dt`부터 `max_lead`까지 미래 궤적을 샘플링한다.
    ///
    /// 타격 평면이나 대표 타격점은 계산하지 않는다.
    pub fn sample_trajectory(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        physics: &PhysicsParams,
    ) -> Vec<TrajectorySample> {
        let params = EstimatorParams::default();
        let mut samples =
            Vec::with_capacity((params.max_lead / params.integrate_dt).ceil() as usize);
        let (mut pos, mut vel, mut spin) = (position, velocity, omega);
        let mut time_secs = 0.0;
        while time_secs + params.integrate_dt <= params.max_lead + f64::EPSILON {
            (pos, vel, spin) =
                ballistics::semi_implicit_euler(pos, vel, spin, params.integrate_dt, physics);
            time_secs += params.integrate_dt;
            if !prediction_state_is_valid(pos, vel) {
                break;
            }
            samples.push(TrajectorySample::new(Point3::from(pos), vel, time_secs));
        }
        return samples;
    }

    /// 구 제어 경로용 임시 어댑터. 궤적 API와 타격점 계산을 분리한다.
    pub fn prediction_from_trajectory(
        trajectory: &BallTrajectory,
        plane: HitPlane,
    ) -> Option<Prediction> {
        let samples = trajectory
            .observed
            .last()
            .into_iter()
            .chain(trajectory.predicted.iter())
            .collect::<Vec<_>>();
        let current = *samples.first()?;
        let params = EstimatorParams::default();
        if current.velocity.y > -params.min_approach_speed_y || current.position.y <= plane.y + 1e-3
        {
            return None;
        }
        let rest_height = table::SURFACE_Z + ball::RADIUS;
        if current.position.z <= rest_height + params.min_strike_clearance
            && current.velocity.z.abs() < 0.5
        {
            return None;
        }

        let net_y = table::LENGTH_Y * 0.5;
        let net_clear_z = table::SURFACE_Z + table::NET_HEIGHT + ball::RADIUS;
        for pair in samples.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a.position.y > net_y && b.position.y <= net_y {
                let dy = b.position.y - a.position.y;
                let fraction = if dy.abs() < 1e-12 {
                    0.0
                } else {
                    (net_y - a.position.y) / dy
                };
                let z = a.position.z + (b.position.z - a.position.z) * fraction;
                const NET_GATE_SLACK_M: f64 = 0.012;
                if z + NET_GATE_SLACK_M < net_clear_z {
                    return None;
                }
            }
            if a.position.y > plane.y && b.position.y <= plane.y {
                let dy = b.position.y - a.position.y;
                let fraction = if dy.abs() < 1e-12 {
                    0.0
                } else {
                    (plane.y - a.position.y) / dy
                };
                let position =
                    a.position.coords + (b.position.coords - a.position.coords) * fraction;
                let velocity = a.velocity + (b.velocity - a.velocity) * fraction;
                let time_secs = a.time_secs + (b.time_secs - a.time_secs) * fraction;
                if time_secs <= params.min_lead || time_secs > params.max_lead {
                    return None;
                }
                return Some(Prediction {
                    time_to_impact_secs: time_secs,
                    impact_position: Point3::from(position),
                    incoming_velocity: velocity,
                });
            }
        }
        return None;
    }

    pub fn step(
        pos: Vector3<f64>,
        vel: Vector3<f64>,
        omega: Vector3<f64>,
        dt: f64,
        physics: &PhysicsParams,
    ) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        return ballistics::semi_implicit_euler(pos, vel, omega, dt, physics);
    }

    pub fn table_ball_mu(physics: &PhysicsParams) -> f64 {
        return bounce::table_ball_mu(physics);
    }

    pub fn rapier_table_ball_mu(physics: &PhysicsParams) -> f64 {
        return bounce::rapier_table_ball_mu(physics);
    }

    pub fn bounce_on_table(
        velocity: Vector3<f64>,
        omega: Vector3<f64>,
        physics: &PhysicsParams,
    ) -> (Vector3<f64>, Vector3<f64>) {
        return bounce::table_bounce(velocity, omega, physics);
    }
}

fn prediction_state_is_valid(position: Vector3<f64>, velocity: Vector3<f64>) -> bool {
    let finite = position
        .iter()
        .chain(velocity.iter())
        .all(|value| value.is_finite());
    // 타격 위치는 테이블 옆 밖의 로봇 작업 공간에도 올 수 있다
    // (fly_07 실측 x=1.598 m). 테이블 경계로 즉시 잘라내지 말고,
    // 캘리브레이션/로봇 작업 공간을 포함하는 여유 영역만 허용한다.
    const WORKSPACE_MARGIN_M: f64 = 0.5;
    return finite
        && position.z >= ball::RADIUS
        && position.x >= -WORKSPACE_MARGIN_M
        && position.x <= table::WIDTH_X + WORKSPACE_MARGIN_M
        && position.y >= -WORKSPACE_MARGIN_M
        && position.y <= table::LENGTH_Y + WORKSPACE_MARGIN_M;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampled_trajectory_uses_configured_interval_and_is_dynamically_continuous() {
        let physics = PhysicsParams::default();
        let start_position = Vector3::new(table::WIDTH_X * 0.5, 2.2, table::SURFACE_Z + 0.35);
        let start_velocity = Vector3::new(0.0, -4.0, 0.8);
        let samples = Kinematics::sample_trajectory(
            start_position,
            start_velocity,
            Vector3::zeros(),
            &physics,
        );
        assert!(samples.len() > 10);
        let dt = EstimatorParams::default().integrate_dt;
        assert!((samples[0].time_secs - dt).abs() < 1e-12);

        let mut previous_position = start_position;
        let mut previous_velocity = start_velocity;
        let mut previous_spin = Vector3::zeros();
        for sample in &samples {
            let (expected_position, expected_velocity, expected_spin) = Kinematics::step(
                previous_position,
                previous_velocity,
                previous_spin,
                dt,
                &physics,
            );
            assert!((sample.position.coords - expected_position).norm() < 1e-12);
            assert!((sample.velocity - expected_velocity).norm() < 1e-12);
            previous_position = sample.position.coords;
            previous_velocity = sample.velocity;
            previous_spin = expected_spin;
        }
    }

    #[test]
    fn sampled_trajectory_has_no_hit_plane_payload() {
        let fields = Kinematics::sample_trajectory(
            Vector3::new(0.7, 2.0, 1.0),
            Vector3::new(0.0, -3.0, 0.0),
            Vector3::zeros(),
            &PhysicsParams::default(),
        );
        assert!(fields.iter().all(|sample| sample.time_secs > 0.0));
    }

    #[test]
    fn trajectory_keeps_robot_workspace_just_outside_table_edge() {
        let fields = Kinematics::sample_trajectory(
            Vector3::new(table::WIDTH_X + 0.08, 1.0, table::SURFACE_Z + 0.25),
            Vector3::new(0.0, -2.0, 0.0),
            Vector3::zeros(),
            &PhysicsParams::default(),
        );
        assert!(!fields.is_empty());
    }
}
