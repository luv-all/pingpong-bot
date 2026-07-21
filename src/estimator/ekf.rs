//! 확장 칼만 필터.
//!
//! 상태 x = [p, v], 측정은 삼각측량 3D 위치.
//! 짧은 전파와 hit-plane 예측은 반암시적 오일러 (`ballistics`).
//!
//! sim Rapier에는 이차 항력이 없어서 파이프라인은 BallEkf::new(0.0) 을 쓴다.
//! with_defaults(DEFAULT_DRAG) 는 실측 k 가 있을 때.

use std::time::Instant;

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

use super::ballistics::{predict_hit_plane_with, semi_implicit_euler};
use crate::constants::DEFAULT_DRAG;
use crate::constants::control::EKF_MEAS_JUMP_M;
use crate::constants::estimator::{Q_POS, Q_VEL, R_MEAS};
use crate::estimator::Estimator;
use crate::physics_config::PhysicsParams;
use crate::{HitPlane, Point3, Prediction};

/// EKF 상태: 위치/속도 + 공분산.
#[derive(Debug, Clone)]
pub struct BallEkf {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    covariance: Matrix6<f64>,
    last_time: Option<Instant>,
    physics: PhysicsParams,
    initialized: bool,
    /// 두 번째 측정에서 finite-difference로 속도를 심었는지.
    velocity_seeded: bool,
}

impl BallEkf {
    /// 항력 계수를 지정해 생성한다 (바운스는 default physics).
    pub fn new(drag_coefficient: f64) -> Self {
        return Self::with_physics(PhysicsParams {
            drag: drag_coefficient,
            ..PhysicsParams::default()
        });
    }

    /// config `[physics]` 등에서 만든 파라미터로 생성.
    pub fn with_physics(physics: PhysicsParams) -> Self {
        return Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            covariance: Matrix6::identity(),
            last_time: None,
            physics,
            initialized: false,
            velocity_seeded: false,
        };
    }

    /// 기본 항력으로 생성 (실측 전 추정 k).
    pub fn with_defaults() -> Self {
        return Self::new(DEFAULT_DRAG);
    }

    /// 필터를 비운다 (다음 관측에서 재시드).
    pub fn reset(&mut self) {
        self.initialized = false;
        self.velocity_seeded = false;
        self.last_time = None;
        self.position = Vector3::zeros();
        self.velocity = Vector3::zeros();
        self.covariance = Matrix6::identity();
    }

    /// 현재 위치 추정.
    pub fn position(&self) -> Option<Point3> {
        if !self.initialized {
            return None;
        }
        return Some(Point3::from(self.position));
    }

    /// 현재 속도 추정.
    pub fn velocity(&self) -> Option<Vector3<f64>> {
        if !self.initialized || !self.velocity_seeded {
            return None;
        }
        return Some(self.velocity);
    }

    /// 테스트/sim ground truth 경로용: 상태 직접 설정.
    pub fn set_state(&mut self, position: Vector3<f64>, velocity: Vector3<f64>, time: Instant) {
        self.position = position;
        self.velocity = velocity;
        self.covariance = Matrix6::identity() * 0.01;
        self.initialized = true;
        self.velocity_seeded = true;
        self.last_time = Some(time);
    }

    /// 3D 관측으로 보정한다.
    pub fn update_position(&mut self, measured: Point3, timestamp: Instant) {
        if let Some(prev) = self.last_time {
            let dt = timestamp.duration_since(prev).as_secs_f64();
            if dt < 0.0 {
                return;
            }
            // 긴 공백(세션 공백/프레임 드롭) -> 하드 리셋
            if dt >= 0.5 {
                self.reset();
            } else if self.initialized && self.velocity_seeded && dt > 1e-4 {
                self.predict_step(dt);
                // 주차<->발사 텔레포트: 예측 후에도 잔차가 크면 리셋
                if (measured.v - self.position).norm() > EKF_MEAS_JUMP_M {
                    self.reset();
                }
            } else if self.initialized && !self.velocity_seeded {
                // 시드 전: 원시 위치 점프만 검사
                if (measured.v - self.position).norm() > EKF_MEAS_JUMP_M {
                    self.reset();
                }
            }
        }

        if !self.initialized {
            self.position = measured.v;
            self.velocity = Vector3::zeros();
            self.covariance = Matrix6::identity() * 0.1;
            self.initialized = true;
            self.velocity_seeded = false;
            self.last_time = Some(timestamp);
            return;
        }

        // 두 번째 측정: Δp/Δt로 속도 시드 (v=0 초기화 잔여 제거)
        if !self.velocity_seeded {
            if let Some(prev) = self.last_time {
                let dt = timestamp.duration_since(prev).as_secs_f64();
                if dt > 1e-4 {
                    self.velocity = (measured.v - self.position) / dt;
                    self.position = measured.v;
                    self.covariance = Matrix6::identity() * 0.05;
                    self.velocity_seeded = true;
                    self.last_time = Some(timestamp);
                    return;
                }
            }
        }

        let r = Matrix3::identity() * R_MEAS;
        let p_ht = self.covariance.fixed_view::<6, 3>(0, 0).into_owned();
        let s = self.covariance.fixed_view::<3, 3>(0, 0) + r;
        let Some(s_inv) = s.try_inverse() else {
            self.last_time = Some(timestamp);
            return;
        };
        let gain = p_ht * s_inv;
        let innovation = measured.v - self.position;
        let dx: Vector6<f64> = gain * innovation;
        self.position += Vector3::new(dx[0], dx[1], dx[2]);
        self.velocity += Vector3::new(dx[3], dx[4], dx[5]);

        let mut i_kh = Matrix6::identity();
        {
            let mut view = i_kh.fixed_view_mut::<6, 3>(0, 0);
            view -= gain;
        }
        self.covariance = i_kh * self.covariance;
        // 대칭성 유지
        self.covariance = 0.5 * (self.covariance + self.covariance.transpose());

        self.last_time = Some(timestamp);
    }

    fn predict_step(&mut self, dt: f64) {
        let (pos, vel) = semi_implicit_euler(self.position, self.velocity, dt, self.physics.drag);
        self.position = pos;
        self.velocity = vel;

        let mut f = Matrix6::identity();
        f.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Matrix3::identity() * dt));
        let q = process_noise(dt);
        self.covariance = f * self.covariance * f.transpose() + q;
    }
}

fn process_noise(dt: f64) -> Matrix6<f64> {
    let mut q = Matrix6::zeros();
    let qp = Q_POS * dt.max(1e-3);
    let qv = Q_VEL * dt.max(1e-3);
    for i in 0..3 {
        q[(i, i)] = qp;
        q[(i + 3, i + 3)] = qv;
    }
    return q;
}

impl Estimator for BallEkf {
    fn update(&mut self, position: Point3, timestamp: Instant) {
        self.update_position(position, timestamp);
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        if !self.initialized || !self.velocity_seeded {
            return None;
        }
        return predict_hit_plane_with(self.position, self.velocity, plane, &self.physics);
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::*;
    use crate::constants::{control, table};
    use crate::estimator::ballistics::{predict_hit_plane, semi_implicit_euler};
    use crate::planner::physics::in_swing_commit_window;

    #[test]
    fn ekf_predicts_hit_plane_from_state() {
        let mut ekf = BallEkf::new(0.0);
        let t0 = Instant::now();
        ekf.set_state(
            Vector3::new(table::WIDTH_X * 0.5, 2.0, table::SURFACE_Z + 0.3),
            Vector3::new(0.0, -6.0, 0.2),
            t0,
        );
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let pred = ekf.predict_to(plane).expect("hit plane");
        assert!((pred.impact_position.v.y - plane.y).abs() < 1e-4);
        assert!(pred.incoming_velocity.y < 0.0);
    }

    #[test]
    fn ekf_update_accepts_measurements() {
        let mut ekf = BallEkf::new(0.0);
        let t0 = Instant::now();
        for i in 0..10 {
            ekf.update_position(
                Point3::new(0.7, 2.0 - 0.05 * f64::from(i as u32), 0.9),
                t0 + Duration::from_millis(i * 8),
            );
        }
        assert!(ekf.position().is_some());
        assert!(ekf.velocity().is_some());
    }

    #[test]
    fn velocity_seeded_on_second_measurement() {
        let mut ekf = BallEkf::new(0.0);
        let t0 = Instant::now();
        ekf.update_position(Point3::new(0.7, 2.0, 0.95), t0);
        assert!(ekf.velocity().is_none());
        ekf.update_position(Point3::new(0.7, 1.95, 0.95), t0 + Duration::from_millis(8));
        let v = ekf.velocity().expect("seeded");
        assert!(
            (v.y - (-0.05 / 0.008)).abs() < 1.0,
            "finite-diff vy~=-6.25, got {}",
            v.y
        );
    }

    #[test]
    fn jump_reinitializes_filter() {
        let mut ekf = BallEkf::new(0.0);
        let t0 = Instant::now();
        ekf.set_state(
            Vector3::new(0.2, 0.3, 0.9),
            Vector3::new(0.0, -1.0, 0.0),
            t0,
        );
        // 슈터로 텔레포트
        ekf.update_position(Point3::new(0.7, 2.5, 1.0), t0 + Duration::from_millis(16));
        // 리셋 후 첫 측정만 - 속도 미시드
        assert!(ekf.velocity().is_none());
    }

    #[test]
    fn tracked_ballistic_impact_near_truth_in_commit_window() {
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let p0 = Vector3::new(table::WIDTH_X * 0.5, 2.4, table::SURFACE_Z + 0.25);
        let v0 = Vector3::new(0.0, -5.5, 0.8);
        let truth0 = predict_hit_plane(p0, v0, plane, 0.0).expect("truth");

        let mut ekf = BallEkf::new(0.0);
        let t0 = Instant::now();
        let dt = Duration::from_millis(8);
        let mut pos = p0;
        let mut vel = v0;
        let mut t = 0.0_f64;
        let mut best_err = f64::MAX;

        for i in 0..200 {
            let time = t0 + dt * i;
            ekf.update_position(Point3::from(pos), time);
            if let Some(pred) = ekf.predict_to(plane) {
                if in_swing_commit_window(pred.time_to_impact_secs)
                    && pos.y <= table::LENGTH_Y * control::SWING_COMMIT_MAX_BALL_Y_FRAC
                {
                    let err = (pred.impact_position.v - truth0.impact_position.v).norm();
                    best_err = best_err.min(err);
                }
            }
            let (np, nv) = semi_implicit_euler(pos, vel, 0.008, 0.0);
            pos = np;
            vel = nv;
            t += 0.008;
            if pos.y < plane.y {
                break;
            }
            let _ = t;
        }

        assert!(
            best_err < 0.08,
            "commit 창에서 impact RMSE {best_err:.3} m (목표 < 8 cm), truth tti={}",
            truth0.time_to_impact_secs
        );
    }
}
