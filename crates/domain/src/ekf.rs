//! 확장 칼만 필터 (plan §6.1–§6.2).
//!
//! 상태 `x = [p, v]`, 측정은 삼각측량 3D 위치.
//! 짧은 전파·hit-plane 예측 모두 반암시적 오일러 ([`crate::ballistics`]).

use std::time::Instant;

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

use crate::ballistics::{predict_hit_plane, semi_implicit_euler};
use crate::constants::estimator::{Q_POS, Q_VEL, R_MEAS};
use crate::constants::DEFAULT_DRAG;
use crate::ports::Estimator;
use crate::types::{HitPlane, Point3, Prediction, World};

/// EKF 상태: 위치·속도 + 공분산.
#[derive(Debug, Clone)]
pub struct BallEkf {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    covariance: Matrix6<f64>,
    last_time: Option<Instant>,
    drag_coefficient: f64,
    initialized: bool,
}

impl BallEkf {
    /// 항력 계수를 지정해 생성한다.
    pub fn new(drag_coefficient: f64) -> Self {
        return Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            covariance: Matrix6::identity(),
            last_time: None,
            drag_coefficient,
            initialized: false,
        };
    }

    /// 기본 항력으로 생성.
    pub fn with_defaults() -> Self {
        return Self::new(DEFAULT_DRAG);
    }

    /// 현재 위치 추정.
    pub fn position(&self) -> Option<Point3<World>> {
        if !self.initialized {
            return None;
        }
        return Some(Point3::from_vector(self.position));
    }

    /// 현재 속도 추정.
    pub fn velocity(&self) -> Option<Vector3<f64>> {
        if !self.initialized {
            return None;
        }
        return Some(self.velocity);
    }

    /// 테스트·sim 오라클용: 상태 직접 설정.
    pub fn set_state(&mut self, position: Vector3<f64>, velocity: Vector3<f64>, time: Instant) {
        self.position = position;
        self.velocity = velocity;
        self.covariance = Matrix6::identity() * 0.01;
        self.initialized = true;
        self.last_time = Some(time);
    }

    /// 3D 관측으로 보정한다.
    pub fn update_position(&mut self, measured: Point3<World>, timestamp: Instant) {
        if let Some(prev) = self.last_time {
            let dt = timestamp.duration_since(prev).as_secs_f64();
            if dt > 1e-4 && dt < 0.5 {
                self.predict_step(dt);
            }
        }

        if !self.initialized {
            self.position = measured.v;
            self.velocity = Vector3::zeros();
            self.covariance = Matrix6::identity() * 0.1;
            self.initialized = true;
            self.last_time = Some(timestamp);
            return;
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
        let (pos, vel) =
            semi_implicit_euler(self.position, self.velocity, dt, self.drag_coefficient);
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
    fn update(&mut self, position: Point3<World>, timestamp: Instant) {
        self.update_position(position, timestamp);
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        if !self.initialized {
            return None;
        }
        return predict_hit_plane(self.position, self.velocity, plane, self.drag_coefficient);
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::*;
    use crate::constants::table;

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
        let mut ekf = BallEkf::with_defaults();
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
}
