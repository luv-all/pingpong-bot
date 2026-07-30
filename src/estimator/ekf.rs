//! 확장 칼만 필터.
//!
//! 상태 x = [p, v], 측정은 삼각측량 3D 위치.
//! 짧은 전파와 hit-plane 예측은 반암시적 오일러 (`ballistics`).
//!
//! 오검출은 마할라노비스 게이트로 걸러 **무시**한다 ([`GateOutcome`]) — 검출기가
//! 한 프레임 엉뚱한 곳을 잡아도 필터는 예측으로 넘기고 다음 측정에서 이어붙어,
//! 위치·속도 추정이 끊기지 않는다. 게이트가 연속으로 막으면 그때 리셋한다.
//!
//! sim Rapier에는 이차 항력이 없어서(기본 drag=0) 파이프라인은
//! `estimator::Ekf::new(0.0)` 을 쓴다. Magnus는 ω 상태 확장 전까지 예측에서 0.

use std::time::Instant;

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

use crate::Point3;
use crate::defaults;
use crate::defaults::PhysicsParams;
use crate::estimator;
use crate::estimator::{Estimator, HitPlane, Prediction};

/// 시드 직후 공분산 대각 (위치만 알고 속도는 모름).
const SEED_COV: f64 = 0.1;

/// 속도 시드 직후 공분산 — **상수가 아니라 측정 노이즈와 실제 간격에서 계산한다**.
///
/// 속도는 측정되지 않는다. 두 번째 측정에서 `v = Δp/Δt`로 만드는데, 두 위치 모두
/// 분산 `r_meas`의 노이즈를 가지므로 `Var(v) = 2·r_meas/Δt²`다. 위치는 방금 그 측정으로
/// 놓았으니 `Var(p) = r_meas`.
///
/// 예전에는 6축 전부 상수 0.05였다. 속도 쪽이 σ 0.22 m/s라는 뜻인데, 실측은 전혀 다르다 —
/// 클립 3개에서 σ_p 2.8~3.8 cm, dt 24~29 ms이므로 σ_v는 **1.4~2.2 m/s**(분산 1.9~4.9)다.
/// 즉 필터가 자기 속도 시드를 분산 기준 **37~99배 과신**했다. 그러면 그 틀린 속도로 예측한
/// 위치와 어긋나는 측정이 마할라노비스 게이트에 "튄 값"으로 걸려 거부되고, 필터는 스스로
/// 고칠 기회를 막은 채 관성 주행한다. fly_04에서 예측 x가 2.568 m에 얼어붙은 채 실제 도달
/// 0.367 m와 2.2 m 어긋난 게 이 모습이었다 (`diag_clip_prediction`).
///
/// 프레임레이트가 바뀌면 σ_v도 같이 바뀌므로 상수로 박으면 안 된다.
fn velocity_seed_covariance(r_meas: f64, dt: f64) -> Matrix6<f64> {
    let position_var = r_meas;
    let velocity_var = 2.0 * r_meas / (dt * dt);
    let mut covariance = Matrix6::zeros();
    for axis in 0..3 {
        covariance[(axis, axis)] = position_var;
        covariance[(axis + 3, axis + 3)] = velocity_var;
    }
    return covariance;
}

/// 측정 한 건을 필터가 어떻게 처리했는지.
///
/// 툴 HUD·텔레메트리가 "지금 튄 건지"를 프레임 단위로 읽는 용도.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateOutcome {
    /// 첫 측정 — 위치만 심었다. 속도는 아직 없다.
    Seeded,
    /// 두 번째 측정 — 유한차분으로 속도를 심었다.
    VelocitySeeded,
    /// 게이트 통과 — 칼만 보정을 적용했다.
    Accepted,
    /// 게이트 밖 — **무시**했다. 필터는 예측만으로 그 프레임을 넘긴다.
    Rejected,
    /// 연속 거부 한도 초과 또는 긴 공백 — 트랙을 버리고 이 측정으로 재시드.
    Reset,
    /// 타임스탬프 역행 등 쓸 수 없는 측정 — 상태를 건드리지 않았다.
    Ignored,
}

impl GateOutcome {
    /// 필터 상태가 이 측정을 반영했는가.
    pub fn used_measurement(&self) -> bool {
        return !matches!(self, GateOutcome::Rejected | GateOutcome::Ignored);
    }
}

/// EKF 상태: 위치/속도 + 공분산.
#[derive(Debug, Clone)]
pub struct Ekf {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    covariance: Matrix6<f64>,
    last_time: Option<Instant>,
    physics: PhysicsParams,
    initialized: bool,
    /// 두 번째 측정에서 finite-difference로 속도를 심었는지.
    velocity_seeded: bool,
    /// 연속으로 게이트에 막힌 측정 수 (통과하면 0).
    reject_streak: u32,
    /// 직전 측정의 마할라노비스 d² (게이트 판정 전 값).
    last_gate_d2: Option<f64>,
}

impl Ekf {
    /// 항력 계수를 지정해 생성한다 (바운스는 default physics).
    pub fn new(drag_coefficient: f64) -> Self {
        return Self::with_physics(PhysicsParams {
            drag: drag_coefficient,
            ..crate::defaults::PhysicsParams::default()
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
            reject_streak: 0,
            last_gate_d2: None,
        };
    }

    /// 필터를 비운다 (다음 관측에서 재시드).
    pub fn reset(&mut self) {
        self.initialized = false;
        self.velocity_seeded = false;
        self.last_time = None;
        self.position = Vector3::zeros();
        self.velocity = Vector3::zeros();
        self.covariance = Matrix6::identity();
        self.reject_streak = 0;
        self.last_gate_d2 = None;
    }

    /// 연속 거부 수 — 0이면 직전 측정을 받아들였다.
    pub fn reject_streak(&self) -> u32 {
        return self.reject_streak;
    }

    /// 직전 측정의 마할라노비스 d². 임계는 [`EstimatorParams::gate_chi2`].
    ///
    /// [`EstimatorParams::gate_chi2`]: crate::defaults::EstimatorParams::gate_chi2
    pub fn last_gate_d2(&self) -> Option<f64> {
        return self.last_gate_d2;
    }

    /// 위치·속도가 모두 서 있어 예측을 낼 수 있는 상태인가.
    pub fn is_tracking(&self) -> bool {
        return self.initialized && self.velocity_seeded;
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
        self.reject_streak = 0;
        self.last_gate_d2 = None;
    }

    /// 위치만 심는다 (첫 측정 · 리셋 직후).
    fn seed(&mut self, measured: Point3, timestamp: Instant) {
        self.position = measured.coords;
        self.velocity = Vector3::zeros();
        self.covariance = Matrix6::identity() * SEED_COV;
        self.initialized = true;
        self.velocity_seeded = false;
        self.reject_streak = 0;
        self.last_gate_d2 = None;
        self.last_time = Some(timestamp);
    }

    /// 3D 관측으로 보정한다.
    ///
    /// 예측 대비 잔차가 마할라노비스 게이트 밖이면 그 측정을 **무시**한다 —
    /// 필터는 자기 예측대로 그 프레임을 넘기고 다음 정상 측정에서 이어붙는다.
    /// 오검출 하나로 속도 추정이 끊기지 않게 하는 게 목적이다. 거부가
    /// [`gate_reject_limit`] 회 연속되면 그때는 필터 쪽이 틀린 것으로 보고
    /// 트랙을 버린 뒤 그 측정으로 재시드한다.
    ///
    /// [`gate_reject_limit`]: crate::defaults::EstimatorParams::gate_reject_limit
    pub fn update_position(&mut self, measured: Point3, timestamp: Instant) -> GateOutcome {
        let params = defaults::EstimatorParams::default();
        let mut stale = false;

        if let Some(prev) = self.last_time {
            let dt = timestamp.duration_since(prev).as_secs_f64();
            if dt < 0.0 {
                return GateOutcome::Ignored;
            }
            // 긴 공백(세션 공백/프레임 드롭) -> 하드 리셋
            if dt >= params.stale_gap_secs {
                self.reset();
                stale = true;
            } else if self.initialized && self.velocity_seeded && dt > 1e-4 {
                self.predict_step(dt);
            }
        }

        if !self.initialized {
            self.seed(measured, timestamp);
            return if stale {
                GateOutcome::Reset
            } else {
                GateOutcome::Seeded
            };
        }

        let r = Matrix3::identity() * params.r_meas;
        let p_ht = self.covariance.fixed_view::<6, 3>(0, 0).into_owned();
        let s = self.covariance.fixed_view::<3, 3>(0, 0) + r;
        let Some(s_inv) = s.try_inverse() else {
            self.last_time = Some(timestamp);
            return GateOutcome::Ignored;
        };
        let innovation = measured.coords - self.position;

        // 게이트: 잔차를 혁신 공분산으로 정규화 — 필터가 확신할수록 좁아진다.
        let d2 = (innovation.transpose() * s_inv * innovation)[(0, 0)];
        self.last_gate_d2 = Some(d2);
        if d2 > params.gate_chi2 {
            self.reject_streak += 1;
            if self.reject_streak >= params.gate_reject_limit {
                self.reset();
                self.seed(measured, timestamp);
                return GateOutcome::Reset;
            }
            // 예측으로 이미 timestamp까지 전파했으면 시계를 맞춰둔다. 속도 시드
            // 전이면 그대로 둬야 다음 정상 측정의 유한차분 Δt가 맞는다.
            if self.velocity_seeded {
                self.last_time = Some(timestamp);
            }
            return GateOutcome::Rejected;
        }
        self.reject_streak = 0;

        // 두 번째 측정: Δp/Δt로 속도 시드 (v=0 초기화 잔여 제거)
        if !self.velocity_seeded {
            if let Some(prev) = self.last_time {
                let dt = timestamp.duration_since(prev).as_secs_f64();
                if dt > 1e-4 {
                    self.velocity = innovation / dt;
                    self.position = measured.coords;
                    self.covariance = velocity_seed_covariance(params.r_meas, dt);
                    self.velocity_seeded = true;
                    self.last_time = Some(timestamp);
                    return GateOutcome::VelocitySeeded;
                }
            }
        }

        let gain = p_ht * s_inv;
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
        return GateOutcome::Accepted;
    }

    fn predict_step(&mut self, dt: f64) {
        // ω 추정 전 — 각속도 0으로 전파.
        let (pos, vel, _) = estimator::Kinematics::step(
            self.position,
            self.velocity,
            Vector3::zeros(),
            dt,
            &self.physics,
        );
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
    let qp = defaults::EstimatorParams::default().q_pos * dt.max(1e-3);
    let qv = defaults::EstimatorParams::default().q_vel * dt.max(1e-3);
    for i in 0..3 {
        q[(i, i)] = qp;
        q[(i + 3, i + 3)] = qv;
    }
    return q;
}

impl Estimator for Ekf {
    fn update(&mut self, position: Point3, timestamp: Instant) {
        let _ = self.update_position(position, timestamp);
    }

    fn predict_to(&self, plane: HitPlane) -> Option<Prediction> {
        if !self.initialized || !self.velocity_seeded {
            return None;
        }
        return estimator::Kinematics::predict_to(
            self.position,
            self.velocity,
            Vector3::zeros(),
            plane,
            &self.physics,
        );
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::*;
    use crate::constants::table;
    use crate::estimator;
    use crate::estimator::HitPlane;
    use crate::robot::motion;

    #[test]
    fn ekf_predicts_hit_plane_from_state() {
        let mut ekf = Ekf::new(0.0);
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
        assert!((pred.impact_position.coords.y - plane.y).abs() < 1e-4);
        assert!(pred.incoming_velocity.y < 0.0);
    }

    #[test]
    fn ekf_update_accepts_measurements() {
        let mut ekf = Ekf::new(0.0);
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
        let mut ekf = Ekf::new(0.0);
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

    /// 실제 탄도를 8 ms 간격으로 먹여 트랙을 세운다. 다음 프레임의 진실
    /// 상태도 같이 돌려줘 테스트가 궤적을 이어갈 수 있게 한다.
    fn tracked_ekf(t0: Instant, frames: u32) -> (Ekf, Vector3<f64>, Vector3<f64>) {
        let physics = crate::defaults::PhysicsParams::default();
        let mut ekf = Ekf::new(0.0);
        let mut pos = Vector3::new(table::WIDTH_X * 0.5, 2.4, table::SURFACE_Z + 0.25);
        let mut vel = Vector3::new(0.0, -5.5, 0.8);
        for i in 0..frames {
            ekf.update_position(Point3::from(pos), t0 + Duration::from_millis(8) * i);
            let (np, nv, _) =
                estimator::Kinematics::step(pos, vel, Vector3::zeros(), 0.008, &physics);
            pos = np;
            vel = nv;
        }
        return (ekf, pos, vel);
    }

    #[test]
    fn outlier_is_rejected_without_losing_track() {
        let t0 = Instant::now();
        let (mut ekf, _, truth_v) = tracked_ekf(t0, 10);
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let before = ekf.velocity().expect("tracking");
        let before_impact = ekf.predict_to(plane).expect("예측 성립").impact_position;

        // 화면 반대편 오검출 한 프레임
        let outcome =
            ekf.update_position(Point3::new(-1.5, 0.2, 0.3), t0 + Duration::from_millis(80));

        assert_eq!(outcome, GateOutcome::Rejected);
        assert_eq!(ekf.reject_streak(), 1);
        // 속도는 살아 있고 오검출 방향으로 끌려가지 않았다
        let after = ekf.velocity().expect("속도 유지");
        assert!(
            (after - before).norm() < 0.5,
            "거부된 측정이 속도를 흔들었다: {before:?} -> {after:?}"
        );
        assert!(
            (after.y - truth_v.y).abs() < 1.0,
            "vy≈{:.1} 유지, got {}",
            truth_v.y,
            after.y
        );
        // 예측도 끊기지 않고, 튄 점 쪽으로 흔들리지 않는다
        let after_impact = ekf.predict_to(plane).expect("예측 유지").impact_position;
        assert!(
            (after_impact.coords - before_impact.coords).norm() < 0.05,
            "거부 후 임팩트 예측이 흔들렸다: {before_impact:?} -> {after_impact:?}"
        );
    }

    #[test]
    fn track_recovers_on_next_good_measurement() {
        let t0 = Instant::now();
        let (mut ekf, pos10, vel10) = tracked_ekf(t0, 10);
        ekf.update_position(Point3::new(-1.5, 0.2, 0.3), t0 + Duration::from_millis(80));

        // 궤적을 이어가는 정상 측정 — 아무 일 없었다는 듯 받아들인다
        let physics = crate::defaults::PhysicsParams::default();
        let (good, _, _) =
            estimator::Kinematics::step(pos10, vel10, Vector3::zeros(), 0.008, &physics);
        let outcome = ekf.update_position(Point3::from(good), t0 + Duration::from_millis(88));

        assert_eq!(outcome, GateOutcome::Accepted);
        assert_eq!(ekf.reject_streak(), 0);
        assert!(ekf.is_tracking());
    }

    #[test]
    fn sustained_jump_resets_track() {
        let t0 = Instant::now();
        let (mut ekf, _, _) = tracked_ekf(t0, 10);
        let limit = defaults::EstimatorParams::default().gate_reject_limit;

        // 슈터로 텔레포트 — 새 위치에 계속 머문다
        let mut outcome = GateOutcome::Accepted;
        for i in 0..limit {
            outcome = ekf.update_position(
                Point3::new(0.2, 0.3, 0.9),
                t0 + Duration::from_millis(80) + Duration::from_millis(8) * i,
            );
        }

        assert_eq!(outcome, GateOutcome::Reset);
        // 리셋 후 첫 측정만 — 속도 미시드
        assert!(ekf.velocity().is_none());
        assert_eq!(ekf.reject_streak(), 0);
    }

    #[test]
    fn bounce_does_not_trip_the_gate() {
        let t0 = Instant::now();
        let mut ekf = Ekf::new(0.0);
        let physics = crate::defaults::PhysicsParams::default();
        let mut pos = Vector3::new(0.7, 2.4, table::SURFACE_Z + 0.30);
        let mut vel = Vector3::new(0.0, -5.0, -2.0);
        let mut resets = 0;

        for i in 0..40_u32 {
            let outcome = ekf.update_position(Point3::from(pos), t0 + Duration::from_millis(8) * i);
            if outcome == GateOutcome::Reset {
                resets += 1;
            }
            let (np, nv, _) =
                estimator::Kinematics::step(pos, vel, Vector3::zeros(), 0.008, &physics);
            pos = np;
            vel = nv;
            // 테이블 반발 — 위치는 이어지고 vz만 뒤집힌다
            if pos.z <= table::SURFACE_Z && vel.z < 0.0 {
                let (bv, _) =
                    estimator::Kinematics::bounce_on_table(vel, Vector3::zeros(), &physics);
                vel = bv;
            }
        }

        assert_eq!(resets, 0, "바운스는 위치 연속 — 리셋되면 안 된다");
        assert!(ekf.is_tracking());
    }

    #[test]
    fn stale_gap_resets_track() {
        let t0 = Instant::now();
        let (mut ekf, _, _) = tracked_ekf(t0, 10);
        let gap = defaults::EstimatorParams::default().stale_gap_secs;

        let outcome = ekf.update_position(
            Point3::new(0.7, 1.9, 0.95),
            t0 + Duration::from_secs_f64(gap + 0.1),
        );

        assert_eq!(outcome, GateOutcome::Reset);
        assert!(ekf.velocity().is_none());
    }

    #[test]
    fn tracked_ballistic_impact_near_truth_in_commit_window() {
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let p0 = Vector3::new(table::WIDTH_X * 0.5, 2.4, table::SURFACE_Z + 0.25);
        let v0 = Vector3::new(0.0, -5.5, 0.8);
        let physics = crate::defaults::PhysicsParams::default();
        let truth0 = estimator::Kinematics::predict_to(p0, v0, Vector3::zeros(), plane, &physics)
            .expect("truth");

        let mut ekf = Ekf::new(0.0);
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
                if motion::Planner::in_commit_window(pred.time_to_impact_secs)
                    && pos.y
                        <= table::LENGTH_Y
                            * defaults::ControlParams::default().swing_commit_max_ball_y_frac
                {
                    let err = (pred.impact_position.coords - truth0.impact_position.coords).norm();
                    best_err = best_err.min(err);
                }
            }
            let (np, nv, _) =
                estimator::Kinematics::step(pos, vel, Vector3::zeros(), 0.008, &physics);
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
