//! 상태 `[p, v]` 6차원. 관측은 삼각측량한 3D 점이 아니라 픽셀 2차원이다.
//!
//! 두 카메라는 하드웨어 동기가 없어 최대 18.9 ms 어긋난다 (실측 p95). 삼각측량은 두 시선이
//! 같은 순간이라고 가정하므로 5 m/s 공이면 9.5 cm가 틀어진다. 픽셀은 각자 자기 시각에 쓰니
//! 그 가정이 없다.
//!
//! 대신 [`Ekf::seed`]에서는 3D 점이 필요해 그 오차를 한 번 문다. 매 프레임이 아니라
//! 샷당 한 번이다.

use std::time::Duration;

use nalgebra::{Matrix2, Matrix2x3, Matrix2x6, Matrix6, Vector2, Vector6};

use crate::camera::{self, Calibration};
use crate::constants::table;
use crate::defaults::PhysicsParams;
use crate::estimator::Kinematics;
use crate::{Point3, Vector3};

use super::contract::{State, Track, Trajectory};
use super::detect::Candidate;
use super::seed;
use super::trigger::Trigger;

/// 검출 픽셀 노이즈 σ [px]. 실측 가능한 값이라 모델 오차는 R이 아니라 Q로 간다.
pub const SIGMA_PX: f64 = 1.5;
/// 프로세스 노이즈 — 위치 [m²/s], 속도 [m²/s³].
pub const Q_POSITION: f64 = 1.0e-4;
pub const Q_VELOCITY: f64 = 1.0e-2;
/// χ²(2) 99 % — 픽셀 잔차 게이트.
pub const GATE_CHI2: f64 = 9.21;
/// 연속 거부 한도.
pub const REJECT_LIMIT: u32 = 5;
/// 관측이 이만큼 끊기면 트랙을 버린다.
pub const STALE_GAP: Duration = Duration::from_millis(500);
/// 공이 멀어졌다고 볼 y 증가량 [m]과 연속 횟수.
pub const RECEDE_STEP: f64 = 0.05;
pub const RECEDE_LIMIT: u32 = 3;
/// 시드 직후 속도 불확실성 [m/s]. 속도는 측정되지 않으므로 정직하게 크게 잡는다.
pub const SEED_SPEED_SIGMA: f64 = 15.0;
/// 예측 궤적 적분 스텝과 표본 간격.
pub const INTEGRATE_DT: Duration = Duration::from_millis(1);
pub const SAMPLE_DT: Duration = Duration::from_millis(5);
/// 예측 궤적 상한.
pub const HORIZON: Duration = Duration::from_secs(2);
/// 플레이 부피 여유 [m].
const VOLUME_MARGIN: f64 = 1.0;
/// 시드 버퍼에 검출을 들고 있을 시간. 시드는 두 시선이 필요한데 프레임은 한 대씩 온다.
pub const PENDING_TTL: Duration = Duration::from_millis(50);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    Seeded,
    Accepted,
    /// 예측과 어긋나 무시했다. 트랙은 유지한다.
    Rejected {
        d2: f64,
    },
    /// 상태가 없거나 쓸 수 없는 관측이라 아무것도 안 했다.
    Idle,
}

/// 공 하나의 추정. 시드, 트리거 판정, 예측 궤적 만들기를 전부 안에서 한다.
///
/// 카메라 파라미터를 직접 들고 있어 검출기와 분리된다. 실기 경로는 카메라마다 스레드를
/// 띄워 검출하고 그 결과만 여기로 보낸다.
pub struct Ekf {
    x: Vector6<f64>,
    p: Matrix6<f64>,
    seq: u64,
    physics: PhysicsParams,
    trigger: Box<dyn Trigger>,
    /// 개수는 캘리브레이션 파일이 정한다.
    cameras: Vec<camera::Params>,
    measured: Track,
    /// 비어 있으면 아직 안 만든 것이다. 별도 플래그를 두지 않는다.
    predicted: Track,
    /// 시드 전에만 쓴다.
    pending: Vec<(camera::Id, Candidate, Duration)>,
    rejects: u32,
    recedes: u32,
    last_seen: Option<Duration>,
}

impl Ekf {
    pub fn new(calibration: &Calibration, trigger: Box<dyn Trigger>) -> Self {
        return Self {
            x: Vector6::zeros(),
            p: Matrix6::identity(),
            seq: 0,
            physics: PhysicsParams::default(),
            trigger,
            cameras: calibration.cameras.clone(),
            measured: Track::default(),
            predicted: Track::default(),
            pending: Vec::new(),
            rejects: 0,
            recedes: 0,
            last_seen: None,
        };
    }

    /// `false`면 아직 시드 전이다.
    pub fn has_track(&self) -> bool {
        return !self.measured.is_empty();
    }

    pub fn seq(&self) -> u64 {
        return self.seq;
    }

    /// 검출 하나를 먹인다. 트랙이 없으면 시드로, 있으면 보정으로 간다.
    pub fn observe(
        &mut self,
        camera_id: camera::Id,
        found: Option<Candidate>,
        t: Duration,
    ) -> Outcome {
        let Some(candidate) = found else {
            return Outcome::Idle;
        };
        if self.has_track() {
            return self.correct(camera_id, candidate, t);
        }
        // 같은 카메라의 이전 검출은 버린다 — 시드엔 서로 다른 시선이 필요하다.
        self.pending
            .retain(|(id, _, at)| *id != camera_id && t.saturating_sub(*at) <= PENDING_TTL);
        self.pending.push((camera_id, candidate, t));
        if self.pending.len() < 2 || !self.seed() {
            return Outcome::Idle;
        }
        self.pending.clear();
        return Outcome::Seeded;
    }

    /// 첫 상태를 세운다. 삼각측량 1회이고, 여기서만 다른 카메라를 기다린다.
    ///
    /// 시드 시각은 두 시각의 중간으로 잡고, 어긋난 `skew × 속도`만큼 위치 공분산을 부풀린다.
    /// 속도는 측정되지 않으므로 0으로 두고 σ를 [`SEED_SPEED_SIGMA`]로 크게 잡는다.
    fn seed(&mut self) -> bool {
        let views: Vec<_> = self
            .pending
            .iter()
            .filter_map(|(id, candidate, at)| {
                let params = self.cameras.iter().find(|p| p.camera_id == *id)?;
                return Some((params, *candidate, *at));
            })
            .collect();
        if views.len() < 2 {
            return false;
        }
        let Some(point) = seed::seed_state(&views) else {
            return false;
        };
        let skew = seed::skew(&views).as_secs_f64();
        let mid = seed::midpoint(&views);
        drop(views);

        self.x = Vector6::new(point.x, point.y, point.z, 0.0, 0.0, 0.0);
        self.p = Matrix6::zeros();
        let position_var =
            seed::TRIANGULATION_SIGMA.powi(2) + (0.5 * skew * SEED_SPEED_SIGMA).powi(2);
        for axis in 0..3 {
            self.p[(axis, axis)] = position_var;
            self.p[(axis + 3, axis + 3)] = SEED_SPEED_SIGMA.powi(2);
        }
        self.rejects = 0;
        self.recedes = 0;
        self.last_seen = Some(mid);
        self.measured.0.push(self.state_at(mid));
        return true;
    }

    /// 검출 하나로 보정한다.
    ///
    /// 야코비안은 핀홀 투영의 미분(2×6), `R = σ_px² · I₂`. 잔차가 게이트 밖이면 무시하고
    /// 트랙은 유지한다.
    fn correct(&mut self, camera_id: camera::Id, found: Candidate, t: Duration) -> Outcome {
        let Some(index) = self.cameras.iter().position(|p| p.camera_id == camera_id) else {
            return Outcome::Idle;
        };
        let Some(last) = self.last_seen else {
            return Outcome::Idle;
        };
        // 순서 뒤집힌 프레임은 `checked_sub`가 잡는다.
        let Some(dt) = t.checked_sub(last) else {
            return Outcome::Idle;
        };
        if dt >= STALE_GAP {
            self.drop_track();
            return Outcome::Idle;
        }
        self.predict(dt.as_secs_f64());

        let Some((residual, h, s_inv, d2)) = self.innovation(index, found) else {
            self.last_seen = Some(t);
            return Outcome::Idle;
        };
        if d2 > GATE_CHI2 {
            self.rejects += 1;
            self.last_seen = Some(t);
            if self.rejects >= REJECT_LIMIT {
                self.drop_track();
            }
            return Outcome::Rejected { d2 };
        }
        self.rejects = 0;

        let gain = self.p * h.transpose() * s_inv;
        self.x += gain * residual;
        self.p = (Matrix6::identity() - gain * h) * self.p;
        self.p = 0.5 * (self.p + self.p.transpose());
        self.last_seen = Some(t);

        let state = self.state_at(t);
        self.note_recede(&state);
        self.measured.0.push(state);

        if self.predicted.is_empty() && self.trigger.ready(&self.measured) {
            self.predicted = self.integrate_to_robot(t);
        }
        if self.recedes >= RECEDE_LIMIT || outside_volume(self.position()) {
            self.drop_track();
        }
        return Outcome::Accepted;
    }

    /// 트리거 전이면 `None`. `predicted`는 고정이고 `measured`만 자란다.
    ///
    /// `origin`은 밖에서 준다. 필터는 벽시계를 모르고 [`State::t`]만 다룬다.
    pub fn trajectory(&self, origin: std::time::Instant) -> Option<Trajectory> {
        if self.predicted.is_empty() {
            return None;
        }
        return Some(Trajectory {
            seq: self.seq,
            origin,
            measured: self.measured.clone(),
            predicted: self.predicted.clone(),
        });
    }

    /// 트리거 전에도 지금까지의 관측은 볼 수 있다. 툴 전용.
    pub fn measured(&self) -> &Track {
        return &self.measured;
    }

    fn position(&self) -> Point3 {
        return Point3::new(self.x[0], self.x[1], self.x[2]);
    }

    fn velocity(&self) -> Vector3 {
        return Vector3::new(self.x[3], self.x[4], self.x[5]);
    }

    fn state_at(&self, t: Duration) -> State {
        let sigma = |i: usize| self.p[(i, i)].max(0.0).sqrt();
        return State {
            t,
            position: self.position(),
            velocity: self.velocity(),
            sigma_position: Vector3::new(sigma(0), sigma(1), sigma(2)),
            sigma_velocity: Vector3::new(sigma(3), sigma(4), sigma(5)),
            spin: None,
        };
    }

    /// 물리로 상태를 밀고 공분산을 키운다. 물리는 `estimator::Kinematics` SSOT.
    fn predict(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let (position, velocity, _) = Kinematics::step(
            self.position().coords,
            self.velocity(),
            Vector3::zeros(),
            dt,
            &self.physics,
        );
        self.x = Vector6::new(
            position.x, position.y, position.z, velocity.x, velocity.y, velocity.z,
        );

        let mut f = Matrix6::identity();
        let mut q = Matrix6::zeros();
        for axis in 0..3 {
            f[(axis, axis + 3)] = dt;
            q[(axis, axis)] = Q_POSITION * dt;
            q[(axis + 3, axis + 3)] = Q_VELOCITY * dt;
        }
        self.p = f * self.p * f.transpose() + q;
    }

    /// 잔차, 야코비안, `S⁻¹`, 마할라노비스 `d²`. 카메라 뒤면 `None`.
    fn innovation(
        &self,
        camera: usize,
        found: Candidate,
    ) -> Option<(Vector2<f64>, Matrix2x6<f64>, Matrix2<f64>, f64)> {
        let params = &self.cameras[camera];
        let local = params.rotation * self.position().coords + params.translation;
        if local.z <= 0.05 {
            return None;
        }
        let expected = Vector2::new(
            params.fx * local.x / local.z + params.cx,
            params.fy * local.y / local.z + params.cy,
        );
        let residual = Vector2::new(found.pixel.x, found.pixel.y) - expected;

        // ∂pixel/∂camera 에 회전을 곱해 ∂pixel/∂world 로.
        let inv_z = 1.0 / local.z;
        let d_pixel = Matrix2x3::new(
            params.fx * inv_z,
            0.0,
            -params.fx * local.x * inv_z * inv_z,
            0.0,
            params.fy * inv_z,
            -params.fy * local.y * inv_z * inv_z,
        ) * params.rotation;

        let mut h = Matrix2x6::zeros();
        h.fixed_view_mut::<2, 3>(0, 0).copy_from(&d_pixel);

        let s = h * self.p * h.transpose() + Matrix2::identity() * SIGMA_PX.powi(2);
        let s_inv = s.try_inverse()?;
        let d2 = (residual.transpose() * s_inv * residual)[(0, 0)];
        return Some((residual, h, s_inv, d2));
    }

    /// 지금 상태에서 로봇까지 물리로 적분한다. 트리거가 걸렸을 때 딱 한 번 돈다.
    fn integrate_to_robot(&self, t0: Duration) -> Track {
        let step = INTEGRATE_DT.as_secs_f64();
        let mut position = self.position().coords;
        let mut velocity = self.velocity();
        let mut elapsed = Duration::ZERO;
        let mut since_sample = Duration::ZERO;
        let mut out = vec![self.state_at(t0)];

        while elapsed < HORIZON {
            let (next_p, next_v, _) =
                Kinematics::step(position, velocity, Vector3::zeros(), step, &self.physics);
            position = next_p;
            velocity = next_v;
            elapsed += INTEGRATE_DT;
            since_sample += INTEGRATE_DT;

            if since_sample >= SAMPLE_DT {
                since_sample = Duration::ZERO;
                let mut sample = self.state_at(t0 + elapsed);
                sample.position = Point3::from(position);
                sample.velocity = velocity;
                out.push(sample);
            }
            if outside_volume(Point3::from(position)) {
                break;
            }
        }
        return Track(out);
    }

    fn note_recede(&mut self, state: &State) {
        let Some(previous) = self.measured.last() else {
            return;
        };
        if state.position.y > previous.position.y + RECEDE_STEP {
            self.recedes += 1;
        } else {
            self.recedes = 0;
        }
    }

    /// 버리는 조건: 연속 거부 한도, 관측 공백, `y` 재증가(다음 샷), 부피 이탈.
    pub fn drop_track(&mut self) {
        self.x = Vector6::zeros();
        self.p = Matrix6::identity();
        self.measured.0.clear();
        self.predicted.0.clear();
        self.pending.clear();
        self.rejects = 0;
        self.recedes = 0;
        self.last_seen = None;
        self.seq += 1;
    }
}

fn outside_volume(p: Point3) -> bool {
    return p.y < -VOLUME_MARGIN
        || p.y > table::LENGTH_Y + VOLUME_MARGIN
        || p.x < -VOLUME_MARGIN
        || p.x > table::WIDTH_X + VOLUME_MARGIN;
}
