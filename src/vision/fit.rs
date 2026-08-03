//! 궤적 추정 — 지금까지 본 픽셀 **전부**에 탄도 하나를 맞춘다.
//!
//! 재귀 필터가 아니다. 비행이 0.5 초 40프레임이라 관측을 다 들고 있어도 되고, 미지수는
//! 초기 조건 6개(`p0`, `v0`)뿐이다. 한 프레임이 튀어도 나머지가 잡는다 — 재귀 필터는 그
//! 한 프레임에 상태를 끌려간 뒤 다음 관측을 거부하고, 몇 번 거부하면 트랙을 통째로 버린다.
//!
//! 실측 (클립 9개, 접수 평면 타점 오차, 생 삼각측량 기준):
//!
//! | 방식 | 오차 | 성공 |
//! |---|---|---|
//! | 재귀 EKF | 24.5 cm | 6/9 |
//! | 일괄 적합, 관측 5개 | 16.4 cm | 8/9 |
//! | 일괄 적합, 관측 12개 | 5.7 cm | 8/9 |
//! | 일괄 적합, 타점 전 전부 | 2.9 cm | 8/9 |
//!
//! 관측은 삼각측량한 3D 점이 아니라 픽셀이다. 두 카메라는 하드웨어 동기가 없어 최대
//! 18.9 ms 어긋나는데(실측 p95), 삼각측량은 두 시선이 같은 순간이라고 가정하므로 5 m/s
//! 공이면 9.5 cm 가 틀어진다. 픽셀은 각자 자기 시각에 쓰이니 그 가정이 없고, 한쪽만 본
//! 프레임도 버리지 않는다.

use std::time::{Duration, Instant};

use nalgebra::{Matrix6, Vector6};

use crate::camera::{self, Calibration, Triangulate};
use crate::constants::table;
use crate::defaults::PhysicsParams;
use crate::estimator::Kinematics;
use crate::{Point3, Vector3};

use super::contract::{State, Track, Trajectory};
use super::detect::Candidate;
use super::trigger::Trigger;

/// 검출 자체의 픽셀 노이즈 σ [px].
///
/// 이것만으로 가중치를 정하면 안 된다 — 캘리브가 틀어진 만큼도 그 카메라의 픽셀은 못
/// 믿는다. [`sigma_px`]가 둘을 합친다.
pub const SIGMA_PX: f64 = 1.5;
/// 이보다 크게 어긋난 관측은 창에서 뺀다 [σ 배수].
///
/// 로버스트 손실이 이미 무게를 줄이지만, 완전히 다른 것을 잡은 관측은 아예 빼야 한다.
pub const OUTLIER_SIGMA: f64 = 8.0;
/// 로버스트 손실이 선형으로 꺾이는 지점 [σ 배수].
pub const HUBER_SIGMA: f64 = 3.0;
/// 적합을 시작할 최소 관측 수. 미지수가 6개라 3개(6행)면 딱 맞고, 그건 너무 빠듯하다.
pub const MIN_SIGHTINGS: usize = 6;
/// 창에 들고 있을 시간. 이보다 오래된 관측은 다음 공일 수 있다.
pub const WINDOW: Duration = Duration::from_millis(1500);
/// 관측이 이만큼 끊기면 트랙을 버린다.
pub const STALE_GAP: Duration = Duration::from_millis(500);
/// 공이 멀어졌다고 볼 y 증가량 [m]과 연속 횟수.
pub const RECEDE_STEP: f64 = 0.05;
pub const RECEDE_LIMIT: u32 = 3;
/// 적분 스텝과 예측 궤적 표본 간격.
pub const INTEGRATE_DT: Duration = Duration::from_millis(1);
pub const SAMPLE_DT: Duration = Duration::from_millis(5);
/// 예측 궤적 상한.
pub const HORIZON: Duration = Duration::from_secs(2);
/// 항력계수 `k` [1/m] — `a = -k·|v|·v`. 지금은 **끈다**.
///
/// 교과서 값(`½ρC_dA/m` 에 `C_d = 0.45` → 0.126)을 넣으면 예측이 **2배 나빠진다**.
/// 클립 9개, 리드타임별 타점 오차 [cm]:
///
/// | drag | spin | 0.4s | 0.3s | 0.2s | 0.1s |
/// |---|---|---|---|---|---|
/// | 0 | 0 | 8.4 | 7.4 | 6.9 | 5.6 |
/// | **0.126** | 0 | 15.7 | 12.9 | 13.4 | 12.4 |
/// | 0 | **210** | **7.6** | **5.0** | **5.3** | **4.1** |
/// | 0.126 | 210 | 9.2 | 7.2 | 7.4 | 6.2 |
///
/// 데이터도 같은 말을 한다 — `diag_physics` 에서 ω 를 같이 풀면 항력이 0.013 으로
/// 주저앉는다. 이 속도·거리대에서는 항력이 잴 만큼 안 실린다는 뜻이다. 없는 힘을 넣고
/// 초기 속도로 벌충하면 적합 구간 안에서만 맞고 그 너머로 갈수록 벌어진다.
///
/// TODO(측정): 이론과 데이터가 어긋나므로 전용 측정이 필요하다. **바운스 없는 긴 비행**을
/// ω 를 아는 상태로 찍어야 한다 — 바운스가 끼면 반발계수 오차가 항력으로 새고
/// (바운스 전만 보면 0.150 이 나온다), ω 를 모르면 Magnus 가 항력 자리로 샌다.
pub const DRAG: f64 = 0.0;

/// 슈터가 먹이는 스핀 [rad/s]. `+x` 축 회전이 진행 방향(`-y`) 기준 톱스핀이다.
///
/// 클립 9개에서 잰 값이다 (`diag_physics`): ω_x 137~251, 중앙값 210, 부호는 전부 같다.
/// 넣으면 리드 0.3 s 타점 오차가 7.4 -> 5.0 cm 로, 0.1 s 는 5.6 -> 4.1 cm 로 준다.
///
/// 효과는 거의 **바운스**에서 온다. `MAGNUS_OMEGA_MAX` 가 80 rad/s 라 비행 중 Magnus 에는
/// 이 값의 38 % 만 실리는데, 바운스 접선 임펄스는 클립을 안 쓰고 210 을 그대로 쓴다.
/// 같은 ω 를 두 곳에서 다르게 쓰는 셈이라 그 상한도 재검토 대상이다 (sim 거동도 바뀐다).
///
/// TODO(측정): 슈터 눈금이 바뀌면 이 값도 바뀐다. 지금은 클립을 찍을 때의 한 설정에서만
/// 맞다. 눈금-스핀 대응표를 만들거나(계획 문서 Stage 5), 관측이 충분할 때 ω 를 미지수로
/// 풀어 이 상수를 사전값으로만 쓰는 게 다음이다.
///
/// ω_y·ω_z 는 0 으로 둔다. 클립에서 흩어졌고, 특히 ω_z 는 바운스 접촉점 속도
/// `(v_x - Rω_y, v_y + Rω_x, 0)` 에 아예 안 들어가 관측되지 않는다.
pub const ASSUMED_SPIN: Vector3 = Vector3::new(210.0, 0.0, 0.0);

/// 트랙을 유지할 부피 여유 [m].
const VOLUME_MARGIN: f64 = 1.0;
/// 예측을 끊을 로봇 쪽 y [m]. 그 뒤는 이미 지나친 자리다.
const PREDICT_UNTIL_Y: f64 = -0.2;
/// 가우스-뉴턴 반복. 이전 해에서 이어 풀므로 몇 번이면 된다.
const ITERATIONS: usize = 6;
/// 처음 풀 때만 더 돈다 — 초기값이 삼각측량 두 점이라 멀다.
const ITERATIONS_COLD: usize = 20;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    /// 이 관측으로 처음 궤적이 섰다.
    Seeded,
    Accepted,
    /// 적합과 너무 어긋나 창에서 뺐다 [px].
    Rejected {
        px: f64,
    },
    /// 아직 관측이 모자라거나 쓸 수 없는 관측이라 아무것도 안 했다.
    Idle,
}

/// 한 카메라가 한 순간에 본 것.
#[derive(Debug, Clone, Copy)]
struct Sighting {
    camera: usize,
    pixel: camera::Pixel,
    t: Duration,
}

/// 탄도 하나의 초기 조건. `t0` 기준이다.
#[derive(Debug, Clone, Copy)]
struct Ballistic {
    t0: Duration,
    position: Point3,
    velocity: Vector3,
}

/// 공 하나의 추정.
pub struct Fit {
    cameras: Vec<camera::Params>,
    physics: PhysicsParams,
    trigger: Box<dyn Trigger>,
    /// 지금 공에 대해 본 것 전부. 적합의 입력이다.
    sightings: Vec<Sighting>,
    solution: Option<Ballistic>,
    /// 적합된 궤적을 관측 시각마다 표본한 것.
    measured: Track,
    /// 트리거 뒤 **매 관측마다** 다시 적분한다.
    predicted: Track,
    /// 트리거는 걸쇠다 — 한 번 걸리면 트랙이 끝날 때까지 안 풀린다.
    predicting: bool,
    recedes: u32,
    seq: u64,
}

impl Fit {
    pub fn new(calibration: &Calibration, trigger: Box<dyn Trigger>) -> Self {
        return Self {
            cameras: calibration.cameras.clone(),
            // TODO(합의): 이 둘을 `PhysicsParams::default()` 로 올리면 sim 도 같이 쓴다.
            // 지금 올리면 jog 테스트 4개가 깨진다 — 기본 슈터 설정이 항력을 못 견뎌 접수
            // 창에 도달을 못 한다. 슈터 속도를 같이 올려야 하고 그건 sim 담당 결정이다.
            physics: PhysicsParams {
                drag: DRAG,
                ..PhysicsParams::default()
            },
            trigger,
            sightings: Vec::new(),
            solution: None,
            measured: Track::default(),
            predicted: Track::default(),
            predicting: false,
            recedes: 0,
            seq: 0,
        };
    }

    /// `false`면 아직 궤적이 안 섰다.
    pub fn has_track(&self) -> bool {
        return self.solution.is_some();
    }

    pub fn seq(&self) -> u64 {
        return self.seq;
    }

    /// 적합에 실제로 쓰이고 있는 관측 수. 진단용.
    pub fn sightings(&self) -> usize {
        return self.sightings.len();
    }

    /// 지금까지의 궤적. 트리거 전에도 볼 수 있다.
    pub fn measured(&self) -> &Track {
        return &self.measured;
    }

    /// 트리거 전이면 `None`. 둘 다 매 관측마다 갱신된다.
    ///
    /// `origin`은 밖에서 준다. 추정기는 벽시계를 모르고 [`State::t`]만 다룬다.
    pub fn trajectory(&self, origin: Instant) -> Option<Trajectory> {
        if !self.predicting || self.predicted.is_empty() {
            return None;
        }
        return Some(Trajectory {
            seq: self.seq,
            origin,
            measured: self.measured.clone(),
            predicted: self.predicted.clone(),
        });
    }

    /// 프레임 하나를 먹인다. 그 프레임에서 못 찾았으면 `found` 가 `None` 이다.
    ///
    /// 못 찾은 프레임도 넘겨야 한다 — 공이 화면을 떠난 걸 아는 유일한 신호가 "그 뒤로
    /// 아무것도 안 온다"이기 때문이다. 검출된 것만 넘기면 트랙이 영영 안 죽고, 제어는
    /// 지나간 공의 예측을 계속 받는다.
    pub fn observe(
        &mut self,
        camera_id: camera::Id,
        found: Option<Candidate>,
        t: Duration,
    ) -> Outcome {
        // 순서 뒤집힌 프레임은 여기서 걸린다.
        if self.sightings.last().is_some_and(|last| t < last.t) {
            return Outcome::Idle;
        }
        if let Some(last) = self.sightings.last()
            && t.saturating_sub(last.t) >= STALE_GAP
        {
            self.drop_track();
        }
        let Some(candidate) = found else {
            return Outcome::Idle;
        };
        let Some(camera) = self.cameras.iter().position(|p| p.camera_id == camera_id) else {
            return Outcome::Idle;
        };

        self.sightings.push(Sighting {
            camera,
            pixel: candidate.pixel,
            t,
        });
        self.sightings.retain(|s| t.saturating_sub(s.t) <= WINDOW);

        let had = self.solution.is_some();
        if !self.solve() {
            return Outcome::Idle;
        }

        // 새 관측이 적합과 얼마나 어긋나나. 크면 창에서 빼고 다시 푼다.
        let residual = self
            .reprojection_px(self.sightings.len() - 1)
            .unwrap_or(f64::INFINITY);
        let limit = OUTLIER_SIGMA * sigma_px(&self.cameras[camera]);
        if residual > limit {
            self.sightings.pop();
            self.solve();
            return Outcome::Rejected { px: residual };
        }

        self.refresh();
        if self.recedes >= RECEDE_LIMIT || self.outside_volume() {
            self.drop_track();
            return Outcome::Idle;
        }
        return if had {
            Outcome::Accepted
        } else {
            Outcome::Seeded
        };
    }

    /// 트랙을 버린다: 관측 공백, `y` 재증가(다음 샷), 부피 이탈.
    pub fn drop_track(&mut self) {
        self.sightings.clear();
        self.solution = None;
        self.measured.0.clear();
        self.predicted.0.clear();
        self.predicting = false;
        self.recedes = 0;
        self.seq += 1;
    }

    /// 관측 전부에 탄도를 맞춘다. 이전 해가 있으면 거기서 이어 푼다.
    fn solve(&mut self) -> bool {
        if self.sightings.len() < MIN_SIGHTINGS {
            return false;
        }
        // 서로 다른 카메라가 최소 둘은 있어야 깊이가 잡힌다.
        let first = self.sightings[0].camera;
        if self.sightings.iter().all(|s| s.camera == first) {
            return false;
        }
        let t0 = self.sightings[0].t;
        let (mut start, iterations) = match self.solution {
            // 창이 밀려 t0 가 바뀌었을 수 있으니 그 시각의 상태로 옮겨 앉는다.
            Some(previous) => (self.advance(previous, t0), ITERATIONS),
            None => match self.bootstrap(t0) {
                Some(guess) => (guess, ITERATIONS_COLD),
                None => return false,
            },
        };

        for _ in 0..iterations {
            let Some(delta) = self.gauss_newton_step(&start) else {
                break;
            };
            for axis in 0..3 {
                start.position[axis] -= delta[axis];
                start.velocity[axis] -= delta[axis + 3];
            }
            if delta.norm() < 1e-9 {
                break;
            }
        }
        if !start.position.coords.iter().all(|c| c.is_finite())
            || !start.velocity.iter().all(|c| c.is_finite())
        {
            self.solution = None;
            return false;
        }
        self.solution = Some(start);
        return true;
    }

    /// 초기값 — 앞뒤 두 시각을 각각 삼각측량해 위치와 속도를 잡는다.
    fn bootstrap(&self, t0: Duration) -> Option<Ballistic> {
        let early = self.triangulate_near(t0)?;
        let last = self.sightings.last()?.t;
        let late = self.triangulate_near(last)?;
        let span = last.saturating_sub(t0).as_secs_f64();
        let velocity = if span > 1e-6 {
            (late - early) / span
        } else {
            Vector3::zeros()
        };
        return Some(Ballistic {
            t0,
            position: early,
            velocity,
        });
    }

    /// `at` 에 가장 가까운 두 카메라 관측을 짝지어 삼각측량한다. 초기값 전용이라 거칠어도 된다.
    fn triangulate_near(&self, at: Duration) -> Option<Point3> {
        let mut best: Vec<(usize, camera::Pixel, u128)> = Vec::new();
        for sighting in &self.sightings {
            let gap = at.abs_diff(sighting.t).as_micros();
            match best.iter_mut().find(|(c, _, _)| *c == sighting.camera) {
                Some(slot) if gap < slot.2 => *slot = (sighting.camera, sighting.pixel, gap),
                Some(_) => {}
                None => best.push((sighting.camera, sighting.pixel, gap)),
            }
        }
        if best.len() < 2 {
            return None;
        }
        let views: Vec<_> = best
            .iter()
            .map(|(camera, pixel, _)| (self.cameras[*camera].projection_matrix(), *pixel))
            .collect();
        return Triangulate::views(&views);
    }

    /// 정규방정식 한 걸음. 야코비안은 수치 미분이다 — 미지수가 6개뿐이라 유도할 이유가 없고,
    /// 적분기를 그대로 쓰므로 예측과 **같은 물리**를 푼다.
    fn gauss_newton_step(&self, start: &Ballistic) -> Option<Vector6<f64>> {
        const STEP: f64 = 1e-4;
        let base = self.residuals(start)?;
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(6);
        for k in 0..6 {
            let mut bumped = *start;
            if k < 3 {
                bumped.position[k] += STEP;
            } else {
                bumped.velocity[k - 3] += STEP;
            }
            let moved = self.residuals(&bumped)?;
            if moved.len() != base.len() {
                return None;
            }
            columns.push(
                moved
                    .iter()
                    .zip(&base)
                    .map(|(b, a)| (b - a) / STEP)
                    .collect(),
            );
        }

        // 후버 가중 — 크게 어긋난 행의 무게를 1/|r| 로 줄인다. 하나가 다 끌고 가지 않게.
        let weights: Vec<f64> = base
            .iter()
            .map(|r| {
                let limit = HUBER_SIGMA * SIGMA_PX;
                return if r.abs() <= limit {
                    1.0
                } else {
                    limit / r.abs()
                };
            })
            .collect();

        let mut normal = Matrix6::<f64>::zeros();
        let mut gradient = Vector6::<f64>::zeros();
        for a in 0..6 {
            for b in 0..6 {
                normal[(a, b)] = (0..base.len())
                    .map(|i| weights[i] * columns[a][i] * columns[b][i])
                    .sum();
            }
            gradient[a] = (0..base.len())
                .map(|i| weights[i] * columns[a][i] * base[i])
                .sum();
        }
        // 리지 항으로 특이해를 막는다 — 한 카메라만 보이는 구간에서 깊이가 안 잡힌다.
        for a in 0..6 {
            normal[(a, a)] += 1e-6;
        }
        return normal.try_inverse().map(|inverse| inverse * gradient);
    }

    /// 관측마다 재투영 오차 2행 (x, y), σ 로 정규화 [px 단위].
    fn residuals(&self, start: &Ballistic) -> Option<Vec<f64>> {
        let path = self.walk(start)?;
        let mut out = Vec::with_capacity(self.sightings.len() * 2);
        for (sighting, point) in self.sightings.iter().zip(&path) {
            let params = &self.cameras[sighting.camera];
            let projected = params.project_world_unclipped(*point)?;
            let scale = SIGMA_PX / sigma_px(params);
            out.push((projected.x - sighting.pixel.x) * scale);
            out.push((projected.y - sighting.pixel.y) * scale);
        }
        return Some(out);
    }

    /// 초기 조건에서 굴리며 관측 시각마다 위치를 뽑는다.
    ///
    /// ω 는 바운스에서 바뀌므로 들고 다녀야 한다 — 버리면 반발 뒤가 통째로 틀린다.
    fn walk(&self, start: &Ballistic) -> Option<Vec<Point3>> {
        let last = self.sightings.last()?.t;
        let step = INTEGRATE_DT.as_secs_f64();
        let (mut position, mut velocity) = (start.position.coords, start.velocity);
        let mut spin = ASSUMED_SPIN;
        let (mut elapsed, mut next) = (Duration::ZERO, 0usize);
        let mut out = Vec::with_capacity(self.sightings.len());
        let horizon = last.saturating_sub(start.t0);

        loop {
            while next < self.sightings.len()
                && self.sightings[next].t.saturating_sub(start.t0) <= elapsed
            {
                out.push(Point3::from(position));
                next += 1;
            }
            if next >= self.sightings.len() || elapsed > horizon + INTEGRATE_DT {
                break;
            }
            let (p, v, w) = Kinematics::step(position, velocity, spin, step, &self.physics);
            position = p;
            velocity = v;
            spin = w;
            elapsed += INTEGRATE_DT;
        }
        // 시각이 창 끝을 넘는 관측이 남으면 마지막 위치로 채운다 (적합이 못 미치는 구간).
        while out.len() < self.sightings.len() {
            out.push(Point3::from(position));
        }
        return Some(out);
    }

    /// 같은 탄도를 다른 기준 시각으로 옮겨 앉힌다.
    fn advance(&self, ballistic: Ballistic, t0: Duration) -> Ballistic {
        let Some(dt) = t0.checked_sub(ballistic.t0) else {
            return Ballistic { t0, ..ballistic };
        };
        let step = INTEGRATE_DT.as_secs_f64();
        let (mut position, mut velocity) = (ballistic.position.coords, ballistic.velocity);
        let (mut spin, mut elapsed) = (ASSUMED_SPIN, Duration::ZERO);
        while elapsed < dt {
            let (p, v, w) = Kinematics::step(position, velocity, spin, step, &self.physics);
            position = p;
            velocity = v;
            spin = w;
            elapsed += INTEGRATE_DT;
        }
        return Ballistic {
            t0,
            position: Point3::from(position),
            velocity,
        };
    }

    /// 관측 `index` 의 재투영 오차 [px].
    fn reprojection_px(&self, index: usize) -> Option<f64> {
        let start = self.solution?;
        let path = self.walk(&start)?;
        let sighting = self.sightings.get(index)?;
        let projected = self.cameras[sighting.camera].project_world_unclipped(*path.get(index)?)?;
        return Some((projected - sighting.pixel).norm());
    }

    /// 적합 결과로 `measured`·`predicted` 를 다시 만든다.
    fn refresh(&mut self) {
        let Some(start) = self.solution else {
            return;
        };
        let Some(path) = self.walk(&start) else {
            return;
        };
        let sigma = self.parameter_sigma(&start);

        let previous_y = self.measured.last().map(|s| s.position.y);
        self.measured = Track(
            self.sightings
                .iter()
                .zip(&path)
                .map(|(sighting, position)| {
                    let lead = sighting.t.saturating_sub(start.t0).as_secs_f64();
                    let (velocity, spin) = self.state_after(&start, lead);
                    return State {
                        t: sighting.t,
                        position: *position,
                        velocity,
                        sigma_position: sigma.0 + sigma.1 * lead,
                        sigma_velocity: sigma.1,
                        // 추정한 게 아니라 상수를 굴린 값이다. 소비자가 그걸 알아야 한다.
                        spin: Some(spin),
                    };
                })
                .collect(),
        );
        if let (Some(before), Some(now)) = (previous_y, self.measured.last().map(|s| s.position.y))
        {
            if now > before + RECEDE_STEP {
                self.recedes += 1;
            } else {
                self.recedes = 0;
            }
        }

        self.predicting |= self.trigger.ready(&self.measured);
        if self.predicting
            && let Some(last) = self.measured.last().copied()
        {
            self.predicted = self.integrate_to_robot(&last);
        }
    }

    /// `lead` 초 뒤의 속도와 스핀. 바운스를 지나면 둘 다 바뀐다.
    fn state_after(&self, start: &Ballistic, lead: f64) -> (Vector3, Vector3) {
        let step = INTEGRATE_DT.as_secs_f64();
        let (mut position, mut velocity, mut spin) =
            (start.position.coords, start.velocity, ASSUMED_SPIN);
        let mut elapsed = 0.0;
        while elapsed < lead {
            let (p, v, w) = Kinematics::step(position, velocity, spin, step, &self.physics);
            position = p;
            velocity = v;
            spin = w;
            elapsed += step;
        }
        return (velocity, spin);
    }

    /// 초기 조건의 축별 σ — `(위치, 속도)`.
    ///
    /// 정규방정식 `(JᵀJ)⁻¹ σ_px²` 의 대각. 적합이 데이터에서 직접 낸 값이라 "필터가 얼마나
    /// 확신하나"를 손으로 튜닝하지 않는다.
    fn parameter_sigma(&self, start: &Ballistic) -> (Vector3, Vector3) {
        let fallback = (Vector3::repeat(0.5), Vector3::repeat(5.0));
        let Some(base) = self.residuals(start) else {
            return fallback;
        };
        const STEP: f64 = 1e-4;
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(6);
        for k in 0..6 {
            let mut bumped = *start;
            if k < 3 {
                bumped.position[k] += STEP;
            } else {
                bumped.velocity[k - 3] += STEP;
            }
            let Some(moved) = self.residuals(&bumped) else {
                return fallback;
            };
            columns.push(
                moved
                    .iter()
                    .zip(&base)
                    .map(|(b, a)| (b - a) / STEP)
                    .collect(),
            );
        }
        let mut normal = Matrix6::<f64>::zeros();
        for a in 0..6 {
            for b in 0..6 {
                normal[(a, b)] = (0..base.len()).map(|i| columns[a][i] * columns[b][i]).sum();
            }
            normal[(a, a)] += 1e-6;
        }
        let Some(covariance): Option<Matrix6<f64>> = normal.try_inverse() else {
            return fallback;
        };
        let sigma = |i: usize| -> f64 { (covariance[(i, i)] * SIGMA_PX.powi(2)).max(0.0).sqrt() };
        return (
            Vector3::new(sigma(0), sigma(1), sigma(2)),
            Vector3::new(sigma(3), sigma(4), sigma(5)),
        );
    }

    /// 지금 상태에서 로봇까지 물리로 적분한다.
    fn integrate_to_robot(&self, from: &State) -> Track {
        let step = INTEGRATE_DT.as_secs_f64();
        let (mut position, mut velocity) = (from.position.coords, from.velocity);
        let mut spin = from.spin.unwrap_or(ASSUMED_SPIN);
        let (mut elapsed, mut since_sample) = (Duration::ZERO, Duration::ZERO);
        let mut out = vec![*from];

        while elapsed < HORIZON {
            let (p, v, w) = Kinematics::step(position, velocity, spin, step, &self.physics);
            position = p;
            velocity = v;
            spin = w;
            elapsed += INTEGRATE_DT;
            since_sample += INTEGRATE_DT;

            if since_sample >= SAMPLE_DT {
                since_sample = Duration::ZERO;
                out.push(State {
                    t: from.t + elapsed,
                    position: Point3::from(position),
                    velocity,
                    sigma_position: from.sigma_position
                        + from.sigma_velocity * elapsed.as_secs_f64(),
                    sigma_velocity: from.sigma_velocity,
                    spin: None,
                });
            }
            // 로봇을 지났거나 옆으로 빠졌으면 끝. 뒤는 칠 수 없는 자리다.
            if position.y < PREDICT_UNTIL_Y || outside_volume(Point3::from(position)) {
                break;
            }
        }
        return Track(out);
    }

    fn outside_volume(&self) -> bool {
        return self
            .measured
            .last()
            .is_some_and(|state| outside_volume(state.position));
    }
}

/// 이 카메라의 픽셀을 얼마나 믿을 것인가 [px].
///
/// 검출 노이즈와 캘리브 재투영 오차를 독립으로 보고 합친다. 캘리브 오차는 사실 계통
/// 편향이라 진짜 노이즈는 아니지만, 적합에 편향을 넣을 자리가 없으므로 가중치가 문다.
///
/// 이걸 안 하면 캘리브가 나쁜 카메라가 통째로 밀려난다 (실측 cam0 rmse 4.15 px /
/// cam1 1.35 px).
fn sigma_px(params: &camera::Params) -> f64 {
    return SIGMA_PX.hypot(params.reprojection_rmse_px.unwrap_or(0.0).max(0.0));
}

fn outside_volume(p: Point3) -> bool {
    return p.y < -VOLUME_MARGIN
        || p.y > table::LENGTH_Y + VOLUME_MARGIN
        || p.x < -VOLUME_MARGIN
        || p.x > table::WIDTH_X + VOLUME_MARGIN;
}

#[cfg(test)]
#[path = "fit_tests.rs"]
mod tests;
