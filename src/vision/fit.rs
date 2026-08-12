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
use crate::defaults::vision::fit::{
    ASSUMED_SPIN, DRAG, FRICTION, HORIZON, HUBER_SIGMA, INTEGRATE_DT, MIN_PIXEL_SPAN, MIN_SIGHTINGS,
    MIN_SPEED, OUTLIER_LIMIT, OUTLIER_SIGMA, RESTITUTION, SAMPLE_DT, SHOOTER_X, SHOOTER_X_SIGMA,
    SIGMA_PX, STALE_GAP, WINDOW,
};
use crate::physics::Kinematics;
use crate::{Point3, Vector3};

use super::contract::{State, Track, Trajectory};
use super::detect::Candidate;
use super::trigger::Trigger;

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
    outliers: u32,
    seq: u64,
    /// 이 트랙의 바운스에서 닫힌식으로 푼 스핀 — 구름 전이가 안 잡히면 계속 `None`이고
    /// `ASSUMED_SPIN`으로 접는다. 바운스는 트랙당 한 번이라 한 번 풀리면 안 다시 푼다.
    solved_spin: Option<Vector3>,
}

impl Fit {
    pub fn new(calibration: &Calibration, trigger: Box<dyn Trigger>) -> Self {
        // TODO(합의): 이 셋을 `PhysicsParams::default()` 로 올리면 sim 도 같이 쓴다.
        // 지금 올리면 jog 테스트가 깨진다 — 기본 슈터 설정이 그 힘을 못 견뎌 접수 창에
        // 도달을 못 한다. 슈터 속도를 같이 올려야 하고 그건 sim 담당 결정이다. 게다가
        // `RESTITUTION`·`FRICTION`·`DRAG`는 `ω=0` 가정을 보정하려고 맞춘 값이라(문서
        // 참고) sim의 진짜 반발 시뮬레이션에 그대로 올리면 안 된다 — 이건 TODO가 아니라
        // 원래 여기 있어야 하는 값이다.
        return Self::with_physics(
            calibration,
            trigger,
            PhysicsParams {
                restitution: RESTITUTION,
                friction: FRICTION,
                drag: DRAG,
                ..PhysicsParams::default()
            },
        );
    }

    /// [`Self::new`]와 같지만 물리 상수를 밖에서 받는다 — e·mu·drag 보정 탐색처럼
    /// `src/defaults`를 고쳐 재빌드하지 않고도 후보를 빠르게 돌려 볼 때 쓴다. 실기·클립
    /// 도구는 항상 [`Self::new`]를 쓴다.
    pub fn with_physics(calibration: &Calibration, trigger: Box<dyn Trigger>, physics: PhysicsParams) -> Self {
        return Self {
            cameras: calibration.cameras.clone(),
            physics,
            trigger,
            sightings: Vec::new(),
            solution: None,
            measured: Track::default(),
            predicted: Track::default(),
            predicting: false,
            outliers: 0,
            seq: 0,
            solved_spin: None,
        };
    }

    /// `false`면 아직 궤적이 안 섰다.
    pub fn has_track(&self) -> bool {
        return self.solution.is_some();
    }

    /// 이 트랙에 쓸 스핀 — 라이브 바운스에서 풀었으면 그걸, 아니면 사전값.
    fn assumed_spin(&self) -> Vector3 {
        return self.solved_spin.unwrap_or(ASSUMED_SPIN);
    }

    /// 지금까지 바운스에서 닫힌식으로 푼 스핀. 안 풀렸으면(구름 전이 안 잡혔거나 아직
    /// 바운스 전이면) `None` — 진단·툴 전용, 소비자는 `assumed_spin`을 통해 간접으로만 쓴다.
    pub fn solved_spin(&self) -> Option<Vector3> {
        return self.solved_spin;
    }

    /// `self.sightings`를 원시 삼각측량 궤적으로 만든다 — 물리 모델이 전혀 안 낀 순수
    /// 기하다.
    ///
    /// **`measured.position`을 안 쓰는 이유**: `measured`는 `walk()`가 (아직 모르는) 가정
    /// 스핀으로 물리를 굴려 만든 값이다 — 바운스 앞은 거의 안 물들지만(스핀은 Magnus로만
    /// 약하게 영향), 바운스 뒤는 그 가정된 반사 방향으로 통째로 굴러가 있다. 스핀을
    /// 풀려는 바로 그 가정이 입력에 이미 섞여 있는 셈이라, 이걸로 v_out을 재면 순환이다
    /// (실측: 이 값 대신 원시 삼각측량을 쓰기 전엔 라이브 발동률이 11%였다 — 오프라인
    /// 전체 클립 스캔은 같은 자리에서 75%가 풀렸다. 격차가 그 순환 오염이었다).
    ///
    /// 카메라가 다른 두 관측을 **상호 최근접**으로 짝지어 삼각측량한다. 도착 순서로
    /// 훑으며 "직전의 다른 카메라 관측"을 짝으로 삼으면 안 된다 — 두 카메라가 프레임을
    /// 번갈아 도착시키므로, 그러면 매번 **한 프레임 전** 관측과 짝지어져(간격이 프레임
    /// 주기보다도 가까워 통과는 하지만 같은 순간이 아니다) 시차가 그대로 위치 오차로
    /// 새어 들어간다 — 처음 이렇게 짰다가 z가 원래 낙하 높이보다 더 높이 발산하는 걸
    /// 보고서야 잡았다. 상호 최근접(서로가 서로의 가장 가까운 짝)만 채택하면 그 오프바이원이
    /// 안 생긴다.
    fn raw_trajectory(&self) -> Vec<crate::physics::TrajPoint> {
        // 카메라 둘은 하드웨어 동기가 없다 — 실측 p95 skew가 18.9ms다(better-vision.md §0).
        // 6ms로 뒀더니 실클립에서 짝이 거의 안 잡혀 라이브 발동률이 안 올랐다 — 그 skew를
        // 덮을 만큼은 넉넉해야 한다.
        const MAX_PAIR_GAP: Duration = Duration::from_millis(20);
        let mut out = Vec::new();
        for anchor in &self.sightings {
            let Some(partner) = self
                .sightings
                .iter()
                .filter(|s| s.camera != anchor.camera)
                .min_by_key(|s| anchor.t.abs_diff(s.t))
            else {
                continue;
            };
            if anchor.t.abs_diff(partner.t) > MAX_PAIR_GAP {
                continue;
            }
            // 상호 최근접 확인 — anchor도 partner 입장에서 가장 가까운 반대 카메라 관측
            // 이어야 한다. 안 그러면 같은 partner가 여러 anchor에 중복으로 물린다.
            let mutual_ok = self
                .sightings
                .iter()
                .filter(|s| s.camera == anchor.camera)
                .min_by_key(|s| partner.t.abs_diff(s.t))
                .is_some_and(|m| m.t == anchor.t);
            if !mutual_ok {
                continue;
            }
            // 상호 최근접은 대칭이라 (anchor, partner)와 (partner, anchor) 양쪽에서 다
            // 채택된다 — 같은 쌍을 두 번 넣지 않게 카메라 인덱스가 작은 쪽에서 볼 때만 낸다.
            if anchor.camera > partner.camera {
                continue;
            }
            let views = [
                (
                    self.cameras[anchor.camera].projection_matrix(),
                    anchor.pixel,
                ),
                (
                    self.cameras[partner.camera].projection_matrix(),
                    partner.pixel,
                ),
            ];
            let Some(point) = Triangulate::views(&views) else {
                continue;
            };
            out.push(crate::physics::TrajPoint {
                t: anchor.t.as_secs_f64(),
                pos: point,
                pixels: Vec::new(),
            });
        }
        return out;
    }

    /// 원시 삼각측량 궤적에서 첫 바운스를 찾아 되튐 후 스핀을 닫힌식으로 푼다. 구름이
    /// 아니면 `None`.
    ///
    /// 후보 자리를 찾는 것과 v_in/v_out을 재는 걸 **같은 창 회귀**로 한다. 처음엔
    /// 인접 2점차로 후보를 찾고 그 자리에서만 창 회귀를 했는데, 라이브에서는(오프라인
    /// 전체 클립 스캔과 달리 표본이 적어) 그 둘이 서로 다른 답을 냈다 — 2점차는 바운스로
    /// 보이는데 같은 자리의 창 회귀는 `v_in.z`가 되레 양수로 나오는 경우가 실측
    /// 383번 중 376번이었다. 인접 2점 잡음 하나에 후보 채택 여부가 갈리면 안 된다.
    ///
    /// 창 폭은 4가 아니라 2다 — 4로 뒀을 때 합성 구름-바운스 테스트에서 22% 어긋났다.
    /// `after` 창의 중앙 시각이 접촉에서 멀수록(120fps 4칸이면 ~21ms) 그새 Magnus가
    /// 이미 v_out을 실측만큼 휘어 놓는다(ω=60 rad/s면 그 21ms에 v_y가 0.27 m/s나
    /// 움직였다) — 이건 잡음이 아니라 실제 물리라 창을 넓힌다고 안 지워진다. 2로
    /// 줄이면 그 지연이 절반(~13ms)으로 줄어 합성 테스트가 15% 안으로 들어온다. 대신
    /// 실클립 잡음 평균 효과는 그만큼 약해진다 — `spin_after_bounce_if_rolling`의 롤
    /// 마진과 `refine_spin`의 `MIN_IMPROVEMENT_RATIO`가 그 잡음을 걸러내는 안전망이다.
    fn solve_spin_from_bounce(&self) -> Option<Vector3> {
        let raw = self.raw_trajectory();
        const HALF_WINDOW: usize = 2;
        if raw.len() <= 2 * HALF_WINDOW {
            return None;
        }
        for index in HALF_WINDOW..=raw.len() - 1 - HALF_WINDOW {
            // 접촉 자체(index)는 창 어느 쪽에도 안 넣는다 — 접히는 순간 자체를 넣으면 그
            // 전이가 양쪽 회귀를 다 끌어당겨 v_in·v_out 둘 다 왜곡된다(실측: v_out.z가
            // 물리상 나올 수 없는 값까지 깎여서 나옴 — e=0.72인데 e≈0.14로 보임).
            let before = &raw[index - HALF_WINDOW..index];
            let after = &raw[index + 1..=index + HALF_WINDOW];
            let Some(v_in) = crate::physics::TrajAnalysis::windowed_velocity(before) else {
                continue;
            };
            let Some(v_out) = crate::physics::TrajAnalysis::windowed_velocity(after) else {
                continue;
            };
            if v_in.z >= -0.25 || v_out.z <= 0.15 {
                continue;
            }
            // 닫힌식 먼저 — 구름 전이면 이걸로 끝난다(대수적 정확해, 반복 없음).
            if let Some(spin) =
                crate::physics::PhysicsIdentify::spin_after_bounce_if_rolling(v_in, v_out, &self.physics)
            {
                return Some(spin);
            }
            // 슬립이면 닫힌식은 원리상 못 푼다(슬립 방향만 보이고 크기는 안 보임,
            // `spin_from_bounce.rs` 문서) — 바운스 뒤 비행의 Magnus로 대신 잡는다.
            return self.refine_spin(&self.solution?);
        }
        return None;
    }

    /// 바운스 뒤 픽셀 잔차로 ω(x, y)를 다시 푼다. `p0`·`v0`는 이미 푼 값을 고정한다.
    ///
    /// `solve_spin_from_bounce`의 닫힌식은 슬립 바운스를 못 푼다 — 접선 임펄스가
    /// `μ·J_n`에 고정돼 슬립 크기(=ω)가 출사 속도에 아예 안 남기 때문이다(대수적으로
    /// 안 보임, 반복을 더 돌려도 안 풀린다). 그런데 바운스 **뒤** 비행에서는 Magnus
    /// (`a += k_m · ω×v`)가 ω 크기에 그대로 걸린다 — 관측이 몇 개만 더 있어도 픽셀
    /// 잔차가 ω를 구속한다. `walk`가 바운스를 물리로 자동으로 통과시키므로 여기서
    /// 바운스 시각을 따로 몰라도 된다 — `p0,v0`에서 ω 후보로 쭉 굴리면 바운스도
    /// 알아서 그 자리에서 일어난다.
    fn refine_spin(&self, start: &Ballistic) -> Option<Vector3> {
        const ITERATIONS: usize = 20;
        const STEP: f64 = 1e-2;
        // 이 리그 실측 최대가 153 rad/s였다(2026-08-12, fly_45~53 클립 반발 역산) — 여유
        // 두 배 두고 300으로 막는다. `diag_physics.rs`의 940은 슈터-비행-전체 적합용
        // 하한이라 여기(바운스 뒤 몇 표본짜리 국소 적합)엔 너무 느슨해서 폭주를 못 막았다
        // (실측: 씨앗 0에서 228까지 뛰어가 버림 — Magnus가 `MAGNUS_OMEGA_MAX=80` 위에서는
        // 클램프돼 그 방향으로 더 가도 안 막히는 평평한 골짜기가 있었다).
        const SPIN_MAX: f64 = 300.0;
        // 씨앗(보통 0) 대비 이만큼도 못 줄이면 못 믿는다 — 관측이 모자라 잡음을 스핀으로
        // 착각한 것일 수 있다. 없는 값보다 나은 값만 내보낸다.
        //
        // 이 문턱을 낮춰서 발동률을 올리고 싶어질 수 있는데, 실측(2026-08-12,
        // fly_45~53, 1010번 호출)으로는 안 통한다 — ratio(=best/seed_rmse)가 가장
        // 좋았던 경우도 0.952(개선 4.8%)였고 40%는 1.000(개선 0, 첫 LM 스텝부터
        // 못 줄임)이었다. 0.7(개선 30%)에 한참 못 미친다. 원인은 버그가 아니라 신호
        // 자체가 약한 것으로 보인다 — Magnus 계수(`magnus≈0.0036`)가 작아서 바운스 뒤
        // 짧은 비행 구간에서 곡률이 픽셀 잡음(`SIGMA_PX`)보다도 작게 실린다. 합성
        // 테스트(`fit_tests.rs`)가 통과하는 건 잡음이 없어서지, 실카메라에서 이 방법이
        // 되는지와는 별개다 — 문턱을 낮추면 잡음을 스핀으로 착각해 "없는 것보다 나쁜"
        // 값을 내보내게 된다.
        const MIN_IMPROVEMENT_RATIO: f64 = 0.7;

        let rmse = |residual: &[f64]| -> f64 {
            return (residual.iter().map(|r| r * r).sum::<f64>() / residual.len().max(1) as f64)
                .sqrt();
        };
        let residuals_at = |spin: nalgebra::Vector2<f64>| -> Option<Vec<f64>> {
            let path = self.walk_with_spin(start, Vector3::new(spin.x, spin.y, 0.0))?;
            return self.residuals_with_spin(start, path);
        };

        let seed = self.assumed_spin();
        let mut spin = nalgebra::Vector2::new(seed.x, seed.y);
        let seeded_rmse = rmse(&residuals_at(spin)?);
        let mut best = seeded_rmse;
        // 감쇠 — 바운스 뒤 관측이 몇 개뿐일 때 순수 가우스-뉴턴은 한 걸음에 과하게
        // 튈 수 있다(`diag_physics.rs`가 바운스 낀 적합에 감쇠를 쓰는 것과 같은 이유,
        // 여기선 p0,v0가 고정이라 그 정도로 안 휘지만 관측이 적을 땐 여전히 보탬이 된다).
        let mut lambda = 1e-3;

        for _ in 0..ITERATIONS {
            let base = residuals_at(spin)?;
            let mut columns: [Vec<f64>; 2] = [Vec::new(), Vec::new()];
            for (axis, column) in columns.iter_mut().enumerate() {
                let mut bumped = spin;
                bumped[axis] += STEP;
                let moved = residuals_at(bumped)?;
                if moved.len() != base.len() {
                    return None;
                }
                *column = moved.iter().zip(&base).map(|(b, a)| (b - a) / STEP).collect();
            }
            let mut normal = nalgebra::Matrix2::<f64>::zeros();
            let mut gradient = nalgebra::Vector2::<f64>::zeros();
            for a in 0..2 {
                for b in 0..2 {
                    normal[(a, b)] = (0..base.len())
                        .map(|i| columns[a][i] * columns[b][i])
                        .sum();
                }
                gradient[a] = (0..base.len()).map(|i| columns[a][i] * base[i]).sum();
            }

            let mut improved = false;
            for _ in 0..8 {
                let mut damped = normal;
                for a in 0..2 {
                    damped[(a, a)] += lambda * (normal[(a, a)].abs() + 1e-9);
                }
                let Some(delta) = damped.try_inverse().map(|inverse| inverse * gradient) else {
                    lambda *= 10.0;
                    continue;
                };
                let mut candidate = spin - delta;
                candidate.x = candidate.x.clamp(-SPIN_MAX, SPIN_MAX);
                candidate.y = candidate.y.clamp(-SPIN_MAX, SPIN_MAX);
                let Some(candidate_residual) = residuals_at(candidate) else {
                    lambda *= 10.0;
                    continue;
                };
                let score = rmse(&candidate_residual);
                if score < best {
                    spin = candidate;
                    best = score;
                    lambda = (lambda * 0.3).max(1e-9);
                    improved = true;
                    break;
                }
                lambda *= 10.0;
            }
            if !improved {
                break;
            }
        }
        if best > seeded_rmse * MIN_IMPROVEMENT_RATIO {
            return None; // 씨앗보다 뚜렷이 낫지 않다 — 뭘 풀었다고 우기지 않는다.
        }
        return Some(Vector3::new(spin.x, spin.y, 0.0));
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
            self.outliers += 1;
            // 연속으로 어긋나면 이건 다른 국면이다. 하나씩 빼면서 버티면 안 된다.
            if self.outliers >= OUTLIER_LIMIT {
                self.drop_track();
                return Outcome::Idle;
            }
            self.solve();
            return Outcome::Rejected { px: residual };
        }
        self.outliers = 0;

        self.refresh();
        if self.finished() {
            // 이미 서 있던 트랙이 끝난 것과, 애초에 샷이 아니었던 것은 다르다. 후자까지
            // seq 를 올리면 라켓에 맞고 돌아가는 공이 매 프레임 트랙 하나를 세웠다 죽인다
            // (실측 fly_10 에서 51번). 소비자는 seq 로 "같은 공인가"를 판단한다.
            if had {
                self.drop_track();
            } else {
                self.abandon();
            }
            return Outcome::Idle;
        }
        return if had {
            Outcome::Accepted
        } else {
            Outcome::Seeded
        };
    }

    /// 샷이 아니었던 적합을 버린다. `seq` 는 그대로 — 선 적이 없으니 끝난 것도 아니다.
    fn abandon(&mut self) {
        self.sightings.clear();
        self.solution = None;
        self.measured.0.clear();
        self.predicted.0.clear();
        self.predicting = false;
        self.outliers = 0;
        self.solved_spin = None;
    }

    /// 서 있던 트랙을 끝낸다: 관측 공백, 되돌아감, 부피 이탈, 멈춤.
    pub fn drop_track(&mut self) {
        self.sightings.clear();
        self.solution = None;
        self.measured.0.clear();
        self.predicted.0.clear();
        self.predicting = false;
        self.outliers = 0;
        self.seq += 1;
        self.solved_spin = None;
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
        // 안 움직이면 샷이 아니다. 3D 를 풀기 전에 화소로 거른다.
        if self.pixel_span() < MIN_PIXEL_SPAN {
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

    /// 카메라 하나 안에서 관측이 이미지 위를 얼마나 움직였나 [px].
    ///
    /// 카메라를 섞으면 시차가 이동으로 둔갑하므로 카메라별로 재고 가장 큰 값을 쓴다.
    fn pixel_span(&self) -> f64 {
        let mut best = 0.0_f64;
        for camera in 0..self.cameras.len() {
            let mut seen: Option<(camera::Pixel, camera::Pixel)> = None;
            for sighting in self.sightings.iter().filter(|s| s.camera == camera) {
                seen = Some(match seen {
                    None => (sighting.pixel, sighting.pixel),
                    Some((first, _)) => (first, sighting.pixel),
                });
            }
            if let Some((first, last)) = seen {
                best = best.max((last - first).norm());
            }
        }
        return best;
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
        //
        // 터키 이중가중(문턱 밖 무게 0)으로 바꿔서 fly_45~53에 실측해 봤다(2026-08-12) —
        // fly_46·50은 나아졌지만 45·48·53은 오히려 나빠졌고 47·48은 트랙이 더 잘게
        // 쪼개졌다(7→10, 6→7). 순증거 없이 뒤섞여서 되돌림 — 원래 fly_49의 75cm 오차를
        // 노리고 바꾼 거였는데, 그건 적합 문제가 아니라 채점 쪽 정답(`observed`)에 낀
        // 확 튄 검출 하나였다(고쳤다: `tools/clip_review/src/track.rs`의 `plausible`).
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
    ///
    /// 마지막 한 행은 슈터 위치 사전값이다. 같은 σ 정규화를 쓰므로 관측 하나와 같은
    /// 무게를 갖는다 — 관측이 몇 개 없을 때만 실질적으로 작용하고, 많아지면 묻힌다.
    fn residuals(&self, start: &Ballistic) -> Option<Vec<f64>> {
        return self.residuals_with_spin(start, self.walk(start)?);
    }

    /// [`Self::residuals`]와 같지만 이미 굴려 둔 경로(`path`)를 받는다 — [`Self::refine_spin`]이
    /// [`Self::walk_with_spin`]으로 후보 ω를 굴린 경로를 그대로 넘기는 자리다.
    fn residuals_with_spin(&self, start: &Ballistic, path: Vec<Point3>) -> Option<Vec<f64>> {
        let mut out = Vec::with_capacity(self.sightings.len() * 2 + 1);
        for (sighting, point) in self.sightings.iter().zip(&path) {
            let params = &self.cameras[sighting.camera];
            let projected = params.project_world_unclipped(*point)?;
            let scale = SIGMA_PX / sigma_px(params);
            out.push((projected.x - sighting.pixel.x) * scale);
            out.push((projected.y - sighting.pixel.y) * scale);
        }
        out.push((start.position.x - SHOOTER_X) / SHOOTER_X_SIGMA * SIGMA_PX);
        return Some(out);
    }

    /// 초기 조건에서 굴리며 관측 시각마다 위치를 뽑는다. 시작 스핀은 사전값
    /// (`assumed_spin`) — 후보 스핀으로 굴려야 하면 [`Self::walk_with_spin`].
    fn walk(&self, start: &Ballistic) -> Option<Vec<Point3>> {
        return self.walk_with_spin(start, self.assumed_spin());
    }

    /// [`Self::walk`]과 같지만 시작 스핀을 밖에서 받는다 — [`Self::refine_spin`]이 후보
    /// ω 를 넣어 가며 재투영 잔차를 재는 자리다.
    ///
    /// ω 는 바운스에서 바뀌므로 들고 다녀야 한다 — 버리면 반발 뒤가 통째로 틀린다.
    fn walk_with_spin(&self, start: &Ballistic, spin0: Vector3) -> Option<Vec<Point3>> {
        let last = self.sightings.last()?.t;
        let step = INTEGRATE_DT.as_secs_f64();
        let (mut position, mut velocity) = (start.position.coords, start.velocity);
        let mut spin = spin0;
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
        let (mut spin, mut elapsed) = (self.assumed_spin(), Duration::ZERO);
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
        if self.solved_spin.is_none() {
            self.solved_spin = self.solve_spin_from_bounce();
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
            (start.position.coords, start.velocity, self.assumed_spin());
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
        let mut spin = from.spin.unwrap_or_else(|| self.assumed_spin());
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

    /// 이 트랙은 끝났나.
    ///
    /// 셋 중 하나면 끝이다 — 부피를 벗어났거나, 로봇에서 멀어지고 있거나, 거의 멈췄거나.
    /// 클립에는 한 샷만 있는 게 아니다. 라켓에 맞고 돌아가는 공, 굴러다니는 공, 놓여 있는
    /// 공이 다 같은 영상에 있고, 그걸 한 탄도로 맞추려 들면 전부 망가진다.
    fn finished(&self) -> bool {
        let Some(state) = self.measured.last() else {
            return false;
        };
        return outside_volume(state.position)
            // 샷은 로봇 쪽(-y)으로 온다. 부호가 뒤집혔으면 맞고 돌아가는 중이다.
            || state.velocity.y >= 0.0
            || state.velocity.norm() < MIN_SPEED;
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
