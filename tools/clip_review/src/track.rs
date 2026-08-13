//! 클립 한 번 훑기 — [`Vision`]에 프레임을 먹여 프레임별 상태를 남긴다.
//!
//! 두 AVI는 동시 녹화라 같은 인덱스가 같은 시각이다. `OpenCvCapture`의 타임스탬프는
//! 읽은 시각이라 쓸 수 없으므로 `인덱스 / fps`로 다시 찍는다.
//!
//! 두 번 훑지 않는다 — 검출은 여기서 한 번만 하고, 재생 루프는 그 결과를 다시 쓴다.
//! 되감아도 검출이 다시 돌지 않으므로 0.1x로 앞뒤를 오가도 값이 흔들리지 않는다.

use std::path::Path;
use std::time::{Duration, Instant};

use pingpong_bot::{Point3, Vector3};
use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture, Triangulate};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::defaults::vision::seed::MAX_REPROJECTION_PX;
use pingpong_bot::vision::{Outcome, State, Track, Trajectory, Vision};

/// 로봇 마운트 y [m] — 궤적을 그릴 하한.
///
/// 그 뒤는 로봇이 이미 지나친 자리라 볼 이유가 없고, 길게 그려 두면 화면만 어지럽다.
/// 예측은 여전히 그 너머까지 적분한다(바운스·평면 통과 계산에 필요하다) — **그리기만**
/// 여기서 자른다.
pub fn draw_limit_y() -> f64 {
    return defaults::rail_frame().mount_y();
}

/// `y >= draw_limit_y()` 구간만 남긴다. 경계를 지나는 선분은 그 지점에서 끊는다.
pub fn clip_to_mount(points: &[Point3]) -> Vec<Point3> {
    let limit = draw_limit_y();
    let mut out = Vec::with_capacity(points.len());
    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if a.y >= limit {
            out.push(a);
        }
        // 경계를 가로지르면 정확히 그 점까지만 그린다.
        if (a.y >= limit) != (b.y >= limit) {
            let span = a.y - b.y;
            if span.abs() > f64::EPSILON {
                out.push(a.lerp(&b, (a.y - limit) / span));
            }
            break;
        }
    }
    if let Some(last) = points.last()
        && last.y >= limit
        && points.len() > 1
    {
        out.push(*last);
    }
    if points.len() == 1 && points[0].y >= limit {
        out.push(points[0]);
    }
    return out;
}

/// 한 프레임에서 두 캠이 같이 잡아 복원한 점 — **필터를 안 거친** 기준값.
///
/// 필터가 뭉갠 건지 입력이 이미 튄 건지를 가르는 유일한 근거라 EKF와 별개로 유지한다.
#[derive(Debug, Clone, Copy)]
pub struct Observed {
    pub frame: usize,
    pub t: f64,
    pub point: Point3,
    /// 복원한 점을 두 캠에 되쏜 최대 오차 [px].
    pub reprojection_px: f64,
}

/// 프레임 하나의 재생 상태 — 창이 그릴 것 전부.
#[derive(Debug, Clone, Default)]
pub struct FrameState {
    /// 캠별 검출 픽셀 (`None` = 못 찾음).
    pub pixels: [Option<camera::Pixel>; 2],
    /// 이 프레임의 삼각측량 (둘 다 잡고 재투영 게이트를 통과했을 때만).
    pub observed: Option<Observed>,
    /// 이 프레임까지 필터가 낸 상태 — HUD·콘솔의 숫자용.
    ///
    /// 그리는 선은 여기서 뽑지 않는다. 트리거 전에는 계약이 없어 제어가 아무것도 못 받는데
    /// 화면에만 선이 있으면 거짓말이 된다. 선은 [`Contract`]에서만 나온다.
    pub filtered: Option<State>,
    /// 캠별 필터 판정. 어느 쪽 검출이 거부됐는지 봐야 진단이 된다.
    pub outcomes: [Option<Outcome>; 2],
    /// 적합 상태를 이 카메라로 되쏘았을 때 검출 픽셀과 벌어진 거리 [px].
    ///
    /// 모델이 데이터를 설명하는가의 직접 지표다. 크면 물리 모델이 틀렸거나 캘리브가
    /// 틀렸거나 다른 것을 잡은 것이고, 셋 다 σ 로는 안 보인다.
    pub residual_px: [Option<f64>; 2],
    /// 이 프레임에 적합이 쓰고 있던 관측 수. 오차의 지배 항이다.
    pub sightings: usize,
    /// 두 캠이 같은 순간에 함께 본 표본 수 — 트리거가 보는 바로 그 값.
    pub stereo_samples: usize,
    /// 바운스에서 닫힌식으로 푼 스핀. `None`이면 `ASSUMED_SPIN`(=0)으로 굴렸다는 뜻 —
    /// 그 샷의 튐 반사는 무회전 가정이라 y·z가 어긋난다.
    pub solved_spin: Option<Vector3>,
    /// 이 프레임의 **살아 있는** 예측이 접수 평면에서 찍은 점.
    ///
    /// 얼린 예측은 한 점밖에 안 주므로 "리드가 줄면 나아지나"를 못 본다. 계약이 매
    /// 관측마다 갱신되니 프레임마다 이 한 점만 남겨 두면 그 곡선이 나온다.
    pub predicted_impact: Option<Point3>,
    /// "같은 공인가"의 근거. 트랙을 버리면 올라간다.
    pub seq: u64,
    pub tracking: bool,
}

/// 제어 쪽으로 나가는 그 객체.
///
/// 화면의 선은 **전부 여기서** 나온다. 툴이 따로 모은 상태로 그리면 계약이 실제로 무엇을
/// 담고 있는지가 아니라 툴이 무엇을 봤는지를 보게 된다.
#[derive(Debug, Clone)]
pub struct Contract {
    /// 트리거가 걸린 프레임 — 실기라면 여기서 제어로 넘어갔다.
    pub frame: usize,
    pub t: f64,
    /// 트리거가 걸린 그 순간의 계약 사본.
    ///
    /// 필터는 이제 매 보정마다 예측을 다시 적분하므로 계약의 `predicted`는 계속 바뀐다.
    /// 그건 제어에게 맞는 동작이지만("지금 아는 최선"), 그것만 보면 예측이 실제에 붙어
    /// 보여 "예측이 맞았나"를 못 본다. 그래서 툴이 첫 순간을 따로 잡는다.
    pub at_trigger: Trajectory,
    /// 마지막으로 본 계약. `measured`가 여기까지 자라 있다.
    pub latest: Trajectory,
}

/// 클립 전체 — 창이 필요로 하는 모든 것.
pub struct Reviewed {
    pub frames: Vec<FrameState>,
    /// 클립 전체의 생 궤적 (재생 위치와 무관하게 이미 다 안다).
    pub observed: Vec<Observed>,
    /// 제어로 나간 계약. 트리거를 못 넘었으면 `None`.
    pub contract: Option<Contract>,
    pub fps: f64,
}

impl Reviewed {
    pub fn len(&self) -> usize {
        return self.frames.len();
    }

    /// 프레임 인덱스 → 클립 시작 기준 시각 [s].
    pub fn time_of(&self, frame: usize) -> f64 {
        return frame as f64 / self.fps;
    }

    /// 생 궤적을 `frame` 까지. 아직 안 온 구간은 안 그린다 — 재생인데 미래가 미리 보이면
    /// 무엇이 "지금 아는 것"인지 구분이 안 된다.
    pub fn observed_to(&self, frame: usize) -> Vec<Point3> {
        let cut = self.observed.partition_point(|o| o.frame <= frame);
        return self.observed[..cut].iter().map(|o| o.point).collect();
    }

    /// 재생을 끝낼 프레임 — 추적하던 샷이 끝나는 자리.
    ///
    /// 클립은 라켓에 맞고 되돌아가는 것까지 녹화돼 있는데 그 뒤는 볼 게 없다. 계약이 선
    /// 트랙(`seq`)이 살아 있는 마지막 프레임까지만 튼다. 계약이 없으면 클립 끝까지.
    pub fn last_frame(&self) -> usize {
        let end = self.frames.len().saturating_sub(1);
        let Some(contract) = &self.contract else {
            return end;
        };
        return self.frames[..=end.min(self.frames.len() - 1)]
            .iter()
            .enumerate()
            .filter(|(index, state)| {
                *index >= contract.frame && state.seq == contract.latest.seq && state.tracking
            })
            .map(|(index, _)| index)
            .next_back()
            .unwrap_or(end);
    }

    /// 계약의 `measured`를 `frame` 시각까지 자른 것.
    ///
    /// 자르는 이유는 재생 때문이다. `measured`는 클립 끝까지 자란 상태로 들고 있는데 그걸
    /// 통째로 그리면 아직 안 온 구간까지 보인다.
    pub fn measured_to(&self, frame: usize) -> Vec<Point3> {
        let Some(contract) = &self.contract else {
            return Vec::new();
        };
        let now = Duration::from_secs_f64(self.time_of(frame).max(0.0));
        return contract
            .latest
            .measured
            .iter()
            .take_while(|state| state.t <= now)
            .map(|state| state.position)
            .collect();
    }

    /// 계약의 `predicted`. 트리거 순간에 얼었으므로 재생 위치와 무관하다.
    pub fn predicted(&self) -> &[State] {
        return self
            .contract
            .as_ref()
            .map_or(&[], |contract| &contract.at_trigger.predicted);
    }

    /// 생 궤적이 `y` 평면을 로봇 쪽으로 지난 지점 (표본 사이 선형 보간).
    ///
    /// 커밋 예측의 타점과 비교할 **정답**이다.
    pub fn observed_crossing_y(&self, y: f64) -> Option<Point3> {
        let pair = self
            .observed
            .windows(2)
            .find(|w| w[0].point.y >= y && w[1].point.y < y)?;
        let (a, b) = (pair[0].point, pair[1].point);
        let span = a.y - b.y;
        if span <= f64::EPSILON {
            return Some(b);
        }
        return Some(a.lerp(&b, (a.y - y) / span));
    }
}

/// 클립을 한 번 훑어 검출·필터 재생을 끝낸다.
pub fn review(left: &Path, right: &Path, fps: f64) -> Result<Reviewed, String> {
    // `Fit::new`가 실제로 쓰는 물리(`RESTITUTION`·`FRICTION`·`DRAG` 포함)와 정확히 같은
    // 값 — 아래 물리를 바꿔 끼우는 버전([`review_with_physics`])과 결과가 갈리면 안 된다.
    let physics = pingpong_bot::defaults::PhysicsParams {
        restitution: pingpong_bot::defaults::vision::fit::RESTITUTION,
        friction: pingpong_bot::defaults::vision::fit::FRICTION,
        drag: pingpong_bot::defaults::vision::fit::DRAG,
        ..pingpong_bot::defaults::PhysicsParams::default()
    };
    return review_with_physics(left, right, fps, physics);
}

/// [`review`]와 같지만 물리 상수를 밖에서 받는다 — e·mu·drag 탐색이 재빌드 없이 후보를
/// 빠르게 돌려 볼 때 쓴다(`--calibrate`).
///
/// 검출(비디오 디코드+캐스케이드)이 압도적으로 무겁고 물리와 무관한데, 후보마다 이걸
/// 통째로 다시 도는 게 `--calibrate` 첫 버전이 느렸던 원인이다(실측: 후보 하나 채점에
/// `--all` 전체보다 훨씬 오래 걸림). [`detect_all`]로 검출을 한 번만 캐싱하고
/// [`replay_with_physics`]로 물리만 바꿔 재적합하면 후보마다 비디오를 다시 안 연다 —
/// 여기(단발 호출)는 그 둘을 그냥 이어 붙인 것뿐이라 기존 동작과 동일하다.
pub fn review_with_physics(
    left: &Path,
    right: &Path,
    fps: f64,
    physics: pingpong_bot::defaults::PhysicsParams,
) -> Result<Reviewed, String> {
    let detected = detect_all(left, right, fps)?;
    return replay_with_physics(&detected, physics);
}

/// 캠별 검출 픽셀만 남긴 캐시. 물리(e·mu·drag)와 무관하다 — [`detect_all`] 참고.
pub struct DetectedFrames {
    pixels: Vec<[Option<camera::Pixel>; 2]>,
    fps: f64,
}

/// 클립의 검출만 한 번 돌린다. 비디오 디코드·검출 캐스케이드가 이 함수의 비용 전부고,
/// 물리와는 무관하다 — 결과를 캐싱해 두면 [`replay_with_physics`]로 물리 후보를 몇
/// 개를 돌리든 이 함수는 한 번만 부르면 된다.
///
/// 안에서 [`Vision`]을 만들긴 하지만 그 안의 `Fit`(트리거·물리)은 안 쓴다 — 캠별 검출
/// 픽셀만 뽑아 버린다. 트리거 값은 아무거나 상관없다.
pub fn detect_all(left: &Path, right: &Path, fps: f64) -> Result<DetectedFrames, String> {
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(|e| format!("calibration 로드: {e}"))?;
    let mut vision = Vision::load(&calibration, defaults::vision::trigger())
        .map_err(|e| format!("vision 조립: {e}"))?;

    let mut left_frames = load_all(left, camera::Id(0))?;
    let mut right_frames = load_all(right, camera::Id(1))?;
    let count = left_frames.len().min(right_frames.len());
    let epoch = Instant::now();

    let mut pixels: Vec<[Option<camera::Pixel>; 2]> = Vec::with_capacity(count);
    for index in 0..count {
        let stamp = epoch + Duration::from_secs_f64(index as f64 / fps);
        let mut row = [None, None];
        // 왼쪽부터 먹인다 — [`replay_with_physics`]도 같은 순서로 다시 먹여야 시드
        // 타이밍이 갈리지 않는다.
        for (slot, source) in [&mut left_frames, &mut right_frames]
            .into_iter()
            .enumerate()
        {
            // 캡처 타임스탬프는 "읽은 시각"이라 못 쓴다. 인덱스로 다시 찍는다.
            source[index].timestamp = stamp;
            vision
                .feed(&source[index])
                .map_err(|e| format!("검출: {e}"))?;
            row[slot] = vision.last_found().map(|candidate| candidate.pixel);
        }
        pixels.push(row);
    }
    return Ok(DetectedFrames { pixels, fps });
}

/// [`detect_all`]의 검출 캐시를 물리만 바꿔 가며 다시 채점한다 — 비디오도 검출도
/// 다시 안 돈다. `Fit::observe`가 픽셀만 쓰므로(`radius_px`·`circularity`는 안 읽는다)
/// 캐시가 픽셀만 들고 있어도 충분하다.
pub fn replay_with_physics(
    detected: &DetectedFrames,
    physics: pingpong_bot::defaults::PhysicsParams,
) -> Result<Reviewed, String> {
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(|e| format!("calibration 로드: {e}"))?;
    let fit = pingpong_bot::vision::Fit::with_physics(
        &calibration,
        defaults::vision::trigger(),
        physics,
    );
    return replay_with(detected, fit);
}

/// [`replay_with_physics`]와 같지만 `Fit`(물리+트리거)을 통째로 밖에서 받는다 —
/// 트리거 상수 탐색처럼 물리는 고정하고 트리거만 바꿔 볼 때 쓴다.
pub fn replay_with(
    detected: &DetectedFrames,
    mut fit: pingpong_bot::vision::Fit,
) -> Result<Reviewed, String> {
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(|e| format!("calibration 로드: {e}"))?;
    let origin = Instant::now();
    let fps = detected.fps;

    let mut frames: Vec<FrameState> = Vec::with_capacity(detected.pixels.len());
    let mut observed: Vec<Observed> = Vec::new();
    let mut contract: Option<Contract> = None;

    for (index, row) in detected.pixels.iter().enumerate() {
        let t = index as f64 / fps;
        let stamp = Duration::from_secs_f64(t);
        let mut state = FrameState::default();

        for slot in 0..2 {
            state.pixels[slot] = row[slot];
            let candidate = row[slot].map(|pixel| pingpong_bot::vision::Candidate {
                pixel,
                // Fit::observe는 pixel만 읽는다 — 나머지는 캐시에 없어도 뜻이 없다.
                radius_px: 0.0,
                circularity: 1.0,
            });
            state.outcomes[slot] = Some(fit.observe(camera::Id(slot as u8), candidate, stamp));
        }

        if let Some(hit) = triangulate(&state.pixels, &calibration)
            && plausible(hit.0)
        {
            let hit = Observed {
                frame: index,
                t,
                point: hit.0,
                reprojection_px: hit.1,
            };
            observed.push(hit);
            state.observed = Some(hit);
        }

        state.filtered = fit.measured().last().copied();
        state.sightings = fit.sightings();
        state.stereo_samples = fit.stereo_samples();
        state.solved_spin = fit.solved_spin();
        state.seq = fit.seq();
        state.tracking = state.filtered.is_some();
        // 적합이 이번 관측을 실제로 받아들였을 때만 잔차를 센다. 거부된 프레임의 상태는
        // 이전 시각의 것이라 지금 픽셀과 견주면 시간차가 잔차로 둔갑한다.
        if let Some(fitted) = state.filtered {
            for slot in 0..2 {
                let fresh = matches!(
                    state.outcomes[slot],
                    Some(Outcome::Accepted | Outcome::Seeded)
                );
                state.residual_px[slot] = fresh
                    .then(|| {
                        calibration
                            .params(camera::Id(slot as u8))
                            .zip(state.pixels[slot])
                            .and_then(|(params, pixel)| {
                                Some(
                                    (params.project_world_unclipped(fitted.position)? - pixel)
                                        .norm(),
                                )
                            })
                    })
                    .flatten();
            }
        }
        state.predicted_impact = fit
            .trajectory(origin)
            .and_then(|t| t.predicted.at_plane(table::DEFAULT_HIT_PLANE_Y))
            .map(|s| s.position);

        // 계약이 생기면 잡고, 같은 공인 동안 갱신한다 — measured 가 계속 자란다.
        // 다른 공이 다시 트리거를 넘겨도 첫 계약만 본다 (실기도 샷당 한 번이다).
        if let Some(trajectory) = fit.trajectory(origin) {
            match &mut contract {
                None => {
                    contract = Some(Contract {
                        frame: index,
                        t,
                        at_trigger: trajectory.clone(),
                        latest: trajectory,
                    });
                }
                Some(held) if held.latest.seq == trajectory.seq => held.latest = trajectory,
                Some(_) => {}
            }
        }
        frames.push(state);
    }

    return Ok(Reviewed {
        frames,
        observed,
        contract,
        fps,
    });
}

/// 클립을 통째로 메모리에 올린다. 되감기가 있어 두 번 훑을 수 없다.
fn load_all(path: &Path, camera_id: camera::Id) -> Result<Vec<Frame>, String> {
    let mut source = OpenCvCapture::from_path(camera_id, path)?;
    let mut out = Vec::new();
    while let Some(frame) = source.next_frame() {
        out.push(frame);
    }
    return Ok(out);
}

/// 두 캠이 같은 프레임에서 잡은 것만 복원한다. 재투영이 벌어지면 서로 다른 걸 잡은 것.
fn triangulate(
    pixels: &[Option<camera::Pixel>; 2],
    calibration: &Calibration,
) -> Option<(Point3, f64)> {
    let (Some(left), Some(right)) = (pixels[0], pixels[1]) else {
        return None;
    };
    let hits = [(camera::Id(0), left), (camera::Id(1), right)];
    let point = Triangulate::pixels(&hits, calibration)?;
    let worst = hits
        .iter()
        .filter_map(|(id, pixel)| {
            let params = calibration.params(*id)?;
            let projected = params.project_world_unclipped(point)?;
            return Some((projected - *pixel).norm());
        })
        .fold(0.0_f64, f64::max);
    if worst > MAX_REPROJECTION_PX {
        return None;
    }
    return Some((point, worst));
}

/// 삼각측량 결과가 물리적으로 말이 되는가 — 채점 기준(`observed`)에만 쓰는 문지방이다.
///
/// 재투영 오차만으로는 못 거른다 — 두 카메라가 동시에 라켓·그림자 등 딴 걸 잡으면
/// 서로는 잘 맞아떨어져(재투영은 작게 나옴) 그럴싸한 3D 점을 낸다(실측 fly_49 frame 451:
/// 라켓이 공을 잠깐 가린 순간 바닥 근처로 튐, `point=[-0.28, -0.13, 0.57]`, 이웃 프레임은
/// x 0.68~0.70·z 0.8~1.3 대인데 재투영은 5.8px로 안 큼). 이 점이 그대로 `observed`(채점
/// 정답)에 들어가면 리드타임을 아무리 줄여도 안 나아지는 평평한 큰 오차로 나온다 — 예측이
/// 아니라 정답이 튄 것이다. z가 테이블면보다 한참 낮으면(뜬 공이 실제로 갈 자리가 아니다)
/// 가장 확실한 신호라 그것만 본다. x,y는 여유를 크게 둔다 — 네트 밖·테이블 옆으로도 실제
/// 공이 나갈 수 있어서, 좁히면 진짜 관측을 지운다.
fn plausible(point: Point3) -> bool {
    const Z_FLOOR_TOLERANCE: f64 = 0.10;
    const XY_MARGIN: f64 = 0.5;
    return point.z >= table::SURFACE_Z - Z_FLOOR_TOLERANCE
        && point.x >= -XY_MARGIN
        && point.x <= table::WIDTH_X + XY_MARGIN
        && point.y >= -XY_MARGIN
        && point.y <= table::LENGTH_Y + XY_MARGIN;
}

/// 리드타임 `lead` 뒤의 **예측**과 그때의 **생 관측**이 얼마나 벌어지는가 [m].
///
/// 이게 이 툴의 본론이다. 생 궤적은 클립 전체를 이미 훑어서 알고 있으므로,
/// 재생 중 어느 프레임에서든 "그때 한 예측이 맞았는지"를 바로 잴 수 있다.
pub fn convergence_error(
    predicted: &Track,
    observed: &[Observed],
    now: f64,
    lead: f64,
    fps: f64,
) -> Option<f64> {
    let target = now + lead;
    // 프레임 반 칸 안쪽의 실제 관측만 짝으로 인정한다 — 없으면 잴 수 없다.
    let tolerance = 0.5 / fps;
    let truth = observed
        .iter()
        .filter(|o| (o.t - target).abs() <= tolerance)
        .min_by(|a, b| {
            (a.t - target)
                .abs()
                .partial_cmp(&(b.t - target).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
    let guess = predicted.at_time(Duration::from_secs_f64(truth.t.max(0.0)))?;
    return Some((guess.position - truth.point).norm());
}

#[cfg(test)]
#[path = "track_tests.rs"]
mod tests;
