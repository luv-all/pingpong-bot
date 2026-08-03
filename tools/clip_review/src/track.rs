//! 클립 한 번 훑기 — [`Vision`]에 프레임을 먹여 프레임별 상태를 남긴다.
//!
//! 두 AVI는 동시 녹화라 같은 인덱스가 같은 시각이다. `OpenCvCapture`의 타임스탬프는
//! 읽은 시각이라 쓸 수 없으므로 `인덱스 / fps`로 다시 찍는다.
//!
//! 두 번 훑지 않는다 — 검출은 여기서 한 번만 하고, 재생 루프는 그 결과를 다시 쓴다.
//! 되감아도 검출이 다시 돌지 않으므로 0.1x로 앞뒤를 오가도 값이 흔들리지 않는다.

use std::path::Path;
use std::time::{Duration, Instant};

use pingpong_bot::Point3;
use pingpong_bot::camera::{self, Calibration, Frame, FrameSource, OpenCvCapture, Triangulate};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::vision::{Outcome, State, Track, Trajectory, Vision, triggers};

/// 재투영 오차가 이보다 크면 두 캠이 서로 다른 걸 잡은 것 — 시드 게이트와 같은 상한.
const MAX_REPROJECTION_PX: f64 = 14.0;

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

    /// 생 궤적을 현재 프레임 기준 (지금까지, 이후)로 나눈다.
    ///
    /// 오프라인 재생이라 미래를 **이미 안다** — 커밋 예측이 이후로 어디로 갔는지와 나란히
    /// 보려면 있어야 한다. 다만 과거와 같은 굵기로 그리면 "지금 아는 것"과 구분이 안 되므로
    /// 나눠서 넘긴다. 경계 한 점은 양쪽에 다 들어간다 (선이 끊겨 보이지 않게).
    pub fn observed_split(&self, frame: usize) -> (Vec<Point3>, Vec<Point3>) {
        let cut = self.observed.partition_point(|o| o.frame <= frame);
        let past: Vec<Point3> = self.observed[..cut].iter().map(|o| o.point).collect();
        let future: Vec<Point3> = self.observed[cut.saturating_sub(1)..]
            .iter()
            .map(|o| o.point)
            .collect();
        return (past, future);
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
    let calibration = Calibration::load_json(&defaults::calibration_path())
        .map_err(|e| format!("calibration 로드: {e}"))?;
    // 실기와 **같은** 트리거다. 도구가 더 늦게 걸면 실기에서 쓸 수 있는 구간을 못 본다.
    let mut vision =
        Vision::load(&calibration, triggers::primary()).map_err(|e| format!("vision 조립: {e}"))?;

    let mut left_frames = load_all(left, camera::Id(0))?;
    let mut right_frames = load_all(right, camera::Id(1))?;
    let count = left_frames.len().min(right_frames.len());

    let epoch = Instant::now();
    let mut frames: Vec<FrameState> = Vec::with_capacity(count);
    let mut observed: Vec<Observed> = Vec::new();
    let mut contract: Option<Contract> = None;

    for index in 0..count {
        let t = index as f64 / fps;
        let stamp = epoch + Duration::from_secs_f64(t);
        let mut state = FrameState::default();

        // 왼쪽부터 먹인다. 시드는 두 시선이 다 들어온 뒤에 선다.
        for (slot, source) in [&mut left_frames, &mut right_frames]
            .into_iter()
            .enumerate()
        {
            // 캡처 타임스탬프는 "읽은 시각"이라 못 쓴다. 인덱스로 다시 찍는다.
            source[index].timestamp = stamp;
            vision
                .feed(&source[index])
                .map_err(|e| format!("검출: {e}"))?;
            state.pixels[slot] = vision.last_found().map(|candidate| candidate.pixel);
            state.outcomes[slot] = vision.last_outcome();
        }

        if let Some(hit) = triangulate(&state.pixels, &calibration) {
            let hit = Observed {
                frame: index,
                t,
                point: hit.0,
                reprojection_px: hit.1,
            };
            observed.push(hit);
            state.observed = Some(hit);
        }

        state.filtered = vision.fit().measured().last().copied();
        state.sightings = vision.fit().sightings();
        state.seq = vision.fit().seq();
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
        state.predicted_impact = vision
            .trajectory()
            .and_then(|t| t.predicted.at_plane(table::DEFAULT_HIT_PLANE_Y))
            .map(|s| s.position);

        // 계약이 생기면 잡고, 같은 공인 동안 갱신한다 — measured 가 계속 자란다.
        // 다른 공이 다시 트리거를 넘겨도 첫 계약만 본다 (실기도 샷당 한 번이다).
        if let Some(trajectory) = vision.trajectory() {
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
