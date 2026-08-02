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
    /// 이 프레임까지 필터가 낸 상태. 시드 전이면 `None`.
    pub filtered: Option<State>,
    /// 캠별 필터 판정. 어느 쪽 검출이 거부됐는지 봐야 진단이 된다.
    pub outcomes: [Option<Outcome>; 2],
    pub tracking: bool,
}

/// 트리거가 걸린 순간 — 실기라면 여기서 하드웨어로 넘어갔다.
///
/// 이게 얼려 있어야 "예측이 맞았나"를 볼 수 있다. 매 프레임 다시 굴린 예측은 언제나
/// 현재 공 위치에서 출발하니 실제와 겹쳐 보일 수밖에 없다 — 볼 게 없다.
#[derive(Debug, Clone)]
pub struct Commit {
    pub frame: usize,
    pub t: f64,
    /// 트리거 순간의 예측 궤적 — 이후 프레임에서도 **바뀌지 않는다**.
    pub predicted: Track,
    /// 그때 필터가 말한 축별 불확실성.
    pub sigma_position: pingpong_bot::Vector3,
    pub sigma_velocity: pingpong_bot::Vector3,
}

/// 클립 전체 — 창이 필요로 하는 모든 것.
pub struct Reviewed {
    pub frames: Vec<FrameState>,
    /// 클립 전체의 생 궤적 (재생 위치와 무관하게 이미 다 안다).
    pub observed: Vec<Observed>,
    /// 트리거가 걸린 순간의 예측.
    pub commit: Option<Commit>,
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
    let mut vision = Vision::load(
        &calibration,
        // 네트 통과 — 실기와 달리 σ 조건을 안 건다. 클립마다 같은 자리에서 얼려야 비교된다.
        Box::new(triggers::PlaneCrossing {
            y: table::LENGTH_Y * 0.5,
        }),
    )
    .map_err(|e| format!("vision 조립: {e}"))?;

    let mut left_frames = load_all(left, camera::Id(0))?;
    let mut right_frames = load_all(right, camera::Id(1))?;
    let count = left_frames.len().min(right_frames.len());

    let epoch = Instant::now();
    let mut frames: Vec<FrameState> = Vec::with_capacity(count);
    let mut observed: Vec<Observed> = Vec::new();
    let mut commit: Option<Commit> = None;

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

        state.filtered = vision.ekf().measured().last().copied();
        state.tracking = state.filtered.is_some();

        // 트리거가 처음 걸린 프레임의 예측만 얼린다.
        if commit.is_none()
            && let Some(trajectory) = vision.trajectory()
        {
            commit = Some(freeze(index, t, &trajectory));
        }
        frames.push(state);
    }

    return Ok(Reviewed {
        frames,
        observed,
        commit,
        fps,
    });
}

fn freeze(frame: usize, t: f64, trajectory: &Trajectory) -> Commit {
    let at_trigger = trajectory.predicted.first().copied().unwrap_or_default();
    return Commit {
        frame,
        t,
        predicted: trajectory.predicted.clone(),
        sigma_position: at_trigger.sigma_position,
        sigma_velocity: at_trigger.sigma_velocity,
    };
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
