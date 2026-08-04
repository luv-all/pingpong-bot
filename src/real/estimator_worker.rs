//! 추정 워커 — 삼각측량 → EKF → 목표 선택 → 1·2차 제어 요청.
//!
//! `Ekf`와 `Calibration`을 **단독 소유**한다. 로봇 포즈는 볼 수 없다 (제어 워커만 안다).

use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError};
use pingpong_bot::camera;
use pingpong_bot::camera::Calibration;
use pingpong_bot::constants::table;
use pingpong_bot::detector;
use pingpong_bot::estimator::{Ekf, Estimator, GateOutcome, Triangulate};
use pingpong_bot::robot::control::{
    HitTarget, HitTargetSelector, PredictionStability, PredictionStage,
};
use pingpong_bot::robot::motion::InterceptWindow;
use tracing::{debug, info_span};

use super::ball_receding::{BallReceding, MIN_DELTA_Y, MIN_SAMPLES};
use super::fmt::f2;
use super::{
    CommitRequest, PreviewEvent, RuntimeEvent, Shutdown, SimUpdate, Throttle, VisionEvent,
};

/// 카메라당 보관할 관측 수 — `Triangulate::synced`가 보간에 쓸 앞뒤 프레임.
const SERIES_CAPACITY: usize = 8;

/// 캠 간 최신 관측 시각이 이보다 벌어지면 삼각측량하지 않는다.
///
/// 한쪽이 검출을 놓치는 동안 그 캠의 마지막 관측이 시계열에 남아 `sync_time`을 과거로
/// 끌어당기는 걸 막는다. 30 fps에서 한 프레임이 33 ms이므로 그 안쪽으로 잡는다 —
/// 넉넉하게 두면 낡은 시선이 다시 들어온다.
const MAX_SYNC_LAG: Duration = Duration::from_millis(35);

/// 생 삼각측량 마커를 화면에 남겨두는 시간. 지나면 지운다 — 멈춘 점을 살아있는 값으로
/// 오해하지 않게 한다.
const RAW_MARKER_TTL: Duration = Duration::from_millis(150);

/// 재투영 오차가 이보다 크면 그 삼각측량은 버린다 [px].
///
/// 두 캠이 **서로 다른 것**을 잡으면(한쪽은 공, 한쪽은 배경 blob) 3D 점이 완전히 엉뚱한
/// 곳에 서고, 그 한 점이 EKF 상태를 통째로 끌고 간다. 그런 쌍은 재투영 오차가 즉시 폭발하므로
/// 여기서 거른다 — 색·원형도 게이트를 통과해도 **기하가 안 맞으면** 공이 아니다.
///
/// fly_02 실측(원형도 0.35): p50 1.84 px인데 p95가 168 px였다. 캘리브 rmse가 3.7/3.3 px이고
/// 캘리브 채택 상한이 `MAX_REPROJ_RMSE_PX = 7.0`이므로 그 두 배를 상한으로 둔다.
const MAX_REPROJECTION_PX: f64 = 14.0;

/// 시계열에서 이보다 오래된 관측은 버린다 (보간 구간이 공백을 건너뛰지 않게).
const MAX_OBSERVATION_AGE: Duration = Duration::from_millis(250);

const RECV_TIMEOUT: Duration = Duration::from_millis(50);

/// `--debug` 진척 로그 주기.
const PROGRESS_PERIOD: Duration = Duration::from_secs(1);

/// 종료 요약용 추정 통계.
#[derive(Debug, Clone, Default)]
pub struct EstimatorStats {
    pub triangulated: u64,
    /// 보정 전 좌/우 최신 프레임 시각 차 [s] — 리그 동기 품질.
    pub skew_samples: Vec<f64>,
    /// 삼각측량 재투영 오차 [px] — 3D 복원 품질.
    pub reprojection_samples: Vec<f64>,
    /// 재투영 오차가 커서 버린 삼각측량 수 (`MAX_REPROJECTION_PX` 초과).
    ///
    /// 두 캠이 서로 다른 것을 잡은 쌍이다. 크면 한쪽 검출에 오검출이 섞이고 있다.
    pub reprojection_rejected: u64,
    /// 테이블 주변의 물리적으로 가능한 공간을 벗어나 버린 3D 점.
    pub workspace_rejected: u64,
    /// 두 캠 다 관측이 있는데도 삼각측량을 건너뛴 프레임 수.
    ///
    /// 대부분 `MAX_SYNC_LAG` 초과(한 캠이 검출을 놓쳐 뒤처짐)다. 이 값이 크면 한쪽 캠의
    /// 검출률이 떨어지고 있다는 뜻이다.
    pub stale_skipped: u64,
    pub accepted: u64,
    pub rejected: u64,
    pub seeded: u64,
    pub reset: u64,
    pub preview_dropped: u64,
    pub commit_dropped: u64,
}

impl EstimatorStats {
    fn record(&mut self, outcome: GateOutcome) {
        match outcome {
            GateOutcome::Accepted => self.accepted += 1,
            GateOutcome::Rejected => self.rejected += 1,
            GateOutcome::Seeded | GateOutcome::VelocitySeeded => self.seeded += 1,
            GateOutcome::Reset => self.reset += 1,
            GateOutcome::Ignored => {}
        }
    }

    /// skew 백분위수 [s].
    pub fn skew_percentile(&self, q: f64) -> Option<f64> {
        return percentile(&self.skew_samples, q);
    }

    /// 재투영 오차 백분위수 [px].
    pub fn reprojection_percentile(&self, q: f64) -> Option<f64> {
        return percentile(&self.reprojection_samples, q);
    }
}

/// 추정 워커를 띄운다.
pub fn spawn(
    rx: Receiver<VisionEvent>,
    calibration: Calibration,
    intercept: InterceptWindow,
    commit_tx: Sender<CommitRequest>,
    commit_evict_rx: Receiver<CommitRequest>,
    preview_tx: Option<Sender<PreviewEvent>>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<RuntimeEvent>,
    shutdown: Shutdown,
) -> JoinHandle<EstimatorStats> {
    return thread::spawn(move || {
        let _span = info_span!("estimator").entered();
        let mut stats = EstimatorStats::default();
        let mut ekf = Ekf::default();
        let mut series: Vec<(camera::Id, Vec<detector::Observation>)> = calibration
            .cameras
            .iter()
            .map(|params| (params.camera_id, Vec::with_capacity(SERIES_CAPACITY)))
            .collect();
        // 필터를 거치지 않은 생 삼각측량 점 — EKF 추정과 나란히 띄워 "필터가 뭉갠 건지
        // 입력이 이미 튄 건지"를 눈으로 가른다. 삼각측량이 멎으면 **같이 사라져야** 한다 —
        // 붙들고 있으면 멈춘 점이 살아있는 값처럼 보여 진단을 헷갈리게 한다.
        let mut last_raw: Option<(pingpong_bot::Point3, Instant)> = None;
        // 마지막으로 EKF에 넣은 측정 시각. `sync_time`은 **뒤처진 캠이 갱신될 때만**
        // 전진하므로, 이걸 안 보면 앞선 캠이 프레임을 받을 때마다 같은 측정을 다시
        // 주입하게 된다 — dt=0이라 예측 단계 없이 칼만 갱신만 반복돼 공분산이 실제보다
        // 작아지고, 과신한 게이트가 멀쩡한 새 측정을 거부한다.
        let mut last_sync: Option<Instant> = None;
        let required = calibration.min_cameras_for_triangulation();
        let selector =
            HitTargetSelector::new(intercept.y_min, intercept.y_max).expect("기본 목표 선택 구간");
        let mut prediction_stability = PredictionStability::default();
        let mut track_seq: u64 = 1;
        let mut receding = BallReceding::new(MIN_DELTA_Y, MIN_SAMPLES);
        let mut announced_track = false;
        let mut last_stage: Option<PredictionStage> = None;
        let mut progress = Throttle::new(PROGRESS_PERIOD);

        while !shutdown.is_down() {
            let event = match rx.recv_timeout(RECV_TIMEOUT) {
                Ok(event) => event,
                Err(RecvTimeoutError::Timeout) => continue,
                Err(RecvTimeoutError::Disconnected) => break,
            };

            if let Some(pixel) = event.pixel
                && let Some((_, observations)) = series
                    .iter_mut()
                    .find(|(id, _)| *id == event.frame.camera_id)
            {
                let now = event.frame.timestamp;
                observations.push(detector::Observation {
                    pixel,
                    camera_id: event.frame.camera_id,
                    timestamp: now,
                });
                // 공백을 가로질러 보간하지 않도록 낡은 관측을 버린다.
                observations.retain(|observation| {
                    now.saturating_duration_since(observation.timestamp) <= MAX_OBSERVATION_AGE
                });
                if observations.len() > SERIES_CAPACITY {
                    let drop = observations.len() - SERIES_CAPACITY;
                    observations.drain(0..drop);
                }
            }

            let fused = fuse(&series, required, &calibration)
                .ok()
                // 새 시각이 아니면 버린다 — 같은 측정을 두 번 세지 않는다.
                .filter(|fused| last_sync.is_none_or(|last| fused.sync_time > last));
            if fused.is_none() {
                match fuse(&series, required, &calibration) {
                    Err(FuseSkip::Stale) => stats.stale_skipped += 1,
                    Err(FuseSkip::Reprojection) => stats.reprojection_rejected += 1,
                    _ => {}
                }
            }
            if let Some(fused) = fused {
                let (point, sync_time) = (fused.point, fused.sync_time);
                last_sync = Some(sync_time);
                last_raw = Some((point, Instant::now()));
                stats.triangulated += 1;
                stats.skew_samples.push(fused.skew);
                stats.reprojection_samples.push(fused.reprojection_px);
                if !plausible_ball_point(point) {
                    stats.workspace_rejected += 1;
                    debug!(
                        x = f2(point.x),
                        y = f2(point.y),
                        z = f2(point.z),
                        "물리 작업공간 밖 3D 점 거부"
                    );
                    continue;
                }
                let outcome = ekf.update_position(point, sync_time);
                stats.record(outcome);
                // 버려진 측정만 찍는다 — Accepted는 초당 100건이라 요약 카운트로 충분하다.
                if !outcome.used_measurement() || matches!(outcome, GateOutcome::Reset) {
                    debug!(
                        outcome = ?outcome,
                        reject_streak = ekf.reject_streak(),
                        d2 = ekf.last_gate_d2().map(f2),
                        x = f2(point.coords.x),
                        y = f2(point.coords.y),
                        z = f2(point.coords.z),
                        skew_ms = f2(fused.skew * 1e3),
                        reproj_px = f2(fused.reprojection_px),
                        "측정 거부"
                    );
                }
            }

            let mut ball_y = ekf.position().map(|position| position.coords.y);

            // 공 y가 로봇에서 멀어지면(증가) 다음 공으로 보고 EKF와 단계 상태를 초기화한다.
            if let Some(y) = ball_y
                && receding.observe(y)
            {
                ekf.reset();
                prediction_stability.reset();
                announced_track = false;
                last_stage = None;
                receding.reset();
                track_seq = track_seq.saturating_add(1);
                ball_y = None;
                debug!(
                    track = track_seq,
                    y = f2(y),
                    "공 y 증가 — 새 궤적으로 초기화"
                );
            }

            let tracking = ekf.is_tracking();
            if ball_y.is_none() {
                ball_y = ekf.position().map(|position| position.coords.y);
            }

            if tracking && !announced_track {
                announced_track = true;
                if let (Some(position), Some(velocity)) = (ekf.position(), ekf.velocity()) {
                    let _ = event_tx.send(RuntimeEvent::Tracking {
                        track_seq,
                        position,
                        speed: velocity.norm(),
                    });
                }
            }

            let trajectory = tracking.then(|| ekf.trajectory()).flatten();
            let selected = trajectory
                .as_ref()
                .and_then(|trajectory| selector.select(trajectory).ok());
            let stage = selected.map(|target| {
                let observed_span_secs = trajectory
                    .as_ref()
                    .and_then(|trajectory| trajectory.observed.first())
                    .map(|sample| -sample.time_secs)
                    .unwrap_or(0.0);
                prediction_stability.observe(target.position, observed_span_secs)
            });

            // 선택 목표의 리드타임으로 예측 불확실성을 표시한다. 제어 게이트로 쓰지는 않는다.
            let target_sigma = selected.and_then(|target| {
                let (sp, sv) = (ekf.position_sigma()?, ekf.velocity_sigma()?);
                Some(sp.hypot(sv * target.time_secs))
            });
            if stage != last_stage {
                last_stage = stage;
                debug!(track = track_seq, ?stage, "2단계 제어 상태 변경");
            }
            if progress.ready() {
                debug!(
                    triangulated = stats.triangulated,
                    accepted = stats.accepted,
                    rejected = stats.rejected,
                    tracking,
                    track = track_seq,
                    ?stage,
                    "추정 진척"
                );
            }

            if let (Some(trajectory), Some(stage)) = (trajectory, stage) {
                let request = CommitRequest {
                    track_seq,
                    trajectory,
                    stage,
                    at: Instant::now(),
                };
                send_latest_commit(&commit_tx, &commit_evict_rx, request, &mut stats);
            }

            if let Some(sim_tx) = &sim_tx {
                let _ = sim_tx.try_send(SimUpdate {
                    ball: ekf.position(),
                    target: selected.map(|target| target.position),
                    ..SimUpdate::default()
                });
            }

            if let Some(preview_tx) = &preview_tx {
                // 예측 도달점을 **이 카메라로 재투영**한다. 자르지 않고 프레임 밖인지만
                // 따로 알려 준다 — 안 보이는 이유가 "예측 없음"인지 "화각 밖"인지 구분해야
                // 벤치에서 진단이 된다 (접수 평면 y 0.08~0.35는 로봇 코앞이라 화각을
                // 벗어나기 쉽다).
                let params = calibration.params(event.frame.camera_id);
                let raw_pixel = last_raw
                    .filter(|(_, at)| at.elapsed() <= RAW_MARKER_TTL)
                    .zip(params)
                    .and_then(|((point, _), params)| params.project_world_unclipped(point));
                let target_pixel = selected
                    .zip(params)
                    .and_then(|(target, params)| params.project_world_unclipped(target.position));
                let target_offscreen = target_pixel.is_some_and(|pixel| {
                    params.is_some_and(|params| {
                        pixel.x < 0.0
                            || pixel.y < 0.0
                            || pixel.x >= f64::from(params.width)
                            || pixel.y >= f64::from(params.height)
                    })
                });
                let hud = hud_lines(
                    &ekf,
                    tracking,
                    stage,
                    selected.as_ref(),
                    target_offscreen,
                    stats.reprojection_samples.last().copied(),
                    target_sigma,
                );
                let preview = PreviewEvent {
                    frame: event.frame,
                    pixel: event.pixel,
                    target_pixel,
                    target_offscreen,
                    raw_pixel,
                    hud,
                };
                if let Err(TrySendError::Full(_)) = preview_tx.try_send(preview) {
                    stats.preview_dropped += 1;
                }
            }
        }

        debug!(
            triangulated = stats.triangulated,
            accepted = stats.accepted,
            "추정 워커 종료"
        );
        return stats;
    });
}

/// 캘리브레이션이 순간적으로 잘못 대응돼도 수십 미터 밖 점을 EKF에 넣지 않는다.
///
/// 로그의 (-26, -5.5, -18)m는 재투영 오차만으로 통과했지만 탁구공일 수 없다.
/// 테이블 둘레와 슈터 쪽 여유만 허용하고 실제 접수·바운스 공간은 넉넉히 남긴다.
fn plausible_ball_point(point: pingpong_bot::Point3) -> bool {
    const SIDE_MARGIN_M: f64 = 0.75;
    const ROBOT_SIDE_MARGIN_M: f64 = 0.75;
    const SHOOTER_SIDE_MARGIN_M: f64 = 1.25;
    const Z_MIN_M: f64 = -0.10;
    const Z_MAX_M: f64 = 3.0;

    return point.x >= -SIDE_MARGIN_M
        && point.x <= table::WIDTH_X + SIDE_MARGIN_M
        && point.y >= -ROBOT_SIDE_MARGIN_M
        && point.y <= table::LENGTH_Y + SHOOTER_SIDE_MARGIN_M
        && point.z >= Z_MIN_M
        && point.z <= Z_MAX_M;
}

/// 제어가 계획 중이어도 오래된 요청 대신 최신 요청 한 건만 큐에 남긴다.
fn send_latest_commit(
    tx: &Sender<CommitRequest>,
    evict_rx: &Receiver<CommitRequest>,
    request: CommitRequest,
    stats: &mut EstimatorStats,
) {
    match tx.try_send(request) {
        Ok(()) => {}
        Err(TrySendError::Full(request)) => {
            let _ = evict_rx.try_recv();
            if tx.try_send(request).is_err() {
                stats.commit_dropped += 1;
            }
        }
        Err(TrySendError::Disconnected(_)) => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::estimator::BallTrajectory;
    use pingpong_bot::robot::control::PredictionStage;

    fn request(track_seq: u64) -> CommitRequest {
        return CommitRequest {
            track_seq,
            trajectory: BallTrajectory::new(Vec::new(), Vec::new(), Instant::now()).unwrap(),
            stage: PredictionStage::Provisional,
            at: Instant::now(),
        };
    }

    #[test]
    fn impossible_world_point_is_rejected_before_ekf() {
        assert!(!plausible_ball_point(pingpong_bot::Point3::new(
            -26.0, -5.51, -18.15
        )));
        assert!(plausible_ball_point(pingpong_bot::Point3::new(
            0.99, 2.16, 1.03
        )));
    }

    #[test]
    fn full_commit_queue_keeps_latest_request() {
        let (tx, rx) = crossbeam_channel::bounded(1);
        let mut stats = EstimatorStats::default();
        send_latest_commit(&tx, &rx, request(1), &mut stats);
        send_latest_commit(&tx, &rx, request(2), &mut stats);
        assert_eq!(rx.recv().unwrap().track_seq, 2);
    }
}

/// 정렬 후 백분위수. 표본이 없으면 `None`.
fn percentile(samples: &[f64], q: f64) -> Option<f64> {
    if samples.is_empty() {
        return None;
    }
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let index = ((sorted.len() - 1) as f64 * q).round() as usize;
    return sorted.get(index).copied();
}

/// 한 번의 삼각측량 결과.
pub struct Fused {
    pub point: pingpong_bot::Point3,
    /// 보간 기준 시각 — EKF에 넘길 측정 시각.
    pub sync_time: Instant,
    /// 보정 **전** 좌/우 최신 프레임의 시각 차 [s]. 리그 동기 품질 계측용.
    pub skew: f64,
    /// 복원한 3D 점을 각 카메라로 되쏘았을 때의 최대 오차 [px].
    ///
    /// 검출(2D)은 멀쩡한데 3D가 나쁜 경우를 잡아낸다 — 그러면 원인은 검출기가 아니라
    /// 캘리브레이션이나 동기다.
    pub reprojection_px: f64,
}

/// 카메라별 관측 시계열을 **공통 시각으로 보간해** 삼각측량한다.
///
/// 예전에는 각 캠의 최신 픽셀을 그대로 짝지었다(`Triangulate::pixels`). UVC 캠은 하드웨어
/// 동기가 없어서 그 둘이 최대 20 ms까지 어긋나는데, 5 m/s 공이면 그동안 10 cm를 간다 —
/// 서로 다른 순간의 시선을 교차시키니 3D 점이 **체계적으로** 밀렸다 (실측 skew p95 18.9 ms).
/// `Triangulate::synced`는 각 캠 시계열을 `sync_time`으로 선형 보간해 그 편향을 없앤다.
///
/// `sync_time`은 **캠별 최신 시각 중 가장 이른 것**이다 — 그래야 모든 캠이 그 시각을 감싸
/// 외삽이 아니라 보간이 된다. 대가는 지연 최대 한 프레임.
///
/// # 신선도
///
/// 한 캠이 검출을 놓치면 그 시계열의 마지막 관측이 그대로 남는다. 만료시키지 않으면
/// `sync_time`이 그 낡은 시각에 고정돼 **멀쩡한 캠을 몇 초 전으로 보간**하게 된다
/// (실측: skew p50 2.7 s, p95 44 s — 예측이 간헐적으로 완전히 튀던 원인).
/// 그래서 모든 캠의 최신 관측이 [`MAX_SYNC_LAG`] 안에 있을 때만 삼각측량한다.
fn fuse(
    series: &[(camera::Id, Vec<detector::Observation>)],
    required: usize,
    calibration: &Calibration,
) -> Result<Fused, FuseSkip> {
    let ready: Vec<(camera::Id, &[detector::Observation])> = series
        .iter()
        .filter(|(_, observations)| !observations.is_empty())
        .map(|(id, observations)| (*id, observations.as_slice()))
        .collect();
    if ready.len() < required {
        return Err(FuseSkip::NotReady);
    }

    let latest: Vec<Instant> = ready
        .iter()
        .filter_map(|(_, observations)| observations.last().map(|o| o.timestamp))
        .collect();
    // 한 캠이라도 뒤처져 있으면 이번 프레임은 버린다 — 낡은 시선으로 만든 3D 점이
    // EKF를 통째로 흔든다.
    let (Some(newest), Some(oldest)) = (latest.iter().copied().max(), latest.iter().copied().min())
    else {
        return Err(FuseSkip::NotReady);
    };
    if newest.saturating_duration_since(oldest) > MAX_SYNC_LAG {
        return Err(FuseSkip::Stale);
    }
    // 모든 캠이 감싸는 시각으로 보간한다 (외삽 금지).
    let sync_time = oldest;
    let skew = newest.saturating_duration_since(sync_time).as_secs_f64();

    let point =
        Triangulate::synced(&ready, sync_time, calibration).map_err(|_| FuseSkip::NotReady)?;
    let reprojection_px = reprojection_error_px(&ready, sync_time, calibration, point);
    if reprojection_px > MAX_REPROJECTION_PX {
        return Err(FuseSkip::Reprojection);
    }
    return Ok(Fused {
        point,
        sync_time,
        skew,
        reprojection_px,
    });
}

/// 삼각측량을 건너뛴 이유 — 계측을 나눠 세려고 구분한다.
enum FuseSkip {
    /// 관측이 모자라거나 보간에 실패.
    NotReady,
    /// 한 캠이 뒤처짐 (`MAX_SYNC_LAG` 초과).
    Stale,
    /// 두 캠이 서로 다른 것을 잡음 (`MAX_REPROJECTION_PX` 초과).
    Reprojection,
}

/// 복원한 점을 각 카메라로 되쏘아 보간 픽셀과의 최대 거리 [px].
fn reprojection_error_px(
    ready: &[(camera::Id, &[detector::Observation])],
    sync_time: Instant,
    calibration: &Calibration,
    point: pingpong_bot::Point3,
) -> f64 {
    let mut worst = 0.0_f64;
    for (camera_id, observations) in ready {
        let Some(params) = calibration.params(*camera_id) else {
            continue;
        };
        let (Some(measured), Some(projected)) = (
            Triangulate::sample_at(observations, sync_time),
            params.project_world_unclipped(point),
        ) else {
            continue;
        };
        let dx = projected.x - measured.x;
        let dy = projected.y - measured.y;
        worst = worst.max(dx.hypot(dy));
    }
    return worst;
}

/// HUD 문자열은 **ASCII만** 쓴다 — Hershey 폰트가 유니코드를 못 그린다 (한글은 `??????`).
fn hud_lines(
    ekf: &Ekf,
    tracking: bool,
    stage: Option<PredictionStage>,
    target: Option<&HitTarget>,
    target_offscreen: bool,
    reprojection_px: Option<f64>,
    target_sigma: Option<f64>,
) -> Vec<String> {
    let state = match (tracking, stage) {
        (false, _) => "WAIT no track",
        (true, None) => "WAIT no target",
        (true, Some(PredictionStage::Provisional)) => "CONTROL provisional",
        (true, Some(PredictionStage::Refined)) => "CONTROL refined",
    };
    let mut lines = vec![state.to_owned()];
    if let Some(position) = ekf.position() {
        let speed = ekf.velocity().map(|v| v.norm()).unwrap_or(0.0);
        lines.push(format!(
            "ball   x{:+.2} y{:+.2} z{:+.2}  |v|{:.1}",
            position.coords.x, position.coords.y, position.coords.z, speed
        ));
    }
    if let Some(target) = target {
        let offscreen = if target_offscreen {
            "  [OFF-FRAME]"
        } else {
            ""
        };
        lines.push(format!(
            "target x{:+.2} y{:+.2} z{:+.2}  eta {:.2}s{offscreen}",
            target.position.coords.x,
            target.position.coords.y,
            target.position.coords.z,
            target.time_secs
        ));
    } else {
        lines.push("target none".to_owned());
    }
    if let Some(d2) = ekf.last_gate_d2() {
        lines.push(format!("gate   d2 {d2:.1}  reject {}", ekf.reject_streak()));
    }
    if let Some(sigma) = target_sigma {
        lines.push(format!("sigma  {:.0} cm", sigma * 100.0));
    }
    // 3D 복원 품질 — 초록(검출)과 흰 원(생 삼각측량 재투영)이 벌어진 픽셀 거리.
    if let Some(px) = reprojection_px {
        lines.push(format!("reproj {px:.1} px"));
    }
    return lines;
}
