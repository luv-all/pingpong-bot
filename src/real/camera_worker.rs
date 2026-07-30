//! 카메라 1대 워커 — 캡처 → (왜곡 보정) → 검출 → [`VisionEvent`].
//!
//! `FrameSource`와 `Detector`를 **단독 소유**한다. 바깥에서 검출기 상태(ROI 추적 등)를 만질
//! 방법이 없다.

use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Sender, TrySendError};
use pingpong_bot::camera;
use pingpong_bot::camera::FrameSource;
use pingpong_bot::detector::Detector;
use tracing::{debug, info_span, warn};

use super::{Shutdown, Throttle, VisionEvent};

/// `next_frame`은 최신 프레임을 즉시 돌려준다 (새 프레임을 기다리지 않는다). 같은 프레임을
/// 다시 검출하지 않도록 타임스탬프로 거르고, 새 프레임이 없으면 잠깐 쉰다.
const IDLE_POLL: Duration = Duration::from_millis(1);

/// `--debug` 진척 로그 주기.
const PROGRESS_PERIOD: Duration = Duration::from_secs(1);

/// 종료 요약용 카메라 통계.
#[derive(Debug, Clone, Copy, Default)]
pub struct CameraStats {
    pub camera_id: u8,
    /// 검출을 실제로 돌린 프레임 수 (중복 타임스탬프 제외).
    pub frames: u64,
    /// 공을 찾은 프레임 수.
    pub detections: u64,
    /// 추정 워커가 밀려 버린 이벤트 수.
    pub dropped: u64,
    /// 왜곡 보정 실패 수.
    pub undistort_failures: u64,
}

impl CameraStats {
    pub fn detection_rate(&self) -> f64 {
        return detection_rate(self.detections, self.frames);
    }
}

fn detection_rate(detections: u64, frames: u64) -> f64 {
    if frames == 0 {
        return 0.0;
    }
    return detections as f64 / frames as f64;
}

/// 카메라 워커를 띄운다. 반환 핸들을 join하면 통계가 나온다.
pub fn spawn(
    mut source: Box<dyn FrameSource>,
    mut detector: Box<Detector>,
    params: camera::Params,
    tx: Sender<VisionEvent>,
    shutdown: Shutdown,
) -> JoinHandle<CameraStats> {
    let camera_id = source.camera_id();
    // 커밋된 calibration은 dist가 비어 있다 (table-PnP). 그 경우 프레임당 remap을 통째로 건너뛴다.
    let needs_undistort = !params.dist.is_empty();

    return thread::spawn(move || {
        let _span = info_span!("cam", id = camera_id.0).entered();
        let mut stats = CameraStats {
            camera_id: camera_id.0,
            ..CameraStats::default()
        };
        let mut last_timestamp: Option<Instant> = None;
        let mut progress = Throttle::new(PROGRESS_PERIOD);
        let (mut last_frames, mut last_detections) = (0_u64, 0_u64);

        while !shutdown.is_down() {
            let Some(frame) = source.next_frame() else {
                // `ThreadedCapture`는 첫 프레임을 최대 8초 기다린 뒤 None을 준다. 조용히 끊으면
                // 요약에 `frames=0`만 남아 이유를 알 수 없다 — 장치 경합·잘못된 device 인덱스.
                warn!(
                    frames = stats.frames,
                    "프레임 소스 종료 — 이 카메라는 더 이상 프레임을 내지 않는다"
                );
                break;
            };
            if last_timestamp == Some(frame.timestamp) {
                thread::sleep(IDLE_POLL);
                continue;
            }
            last_timestamp = Some(frame.timestamp);

            let frame = if needs_undistort {
                match Detector::undistort(&frame, &params) {
                    Ok(undistorted) => undistorted,
                    Err(error) => {
                        stats.undistort_failures += 1;
                        warn!(%error, "undistort 실패 — 프레임 스킵");
                        continue;
                    }
                }
            } else {
                frame
            };

            stats.frames += 1;
            let pixel = detector.detect(&frame);
            if pixel.is_some() {
                stats.detections += 1;
            }

            match tx.try_send(VisionEvent { frame, pixel }) {
                Ok(()) => {}
                // 실시간 경로 — 밀린 프레임은 쓸모가 없다. 버리고 센다.
                Err(TrySendError::Full(_)) => stats.dropped += 1,
                Err(TrySendError::Disconnected(_)) => break,
            }

            // 검출률이 무너지는 걸 끝나고서가 아니라 도중에 보게 한다.
            if progress.ready() {
                let window_frames = stats.frames - last_frames;
                debug!(
                    fps = window_frames as f64 / PROGRESS_PERIOD.as_secs_f64(),
                    detection_rate =
                        detection_rate(stats.detections - last_detections, window_frames),
                    frames = stats.frames,
                    dropped = stats.dropped,
                    "카메라 진척"
                );
                last_frames = stats.frames;
                last_detections = stats.detections;
            }
        }
        return stats;
    });
}
