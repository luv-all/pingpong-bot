//! 프레임 한 장을 먹이면 [`Trajectory`]가 나온다.
//!
//! ```text
//! Frame ─▶ Detector ─▶ Candidate ─▶ Ekf ─▶ Trajectory ─▶ 기구학
//! ```
//!
//! 이미지는 이 경계를 넘지 않는다. 비전은 로봇 도달 범위도 접수 평면도 모른다.
//!
//! [`Vision`]은 이 둘을 한 스레드에서 묶은 것이다. 실기 경로는 카메라마다 [`Detector`]를
//! 따로 돌리고 [`Ekf`]만 공유하므로 여기를 쓰지 않는다 — 검출이 직렬화되면 안 된다.
//!
//! 설계 근거는 `docs/better-vision.md`.

pub mod contract;
pub mod detect;
pub mod fit;
pub mod seed;
pub mod trace;
pub mod trigger;
pub mod triggers;

pub use contract::{State, Track, Trajectory};
pub use detect::{Candidate, Detector, Layer};
pub use fit::{Fit, Outcome};
pub use trace::Trace;
pub use trigger::Trigger;

use std::time::{Duration, Instant};

use anyhow::Result;

use crate::camera::{self, Calibration, Frame};

/// 검출기와 필터를 한 스레드에서 묶는다. 클립 도구와 진단이 쓴다.
pub struct Vision {
    detectors: Vec<(camera::Id, Detector)>,
    fit: Fit,
    /// 첫 프레임 시각. 클립이든 실기든 거기서 t=0 이 시작한다.
    origin: Option<Instant>,
    last_outcome: Option<Outcome>,
    last_found: Option<Candidate>,
}

impl Vision {
    pub fn load(calibration: &Calibration, trigger: Box<dyn Trigger>) -> Result<Self> {
        let detectors = calibration
            .cameras
            .iter()
            .map(|params| Ok((params.camera_id, Detector::for_camera(params)?)))
            .collect::<Result<Vec<_>>>()?;
        return Ok(Self {
            fit: Fit::new(calibration, trigger),
            detectors,
            origin: None,
            last_outcome: None,
            last_found: None,
        });
    }

    pub fn origin(&self) -> Option<Instant> {
        return self.origin;
    }

    pub fn fit(&self) -> &Fit {
        return &self.fit;
    }

    /// 직전 [`Self::feed`]의 필터 판정. 진단용.
    pub fn last_outcome(&self) -> Option<Outcome> {
        return self.last_outcome;
    }

    /// 직전 [`Self::feed`]가 고른 후보. 진단용.
    pub fn last_found(&self) -> Option<Candidate> {
        return self.last_found;
    }

    /// 예측이 만들어졌으면 그 순간의 계약을 돌려준다.
    pub fn feed(&mut self, frame: &Frame) -> Result<Option<Trajectory>> {
        let t = self.elapsed(frame);
        let Some(detector) = self.detector_for(frame.camera_id) else {
            return Ok(None);
        };
        let found = detector.detect(frame, None)?;
        self.absorb(frame.camera_id, found, t);
        return Ok(self.trajectory());
    }

    /// 툴 전용. 단계별 마스크를 남기며 돈다.
    pub fn feed_traced(&mut self, frame: &Frame) -> Result<(Option<Trajectory>, Trace)> {
        let t = self.elapsed(frame);
        let Some(detector) = self.detector_for(frame.camera_id) else {
            return Ok((None, Trace::default()));
        };
        let (stages, candidates) = detector.trace(frame, None)?;
        let found = best(&candidates);
        self.absorb(frame.camera_id, found, t);
        return Ok((
            self.trajectory(),
            Trace {
                stages,
                candidates,
                chosen: found,
                outcome: self.last_outcome,
            },
        ));
    }

    /// 지금 계약. 첫 프레임 전이거나 트리거 전이면 `None`.
    pub fn trajectory(&self) -> Option<Trajectory> {
        return self.fit.trajectory(self.origin?);
    }

    fn detector_for(&mut self, camera_id: camera::Id) -> Option<&mut Detector> {
        return self
            .detectors
            .iter_mut()
            .find(|(id, _)| *id == camera_id)
            .map(|(_, detector)| detector);
    }

    /// 첫 프레임 시각을 기준으로 한 경과. 첫 호출에서 기준을 잡는다.
    fn elapsed(&mut self, frame: &Frame) -> Duration {
        let origin = *self.origin.get_or_insert(frame.timestamp);
        return frame.timestamp.saturating_duration_since(origin);
    }

    fn absorb(&mut self, camera_id: camera::Id, found: Option<Candidate>, t: Duration) {
        self.last_found = found;
        self.last_outcome = Some(self.fit.observe(camera_id, found, t));
    }
}

/// 편차가 가장 작은 후보.
fn best(candidates: &[(Candidate, f64)]) -> Option<Candidate> {
    return candidates
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(candidate, _)| *candidate);
}
