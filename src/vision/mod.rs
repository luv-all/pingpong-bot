//! 프레임 한 장을 먹이면 [`Trajectory`]가 나온다.
//!
//! ```text
//! Frame ─▶ Detector ─▶ Candidate ─▶ Ekf ─▶ Trajectory ─▶ 기구학
//! ```
//!
//! 이미지는 이 경계를 넘지 않는다. 비전은 로봇 도달 범위도 접수 평면도 모른다.
//!
//! 설계 근거는 `docs/better-vision.md`.

pub mod contract;
pub mod detect;
pub mod ekf;
pub mod seed;
pub mod trace;
pub mod trigger;
pub mod triggers;

pub use contract::{State, Track, Trajectory};
pub use detect::{Candidate, Detector, Layer};
pub use ekf::{Ekf, Outcome};
pub use trace::Trace;
pub use trigger::Trigger;

use std::time::{Duration, Instant};

use anyhow::Result;

use crate::camera::{self, Calibration, Frame};

/// 캘리브와 검출기를 카메라가 직접 든다. 상위가 `Calibration`을 들면 `params(id)` 조회가
/// 매번 `Option`이라 불필요한 분기가 생긴다.
pub struct Camera {
    pub id: camera::Id,
    pub params: camera::Params,
    pub detector: Detector,
}

/// 단일 진입점.
pub struct Vision {
    /// 개수는 캘리브레이션 파일이 정한다.
    cameras: Vec<Camera>,
    ekf: Ekf,
    /// 시드 전에만 쓴다. 시드는 두 시선이 필요한데 프레임은 한 대씩 오기 때문이다.
    pending: Vec<(camera::Id, Candidate, Duration)>,
    /// 첫 프레임 시각. 클립이든 실기든 거기서 t=0 이 시작한다.
    origin: Option<Instant>,
    last_outcome: Option<Outcome>,
    last_detected: bool,
}

/// 시드 버퍼에 검출을 얼마나 들고 있을지.
pub const PENDING_TTL: Duration = Duration::from_millis(50);

impl Vision {
    /// 캘리브를 카메라들에게 나눠 주고 끝.
    pub fn load(calibration: &Calibration, trigger: Box<dyn Trigger>) -> Result<Self> {
        let cameras = calibration
            .cameras
            .iter()
            .map(|params| {
                let picker = detect::Picker::from_calib(params, detect::MIN_CIRCULARITY)?;
                let background = detect::Background::new(
                    detect::background::HISTORY,
                    detect::background::VAR_THRESHOLD,
                    detect::background::SCALE,
                    detect::background::LEARNING_RATE,
                )?;
                let volume = detect::Volume::from_calib(params)?;
                let color = detect::ColorBox::load(params.camera_id)?;
                return Ok(Camera {
                    id: params.camera_id,
                    params: params.clone(),
                    // 싸고 잘 거르는 것부터. 부피는 정적 AND 라 가장 싸다.
                    detector: Detector::new(
                        vec![Box::new(volume), Box::new(background), Box::new(color)],
                        picker,
                    ),
                });
            })
            .collect::<Result<Vec<_>>>()?;

        return Ok(Self {
            ekf: Ekf::new(trigger),
            cameras,
            pending: Vec::new(),
            origin: None,
            last_outcome: None,
            last_detected: false,
        });
    }

    pub fn origin(&self) -> Option<Instant> {
        return self.origin;
    }

    pub fn ekf(&self) -> &Ekf {
        return &self.ekf;
    }

    pub fn cameras(&self) -> &[Camera] {
        return &self.cameras;
    }

    /// 직전 [`Self::feed`]의 필터 판정. 진단용.
    pub fn last_outcome(&self) -> Option<Outcome> {
        return self.last_outcome;
    }

    /// 직전 [`Self::feed`]에서 검출이 있었나. 진단용.
    pub fn last_detected(&self) -> bool {
        return self.last_detected;
    }

    /// 예측이 만들어졌으면 그 순간의 계약을 돌려준다.
    pub fn feed(&mut self, frame: &Frame) -> Result<Option<Trajectory>> {
        let t = self.elapsed(frame);
        let Some(index) = self.cameras.iter().position(|c| c.id == frame.camera_id) else {
            return Ok(None);
        };
        let expect = None; // 트랙 깊이 기반 기대 반지름은 후속
        let found = self.cameras[index].detector.detect(frame, expect)?;
        self.last_detected = found.is_some();
        self.last_outcome = self.absorb(index, found, t);
        return Ok(self.trajectory());
    }

    /// 툴 전용. 단계별 마스크를 남기며 돈다.
    pub fn feed_traced(&mut self, frame: &Frame) -> Result<(Option<Trajectory>, Trace)> {
        let t = self.elapsed(frame);
        let Some(index) = self.cameras.iter().position(|c| c.id == frame.camera_id) else {
            return Ok((None, Trace::default()));
        };
        let (stages, candidates) = self.cameras[index].detector.trace(frame, None)?;
        let found = candidates
            .iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(c, _)| *c);
        self.last_detected = found.is_some();
        let outcome = self.absorb(index, found, t);
        self.last_outcome = outcome;
        return Ok((
            self.trajectory(),
            Trace {
                stages,
                candidates,
                chosen: found,
                outcome,
            },
        ));
    }

    /// 지금 계약. 첫 프레임 전이거나 트리거 전이면 `None`.
    pub fn trajectory(&self) -> Option<Trajectory> {
        return self.ekf.trajectory(self.origin?);
    }

    /// 첫 프레임 시각을 기준으로 한 경과. 첫 호출에서 기준을 잡는다.
    fn elapsed(&mut self, frame: &Frame) -> Duration {
        let origin = *self.origin.get_or_insert(frame.timestamp);
        return frame.timestamp.saturating_duration_since(origin);
    }

    /// 검출 하나를 시드나 보정으로 흘려보낸다.
    fn absorb(&mut self, index: usize, found: Option<Candidate>, t: Duration) -> Option<Outcome> {
        let Some(candidate) = found else {
            return None;
        };
        let id = self.cameras[index].id;
        if self.ekf.has_track() {
            return Some(self.ekf.observe(&self.cameras[index], candidate, t));
        }
        self.pending
            .retain(|(other, _, at)| *other != id && t.saturating_sub(*at) <= PENDING_TTL);
        self.pending.push((id, candidate, t));
        let views = fresh_views(&self.cameras, &self.pending);
        if views.len() >= 2 && self.ekf.seed(&views) {
            self.pending.clear();
            return Some(Outcome::Seeded);
        }
        return None;
    }
}

/// 호출자가 `cameras`(읽기)와 `ekf`(쓰기)를 동시에 빌려야 해서 자유 함수로 둔다.
fn fresh_views<'a>(
    cameras: &'a [Camera],
    pending: &[(camera::Id, Candidate, Duration)],
) -> Vec<(&'a Camera, Candidate, Duration)> {
    return pending
        .iter()
        .filter_map(|(id, candidate, at)| {
            let camera = cameras.iter().find(|c| c.id == *id)?;
            return Some((camera, *candidate, *at));
        })
        .collect();
}
