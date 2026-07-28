//! YCrCb / HSV 색 마스크로 공 검출.

use anyhow::{Context, Result, ensure};
use clap::ValueEnum;
use opencv::core::{Point, Scalar, Vector};
use opencv::imgproc;
use opencv::prelude::*;

use super::super::BallDetector;
use super::super::candidate::{Candidate, candidates_from_contours};
use super::super::fuse::CandidateGenerator;
use super::super::motion::draw_candidate_contour;
use super::super::scorer::Scorer;
use crate::PixelPoint;
use crate::camera::Frame;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, ValueEnum, serde::Serialize, serde::Deserialize,
)]
#[value(rename_all = "lower")]
#[serde(rename_all = "lowercase")]
pub enum ColorSpace {
    #[default]
    Ycrcb,
    Hsv,
}

impl std::str::FromStr for ColorSpace {
    type Err = ParseColorSpaceError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        return match s {
            "ycrcb" | "YCrCb" => Ok(Self::Ycrcb),
            "hsv" | "HSV" => Ok(Self::Hsv),
            _ => Err(ParseColorSpaceError),
        };
    }
}

impl std::fmt::Display for ColorSpace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(match self {
            Self::Ycrcb => "ycrcb",
            Self::Hsv => "hsv",
        });
    }
}

/// [`ColorSpace`] 파싱 실패.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseColorSpaceError;

impl std::fmt::Display for ParseColorSpaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str("expected ycrcb|hsv");
    }
}

impl std::error::Error for ParseColorSpaceError {}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ColormaskParams {
    pub space: ColorSpace,
    pub c0_min: u8,
    pub c0_max: u8,
    pub c1_min: u8,
    pub c1_max: u8,
    pub c2_min: u8,
    pub c2_max: u8,
}

impl ColormaskParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.c0_min <= self.c0_max, "c0_min <= c0_max");
        ensure!(self.c1_min <= self.c1_max, "c1_min <= c1_max");
        ensure!(self.c2_min <= self.c2_max, "c2_min <= c2_max");
        return Ok(());
    }
}

/// tune-colormask 픽 샘플 — BGR 트리플. detector는 무시.
pub type ColormaskBgr = [u8; 3];

/// 한 카메라의 colormask 엔트리 (`camera_id` + flatten params + optional samples).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ColormaskCam {
    pub camera_id: crate::CameraId,
    #[serde(flatten)]
    pub params: ColormaskParams,
    /// `[[B,G,R], …]` — 픽셀 좌표 없음 (공은 움직이므로 색만 SSOT).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub samples: Vec<ColormaskBgr>,
}

/// 멀티캠 colormask 번들 (`data/colormask.json`).
#[derive(Debug, Clone, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct ColormaskSet {
    pub cameras: Vec<ColormaskCam>,
}

impl ColormaskSet {
    pub fn params(&self, camera_id: crate::CameraId) -> Option<&ColormaskParams> {
        return self
            .cameras
            .iter()
            .find(|c| c.camera_id == camera_id)
            .map(|c| &c.params);
    }

    pub fn samples(&self, camera_id: crate::CameraId) -> Option<&[ColormaskBgr]> {
        return self
            .cameras
            .iter()
            .find(|c| c.camera_id == camera_id)
            .map(|c| c.samples.as_slice());
    }

    pub fn upsert(
        &mut self,
        camera_id: crate::CameraId,
        params: ColormaskParams,
        samples: Vec<ColormaskBgr>,
    ) {
        if let Some(slot) = self.cameras.iter_mut().find(|c| c.camera_id == camera_id) {
            slot.params = params;
            slot.samples = samples;
            return;
        }
        self.cameras.push(ColormaskCam {
            camera_id,
            params,
            samples,
        });
        self.cameras.sort_by_key(|c| c.camera_id);
    }
}

/// JSON에서 [`ColormaskSet`] 로드. 파일 없으면 에러.
pub fn load_colormask_set(path: &std::path::Path) -> Result<ColormaskSet> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("colormask 읽기: {}", path.display()))?;
    let set: ColormaskSet = serde_json::from_str(&text)
        .with_context(|| format!("colormask JSON: {}", path.display()))?;
    for cam in &set.cameras {
        cam.params.validate()?;
    }
    return Ok(set);
}

/// 있으면 로드, 없으면 빈 셋 (upsert 시작용).
pub fn load_colormask_set_or_empty(path: &std::path::Path) -> Result<ColormaskSet> {
    if !path.is_file() {
        return Ok(ColormaskSet::default());
    }
    return load_colormask_set(path);
}

/// [`ColormaskSet`]을 pretty JSON으로 저장 (부모 dir 생성).
pub fn save_colormask_set(path: &std::path::Path, set: &ColormaskSet) -> Result<()> {
    for cam in &set.cameras {
        cam.params.validate()?;
    }
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    let json = serde_json::to_string_pretty(set)?;
    std::fs::write(path, format!("{json}\n"))?;
    return Ok(());
}

pub struct ColormaskDetector {
    params: ColormaskParams,
    last_area: Option<f64>,
}

impl ColormaskDetector {
    pub fn new(params: ColormaskParams) -> Self {
        return Self {
            params,
            last_area: None,
        };
    }

    pub fn space(&self) -> ColorSpace {
        return self.params.space;
    }

    /// 색 마스크 (단일 채널). cascade·디버그용.
    pub fn color_mask(&self, frame: &Frame) -> Option<Mat> {
        let mut converted = Mat::default();
        let code = match self.params.space {
            ColorSpace::Ycrcb => imgproc::COLOR_BGR2YCrCb,
            ColorSpace::Hsv => imgproc::COLOR_BGR2HSV,
        };
        if imgproc::cvt_color(
            &frame.image,
            &mut converted,
            code,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return None;
        }

        let lo = Scalar::new(
            f64::from(self.params.c0_min),
            f64::from(self.params.c1_min),
            f64::from(self.params.c2_min),
            0.0,
        );
        let hi = Scalar::new(
            f64::from(self.params.c0_max),
            f64::from(self.params.c1_max),
            f64::from(self.params.c2_max),
            0.0,
        );
        let mut mask = Mat::default();
        if opencv::core::in_range(&converted, &lo, &hi, &mut mask).is_err() {
            return None;
        }
        return Some(mask);
    }

    /// 검출 + 색 마스크(BGR). 선택 컨투어는 초록.
    /// hard cut은 호출측 `Scorer`를 쓴다.
    pub fn detect_debug(&mut self, frame: &Frame, scorer: &Scorer) -> (Option<PixelPoint>, Mat) {
        self.last_area = None;
        let empty = || empty_bgr(frame);
        let Some(mask) = self.color_mask(frame) else {
            return (None, empty());
        };

        let mut mask_bgr = Mat::default();
        if imgproc::cvt_color(
            &mask,
            &mut mask_bgr,
            imgproc::COLOR_GRAY2BGR,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )
        .is_err()
        {
            return (None, empty());
        }

        let cands = self.candidates_from_mask(&mask);
        let best = scorer.pick_best(&cands, |_| 0.0);
        if let Some(c) = best {
            self.last_area = Some(c.area);
            draw_candidate_contour(&mut mask_bgr, &c.contour);
            return (Some(c.pixel), mask_bgr);
        }
        return (None, mask_bgr);
    }

    fn candidates_from_mask(&self, mask: &Mat) -> Vec<Candidate> {
        let mut contours = Vector::<Vector<Point>>::new();
        if imgproc::find_contours(
            mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )
        .is_err()
        {
            return Vec::new();
        }
        return candidates_from_contours(&contours);
    }
}

impl CandidateGenerator for ColormaskDetector {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate> {
        let Some(mask) = self.color_mask(frame) else {
            return Vec::new();
        };
        return self.candidates_from_mask(&mask);
    }
}

impl BallDetector for ColormaskDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        let scorer = Scorer::from(&crate::detector::ScorerParams {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.55,
        });
        return self.detect_debug(frame, &scorer).0;
    }

    fn last_area(&self) -> Option<f64> {
        return self.last_area;
    }
}

fn empty_bgr(frame: &Frame) -> Mat {
    return Mat::zeros(frame.image.rows(), frame.image.cols(), frame.image.typ())
        .ok()
        .and_then(|m| m.to_mat().ok())
        .unwrap_or_default();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CameraId;
    use opencv::core::{CV_8UC3, Size};
    use std::time::Instant;

    #[test]
    fn colormask_finds_blob_center() {
        let mut img =
            Mat::new_size_with_default(Size::new(200, 200), CV_8UC3, Scalar::all(0.0)).unwrap();
        imgproc::circle(
            &mut img,
            Point::new(100, 80),
            15,
            Scalar::new(200.0, 200.0, 200.0, 0.0),
            -1,
            imgproc::LINE_8,
            0,
        )
        .unwrap();
        let frame = Frame::new(CameraId(0), img, Instant::now());
        let params = ColormaskParams {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
        };
        let mut det = ColormaskDetector::new(params);
        let pixel = det.detect(&frame).expect("should find blob");
        assert!((pixel.x - 100.0).abs() < 5.0, "x={}", pixel.x);
        assert!((pixel.y - 80.0).abs() < 5.0, "y={}", pixel.y);
    }

    #[test]
    fn colormask_json_roundtrip_keeps_samples() {
        let mut set = ColormaskSet::default();
        set.upsert(
            CameraId(0),
            ColormaskParams {
                space: ColorSpace::Hsv,
                c0_min: 10,
                c0_max: 20,
                c1_min: 30,
                c1_max: 40,
                c2_min: 50,
                c2_max: 60,
            },
            vec![[40u8, 120, 200]],
        );
        let json = serde_json::to_string(&set).unwrap();
        assert!(json.contains("\"samples\":[[40,120,200]]") || json.contains("[40, 120, 200]"));
        let loaded: ColormaskSet = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.samples(CameraId(0)).unwrap(), &[[40u8, 120, 200]]);
        // 구포맷(samples 없음)도 로드
        let legacy = r#"{"cameras":[{"camera_id":1,"space":"ycrcb","c0_min":1,"c0_max":2,"c1_min":3,"c1_max":4,"c2_min":5,"c2_max":6}]}"#;
        let legacy_set: ColormaskSet = serde_json::from_str(legacy).unwrap();
        assert!(legacy_set.samples(CameraId(1)).unwrap().is_empty());
    }
}
