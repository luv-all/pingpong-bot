//! Appearance generator + [`Scorer`](+ optional [`MotionPrior`]) fusion.
//!
//! ```ignore
//! use pingpong_bot::{fuse, generators, track, ColormaskDetector, MotionPrior, Scorer};
//!
//! let det = fuse(
//!     ColormaskDetector::new(cfg),
//!     Scorer::shape(20.0, 20_000.0, 0.55),
//! )
//! .with_motion(MotionPrior::new());
//!
//! // 여러 appearance (FirstSurviving 순서)
//! let det = fuse(
//!     generators![colormask, contour],
//!     Scorer::shape(20.0, 20_000.0, 0.55).with_motion_weight(0.5),
//! )
//! .with_motion(MotionPrior::new());
//!
//! let mut tracked = track(det, 80);
//! ```

use super::candidate::Candidate;
use super::motion::{MotionPrior, draw_candidate_contour, mask_to_bgr};
use super::scorer::Scorer;
use super::BallDetector;
use crate::PixelPoint;
use crate::camera::Frame;
use opencv::prelude::*;

/// 프레임 → 공 후보 목록. (색·엣지 등 appearance)
pub trait CandidateGenerator: Send {
    fn generate(&mut self, frame: &Frame) -> Vec<Candidate>;
}

/// [`fuse`] 첫 인자 — 단일 generator · 동종 배열 · 이미 boxed 목록.
pub trait IntoCandidateGenerators {
    fn into_candidate_generators(self) -> Vec<Box<dyn CandidateGenerator>>;
}

impl<G> IntoCandidateGenerators for G
where
    G: CandidateGenerator + 'static,
{
    fn into_candidate_generators(self) -> Vec<Box<dyn CandidateGenerator>> {
        return vec![Box::new(self)];
    }
}

impl IntoCandidateGenerators for Vec<Box<dyn CandidateGenerator>> {
    fn into_candidate_generators(self) -> Vec<Box<dyn CandidateGenerator>> {
        return self;
    }
}

impl<const N: usize> IntoCandidateGenerators for [Box<dyn CandidateGenerator>; N] {
    fn into_candidate_generators(self) -> Vec<Box<dyn CandidateGenerator>> {
        return self.into_iter().collect();
    }
}

impl<G, const N: usize> IntoCandidateGenerators for [G; N]
where
    G: CandidateGenerator + 'static,
{
    fn into_candidate_generators(self) -> Vec<Box<dyn CandidateGenerator>> {
        return self.into_iter().map(|g| Box::new(g) as _).collect();
    }
}

/// 이종 appearance를 `fuse`에 넣을 때. `Box::new` 캐스트를 숨긴다.
///
/// ```ignore
/// fuse(generators![colormask, contour], scorer)
/// ```
#[macro_export]
macro_rules! generators {
    ($($g:expr),+ $(,)?) => {
        ::std::vec![
            $(
                ::std::boxed::Box::new($g)
                    as ::std::boxed::Box<dyn $crate::detector::CandidateGenerator>
            ),+
        ]
    };
}

/// generator(들) → scorer → best pixel.
///
/// generators는 앞에서부터 시도하고, Scorer를 통과한 첫 후보에서 멈춘다.
pub struct FuseDetector {
    generators: Vec<Box<dyn CandidateGenerator>>,
    pub scorer: Scorer,
    motion: Option<MotionPrior>,
}

impl FuseDetector {
    pub fn new(
        generators: impl IntoCandidateGenerators,
        scorer: Scorer,
    ) -> Self {
        return Self {
            generators: generators.into_candidate_generators(),
            scorer,
            motion: None,
        };
    }

    /// MotionPrior를 켠다. soft weight는 [`Scorer::with_motion_weight`].
    pub fn with_motion(mut self, motion: MotionPrior) -> Self {
        self.motion = Some(motion);
        return self;
    }

    /// `weight > 0`이면 prior를 켜고 scorer weight도 맞춘다. `0`이면 motion 끔.
    pub fn with_motion_weight(mut self, weight: f64) -> Self {
        self.scorer.motion_weight = weight;
        if weight > 0.0 {
            self.motion = Some(MotionPrior::new());
        } else {
            self.motion = None;
        }
        return self;
    }

    /// 검출 + (있으면) motion 마스크 BGR. 선택 컨투어 초록.
    pub fn detect_debug(&mut self, frame: &Frame) -> (Option<PixelPoint>, Option<Mat>) {
        let motion_mask = self.motion.as_mut().and_then(|m| m.update(frame));
        let overlap = |c: &Candidate| match &motion_mask {
            Some(mask) => MotionPrior::overlap(mask, c),
            None => 0.0,
        };

        let mut best = None;
        for generator in &mut self.generators {
            let cands = generator.generate(frame);
            if let Some(c) = self.scorer.pick_best(&cands, &overlap) {
                best = Some(c.clone());
                break;
            }
        }

        let debug = motion_mask.map(|m| {
            let mut bgr = mask_to_bgr(&m);
            if let Some(ref c) = best {
                draw_candidate_contour(&mut bgr, &c.contour);
            }
            bgr
        });

        return (best.map(|c| c.pixel), debug);
    }
}

impl BallDetector for FuseDetector {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint> {
        return self.detect_debug(frame).0;
    }
}

/// generators + scorer. motion은 [`.with_motion`](FuseDetector::with_motion) /
/// [`.with_motion_weight`](FuseDetector::with_motion_weight).
///
/// ```ignore
/// fuse(ColormaskDetector::new(cfg), Scorer::shape(20.0, 20_000.0, 0.55))
///     .with_motion(MotionPrior::new())
/// ```
pub fn fuse(generators: impl IntoCandidateGenerators, scorer: Scorer) -> FuseDetector {
    return FuseDetector::new(generators, scorer);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CameraId;
    use crate::detector::{ColorSpace, ColormaskConfig, ColormaskDetector, ContourDetector, ScorerParams};
    use opencv::core::{CV_8UC3, Mat, Point, Scalar, Size};
    use opencv::imgproc;
    use std::time::Instant;

    fn white_blob_frame() -> Frame {
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
        return Frame::new(CameraId(0), img, Instant::now());
    }

    #[test]
    fn fuse_dsl_single_generator_no_box() {
        let frame = white_blob_frame();
        let mut det = fuse(
            ColormaskDetector::new(ColormaskConfig {
                space: ColorSpace::Ycrcb,
                c0_min: 50,
                c0_max: 255,
                c1_min: 0,
                c1_max: 255,
                c2_min: 0,
                c2_max: 255,
                min_area_px: 20.0,
                max_area_px: 20_000.0,
            }),
            Scorer::shape(20.0, 20_000.0, 0.5),
        );
        let p = det.detect(&frame).expect("fuse hit");
        assert!((p.x - 100.0).abs() < 5.0);
        assert!((p.y - 80.0).abs() < 5.0);
    }

    #[test]
    fn fuse_dsl_generators_macro_and_motion_weight() {
        let frame = white_blob_frame();
        let colormask = ColormaskDetector::new(ColormaskConfig {
            space: ColorSpace::Ycrcb,
            c0_min: 50,
            c0_max: 255,
            c1_min: 0,
            c1_max: 255,
            c2_min: 0,
            c2_max: 255,
            min_area_px: 20.0,
            max_area_px: 20_000.0,
        });
        let contour = ContourDetector::new(ScorerParams {
            min_area_px: 20.0,
            max_area_px: 20_000.0,
            min_circularity: 0.5,
        });
        let mut det = fuse(
            crate::generators![colormask, contour],
            Scorer::shape(20.0, 20_000.0, 0.5),
        )
        .with_motion_weight(0.5);
        let p = det.detect(&frame).expect("fuse hit");
        assert!((p.x - 100.0).abs() < 5.0);
    }
}
