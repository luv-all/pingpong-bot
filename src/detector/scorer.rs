//! 후보 점수 — hard cut + soft weight.

use super::candidate::Candidate;
use super::params::ScorerParams;

/// 후보 hard filter + 점수. (도메인 안이라 `Blob` 접두 없음.)
#[derive(Debug, Clone, PartialEq)]
pub struct Scorer {
    pub min_area_px: f64,
    pub max_area_px: f64,
    pub min_circularity: f64,
    /// motion overlap `[0,1]`에 곱해 `1 + weight * overlap`로 soft boost.
    /// `0`이면 motion 무시.
    pub motion_weight: f64,
}

impl Scorer {
    pub fn new(
        min_area_px: f64,
        max_area_px: f64,
        min_circularity: f64,
        motion_weight: f64,
    ) -> Self {
        return Self {
            min_area_px,
            max_area_px,
            min_circularity,
            motion_weight,
        };
    }

    /// hard cuts만. motion soft는 [`Self::with_motion_weight`].
    pub fn shape(min_area_px: f64, max_area_px: f64, min_circularity: f64) -> Self {
        return Self::new(min_area_px, max_area_px, min_circularity, 0.0);
    }

    pub fn with_motion_weight(mut self, weight: f64) -> Self {
        self.motion_weight = weight;
        return self;
    }

    /// hard cut 실패 시 `None`. 높을수록 좋음.
    pub fn score(&self, c: &Candidate, motion_overlap: f64) -> Option<f64> {
        if c.area < self.min_area_px || c.area > self.max_area_px {
            return None;
        }
        if c.circularity < self.min_circularity {
            return None;
        }
        let base = c.area * c.circularity.max(f64::EPSILON);
        let overlap = motion_overlap.clamp(0.0, 1.0);
        let motion_factor = 1.0 + self.motion_weight.max(0.0) * overlap;
        return Some(base * motion_factor);
    }

    /// 후보 중 최고 점수. 동점이면 앞선 것.
    pub fn pick_best<'a>(
        &self,
        candidates: &'a [Candidate],
        motion_overlap: impl Fn(&Candidate) -> f64,
    ) -> Option<&'a Candidate> {
        let mut best: Option<(f64, &Candidate)> = None;
        for c in candidates {
            let Some(s) = self.score(c, motion_overlap(c)) else {
                continue;
            };
            match best {
                Some((bs, _)) if bs >= s => {}
                _ => best = Some((s, c)),
            }
        }
        return best.map(|(_, c)| c);
    }
}

impl From<&ScorerParams> for Scorer {
    fn from(p: &ScorerParams) -> Self {
        return Self::shape(p.min_area_px, p.max_area_px, p.min_circularity);
    }
}

impl From<ScorerParams> for Scorer {
    fn from(p: ScorerParams) -> Self {
        return Self::from(&p);
    }
}
