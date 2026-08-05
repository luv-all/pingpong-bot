//! 색공간 채널 AABB.

use pingpong_bot::vision::detect::colormask::{ColorSpace, ColormaskParams};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelRange {
    pub c0_min: u8,
    pub c0_max: u8,
    pub c1_min: u8,
    pub c1_max: u8,
    pub c2_min: u8,
    pub c2_max: u8,
}

/// 정렬된 채널 값에서 선형 보간 퍼센타일 (p ∈ [0, 100]).
pub fn channel_percentile(sorted: &[u8], p: f64) -> u8 {
    debug_assert!(!sorted.is_empty());
    if sorted.len() == 1 {
        return sorted[0];
    }
    let p = p.clamp(0.0, 100.0);
    let rank = p / 100.0 * (sorted.len() - 1) as f64;
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    if lo == hi {
        return sorted[lo];
    }
    let t = rank - lo as f64;
    return (f64::from(sorted[lo]) * (1.0 - t) + f64::from(sorted[hi]) * t).round() as u8;
}

impl ChannelRange {
    /// `trim_pct`: 양꼬리 절단 % (0 → min/max, 10 → p10..p90). 0..=49로 clamp.
    pub fn from_channels(chs: &[[u8; 3]], margin: u8, trim_pct: f64) -> Option<Self> {
        if chs.is_empty() {
            return None;
        }
        let trim = trim_pct.clamp(0.0, 49.0);
        let p_lo = trim;
        let p_hi = 100.0 - trim;
        let mut lo = [0u8; 3];
        let mut hi = [0u8; 3];
        for i in 0..3 {
            let mut vals: Vec<u8> = chs.iter().map(|c| c[i]).collect();
            vals.sort_unstable();
            lo[i] = channel_percentile(&vals, p_lo);
            hi[i] = channel_percentile(&vals, p_hi);
            if lo[i] > hi[i] {
                std::mem::swap(&mut lo[i], &mut hi[i]);
            }
        }
        return Some(Self {
            c0_min: lo[0].saturating_sub(margin),
            c0_max: hi[0].saturating_add(margin),
            c1_min: lo[1].saturating_sub(margin),
            c1_max: hi[1].saturating_add(margin),
            c2_min: lo[2].saturating_sub(margin),
            c2_max: hi[2].saturating_add(margin),
        });
    }

    pub fn to_params(self, space: ColorSpace) -> ColormaskParams {
        return ColormaskParams {
            space,
            c0_min: self.c0_min,
            c0_max: self.c0_max,
            c1_min: self.c1_min,
            c1_max: self.c1_max,
            c2_min: self.c2_min,
            c2_max: self.c2_max,
        };
    }
}
