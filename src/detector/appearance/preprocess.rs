//! 색 게이트 **앞**에 붙는 BGR → BGR 보정 축.
//!
//! 캠이 warm 캐스트라 흰 테이블·목재가 주황 상자를 공유하는 문제를 소프트웨어로 눌러본다.
//! 어느 것이 이기는지는 `eval-colormask`가 스틸 GT로 정한다 — 여기서는 후보만 제공한다.
//! 카메라 하드웨어 WB 재설정은 비범위.

use anyhow::Result;
use opencv::core::{Mat, Size, Vector};
use opencv::imgproc;
use opencv::prelude::*;

/// `WarmPushback` 채널 게인 (B, G, R) — 황·적을 눌러 주황 배경을 줄인다.
const WARM_PUSHBACK_GAINS: [f64; 3] = [1.10, 1.00, 0.85];
/// `ClaheV` clip limit / tile.
const CLAHE_CLIP_LIMIT: f64 = 2.0;
const CLAHE_TILE: i32 = 8;
/// `Bilateral` 지름 / sigma.
const BILATERAL_DIAMETER: i32 = 5;
const BILATERAL_SIGMA: f64 = 50.0;
/// `Gauss` 커널 (홀수).
const GAUSS_KSIZE: i32 = 3;
/// deuteranope 시뮬 행렬 (Machado et al. severity 1.0), **BGR 순서**.
/// sRGB에서 바로 적용하는 근사 — 실험 축이므로 선형화는 생략한다.
const DEUTERANOPE_BGR: [[f64; 3]; 3] = [
    [0.968_881, 0.042_940, -0.011_820],
    [0.047_413, 0.672_501, 0.280_085],
    [-0.227_968, 0.860_646, 0.367_322],
];

/// 전처리 후보. `eval-colormask` 그리드의 A축.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Default,
    clap::ValueEnum,
    serde::Serialize,
    serde::Deserialize,
)]
#[value(rename_all = "snake_case")]
#[serde(rename_all = "snake_case")]
pub enum Preprocess {
    /// 베이스라인 — 그대로.
    #[default]
    None,
    /// gray-world 화이트밸런스.
    GrayWorld,
    /// 고정 게인으로 황·적 바이어스 억제.
    WarmPushback,
    /// Lab L 채널 CLAHE — 작은 공·하이라이트 대비.
    ClaheV,
    /// bilateral — 노이즈↓, 엣지 유지.
    Bilateral,
    /// 작은 Gaussian — 센서 노이즈.
    Gauss,
    /// 색맹(deuteranope) 시뮬 축.
    CbSim,
}

impl Preprocess {
    pub fn all() -> [Self; 7] {
        return [
            Self::None,
            Self::GrayWorld,
            Self::WarmPushback,
            Self::ClaheV,
            Self::Bilateral,
            Self::Gauss,
            Self::CbSim,
        ];
    }

    pub fn as_str(self) -> &'static str {
        return match self {
            Self::None => "none",
            Self::GrayWorld => "gray_world",
            Self::WarmPushback => "warm_pushback",
            Self::ClaheV => "clahe_v",
            Self::Bilateral => "bilateral",
            Self::Gauss => "gauss",
            Self::CbSim => "cb_sim",
        };
    }

    /// `CV_8UC3` in / out. 크기·타입은 보존한다.
    pub fn apply(self, bgr: &Mat) -> Result<Mat> {
        return match self {
            Self::None => Ok(bgr.try_clone()?),
            Self::GrayWorld => gray_world(bgr),
            Self::WarmPushback => scale_channels(bgr, WARM_PUSHBACK_GAINS),
            Self::ClaheV => clahe_lightness(bgr),
            Self::Bilateral => bilateral(bgr),
            Self::Gauss => gauss(bgr),
            Self::CbSim => color_matrix(bgr, DEUTERANOPE_BGR),
        };
    }
}

impl std::fmt::Display for Preprocess {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(self.as_str());
    }
}

impl std::str::FromStr for Preprocess {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        for p in Self::all() {
            if p.as_str() == s {
                return Ok(p);
            }
        }
        return Err(format!("unknown preprocess: {s}"));
    }
}

/// 채널 평균을 맞추는 게인.
fn gray_world(bgr: &Mat) -> Result<Mat> {
    let mean = opencv::core::mean(bgr, &Mat::default())?;
    let avg = (mean[0] + mean[1] + mean[2]) / 3.0;
    let gain = |m: f64| -> f64 {
        if m <= f64::EPSILON {
            return 1.0;
        }
        return avg / m;
    };
    return scale_channels(bgr, [gain(mean[0]), gain(mean[1]), gain(mean[2])]);
}

/// 채널별 게인 (saturate).
fn scale_channels(bgr: &Mat, gains: [f64; 3]) -> Result<Mat> {
    let mut channels = Vector::<Mat>::new();
    opencv::core::split(bgr, &mut channels)?;
    let mut scaled = Vector::<Mat>::new();
    for (i, gain) in gains.iter().enumerate() {
        let mut out = Mat::default();
        channels
            .get(i)?
            .convert_to(&mut out, opencv::core::CV_8U, *gain, 0.0)?;
        scaled.push(out);
    }
    let mut out = Mat::default();
    opencv::core::merge(&scaled, &mut out)?;
    return Ok(out);
}

/// Lab L 채널만 CLAHE.
fn clahe_lightness(bgr: &Mat) -> Result<Mat> {
    let mut lab = Mat::default();
    imgproc::cvt_color(
        bgr,
        &mut lab,
        imgproc::COLOR_BGR2Lab,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    let mut channels = Vector::<Mat>::new();
    opencv::core::split(&lab, &mut channels)?;
    let mut clahe = imgproc::create_clahe(CLAHE_CLIP_LIMIT, Size::new(CLAHE_TILE, CLAHE_TILE))?;
    let mut lightness = Mat::default();
    clahe.apply(&channels.get(0)?, &mut lightness)?;
    channels.set(0, lightness)?;
    let mut merged = Mat::default();
    opencv::core::merge(&channels, &mut merged)?;
    let mut out = Mat::default();
    imgproc::cvt_color(
        &merged,
        &mut out,
        imgproc::COLOR_Lab2BGR,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    return Ok(out);
}

fn bilateral(bgr: &Mat) -> Result<Mat> {
    let mut out = Mat::default();
    imgproc::bilateral_filter(
        bgr,
        &mut out,
        BILATERAL_DIAMETER,
        BILATERAL_SIGMA,
        BILATERAL_SIGMA,
        opencv::core::BORDER_DEFAULT,
    )?;
    return Ok(out);
}

fn gauss(bgr: &Mat) -> Result<Mat> {
    let mut out = Mat::default();
    imgproc::gaussian_blur(
        bgr,
        &mut out,
        Size::new(GAUSS_KSIZE, GAUSS_KSIZE),
        0.0,
        0.0,
        opencv::core::BORDER_DEFAULT,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    return Ok(out);
}

/// 화소별 3×3 선형 변환 (BGR).
fn color_matrix(bgr: &Mat, m: [[f64; 3]; 3]) -> Result<Mat> {
    let kernel = Mat::from_slice_2d(&[
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ])?;
    let mut out = Mat::default();
    opencv::core::transform(bgr, &mut out, &kernel)?;
    return Ok(out);
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::{CV_8UC3, Scalar, Size};

    /// 붉은 캐스트 패치 (B=60, G=120, R=220).
    fn warm_patch() -> Mat {
        return Mat::new_size_with_default(
            Size::new(32, 32),
            CV_8UC3,
            Scalar::new(60.0, 120.0, 220.0, 0.0),
        )
        .unwrap();
    }

    fn mean_bgr(m: &Mat) -> [f64; 3] {
        let s = opencv::core::mean(m, &Mat::default()).unwrap();
        return [s[0], s[1], s[2]];
    }

    fn spread(m: [f64; 3]) -> f64 {
        let hi = m.iter().cloned().fold(f64::MIN, f64::max);
        let lo = m.iter().cloned().fold(f64::MAX, f64::min);
        return hi - lo;
    }

    #[test]
    fn none_is_identity() {
        let src = warm_patch();
        let out = Preprocess::None.apply(&src).unwrap();
        assert_eq!(mean_bgr(&out), mean_bgr(&src));
    }

    #[test]
    fn gray_world_flattens_channel_means() {
        let src = warm_patch();
        let before = mean_bgr(&src);
        let after = mean_bgr(&Preprocess::GrayWorld.apply(&src).unwrap());
        assert!(
            spread(after) < spread(before),
            "{after:?} should be flatter than {before:?}"
        );
    }

    #[test]
    fn warm_pushback_reduces_red_bias() {
        let src = warm_patch();
        let before = mean_bgr(&src);
        let after = mean_bgr(&Preprocess::WarmPushback.apply(&src).unwrap());
        assert!(after[2] < before[2], "red {} -> {}", before[2], after[2]);
        assert!(after[0] > before[0], "blue {} -> {}", before[0], after[0]);
    }

    #[test]
    fn all_variants_preserve_size_and_type() {
        let src = warm_patch();
        for p in Preprocess::all() {
            let out = p.apply(&src).unwrap_or_else(|e| panic!("{p}: {e}"));
            assert_eq!(out.size().unwrap(), src.size().unwrap(), "{p}");
            assert_eq!(out.typ(), src.typ(), "{p}");
        }
    }

    #[test]
    fn cb_sim_changes_orange_but_keeps_range() {
        let src = warm_patch();
        let after = mean_bgr(&Preprocess::CbSim.apply(&src).unwrap());
        assert!(
            after != mean_bgr(&src),
            "colorblind sim should change the color"
        );
        for c in after {
            assert!((0.0..=255.0).contains(&c), "channel out of range: {c}");
        }
    }

    #[test]
    fn id_roundtrips_through_str() {
        for p in Preprocess::all() {
            assert_eq!(p.to_string().parse::<Preprocess>().unwrap(), p);
        }
        assert!("nope".parse::<Preprocess>().is_err());
    }
}
