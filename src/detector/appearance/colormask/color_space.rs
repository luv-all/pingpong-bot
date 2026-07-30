use anyhow::Result;
use clap::ValueEnum;
use opencv::core::{Mat, Vector};
use opencv::imgproc;

use super::ParseColorSpaceError;

/// 색 게이트가 판정하는 3축 공간. `eval-colormask` 그리드의 C축.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, ValueEnum, serde::Serialize, serde::Deserialize,
)]
pub enum ColorSpace {
    #[default]
    #[value(name = "ycrcb")]
    #[serde(rename = "ycrcb")]
    Ycrcb,
    #[value(name = "hsv")]
    #[serde(rename = "hsv")]
    Hsv,
    #[value(name = "lab")]
    #[serde(rename = "lab")]
    Lab,
    /// 커스텀: HSV `H` + Lab `a*` `b*` — 명도를 떼고 주황 chroma만 본다.
    #[value(name = "custom_h_ab")]
    #[serde(rename = "custom_h_ab")]
    CustomHab,
}

impl ColorSpace {
    pub fn all() -> [Self; 4] {
        return [Self::Ycrcb, Self::Hsv, Self::Lab, Self::CustomHab];
    }

    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Ycrcb => "ycrcb",
            Self::Hsv => "hsv",
            Self::Lab => "lab",
            Self::CustomHab => "custom_h_ab",
        };
    }

    /// 채널 이름 — HUD·산점도 축 라벨.
    pub fn channel_names(self) -> [&'static str; 3] {
        return match self {
            Self::Ycrcb => ["Y", "Cr", "Cb"],
            Self::Hsv => ["H", "S", "V"],
            Self::Lab => ["L", "a*", "b*"],
            Self::CustomHab => ["H", "a*", "b*"],
        };
    }

    /// BGR(`CV_8UC3`) → 이 공간의 3채널 `CV_8UC3`.
    pub fn convert(self, bgr: &Mat) -> Result<Mat> {
        return match self {
            Self::Ycrcb => cvt(bgr, imgproc::COLOR_BGR2YCrCb),
            Self::Hsv => cvt(bgr, imgproc::COLOR_BGR2HSV),
            Self::Lab => cvt(bgr, imgproc::COLOR_BGR2Lab),
            Self::CustomHab => custom_h_ab(bgr),
        };
    }
}

fn cvt(bgr: &Mat, code: i32) -> Result<Mat> {
    let mut out = Mat::default();
    imgproc::cvt_color(
        bgr,
        &mut out,
        code,
        0,
        opencv::core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    return Ok(out);
}

/// HSV의 H + Lab의 a*, b* 를 한 Mat으로 합친다.
fn custom_h_ab(bgr: &Mat) -> Result<Mat> {
    let hsv = cvt(bgr, imgproc::COLOR_BGR2HSV)?;
    let lab = cvt(bgr, imgproc::COLOR_BGR2Lab)?;
    let mut hsv_channels = Vector::<Mat>::new();
    let mut lab_channels = Vector::<Mat>::new();
    opencv::core::split(&hsv, &mut hsv_channels)?;
    opencv::core::split(&lab, &mut lab_channels)?;
    let merged = Vector::<Mat>::from_iter([
        hsv_channels.get(0)?,
        lab_channels.get(1)?,
        lab_channels.get(2)?,
    ]);
    let mut out = Mat::default();
    opencv::core::merge(&merged, &mut out)?;
    return Ok(out);
}

impl std::str::FromStr for ColorSpace {
    type Err = ParseColorSpaceError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let lower = s.to_ascii_lowercase();
        for space in Self::all() {
            if space.as_str() == lower {
                return Ok(space);
            }
        }
        return Err(ParseColorSpaceError);
    }
}

impl std::fmt::Display for ColorSpace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(self.as_str());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::{CV_8UC3, Scalar, Size};
    use opencv::prelude::*;

    fn orange() -> Mat {
        return Mat::new_size_with_default(
            Size::new(8, 8),
            CV_8UC3,
            Scalar::new(30.0, 120.0, 235.0, 0.0),
        )
        .unwrap();
    }

    #[test]
    fn all_spaces_convert_to_three_channel_u8() {
        let src = orange();
        for space in ColorSpace::all() {
            let out = space
                .convert(&src)
                .unwrap_or_else(|e| panic!("{space}: {e}"));
            assert_eq!(out.channels(), 3, "{space}");
            assert_eq!(out.depth(), opencv::core::CV_8U, "{space}");
            assert_eq!(out.size().unwrap(), src.size().unwrap(), "{space}");
        }
    }

    #[test]
    fn custom_hab_takes_hue_from_hsv_and_ab_from_lab() {
        let src = orange();
        let hsv: opencv::core::Vec3b = *ColorSpace::Hsv.convert(&src).unwrap().at_2d(0, 0).unwrap();
        let lab: opencv::core::Vec3b = *ColorSpace::Lab.convert(&src).unwrap().at_2d(0, 0).unwrap();
        let custom: opencv::core::Vec3b = *ColorSpace::CustomHab
            .convert(&src)
            .unwrap()
            .at_2d(0, 0)
            .unwrap();
        assert_eq!(custom[0], hsv[0], "c0 = HSV H");
        assert_eq!(custom[1], lab[1], "c1 = Lab a*");
        assert_eq!(custom[2], lab[2], "c2 = Lab b*");
    }

    #[test]
    fn str_roundtrip_covers_new_spaces() {
        for space in ColorSpace::all() {
            assert_eq!(space.to_string().parse::<ColorSpace>().unwrap(), space);
        }
        // 기존 표기 호환
        assert_eq!("YCrCb".parse::<ColorSpace>().unwrap(), ColorSpace::Ycrcb);
        assert_eq!("HSV".parse::<ColorSpace>().unwrap(), ColorSpace::Hsv);
        assert!("rgb".parse::<ColorSpace>().is_err());
    }

    #[test]
    fn serde_ids_stay_stable_for_existing_files() {
        assert_eq!(
            serde_json::to_string(&ColorSpace::Ycrcb).unwrap(),
            "\"ycrcb\""
        );
        assert_eq!(serde_json::to_string(&ColorSpace::Hsv).unwrap(), "\"hsv\"");
        assert_eq!(
            serde_json::to_string(&ColorSpace::CustomHab).unwrap(),
            "\"custom_h_ab\""
        );
        let back: ColorSpace = serde_json::from_str("\"lab\"").unwrap();
        assert_eq!(back, ColorSpace::Lab);
    }
}
