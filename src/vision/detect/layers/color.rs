//! 색 판별면 레이어. 양성·음성 표본으로 피팅한 평면으로 자른다.

use anyhow::{Result, bail};
use nalgebra::Matrix3;
use opencv::core;
use opencv::core::{Mat, Point, Vec3b};
use opencv::prelude::*;

use crate::Vector3;
use crate::camera::Frame;

use super::super::{Layer, Mask};

/// 임계를 잡을 때 남길 양성 비율. 색은 판별자가 아니라 후보 생성기라 재현율을 높게 잡는다.
pub const KEEP_RATIO: f64 = 0.99;

/// `w · bgr + b >= 0`이면 통과. 평면 하나다.
///
/// 상자(`inRange`)나 타원체와 달리 음성 표본으로도 피팅한다. 상자는 공을 더 담으려면 모든
/// 방향으로 커지는데 배경이 바로 옆에 붙어 있어서, 하한을 풀면 오검출이 5에서 382로 늘었다.
/// 판별면은 음성이 없는 방향으로만 민다.
///
/// 반공간이라 공 방향으로 아주 멀리 간 픽셀도 통과한다. 배경 차분이 정지 물체를 이미
/// 지웠으므로 남는 음성은 피부와 옷이고, 그게 한쪽에 몰려 있다는 가정이다.
pub struct ColorPlane {
    w: Vector3,
    b: f64,
    /// 프레임마다 재할당하지 않는다.
    on: Mat,
}

impl ColorPlane {
    /// Fisher 선형 판별. `w = Sw⁻¹ (μ_pos − μ_neg)`.
    ///
    /// 임계는 두 평균의 중점이 아니라 양성의 `keep_ratio` 분위수에 둔다. 중점은 두 분포를
    /// 절반씩 자르는데, 여기서는 공을 놓치는 쪽이 더 비싸다.
    pub fn fit(positive: &[[u8; 3]], negative: &[[u8; 3]], keep_ratio: f64) -> Result<Self> {
        if positive.len() < 2 || negative.len() < 2 {
            bail!(
                "표본 부족: 양성 {} 음성 {} (각각 2개 이상)",
                positive.len(),
                negative.len()
            );
        }
        let (mean_pos, scatter_pos) = moments(positive);
        let (mean_neg, scatter_neg) = moments(negative);

        // 리지 항으로 특이행렬을 막는다.
        let within = scatter_pos + scatter_neg + Matrix3::identity() * 1e-3;
        let Some(inverse) = within.try_inverse() else {
            bail!("클래스 내 산포가 특이하다. 표본이 한 점에 몰렸는지 확인");
        };
        let w = inverse * (mean_pos - mean_neg);
        if w.norm() <= f64::EPSILON {
            bail!("두 클래스 평균이 같다. 판별 방향이 없다");
        }
        let w = w.normalize();

        // 양성을 투영해 하위 (1 - keep_ratio) 지점을 임계로 잡는다.
        let mut projected: Vec<f64> = positive.iter().map(|p| w.dot(&bgr(*p))).collect();
        projected.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let drop = ((1.0 - keep_ratio.clamp(0.0, 1.0)) * projected.len() as f64).floor() as usize;
        let threshold = projected[drop.min(projected.len() - 1)];

        return Ok(Self {
            w,
            b: -threshold,
            on: Mat::default(),
        });
    }

    pub fn keep(&self, pixel: [u8; 3]) -> bool {
        return self.w.dot(&bgr(pixel)) + self.b >= 0.0;
    }
}

impl Layer for ColorPlane {
    fn name(&self) -> &'static str {
        return "color";
    }

    fn narrow(&mut self, frame: &Frame, mask: &mut Mask) -> Result<()> {
        // 켜진 픽셀만 본다. 배경 차분 뒤라 보통 프레임의 0.1 % 미만이다.
        core::find_non_zero(&*mask, &mut self.on)?;
        for i in 0..self.on.rows() {
            let at: Point = *self.on.at(i)?;
            let px: Vec3b = *frame.image.at_2d(at.y, at.x)?;
            if !self.keep([px[0], px[1], px[2]]) {
                *mask.at_2d_mut(at.y, at.x)? = 0u8;
            }
        }
        return Ok(());
    }
}

fn bgr(pixel: [u8; 3]) -> Vector3 {
    return Vector3::new(
        f64::from(pixel[0]),
        f64::from(pixel[1]),
        f64::from(pixel[2]),
    );
}

/// 평균과 산포 행렬 `Σ (x − μ)(x − μ)ᵀ`.
fn moments(samples: &[[u8; 3]]) -> (Vector3, Matrix3<f64>) {
    let n = samples.len() as f64;
    let mean = samples.iter().fold(Vector3::zeros(), |a, s| a + bgr(*s)) / n;
    let scatter = samples.iter().fold(Matrix3::zeros(), |a, s| {
        let d = bgr(*s) - mean;
        return a + d * d.transpose();
    });
    return (mean, scatter);
}

#[cfg(test)]
#[path = "color_tests.rs"]
mod tests;
