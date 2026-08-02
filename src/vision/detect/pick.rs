//! 마스크에서 공 하나를 고른다 — 캐스케이드의 종단.

use anyhow::{Result, bail};
use opencv::core::{Point, Vector};
use opencv::imgproc;

use crate::Point3;
use crate::camera::{self, Pixel};
use crate::constants::{BALL_RADIUS, table};

use super::{Candidate, Mask};

/// 캘리브에서 나온 반지름을 얼마나 벌려 밴드로 쓸지.
const RADIUS_MIN_SCALE: f64 = 0.5;
const RADIUS_MAX_SCALE: f64 = 1.8;

/// 원형도 하한. 순위에 안 쓰고 걸러내기만 하므로 느슨하게 잡는다.
///
/// 완벽한 원이어도 래스터화만으로 떨어진다 — 실측 r=3 px 에서 0.67, r=20 px 에서 0.87
/// (`pick_tests.rs`). 모션 블러가 여기서 더 깎는다.
pub const MIN_CIRCULARITY: f64 = 0.35;

/// 기대 반지름과의 편차로 후보를 고른다. 가장 큰 것을 고르면 팔이 공을 이긴다.
pub struct Picker {
    min_radius_px: f64,
    max_radius_px: f64,
    min_circularity: f64,
    /// 프레임마다 재할당하지 않는다.
    contours: Vector<Vector<Point>>,
}

impl Picker {
    /// 테이블 대표점에 공을 투영해 반지름 밴드를 잡는다.
    pub fn from_calib(params: &camera::Params, min_circularity: f64) -> Result<Self> {
        let z = table::SURFACE_Z + BALL_RADIUS;
        let (w, l) = (table::WIDTH_X, table::LENGTH_Y);
        let corners = [
            Point3::new(0.0, 0.0, z),
            Point3::new(w, 0.0, z),
            Point3::new(w, l, z),
            Point3::new(0.0, l, z),
            Point3::new(w * 0.5, l * 0.5, z),
        ];
        let focal = (params.fx + params.fy) * 0.5;
        let radii: Vec<f64> = corners
            .iter()
            .filter_map(|p| {
                let cam = params.rotation * p.coords + params.translation;
                (cam.z > 0.05).then(|| focal * BALL_RADIUS / cam.z)
            })
            .filter(|r| r.is_finite() && *r > 0.0)
            .collect();
        if radii.is_empty() {
            bail!("cam{}: 테이블 대표점이 전부 카메라 뒤", params.camera_id.0);
        }
        let lo = radii.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = radii.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        return Ok(Self {
            min_radius_px: (lo * RADIUS_MIN_SCALE).max(1.0),
            max_radius_px: hi * RADIUS_MAX_SCALE,
            min_circularity,
            contours: Vector::new(),
        });
    }

    /// 상대 편차. 0이면 정확.
    ///
    /// 원형도는 순위에 쓰지 않는다. 빠른 공일수록 모션 블러로 타원이 되어 불리해진다.
    /// [`Self::passes`]에서 하한으로만 쓴다.
    pub fn deviation(&self, c: &Candidate, expect: f64) -> f64 {
        if expect <= f64::EPSILON {
            return f64::INFINITY;
        }
        return (c.radius_px - expect).abs() / expect;
    }

    pub fn passes(&self, c: &Candidate) -> bool {
        return c.radius_px >= self.min_radius_px
            && c.radius_px <= self.max_radius_px
            && c.circularity >= self.min_circularity;
    }

    /// 밴드 중앙. `expect`가 없을 때 쓴다.
    fn fallback_expect(&self) -> f64 {
        return 0.5 * (self.min_radius_px + self.max_radius_px);
    }

    /// 살아남은 후보 전부와 각자의 편차. 남는 개수가 연관이 필요한지의 근거다.
    pub fn candidates(
        &mut self,
        mask: &Mask,
        expect: Option<f64>,
    ) -> Result<Vec<(Candidate, f64)>> {
        let expect = expect.unwrap_or_else(|| self.fallback_expect());
        imgproc::find_contours(
            mask,
            &mut self.contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )?;

        let mut out = Vec::new();
        for contour in self.contours.iter() {
            let Some(candidate) = measure(&contour)? else {
                continue;
            };
            if self.passes(&candidate) {
                let deviation = self.deviation(&candidate, expect);
                out.push((candidate, deviation));
            }
        }
        return Ok(out);
    }

    /// 밴드·원형도를 통과한 것 중 편차가 가장 작은 것.
    pub fn pick(&mut self, mask: &Mask, expect: Option<f64>) -> Result<Option<Candidate>> {
        return Ok(self
            .candidates(mask, expect)?
            .into_iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(c, _)| c));
    }
}

/// 윤곽 하나 → 중심·등가 반지름·원형도.
fn measure(contour: &Vector<Point>) -> Result<Option<Candidate>> {
    let area = imgproc::contour_area(contour, false)?;
    if area <= f64::EPSILON {
        return Ok(None);
    }
    let moments = imgproc::moments(contour, false)?;
    if moments.m00.abs() <= f64::EPSILON {
        return Ok(None);
    }
    let perimeter = imgproc::arc_length(contour, true)?;
    let circularity = if perimeter > f64::EPSILON {
        (4.0 * std::f64::consts::PI * area / (perimeter * perimeter)).min(1.0)
    } else {
        0.0
    };
    return Ok(Some(Candidate {
        pixel: Pixel::new(moments.m10 / moments.m00, moments.m01 / moments.m00),
        radius_px: (area / std::f64::consts::PI).sqrt(),
        circularity,
    }));
}

#[cfg(test)]
#[path = "pick_tests.rs"]
mod tests;
