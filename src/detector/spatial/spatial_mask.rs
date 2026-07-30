//! floor-edge 컷 AND 테이블 복도 keep — 본선 검출기의 공간 게이트.

use anyhow::{Result, ensure};
use opencv::core::Scalar;
use opencv::prelude::*;

use super::floor_edge::FloorEdgeMask;
use super::table_corridor::TableCorridorMask;

/// 공간 keep 합성 (255=검출 허용).
#[derive(Clone)]
pub struct SpatialMask {
    pub keep: Mat,
    pub floor: FloorEdgeMask,
    pub corridor: Option<TableCorridorMask>,
    pub width: i32,
    pub height: i32,
}

impl SpatialMask {
    /// 복도 없이 바닥 컷만 (하위 트랙 · A/B용).
    pub fn floor_only(floor: FloorEdgeMask) -> Self {
        let keep = floor.keep.clone();
        let (width, height) = (floor.width, floor.height);
        return Self {
            keep,
            floor,
            corridor: None,
            width,
            height,
        };
    }

    /// 바닥 컷 **AND** 복도 keep.
    pub fn with_corridor(floor: FloorEdgeMask, corridor: TableCorridorMask) -> Result<Self> {
        ensure!(
            floor.width == corridor.width && floor.height == corridor.height,
            "mask size mismatch: floor {}x{} vs corridor {}x{}",
            floor.width,
            floor.height,
            corridor.width,
            corridor.height
        );
        let mut keep = Mat::default();
        opencv::core::bitwise_and(&floor.keep, &corridor.keep, &mut keep, &Mat::default())?;
        let (width, height) = (floor.width, floor.height);
        return Ok(Self {
            keep,
            floor,
            corridor: Some(corridor),
            width,
            height,
        });
    }

    /// keep=0 화소를 검게. 크기가 다르면 에러.
    pub fn apply_bgr(&self, bgr: &Mat) -> Result<Mat> {
        ensure!(
            bgr.cols() == self.width && bgr.rows() == self.height,
            "frame size {}x{} != mask {}x{}",
            bgr.cols(),
            bgr.rows(),
            self.width,
            self.height
        );
        let mut out = Mat::zeros(self.height, self.width, bgr.typ())?.to_mat()?;
        bgr.copy_to_masked(&mut out, &self.keep)?;
        return Ok(out);
    }

    /// 남긴 화소 비율 [%] — HUD용.
    pub fn keep_percent(&self) -> f64 {
        let total = self.width.saturating_mul(self.height).max(1);
        let kept = opencv::core::count_non_zero(&self.keep).unwrap_or(0);
        return 100.0 * f64::from(kept) / f64::from(total);
    }

    /// 컷 선 + 복도 hull 오버레이.
    pub fn draw_overlay(
        &self,
        img: &mut Mat,
        floor_color: Scalar,
        corridor_color: Scalar,
    ) -> Result<()> {
        self.floor.draw_edge_line(img, floor_color, 2)?;
        if let Some(corridor) = &self.corridor {
            corridor.draw_hull(img, corridor_color, 2)?;
        }
        return Ok(());
    }
}

impl From<FloorEdgeMask> for SpatialMask {
    fn from(floor: FloorEdgeMask) -> Self {
        return Self::floor_only(floor);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera;
    use crate::constants::table;
    use nalgebra::Vector3;

    /// 실제 리그(cam0 ≈ `(0.76, 2.75, 0.34)`)와 같은 계열 — 테이블 끝에서 길이 방향으로 본다.
    fn end_on_rig_like() -> camera::Params {
        let eye = Vector3::new(table::WIDTH_X * 0.5, table::LENGTH_Y + 0.05, 0.34);
        let target = Vector3::new(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z + 0.15);
        return camera::Params::look_at(
            camera::Id(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            1280,
            800,
            47.3_f64.to_radians(),
        );
    }

    #[test]
    fn corridor_and_floor_keeps_no_more_than_floor_alone() {
        let params = end_on_rig_like();
        let floor = FloorEdgeMask::from_params(camera::Id(0), &params).unwrap();
        let floor_only = SpatialMask::floor_only(floor.clone());
        let corridor = TableCorridorMask::from_params(&params, 1.0).unwrap();
        let combined = SpatialMask::with_corridor(floor, corridor).unwrap();

        let a = opencv::core::count_non_zero(&floor_only.keep).unwrap();
        let b = opencv::core::count_non_zero(&combined.keep).unwrap();
        assert!(b <= a, "AND must not grow keep: {b} > {a}");
        assert!(b > 0, "corridor AND floor should keep something");
        assert!(
            b < a,
            "corridor should cut more than floor alone: {b} vs {a}"
        );
        assert!(combined.keep_percent() <= floor_only.keep_percent());
    }

    #[test]
    fn apply_bgr_blacks_dropped_pixels() {
        let params = end_on_rig_like();
        let floor = FloorEdgeMask::from_params(camera::Id(0), &params).unwrap();
        let corridor = TableCorridorMask::from_params(&params, 1.0).unwrap();
        let mask = SpatialMask::with_corridor(floor, corridor).unwrap();
        let bgr = Mat::new_size_with_default(
            opencv::core::Size::new(1280, 800),
            opencv::core::CV_8UC3,
            Scalar::all(200.0),
        )
        .unwrap();
        let out = mask.apply_bgr(&bgr).unwrap();
        for y in 0..800 {
            for x in 0..1280 {
                let keep: u8 = *mask.keep.at_2d(y, x).unwrap();
                if keep == 0 {
                    let px: opencv::core::Vec3b = *out.at_2d(y, x).unwrap();
                    assert_eq!(px, opencv::core::Vec3b::from([0, 0, 0]));
                    return;
                }
            }
        }
        panic!("mask should drop at least one pixel");
    }

    #[test]
    fn apply_bgr_rejects_size_mismatch() {
        let params = end_on_rig_like();
        let floor = FloorEdgeMask::from_params(camera::Id(0), &params).unwrap();
        let mask = SpatialMask::floor_only(floor);
        let bgr = Mat::new_size_with_default(
            opencv::core::Size::new(320, 240),
            opencv::core::CV_8UC3,
            Scalar::all(0.0),
        )
        .unwrap();
        assert!(mask.apply_bgr(&bgr).is_err());
    }
}
