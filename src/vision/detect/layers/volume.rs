//! 공간 레이어. 공이 날 수 있는 부피 밖을 끈다.

use anyhow::Result;
use opencv::core::{Mat, Scalar};
use opencv::prelude::*;

use crate::Vector3;
use crate::camera::{self, Frame};
use crate::constants::table;

use super::super::{Layer, Mask};

/// 테이블 옆으로 벌릴 여유 [m].
pub const SIDE_MARGIN: f64 = 0.3;
/// 로봇 쪽으로 벌릴 여유 [m]. 공을 로봇 위치까지 따라가야 한다.
pub const ROBOT_MARGIN: f64 = 0.5;
/// 상판 아래 여유와 위쪽 비행 높이 [m].
pub const BELOW: f64 = 0.1;
pub const FLIGHT_BAND: f64 = 1.0;

/// 공이 날 수 있는 3D 부피 밖을 끄는 keep 마스크.
///
/// 화소마다 그 방향으로 광선을 쏘아 상자를 지나는지 본다 (슬래브 검사). 카메라가 고정이라
/// **한 번만 계산하므로** 화소당 비용을 내도 된다. 매 프레임 비용은 AND 한 번이다.
///
/// # 왜 꼭짓점 투영이 아닌가
///
/// 전에는 상자 여덟 꼭짓점을 투영해 볼록 껍질을 채웠다. 그건 상자가 카메라 **앞에 충분히
/// 떨어져 있을 때만** 맞다. 카메라를 테이블 끝으로 옮긴 뒤 가장 가까운 꼭짓점이 0.97 m 가
/// 되면서 원근 나눗셈이 폭발했다 — 실측 투영이 `px (-4144, 6585)` 같은 값이었고, 그런 점들의
/// 껍질은 화면을 통째로 덮는다. 화면에 보이던 경계는 테이블 실루엣이 아니라 그 거대한
/// 다각형이 프레임에 잘린 자국이었다.
///
/// 광선 검사는 카메라가 상자 안에 있어도 맞는다.
///
/// # 지금 리그에서는 거의 안 지운다
///
/// 실측 keep 이 cam0 86 %, cam1 100 % 다 (`diag_layout::how_much_does_the_volume_layer_remove`).
/// 두 카메라가 테이블 끝에서 부피를 정면으로 보고 있어서 화면이 곧 부피다. cam0 이 자르는
/// 14 % 는 아래쪽 바닥과 좌상단 모서리인데, 바닥은 떨어진 공이 구르는 자리라 공짜로 얻는
/// 값이 없진 않다.
///
/// 리그를 옆으로 옮기면 다시 의미가 생긴다. 지금 지우지 않는 이유는 이제 **맞기 때문**이다 —
/// 배치가 바뀌면 알아서 따라온다.
pub struct Volume {
    pub keep: Mask,
    /// 프레임마다 재할당하지 않는다.
    scratch: Mat,
}

impl Volume {
    pub fn from_calib(params: &camera::Params) -> Result<Self> {
        let lo = Vector3::new(-SIDE_MARGIN, -ROBOT_MARGIN, table::SURFACE_Z - BELOW);
        let hi = Vector3::new(
            table::WIDTH_X + SIDE_MARGIN,
            table::LENGTH_Y + SIDE_MARGIN,
            table::SURFACE_Z + FLIGHT_BAND,
        );
        // 카메라 원점과, 화소 방향을 월드로 되돌리는 회전.
        let eye = -params.rotation.transpose() * params.translation;
        let to_world = params.rotation.transpose();

        let (w, h) = (params.width as i32, params.height as i32);
        let mut keep =
            Mat::new_rows_cols_with_default(h, w, opencv::core::CV_8UC1, Scalar::all(0.0))?;
        for v in 0..h {
            for u in 0..w {
                let ray = to_world
                    * Vector3::new(
                        (f64::from(u) - params.cx) / params.fx,
                        (f64::from(v) - params.cy) / params.fy,
                        1.0,
                    );
                if hits_box(eye, ray, lo, hi) {
                    *keep.at_2d_mut::<u8>(v, u)? = 255;
                }
            }
        }
        return Ok(Self {
            keep,
            scratch: Mat::default(),
        });
    }

    /// keep 비율 [%]. 이 레이어가 얼마나 지우는지 보는 값.
    pub fn keep_ratio(&self) -> Result<f64> {
        let on = opencv::core::count_non_zero(&self.keep)?;
        let total = self.keep.rows() * self.keep.cols();
        return Ok(100.0 * f64::from(on) / f64::from(total.max(1)));
    }
}

/// 광선이 축 정렬 상자를 지나나 — 슬래브 검사.
///
/// 축마다 상자에 들어가고 나가는 매개변수 구간을 구해 교집합을 본다. 비면 안 지난다.
/// `origin` 이 상자 안이면 `t` 구간이 0 을 품으므로 그대로 참이 된다.
fn hits_box(origin: Vector3, direction: Vector3, lo: Vector3, hi: Vector3) -> bool {
    // 카메라 뒤는 안 본다 — `t >= 0` 만 센다.
    let (mut enter, mut exit) = (0.0_f64, f64::INFINITY);
    for axis in 0..3 {
        if direction[axis].abs() < 1e-12 {
            // 이 축으로 안 움직이면 시작부터 구간 안에 있어야 한다.
            if origin[axis] < lo[axis] || origin[axis] > hi[axis] {
                return false;
            }
            continue;
        }
        let inverse = 1.0 / direction[axis];
        let a = (lo[axis] - origin[axis]) * inverse;
        let b = (hi[axis] - origin[axis]) * inverse;
        enter = enter.max(a.min(b));
        exit = exit.min(a.max(b));
        if exit < enter {
            return false;
        }
    }
    return true;
}

impl Layer for Volume {
    fn name(&self) -> &'static str {
        return "volume";
    }

    fn narrow(&mut self, _frame: &Frame, mask: &mut Mask) -> Result<()> {
        opencv::core::bitwise_and(
            &*mask,
            &self.keep,
            &mut self.scratch,
            &opencv::core::no_array(),
        )?;
        std::mem::swap(mask, &mut self.scratch);
        return Ok(());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_box() -> (Vector3, Vector3) {
        return (Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn a_ray_through_the_box_hits() {
        let (lo, hi) = unit_box();
        let eye = Vector3::new(0.5, 0.5, -3.0);
        assert!(hits_box(eye, Vector3::new(0.0, 0.0, 1.0), lo, hi));
        assert!(
            !hits_box(eye, Vector3::new(0.0, 0.0, -1.0), lo, hi),
            "뒤로 쏘면 안 맞는다"
        );
        assert!(
            !hits_box(eye, Vector3::new(1.0, 0.0, 0.0), lo, hi),
            "옆으로 지나가면 안 맞는다"
        );
    }

    /// 카메라가 상자 안에 있어도 맞아야 한다 — 꼭짓점 투영이 무너지던 자리다.
    #[test]
    fn a_ray_starting_inside_the_box_hits() {
        let (lo, hi) = unit_box();
        let inside = Vector3::new(0.5, 0.5, 0.5);
        for direction in [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, -1.0),
        ] {
            assert!(hits_box(inside, direction, lo, hi), "{direction:?}");
        }
    }

    /// 커밋된 캘리브로 만든 마스크가 **부피 안의 점은 살리고 밖의 점은 죽이나**.
    ///
    /// 광선 검사 자체가 맞아도 카메라 파라미터를 잘못 쓰면 마스크가 엉뚱한 곳에 선다.
    /// 아는 3D 점을 투영해 그 화소를 보는 게 그걸 가르는 유일한 방법이다.
    #[test]
    fn the_mask_keeps_the_flight_volume_and_drops_the_rest() {
        use crate::Point3;
        let Ok(calibration) =
            crate::camera::Calibration::load_json(&crate::defaults::calibration_path())
        else {
            return; // 캘리브가 없는 환경에서는 건너뛴다.
        };
        for params in &calibration.cameras {
            let volume = Volume::from_calib(params).expect("volume");
            let check = |point: Point3, want: u8, what: &str| {
                let Some(pixel) = params.project_world(point) else {
                    return; // 화각 밖이면 이 점으로는 판정 못 한다.
                };
                let got = *volume
                    .keep
                    .at_2d::<u8>(pixel.y as i32, pixel.x as i32)
                    .expect("at");
                assert_eq!(
                    got, want,
                    "cam{} {what} {point:?} -> px({:.0},{:.0})",
                    params.camera_id.0, pixel.x, pixel.y
                );
            };
            // 테이블 한가운데, 비행 높이 — 반드시 살아야 한다.
            check(
                Point3::new(
                    table::WIDTH_X * 0.5,
                    table::LENGTH_Y * 0.5,
                    table::SURFACE_Z + 0.3,
                ),
                255,
                "비행 부피 안",
            );
            // 천장 쪽 — 부피 위라 죽어야 한다.
            check(
                Point3::new(
                    table::WIDTH_X * 0.5,
                    table::LENGTH_Y * 0.5,
                    table::SURFACE_Z + FLIGHT_BAND + 1.5,
                ),
                0,
                "부피 위",
            );
        }
    }

    /// 한 축으로 안 움직이는 광선. 나눗셈이 무한대가 되는 자리라 따로 본다.
    #[test]
    fn an_axis_aligned_ray_is_handled() {
        let (lo, hi) = unit_box();
        // z 로만 간다. x 는 상자 안, y 는 상자 밖.
        assert!(hits_box(
            Vector3::new(0.5, 0.5, -1.0),
            Vector3::new(0.0, 0.0, 1.0),
            lo,
            hi
        ));
        assert!(!hits_box(
            Vector3::new(0.5, 5.0, -1.0),
            Vector3::new(0.0, 0.0, 1.0),
            lo,
            hi
        ));
    }
}
