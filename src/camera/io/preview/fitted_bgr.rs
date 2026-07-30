use opencv::Result as CvResult;
use opencv::core::Mat;
use opencv::imgproc;
use opencv::prelude::*;

/// downscale 전용 fit 결과.
#[derive(Debug)]
pub struct FittedBgr {
    pub image: Mat,
    /// display = source * scale, 항상 ≤ 1.
    pub scale: f64,
}

/// 모니터보다 클 때만 축소. 작으면 그대로(확대 없음).
pub fn fit_bgr_downscale(image: &Mat, max_w: i32, max_h: i32) -> CvResult<FittedBgr> {
    let w = image.cols();
    let h = image.rows();
    if w <= 0 || h <= 0 || max_w <= 0 || max_h <= 0 {
        return Ok(FittedBgr {
            image: image.try_clone()?,
            scale: 1.0,
        });
    }
    let scale = (max_w as f64 / w as f64)
        .min(max_h as f64 / h as f64)
        .min(1.0);
    if scale >= 1.0 - 1e-12 {
        return Ok(FittedBgr {
            image: image.try_clone()?,
            scale: 1.0,
        });
    }
    let nw = (w as f64 * scale).round().max(1.0) as i32;
    let nh = (h as f64 * scale).round().max(1.0) as i32;
    let mut out = Mat::default();
    imgproc::resize(
        image,
        &mut out,
        opencv::core::Size::new(nw, nh),
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;
    return Ok(FittedBgr {
        image: out,
        scale: nw as f64 / w as f64,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bgr(w: i32, h: i32) -> Mat {
        return Mat::zeros(h, w, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
    }

    #[test]
    fn fit_downscale_keeps_small_image() {
        let img = bgr(100, 50);
        let fitted = fit_bgr_downscale(&img, 200, 200).unwrap();
        assert_eq!(fitted.image.cols(), 100);
        assert_eq!(fitted.image.rows(), 50);
        assert!((fitted.scale - 1.0).abs() < 1e-9);
    }

    #[test]
    fn fit_downscale_shrinks_preserving_aspect() {
        let img = bgr(2000, 1000);
        let fitted = fit_bgr_downscale(&img, 1000, 800).unwrap();
        assert_eq!(fitted.image.cols(), 1000);
        assert_eq!(fitted.image.rows(), 500);
        assert!((fitted.scale - 0.5).abs() < 1e-6);
    }
}
