//! 탁구대 랜드마크 → OpenCV `solvePnP`(IPPE) → [`CameraParams`] 외참.

use nalgebra::{Matrix3, Vector3};
use opencv::calib3d::{self, SolvePnPMethod};
use opencv::core::{CV_64F, Mat, MatTraitConst, Point2d, Point3d, Vector};
use opencv::prelude::*;

use super::table_landmarks::{
    MAX_REPROJ_RMSE_PX, TABLE_LANDMARK_COUNT, table_landmarks,
};
use super::{Calibration, CameraParams};
use crate::camera::{CameraId, PixelPoint};
use crate::constants::table;
use crate::Point3;

/// PnP 결과 (+ 재투영 RMSE).
#[derive(Debug, Clone)]
pub struct TablePnpResult {
    pub params: CameraParams,
    /// 선택 해의 재투영 RMSE [px]
    pub reproj_rmse: f64,
    /// IPPE가 낸 후보 수
    pub candidates: usize,
}

/// FOV로 인트린식을 근사한 뒤 랜드마크 픽셀로 외참을 푼다.
///
/// `pixels.len()`은 [`TABLE_LANDMARK_COUNT`]와 같아야 한다 (순서 = `table_landmarks()`).
pub fn calibrate_table_pnp(
    camera_id: CameraId,
    label: Option<String>,
    width: u32,
    height: u32,
    fov_y_deg: f64,
    pixels: &[PixelPoint],
) -> Result<TablePnpResult, String> {
    if pixels.len() != TABLE_LANDMARK_COUNT {
        return Err(format!(
            "랜드마크 픽셀 {}개 필요 (got {})",
            TABLE_LANDMARK_COUNT,
            pixels.len()
        ));
    }
    if width < 2 || height < 2 {
        return Err("이미지 크기가 너무 작음".into());
    }
    if !(fov_y_deg > 1.0 && fov_y_deg < 179.0) {
        return Err(format!("fov_y_deg 비정상: {fov_y_deg}"));
    }

    let (fx, fy, cx, cy) = intrins_from_fov(width, height, fov_y_deg);
    let marks = table_landmarks();
    let object_pts = object_points_mat(&marks.map(|m| m.world))?;
    let image_pts = image_points_mat(pixels)?;
    let camera_matrix = camera_matrix_mat(fx, fy, cx, cy)?;
    let dist = Mat::zeros(5, 1, CV_64F)
        .map_err(|e| format!("dist: {e}"))?
        .to_mat()
        .map_err(|e| format!("dist mat: {e}"))?;

    let mut rvecs = Vector::<Mat>::new();
    let mut tvecs = Vector::<Mat>::new();
    let mut reproj_errs = Mat::default();
    let empty = Mat::default();
    let n = calib3d::solve_pnp_generic(
        &object_pts,
        &image_pts,
        &camera_matrix,
        &dist,
        &mut rvecs,
        &mut tvecs,
        false,
        SolvePnPMethod::SOLVEPNP_IPPE,
        &empty,
        &empty,
        &mut reproj_errs,
    )
    .map_err(|e| format!("solvePnPGeneric(IPPE): {e}"))?;
    if n < 1 || rvecs.len() < 1 || tvecs.len() < 1 {
        return Err("solvePnP: 해가 없음".into());
    }

    let mut best: Option<(usize, f64, Matrix3<f64>, Vector3<f64>)> = None;
    for i in 0..rvecs.len() {
        let rvec = rvecs.get(i).map_err(|e| format!("rvec[{i}]: {e}"))?;
        let tvec = tvecs.get(i).map_err(|e| format!("tvec[{i}]: {e}"))?;
        let mut rvec_mut = rvec.try_clone().map_err(|e| format!("rvec clone: {e}"))?;
        let mut tvec_mut = tvec.try_clone().map_err(|e| format!("tvec clone: {e}"))?;
        let _ = calib3d::solve_pnp_refine_lm_def(
            &object_pts,
            &image_pts,
            &camera_matrix,
            &dist,
            &mut rvec_mut,
            &mut tvec_mut,
        );
        let rotation = rodrigues_to_matrix(&rvec_mut)?;
        let translation = tvec_to_vector3(&tvec_mut)?;
        let rmse = reprojection_rmse(
            &marks.map(|m| m.world),
            pixels,
            &rotation,
            &translation,
            fx,
            fy,
            cx,
            cy,
        );
        if !rmse.is_finite() {
            continue;
        }
        let prefer = match &best {
            None => true,
            Some((_, best_rmse, best_r, best_t)) => {
                let score = pose_score(rmse, &rotation, &translation);
                let best_score = pose_score(*best_rmse, best_r, best_t);
                score < best_score
            }
        };
        if prefer {
            best = Some((i, rmse, rotation, translation));
        }
    }

    let Some((_idx, reproj_rmse, rotation, translation)) = best else {
        return Err("solvePnP: 유한 RMSE 해 없음".into());
    };

    let params = CameraParams {
        camera_id,
        label: label.or_else(|| {
            Some(format!(
                "table-pnp rmse={reproj_rmse:.2}px fov_y={fov_y_deg:.1}"
            ))
        }),
        width,
        height,
        fx,
        fy,
        cx,
        cy,
        dist: Vec::new(),
        rotation,
        translation,
    };

    return Ok(TablePnpResult {
        params,
        reproj_rmse,
        candidates: rvecs.len(),
    });
}

/// RMSE가 `max_rmse` 이하면 Ok.
pub fn ensure_reproj_below(result: &TablePnpResult, max_rmse: f64) -> Result<(), String> {
    if result.reproj_rmse > max_rmse {
        return Err(format!(
            "재투영 RMSE {:.2} px > 한도 {max_rmse} px (클릭·FOV 확인)",
            result.reproj_rmse
        ));
    }
    return Ok(());
}

/// RMSE가 [`MAX_REPROJ_RMSE_PX`] 이하면 Ok.
pub fn ensure_reproj_ok(result: &TablePnpResult) -> Result<(), String> {
    return ensure_reproj_below(result, MAX_REPROJ_RMSE_PX);
}

/// 기존 번들에 카메라 1대를 넣거나 같은 `camera_id`를 교체한다.
pub fn upsert_camera(calib: &mut Calibration, params: CameraParams) {
    if let Some(slot) = calib
        .cameras
        .iter_mut()
        .find(|c| c.camera_id == params.camera_id)
    {
        *slot = params;
        return;
    }
    calib.cameras.push(params);
    calib.cameras.sort_by_key(|c| c.camera_id.0);
}

fn intrins_from_fov(width: u32, height: u32, fov_y_deg: f64) -> (f64, f64, f64, f64) {
    let fov_y = fov_y_deg.to_radians();
    let fy = (f64::from(height) * 0.5) / (fov_y * 0.5).tan();
    let fx = fy;
    let cx = f64::from(width) * 0.5;
    let cy = f64::from(height) * 0.5;
    return (fx, fy, cx, cy);
}

fn object_points_mat(world: &[Point3]) -> Result<Vector<Point3d>, String> {
    let mut v = Vector::<Point3d>::new();
    for p in world {
        v.push(Point3d::new(p.x, p.y, p.z));
    }
    return Ok(v);
}

fn image_points_mat(pixels: &[PixelPoint]) -> Result<Vector<Point2d>, String> {
    let mut v = Vector::<Point2d>::new();
    for p in pixels {
        v.push(Point2d::new(p.x, p.y));
    }
    return Ok(v);
}

fn camera_matrix_mat(fx: f64, fy: f64, cx: f64, cy: f64) -> Result<Mat, String> {
    let mut k = Mat::zeros(3, 3, CV_64F)
        .map_err(|e| format!("K zeros: {e}"))?
        .to_mat()
        .map_err(|e| format!("K mat: {e}"))?;
    *k.at_2d_mut::<f64>(0, 0).map_err(|e| format!("K00: {e}"))? = fx;
    *k.at_2d_mut::<f64>(1, 1).map_err(|e| format!("K11: {e}"))? = fy;
    *k.at_2d_mut::<f64>(0, 2).map_err(|e| format!("K02: {e}"))? = cx;
    *k.at_2d_mut::<f64>(1, 2).map_err(|e| format!("K12: {e}"))? = cy;
    *k.at_2d_mut::<f64>(2, 2).map_err(|e| format!("K22: {e}"))? = 1.0;
    return Ok(k);
}

fn rodrigues_to_matrix(rvec: &Mat) -> Result<Matrix3<f64>, String> {
    let mut rmat = Mat::default();
    calib3d::rodrigues_def(rvec, &mut rmat).map_err(|e| format!("Rodrigues: {e}"))?;
    let mut out = Matrix3::zeros();
    for r in 0..3 {
        for c in 0..3 {
            out[(r, c)] = *rmat
                .at_2d::<f64>(r as i32, c as i32)
                .map_err(|e| format!("R[{r},{c}]: {e}"))?;
        }
    }
    return Ok(out);
}

fn tvec_to_vector3(tvec: &Mat) -> Result<Vector3<f64>, String> {
    let read = |i: i32| -> Result<f64, String> {
        if tvec.rows() >= 3 && tvec.cols() >= 1 {
            return Ok(*tvec.at_2d::<f64>(i, 0).map_err(|e| format!("t[{i}]: {e}"))?);
        }
        return Ok(*tvec.at::<f64>(i).map_err(|e| format!("t[{i}]: {e}"))?);
    };
    return Ok(Vector3::new(read(0)?, read(1)?, read(2)?));
}

fn reprojection_rmse(
    world: &[Point3],
    pixels: &[PixelPoint],
    rotation: &Matrix3<f64>,
    translation: &Vector3<f64>,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
) -> f64 {
    let mut sum = 0.0;
    let n = world.len().min(pixels.len());
    if n == 0 {
        return f64::INFINITY;
    }
    for i in 0..n {
        let x_cam = rotation * world[i].coords + translation;
        if x_cam.z <= 1e-6 {
            return f64::INFINITY;
        }
        let u = fx * (x_cam.x / x_cam.z) + cx;
        let v = fy * (x_cam.y / x_cam.z) + cy;
        let du = u - pixels[i].x;
        let dv = v - pixels[i].y;
        sum += du * du + dv * dv;
    }
    return (sum / n as f64).sqrt();
}

/// 낮을수록 좋음. RMSE 우선, 그다음 카메라가 테이블 위에 있고 아래로 보는 해.
fn pose_score(rmse: f64, rotation: &Matrix3<f64>, translation: &Vector3<f64>) -> f64 {
    let eye = -rotation.transpose() * translation;
    let mut penalty = 0.0;
    if eye.z < table::SURFACE_Z {
        penalty += 1e3;
    }
    // 카메라 +Z(전방)가 월드 -Z(아래)와 어느 정도 정렬되면 가산점
    let cam_forward = Vector3::new(rotation[(2, 0)], rotation[(2, 1)], rotation[(2, 2)]);
    if cam_forward.z > -0.05 {
        penalty += 200.0;
    }
    return rmse + penalty;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{CameraId, triangulate_projections};
    use nalgebra::Vector3;

    fn overhead_cam() -> CameraParams {
        let target = Vector3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, table::SURFACE_Z);
        let eye = target + Vector3::new(0.0, -0.4, 2.4);
        return CameraParams::look_at(
            CameraId::new(0),
            None,
            eye,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            70.0_f64.to_radians(),
        );
    }

    #[test]
    fn pnp_recovers_sim_camera_noise_free() {
        let truth = overhead_cam();
        let marks = table_landmarks();
        let mut pixels = Vec::new();
        for m in &marks {
            let px = truth
                .project_world(m.world)
                .unwrap_or_else(|| panic!("landmark {} out of FOV", m.id));
            pixels.push(px);
        }
        let fov_y = 2.0 * ((f64::from(truth.height) * 0.5) / truth.fy).atan().to_degrees();
        let result = calibrate_table_pnp(
            CameraId::new(0),
            None,
            truth.width,
            truth.height,
            fov_y,
            &pixels,
        )
        .expect("pnp");
        ensure_reproj_ok(&result).expect("rmse");
        assert!(
            result.reproj_rmse < 0.5,
            "rmse {}",
            result.reproj_rmse
        );

        let eye_t = -truth.rotation.transpose() * truth.translation;
        let eye_e = -result.params.rotation.transpose() * result.params.translation;
        let err = (eye_t - eye_e).norm();
        assert!(err < 0.05, "eye error {err} m (truth={eye_t:?} got={eye_e:?})");
    }

    #[test]
    fn pnp_calibration_triangulates_table_center() {
        let truth = overhead_cam();
        let marks = table_landmarks();
        let pixels: Vec<_> = marks
            .iter()
            .map(|m| truth.project_world(m.world).expect("in FOV"))
            .collect();
        let fov_y = 2.0 * ((f64::from(truth.height) * 0.5) / truth.fy).atan().to_degrees();
        let result = calibrate_table_pnp(
            CameraId::new(0),
            None,
            truth.width,
            truth.height,
            fov_y,
            &pixels,
        )
        .expect("pnp");
        ensure_reproj_ok(&result).expect("rmse");

        // 두 번째 카메라: 반대쪽에서 같은 점 투영 → PnP → 삼각측량
        let target = Vector3::new(table::WIDTH_X * 0.5, table::LENGTH_Y * 0.5, table::SURFACE_Z);
        let eye2 = target + Vector3::new(0.0, 0.4, 2.4);
        let truth2 = CameraParams::look_at(
            CameraId::new(1),
            None,
            eye2,
            target,
            Vector3::new(0.0, 0.0, 1.0),
            640,
            480,
            70.0_f64.to_radians(),
        );
        let pixels2: Vec<_> = marks
            .iter()
            .map(|m| truth2.project_world(m.world).expect("in FOV"))
            .collect();
        let result2 = calibrate_table_pnp(
            CameraId::new(1),
            None,
            truth2.width,
            truth2.height,
            fov_y,
            &pixels2,
        )
        .expect("pnp2");

        let mut calib = Calibration {
            cameras: Vec::new(),
        };
        upsert_camera(&mut calib, result.params.clone());
        upsert_camera(&mut calib, result2.params.clone());

        let json = serde_json::to_string(&calib).expect("ser");
        let back: Calibration = serde_json::from_str(&json).expect("de");
        assert_eq!(back.camera_count(), 2);

        let center = marks[4].world;
        let recovered = triangulate_projections(
            &back,
            &[CameraId::new(0), CameraId::new(1)],
            center,
        )
        .expect("tri");
        let err = (recovered.coords - center.coords).norm();
        assert!(err < 0.02, "triangulation error {err} m");
    }

    #[test]
    fn upsert_replaces_same_id() {
        let mut calib = Calibration::sim(2);
        let mut cam = calib.cameras[0].clone();
        cam.label = Some("replaced".into());
        upsert_camera(&mut calib, cam);
        assert_eq!(calib.camera_count(), 2);
        assert_eq!(calib.cameras[0].label.as_deref(), Some("replaced"));
    }
}
