//! 카메라 배치가 축별 정확도를 어떻게 바꾸나.
//!
//! 두 시선이 90°로 만나면 삼각측량이 가장 잘 되고, 나란하거나 마주 보면(0° 또는 180°)
//! 그 방향은 관측되지 않는다. 지금 리그는 **두 대가 다 테이블 +X 옆면**이라 x 가 양쪽
//! 모두에게 깊이 방향이고, 그게 예측이 옆으로 새는 원인이다.
//!
//! ```bash
//! cargo test --release --test diag_layout -- --ignored --nocapture
//! ```

use nalgebra::{Matrix2x3, Matrix3, Vector3};

use pingpong_bot::camera::{self, Calibration};
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::vision::fit::SIGMA_PX;
use pingpong_bot::{Point3, Vector3 as V3};

const WIDTH: u32 = 1280;
const HEIGHT: u32 = 800;
/// 지금 리그와 같은 화각 (table-PnP 라벨의 fov_y=47.3°).
const FOV_Y: f64 = 47.3_f64 * std::f64::consts::PI / 180.0;
/// 천장 높이 여유 안에서 잡은 설치 높이 [m].
const MOUNT_Z: f64 = 2.05;

fn eye(x: f64, y: f64) -> Vector3<f64> {
    return Vector3::new(x, y, MOUNT_Z);
}

fn center() -> Vector3<f64> {
    return Vector3::new(
        table::WIDTH_X * 0.5,
        table::LENGTH_Y * 0.5,
        table::SURFACE_Z,
    );
}

fn cam(id: u8, at: Vector3<f64>, target: Vector3<f64>) -> camera::Params {
    return camera::Params::look_at(
        camera::Id(id),
        None,
        at,
        target,
        Vector3::new(0.0, 0.0, 1.0),
        WIDTH,
        HEIGHT,
        FOV_Y,
    );
}

/// `∂pixel/∂world` (2×3) — EKF 가 쓰는 것과 같은 핀홀 미분.
fn jacobian(params: &camera::Params, point: Point3) -> Option<Matrix2x3<f64>> {
    let local = params.rotation * point.coords + params.translation;
    if local.z <= 0.05 {
        return None;
    }
    let inv_z = 1.0 / local.z;
    return Some(
        Matrix2x3::new(
            params.fx * inv_z,
            0.0,
            -params.fx * local.x * inv_z * inv_z,
            0.0,
            params.fy * inv_z,
            -params.fy * local.y * inv_z * inv_z,
        ) * params.rotation,
    );
}

/// 이 점을 이 배치로 재면 축별 위치 σ 가 얼마인가 [m].
///
/// 정보행렬 `Σ Jᵀ R⁻¹ J` 를 뒤집는다. 한 프레임의 두 시선만 쓴 값이라 필터가 시간축으로
/// 더 좁히는 몫은 안 들어 있다 — 배치끼리 견주는 용도다.
fn sigma_at(cameras: &[camera::Params], point: Point3) -> Option<V3> {
    let mut information = Matrix3::zeros();
    let mut seen = 0;
    for params in cameras {
        let Some(j) = jacobian(params, point) else {
            continue;
        };
        // 화각 밖이면 그 카메라는 이 점을 못 본다.
        if params.project_world(point).is_none() {
            continue;
        }
        information += j.transpose() * j / SIGMA_PX.powi(2);
        seen += 1;
    }
    if seen < 2 {
        return None;
    }
    let covariance = information.try_inverse()?;
    return Some(V3::new(
        covariance[(0, 0)].max(0.0).sqrt(),
        covariance[(1, 1)].max(0.0).sqrt(),
        covariance[(2, 2)].max(0.0).sqrt(),
    ));
}

/// 두 시선이 만나는 각 [deg]. 90°면 최선, 0°나 180°면 그 방향이 관측되지 않는다.
fn ray_angle(cameras: &[camera::Params], point: Point3) -> Option<f64> {
    let eyes: Vec<Vector3<f64>> = cameras
        .iter()
        .filter(|p| p.project_world(point).is_some())
        .map(|p| -p.rotation.transpose() * p.translation)
        .collect();
    if eyes.len() < 2 {
        return None;
    }
    // 여러 대면 가장 잘 만나는 쌍이 그 점의 조건을 결정한다.
    let mut best: f64 = 0.0;
    for i in 0..eyes.len() {
        for j in (i + 1)..eyes.len() {
            let a = (point.coords - eyes[i]).normalize();
            let b = (point.coords - eyes[j]).normalize();
            let deg = a.dot(&b).clamp(-1.0, 1.0).acos().to_degrees();
            // 180° 는 0° 와 마찬가지로 퇴화라 90°에서 얼마나 먼지로 본다.
            best = best.max(90.0 - (90.0 - deg).abs());
        }
    }
    return Some(best);
}

/// 접수 구간 위 격자에서 축별 σ 의 평균과 최악, 그리고 못 보는 칸 수.
fn score(cameras: &[camera::Params]) -> (V3, V3, usize, usize) {
    let mut sum = V3::zeros();
    let (mut worst, mut ok, mut blind) = (V3::zeros(), 0usize, 0usize);
    // 로봇이 실제로 칠 수 있는 구간: 접수 평면 앞뒤와 테이블 폭 전체, 타점 높이대.
    for i in 0..=8 {
        for j in 0..=8 {
            for k in 0..=3 {
                let point = Point3::new(
                    table::WIDTH_X * i as f64 / 8.0,
                    table::LENGTH_Y * j as f64 / 8.0,
                    table::SURFACE_Z + 0.05 + 0.25 * k as f64,
                );
                match sigma_at(cameras, point) {
                    Some(sigma) => {
                        sum += sigma;
                        worst = worst.sup(&sigma);
                        ok += 1;
                    }
                    None => blind += 1,
                }
            }
        }
    }
    if ok == 0 {
        return (V3::zeros(), V3::zeros(), 0, blind);
    }
    return (sum / ok as f64, worst, ok, blind);
}

#[test]
#[ignore = "설치 후보 비교: cargo test --release --test diag_layout -- --ignored --nocapture"]
fn compare_camera_layouts() {
    let (w, l) = (table::WIDTH_X, table::LENGTH_Y);
    let out = 1.4; // 테이블에서 옆으로 떨어진 거리 [m]
    let end = 1.2; // 테이블 끝에서 떨어진 거리 [m]

    let committed = Calibration::load_json(&defaults::calibration_path()).expect("calibration");
    let mut layouts: Vec<(&str, Vec<camera::Params>)> =
        vec![("지금 리그 (커밋된 캘리브)", committed.cameras.clone())];

    layouts.push((
        "둘 다 +X 옆 (지금 배치 재현)",
        vec![
            cam(0, eye(w + out, 0.1), center()),
            cam(1, eye(w + out, l - 0.1), center()),
        ],
    ));
    layouts.push((
        "대각 맞보기 (0,0)<->(W,L)",
        vec![
            cam(0, eye(-0.7, -0.7), center()),
            cam(1, eye(w + 0.7, l + 0.7), center()),
        ],
    ));
    layouts.push((
        "양 옆 마주보기 (+X / -X)",
        vec![
            cam(0, eye(w + out, l * 0.5), center()),
            cam(1, eye(-out, l * 0.5), center()),
        ],
    ));
    layouts.push((
        "옆 + 끝 (로봇 뒤)",
        vec![
            cam(0, eye(w + out, l * 0.5), center()),
            cam(1, eye(w * 0.5, -end), center()),
        ],
    ));
    layouts.push((
        "옆 + 끝 (슈터 뒤)",
        vec![
            cam(0, eye(w + out, l * 0.5), center()),
            cam(1, eye(w * 0.5, l + end), center()),
        ],
    ));
    layouts.push((
        "옆 + 끝 + 반대옆 (3대)",
        vec![
            cam(0, eye(w + out, l * 0.5), center()),
            cam(1, eye(w * 0.5, -end), center()),
            cam(2, eye(-out, l * 0.5), center()),
        ],
    ));

    println!(
        "격자 9×9×4 = 324칸, 타점 높이 {:.2}~{:.2} m, σ_px={SIGMA_PX}",
        table::SURFACE_Z + 0.05,
        table::SURFACE_Z + 0.80
    );
    println!(
        "{:<28} {:>21} {:>21} {:>8} {:>8}",
        "배치", "평균 σ [mm]", "최악 σ [mm]", "사각", "시선각"
    );
    for (name, cameras) in &layouts {
        let (mean, worst, ok, blind) = score(cameras);
        if ok == 0 {
            println!("{name:<28} {:>52}", "두 대가 같이 보는 칸이 없다");
            continue;
        }
        let angles: Vec<f64> = (0..=8)
            .flat_map(|i| (0..=8).map(move |j| (i, j)))
            .filter_map(|(i, j)| {
                ray_angle(
                    cameras,
                    Point3::new(
                        table::WIDTH_X * i as f64 / 8.0,
                        table::LENGTH_Y * j as f64 / 8.0,
                        table::SURFACE_Z + 0.3,
                    ),
                )
            })
            .collect();
        let mean_angle = angles.iter().sum::<f64>() / angles.len().max(1) as f64;
        println!(
            "{name:<28} {:>6.0}{:>7.0}{:>8.0} {:>6.0}{:>7.0}{:>8.0} {:>8} {:>7.0}°",
            mean.x * 1000.0,
            mean.y * 1000.0,
            mean.z * 1000.0,
            worst.x * 1000.0,
            worst.y * 1000.0,
            worst.z * 1000.0,
            format!("{blind}/{}", ok + blind),
            mean_angle
        );
    }
    println!("(σ 는 x y z 순. 한 프레임 두 시선만 쓴 값 — 배치끼리 견주는 용도다)");
}

/// 커밋된 캘리브가 **접수 창**을 보나.
///
/// 테이블 전체 평균이 좋아도 로봇이 치는 자리가 사각이면 소용없다. 두 대가 같이 보는
/// 칸에서만 삼각측량이 되고, 한 대만 보는 칸은 깊이가 안 잡힌다.
#[test]
#[ignore = "커밋된 캘리브 확인: cargo test --release --test diag_layout -- --ignored --nocapture"]
fn does_the_committed_rig_cover_the_intercept_window() {
    use pingpong_bot::robot::motion::InterceptWindow;

    let calibration = Calibration::load_json(&defaults::calibration_path()).expect("calibration");
    let planes = InterceptWindow::default().hit_planes();
    println!(
        "접수 창 y {:.2}~{:.2}, 타점 높이 {:.2}~{:.2} m",
        planes.first().map_or(0.0, |p| p.y),
        planes.last().map_or(0.0, |p| p.y),
        table::SURFACE_Z + 0.05,
        table::SURFACE_Z + 0.55
    );
    println!(
        "{:<8} {:>7} {:>7} {:>7} {:>8}",
        "y", "σx", "σy", "σz", "보이는 칸"
    );
    for plane in planes {
        let (mut sum, mut ok, mut total) = (V3::zeros(), 0usize, 0usize);
        for i in 0..=8 {
            for k in 0..=2 {
                total += 1;
                let point = Point3::new(
                    table::WIDTH_X * i as f64 / 8.0,
                    plane.y,
                    table::SURFACE_Z + 0.05 + 0.25 * k as f64,
                );
                if let Some(sigma) = sigma_at(&calibration.cameras, point) {
                    sum += sigma;
                    ok += 1;
                }
            }
        }
        if ok == 0 {
            println!(
                "{:<8.2} {:>25} {:>8}",
                plane.y,
                "두 대가 같이 보는 칸 없음",
                format!("0/{total}")
            );
            continue;
        }
        let mean = sum / ok as f64;
        println!(
            "{:<8.2} {:>6.0}mm {:>6.0}mm {:>6.0}mm {:>8}",
            plane.y,
            mean.x * 1000.0,
            mean.y * 1000.0,
            mean.z * 1000.0,
            format!("{ok}/{total}")
        );
    }

    // 사각이 어디인지 — 카메라를 어느 쪽으로 돌릴지가 여기서 갈린다.
    println!("\n접수 창 한가운데(y=0.20)에서 칸별로 몇 대가 보나:");
    println!("        {:>26}", "x [m]");
    print!("{:<8}", "z [m]");
    for i in 0..=8 {
        print!("{:>5.2}", table::WIDTH_X * i as f64 / 8.0);
    }
    println!();
    for k in (0..=2).rev() {
        let z = table::SURFACE_Z + 0.05 + 0.25 * k as f64;
        print!("{z:<8.2}");
        for i in 0..=8 {
            let point = Point3::new(table::WIDTH_X * i as f64 / 8.0, 0.20, z);
            let seen = calibration
                .cameras
                .iter()
                .filter(|params| params.project_world(point).is_some())
                .count();
            print!("{seen:>5}");
        }
        println!();
    }
    println!("2 = 삼각측량 가능, 1 이하 = 깊이 안 잡힘");
}
