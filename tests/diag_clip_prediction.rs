//! 클립으로 **예측 정확도**와 **측정 노이즈**를 잰다.
//!
//! 지금까지의 진단은 입력 품질(검출·삼각측량·게이트)만 봤다. 정작 중요한 건 "예상 도달점이
//! 실제 도달점과 얼마나 다른가"인데, 그건 실기에서는 진실을 몰라 못 쟀다. 클립은 다르다 —
//! 공이 실제로 어디를 지났는지는 **나중 프레임들의 삼각측량**이 알려준다.
//!
//! 두 가지를 낸다:
//!
//! 1. **측정 노이즈 σ** — 비행 구간에 등가속(탄도) 곡선을 축별로 최소자승 피팅하고 잔차 RMS.
//!    `EstimatorParams::r_meas`(현재 0.0009 = σ 3 cm)를 추측이 아니라 이 값으로 정할 수 있다.
//! 2. **예측 오차 vs 리드 타임** — EKF에 측정을 순서대로 먹이면서 매 스텝 접수 평면 도달점을
//!    예측하고, 실측 궤적이 그 평면을 지난 실제 지점과 비교한다. 오차가 리드 타임에 따라
//!    **체계적으로 커지면** 모델 문제(항력·스핀), 무작위면 측정 노이즈다.
//!
//! 삼각측량 쌍은 **프레임 인덱스**로 맞춘다 — 두 AVI는 동시 녹화라 같은 인덱스가 같은 시각이다.
//! (런타임은 타임스탬프 보간을 쓰지만, 여기서는 진실을 만들어야 하므로 인덱스가 더 정확하다.)
//!
//! ```bash
//! cargo test --release --test diag_clip_prediction -- --ignored --nocapture
//! CLIP=fly_02 cargo test --release --test diag_clip_prediction -- --ignored --nocapture
//! ```

use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::{Matrix3, Vector3};
use pingpong_bot::Point3;
use pingpong_bot::camera::{self, Calibration, FrameSource, OpenCvCapture};
use pingpong_bot::defaults;
use pingpong_bot::detector::Detector;
use pingpong_bot::estimator::{Ekf, Estimator, HitPlane, Triangulate};
use pingpong_bot::robot::motion::InterceptWindow;

/// 재투영 오차가 이보다 크면 두 캠이 서로 다른 걸 잡은 것 — 런타임과 같은 상한.
const MAX_REPROJECTION_PX: f64 = 14.0;

/// 한 프레임의 측정.
struct Sample {
    /// 클립 시작 기준 시각 [s].
    t: f64,
    point: Point3,
}

/// 두 캠이 **같은 프레임에서** 잡은 것만 삼각측량한 궤적.
fn stereo_track(dir: &Path, fps: f64) -> Vec<Sample> {
    let calibration =
        Calibration::load_json(&defaults::calibration_path()).expect("calibration 로드");
    let left = detect_all(&dir.join("left.avi"), camera::Id(0));
    let right = detect_all(&dir.join("right.avi"), camera::Id(1));

    let mut track = Vec::new();
    for (index, (l, r)) in left.iter().zip(right.iter()).enumerate() {
        let (Some(l), Some(r)) = (l, r) else { continue };
        let hits = [(camera::Id(0), *l), (camera::Id(1), *r)];
        let Some(point) = Triangulate::pixels(&hits, &calibration) else {
            continue;
        };
        // 두 캠이 서로 다른 걸 잡은 쌍은 버린다 — 진실을 만드는 자리라 더 엄격해야 한다.
        let worst = hits
            .iter()
            .filter_map(|(id, pixel)| {
                let params = calibration.params(*id)?;
                let projected = params.project_world_unclipped(point)?;
                Some((projected.x - pixel.x).hypot(projected.y - pixel.y))
            })
            .fold(0.0_f64, f64::max);
        if worst > MAX_REPROJECTION_PX {
            continue;
        }
        track.push(Sample {
            t: index as f64 / fps,
            point,
        });
    }
    return track;
}

/// 프레임별 검출 픽셀 (`None` = 못 찾음).
fn detect_all(path: &Path, camera_id: camera::Id) -> Vec<Option<camera::Pixel>> {
    let mut source = OpenCvCapture::from_path(camera_id, path).expect("클립 열기");
    let params = defaults::camera_params_for(camera_id).expect("camera_params_for");
    let mut detector = defaults::detector_for(camera_id).expect("detector_for");
    let needs_undistort = !params.dist.is_empty();

    let mut out = Vec::new();
    while let Some(frame) = source.next_frame() {
        let frame = if needs_undistort {
            Detector::undistort(&frame, &params).expect("undistort")
        } else {
            frame
        };
        out.push(detector.detect(&frame));
    }
    return out;
}

/// 2계 차분으로 **측정 노이즈 σ** [m]를 잰다.
///
/// 등간격 표본 셋 `p[i-1], p[i], p[i+1]`의 2계 차분은 참 가속 `a·dt²`에 노이즈가 얹힌 값이다.
/// x·y축은 참 가속이 거의 0(항력만)이라 2계 차분이 사실상 순수 노이즈고, 그 분산은 `6σ²`다.
///
/// 곡선 피팅과 달리 **바운스에 둔감하다** — 바운스가 낀 삼중항 하나만 커질 뿐 전체를
/// 오염시키지 않아 중앙값으로 걸러진다. (앞서 포물선 피팅은 바운스를 걸쳐 σ가 67 cm로
/// 나왔다 — 못 믿을 값이었다.)
fn measurement_sigma(track: &[Sample]) -> Option<f64> {
    let mut seconds: Vec<f64> = Vec::new();
    for triple in track.windows(3) {
        let (a, b, c) = (&triple[0], &triple[1], &triple[2]);
        // 등간격이 아니면 2계 차분 해석이 깨진다 — 프레임 간격이 고른 구간만.
        let (dt0, dt1) = (b.t - a.t, c.t - b.t);
        if (dt0 - dt1).abs() > dt0 * 0.1 {
            continue;
        }
        for axis in [0_usize, 1] {
            let second = a.point.coords[axis] - 2.0 * b.point.coords[axis] + c.point.coords[axis];
            seconds.push(second.abs());
        }
    }
    if seconds.len() < 4 {
        return None;
    }
    seconds.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    // 중앙값 |d| → σ. 정규분포에서 E|d| = σ_d·√(2/π), σ_d = √6·σ.
    let median = seconds[seconds.len() / 2];
    return Some(median / (6.0_f64.sqrt() * (2.0 / std::f64::consts::PI).sqrt()));
}

/// 등가속 최소자승 피팅 `p(t) = a t² + b t + c`의 축별 잔차 RMS [m].
///
/// 바운스가 섞이면 한 포물선으로 안 맞으므로, 호출측이 바운스 없는 구간만 넘겨야 한다.
fn ballistic_residual_rms(track: &[Sample]) -> Option<[f64; 3]> {
    if track.len() < 4 {
        return None;
    }
    let t0 = track[0].t;
    // 정규방정식 (3×3) — [t², t, 1] 기저.
    let mut ata = Matrix3::zeros();
    let mut atb = [Vector3::zeros(); 3];
    for sample in track {
        let t = sample.t - t0;
        let basis = Vector3::new(t * t, t, 1.0);
        ata += basis * basis.transpose();
        for axis in 0..3 {
            atb[axis] += basis * sample.point.coords[axis];
        }
    }
    let inverse = ata.try_inverse()?;
    let coefficients: Vec<Vector3<f64>> = atb.iter().map(|b| inverse * b).collect();

    let mut sums = [0.0_f64; 3];
    for sample in track {
        let t = sample.t - t0;
        let basis = Vector3::new(t * t, t, 1.0);
        for axis in 0..3 {
            let fitted = coefficients[axis].dot(&basis);
            let residual = sample.point.coords[axis] - fitted;
            sums[axis] += residual * residual;
        }
    }
    let n = track.len() as f64;
    return Some([
        (sums[0] / n).sqrt(),
        (sums[1] / n).sqrt(),
        (sums[2] / n).sqrt(),
    ]);
}

/// 실측 궤적이 평면 `y`를 **로봇 쪽으로** 지나는 지점과 시각.
fn actual_crossing(track: &[Sample], plane_y: f64) -> Option<(f64, Point3)> {
    for pair in track.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        // y가 줄어드는 방향(로봇 쪽)으로 평면을 통과하는 구간.
        if a.point.coords.y >= plane_y && b.point.coords.y < plane_y {
            let span = a.point.coords.y - b.point.coords.y;
            let weight = if span.abs() < 1e-9 {
                0.0
            } else {
                (a.point.coords.y - plane_y) / span
            };
            let point = Point3::from(a.point.coords + (b.point.coords - a.point.coords) * weight);
            return Some((a.t + (b.t - a.t) * weight, point));
        }
    }
    return None;
}

#[test]
#[ignore = "순수 진단(클립 필요). 실행: cargo test --release --test diag_clip_prediction -- --ignored --nocapture"]
fn diag_clip_prediction_error() {
    let name = std::env::var("CLIP").unwrap_or_else(|_| "fly_01".to_owned());
    let dir = Path::new(defaults::DEFAULT_CLIPS_DIR).join(&name);
    assert!(dir.is_dir(), "클립 없음: {}", dir.display());
    let fps = clip_fps(&dir);

    let track = stereo_track(&dir, fps);
    println!("\n=== {name} (meas_fps {fps:.2}) ===");
    assert!(!track.is_empty(), "동시 검출된 프레임이 없다");
    println!(
        "실측 궤적 {}점, {:.2}~{:.2}s",
        track.len(),
        track[0].t,
        track[track.len() - 1].t
    );

    // ── 1. 측정 노이즈와 그로부터 오는 속도 시드 불확실성 ──────────────────
    let params = defaults::EstimatorParams::default();
    match measurement_sigma(&track) {
        Some(sigma) => {
            println!(
                "측정 노이즈 σ = {:.1} cm  (현재 r_meas {:.5} → σ {:.1} cm)",
                sigma * 100.0,
                params.r_meas,
                params.r_meas.sqrt() * 100.0
            );
            println!("  → r_meas 제안 {:.5}", sigma * sigma);
            // 속도는 측정되지 않는다 — 첫 두 측정의 유한차분으로 시드된다.
            // 그 불확실성은 σ_v = σ_p·√2/dt 로 **위치 노이즈보다 훨씬 크다**.
            let dt = 1.0 / fps;
            let sigma_v = sigma * 2.0_f64.sqrt() / dt;
            println!(
                "  → 유한차분 속도 시드 σ_v = {:.2} m/s (dt {:.1} ms) ⇒ 분산 {:.2}",
                sigma_v,
                dt * 1e3,
                sigma_v * sigma_v
            );
        }
        None => println!("측정 노이즈: 등간격 삼중항이 모자라 측정 불가"),
    }

    // ── 2. 예측 오차 vs 리드 타임 ─────────────────────────────────────────
    // 실측 궤적이 실제로 지나는 접수 평면을 고른다.
    let planes = InterceptWindow::default().hit_planes();
    let Some((plane, (cross_t, cross_point))) = planes
        .iter()
        .find_map(|plane| actual_crossing(&track, plane.y).map(|c| (*plane, c)))
    else {
        println!(
            "실측 궤적이 접수 창(y {:.2}~{:.2})을 지나지 않는다 — 예측 오차 측정 불가",
            InterceptWindow::default().y_min,
            InterceptWindow::default().y_max
        );
        return;
    };
    println!(
        "실제 도달: 평면 y={:.2}  t={:.3}s  x={:.3} z={:.3}",
        plane.y, cross_t, cross_point.coords.x, cross_point.coords.z
    );

    // 예측이 아무리 정확해도 팔이 못 닿으면 못 친다 — 실제 도달점이 작업영역 안인지.
    // 입사 속도는 평면 통과 직전 두 점의 차분.
    let incoming = track
        .windows(2)
        .find(|pair| pair[1].t >= cross_t)
        .map(|pair| (pair[1].point.coords - pair[0].point.coords) / (pair[1].t - pair[0].t))
        .unwrap_or_else(Vector3::zeros);
    let robot = defaults::robot().expect("robot");
    let start = pingpong_bot::robot::Pose::new(
        robot.arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
        robot.arm.default_joints.clone(),
    );
    let at_impact = pingpong_bot::estimator::Prediction {
        time_to_impact_secs: 0.30,
        impact_position: cross_point,
        incoming_velocity: incoming,
    };
    match pingpong_bot::robot::motion::Planner::feasibility(&robot.arm, &at_impact, &start) {
        Some(feasibility) => println!(
            "  IK 도달 O — 관절속도 비율 {:.2} · 레일 {:.2} (1.0 초과면 한계 밖)",
            feasibility.peak_joint_speed_ratio, feasibility.peak_rail_speed_ratio
        ),
        None => println!("  **IK 도달 X** — 이 지점은 팔 작업영역 밖이다"),
    }

    // 런타임과 같은 필터에 같은 순서로 먹인다.
    let base = Instant::now();
    let mut ekf = Ekf::default();
    // σ_impact ≈ hypot(σ_p, σ_v·리드) — 필터가 스스로 말하는 도달점 불확실성.
    // 이게 실제 오차와 같이 움직이면 "믿어도 되는가"의 게이트로 쓸 수 있다.
    println!("  리드[s]   예측 x    오차[cm]   σ예상[cm]");
    let mut errors = Vec::new();
    for sample in &track {
        if sample.t >= cross_t {
            break;
        }
        ekf.update_position(sample.point, base + Duration::from_secs_f64(sample.t));
        let Some(prediction) = ekf.predict_to(HitPlane { y: plane.y }) else {
            continue;
        };
        let lead = cross_t - sample.t;
        let dx = prediction.impact_position.coords.x - cross_point.coords.x;
        let dz = prediction.impact_position.coords.z - cross_point.coords.z;
        let error = dx.hypot(dz);
        let sigma = match (ekf.position_sigma(), ekf.velocity_sigma()) {
            (Some(sp), Some(sv)) => sp.hypot(sv * lead),
            _ => f64::NAN,
        };
        errors.push((lead, error, sigma));
        println!(
            "  {lead:>6.3}  {:>8.3}  {:>8.1}  {:>9.1}",
            prediction.impact_position.coords.x,
            error * 100.0,
            sigma * 100.0
        );
    }

    if errors.is_empty() {
        println!("예측이 한 번도 나오지 않았다 (EKF가 속도까지 못 감)");
        return;
    }
    // 커밋 창(0.2~0.6 s) 안에서의 오차가 실제로 스윙 정확도를 좌우한다.
    let in_window: Vec<f64> = errors
        .iter()
        .filter(|(lead, _, _)| (0.20..=0.60).contains(lead))
        .map(|(_, error, _)| *error)
        .collect();
    if in_window.is_empty() {
        println!("커밋 창(0.20~0.60 s) 안에 예측이 없다");
    } else {
        let mean = in_window.iter().sum::<f64>() / in_window.len() as f64;
        let worst = in_window.iter().copied().fold(0.0_f64, f64::max);
        println!(
            "커밋 창 예측 오차: 평균 {:.1} cm · 최대 {:.1} cm ({}회)",
            mean * 100.0,
            worst * 100.0,
            in_window.len()
        );
    }

    // σ 게이트를 걸면 커밋 창 오차가 어떻게 되는지 — 임계를 데이터로 정하려는 것.
    for threshold in [0.30_f64, 0.20, 0.15, 0.10] {
        let kept: Vec<&(f64, f64, f64)> = errors
            .iter()
            .filter(|(lead, _, sigma)| (0.20..=0.60).contains(lead) && *sigma <= threshold)
            .collect();
        if kept.is_empty() {
            println!("  σ ≤ {:.2} m → 커밋 창에 남는 예측 없음", threshold);
            continue;
        }
        let mean = kept.iter().map(|(_, e, _)| e).sum::<f64>() / kept.len() as f64;
        let worst = kept.iter().map(|(_, e, _)| *e).fold(0.0_f64, f64::max);
        let earliest = kept
            .iter()
            .map(|(lead, _, _)| *lead)
            .fold(0.0_f64, f64::max);
        println!(
            "  σ ≤ {:.2} m → {}회 남음, 평균 {:.1} cm · 최대 {:.1} cm, 가장 이른 리드 {:.3}s",
            threshold,
            kept.len(),
            mean * 100.0,
            worst * 100.0,
            earliest
        );
    }
}

fn clip_fps(dir: &Path) -> f64 {
    let text = std::fs::read_to_string(dir.join("meta.json")).expect("meta.json");
    let value: serde_json::Value = serde_json::from_str(&text).expect("meta.json 파싱");
    return value["meas_fps"].as_f64().expect("meas_fps");
}

/// `z`가 단조 감소하는 앞 구간만 — 첫 바운스 전.
trait DescendingRun {
    fn take_while_inclusive_descending(self) -> Vec<Sample>;
}

impl<'a, I: Iterator<Item = &'a Sample>> DescendingRun for I {
    fn take_while_inclusive_descending(self) -> Vec<Sample> {
        let mut out: Vec<Sample> = Vec::new();
        for sample in self {
            if let Some(last) = out.last()
                && sample.point.coords.z > last.point.coords.z + 0.02
            {
                break;
            }
            out.push(Sample {
                t: sample.t,
                point: sample.point,
            });
        }
        return out;
    }
}
