//! 멀티캠 3D 궤적에서 바운스(e)·롤(μ) 구간을 뽑는다.

use nalgebra::Vector3;

use crate::constants::{ball, table};

use super::{BounceEvent, RollEvent, TrajPoint};

fn floor_z() -> f64 {
    return table::SURFACE_Z + ball::RADIUS;
}

fn velocity(a: &TrajPoint, b: &TrajPoint) -> Option<Vector3<f64>> {
    let dt = b.t - a.t;
    if dt < 1e-4 {
        return None;
    }
    return Some((b.pos.coords - a.pos.coords) / dt);
}

/// 바운스 접촉 프레임 앞뒤로 `v_in`/`v_out`를 다시 잴 창 폭 [표본 수].
///
/// ~79fps에서 접촉 프레임 인접 2점차는 삼각측량 잡음(몇 mm)이 그대로 속도 오차로
/// 증폭된다 — 그 프레임이 하필 공이 테이블 색에 가깝고 모션블러도 최대라 SNR이 가장
/// 낮다. 창을 넓혀 평균내면 이 증폭을 줄인다.
const BOUNCE_VELOCITY_WINDOW: usize = 4;

/// 시간-위치 표본 창의 최소자승 기울기(=속도).
pub fn windowed_velocity(points: &[TrajPoint]) -> Option<Vector3<f64>> {
    let n = points.len();
    if n < 2 {
        return None;
    }
    let t_bar = points.iter().map(|p| p.t).sum::<f64>() / n as f64;
    let p_bar = points.iter().fold(Vector3::zeros(), |acc, p| acc + p.pos.coords) / n as f64;
    let (mut num, mut den) = (Vector3::zeros(), 0.0);
    for p in points {
        let dt = p.t - t_bar;
        num += (p.pos.coords - p_bar) * dt;
        den += dt * dt;
    }
    return (den > 1e-9).then(|| num / den);
}

fn tangential_speed(v: Vector3<f64>) -> f64 {
    return (v.x * v.x + v.y * v.y).sqrt();
}

/// vz 부호 전환 + 테이블 근접으로 바운스를 찾는다.
pub fn detect_bounces(traj: &[TrajPoint]) -> Vec<BounceEvent> {
    let floor = floor_z();
    let mut out = Vec::new();
    if traj.len() < 3 {
        return out;
    }
    let mut i = 1usize;
    while i + 1 < traj.len() {
        let Some(v_in) = velocity(&traj[i - 1], &traj[i]) else {
            i += 1;
            continue;
        };
        let Some(v_out) = velocity(&traj[i], &traj[i + 1]) else {
            i += 1;
            continue;
        };
        let near_table = traj[i].pos.coords.z < floor + 0.10;
        let bounce = v_in.z < -0.25 && v_out.z > 0.15 && near_table;
        if bounce {
            // 위치 판정은 인접 2점차로 충분하다 — 잡음에 약한 건 여기서 쓰는 v_in/v_out의
            // *크기*뿐이니, 그것만 창 회귀로 다시 잰다. 창이 안 차면(궤적 끝 근처) 2점차로
            // 물러난다.
            let before = &traj[i.saturating_sub(BOUNCE_VELOCITY_WINDOW)..=i];
            let after = &traj[i..(i + 1 + BOUNCE_VELOCITY_WINDOW).min(traj.len())];
            let v_in = windowed_velocity(before).unwrap_or(v_in);
            let v_out = windowed_velocity(after).unwrap_or(v_out);
            let vin_n = -v_in.z;
            let vout_n = v_out.z.max(0.0);
            let e = vout_n / vin_n.max(1e-6);

            // 신뢰도 문턱. 창 회귀가 거의 정지한 구간까지 삼켜서 vin_n이 0 근처로 죽으면
            // e가 분모 0으로 폭발한다(실측 fly_46 등에서 e=93~3061080) — 여러 바운스가
            // 붙어 있는 클립에서 창이 옆 바운스로 새 들어간 신호로 본다. 실제 반발계수가
            // 1을 살짝 넘는 잡음은 있어도 몇 배씩 넘지는 않으므로 여유만 둔다.
            //
            // 접선 속도도 따로 본다 — 서브 클립엔 라켓 접촉(방향이 프레임 한두 개 만에
            // 확 바뀜)이 같은 궤적 토막에 섞여 있어서, 창이 그 경계를 걸치면 vin_n·e는
            // 멀쩡한데 접선 성분만 비정상으로 크게 나온다(실측 16~26 m/s — 이 서브들의
            // 정상 접선속도 1~4 m/s대와 확연히 갈린다).
            const MIN_VIN_Z: f64 = 1.0;
            const MAX_PLAUSIBLE_E: f64 = 1.05;
            const MAX_TANGENTIAL_SPEED: f64 = 8.0;
            let tangential_ok =
                tangential_speed(v_in) < MAX_TANGENTIAL_SPEED && tangential_speed(v_out) < MAX_TANGENTIAL_SPEED;
            if vin_n < MIN_VIN_Z || !(0.0..=MAX_PLAUSIBLE_E).contains(&e) || !tangential_ok {
                i += 1;
                continue;
            }

            out.push(BounceEvent {
                index: i,
                contact: traj[i].pos,
                prev: traj[i - 1].pos,
                next: traj[i + 1].pos,
                v_in,
                v_out,
                e,
            });
            // 같은 바운스 중복 방지
            i += 3;
            continue;
        }
        i += 1;
    }
    return out;
}

/// 테이블에 붙어 |vz|가 작은 연속 구간에서 접선 감속을 μ로 본다.
pub fn detect_rolls(traj: &[TrajPoint]) -> Vec<RollEvent> {
    let floor = floor_z();
    let mut out = Vec::new();
    if traj.len() < 4 {
        return out;
    }

    let mut run_start: Option<usize> = None;
    for i in 1..traj.len() {
        let Some(v) = velocity(&traj[i - 1], &traj[i]) else {
            if let Some(s) = run_start.take() {
                push_roll(traj, s, i.saturating_sub(1), &mut out);
            }
            continue;
        };
        let on_table = (traj[i].pos.coords.z - floor).abs() < 0.04 && v.z.abs() < 0.6;
        if on_table {
            if run_start.is_none() {
                run_start = Some(i.saturating_sub(1));
            }
        } else if let Some(s) = run_start.take() {
            push_roll(traj, s, i.saturating_sub(1), &mut out);
        }
    }
    if let Some(s) = run_start {
        push_roll(traj, s, traj.len() - 1, &mut out);
    }
    return out;
}

fn push_roll(traj: &[TrajPoint], i0: usize, i1: usize, out: &mut Vec<RollEvent>) {
    if i1 <= i0 + 2 {
        return;
    }
    let mid = i0 + (i1 - i0) / 3;
    let late = i0 + 2 * (i1 - i0) / 3;
    let Some(v0) = velocity(&traj[i0], &traj[mid.max(i0 + 1)]) else {
        return;
    };
    let Some(v1) = velocity(&traj[late.min(i1 - 1)], &traj[i1]) else {
        return;
    };
    let vt_in = tangential_speed(v0);
    let vt_out = tangential_speed(v1);
    if vt_in < 0.15 || vt_out > vt_in + 1e-3 {
        return;
    }
    let mu = (1.0 - vt_out / vt_in).clamp(0.0, 1.0);
    out.push(RollEvent {
        i0,
        i1,
        p0: traj[i0].pos,
        p1: traj[i1].pos,
        vt_in,
        vt_out,
        mu,
    });
}

/// 바운스 e 평균.
pub fn mean_bounce_e(events: &[BounceEvent]) -> Option<f64> {
    let xs: Vec<f64> = events
        .iter()
        .map(|e| e.e)
        .filter(|e| e.is_finite() && *e > 0.0)
        .collect();
    if xs.is_empty() {
        return None;
    }
    return Some(xs.iter().sum::<f64>() / xs.len() as f64);
}

/// 롤 μ 평균.
pub fn mean_roll_mu(events: &[RollEvent]) -> Option<f64> {
    let xs: Vec<f64> = events
        .iter()
        .map(|e| e.mu)
        .filter(|m| m.is_finite())
        .collect();
    if xs.is_empty() {
        return None;
    }
    return Some(xs.iter().sum::<f64>() / xs.len() as f64);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Point3;

    fn pt(t: f64, x: f64, y: f64, z: f64) -> TrajPoint {
        TrajPoint {
            t,
            pos: Point3::new(x, y, z),
            pixels: vec![],
        }
    }

    #[test]
    fn bounce_from_synthetic_vz_flip() {
        let floor = floor_z();
        let traj = vec![
            pt(0.00, 0.5, 1.0, floor + 0.20),
            pt(0.02, 0.5, 1.0, floor + 0.08),
            pt(0.04, 0.5, 1.0, floor + 0.01),
            pt(0.06, 0.5, 1.0, floor + 0.07),
            pt(0.08, 0.5, 1.0, floor + 0.15),
        ];
        let b = detect_bounces(&traj);
        assert!(!b.is_empty());
        assert!(b[0].e > 0.0 && b[0].e < 2.0);
    }
}
