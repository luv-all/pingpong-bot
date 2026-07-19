//! 멀티캠 3D 궤적에서 바운스(e)·롤(μ) 구간을 뽑는다.

use nalgebra::Vector3;

use crate::constants::{ball, table};
use crate::{CameraId, PixelPoint, Point3};

/// 삼각측량된 한 샘플.
#[derive(Debug, Clone)]
pub struct TrajPoint {
    pub t: f64,
    pub pos: Point3,
    pub pixels: Vec<(CameraId, PixelPoint)>,
}

/// 바운스 한 번 (반발계수 디버그용).
#[derive(Debug, Clone)]
pub struct BounceEvent {
    /// 접촉에 가까운 샘플 인덱스
    pub index: usize,
    pub contact: Point3,
    pub prev: Point3,
    pub next: Point3,
    pub v_in: Vector3<f64>,
    pub v_out: Vector3<f64>,
    /// e = |vz_out| / |vz_in|
    pub e: f64,
}

/// 테이블 위 롤 구간 (마찰 디버그용).
#[derive(Debug, Clone)]
pub struct RollEvent {
    pub i0: usize,
    pub i1: usize,
    pub p0: Point3,
    pub p1: Point3,
    pub vt_in: f64,
    pub vt_out: f64,
    /// μ = 1 - vt_out / vt_in
    pub mu: f64,
}

fn floor_z() -> f64 {
    return table::SURFACE_Z + ball::RADIUS;
}

fn velocity(a: &TrajPoint, b: &TrajPoint) -> Option<Vector3<f64>> {
    let dt = b.t - a.t;
    if dt < 1e-4 {
        return None;
    }
    return Some((b.pos.v - a.pos.v) / dt);
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
        let near_table = traj[i].pos.v.z < floor + 0.10;
        let bounce = v_in.z < -0.25 && v_out.z > 0.15 && near_table;
        if bounce {
            let vin_n = (-v_in.z).max(1e-6);
            let vout_n = v_out.z.max(0.0);
            let e = vout_n / vin_n;
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
        let on_table = (traj[i].pos.v.z - floor).abs() < 0.04 && v.z.abs() < 0.6;
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
    let xs: Vec<f64> = events.iter().map(|e| e.e).filter(|e| e.is_finite() && *e > 0.0).collect();
    if xs.is_empty() {
        return None;
    }
    return Some(xs.iter().sum::<f64>() / xs.len() as f64);
}

/// 롤 μ 평균.
pub fn mean_roll_mu(events: &[RollEvent]) -> Option<f64> {
    let xs: Vec<f64> = events.iter().map(|e| e.mu).filter(|m| m.is_finite()).collect();
    if xs.is_empty() {
        return None;
    }
    return Some(xs.iter().sum::<f64>() / xs.len() as f64);
}

#[cfg(test)]
mod tests {
    use super::*;

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
