//! 물리 계수 식별: 반발 e, 마찰 mu, 항력 k.
//!
//! tools/measure_* 가 샘플을 모으면 여기 공식에 넣는다.

use nalgebra::Vector3;

use crate::constants::G;

/// 연속 바운스 최고 높이로 반발계수 평균. e ~= sqrt(h1/h0).
///
/// heights 는 테이블 면 기준 정점 높이. 공 중심이면 반지름 보정은 호출 쪽에서.
pub fn restitution_from_bounce_heights(heights: &[f64]) -> Option<f64> {
    if heights.len() < 2 {
        return None;
    }
    let mut ratios = Vec::new();
    for w in heights.windows(2) {
        let (h0, h1) = (w[0], w[1]);
        if h0 <= 1e-6 || h1 < 0.0 {
            continue;
        }
        ratios.push((h1 / h0).sqrt());
    }
    return mean_positive(&ratios);
}

/// 바운스 직전/직후 법선 속력 쌍으로 e = v_out / v_in.
pub fn restitution_from_normal_speeds(pairs: &[(f64, f64)]) -> Option<f64> {
    let mut samples = Vec::new();
    for &(vin, vout) in pairs {
        if vin <= 1e-6 || vout < 0.0 {
            continue;
        }
        samples.push(vout / vin);
    }
    return mean_positive(&samples);
}

/// 접선 속력 쌍으로 mu = 1 - v_out / v_in.
pub fn friction_from_tangential_speeds(pairs: &[(f64, f64)]) -> Option<f64> {
    let mut samples = Vec::new();
    for &(vin, vout) in pairs {
        if vin <= 1e-6 || vout < 0.0 || vout > vin + 1e-6 {
            continue;
        }
        samples.push(1.0 - vout / vin);
    }
    return mean_clamped(&samples, 0.0, 1.0);
}

/// 궤적 샘플 (t, p) 로 이차 항력 k 를 최소제곱 적합.
///
/// a ~= g - k * |v| * v 이므로
/// k = -sum((a-g)*(|v|v)) / sum(||v|v||^2).
///
/// samples 는 시간 오름차순, 길이 3 이상, dt > 0.
pub fn drag_from_trajectory(samples: &[(f64, Vector3<f64>)]) -> Option<f64> {
    if samples.len() < 3 {
        return None;
    }
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 1..samples.len().saturating_sub(1) {
        let (t0, p0) = samples[i - 1];
        let (t1, p1) = samples[i];
        let (t2, p2) = samples[i + 1];
        let dt0 = t1 - t0;
        let dt1 = t2 - t1;
        if dt0 < 1e-4 || dt1 < 1e-4 {
            continue;
        }
        let v0 = (p1 - p0) / dt0;
        let v1 = (p2 - p1) / dt1;
        let dt_mid = 0.5 * (dt0 + dt1);
        if dt_mid < 1e-4 {
            continue;
        }
        let a = (v1 - v0) / dt_mid;
        let v = 0.5 * (v0 + v1);
        let speed = v.norm();
        if speed < 0.3 {
            // 거의 멈춰 있으면 항력 신호가 약하다
            continue;
        }
        let drag_dir = speed * v; // |v| * v
        let residual = a - G;
        num += residual.dot(&drag_dir);
        den += drag_dir.dot(&drag_dir);
    }
    if den < 1e-9 {
        return None;
    }
    // residual ~= -k * |v| * v => k = -num / den
    let k = -num / den;
    if !k.is_finite() || k < 0.0 {
        return None;
    }
    return Some(k);
}

fn mean_positive(xs: &[f64]) -> Option<f64> {
    let ok: Vec<f64> = xs
        .iter()
        .copied()
        .filter(|x| x.is_finite() && *x > 0.0 && *x < 1.5)
        .collect();
    if ok.is_empty() {
        return None;
    }
    return Some(ok.iter().sum::<f64>() / ok.len() as f64);
}

fn mean_clamped(xs: &[f64], lo: f64, hi: f64) -> Option<f64> {
    let ok: Vec<f64> = xs
        .iter()
        .copied()
        .filter(|x| x.is_finite() && *x >= lo && *x <= hi)
        .collect();
    if ok.is_empty() {
        return None;
    }
    return Some(ok.iter().sum::<f64>() / ok.len() as f64);
}

/// 측정값을 TOML 스니펫으로 포맷. config / constants 반영용.
pub fn physics_coeffs_toml(
    restitution: Option<f64>,
    friction: Option<f64>,
    drag: Option<f64>,
) -> String {
    let mut lines = vec![
        "# pingpong physics coeffs (tools/measure_*)".to_string(),
        "# domain::constants::ball / physics 또는 [physics] 섹션에 반영".to_string(),
    ];
    if let Some(e) = restitution {
        lines.push(format!("restitution = {e:.6}"));
    }
    if let Some(mu) = friction {
        lines.push(format!("friction = {mu:.6}"));
    }
    if let Some(k) = drag {
        lines.push(format!("drag = {k:.8}"));
    }
    lines.push(String::new());
    return lines.join("\n");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn restitution_from_equal_height_ratio() {
        // e=0.9 이면 h1/h0 = 0.81
        let h0 = 1.0;
        let h1 = 0.81;
        let e = restitution_from_bounce_heights(&[h0, h1]).unwrap();
        assert!((e - 0.9).abs() < 1e-9);
    }

    #[test]
    fn restitution_from_speed_pairs() {
        let e = restitution_from_normal_speeds(&[(2.0, 1.7), (1.8, 1.53)]).unwrap();
        assert!((e - 0.85).abs() < 1e-6);
    }

    #[test]
    fn friction_from_tangential() {
        // vt_out = 0.7 * vt_in 이면 mu = 0.3
        let mu = friction_from_tangential_speeds(&[(1.0, 0.7), (2.0, 1.4)]).unwrap();
        assert!((mu - 0.3).abs() < 1e-9);
    }

    #[test]
    fn drag_recovers_known_k_on_synthetic_arc() {
        let k_true = 0.02;
        let mut samples = Vec::new();
        let mut p = Vector3::new(0.0, 0.0, 1.0);
        let mut v = Vector3::new(0.0, -5.0, 2.0);
        let dt = 0.008;
        for i in 0..80 {
            let t = f64::from(i) * dt;
            samples.push((t, p));
            let a = G - k_true * v.norm() * v;
            v += a * dt;
            p += v * dt;
        }
        let k_hat = drag_from_trajectory(&samples).expect("fit");
        assert!(
            (k_hat - k_true).abs() < 0.005,
            "k_hat={k_hat} true={k_true}"
        );
    }
}
