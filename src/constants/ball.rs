//! 탁구공 규격·시뮬 collider 고정값.

/// 공 반지름 [m].
pub const RADIUS: f64 = 0.02;

/// 공 질량 [kg] (ITTF ≈ 2.7 g).
pub const MASS: f64 = 0.0027;

/// 체적평균 밀도 [kg/m³] = `MASS / (4/3 π R³)` ≈ 80.6.
/// 실제 공은 중공 셸(재질 밀도 ~1400 kg/m³, 벽 ~0.4 mm).
pub const BULK_DENSITY: f64 = MASS / ((4.0 / 3.0) * std::f64::consts::PI * RADIUS * RADIUS * RADIUS);

/// 얇은 중공 구 셸 관성모멘트 [kg·m²] — `I = (2/3) m R²` (직경축).
///
/// Rapier 기본 구는 균질 솔리드 `I = (2/5) m R²`라서, collider에는 이 값을
/// [`MassProperties`]로 직접 넣는다.
pub const SHELL_INERTIA: f64 = (2.0 / 3.0) * MASS * RADIUS * RADIUS;

/// Rapier 공 angular damping — 규격 없음. 바운스 마찰 ω가 Magnus로 폭주하지 않게.
pub const ANGULAR_DAMPING: f64 = 0.8;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bulk_density_is_mass_over_sphere_volume() {
        let volume = (4.0 / 3.0) * std::f64::consts::PI * RADIUS.powi(3);
        assert!((BULK_DENSITY * volume - MASS).abs() < 1e-15);
        assert!((BULK_DENSITY - 80.6).abs() < 0.1);
    }

    #[test]
    fn shell_inertia_is_two_thirds_m_r_squared() {
        let expected = (2.0 / 3.0) * MASS * RADIUS * RADIUS;
        assert!((SHELL_INERTIA - expected).abs() < 1e-18);
        // 솔리드 구 (2/5)보다 크다.
        let solid = (2.0 / 5.0) * MASS * RADIUS * RADIUS;
        assert!(SHELL_INERTIA > solid);
    }
}
