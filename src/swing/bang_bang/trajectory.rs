//! bang-bang 적분 샘플 궤적.

use crate::robot::Joints;

/// bang-bang 적분으로 얻은 샘플 기반 궤적. quintic처럼 닫힌 형태 계수가
/// 아니라 매 스텝 실제 좌표를 그대로 담는다 — `sample_at`/`sample_rail_at`은
/// 가장 가까운 두 샘플을 선형보간한다.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    pub(crate) dt: f64,
    pub(crate) joint_samples: Vec<Joints>,
    pub(crate) rail_samples: Vec<f64>,
}

impl Trajectory {
    pub fn duration_secs(&self) -> f64 {
        return (self.joint_samples.len().saturating_sub(1)) as f64 * self.dt;
    }

    fn sample_index(&self, t: f64) -> (usize, usize, f64) {
        let clamped = t.clamp(0.0, self.duration_secs());
        let raw = clamped / self.dt;
        let lo = (raw.floor() as usize).min(self.joint_samples.len() - 1);
        let hi = (lo + 1).min(self.joint_samples.len() - 1);
        let frac = if hi == lo { 0.0 } else { raw - lo as f64 };
        return (lo, hi, frac);
    }

    pub fn sample_at(&self, t: f64) -> Joints {
        let (lo, hi, frac) = self.sample_index(t);
        let a = &self.joint_samples[lo];
        let b = &self.joint_samples[hi];
        let values = a
            .values
            .iter()
            .zip(&b.values)
            .map(|(x, y)| x + (y - x) * frac)
            .collect();
        return Joints { values };
    }

    pub fn sample_rail_at(&self, t: f64) -> f64 {
        let (lo, hi, frac) = self.sample_index(t);
        return self.rail_samples[lo] + (self.rail_samples[hi] - self.rail_samples[lo]) * frac;
    }

    /// `t` [s]에서 관절 각속도 [rad/s].
    ///
    /// 닫힌 형태 계수가 없어 인접 샘플 차분으로 근사한다 — 적분 스텝이
    /// [`PLAN_DT_SECS`](1 ms)라 quintic의 해석 미분과 같은 수준으로 매끄럽다.
    pub fn sample_velocity_at(&self, t: f64) -> Vec<f64> {
        let (lo, hi, _) = self.sample_index(t);
        if hi == lo {
            return vec![0.0; self.joint_samples[lo].values.len()];
        }
        return self.joint_samples[lo]
            .values
            .iter()
            .zip(&self.joint_samples[hi].values)
            .map(|(a, b)| (b - a) / self.dt)
            .collect();
    }

    pub fn end_joints(&self) -> &Joints {
        return self.joint_samples.last().expect("최소 1개 샘플");
    }

    pub fn follow_through_rail_x(&self) -> f64 {
        return *self.rail_samples.last().expect("최소 1개 샘플");
    }
}
