use crate::vision::trigger::{Evidence, Trigger};

/// 두 캠이 **각자** 충분히 본 뒤에만 건다.
///
/// [`SigmaThreshold`](super::SigmaThreshold)와 뭐가 다른가: σ는 적합에서 나온
/// **파생값**이라 우회당할 수 있다. 실측 fly_48에서 우캠이 서브 스윙 중 23프레임을
/// 놓쳤을 때 σ는 정직하게 컸는데(`sigma_v.x`가 문턱의 5배), 위치만 보는
/// [`PlaneCrossing`](super::PlaneCrossing)이 `Any` 안에서 그걸 그냥 지나쳐 버렸다.
/// 관측 **개수**는 파생값이 아니라 셈이라 그런 우회가 성립하지 않는다 — 어떤 조건과
/// `All`로 묶든 하방이 먼저 막힌다.
///
/// 왜 총 관측 수가 아니라 **스테레오 짝**인가: 단안 관측은 시선 방향 깊이를 거의
/// 안 묶는다. fly_48은 트리거 시점 관측이 26개나 됐지만 그중 우캠은 두어 개뿐이라
/// 총계로 세면 정상 트랙과 구분이 안 된다. 캠별 개수의 최솟값도 근사일 뿐이다 —
/// 한쪽이 여섯 번 봤어도 그게 다 반대편이 못 본 순간이면 짝은 0이다. 짝을 직접
/// 세면 그 착시가 없고, 그 수가 곧 깊이가 묶인 표본 수다.
pub struct StereoSamples {
    /// 두 캠이 같은 순간에 함께 본 표본이 최소 몇 개는 있어야 하나.
    pub min_samples: usize,
}

impl Trigger for StereoSamples {
    fn name(&self) -> &'static str {
        return "stereo";
    }

    fn ready(&self, evidence: &Evidence) -> bool {
        return evidence.stereo_samples >= self.min_samples;
    }
}
