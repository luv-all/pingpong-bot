//! 비행 부피 밖(테이블 x·y 변 너머 바닥) 끄기. 배경 차분보다 앞에 둔다 — 사람이 서
//! 있는 자리는 애초에 정지·움직임을 안 가리고 통째로 지운다.
//!
//! 계산은 [`crate::detector::FloorEdgeMask`] 재사용 — 월드 `x ≥ W+δ`·`y ≥ L+δ`를 캘리브로
//! 이미지에 투영해 만든 keep 폴리곤. 캠 고정이라 프레임마다 다시 만들 이유가 없다.
//!
//! `0 ≤ x ≤ W` 공중(막 던진 공)만 다시 열어주는 안을 2026-08-11에 실측했다 — fly_22·23
//! RMSE가 도로 58·100cm까지 올라갔다. 사람이 테이블 폭 전체로 팔을 뻗어 받아치므로
//! 서 있는 자리 x-범위가 공 궤적 x-범위와 거의 겹친다 — x·y만으로는 둘을 못 가른다.
//! 그래서 되돌렸다: `y ≥ L+δ`는 높이 안 가리고 통째로 지운다.
//!
//! **y 컷도 서브 토스를 잘라낸다** — 2026-08-13에 fly_45~53 `_sim.png`에서 파란(적합)
//! 궤적이 하나같이 한참 뒤늦게 시작하는 걸 보고 확인했다: y컷을 통째로 꺼 보니
//! 트리거 시점 관측 수가 클립마다 거의 두 배로 뛰었다(예: fly_48 14→31,
//! fly_51 13→32) — 서버가 서 있는 y≥L 쪽에서 토스가 뜨는 동안 실제 공 검출이
//! 잘려나가고 있었다는 뜻. 근데 그냥 끄면 사람 피부색이 그 자리로 새 들어와
//! RMSE가 20~40cm대에서 76~185cm로 폭발했다(트랙 수도 거의 2배, 일부는 잔차 NaN).
//! 완화책 두 가지도 실측했다 — Y컷 여유 +0.5m, `MIN_CIRCULARITY` 0.4·0.5로 올려
//! 팔·몸통을 원형도로 거르기(배경 차분이 못 거르므로) — 셋 다 관측 수는 늘어도
//! 0.2s 리드 오차 중앙값이 1.3→2.4~3.1cm로 오히려 나빠졌다. 실제 공 검출 중 일부가
//! 이미 원형도 0.35 문턱 가까이서 도는 클립이 있어서(모션블러·원거리), 문턱을
//! 올리면 팔보다 진짜 공을 더 잃는다. 세 실험 다 되돌림 — 색상·모양만으로는 이
//! 자리의 사람과 공을 못 가른다. 제대로 풀려면 검출기 자체(질감·움직임 일관성
//! 등 다른 단서)를 손봐야 한다, 마스크·문턱 조정으로는 안 됨.

use anyhow::Result;
use opencv::core;

use crate::camera::{self, Frame};
use crate::detector::FloorEdgeMask;

use super::super::{Layer, Mask};

pub struct Spatial {
    keep: FloorEdgeMask,
    scratch: Mask,
}

impl Spatial {
    pub fn from_params(params: &camera::Params) -> Result<Self> {
        return Ok(Self {
            keep: FloorEdgeMask::from_params(params)?,
            scratch: Mask::default(),
        });
    }
}

impl Layer for Spatial {
    fn name(&self) -> &'static str {
        return "spatial";
    }

    fn narrow(&mut self, _frame: &Frame, mask: &mut Mask) -> Result<()> {
        core::bitwise_and(
            &*mask,
            &self.keep.keep,
            &mut self.scratch,
            &core::no_array(),
        )?;
        std::mem::swap(mask, &mut self.scratch);
        return Ok(());
    }
}
