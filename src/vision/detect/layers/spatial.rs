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
