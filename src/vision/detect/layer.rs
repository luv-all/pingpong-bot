//! 캐스케이드 한 단계의 계약. 구현은 [`super::layers`].

use anyhow::Result;

use crate::camera::Frame;

use super::Mask;

/// 캐스케이드 한 단계. 켜진 픽셀을 끈다.
pub trait Layer: Send {
    /// 패널 제목·스윕 라벨.
    fn name(&self) -> &'static str;

    /// `frame`은 원본 그대로 두고 `mask`만 고친다. 뒤 단계도 원본 픽셀을 읽는다.
    ///
    /// 켜진 것을 끄기만 한다. 늘리면 뒤 단계의 비용 가정과 순서 교환이 깨진다.
    ///
    /// 실패를 삼키면 마스크가 열린 채로 다음 단계에 간다.
    fn narrow(&mut self, frame: &Frame, mask: &mut Mask) -> Result<()>;
}
