//! 메인↔캡처 공유 상태.

use crate::preview_slot::PreviewSlot;

#[derive(Default)]
pub struct CaptureShared {
    pub preview: Option<PreviewSlot>,
    /// 마지막 저장 결과 메시지 (HUD/콘솔)
    pub last_status: Option<String>,
    pub error: Option<String>,
}
