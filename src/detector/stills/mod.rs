//! 정량 eval용 스틸 GT — `data/detect_stills/manifest.json`.
//!
//! 비디오 전 프레임 라벨은 비범위. 클립 타임라인을 등분한 ~10장에
//! 공 중심(`pixel`) 또는 무공(`null`)만 준다.

mod still_item;
mod stills_manifest;

pub use still_item::StillItem;
pub use stills_manifest::StillsManifest;
