//! 카메라 하드웨어 datasheet·규격 (고정값).
//!
//! USB device·스트림 **요청**·캘리브 임계는 [`crate::defaults::calib`].

pub mod arducam_b0332;

pub use arducam_b0332::{
    BUFFER_SIZE, EFL_MM, FOURCC_MJPG, FOURCC_YUY2, FPS_MJPG, FPS_YUY2, HEIGHT, HFOV_DEG, VFOV_DEG,
    WIDTH,
};

/// 탁구대 table-PnP 랜드마크 개수 (규격 8점).
pub const TABLE_LANDMARK_COUNT: usize = 8;
