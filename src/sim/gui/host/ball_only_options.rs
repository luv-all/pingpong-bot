//! 하위 호환: 공유 슬롯으로 테이블+공 뷰어.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use super::super::scene::TableSceneOptions;
use crate::Point3;

/// 하위 호환: 공유 슬롯으로 테이블+공 뷰어.
pub struct BallOnlyViewerOptions {
    pub ball_position: Arc<Mutex<Option<Point3>>>,
    pub shutdown: Arc<AtomicBool>,
    pub table: TableSceneOptions,
}
