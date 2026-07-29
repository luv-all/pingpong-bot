//! 풀 sim 뷰어 옵션.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use crate::robot::urdf::UrdfModel;
use crate::sim::physics::world::SimWorld;
use crate::sim::session::controls::SimRuntimeControls;

/// sim 3D + 제어 패널 옵션.
pub struct SimViewerOptions {
    /// 발사·sim 설정
    pub controls: Arc<Mutex<SimRuntimeControls>>,
    /// 공유 sim 월드
    pub world: Arc<Mutex<SimWorld>>,
    /// URDF 모델 (kiss3d 로봇 mesh 대신 사용)
    pub urdf: Option<Arc<UrdfModel>>,
    /// 창 닫을 때 파이프라인 종료
    pub shutdown: Arc<AtomicBool>,
}
