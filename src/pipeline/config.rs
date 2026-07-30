//! 파이프라인 실행 설정.

use std::sync::Arc;

use crate::camera::Calibration;
use crate::defaults::shared_robot;
use crate::robot::Robot;
use crate::robot::motion::InterceptWindow;

const CONTROL_HZ: f64 = 100.0;

/// 파이프라인 실행 설정.
pub struct PipelineConfig {
    /// 실제 도달 가능한 타격점을 탐색할 y 구간.
    pub intercept: InterceptWindow,
    /// 제어 루프 주파수 [Hz]
    pub control_hz: f64,
    /// sim·real 공통 불변 로봇 모델 (plan §2, §7.2)
    pub robot: Arc<Robot>,
    /// 카메라 캘리브 (삼각측량)
    pub calibration: Calibration,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        return Self {
            intercept: InterceptWindow::default(),
            control_hz: CONTROL_HZ,
            robot: shared_robot(),
            calibration: Calibration::sim(3),
        };
    }
}
