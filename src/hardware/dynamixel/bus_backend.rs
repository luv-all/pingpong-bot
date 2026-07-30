#[cfg(feature = "real")]
use super::real_backend::RealBackend;

pub(super) enum BusBackend {
    DryRun {
        /// `motor_ids` 순서 Present/Goal (읽기·논리 관절).
        ticks: Vec<i32>,
        /// 마지막 Goal SyncWrite 전체 (미러 슬레이브 포함).
        last_bus_goals: Vec<(u8, i32)>,
        /// 마지막 Operating Mode 값.
        last_operating_mode: Option<u8>,
        /// 마지막 PWM Limit SyncWrite (버스 ID, 값).
        last_pwm_limits: Vec<(u8, u16)>,
        /// 마지막 Current Limit SyncWrite (MX-64 ID만).
        last_current_limits: Vec<(u8, u16)>,
    },
    #[cfg(feature = "real")]
    Real(RealBackend),
}
