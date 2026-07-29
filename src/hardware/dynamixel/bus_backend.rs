#[cfg(feature = "real")]
use super::real_backend::RealBackend;

pub(super) enum BusBackend {
    DryRun {
        /// `motor_ids` 순서 Present/Goal (읽기·논리 관절).
        ticks: Vec<i32>,
        /// 마지막 Goal SyncWrite 전체 (미러 슬레이브 포함).
        last_bus_goals: Vec<(u8, i32)>,
        /// 마지막 Goal Current (논리 모터 순서, signed units).
        last_goal_currents: Vec<i16>,
    },
    #[cfg(feature = "real")]
    Real(RealBackend),
}
