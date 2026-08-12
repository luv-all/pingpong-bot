//! 리니어 레일 — 설정 + AXL 드라이버.

#[cfg(all(windows, feature = "real"))]
#[allow(dead_code)]
mod axl_ffi;
#[cfg(all(windows, feature = "real"))]
mod axl_live;
mod axl_rail;
mod rail_calibration;
mod rail_config;
mod rail_config_error;
mod rail_kind;
mod soft_limit_args;

pub use axl_rail::{AxlRail, RailHomeResult};
pub use rail_calibration::RailCalibration;
pub use rail_config::{RailConfig, RailEnd};
pub use rail_config_error::RailConfigError;
pub use soft_limit_args::SoftLimitArgs;
