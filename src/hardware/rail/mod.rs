//! 리니어 레일 — 설정 + AXL 드라이버.

mod config;
#[cfg(all(windows, feature = "real"))]
#[allow(dead_code)]
mod axl_ffi;
mod axl;

pub use axl::AxlRail;
pub use config::{RailConfig, RailConfigError, SoftLimitArgs};
