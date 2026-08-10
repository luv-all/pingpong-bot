//! 실기 라켓·리니어 레일 정렬·후속 팔 보정 런타임 (`--mode real`).
//!
//! # 동시성
//!
//! 채널과 단일 소유권으로 카메라·추정·하드웨어 상태를 분리한다.
//!
//! | 상태 | 유일한 소유자 |
//! |------|--------------|
//! | `FrameSource` + 새 `vision::Detector` | [`camera_worker`] (캠당 1 스레드) |
//! | `vision::Fit` · `Calibration` | [`estimator_worker`] |
//! | `Hardware` (Dynamixel · AXL 레일) | [`control_worker`] |
//! | highgui 창 | 메인 스레드 ([`PreviewWindow`]) |
//!
//! ```text
//!   cam-left  ─┐
//!              ├─ VisionEvent ──►  estimator ──┬─ CommitRequest ──►  control
//!   cam-right ─┘   bounded, drop-on-full       │   bounded(1)        (Hardware 단독 소유)
//!                                 PreviewEvent │
//!                                 drop-on-full ▼
//!                                           main (highgui + 로그)
//! ```

pub mod camera_worker;
pub mod control_worker;
pub mod estimator_worker;

mod commit_request;
pub mod fmt;
mod options;
mod paced_source;
mod preview;
mod preview_event;
mod run;
mod runtime_event;
mod shutdown;
mod sim_child;
mod sim_update;
mod test_control;
mod throttle;
mod vision_event;

pub use commit_request::CommitRequest;
pub use options::Options;
pub use paced_source::PacedSource;
pub use preview::PreviewWindow;
pub use preview_event::PreviewEvent;
pub use runtime_event::{ControlStateSnapshot, RuntimeEvent};
pub use shutdown::{Shutdown, ShutdownGuard, shutdown_channel};
pub use sim_update::{PoseMsg, SimUpdate};
pub use test_control::{TestControl, TestZone};
pub use throttle::Throttle;
pub use vision_event::VisionEvent;

pub use run::run;
pub use sim_child::run as run_sim_child;

/// 관전용 sim 창을 띄우는 내부 플래그 (사용자 대상 아님).
pub const SIM_CHILD_FLAG: &str = "--sim-child";
