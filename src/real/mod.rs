//! 실기 연속 급구 런타임 — `--mode real` 전용 (bin 로컬, 라이브러리 아님).
//!
//! 1·2차: 스윙 완주·센터 복귀 후 다음 급구를 반복한다. 결선(진짜 랠리)은 아직 아니다.
//! 설계: `docs/superpowers/specs/2026-07-31-real-continuous-feed-design.md`  
//! 계획: `docs/superpowers/plans/2026-07-31-real-continuous-feed.md`
//!
//! # 동시성
//!
//! [`tools/jog`]의 불변식 — "동기화한 포즈 스냅샷 하나로 계획하고, 그 궤적을 그대로 보낸다" —
//! 을 채널 + 단일 소유권으로 **구조적으로** 강제한다. 공유 가변 상태가 없어서 race condition이
//! "안 생기게 조심하는" 게 아니라 표현 불가능하다.
//!
//! | 상태 | 유일한 소유자 |
//! |------|--------------|
//! | `FrameSource` + `Detector` | [`camera_worker`] (캠당 1 스레드) |
//! | `Ekf` · `Calibration` · 게이트 | [`estimator_worker`] |
//! | `Hardware` (버스 · 레일 · 샷 루프) | [`control_worker`] |
//! | highgui 창 | 메인 스레드 ([`PreviewWindow`]) |
//!
//! `read_pose → plan_best → command` 세 단계가 전부 [`control_worker`] 안에서만 일어난다.
//! 추정 스레드는 로봇 포즈를 **볼 수 없어서** 낡은 포즈로 계획하는 일이 애초에 불가능하다.
//! Recovering 동안 추정은 `Attempt`를 보내지 않는다 (`ControlStatus`).
//!
//! ```text
//!   cam-left  ─┐
//!              ├─ VisionEvent ──►  estimator ──┬─ CommitRequest ──►  control
//!   cam-right ─┘   bounded, drop-on-full       │   bounded(1)        (Hardware 단독 소유)
//!                                              │◄─ ControlStatus ───┘
//!                                 PreviewEvent │   Ready / Recovering
//!                                 drop-on-full ▼
//!                                           main (highgui + 로그; 세션 유지)
//! ```
//!
//! [`tools/jog`]: https://github.com/luv-all/pingpong-bot/tree/main/tools/jog

pub mod camera_worker;
pub mod control_worker;
pub mod estimator_worker;

mod ball_receding;
mod commit_request;
mod control_status;
mod decision;
pub mod fmt;
mod options;
mod paced_source;
mod preview;
mod preview_event;
mod run;
mod shot_event;
mod shutdown;
mod sim_child;
mod sim_host;
mod sim_update;
mod throttle;
mod track_request;
mod vision_event;

pub use commit_request::CommitRequest;
pub use control_status::ControlStatus;
pub use decision::{Decision, decide};
pub use options::Options;
pub use paced_source::PacedSource;
pub use preview::PreviewWindow;
pub use preview_event::PreviewEvent;
pub use shot_event::ShotEvent;
pub use shutdown::{Shutdown, ShutdownGuard, shutdown_channel};
pub use sim_update::{PoseMsg, SimUpdate, SwingMsg};
pub use throttle::Throttle;
pub use track_request::TrackRequest;
pub use vision_event::VisionEvent;

pub use run::run;
pub use sim_child::run as run_sim_child;

/// 관전용 sim 창을 띄우는 내부 플래그 (사용자 대상 아님).
pub const SIM_CHILD_FLAG: &str = "--sim-child";
