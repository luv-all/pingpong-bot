//! 평가 프로토콜 — 좌/중/우 각 10발, 0~3점.

mod flags;
mod launch_params;
pub(crate) mod live_observer;
mod mode;
mod progress;
mod protocol;
mod report;
mod shot;
mod zone;
mod zone_score;

pub use crate::defaults::sim::{
    EVAL_MAX_SCORE as MAX_SCORE, EVAL_NET_PASSTHROUGH_RETRIES,
    EVAL_PASS_SCORE_EXCLUSIVE as PASS_SCORE_EXCLUSIVE, EVAL_PITCH_JITTER_DEG,
    EVAL_RACKET_REHIT_MIN_STEPS as RACKET_REHIT_MIN_STEPS, EVAL_SHOTS_PER_ZONE as SHOTS_PER_ZONE,
    EVAL_SPEED_JITTER_MPS, EVAL_TOTAL_SHOTS as TOTAL_SHOTS, EVAL_YAW_JITTER_DEG,
};

pub use flags::Flags;
pub use launch_params::LaunchParams;
pub use live_observer::LiveObserver;
pub use mode::Mode;
pub use progress::Progress;
pub use protocol::Protocol;
pub use report::Report;
pub use shot::Shot;
pub use zone::Zone;
pub use zone_score::ZoneScore;
