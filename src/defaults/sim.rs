//! 시뮬·랜덤 샷·평가 프로토콜 휴리스틱.

use crate::shooter;

/// 랜덤 샷: 네트 통과 재시도 상한.
pub const RANDOM_SHOT_NET_GATE_MAX_TRIES: usize = 48;
pub const RANDOM_SHOT_LATERAL_MIN_M: f64 = -0.5;
pub const RANDOM_SHOT_LATERAL_MAX_M: f64 = 0.5;
pub const RANDOM_SHOT_TARGET_PADDING_M: f64 = 0.25;
pub const RANDOM_SHOT_SPEED_MIN_MPS: f64 = 5.7;
pub const RANDOM_SHOT_SPEED_MAX_MPS: f64 = 6.3;
pub const RANDOM_SHOT_HEIGHT_MIN_M: f64 = 0.22;
pub const RANDOM_SHOT_HEIGHT_MAX_M: f64 = 0.28;
pub const RANDOM_SHOT_TOPSPIN_MIN: f64 = -20.0;
pub const RANDOM_SHOT_TOPSPIN_MAX: f64 = 20.0;
pub const RANDOM_SHOT_SIDESPIN_MIN: f64 = -15.0;
pub const RANDOM_SHOT_SIDESPIN_MAX: f64 = 15.0;
pub const RANDOM_SHOT_PITCH_MIN_DEG: f64 = -4.0;
pub const RANDOM_SHOT_PITCH_MAX_DEG: f64 = -2.0;
pub const RANDOM_SHOT_ROLL_MIN_DEG: f64 = -15.0;
pub const RANDOM_SHOT_ROLL_MAX_DEG: f64 = 15.0;

/// eval_protocol 지터·합격선.
pub const EVAL_SPEED_JITTER_MPS: f64 = 0.15;
pub const EVAL_YAW_JITTER_DEG: f64 = 0.5;
pub const EVAL_PITCH_JITTER_DEG: f64 = 0.5;
pub const EVAL_NET_PASSTHROUGH_RETRIES: usize = 12;
pub const EVAL_SHOTS_PER_ZONE: usize = 10;
pub const EVAL_TOTAL_SHOTS: usize = EVAL_SHOTS_PER_ZONE * 3;
pub const EVAL_MAX_SCORE: u32 = (EVAL_TOTAL_SHOTS * 3) as u32;
pub const EVAL_PASS_SCORE_EXCLUSIVE: u32 = 45;
pub const EVAL_RACKET_REHIT_MIN_STEPS: u32 = 30;

impl Default for shooter::Settings {
    fn default() -> Self {
        // speed 6.0: 시중 슈터 초보~중급 피딩 하단.
        // height 0.24 / pitch −1.0: Rapier·ballistics 네트 통과 최소값.
        return Self {
            speed_mps: 6.0,
            yaw_deg: 0.0,
            pitch_deg: -1.0,
            roll_deg: 0.0,
            pos_offset_x_m: 0.0,
            pos_offset_y_m: 0.0,
            pos_offset_z_m: 0.0,
            lateral_offset_m: 0.0,
            height_offset_m: 0.24,
            topspin_rad_s: 0.0,
            sidespin_rad_s: 0.0,
            drill_spin_rad_s: 0.0,
        };
    }
}
