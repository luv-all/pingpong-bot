//! 시뮬·랜덤 샷·평가 프로토콜 휴리스틱.

use crate::sim::launch;

/// 랜덤 샷: 네트 통과 재시도 상한.
pub const RANDOM_SHOT_NET_GATE_MAX_TRIES: usize = 48;
pub const RANDOM_SHOT_LATERAL_MIN_M: f64 = -0.5;
pub const RANDOM_SHOT_LATERAL_MAX_M: f64 = 0.5;
pub const RANDOM_SHOT_TARGET_PADDING_M: f64 = 0.25;
/// 랜덤 샷 속도 하한 [m/s] — **로봇에 실제로 닿는 공**의 최저 속도.
///
/// 이 값보다 느리면 공이 마지막 바운스 뒤 로봇 앞에서 굴러 멈춰
/// hit plane(가장 먼 y=0.35)에 도달하지 못한다. 그러면 `predict_impact`가
/// 매 스텝 `None`이라 `plan_coarse_track`이 아예 호출되지 않고, 로봇은
/// 커밋도 포기도 못 한 채 공만 지나간다.
///
/// 실측(좌우 5 × yaw 5 = 25개 격자, 순수 탄도, 기본 pitch/높이):
///
/// | 속도 | 도달/25 | 최악 min-y |
/// |------|---------|-----------|
/// | 5.5  |  6/25   | 1.395 |
/// | 5.6  |  8/25   | 1.394 |
/// | 5.7  | 16/25   | 1.395 |
/// | 5.8  | 21/25   | 1.388 |
/// | 5.9  | **25/25** | **−0.147** |
/// | 6.0+ | 25/25   | −0.146 |
///
/// 절벽이다 — 5.9에서 최악 min-y가 1.388(테이블 위에서 멈춤)에서
/// −0.147(로봇을 0.5 m 지나침)로 한 번에 넘어간다. 코너 샷일수록 비행거리가
/// 길어 더 빠른 속도가 필요하므로 격자 **최악값** 기준으로 잡았다.
///
/// 6.0을 고른 이유: 절벽(5.9)에서 한 칸 여유가 있고, 기본 샷 속도
/// (`BallShooterSettings::default().speed_mps`)와 같아 "시중 슈터 초보~중급
/// 피딩 하단"이라는 기준점과 일치한다. 근거 측정은
/// `diag_random_shot_speed_reachability` (`sim::physics::world` 테스트).
pub const RANDOM_SHOT_SPEED_MIN_MPS: f64 = 6.0;
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

/// GUI Random이 고정하는 발사구·자세 (실측 슈터).
///
/// 발사구 = `(WIDTH_X/2, LENGTH_Y − inset_y, SURFACE_Z + height_z)`.
/// yaw만 `RANDOM_SHOT_FIXED_YAW_DEGS` 중 하나를 고르고, 속도·스핀만 랜덤.
pub const RANDOM_SHOT_FIXED_MUZZLE_INSET_Y_M: f64 = 0.275;
pub const RANDOM_SHOT_FIXED_MUZZLE_HEIGHT_Z_M: f64 = 0.265;
pub const RANDOM_SHOT_FIXED_PITCH_DEG: f64 = 15.0;
pub const RANDOM_SHOT_FIXED_ROLL_DEG: f64 = 0.0;
pub const RANDOM_SHOT_FIXED_YAW_DEGS: [f64; 3] = [-10.0, 0.0, 10.0];

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

impl Default for launch::Settings {
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
