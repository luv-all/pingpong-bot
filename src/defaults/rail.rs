//! 리니어 레일 좌표계·프레임 SSOT — 영점·범위·마운트·모션 상수를 한 곳에 모은다.
//!
//! 물리 규격(단면 두께 등)은 [`crate::constants::geometry::RAIL_THICKNESS`]에 남는다 —
//! CAD 실측 규격은 `constants`, 배선·튜닝값은 `defaults`가 맞는 자리라서다.

use crate::robot::RailFrame;

/// 실기 좌측 안전 마진 [m].
pub const RAIL_LEFT_END_MARGIN_M: f64 = 0.0100;
/// 실기 우측 안전 마진 [m].
pub const RAIL_RIGHT_END_MARGIN_M: f64 = 0.0705;
/// 실기에서 확인한 레일 좌표 범위 [m].
pub const RAIL_PHYSICAL_X_MIN_M: f64 = 0.0;
pub const RAIL_PHYSICAL_X_MAX_M: f64 = 1.41;
/// AXL 보드 실측 원점(보드 0.0m)에 대응하는 제어 좌표 [m].
///
/// 레일 기하학적 원점에 더하는 논리 +X 좌표계 보정 [m].
/// 타격 목표나 IK 결과가 아니라 AXL board↔domain 좌표 변환에 한 번만 적용한다.
/// `reverse=true`이므로 실물 +X 2.5cm 보정은 보드 목표에서 2.5cm를 뺀다.
/// 기존 +4.0cm 기준에서 2.5cm를 뺀 최종 좌표 오프셋이다.
pub const RAIL_COORDINATE_POSITIVE_X_OFFSET_M: f64 = 0.015;
/// 보드 실측 0.745m를 준비 중앙 0.675m로 해석하는 영점 이동.
pub const RAIL_POSITIVE_X_TRIM_M: f64 = 0.030;
pub const RAIL_NEGATIVE_X_ZERO_SHIFT_M: f64 =
    (RAIL_PHYSICAL_X_MAX_M - RAIL_PHYSICAL_X_MIN_M) / 2.0 - RAIL_POSITIVE_X_TRIM_M;
/// **`--calibrate-rail` 홈잉 미실행 시 폴백 기본값.** 홈잉을 한 번이라도 실행하면
/// `data/rail_calibration.json`의 값이 런타임에 이 상수를 덮어쓴다
/// (`hardware::rail::rail_calibration`).
pub const RAIL_BOARD_ZERO_DOMAIN_M: f64 =
    0.7050 + RAIL_COORDINATE_POSITIVE_X_OFFSET_M + RAIL_NEGATIVE_X_ZERO_SHIFT_M;
/// sim·real 공통 이동 범위 [m].
pub const RAIL_X_MIN_M: f64 = RAIL_PHYSICAL_X_MIN_M + RAIL_LEFT_END_MARGIN_M;
pub const RAIL_X_MAX_M: f64 = RAIL_PHYSICAL_X_MAX_M - RAIL_RIGHT_END_MARGIN_M;
/// 탁구대 실측 중앙 보정 위치 [m].
pub const RAIL_READY_X_M: f64 = 0.6750;
/// 최대 이동 속도 [m/s].
pub const RAIL_MAX_SPEED: f64 = 7.5;
/// 실기 AXL 레일 가속/감속 [m/s²] — `RailConfig::default()`도 이 값을 쓴다.
///
/// 기존 24 m/s²는 최단시간 이동과 겹치면 출발·정지 충격이 크다.
/// 실물 안전 운전은 12 m/s²로 낮춰 예측 보정 중에도 부드럽게
/// 가감속하고, 시뮬레이션과 계획기도 같은 한계를 사용한다.
pub const RAIL_ACCEL_M_S2: f64 = 12.0;
/// 홈잉 이동 속도 [m/s] — `min_vel`보다 크고 `max_vel`보다 훨씬 작다. 엔드스톱에
/// 부딪히는 순간의 충격·오버런을 줄이려는 값이다.
pub const RAIL_HOMING_VELOCITY_M_S: f64 = 0.02;
/// 홈잉 중 알람 대기 타임아웃 [s].
///
/// 전체 물리 범위(`RAIL_PHYSICAL_X_MAX_M` - `RAIL_PHYSICAL_X_MIN_M` ≈ 1.41m)를
/// `RAIL_HOMING_VELOCITY_M_S`(0.02 m/s)로 끝까지 가는 데 최대 ~70초가 걸린다.
/// 일반 이동에 쓰는 `MOVE_POLL_TIMEOUT`(30s)를 그대로 재사용하면 현재 위치가
/// 엔드스톱에서 먼 경우 도달 전에 타임아웃돼 정지하고, 다시 실행해야 남은 거리를
/// 이어서 가는 문제가 있었다 — 여유를 크게 둔다.
pub const RAIL_HOMING_TIMEOUT_SECS: f64 = 120.0;

/// 홈잉 결과 캘리브레이션 JSON 경로. `data/calibration.json`(카메라)과 같은 자리.
pub const DEFAULT_RAIL_CALIBRATION_PATH: &str = "data/rail_calibration.json";

/// [`DEFAULT_RAIL_CALIBRATION_PATH`]의 `PathBuf`.
pub fn rail_calibration_path() -> std::path::PathBuf {
    return std::path::PathBuf::from(DEFAULT_RAIL_CALIBRATION_PATH);
}

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·바닥 기준).
///
/// **높이는 실측(2026-07-30).** 바닥→프로파일 하단 0.88 m,
/// 두께 [`RAIL_THICKNESS`](crate::constants::geometry::RAIL_THICKNESS) 0.055 m →
/// 베이스 z = **0.935**. 이전 값은 `SURFACE_Z + 0.05` = 0.81로, "실기 브래킷
/// (~면 위 3~5cm)과 맞춤"이라는 추정에 기대고 있었는데 실측이 그 가정을
/// 뒤집었다 — 시뮬 베이스가 실물보다 12.5 cm 낮았다.
///
/// `mount_y`는 실측값 **-0.128**을 쓴다 — `mount_search`(2026-07-26)가 낮은
/// 베이스 기준으로 추천한 `behind=0.10`(y=−0.10, `behind=0.02` 대비 ratio≤1이
/// **10/150**, mean≈2.48)은 그 스윕이 **낮은 베이스 기준**이라 0.935에서는
/// 최적값이 아니었고, 이후 실측이 이 값으로 대체했다.
///
/// 두 값 모두 sim GUI "Rig" 패널에서 공이 주차된 동안 런타임 조정 가능하다
/// (`SimRuntimeControls::rail_frame`). 좋은 위치를 눈으로 찾은 뒤
/// `mount_search`/`--rest-pose-search`를 그 위치에서 다시 돌려 여기와
/// [`crate::defaults::robot::READY_JOINTS_4DOF`]를 확정하는 것이 순서다.
pub fn rail_frame() -> RailFrame {
    return RailFrame {
        mount_y: -0.128,
        rail_bottom_z: 0.88,
    };
}
