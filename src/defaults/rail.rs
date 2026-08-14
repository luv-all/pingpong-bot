//! 리니어 레일 좌표계·프레임 SSOT — 영점·범위·마운트·모션 상수를 한 곳에 모은다.
//!
//! 물리 규격(단면 두께 등)은 [`crate::constants::geometry::RAIL_THICKNESS`]에 남는다 —
//! CAD 실측 규격은 `constants`, 배선·튜닝값은 `defaults`가 맞는 자리라서다.

use crate::robot::RailFrame;

/// 바닥(z=0)에서 레일 프로파일 하단까지의 실측 높이 [m].
/// 2026-08-13 설치 위치를 기존 0.88m에서 12cm 낮췄다.
pub const RAIL_BOTTOM_Z_M: f64 = 0.760;
/// 레일 위 로봇 베이스의 월드 Z [m] — 프로파일 하단 + 고정 두께.
///
/// 이 값이 바뀌면 로봇 베이스가 옮겨지고, [`crate::defaults::robot::READY_JOINTS_4DOF`]의
/// FK로 정의되는 준비 라켓 높이(`crate::defaults::motion::ready_racket_height_m`)도
/// 그 FK를 통해 자동으로 같이 이동한다 — 따로 맞출 필요가 없다.
pub const RAIL_MOUNT_Z_M: f64 = RAIL_BOTTOM_Z_M + crate::constants::geometry::RAIL_THICKNESS;

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
pub const RAIL_POSITIVE_X_TRIM_M: f64 = 0.000;
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
/// AXL 위치 단위 1m당 엔코더 펄스 수 [pulse/m].
///
/// 기존 250,000에서 논리 0.50m 명령이 좌우 모두 실측 0.52m였으므로
/// `250_000 * 0.50 / 0.52 = 240_384.6`을 반올림했다. 방향별 결과가 같아
/// 영점·백래시가 아니라 전역 거리 스케일로 반영한다.
pub const RAIL_PULSES_PER_METER: u32 = 240_385;
/// 실기 AXL 레일 가속/감속 [m/s²] — `RailConfig::default()`도 이 값을 쓴다.
/// 짧은 정렬 이동에서는 7.5m/s 최고속도보다 가속도 제한이 먼저 걸리므로,
/// 기존 16m/s²보다 빠른 정렬 응답을 위해 24m/s²를 사용한다.
pub const RAIL_ACCEL_M_S2: f64 = 24.0;
/// 홈잉 이동 속도 [m/s] — `min_vel`보다 크고 `max_vel`보다 훨씬 작다. 엔드스톱에
/// 부딪히는 순간의 충격·오버런을 줄이려는 값이다.
pub const RAIL_HOMING_VELOCITY_M_S: f64 = 0.02;
/// 홈잉 완료 후 준비 위치로 복귀할 때의 속도 [m/s].
///
/// `RailConfig::vel`(기본 `RAIL_MAX_SPEED` 7.5 m/s)을 그대로 쓰면 엔드스톱에 막
/// 부딪힌 직후 전속력으로 복귀하게 된다 — 홈잉 속도(0.02)보다는 빠르되 정상 운전
/// 속도보다는 훨씬 느린 중간값을 쓴다.
pub const RAIL_HOMING_RETURN_VELOCITY_M_S: f64 = 0.10;
/// 홈잉 중 알람 대기 타임아웃 [s].
///
/// 전체 물리 범위(`RAIL_PHYSICAL_X_MAX_M` - `RAIL_PHYSICAL_X_MIN_M` ≈ 1.41m)를
/// `RAIL_HOMING_VELOCITY_M_S`(0.02 m/s)로 끝까지 가는 데 최대 ~70초가 걸린다.
/// 일반 이동에 쓰는 `MOVE_POLL_TIMEOUT`(30s)를 그대로 재사용하면 현재 위치가
/// 엔드스톱에서 먼 경우 도달 전에 타임아웃돼 정지하고, 다시 실행해야 남은 거리를
/// 이어서 가는 문제가 있었다 — 여유를 크게 둔다.
pub const RAIL_HOMING_TIMEOUT_SECS: f64 = 120.0;
/// 홈잉 이동이 현재 위치보다 얼마나 더 갈 수 있게 여유를 두는지 [m], 전체 물리
/// 범위 위에 더한다.
///
/// 목표를 `domain_to_board_abs(physical_x_{min,max}_m)`으로 계산하면 그 변환 자체가
/// 지금 갖고 있는(틀렸을 수 있는) `board_zero_domain_m`에 의존한다 — 재정렬하려는
/// 값으로 재정렬용 이동 목표를 계산하는 순환 오류다. 대신 홈잉 이동은 **현재 보드
/// 위치 + 방향 × (전체 범위 + 이 여유)**로 좌표계 원점과 무관하게 계산한다.
pub const RAIL_HOMING_OVERTRAVEL_MARGIN_M: f64 = 0.20;

/// 홈잉 결과 캘리브레이션 JSON 경로. `data/calibration.json`(카메라)과 같은 자리.
pub const DEFAULT_RAIL_CALIBRATION_PATH: &str = "data/rail_calibration.json";

/// [`DEFAULT_RAIL_CALIBRATION_PATH`]의 `PathBuf`.
pub fn rail_calibration_path() -> std::path::PathBuf {
    return std::path::PathBuf::from(DEFAULT_RAIL_CALIBRATION_PATH);
}

/// 레일 마운트 y [m] — [`rail_frame`]의 `mount_y`와 [`crate::defaults::motion`]의
/// 인터셉트 구간(`INTERCEPT_Y_MIN_M`/`INTERCEPT_Y_MAX_M`)이 공유하는 값. 인터셉트
/// 구간은 이 값에 대한 고정 오프셋으로 정의되므로, 마운트 실측이 바뀌어도 둘을
/// 따로 맞출 필요가 없다. (준비 타격 y `ready_racket_y_m`은 이 값이 아니라
/// [`crate::defaults::robot::READY_JOINTS_4DOF`]의 FK를 따른다 — 마운트가
/// 바뀌면 로봇 베이스가 옮겨져 그 FK도 자동으로 같이 이동한다.)
///
/// **2026-08-13 실측** — 이전 값 **-0.128**은 `mount_search`(2026-07-26)가 낮은
/// 베이스 기준으로 추천한 `behind=0.10`(y=−0.10)을, 이후 베이스 z 실측(0.935)에
/// 맞춰 대체한 값이었다. 이번 실측으로 -0.068로 갱신한다.
pub const RAIL_MOUNT_Y_M: f64 = -0.068;

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·바닥 기준).
///
/// **높이는 실측(2026-08-13).** 바닥→프로파일 하단은 [`RAIL_BOTTOM_Z_M`](0.76 m),
/// 두께 [`RAIL_THICKNESS`](crate::constants::geometry::RAIL_THICKNESS) 0.055 m →
/// 베이스 z는 [`RAIL_MOUNT_Z_M`](0.815). 기존 프로파일 하단 0.88m(베이스 z
/// 0.935)에서 12cm 내린 설치값이다. 그 0.88m 자체는 `SURFACE_Z + 0.05` = 0.81로
/// "실기 브래킷(~면 위 3~5cm)과 맞춤"이라는 추정에 기대고 있었는데 2026-07-30
/// 실측이 그 가정을 뒤집었었다 — 시뮬 베이스가 실물보다 12.5 cm 낮았다.
///
/// `mount_y`는 [`RAIL_MOUNT_Y_M`] 참고 — `mount_search`(2026-07-26)가 낮은
/// 베이스 기준으로 추천한 `behind=0.10`(y=−0.10)은 그 스윕이 **낮은 베이스
/// 기준**이라, 지금 다시 낮아진 베이스(0.815)에서는 0.935 시절보다 오히려
/// 더 근접한 참고값이다.
///
/// 두 값 모두 sim GUI "Rig" 패널에서 공이 주차된 동안 런타임 조정 가능하다
/// (`SimRuntimeControls::rail_frame`). 좋은 위치를 눈으로 찾은 뒤
/// `mount_search`/`--rest-pose-search`를 그 위치에서 다시 돌려 여기와
/// [`crate::defaults::robot::READY_JOINTS_4DOF`]를 확정하는 것이 순서다.
pub fn rail_frame() -> RailFrame {
    return RailFrame {
        // 2026-08-13 실측 양쪽 마진이 말하는 원점차 9.00cm/9.05cm의 평균.
        mount_x: 0.09025,
        mount_y: RAIL_MOUNT_Y_M,
        rail_bottom_z: RAIL_BOTTOM_Z_M,
    };
}
