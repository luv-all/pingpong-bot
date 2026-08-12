# Design: 리니어 레일 정렬 통합 + 온디맨드 홈잉 (rail homing & calibration consolidation)

**작성일:** 2026-08-12
**상태:** 사용자 리뷰 대기
**범위:** `src/defaults/rail.rs`(신규), `src/defaults/hardware.rs`, `src/defaults/robot.rs`,
`src/defaults/motion.rs`, `src/hardware/rail/*`, `src/cli/args.rs`, `tools/jog`,
`data/rail_calibration.json`(신규), `docs/rail-calibration.md`(신규). 볼 정렬(`ALIGNMENT_*`,
`plan_ball_alignment*`) 로직·상태 머신은 건드리지 않는다 — 이름이 같을 뿐 이 spec의
"정렬"과는 다른 개념이다(§배경 참고).

---

## 배경

사용자가 겪는 문제 세 가지:

1. 레일/프레임 관련 상수가 `src/defaults/`와 `src/constants/` 여러 파일에 흩어져 있다
   (`defaults/hardware.rs`의 `RAIL_*` 영점/범위/오프셋, `defaults/robot.rs`의
   `rail_frame()`·`RAIL_MAX_SPEED`, `defaults/motion.rs`의 `RAIL_ACCEL_M_S2`,
   `constants/geometry.rs`의 `RAIL_THICKNESS`).
2. 리니어 레일 **정렬(영점 확인) 알고리즘이 없다** — 영점(`RAIL_BOARD_ZERO_DOMAIN_M`)과
   범위는 한 번 줄자로 측정해 하드코딩한 값이고, `AxlRail::open`(`axl_rail.rs:30`)은
   `AxlOpenNoReset`으로 보드 엔코더 위치를 그대로 신뢰할 뿐 물리적 기준과 대조해
   재확인하는 코드가 없다.
3. 이 측정을 다시 할 때 참고할 일관된 셋업 가이드가 없다 — 각주(`hardware.rs:15-27`,
   `robot.rs:94-110`)에 그날그날의 측정 기록만 남아 있다.

**용어 정리:** 이 코드베이스에서 이미 쓰이는 "정렬"(`ALIGNMENT_*` 상수, `Aligning` 상태,
`plan_ball_alignment*`, `src/robot/motion/physics.rs:585-680`)은 **공 요격 조준**을 뜻한다 —
예측된 공 위치에 라켓을 겨누는 다운스트림 로직이다. 이 spec이 다루는 건 그 입력값인
**레일 좌표계 자체의 물리적 영점/범위**이며, 위 볼 정렬 코드는 변경하지 않는다.

**참고 구현:** `github.com/luv-all/robot-pingpong-cpp`(`src/control/linear_motor.cpp`)가 동일한
AXL 보드를 쓰는 C++ 레퍼런스다. 여기서도 `AxmHomeSetMethod` 등으로 AXL 내장 홈잉을 설정만
해 두고 실제로는 호출하지 않으며, 대신 `AxmSignalReadServoAlarm`/`AxmSignalServoAlarmReset`으로
엔드스톱 충돌(알람)을 감지·해제한다. 이 spec은 그 패턴을 그대로 따른다 — AXL 내장
홈잉 시퀀스가 아니라, 저속 이동 + 알람 감지로 물리적 기준점을 얻는다.

## 목표 / 비목표

**목표**

- 레일 관련 상수를 `src/defaults/rail.rs` 한 곳으로 모은다.
- 실기에서 **온디맨드로** 실행 가능한 레일 홈잉 루틴을 추가한다: 저속으로 엔드스톱까지
  이동 → AXL 알람으로 도달 감지 → 알람 해제 → 그 지점 기준으로 `board_zero_domain_m`을
  다시 계산.
- 홈잉 결과를 `data/rail_calibration.json`에 저장해, 재조립 후에도 코드를 고치거나
  재빌드하지 않고 다음 실행부터 바로 반영되게 한다.
- 레일 셋업/재정렬 절차를 문서화한다(`docs/rail-calibration.md`).

**비목표**

- 볼 정렬(`ALIGNMENT_*`, `plan_ball_alignment*`) 로직 변경 — 이름은 비슷하지만 다른
  기능이다.
- 기동 시 자동 홈잉 — 사용자가 명시적으로 온디맨드만 원함(`AxlOpenNoReset`의
  엔코더 지속성을 기본 경로로 계속 신뢰한다).
- AXL 내장 `AxmHomeSetMethod`/`AxmHomeStart` 홈잉 시퀀스 도입 — 레퍼런스 구현도
  설정만 해 두고 쓰지 않으며, 이 하드웨어에 홈 센서가 별도로 있는지 확인되지 않았다.
  대신 이미 배선된 엔드스톱 알람(`AxmSignalSetLimit(..., EMERGENCY_STOP, ...)`)을 쓴다.
- `rail_frame()`의 `mount_y`/`rail_bottom_z`(레일 **마운트 위치**, 탁구대 기준 물리적
  설치 좌표) — 이건 레일 이동 범위/영점이 아니라 베이스가 레일에 얹히는 높이·오프셋이라
  홈잉으로 얻을 수 없다. 손 측정 절차만 가이드 문서에 남긴다.

## 상수 통합

새 파일 `src/defaults/rail.rs`가 다음을 전부 모은다(현재 위치 → 이유):

| 상수/함수 | 현재 위치 | 비고 |
|---|---|---|
| `RAIL_LEFT_END_MARGIN_M`, `RAIL_RIGHT_END_MARGIN_M` | `defaults/hardware.rs` | 그대로 이동 |
| `RAIL_PHYSICAL_X_MIN_M`, `RAIL_PHYSICAL_X_MAX_M` | 〃 | 그대로 이동 |
| `RAIL_COORDINATE_POSITIVE_X_OFFSET_M`, `RAIL_POSITIVE_X_TRIM_M`, `RAIL_NEGATIVE_X_ZERO_SHIFT_M`, `RAIL_BOARD_ZERO_DOMAIN_M` | 〃 | 그대로 이동. 홈잉 미실행 시 폴백 기본값이 된다 |
| `RAIL_X_MIN_M`, `RAIL_X_MAX_M`, `RAIL_READY_X_M` | 〃 | 그대로 이동 |
| `RAIL_MAX_SPEED` | `defaults/robot.rs` | 그대로 이동 |
| `RAIL_ACCEL_M_S2` | `defaults/motion.rs` | 그대로 이동 — 볼 정렬 관련 상수(`ALIGNMENT_*`)와 같은 파일에 있었을 뿐 레일 모션값이라 옮긴다 |
| `rail_frame()` (`RailFrame { mount_y, rail_bottom_z }`) | `defaults/robot.rs` | 그대로 이동. `primitive_4dof`가 이걸 계속 호출 |
| `RAIL_HOMING_VELOCITY_M_S`(신규) | — | 홈잉 전용 저속값. `min_vel`보다는 크고 `max_vel`보다 훨씬 작게(예: `0.02`) |
| `DEFAULT_RAIL_CALIBRATION_PATH`(신규) | — | `"data/rail_calibration.json"`. `defaults/calib.rs`의 `DEFAULT_CALIBRATION_PATH` 패턴을 그대로 따름 |

`WRIST_JOINT_ZERO_OFFSET_RAD`/`BASE_JOINT_ZERO_OFFSET_RAD`(관절 영점, 레일이 아님)는
`defaults/hardware.rs`에 남는다. `RAIL_THICKNESS`는 `constants/geometry.rs`에 남는다 —
CAD 실측 규격이라 `defaults`(배선 SSOT)가 아니라 `constants`(규격 SSOT)가 맞는 자리다.
대신 `defaults/rail.rs` 모듈 문서 주석에서 `RAIL_THICKNESS`로 상호 참조해, 레일 관련
값을 찾을 때 두 곳을 다 봐야 한다는 걸 코드만 보고도 알 수 있게 한다.

`defaults/mod.rs`의 재수출(`pub use`) 목록도 이동에 맞춰 갱신한다. 외부에서 보는
경로(`crate::defaults::RAIL_X_MIN_M` 등)는 바뀌지 않는다.

## 홈잉 알고리즘

### FFI 바인딩 추가 (`src/hardware/rail/axl_ffi.rs`)

두 심볼을 추가한다(레퍼런스 구현에서 확인된 실제 AXL 심볼명):

```rust
type AxmSignalReadServoAlarm = unsafe extern "system" fn(i32, *mut u32) -> u32;
type AxmSignalServoAlarmReset = unsafe extern "system" fn(i32, u32) -> u32;
```

`AxlFfi` 구조체·`load()`에 다른 심볼과 같은 패턴으로 추가.

### `AxlLive`에 저수준 동작 추가 (`axl_live.rs`)

- `read_alarm(&mut self, axis: i32) -> Result<bool, HwError>` — `AxmSignalReadServoAlarm` 래핑.
- `reset_alarm(&mut self, axis: i32) -> Result<(), HwError>` — 레퍼런스의 시퀀스
  (`LOW` → 500ms 대기 → `HIGH` → `read_alarm`이 `false`가 될 때까지 대기 → `LOW`)를
  그대로 옮긴다. 대기에는 이미 있는 `MOVE_POLL_TIMEOUT`(30s)을 재사용해 무한 루프를
  막는다.

### `AxlRail::home` (`axl_rail.rs`, Live 전용)

```rust
pub enum RailEnd { Min, Max }

#[cfg(all(windows, feature = "real"))]
pub fn home(&mut self, end: RailEnd) -> Result<f64, HwError>
```

동작:

1. `RailKind::Live`가 아니면(`DryRun`) 에러 — 시뮬/드라이런엔 물리 엔드스톱이 없다.
2. `end`에 따라 `physical_x_min_m`(여유 `RAIL_LEFT_END_MARGIN_M`이 작은 쪽, 기본
   추천 방향) 또는 `physical_x_max_m` 쪽으로 `RAIL_HOMING_VELOCITY_M_S`의 저속
   `AxmMoveStartPos`를 건다.
3. `AxmStatusReadInMotion`이 `true`인 동안 매 폴링 틱(기존 `wait_idle`과 같은 1ms 간격)에
   `read_alarm`을 확인한다.
   - 알람이 뜨면: `stop()`으로 정지 → `read_actual_and_command_m`으로 그 순간의 보드
     위치를 읽음 → `reset_alarm()` → 이 보드 위치를 물리적 기준점으로 채택.
   - `MOVE_POLL_TIMEOUT` 안에 알람이 안 뜨면: 이동을 멈추고
     `HwError::InvalidConfig`("엔드스톱 도달 못 함 — 배선/알람 설정 확인") 반환. 배선
     문제로 알람이 안 잡히는 상황에서 "홈잉했다"고 조용히 착각하는 것보다 낫다.
4. 채택한 보드 위치와 `end`가 대응하는 `physical_x_min_m`/`max_m`으로부터
   `board_zero_domain_m`을 역산한다(`RailConfig::board_to_domain_abs`의 역변환:
   `board_zero_domain_m = board_position_m + physical_x_{min,max}_m`, `reverse` 부호는
   기존 `domain_to_board_abs` 공식을 그대로 뒤집어 사용).
5. 새 `board_zero_domain_m`으로 `self.config`를 갱신하고, 호출자가 파일에 쓸 수 있게
   그 값을 반환한다.

이 루틴은 소프트 리밋을 일시적으로 무시하지 않는다 — 대상 엔드(`physical_x_min/max`)
자체가 이미 하드 리밋이므로 `command_abs_in_secs`처럼 `x_min_m..=x_max_m`으로 다시
클램프하지 않고, `physical_x_min_m`/`physical_x_max_m` 원시값으로 직접 이동을 건다(안전
마진을 넘어 실제 엔드스톱까지 가야 하므로).

### 호출 경로

- **CLI:** `src/cli/args.rs`에 `--calibrate-rail[=min|max]` 플래그 추가(기존 `--home`은
  "시작 시 ready 자세로 이동"이라는 별개 의미로 이미 쓰이고 있어 이름을 다르게 잡는다).
  값을 생략하면 `min`(여유 마진이 작아 더 신뢰할 수 있는 쪽, `RAIL_LEFT_END_MARGIN_M
  = 0.0100` vs `RAIL_RIGHT_END_MARGIN_M = 0.0705`)을 기본값으로 쓴다. 지정 시
  `main.rs`가 정상 기동 루프 대신 홈잉만 실행하고 결과를 로그·파일에 남기고 종료한다.
- **jog 툴:** `tools/jog`에 "Home Rail" 버튼 하나 추가 — 같은 `AxlRail::home` 호출,
  성공/실패를 툴 로그 패널에 표시. (`tools/jog/README.md`에 이미 있는 "sim 프리뷰 →
  실기 적용" 게이팅 패턴을 따름 — 이 동작은 되돌릴 수 없는 실기 이동이므로 dry-run
  모드에서는 버튼을 비활성화한다.)

## 영점 저장 (`data/rail_calibration.json`)

`defaults/calib.rs`의 `DEFAULT_CALIBRATION_PATH`/`calibration_path()` 패턴을 그대로 따른다.

```json
{
  "board_zero_domain_m": 0.7050,
  "homed_at_end": "min",
  "board_position_at_home_m": 0.0,
  "measured_unix_secs": 1786412345
}
```

- `RailConfig::default()`는 그대로 `defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`(하드코딩
  폴백)을 쓴다. 파일 오버라이드는 `RailConfig`를 만드는 **호출부**(`main.rs`가
  `Hardware`를 조립하는 지점)에서 적용한다: 파일이 있으면 읽어서
  `board_zero_domain_m`만 덮어쓰고, 없거나 파싱 실패하면 경고 로그 후 기본값 그대로
  진행한다. `RailConfig` 구조체 자체나 `validate()`는 바꾸지 않는다 — 이 결정은
  "어떤 값을 넣을지"이고 `RailConfig`는 이미 그 값을 받는 필드가 있다.
- `--calibrate-rail`을 실행하면 홈잉 성공 시 이 파일을 새로 쓴다(있으면 덮어씀).
- sim/dry-run 빌드는 이 파일을 읽지 않는다 — 물리 레일이 없으므로 오버라이드할 게 없다.

## 문서 (`docs/rail-calibration.md`)

다음을 포함한다:

- 언제 다시 정렬해야 하는가: 재조립 후, 레일을 다른 마운트로 옮긴 후, `AxlRail::open`이
  이미 찍는 진단 로그(`axl_rail.rs:56-70`, "AXL 시작 좌표 진단")의
  `domain_position_m`이 `RAIL_READY_X_M` 부근 기대치와 크게 벗어날 때.
- `--calibrate-rail` 실행 절차와 안전 주의사항(엔드스톱까지 실제로 이동하니 주변 정리,
  비상정지 위치 확인).
- `data/rail_calibration.json`의 내용과, 필요하면 이 파일을 지우고 하드코딩 기본값으로
  되돌리는 법.
- 홈잉으로 얻을 수 **없는** 값들(`rail_frame()`의 `mount_y`/`rail_bottom_z`)에 대한
  손 측정 절차 — 기존 각주(`robot.rs:94-110`)에 있던 실측 방법(줄자, sim GUI "Rig"
  패널로 눈으로 맞춘 뒤 `mount_search` 재실행)을 절차화해서 옮긴다.

## 에러 처리

- `home()`을 `enabled=false`나 `DryRun` 레일에 호출 → 즉시 `HwError::InvalidConfig`
  (`open()`의 기존 가드와 같은 패턴).
- 엔드스톱 도달 전 `MOVE_POLL_TIMEOUT` 초과 → 이동 정지 + 에러 반환(위 §3).
- `rail_calibration.json` 파싱 실패/스키마 불일치 → `warn!` 로그 한 줄, 하드코딴
  기본값으로 계속 진행. 캘리브레이션 파일 문제로 로봇이 아예 못 뜨면 안 된다 — 카메라
  calib 로더가 `data/calibration.json` 없을 때 취하는 태도와 같은 원칙.
- 홈잉 중 알람 리셋(`reset_alarm`)이 실패하면(레퍼런스 구현처럼 무한 대기 대신)
  `MOVE_POLL_TIMEOUT` 안에서 실패로 반환 — 알람이 안 풀린 채로 리턴하면 다음 이동
  명령이 전부 막히므로, 실패 시에도 로그에 "알람 해제 실패, 수동 확인 필요"를 명확히
  남긴다.

## 테스트

- `board_zero_domain_m` 역산 공식은 순수 함수로 분리해 단위 테스트: `reverse=true/false`
  양쪽, `RailEnd::Min`/`Max` 양쪽 조합에서 기존 `domain_to_board_abs`/`board_to_domain_abs`
  테스트(`rail_config.rs:204-238`)와 라운드트립이 맞는지 확인.
- `AxlRail::home`의 폴링/타임아웃 로직은 실물 AXL 없이 테스트하기 어렵다 —
  `RailKind::DryRun`에는 알람 개념이 없으므로, `read_alarm`을 주입 가능한 트레이트나
  클로저로 감싸 가짜 "N틱 후 알람 발생" 시퀀스를 흉내내는 테스트 더블을 추가한다
  (기존 dry-run 위치 페이크와 같은 결로, `#[cfg(all(windows, feature = "real"))]`
  경로 안에서 로직만 분리).
- `rail_calibration.json` 로드 경로: 존재/부재/파싱 실패 세 케이스를 임시 디렉터리에서
  단위 테스트.
- 실제 엔드스톱 충돌·알람 감지 자체는 실기에서만 확인 가능 — `docs/rail-calibration.md`에
  "처음 실행할 때는 저속에서 사람이 지켜보며 1회 확인" 체크리스트를 남긴다.

## 후속 작업 (이 spec 범위 밖)

- 레일 `mount_y`/`rail_bottom_z` 측정 자체를 자동화하는 것(예: 비전 기반 자동 리그 측정)
  — 현재는 사람이 sim GUI로 맞추고 `mount_search`로 확정하는 수동 절차이며, 이 spec은
  그 절차를 문서화만 한다.
- AXL 내장 홈 센서를 실제로 배선하게 되면 `AxmHomeSetMethod`/`AxmHomeStart` 기반의
  진짜 홈잉으로 전환하는 것 — 지금은 엔드스톱 알람으로 대체한다.
