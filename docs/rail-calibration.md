# 리니어 레일 정렬·캘리브레이션 가이드

리니어 레일(AXL 보드) 좌표계는 두 종류의 값으로 나뉜다:

- **자동으로 다시 잡을 수 있는 값** — 레일 영점(`board_zero_domain_m`). `calib-rail`
  툴로 물리적 엔드스톱까지 저속 이동해 재측정한다. 이 문서가 다루는 대상.
- **손으로만 측정할 수 있는 값** — 레일 **마운트 위치**(`rail_frame()`의 `mount_y`,
  `rail_bottom_z`). 레일 이동 범위/영점이 아니라 베이스가 레일에 얹히는 물리적 설치
  좌표라 홈잉으로는 얻을 수 없다. §손 측정이 필요한 값 참고.

관련 상수는 모두 `src/defaults/rail.rs` 한 곳에 모여 있다(`RAIL_LEFT_END_MARGIN_M`,
`RAIL_PHYSICAL_X_MIN_M`/`MAX_M`, `RAIL_BOARD_ZERO_DOMAIN_M`, `RAIL_X_MIN_M`/`MAX_M`,
`RAIL_READY_X_M`, `RAIL_MAX_SPEED`, `RAIL_ACCEL_M_S2`, `RAIL_HOMING_VELOCITY_M_S`,
`rail_frame()`). 프로파일 단면 두께처럼 CAD 실측 규격은 예외로
`src/constants/geometry.rs`의 `RAIL_THICKNESS`에 남아 있다 — 배선/튜닝값이 아니라
이미 제작된 프로파일의 고정 치수라서다.

## 언제 다시 정렬해야 하는가

- 레일을 분해·재조립한 뒤.
- 레일을 다른 마운트/위치로 옮긴 뒤.
- `AxlRail::open`이 실기 시작 시 찍는 "AXL 시작 좌표 진단" 로그의 `domain_position_m`이
  `RAIL_READY_X_M`(0.675m) 부근 기대치에서 크게 벗어날 때.

## `calib-rail`로 영점 다시 잡기

```bash
# 기본: min(좌측) 엔드스톱까지 홈잉
cargo run -p calib-rail

# max(우측) 엔드스톱
cargo run -p calib-rail -- --end max

# DLL 경로가 기본값과 다르면
cargo run -p calib-rail -- --dll-path "C:/path/to/AXL.dll"
```

동작:

1. 레일이 지정한 엔드스톱(`min`/`max`) 쪽으로 `RAIL_HOMING_VELOCITY_M_S`(0.02 m/s)
   저속 이동을 시작한다.
2. AXL 서보 알람(`AxmSignalReadServoAlarm`)이 뜨면 도달로 간주해 즉시 정지한다.
3. 그 지점의 원시 보드 좌표로부터 `board_zero_domain_m`을 다시 계산하고, 알람을
   해제한다.
4. **레일을 안전 이동 범위(`x_min_m..x_max_m`) 안의 준비 위치(`RAIL_READY_X_M`)로
   복귀시킨다.** 홈잉 직후 레일은 엔드스톱 근처, 즉 안전 범위 밖에 있다 — 복귀시키지
   않으면 다음 `--mode real` 기동의 ready-pose 이동 계획이 "현재 위치가 이미 범위
   밖"이라는 이유로 가속도 한계를 넘어 실패한다.
5. 결과를 `data/rail_calibration.json`에 저장한다.

`calib-rail`은 `AxlRail::open`만 사용한다 — Dynamixel 팔은 열지 않으므로, 팔이 정렬
안 됐거나 연결 안 돼 있어도 레일 홈잉과는 무관하다.

**안전 주의사항:**

- 저속이지만 실제로 물리적 엔드스톱까지 이동한다. 실행 전 레일 이동 경로에 장애물이
  없는지, 비상정지 스위치 위치를 확인할 것.
- 처음 실행할 때는 사람이 옆에서 지켜보며 1회 확인할 것 — 알람이 예상대로 잡히는지,
  배선이 맞는지 실기로만 검증할 수 있다.
- `--dry-run` 개념이 없다 — 물리 레일이 연결된 실기에서만 의미가 있고, 시뮬레이션에는
  적용할 엔드스톱이 없어 `home()` 호출 자체가 항상 에러를 반환한다.
- 정상적인 `--mode real` 기동은 이 홈잉을 자동으로 실행하지 않는다. 저장된
  `data/rail_calibration.json`(없으면 `RAIL_BOARD_ZERO_DOMAIN_M` 하드코딩 기본값)을
  그대로 신뢰하고 시작한다.

같은 동작을 `jog` 툴의 "레일 홈잉" 버튼으로도 실행할 수 있다(dry-run에서는 비활성화).
다만 `jog`는 캘리브레이션 파일을 저장하지 않는다 — 결과를 영속화하려면 `calib-rail`을
쓴다.

## `data/rail_calibration.json`

```json
{
  "board_zero_domain_m": 0.7050,
  "homed_at_end": "min",
  "board_position_at_home_m": -0.6923,
  "measured_unix_secs": 1786412345
}
```

- `board_zero_domain_m` — 실제로 적용되는 값. 다음 실행부터 `RailConfig::board_zero_domain_m`
  기본값을 덮어쓴다.
- `homed_at_end`, `board_position_at_home_m` — 어느 엔드스톱에서 어떤 원시 보드
  좌표를 읽었는지 남기는 진단용 기록. 동작에는 영향을 주지 않는다.
- `measured_unix_secs` — 측정 시각.

파일을 지우면 다음 실행부터 하드코딩 기본값(`defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`)
으로 되돌아간다. 파일이 있지만 파싱에 실패하면 경고 로그 한 줄을 남기고 마찬가지로
기본값으로 계속 진행한다 — 캘리브레이션 파일 문제로 로봇이 못 뜨면 안 된다는 원칙이다.

## 손 측정이 필요한 값

`rail_frame()`(`src/defaults/rail.rs`)의 `mount_y`(탁구대 로봇쪽 끝면 기준 레일 y
오프셋)와 `rail_bottom_z`(바닥→레일 프로파일 하단 높이)는 홈잉으로 얻을 수 없다.
재측정 절차:

1. 줄자로 바닥→프로파일 하단 높이, 탁구대 끝면→레일 y 오프셋을 잰다.
2. sim GUI의 "Rig" 패널에서 공이 주차된 동안 두 값을 런타임으로 조정하며 눈으로
   맞춘다(`SimRuntimeControls::rail_frame`).
3. 좋은 위치를 찾으면 `mount_search`(또는 `--rest-pose-search`)를 그 위치에서 다시
   돌려 `rail_frame()`과 `READY_JOINTS_4DOF`(`src/defaults/robot.rs`)를 함께
   확정한다.
4. `src/defaults/rail.rs`의 `rail_frame()`에 새 값과 측정 날짜를 doc 주석으로 남기고
   손으로 갱신한다.
