# AXL InMotion 시 스윙용 AxmMoveStartPos 무시

## 문제

스윙 시작 시 `RealHardware::command`가 `command_abs_in_secs` → `AxmMoveStartPos`를 호출한다. 축이 이미 `InMotion`이면 `AxmMoveStartPos`는 실행할 수 없다. 현재는 이 실패를 `Err`로 올려 Dynamixel 스윙까지 중단한다.

## 목표

축이 `InMotion`일 때 스윙용 레일 명령만 무시하고 경고를 남긴다. Dynamixel 스윙은 계속 실행한다.

## 비목표

- 센터 복귀 등 `AxmMovePos` / `move_abs_m` 경로 변경 없음
- InMotion이 아닌 AXL 실패 처리 변경 없음 (기존처럼 `Err` → 스윙 중단)

## 동작

1. Live `start_move_abs_m`에서 `AxmStatusReadInMotion`을 먼저 읽는다.
2. `in_motion != 0`이면 `AxmMoveStartPos`를 호출하지 않고 `warn` 후 `Ok(())`를 반환한다.
3. Idle일 때만 기존처럼 `AxmMotSetAbsRelMode` + `AxmMoveStartPos`를 호출한다.
4. 그 외 AXL API 실패는 기존 `check_axl` → `Err` 경로를 유지한다.

호출 체인: `RealHardware::command` → `AxlRail::command_abs_in_secs` → `AxlLive::start_move_abs_m`.

InMotion soft-skip은 `Ok`이므로 `real.rs`의 “레일 실패 → 스윙 중단” 분기는 타지 않는다. 해당 분기 로직 문구는 변경하지 않는다.

## 테스트

- dry-run 경로는 InMotion 개념이 없으므로 기존 dry-run 테스트는 그대로 둔다.
- Live InMotion 분기는 하드웨어 없이 unit으로 강제하기 어렵다. `start_move_abs_m` 주석으로 동작을 명시하고, 가능하면 InMotion 읽기 헬퍼의 반환 로직만 검증한다.
