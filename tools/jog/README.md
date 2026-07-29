# jog

Dynamixel 관절 + AXL 리니어 레일을 **시뮬에서 미리보기**한 뒤 **Apply**로만 실기에 보내는 GUI.

공 추적·`plan_swing` 같은 planner는 **쓰지 않는다**. 목표만 정하면 FK/IK와 quintic 궤적으로 팔·레일을 같은 시간에 보낸다.

설정 SSOT: [`src/defaults/`](../../src/defaults/) — `dynamixel()` · `rail()` · `robot()` · `control()`.

공유 씬: [`SimScene`](../../src/sim/gui/) + [`RobotHandle`](../../src/sim/gui/layers/robot.rs) (`set_pose` / `play` / `cancel`).

## 실행

```bash
# 통신 없이 IK·궤적·executor + sim 미리보기
cargo run -p jog -- --dry-run

# 실기 (Windows 벤치)
cargo run -p jog -- --port COM8
cargo run -p jog -- --port COM8 --dll-path "C:/path/to/AXL.dll"

# 디버그: Dynamixel 재시도·AXL API code·초기 배선 덤프
cargo run -p jog -- --port COM8 --debug
```

창이 열리면 **Jog** 패널에서 Sync / Preview / Apply / Discard를 사용한다. 창을 닫으면 종료.

## Sync / Apply 게이트

| 상태 | Preview | Apply | Discard | Sync |
|------|---------|-------|---------|------|
| NeedsSync / AwaitingSync | 잠금 | 잠금 | 잠금 | 가능 |
| Ready | 1회 | 잠금 | — | 가능 |
| Previewed | 잠금 | 가능 | sync 포즈 복귀 | staged 무효화 후 Ready |

- **boot**: `read_pose` → sim Sync → Ready
- **Preview**: 툴에서 궤적 조합 → `RobotHandle::play` (sim만). Sync당 **1회**
- **Discard**: sync 포즈로 복귀 → Ready
- **Apply**: 스테이징된 **동일** `SwingTrajectory` → `RealHardware::command` (dry-run이면 dry-run HW)
- **Apply 후**: AwaitingSync — 실기 종료 자동 감지 없음 → **수동 Sync** 필수

## 모션 (패널)

기존 REPL과 동일 의미:

| 종류 | 설명 |
|------|------|
| `j` | 관절 `i`만 목표각 [deg] |
| `angles` | 전축 목표 [deg] |
| `r` / `rd` | 레일 절대 / 상대 [m] |
| `ik` | 위치 IK (레일 x 유지) |
| `pose` | 위치+법선 IK |
| `swing` | 임팩트 속도 [m/s] 스윙 |
| `duration` / `maxdelta` | 기본 이동 시간 · 관절 최대 Δ |

### 스윙 세기

`speed`는 토크가 아니라 **임팩트 순간 라켓 선속도 [m/s]** 다.  
법선 방향으로 `velocities_for_racket_velocity` → 관절·레일 속도 → quintic + 팔로스루.

## 동작 요약

1. Sync: 실기 `read_pose` → `RobotHandle::set_pose`
2. Preview: 목표 결정 (직접 / IK / pose / swing) → `SwingTrajectory` → sim `play`
3. Apply: 같은 궤적 → `RealHardware::command` → busy 대기 → AwaitingSync
4. Sync 후 다시 Ready

## 안전

- `maxdelta` 초과 관절 점프는 거부
- IK/pose 실패는 에러만 표시, 하드웨어는 그대로
- Sync 없이 두 번째 Preview / Apply 불가
- 실기는 작은 `j` / `rd`부터, Preview로 확인 후 Apply
