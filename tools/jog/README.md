# jog

Dynamixel 관절 + AXL 리니어 레일을 **시뮬에서 미리보기**한 뒤 **Apply**로만 실기에 보내는 GUI.

직접 조그(`j`/`angles`/`r`/`ik`/`pose`)는 planner 없이 FK/IK + quintic으로 보낸다.
**스윙만은 시뮬과 똑같이 동작한다** — 슈터 파라미터만 주면 타점·라켓 각도·임팩트
속도는 `plan_best_swing`이 고른다.

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
| `swing` | 슈터가 쏜 공을 시뮬과 같은 planner로 받아치기 |
| `duration` / `maxdelta` | 기본 이동 시간 · 관절 최대 Δ |

### 스윙 (슈터 공)

입력은 **슈터 창의 파라미터뿐**이다. 접수 평면도, 라켓 각도도 사람이 고르지 않는다 —
시뮬과 완전히 같은 경로를 탄다:

1. 슈터 발사 상태를 `Kinematics::step`으로 **시뮬이 스윙을 커밋하는 시점**
   (`ball_past_midcourt_for_commit`)까지 굴린다
2. 그 상태에서 `InterceptWindow`의 **모든** hit plane에 `predict_to`
3. `plan_best_swing`이 후보를 전부 채점해 최적 타점·라켓 자세·임팩트 속도를 고르고
   quintic 궤적까지 만든다

도달점·입사속도를 직접 넣지 않으므로 물리적으로 불가능한 조합이 들어올 수 없다.
계획이 실패하면 사유가 표시되고 미리보기가 잠긴다.

미리보기는 **두 단계**로 재생된다 — 시뮬이 공이 날아오는 동안 레일·관절을 미리
옮겨두는 것(`plan_coarse_track`)과 같다:

1. **코스 추종 이동** — 타점 쪽 대기 포즈로 (`이동 시간` 슬라이더만큼)
2. **스윙** — 커밋 창 안에서

이게 없으면 레일이 대기 끝단(x=0)에 있을 때 커밋 시간창 안에 도달할 수 없어
모든 해가 실패한다. Apply도 두 궤적을 순서대로 실기에 보낸다.

planner는 **입력이 바뀔 때만** 돈다 — 매 프레임 돌리면 후보마다 뱉는 경고로
터미널이 잠긴다.

슈터 창에는 **Random만** 있다. jog는 공을 실제로 쏘지 않는다 — 스윙만 본다.

## 동작 요약

1. Sync: 실기 `read_pose` → `RobotHandle::set_pose`
2. Preview: 목표 결정 (직접 / IK / pose) 또는 `plan_best_swing` → `SwingTrajectory` → sim `play`
3. Apply: 같은 궤적 → `RealHardware::command` → busy 대기 → AwaitingSync
4. Sync 후 다시 Ready

## 안전

- `maxdelta` 초과 관절 점프는 거부
- IK/pose 실패는 에러만 표시, 하드웨어는 그대로
- Sync 없이 두 번째 Preview / Apply 불가
- 실기는 작은 `j` / `rd`부터, Preview로 확인 후 Apply
