# Design: `--mode real` 수동 테스트 — 대기 3초 자동 idle · 스윙 후 제자리 유지

**작성일:** 2026-08-06
**상태:** approved (user 2026-08-06)
**범위:** `src/real/control_worker.rs`만 건드린다. `test_control.rs`의
`TestControl`/`TestZone` 타입, `run.rs`의 키 라우팅, `preview.rs`는 그대로 둔다
(동작 문구만 일부 주석에서 갱신).

---

## 배경

`2026-08-06-real-mode-test-wait-gate-design.md`가 매 스윙(정렬→고정 스윙→중립
복귀) 뒤 `n`을 눌러야 다음 공을 받는 `Waiting` 게이트를 도입했다. 실기 운용
중 두 가지를 더 손보기로 한다.

1. **`n`을 계속 누르고 있어야 하는 게 번거롭다.** 스윙 후 자동으로 들어간
   `Waiting`은 3초가 지나면 사람이 키를 누르지 않아도 자동으로 `Idle`(공을
   받는 상태)로 돌아가야 한다. 단, `w` 키로 운영자가 **수동으로** 들어간
   `Waiting`은 그대로 둔다 — 그건 의도적으로 멈춰 둔 것이므로 계속 `n`을
   요구한다.
2. **스윙 후 레일이 홈 포지션으로 돌아가 버린다.** 운영자가 맞은 위치를 눈으로
   확인하고 싶은데, 지금은 스윙이 끝나자마자 레일이 `home_rail_x`로
   복귀한다. 레일(리니어 모터)은 공을 친 자리에 그대로 두고, 관절
   (Dynamixel)만 `r`(ResetPosition)을 눌렀을 때와 같은 준비 자세로 되돌아가야
   한다.

## 상태 다이어그램 (변경 부분만)

```
   ┌─────────────┐  swing 완료, 관절만 준비 자세로 복귀   ┌─────────────┐
   │  ALIGNING   │ ─────────────────────────────────────► │   WAITING   │
   └─────────────┘  (레일은 스윙 시점 위치 그대로 유지)     │ auto-resume │
                                                            │ at = now+3s │
                                                            └─────────────┘
                                                                   │
                       3초 경과 (자동) ─────────────────────────────┤
                       또는 n (건너뛰기) ─────────────────────────────┤
                       — 어느 쪽이든 하드웨어를 움직이지 않는다        │
                                                                   ▼
                                                              ┌─────────┐
                                                              │  IDLE   │
                                                              └─────────┘

  w(Wait)로 수동 진입한 WAITING은 auto-resume 타이머가 없다 — 여전히 n 필요.
  r/w/zone 키는 지금처럼 home_rail_x로 완전 복귀한다(변경 없음).
```

## 컴포넌트

### 새 상수

```rust
const AUTO_IDLE_AFTER_WAIT: Duration = Duration::from_secs(3);
```

### `spawn()` 루프 로컬 상태

`BallControlState`는 건드리지 않는다(필드를 추가하면 기존 매치 지점 십여
곳을 전부 고쳐야 한다). 대신 루프 지역 변수 하나를 추가한다:

```rust
let mut waiting_auto_resume_at: Option<Instant> = None;
```

- `Aligning`의 `due_for_return`이 발화해 `Waiting`으로 들어갈 때만
  `Some(Instant::now() + AUTO_IDLE_AFTER_WAIT)`로 설정한다.
- `w`(`TestControl::Wait`) 처리 시작 지점에서 무조건 `None`으로 지운다 —
  스윙 후 자동 대기 중 운영자가 `w`를 다시 눌러 "의도적으로 멈춤"으로
  바꾸는 경우, 남아 있던 타이머가 몰래 재개시키는 걸 막는다.
- `TestControl::Next` 처리에서 `Some`이면 `take()`하고(소비 즉시 `None`),
  `None`이면 기존 경로(`apply_immediate_control`, 홈 복귀 포함)를 그대로
  탄다.
- 주기 점검(매 루프 tick, `matches!(state, BallControlState::Waiting)`
  가드 하에): 마감이 지났으면 소비하고 자동 전이한다.

### 스윙 후 복귀 — 레일 유지, 관절만 복귀

`idle_ready && due_for_return` 분기(현재 `move_to_ready(hardware.as_mut(),
&arm, home_rail_x)` 호출부)를 바꾼다:

- 이미 그 분기에서 실측용으로 `hardware.read_pose()`를 한 번 호출한다
  (`"공 위치·방향 정렬 완료 후 실측"` 로그). 이 실측 `rail_x`를 바깥
  변수로 꺼내 재사용한다.
- `move_to_ready(hardware.as_mut(), &arm, home_rail_x)` →
  `move_to_ready(hardware.as_mut(), &arm, measured_rail_x)`로 바꾼다.
  `move_to_ready`는 항상 관절을 `arm.default_joints`로 되돌리고 레일만
  넘겨받은 x로 옮기므로, 이미 있는 위치를 그대로 넘기면 레일은 움직이지
  않고 관절만 준비 자세로 복귀한다 — 별도 분기 없이 기존 함수 재사용.
- 성공 후 `state = BallControlState::Waiting;`은 그대로 두고, 그 직후
  `waiting_auto_resume_at = Some(Instant::now() + AUTO_IDLE_AFTER_WAIT);`를
  추가한다.
- 이 분기의 하드웨어 오류 처리(`move_to_ready` 실패)는 그대로 — 실패하면
  `Idle`로 정리하고 `continue`하므로 `waiting_auto_resume_at`은 애초에
  설정되지 않는다.

### 조용한 idle 전이 헬퍼

`TestControl::Next`(타이머 소비 경로)와 주기 점검이 같은 동작을 하므로
작은 헬퍼로 뽑는다:

```rust
/// 스윙 후 자동 대기가 만료됐거나 `n`으로 건너뛴 경우 — 레일은 친 위치
/// 그대로 두고 idle로 전환한다(하드웨어를 움직이지 않는다).
fn resume_waiting_in_place(
    hardware: &mut dyn Hardware,
    state: &mut BallControlState,
    cached_idle_pose: &mut Option<pingpong_bot::robot::Pose>,
    event_tx: &Sender<RuntimeEvent>,
) {
    *state = BallControlState::Idle;
    *cached_idle_pose = hardware.read_pose().ok();
    let _ = event_tx.send(RuntimeEvent::ControlState {
        state: ControlStateSnapshot::Idle,
    });
}
```

`TestControl::Next` 분기:

```rust
if matches!(state, BallControlState::Waiting) {
    pending_verification = None;
    pending_refined = None;
    consecutive_misses = 0;
    if waiting_auto_resume_at.take().is_some() {
        resume_waiting_in_place(hardware.as_mut(), &mut state, &mut cached_idle_pose, &event_tx);
    } else if apply_immediate_control(/* 기존 인자 그대로 */).is_break() {
        break 'control;
    }
} else {
    debug!("대기 상태가 아닐 때 'n' 입력 — 무시");
}
```

주기 점검(루프 tick마다, `idle_ready` 계산 이전 아무 곳):

```rust
if matches!(state, BallControlState::Waiting)
    && waiting_auto_resume_at.is_some_and(|deadline| Instant::now() >= deadline)
{
    waiting_auto_resume_at = None;
    resume_waiting_in_place(hardware.as_mut(), &mut state, &mut cached_idle_pose, &event_tx);
    info!("대기 3초 경과 — 자동으로 idle 전환 (n 불필요)");
}
```

루프는 `rx.recv_timeout`이 기본 `RECV_TIMEOUT`(100ms)마다 깨어나므로 별도
타임아웃 계산 조정 없이 3초 마감을 최대 100ms 오차로 감지한다.

## 건드리지 않는 것

- `r`(ResetPosition), 수동 `w`(Wait), 존 변경(`1`/`2`/`3`/`4`)은 지금처럼
  `home_rail_x`로 완전 복귀한다. "제자리 유지"는 스윙 직후 자동 복귀
  단계에만 적용된다.
- `apply_test_control`/`apply_immediate_control` 함수 시그니처와 내부 로직은
  변경하지 않는다 — 새 "제자리 유지" 경로는 이 함수들을 아예 호출하지
  않는 별도 분기다.
- `BallControlState`, `ControlStateSnapshot`, `TestControl` 타입 자체는
  변경 없음 — 외부에서 관찰되는 이벤트 시퀀스(`Waiting` → `Idle`)는 기존과
  동일하다.

## 테스트

- 기존 통합 테스트 `spawn_ignores_balls_while_waiting_and_resumes_after_next`는
  수정 없이 계속 통과해야 한다(3초를 기다리지 않고 `n`으로 즉시 전이하는
  경로를 이미 그대로 검증한다).
- 신규: `spawn()` 통합 테스트로 스윙 후 자동 대기가 **3초 뒤 `n` 없이도**
  `ControlState::Idle`로 전이하는지 확인한다(타이머 상수를 테스트에서
  줄일 수 없으므로 3초 초과 타임아웃으로 대기).
- 신규: `move_to_ready` 목표 rail_x 관련 — 스윙 후 복귀 시 레일 좌표가
  스윙 직전 값과 같고(변화 없음), 관절만 `arm.default_joints`와 일치하는지
  확인하는 좁은 단위 테스트(순수 함수 `plan_neutral_return_segments`
  또는 `move_to_ready` 호출부 근처에서 mock hardware로 검증).
