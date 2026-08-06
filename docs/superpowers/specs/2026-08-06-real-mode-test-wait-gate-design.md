# Design: `--mode real` 수동 테스트 — 스윙 후 `n` 대기 게이트

**작성일:** 2026-08-06
**상태:** approved (user 2026-08-06)
**범위:** `src/real/` (`test_control.rs`, `control_worker.rs`, `preview.rs`, `runtime_event.rs`)만 건드린다.
`run.rs`의 키 라우팅(`TestControl::from_key` 호출)과 `CommitRequest`/추정·카메라 워커는 그대로 둔다.

---

## 배경

`2026-08-05-real-mode-manual-test-controls-design.md`가 슈터 무랠리 테스트용
`1`/`2`/`3`(존 선택) · `w`(wait) · `r`(reset) 수동 컨트롤을 도입했다. 실기로
써보니 두 가지가 어긋난다:

1. **`w`가 안 눌리는 것처럼 보인다.** 지금 `Wait`는 "다음 idle 시점에" 존
   변경 없이 latch·상태만 정리하고 현재 `home_rail_x`로 복귀하는 동작이다.
   시스템이 이미 idle(공을 기다리는 중)일 때 누르면 겉보기에 아무 변화가
   없다 — 실제로는 latch를 리셋했을 뿐, 화면에 드러나는 상태 전이가
   없었다.
2. **스윙(정렬→0.5초 유지→중립 복귀) 후 다음 공을 곧바로 받는다.** 운영자가
   원하는 사이클은: 슈터가 쏘고 → 로봇이 정렬-유지-복귀(swing)하고 →
   운영자가 결과를 눈으로 확인할 시간을 갖고 → `n`을 눌러야 다음 공을
   받는다. 지금은 복귀가 끝나는 즉시 다시 공을 받으므로, 운영자가 결과를
   보기도 전에 슈터가 다음 공을 쏘면 놓치거나 뒤섞인다.

이 설계는 매 스윙 뒤에 **항상 켜지는** 대기 게이트를 추가한다 — 이 파일이
다루는 범위 전체가 이미 이 수동 테스트 프로토콜 전용이므로 별도 CLI
플래그는 두지 않는다. `w`는 "언제든 수동으로 대기 상태에 들어간다"로
재정의해 눌렀을 때 항상 눈에 보이는 효과를 갖게 한다.

## 상태 다이어그램

```
                     ball detected & aligned
        ┌─────────────────────────────────────────┐
        │                                          ▼
   ┌─────────┐                              ┌─────────────┐
   │  IDLE   │                              │  ALIGNING   │
   │(catching)│                             │ (hold 0.5s) │
   └─────────┘                              └─────────────┘
        ▲                                          │
        │ n (Next)                                 │ hold elapsed →
        │                                          │ auto return-to-home
   ┌─────────────┐                                 │
   │   WAITING   │◄────────────────────────────────┘
   │ (press 'n') │
   └─────────────┘

  ANY 상태에서 즉시 적용(진행 중인 동작을 cancel하고 복귀 후 전이):
    r (ResetPosition) ──► IDLE      (복귀, 계속 공 받기)
    w (Wait)           ──► WAITING  (복귀, n 눌러야 재개)

  1 / 2 / 3 / 4 (SetZone / DefaultMode) — 다음 유휴 시점에 적용,
  home_rail_x·zone_filter만 바꾸고 IDLE/WAITING 여부는 유지:
    IDLE    + zone 키 → IDLE    (새 존, 계속 공 받기)
    WAITING + zone 키 → WAITING (새 존, 여전히 n 필요)
```

`Failed`(정렬 계획/하드웨어 오류)는 이 다이어그램에 없다 — 스윙이 실제로
끝난 게 아니므로 `WAITING`이 아니라 기존과 동일하게 `IDLE`로 정리된다(아래
"에러 처리" 참고).

## 컴포넌트

### `BallControlState` — `control_worker.rs`

```rust
enum BallControlState {
    Idle,
    Aligning { return_due_at: Instant, measurement: PendingAlignmentMeasurement },
    Waiting,
}
```

필드 없는 단순 variant. `active_track_seq()`는 `Waiting`에서도 `None`을
반환(Idle과 동일하게 "지금 이 track에 묶여 있지 않음"을 뜻한다).

### `TestControl` — `test_control.rs`

```rust
pub enum TestControl {
    ResetPosition,      // 즉시 적용 → Idle
    Wait,                // 즉시 적용 → Waiting (신규 의미)
    SetZone(TestZone),   // 다음 유휴 시점 → 이전 Idle/Waiting 유지
    DefaultMode,          // 다음 유휴 시점 → 이전 Idle/Waiting 유지
    Next,                 // 신규. 즉시 적용. Waiting에서만 의미 있음 → Idle
}
```

`from_key`에 `n`/`N` → `Some(Self::Next)` 매핑을 추가한다. 나머지 매핑은
그대로.

### `control_worker.rs` — 메인 루프 배선

- **최상단 `test_control_rx` 드레인 루프**: `ResetPosition`은 지금처럼 즉시
  cancel-and-apply. **`Wait`도 같은 즉시 경로로 옮긴다** — 지금은
  `pending_test_control`에 저장돼 다음 유휴 시점에야 적용됐는데, 이제
  cancel-and-apply를 거쳐 곧바로 `Waiting`으로 전이한다. **`Next`**는
  `matches!(state, BallControlState::Waiting)`일 때만 cancel 없이(어차피
  하드웨어는 이미 정지·유휴 상태) `apply_test_control`을 호출해 `Idle`로
  전이하고, 그 외에는 아무 것도 하지 않는다(no-op). `SetZone`/`DefaultMode`는
  지금처럼 `pending_test_control`에 저장해 다음 유휴 시점에 적용한다.
- `ResetPosition`과 `Wait`가 공유하는 "busy면 cancel하고 멈출 때까지
  대기" 루프를 작은 헬퍼로 뽑아 중복을 없앤다(동작 변경 없음).
- **`apply_test_control`**: 지금은 끝에서 항상 `*state = BallControlState::Idle`로
  고정한다. 이를 컨트롤 종류 + 호출 시점의 `*state`로 결정하는 로직으로
  바꾼다:
  - `ResetPosition`, `Next` → `Idle`
  - `Wait` → `Waiting`
  - `SetZone(_)`, `DefaultMode` → 호출 시점 상태가 `Waiting`이면 `Waiting`
    유지, 아니면(`Idle`/`Aligning`) `Idle`
  - 결정된 상태에 맞는 `RuntimeEvent::ControlState` 스냅숏(`Idle` 또는
    `Waiting`)을 보낸다.
- **공 무시 로직**: `CommitRequest`를 받아 `track_seq`를 뽑은 직후, 어떤
  latch/정렬 로직에도 닿기 전에 `matches!(state, BallControlState::Waiting)`이면
  이 요청을 건너뛴다. 기존 `zone_filter` 불일치 처리와 같은 스타일로,
  같은 track을 반복 로그하지 않도록 `last_waiting_ignored_track_seq: Option<u64>`
  하나로 중복 로그를 막는다(연속된 같은 track에는 1회만 `info!`).
- `due_for_return` 매치에 `BallControlState::Waiting => false` 추가(exhaustive
  matching으로 컴파일러가 강제).

### `ControlStateSnapshot` — `runtime_event.rs`

```rust
pub enum ControlStateSnapshot {
    Idle,
    Aligning { .. },   // 필드 변경 없음
    Waiting,
}
```

### `PreviewWindow` — `preview.rs`

- 상태 패널을 2박스(IDLE/ALIGN)에서 3박스(IDLE/ALIGN/WAIT)로 확장한다.
  패널 폭(`STATE_PANEL_W`)을 세 박스가 들어가도록 넓힌다. 정확히 하나의
  박스만 `STATE_ACTIVE_COLOR`로 강조되고 나머지 둘은 `STATE_IDLE_COLOR` —
  지금의 "둘 중 하나만 강조" 로직을 세 개로 일반화한다.
- 도움말 범례 줄(현재 `"1:0-45 2:20-60 3:55-100 4:all"`)에 `n:next`,
  `w:pause`, `r:reset`을 추가해 키가 눈에 보이게 한다 — `w`가 "안 눌리는
  것처럼 보인다"는 원래 문제의 상당 부분은 눌렀을 때 뭘 해야 하는지 화면에
  전혀 안 나왔던 것도 원인이다.

## 에러 처리

- **`Failed`는 `Waiting`으로 가지 않는다.** 정렬 계획 실패·하드웨어 오류로
  `Aligning`이 중단되면 지금처럼 `Idle`로 정리하고 다음 공을 계속
  기다린다. 스윙이 실제로 끝난 게 아니므로 운영자가 검토할 결과가 없고,
  거기서까지 `n`을 요구하면 실패와 무관한 마찰만 늘어난다.
- **`Next`를 `Waiting`이 아닐 때 누르면 아무 일도 없다.** `Idle`이나
  `Aligning` 중에 눌러도 무시(no-op) — 별도 에러나 로그 스팸을 만들지
  않는다.
- **`Waiting` 중 도착하는 공은 조용히(중복 로그 방지) 버려진다.** latch,
  `pending_refined`, 하드웨어 명령 어느 것도 건드리지 않는다 — 슈터가
  일찍 쏴도 다음 `n` 전까지 아무 정렬도 일어나지 않는다.
- **`Aligning`의 0.5초 유지 중에 zone 키가 적용되면 결과 상태는 `Idle`이다.**
  auto-return(그리고 그에 따른 `Waiting` 전이)이 아직 일어나기 전에
  운영자가 존을 바꿔 개입한 경우, 그 개입이 이번 공 사이클을 명시적으로
  끝낸 것으로 보고 `n`을 추가로 요구하지 않는다 — "호출 시점 상태가
  `Waiting`이면 유지, 아니면 `Idle`" 규칙 그대로다.
- `w`/`r`이 진행 중인 정렬을 끊을 때의 하드웨어 정지 경로(`cancel()` +
  busy 해제 대기)는 기존 `ResetPosition` 경로와 동일 — 새 실패 모드
  없음.

## 테스트

기존 스타일을 따른다 — `spawn()`의 while 루프 전체를 통합 테스트하지
않고, 뽑아낸 순수 함수 단위로 검증한다. 다만 이번엔 "공이 대기 중 무시된다"가
루프 배선 자체의 동작이므로, 그 부분만 실 스레드로 도는 좁은 통합 테스트
하나를 추가한다.

- `TestControl::from_key`: `n`/`N` → `Some(Next)` (기존 `w`/`r` 테스트와
  같은 스타일로 대소문자 둘 다).
- `apply_test_control`:
  - `Wait` 적용 → `BallControlState::Waiting`, `ControlStateSnapshot::Waiting`
    이벤트 발행 확인(기존 `apply_test_control_wait_keeps_current_zone`
    테스트를 이 기대값으로 갱신).
  - `Next`를 `Waiting` 상태에서 적용 → `BallControlState::Idle`로 전이.
  - `SetZone`을 `Waiting` 상태에서 적용 → 존/`home_rail_x`는 바뀌지만
    상태는 `Waiting` 유지(신규).
  - `SetZone`을 `Idle`/`Aligning` 상태에서 적용 → 기존 테스트대로 `Idle`
    (회귀 없음 확인).
- 신규 통합 테스트(`control_worker::spawn` 실 스레드): 최소 `Hardware`
  더미(`command`/`read_pose`만 구현, 나머지는 트레이트 기본값)로 워커를
  띄우고 —
  1. 유효한 `CommitRequest`를 보내 `Commanded` 이벤트를 확인한다.
  2. `POST_ALIGNMENT_HOLD_SECS`(0.5s) 경과 후 `ControlState::Waiting` 이벤트를
     확인한다.
  3. 같은 궤적으로 두 번째 `CommitRequest`를 보내고, 일정 시간 동안
     `Commanded` 이벤트가 **오지 않음**을 확인한다(무시됨).
  4. `TestControl::Next`를 보내고 `ControlState::Idle` 이벤트를 확인한다.
  5. 세 번째 `CommitRequest`를 보내 다시 `Commanded` 이벤트가 옴을
     확인한다(재개).
