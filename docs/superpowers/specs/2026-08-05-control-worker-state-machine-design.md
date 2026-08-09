# Design: `control_worker` 명시적 상태 기계

**작성일:** 2026-08-05
**상태:** approved (user 2026-08-05)
**범위:** `src/real/control_worker.rs`만. `estimator_worker.rs`·`run.rs`는 건드리지 않는다.

---

## 목적

`control_worker`의 while 루프는 공 하나의 처리 상태(대기 중인지, 쳤는지, 복귀
대기 중인지)를 여섯 개의 독립된 지역 변수로 표현한다.

```rust
let mut latch = CommandLatch::default();
let mut last_command: Option<Instant> = None;
let mut pending_verification: Option<PendingVerification> = None;
let mut return_due_at: Option<Instant> = None;
let mut struck_track_seq: Option<u64> = None;
let mut pending_impact_measurement: Option<(u64, f64, Joints)> = None;
let mut consecutive_misses: u8 = 0;
```

이 중 `struck_track_seq`, `return_due_at`, `pending_impact_measurement` 세
개는 코드 전체에서 **항상 함께 설정되고 함께 해제된다** — 그런데 그 불변식은
타입에 없고 관례로만 유지된다. 목표는 이 불변식을 타입으로 만들어, 셋 중
하나만 설정하고 나머지를 빠뜨리는 게 구조적으로 불가능하게 만드는 것이다.
동작은 바꾸지 않는다 — 표현만 바꾼다.

## 조사 중 발견한 것 (참고, 이번 범위 아님)

이 설계를 준비하며 실제 루프를 추적하다가 두 가지를 확인했다. 둘 다 사용자
결정으로 **이번 패스에서 고치지 않고 기록만 한다.**

1. **`PendingVerification` 경로가 실제로는 죽어 있다.** `pending_verification`은
   선언(`None`)과 매 명령 후 재설정(`None`, `control_worker.rs:314`) 외에는
   실제 루프에서 `Some(...)`으로 대입되는 곳이 없다 — 유닛 테스트가 직접
   구성해서 `verify_due_command`를 호출하는 경우뿐이다. 그 결과 README·TODO가
   "완료"로 적어 둔 레일·조준축 재측정 수렴 판정과 3회 연속 실패 시 중단 로직은
   현재 실기에서 한 번도 발동하지 않는다.
2. **`struck_track_seq`가 `Refined` 단계를 사실상 막는다.** 명령이 하나
   성공하면(단계와 무관하게) 그 즉시 `struck_track_seq`가 채워지고, 이후 같은
   `track_seq`의 모든 요청은 단계와 무관하게 건너뛴다. `Provisional`이 거의
   즉시 도착하므로 `Refined`(0.25초 관측 후)는 도착 전에 이미 막힌다. 이는
   README/TODO의 "공마다 Provisional·Refined를 최대 한 번씩 보낸다"는
   설명과 어긋난다.

두 항목 모두 아래 "문서 갱신"에 각주로 남기고, 코드 동작은 바꾸지 않는다.

## 설계

### `BallControlState`

```rust
enum BallControlState {
    Idle,
    Struck {
        track_seq: u64,
        return_due_at: Instant,
        measurement: PendingImpactMeasurement,
    },
}

/// 복귀 시점에 로그로 남길 임팩트 직후 실측 대상.
struct PendingImpactMeasurement {
    track_seq: u64,
    rail_commanded_m: f64,
    joints_commanded: Joints,
}
```

- `struck_track_seq` · `return_due_at` · `pending_impact_measurement` 세
  로컬을 이 하나의 값으로 대체한다. `Struck` 변형은 세 필드를 동시에
  요구하므로, 셋 중 하나만 설정하는 상태를 만들 수 없다.
- `(u64, f64, Joints)` 익명 튜플을 `PendingImpactMeasurement`라는 이름 있는
  구조체로 바꾼다 — 호출부에서 `.0`/`.1`/`.2` 대신 필드명을 쓴다.
- `CommandLatch`는 그대로 둔다. 이미 이름 있는 작은 상태 타입이고, 관심사가
  다르다(단계별 중복 방지는 `track_seq`가 바뀌면 독립적으로 리셋되며, 쳤는지
  여부와는 별개다).
- `PendingVerification` · `verify_due_command` · `log_verification` ·
  `consecutive_misses`와 관련 상수는 **동작 변경 없이 그대로 둔다.** 다만
  구조체·함수 바로 위에 "현재 실기 루프에서 도달 불가 — `pending_verification`이
  테스트 밖에서는 `Some`으로 대입되지 않는다"는 주석을 추가하고, `BallControlState`
  안에 포함하지 않는다(포함하면 죽은 경로를 살아있는 것처럼 모델링하게 된다).

### 상태 전이

| 상태 | 도착 경로 | 보유 데이터 | 종료 조건 | 종료 시 동작 |
|---|---|---|---|---|
| `Idle` | 워커 시작, 또는 이전 공의 `move_to_center` 완주 직후 | 없음 | `CommitRequest`가 다음을 모두 통과: 아직 안 쳤음(`state`가 이 `track_seq`로 `Struck`이 아님) · latch 허용 · age < 50ms · 마지막 명령 후 ≥20ms | `DirectControlCommand` 계산 → `command_rail_and_racket` → `fixed_impact_push_in` 계획 → `command_joints`. 둘 다 성공하면 그 자리에서 `Struck{ track_seq, return_due_at, measurement }` 생성 |
| `Struck` | `Idle`의 종료 동작 직후 | `track_seq`, `return_due_at`, `measurement` | `Instant::now() >= return_due_at && !hardware.is_busy()` | 실측 로그(commanded vs measured) → `move_to_center`(완주까지 블로킹) → `Idle`로 전이 |

`struck_track_seq == Some(request.track_seq)` 필터는
`matches!(&state, BallControlState::Struck { track_seq, .. } if *track_seq == request.track_seq)`로
바뀐다. 그 외 게이트(age, throttle, latch)는 그대로다.

### 시각화 — `PreviewWindow`에 상태 패널

터미널 로그 대신 `--preview` 창에서 현재 상태를 바로 볼 수 있게 한다.

1. `control_worker`가 `Idle → Struck` · `Struck → Idle` 전이마다 현재 상태를
   메인 스레드에 알린다. 기존 `RuntimeEvent` 열거형에 새 변형
   `ControlState { state: ControlStateSnapshot }`을 추가해 보낸다 — 새 채널은
   만들지 않는다. `Commanded`는 재사용하지 않는다: `Commanded`는
   `command_rail_and_racket` 성공 직후(임팩트 궤적 계산 전)에 나가므로 그
   시점엔 아직 `return_due_at`이 없다 — 관심사가 다른 이벤트를 억지로 겹쳐
   쓰지 않는다. `ControlStateSnapshot`은 `Idle` 또는
   `Struck { track_seq, return_due_at, rail_commanded_m, aim_commanded_rad }`.
2. `PreviewWindow`가 모자이크 우상단에 고정 크기 패널을 그린다: `IDLE`/`STRUCK`
   박스 두 개, 화살표 하나, 현재 상태 강조(칠하기), `Struck`일 때는
   `track_seq`·복귀까지 남은 시간·레일/조준 실측을 짧게 덧붙인다.
3. `src/camera/io/preview`에 사각형·짧은 텍스트를 그리는 프리미티브를
   `draw_circle_px`와 같은 방식으로 추가하고 `camera::Preview` 파사드에 노출한다.
   범용 다이어그램 엔진을 만들지 않는다 — 노드 두 개짜리 고정 레이아웃이다.
4. 이 패널은 `--preview`가 켜져 있을 때만 그려진다(기존 창이 그 조건으로만
   뜨므로). `--sim` 관전 창은 건드리지 않는다.

## 테스트

- 기존 4개 유닛 테스트(`startup_initialization_sets_ready_rail_and_all_joints`,
  `each_prediction_stage_is_sent_only_once_per_ball`,
  `new_track_resets_latch_before_refined_stage`,
  `due_command_needs_two_stable_readbacks`)는 시그니처 변경 없이 그대로 통과해야
  한다.
- `BallControlState`에 대한 새 유닛 테스트 추가: 같은 `track_seq`로 `Struck` 중
  들어온 요청은 무시된다, `Idle`은 새 명령을 허용한다, `Struck` 생성 시 세
  필드가 항상 함께 채워진다(컴파일 타임 보장이라 별도 런타임 단언은 최소).
- 프리뷰 패널은 렌더링 로직이라 유닛 테스트 대상이 아니다 — Windows 실기 또는
  로컬 프리뷰로 육안 확인.

## 문서 갱신

- `src/real/README.md`: "제어 워커" 섹션의 7~8번(재측정·3회 실패 중단)에
  현재 비활성 상태임을 각주로 남긴다.
- `TODO.md` §2.5: 위 "조사 중 발견한 것" 두 항목을 추가한다.
- `docs/two-stage-position-control.md`: "명령 후 실측 오차" 섹션에 같은 각주.

## 범위 제외

- `estimator_worker.rs`의 `track_seq`/`announced_track`/`last_stage`/
  `BallReceding` 상태는 이번 패스에서 건드리지 않는다.
- `run.rs`의 `Outcome`/`LastState`/`main_loop` 지역 상태도 건드리지 않는다.
- `PendingVerification` 부활 또는 제거는 별도 결정 사항으로 남긴다.
- `--sim` kiss3d 관전 창에는 상태 패널을 추가하지 않는다.
