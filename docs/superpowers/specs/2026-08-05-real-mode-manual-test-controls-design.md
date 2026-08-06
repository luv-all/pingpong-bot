# Design: `--mode real` 수동 테스트 컨트롤 (reset / wait / zone)

**작성일:** 2026-08-05
**상태:** approved (user 2026-08-05); **v2 rebase approved (user 2026-08-05)**
**범위:** `src/real/` (`control_worker.rs`, `run.rs`, `preview.rs`, `runtime_event.rs`, `mod.rs`) +
`src/robot/motion/`(`planner.rs`, `physics.rs`)에 좁은 sibling 함수 하나 추가.
estimator/camera 워커, `CommitRequest` 포맷은 건드리지 않는다.

---

## 배경

테스트 프로토콜: 센터·좌·우 각 10개씩, 슈터(사람이 아닌 발사기)가 공을
공급하고, 랠리 없이 한 번씩 친다. 지금 `--mode real`은 이 워크플로를
전제하지 않는다 — 공 하나를 치고 나면 자동으로 `arm.rail.default_x()`
(고정된 센터) 준비 자세로 복귀해, 다음 공을 계속 기다리는 랠리형 루프다.
운영자가 슈터를 좌/우로 겨눌 때 로봇이 여전히 센터에서 기다리므로 응답이
늦다. 또한 뭔가 꼬였을 때(레일·조준축 수렴 실패, latch가 이상한 상태로
남는 등) 프로세스를 재시작하지 않고 수동으로 복구할 방법이 없다.

이 설계는 운영자가 이미 떠 있는 `--preview` highgui 창에서 키를 눌러:
1. 로봇을 즉시 준비 자세로 되돌리고(`reset`),
2. 다음 공을 위해 내부 상태를 정리하고 준비 자세로 보내고(`wait`),
3. 준비 자세의 레일 x를 좌/센터/우 중 하나로 바꾸는(`1`/`2`/`3`)

세 가지를 할 수 있게 한다.

## v2 리베이스 — 2026-08-05, 구현 완료 후 발견

원래 이 설계·구현은 `codex/wrist-linear-control-base`의 한 시점(커밋
`6288585`)에서 5개 태스크로 완료되고 리뷰까지 끝났다. 그 브랜치로 다시
합치려던 중, base 브랜치가 그 사이 같은 파일(`control_worker.rs`)의 같은
함수를 완전히 다른 제어 모델로 재작성했다는 것을 발견했다:

- **스윙 → 정지 정렬.** `Planner::aligned_impact_sequence`(백스윙 +
  임팩트 + 팔로스루, 실제로 공을 침)가 실제 하드웨어 경로에서
  `Planner::ball_alignment`(타격 없이 라켓 중심을 공 예측 위치에 정지
  정렬만 함)로 교체됐다. `aligned_impact_sequence` 자체는 라이브러리에
  남아 있지만 `control_worker.rs`는 더 이상 부르지 않는다.
- **2단계 예측 폐지.** `PredictionStage::{Provisional, Refined}`가
  실제 제어 흐름에서 사라졌다 — `CommandLatch::should_send`/`mark_*`가
  단계 인자를 받지 않고, `RuntimeEvent::Commanded`에 `stage` 필드가
  없다. 타입 자체는 `robot::control`에 남아 있지만 (다른 테스트가 참조)
  실기 루프와는 무관해졌다.
- **`Struck` → `Aligning`.** `BallControlState`/`ControlStateSnapshot`의
  치는 중 상태 variant 이름과 의미가 "타격 완료 후 대기"에서 "정렬 완료
  후 대기"로 바뀌었다. 세 필드(`track_seq`, `return_due_at`,
  `measurement`) 불변식은 그대로다 — 이름만 다르다.
- **복귀 경로가 테이블-충돌-회피형으로 강화됐다.** 예전 `move_to_center`는
  `Planner::ready_prewind`(감긴 준비 자세)로 바로 이동했다. 지금은
  `Planner::return_to_center`(감김 없는 중립 자세)로 이동하되, 직접
  경로가 테이블을 스치면(`plan_neutral_return_segments`) 먼저 위로
  들었다가 중립 자세로 가는 2구간 경로로 자동 전환한다. `ready_prewind`
  자체는 라이브러리에 남아 있지만 실기 경로 어디서도 더는 부르지 않는다
  (`grep`으로 실기 경로 호출부 0건 확인).

**결론: 이건 기계적 merge 충돌이 아니라 재구현이 필요하다.** 사용자
결정(2026-08-05): base의 새 제어 모델(정지 정렬, 테이블 회피 복귀) 위에
이 기능(수동 리셋/대기/존 선택)을 다시 얹는다 — 예전 스윙 기반 구현을
그대로 되살리지 않는다.

### 재적용 매핑

| 원래(스윙 모델) | v2(정렬 모델) |
|---|---|
| `Planner::ready_prewind_at` (신규, `ready_prewind` 곁에 추가) | `Planner::return_to_center_at` (신규, `return_to_center` 곁에 추가) |
| `move_to_ready(hardware, arm, rail_x)`가 `Planner::ready_prewind_at` 한 번만 호출 | `move_to_ready(hardware, arm, rail_x)`가 `plan_neutral_return_segments(arm, start, rail_x)`(테이블 회피 2구간 로직 포함, 목표 rail_x를 파라미터로 받도록 일반화)를 호출 |
| `BallControlState::Struck { .. }` | `BallControlState::Aligning { .. }` (필드 동일) |
| `CommandLatch::should_send(track_seq, stage)` / `mark_sent(stage)` / `mark_struck()` | `CommandLatch::should_send(track_seq)` / `mark_finished()` (base가 이미 이렇게 단순화함 — 그대로 재사용) |
| `apply_test_control`이 `move_to_ready` 실패 시 항상 `Failed` + `break` | base의 자동 복귀 분기가 이미 확립한 관례를 따른다: `MoveError::Hardware` → `Failed` + `break`(치명적), `MoveError::Plan` → `Failed` + `Idle` 상태로 정리 + `continue`(치명적이지 않음, 다음 공 계속 대기). 운영자가 누른 버튼이라도 계획 실패가 전체 세션을 죽일 이유는 없다는 base의 이미 확립된 철학을 그대로 따른다. |
| `TestZone`/`TestControl` 타입, `RAIL_ZONE_SAFETY_MARGIN_RATIO` | **변경 없음** — `LinearRail`/`defaults::hardware`에만 의존, control_worker 내부와 무관해 그대로 이식 |
| `PreviewWindow::set_zone`/상태 패널 zone 줄/키 범례, `run.rs` 키 라우팅 | **변경 없음** — `ControlStateSnapshot::Aligning`으로 이름만 참조가 바뀔 뿐 (좌표는 그대로) |

## 아키텍처

```
preview 키 입력 → run.rs main_loop → TestControl 채널 → control_worker 루프
                                                              │
                                              (ResetPosition은 즉시 적용,
                                               Wait/SetZone은 하드웨어가
                                               idle일 때 적용)
```

기존 `CommitRequest`(공 추적 → 명령) 경로는 그대로 둔다. `TestControl`은
완전히 별도 채널로, 운영자 입력만 실어 나른다.

## 컴포넌트

### `TestZone` — `src/real/test_control.rs` (신규, v1과 동일)

```rust
pub enum TestZone { Left, Center, Right }

impl TestZone {
    /// 이 존의 준비 자세 레일 x. 기존 `LinearRail` 상수를 그대로 쓴다 —
    /// 슈터 위치에 맞춰 튜닝하려면 `defaults::hardware`의
    /// `RAIL_X_MIN_M`/`RAIL_X_MAX_M`/`RAIL_READY_X_M`만 바꾸면 된다.
    pub fn rail_x(self, rail: LinearRail) -> f64 {
        let margin = (rail.x_max - rail.x_min) * RAIL_ZONE_SAFETY_MARGIN_RATIO;
        match self {
            Self::Left => rail.x_min + margin,
            Self::Center => rail.default_x(),
            Self::Right => rail.x_max - margin,
        }
    }
}
```

**Left = `x_min`, Right = `x_max`는 미검증 가정이다.** 실기에서 방향이
반대로 확인되면 이 매핑 두 줄만 뒤집으면 된다 — 그 외 어디도 좌/우 방향에
의존하지 않는다.

좌/우 존은 레일 전체 구간의 5%만큼 안쪽으로 물러난 위치를 목표로 한다 —
기계적 하드 스탑에 반복적으로 바짝 붙는 것을 피하기 위한 안전 여유다.
비율은 `pingpong_bot::defaults::RAIL_ZONE_SAFETY_MARGIN_RATIO`
(`src/defaults/hardware.rs`) 하나로 튜닝한다. Center는 영향받지 않는다.

### `TestControl` — 같은 파일 (v1과 동일)

```rust
pub enum TestControl {
    /// 즉시 적용 — 하드웨어가 움직이는 중이어도 `cancel()`로 멈추고 복귀.
    ResetPosition,
    /// 다음 idle 시점에 적용 — 존 변경 없이 latch·상태만 정리하고 현재
    /// home_rail_x로 복귀.
    Wait,
    /// 다음 idle 시점에 적용 — home_rail_x를 바꾸고 Wait과 동일하게 정리.
    SetZone(TestZone),
}
```

### 키 바인딩 — `src/real/preview.rs` / `run.rs` (v1과 동일)

`highgui::wait_key_ex`가 이미 `q`/ESC 외의 키를 `PreviewAction::Key(i32)`로
돌려주는데, 지금은 `preview.show() -> bool`이 그 정보를 버린다. `show()`가
`PreviewAction`을 그대로 반환하도록 바꾸고, `run.rs`의 `main_loop`에서
매핑한다:

| 키 | 동작 |
|---|---|
| `1` | `SetZone(Left)` |
| `2` | `SetZone(Center)` |
| `3` | `SetZone(Right)` |
| `w` | `Wait` |
| `r` | `ResetPosition` |
| `q` / ESC | (기존 그대로) Quit |

매핑에 없는 키는 무시(`debug!`로만 남김).

### `control_worker.rs` 변경 (v2 — 정렬 모델 위에 재적용)

- `home_rail_x: f64` 지역 변수 추가, `arm.rail.default_x()`로 시작.
  `current_zone: TestZone` 지역 변수 추가, `TestZone::Center`로 시작.
- `Receiver<TestControl>`을 `spawn()` 파라미터로 추가. 메인 while 루프
  최상단(`'control:` 라벨 추가), `verify_due_command` 다음에 non-blocking
  `try_recv()`로 확인.
- `SetZone`/`Wait`는 즉시 실행하지 않고 `pending_test_control: Option<TestControl>`에
  저장 — 이미 있는 due-for-return 체크(`pending_verification.is_none() && !hardware.is_busy()`)
  자리에서 함께 소비한다. 여러 번 눌리면 마지막 것만 남는다.
- 적용 시 공통 동작(`apply_test_control`): `pending_verification = None`,
  `latch = CommandLatch::default()`, `state = BallControlState::Idle`,
  `ControlStateSnapshot::Idle` 이벤트 발행, `home_rail_x`로 이동
  (`move_to_ready` → `plan_neutral_return_segments(arm, start, home_rail_x)`,
  base의 테이블 회피 로직 그대로 재사용), `RuntimeEvent::TestZoneChanged` 발행.
- `move_to_ready` 실패 시: `MoveError::Hardware`는 `Failed` 이벤트 + 호출부가
  `break`(치명적, base의 자동 복귀 분기와 동일 관례). `MoveError::Plan`은
  `Failed` 이벤트 + `Idle` 상태 정리 후 계속 진행(치명적이지 않음) — 다음
  공을 계속 기다린다.
- `ResetPosition`은 위 로직을 idle 대기 없이 바로 실행하되, `hardware.is_busy()`면
  먼저 `hardware.cancel()`을 호출한다(기존 `MAX_CONSECUTIVE_MISSES` 경로가 쓰는
  것과 같은 안전 정지).
- `move_to_center(hardware, arm)` → `move_to_ready(hardware, arm, rail_x)`로
  개명·확장(호출부 갱신). `plan_neutral_return_segments`도 목표 rail_x를
  파라미터로 받도록 일반화(`initialize_pose_attempt`는 계속
  `rail.default_x()`를 명시적으로 넘겨 시작 동작 불변).

### `Planner::return_to_center_at` — `src/robot/motion/planner.rs` / `physics.rs`

기존 `plan_return_to_center(arm, start)`는 내부에서
`center_rail_x = arm.rail.default_x()`를 계산한다. 이 값을 파라미터로 받는
사설 헬퍼로 뽑아내고, 공개 API 두 개(`return_to_center`, `return_to_center_at`)가
그 헬퍼를 감싼다. 기존 호출부가 쓰는 `return_to_center(arm, start)`는
동작이 바뀌지 않는다.

### `RuntimeEvent` / `PreviewWindow` (v1과 동일, `Struck`→`Aligning` 이름만 참조)

- `RuntimeEvent::TestZoneChanged { zone: TestZone, home_rail_x: f64 }` 추가 —
  실제로 적용됐을 때만 발행(버튼을 눌렀다고 바로 보내지 않는다).
- `PreviewWindow`에 `current_zone: Option<(TestZone, f64)>` 필드, 상태 패널에
  "ZONE LEFT x=0.070" 한 줄과 키 범례("1/2/3 zone  w wait  r reset")를 추가.

## 에러 처리

- 알 수 없는 키: 무시.
- `return_to_center_at` 실패(이론상 x_min/x_max는 이미 유효한 레일 범위라
  거의 발생하지 않음): `MoveError::Plan`으로 분류돼 세션을 죽이지 않는다
  (위 "재적용 매핑" 참고).
- `ResetPosition`을 눌렀는데 `pending_test_control`에 `Wait`/`SetZone`이
  이미 대기 중이면 `ResetPosition`이 그 자리에서 즉시 이겨서 소비하고,
  대기 중이던 것은 버린다(둘 다 결국 같은 정리 동작이므로 유실 문제 없음).

## 테스트

- `TestZone::rail_x` 매핑 단위 테스트(Left/Center/Right → margin 적용된
  x_min/default_x/x_max).
- `CommandLatch`/`BallControlState`를 "완전 리셋"하는 헬퍼의 단위 테스트
  (기존 "aligned track는 영구 차단" 테스트들과 같은 스타일).
- idle 적용 게이팅을 순수 함수로 뽑아 `(hardware_busy, pending_verification) -> bool`
  단위 테스트(실기 없이 검증).
- `return_to_center_at(arm, start, x)`가 `x`로 정확히 레일을 이동하는지 —
  기존 `return_to_center` 테스트와 나란히 추가.
