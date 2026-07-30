# Design: real continuous feed (연속 급구)

**Date:** 2026-07-31  
**Status:** approved (user 2026-07-31)  
**Out of scope (this slice):** true rally (결선), omitting full return-to-center, max_shots session cap, changing Hardware ownership

---

## Goal

`--mode real`이 **공 하나 = 스윙 하나**에서 끝나지 않고, 사람이 공을 연속으로 넣어 주는 **연속 급구**를 반복한다.

**Done when:**

- 스윙 커밋 → 완주 → 센터 복귀 후, 다음 급구를 다시 추적·커밋할 수 있다.
- 이번 스윙이 `Infeasible`(관절·토크)여도 **세션은 유지**되고, 센터 복귀 뒤 다음 공을 다시 시도한다.
- 추정은 샷마다 **새 EKF 궤적**을 쓴다. 공 y가 로봇에서 멀어지는 방향(증가)이면 새 루프로 본다.
- 메인은 `Committed` / `Infeasible`로 프로세스를 종료하지 않는다 (종료는 현행 ESC/`q` 등 사용자 종료).
- 결선(진짜 랠리)용 변경점은 코드 주석 + 이 문서 §Future에만 기록한다 (구현 없음).

---

## Context

| 항목 | 현재 (단발) | 이 슬라이스 |
|------|-------------|-------------|
| `control_worker` | `wait_for_commit` 1회 후 `Done` | 바깥 루프: commit → idle → center → 반복 |
| `ShotEvent::ends_shot` | 세션 종료 신호 | 샷 로그만 (세션 종료 아님) |
| EKF / `announced_track` | 프로세스 수명 | 샷 경계에서 리셋 |
| 좌표 | 로봇 y≈0, 상대/급구 y→`LENGTH_Y` | y **증가** = 멀어짐 = 새 루프 후보 |
| 1·2차 테스트 | — | 연속 급구 |
| 결선 | — | 진짜 랠리 (나중에, §Future) |

참고: `src/real/README.md` “랠리로 넓힐 때”, sim `shot_seq` · `park_if_out_of_play` · `hard_fail_streak`.

**선택한 접근 (A):** 제어 워커에 단발 시퀀스를 루프하고, 추정 워커는 Recovering 동안 `Attempt`를 막는다.  
거절: 샷마다 워커 재시작(DXL/캠 비용), 메인이 궤적 타이밍을 오케스트레이션(소유권 불변식 약화).

---

## Behavior

### Shot cycle

```text
Armed/Ready
  → Tracking (EKF velocity seeded, 샷당 1회 로그)
  → Attempt / plan_best 재시도
  → Committed 또는 Infeasible(이번 스윙만)
  → Recovering: wait_idle + return_to_center
  → Ready (다음 급구)
```

**재무장 조건 (1·2차):** 스윙 완주 **그리고** 센터 복귀 완주.

```text
// NOTE(결선): 진짜 랠리에서는 풀 센터 복귀 전에 다음 스윙을
// 허용하도록 이 재무장 조건을 바꿀 수 있다. 지금은 연속 급구만.
```

### New-loop signal (estimator)

추적 중 추정 공의 **y가 증가하는 방향**(로봇에서 멀어짐)이면 새 루프로 본다:

- EKF를 리셋해 **이전 추정과 다른 새 추정**을 시작
- 샷 단위 플래그(`announced_track` 등) 초기화
- 진행 중 `CommitRequest`는 무효 (채널 drain 또는 Ready 전 ignore)

노이즈 오인 방지: 최소 Δy 및/또는 지속 샘플(히스테리시스). 정확한 임계는 구현 시 `ControlParams` 또는 real 상수로 두고 주석으로 근거를 남긴다.

### Failure policy

| 사건 | 이번 스윙 | 세션 |
|------|-----------|------|
| `Committed` | 완주 → 센터 → Ready | 계속 |
| `Infeasible` | 스윙 안 함(또는 중단) → 센터 → Ready | 계속, 다음 공 재시도 |
| `Failed` (버스 등) | 치명적이면 기존처럼 종료 가능 | HW 복구 불가면 종료 |
| `PlanFailed` / stale | 재시도 (현행) | 계속 |

`hard_fail_streak`는 이번 슬라이스에서 **필수가 아니다**. 필요해지면 sim과 같이 로그·백오프만 추가.

### Main loop

- `Committed` / `Infeasible`로 `outcome`을 잡고 프로세스를 끝내지 **않는다**.
- 샷 이벤트 로그에 `shot_seq`를 붙인다.
- `--preview` freeze-after-first-shot 동작은 **제거 또는 샷 HUD만 갱신**으로 바꾼다 (연속 급구와 충돌).
- 창 없는 모드도 사용자 중단/프로세스 킬 전까지 루프 (타임아웃을 샷당으로 둘지는 구현 시 현행 `--timeout-secs`와 맞춰 결정; 기본 의도: **대기 타임아웃은 “첫 공”이 아니라 “Ready 이후 다음 공”에 재적용 가능**).

---

## Loop caveats (구현 시 체크리스트)

1. **Hardware 단독 소유** — `read_pose → plan_best → command`는 제어 워커만.
2. **Recovering 중 Attempt 금지** — 복귀 중 커밋하면 포즈/궤적 불일치. Ready 신호 또는 busy 시 요청 discard.
3. **커밋 채널 drain** — `bounded(1)`에 남은 샷 N 요청이 샷 N+1에 쓰이지 않게.
4. **y-증가 히스테리시스** — 측정 노이즈로 EKF가 매 프레임 리셋되지 않게.
5. **샷당 플래그** — `Tracking` 로그·게이트 전이 로그가 샷마다 다시 의미 있게.
6. **Infeasible ≠ 세션 종료** — 바깥 루프 유지.
7. **`MAX_REQUEST_AGE` / plan throttle 유지** — 루프여도 낡은 예측으로 command 금지.
8. **센터 복귀 실패** — warn 후 Ready로 갈지, Failed로 세션을 끊을지 명시 (권장: warn + 현재 포즈에서 Ready, 치명적 HW만 Failed).

---

## Files (expected touch)

| 파일 | 변경 |
|------|------|
| `src/real/control_worker.rs` | 바깥 루프, Recovering→Ready, 채널 drain, 결선 NOTE 주석 |
| `src/real/estimator_worker.rs` | 샷 리셋, y-증가 새 루프, Ready 게이트 |
| `src/real/shot_event.rs` | `shot_seq`, `ends_shot` 의미 축소 또는 제거 |
| `src/real/run.rs` | 세션 종료 조건 완화, 샷 로그 |
| `src/real/README.md` | 라이프사이클·한계 갱신 |
| (선택) `decision.rs` | 변경 최소 — 게이트는 샷 안에서 현행 유지 |

---

## Future: true rally (결선) — 기록만, 이 슬라이스에서 구현하지 않음

1·2차는 **연속 급구**. 결선은 **진짜 랠리**(돌아오는 공을 이어서 침). 추가 시 검토할 것:

1. **재무장 조건** — 풀 `return_to_center` 완주를 기다리지 않을 수 있음 (부분 복귀, follow-through 종료, `is_busy==false`만 등). 1·2차 코드의 NOTE가 이 분기점.
2. **샷 경계** — y-증가만이 아니라 네트 통과 후 **재접근(y 감소 재개)**, 임팩트 직후 추적 공백, 상대 리턴 궤적.
3. **out-of-play** — sim `park_if_out_of_play`에 해당하는 lost-track / 테이블 밖 / 비행 시간 상한.
4. **자기 간섭** — 스윙 직후 라켓·팔이 검출에 섞이지 않게 게이트.
5. **sim 정렬** — `shot_seq`, `swing_committed` 리셋, cancel_inflight에 해당하는 실기 정책.

연속 급구 구현이 이 §Future를 막지 않도록: 재무장 조건을 한 함수/한 분기로 모아 두고, 추정 리셋 트리거를 교체 가능하게 둔다.

---

## Testing

- 단위: y-증가 → 리셋 트리거 (히스테리시스 포함), Recovering 중 Attempt 미전송.
- dry-run: 커밋 2회 이상 로그(`shot_seq` 증가), Infeasible 후에도 다음 Armed/Ready.
- 실기(Windows): 연속 급구 2~3구, 매 구 센터 복귀 확인.
