# Real Continuous Feed (연속 급구) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `--mode real`이 스윙 완주·센터 복귀 후 다음 급구를 반복 커밋하게 하고, 결선(진짜 랠리)용 확장 지점만 주석·스펙 §Future로 남긴다.

**Architecture:** 제어 워커가 단발 시퀀스(`wait_for_commit → idle → return_to_center`)를 바깥 루프로 돌린다. `ControlStatus::{Ready, Recovering}` 채널로 추정 워커의 `Attempt`를 게이트한다. 공 y 증가(로봇에서 멀어짐) 히스테리시스로 EKF를 샷 경계에서 리셋한다. 메인은 `Committed`/`Infeasible`로 세션을 끝내지 않는다.

**Tech Stack:** Rust, crossbeam-channel, 기존 `src/real/*`, `Ekf::reset`, `Planner::return_to_center`.

**Spec:** [`docs/superpowers/specs/2026-07-31-real-continuous-feed-design.md`](../specs/2026-07-31-real-continuous-feed-design.md)

## Global Constraints

- Hardware 단독 소유 유지 — `read_pose → plan_best → command`는 제어 워커만
- 공유 가변 상태 금지 — 스레드 간은 채널만
- 1·2차 재무장 = 스윙 완주 **그리고** 센터 복귀 완주 (결선에서 바꿀 수 있음을 NOTE로 남김)
- 진짜 랠리 구현 금지 — §Future 기록·주석만
- `Infeasible`는 이번 스윙만 포기, 세션·바깥 루프는 유지
- 파일당 주 타입 1개 (repo 규약)
- 숫자 후보는 real 모듈 상수로 두고 주석에 근거를 남김 (`ControlParams`에 새 필드 넣지 않음 — sim 공유 SSOT 오염 방지)

---

## File map

| File | Responsibility |
|------|----------------|
| `src/real/ball_receding.rs` | **Create.** 공 y 증가 히스테리시스 순수 판정 |
| `src/real/control_status.rs` | **Create.** 제어 → 추정 `Ready`/`Recovering` + `shot_seq` |
| `src/real/shot_event.rs` | `shot_seq` 필드, `ends_shot` 제거(Task 6) |
| `src/real/control_worker.rs` | 바깥 루프, drain, status 송신, 결선 NOTE |
| `src/real/estimator_worker.rs` | Ready 게이트, y-리셋, 샷 플래그 초기화 |
| `src/real/run.rs` | status 채널 배선, 세션 비종료, timeout을 Armed마다 재장전 |
| `src/real/mod.rs` | 모듈 문서·export, 단발→연속 급구 문구 |
| `src/real/README.md` | 라이프사이클·한계·§Future 포인터 |
| `docs/superpowers/specs/2026-07-31-real-continuous-feed-design.md` | Status → approved |

```text
  cam ──► estimator ──┬─ CommitRequest ──► control ──► Hardware
                      │                      │
                      │◄── ControlStatus ────┘
                      │     (Ready / Recovering)
                      └─ ShotEvent ──► main (로그·프리뷰, 세션 유지)
```

---

### Task 1: `BallReceding` 순수 히스테리시스

**Files:**
- Create: `src/real/ball_receding.rs`
- Modify: `src/real/mod.rs` — `mod ball_receding;`
- Test: `src/real/ball_receding.rs` (`#[cfg(test)]`)

**Interfaces:**
- Produces:
  - `pub struct BallReceding`
  - `pub fn new(min_delta_y: f64, min_samples: u32) -> Self`
  - `pub fn observe(&mut self, ball_y: f64) -> bool` — true면 새 루프 신호; true 반환 후 streak 0
  - `pub fn reset(&mut self)`
  - `pub const MIN_DELTA_Y: f64 = 0.05;` (5 cm)
  - `pub const MIN_SAMPLES: u32 = 3;`
- Consumes: 없음

- [ ] **Step 1: Write the failing tests**

`src/real/ball_receding.rs`에 모듈 + 테스트. Step 1에서는 `observe` stub가 항상 `false`.

```rust
//! 추정 공 y가 로봇에서 멀어지는지(증가) 히스테리시스로 본다.
//!
//! 로봇은 y≈0, 상대/급구는 y→LENGTH_Y. y 증가 = 새 급구 루프 후보.
//! 노이즈로 EKF가 매 프레임 리셋되지 않게 Δy·연속 샘플을 요구한다.

/// 한 샘플에서 인정할 최소 y 증가 [m].
pub const MIN_DELTA_Y: f64 = 0.05;
/// `MIN_DELTA_Y` 이상 증가가 연속으로 이만큼 나와야 확정.
pub const MIN_SAMPLES: u32 = 3;

#[derive(Debug, Clone)]
pub struct BallReceding {
    min_delta_y: f64,
    min_samples: u32,
    last_y: Option<f64>,
    streak: u32,
}

impl BallReceding {
    pub fn new(min_delta_y: f64, min_samples: u32) -> Self {
        return Self {
            min_delta_y,
            min_samples,
            last_y: None,
            streak: 0,
        };
    }

    pub fn reset(&mut self) {
        self.last_y = None;
        self.streak = 0;
    }

    /// `true` = 새 루프 신호. 호출 측에서 EKF 리셋 후 이 검출기도 `reset`할 것.
    pub fn observe(&mut self, ball_y: f64) -> bool {
        let _ = ball_y;
        return false; // Step 3에서 교체
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ignores_small_noise() {
        let mut d = BallReceding::new(0.05, 3);
        assert!(!d.observe(1.0));
        assert!(!d.observe(1.02));
        assert!(!d.observe(1.04));
        assert!(!d.observe(1.06));
    }

    #[test]
    fn fires_after_sustained_increase() {
        let mut d = BallReceding::new(0.05, 3);
        assert!(!d.observe(0.5));
        assert!(!d.observe(0.56));
        assert!(!d.observe(0.62));
        assert!(d.observe(0.68));
    }

    #[test]
    fn decreasing_y_clears_streak() {
        let mut d = BallReceding::new(0.05, 3);
        assert!(!d.observe(0.5));
        assert!(!d.observe(0.56));
        assert!(!d.observe(0.50));
        assert!(!d.observe(0.56));
        assert!(!d.observe(0.62));
        assert!(d.observe(0.68));
    }
}
```

`mod.rs`에 `mod ball_receding;` 추가.

스펙 파일 상단 `Status`를 `approved (user 2026-07-31)`로 바꾼다.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p pingpong-bot --bin pingpong-bot fires_after_sustained_increase -- --exact`

Expected: FAIL (`assert!(d.observe(0.68))` is false)

- [ ] **Step 3: Implement `observe`**

```rust
pub fn observe(&mut self, ball_y: f64) -> bool {
    let Some(prev) = self.last_y else {
        self.last_y = Some(ball_y);
        return false;
    };
    self.last_y = Some(ball_y);
    if ball_y - prev >= self.min_delta_y {
        self.streak = self.streak.saturating_add(1);
    } else {
        self.streak = 0;
    }
    if self.streak >= self.min_samples {
        self.streak = 0;
        return true;
    }
    return false;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p pingpong-bot --bin pingpong-bot ball_receding -- --nocapture`

Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/real/ball_receding.rs src/real/mod.rs \
  docs/superpowers/specs/2026-07-31-real-continuous-feed-design.md
git commit -m "$(cat <<'EOF'
feat(real): add ball-y receding hysteresis for shot boundaries

EOF
)"
```

---

### Task 2: `ControlStatus` 채널 타입

**Files:**
- Create: `src/real/control_status.rs`
- Modify: `src/real/mod.rs` — `mod control_status; pub use control_status::ControlStatus;`

**Interfaces:**
- Produces:

```rust
/// 제어 워커 → 추정 워커. Recovering 동안 Attempt를 막는다.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlStatus {
    /// 커밋 요청을 받아도 된다. `shot_seq`는 이번 급구 번호(1부터).
    Ready { shot_seq: u64 },
    /// 스윙 완주·센터 복귀 중 — CommitRequest 보내지 말 것.
    Recovering { shot_seq: u64 },
}
```

- Consumes: 없음

- [ ] **Step 1: Add the file and export**

위 enum을 `src/real/control_status.rs`에 두고 `mod.rs`에서 export.

- [ ] **Step 2: Compile check**

Run: `cargo check -p pingpong-bot --bin pingpong-bot`

Expected: success

- [ ] **Step 3: Commit**

```bash
git add src/real/control_status.rs src/real/mod.rs
git commit -m "$(cat <<'EOF'
feat(real): add ControlStatus Ready/Recovering messages

EOF
)"
```

---

### Task 3: `ShotEvent`에 `shot_seq`

**Files:**
- Modify: `src/real/shot_event.rs`
- Modify: `src/real/control_worker.rs` — 생성부에 `shot_seq` 전달 (임시 `0` 가능, Task 4에서 실값)
- Modify: `src/real/estimator_worker.rs` — 동일
- Modify: `src/real/run.rs` — `log_event` / `result_lines` / `Outcome::from_event` 패턴 매칭

**Interfaces:**
- Produces: `Armed` / `Tracking` / `Committed` / `Infeasible` / `PlanFailed` / `Failed`에 `shot_seq: u64`
- `Done`은 `shot_seq` 없음 (워커 종료)
- `ends_shot`은 Task 6까지 유지. doc만 “연속 급구에서는 세션 종료에 쓰지 말 것”으로 변경

- [ ] **Step 1: Add `shot_seq` to variants**

```rust
pub enum ShotEvent {
    Armed { shot_seq: u64, pose: robot::Pose },
    Tracking { shot_seq: u64, position: Point3, speed: f64 },
    Committed {
        shot_seq: u64,
        time_to_impact_secs: f64,
        duration_secs: f64,
        impact: Point3,
        rail_start: f64,
        rail_end: f64,
        peak_joint_speed: f64,
    },
    Infeasible { shot_seq: u64, reason: String },
    PlanFailed { shot_seq: u64, reason: String },
    Failed { shot_seq: u64, reason: String },
    Done,
}
```

- [ ] **Step 2: Fix all constructors and match arms** so the bin compiles (`shot_seq: 0` placeholder OK).

- [ ] **Step 3: Compile**

Run: `cargo check -p pingpong-bot --bin pingpong-bot`

Expected: success

- [ ] **Step 4: Commit**

```bash
git add src/real/shot_event.rs src/real/control_worker.rs src/real/estimator_worker.rs src/real/run.rs
git commit -m "$(cat <<'EOF'
feat(real): thread shot_seq through ShotEvent variants

EOF
)"
```

---

### Task 4: `control_worker` 바깥 루프 + status + drain

**Files:**
- Modify: `src/real/control_worker.rs`
- Modify: `src/real/run.rs` — `status_tx` 생성·전달, `shutdown` 클론을 control에 전달

**Interfaces:**
- Consumes: `Receiver<CommitRequest>`, `Shutdown`
- Produces: `Sender<ControlStatus>`; 루프마다 `Ready` → commit → `Recovering` → idle/center → drain
- `spawn(..., rx, status_tx, sim_tx, event_tx, shutdown)`

**CommitOutcome (private):**

```rust
enum CommitOutcome {
    Committed,
    Infeasible,
    Disconnected,
    Failed,
}
```

- [ ] **Step 1: Rewrite `spawn` outer loop**

요지:

```rust
let mut shot_seq: u64 = 0;
while !shutdown.is_down() {
    shot_seq = shot_seq.saturating_add(1);
    // read_pose → Armed { shot_seq, pose }
    let _ = status_tx.send(ControlStatus::Ready { shot_seq });

    let outcome = wait_for_commit(..., shot_seq);

    if matches!(outcome, CommitOutcome::Failed) {
        break;
    }
    if matches!(outcome, CommitOutcome::Disconnected) {
        break;
    }

    let _ = status_tx.send(ControlStatus::Recovering { shot_seq });

    // NOTE(결선): 진짜 랠리에서는 풀 센터 복귀 전에 다음 스윙을
    // 허용하도록 이 재무장 조건을 바꿀 수 있다. 지금은 연속 급구만.
    if matches!(outcome, CommitOutcome::Committed) {
        wait_idle(hardware.as_mut());
    }
    if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
        warn!(%error, "센터 복귀 실패 — 현재 자세에서 Ready");
    }
    while rx.try_recv().is_ok() {}
}
let _ = event_tx.send(ShotEvent::Done);
```

`Infeasible`여도 break하지 않는다. `wait_for_commit` 시그니처에 `shot_seq: u64`를 추가하고 모든 `ShotEvent`에 넣는다.

홈 이동은 루프 **밖**에서 1회만 (기존과 동일).

- [ ] **Step 2: Wire `status_tx` / `shutdown` in `run.rs`**

```rust
let (status_tx, status_rx) = unbounded::<ControlStatus>();
// control_worker::spawn(..., status_tx, ..., shutdown.clone())
// estimator는 Task 5에서 status_rx 수신 — Task 4·5를 이어서 구현한 뒤 한 번에 check
```

- [ ] **Step 3: Compile together with Task 5**

Run: `cargo check -p pingpong-bot --bin pingpong-bot`

Expected: success

- [ ] **Step 4: Commit**

```bash
git add src/real/control_worker.rs src/real/run.rs
git commit -m "$(cat <<'EOF'
feat(real): loop control worker for continuous feed rearm

EOF
)"
```

---

### Task 5: `estimator_worker` Ready 게이트 + y-리셋

**Files:**
- Modify: `src/real/estimator_worker.rs`
- Modify: `src/real/run.rs` — `status_rx`를 estimator에 전달

**Interfaces:**
- Consumes: `Receiver<ControlStatus>`, `BallReceding`
- Produces: `accepting == true`일 때만 `CommitRequest`; receding 시 `ekf.reset()`

- [ ] **Step 1: Extend `spawn` and loop state**

```rust
pub fn spawn(
    rx: Receiver<VisionEvent>,
    calibration: Calibration,
    intercept: InterceptWindow,
    commit_tx: Sender<CommitRequest>,
    status_rx: Receiver<ControlStatus>,
    preview_tx: Option<Sender<PreviewEvent>>,
    sim_tx: Option<Sender<SimUpdate>>,
    event_tx: Sender<ShotEvent>,
    shutdown: Shutdown,
) -> JoinHandle<EstimatorStats>
```

상태:

```rust
let mut accepting = false;
let mut shot_seq: u64 = 0;
let mut receding = BallReceding::new(MIN_DELTA_Y, MIN_SAMPLES);
let mut announced_track = false;
```

매 vision 이벤트 처리 전:

```rust
while let Ok(status) = status_rx.try_recv() {
    match status {
        ControlStatus::Ready { shot_seq: seq } => {
            accepting = true;
            shot_seq = seq;
            announced_track = false;
            last_decision = None;
            receding.reset();
            ekf.reset();
        }
        ControlStatus::Recovering { .. } => {
            accepting = false;
        }
    }
}
```

Attempt:

```rust
Decision::Attempt if accepting => { let _ = commit_tx.try_send(request); }
Decision::Attempt => {}
```

y-리셋 (`accepting && tracking`일 때만):

```rust
if accepting {
    if let Some(y) = ball_y {
        if receding.observe(y) {
            ekf.reset();
            announced_track = false;
            last_decision = None;
            receding.reset();
            debug!(shot = shot_seq, y = f2(y), "공 y 증가 — EKF 리셋 (새 루프)");
        }
    }
}
```

`Tracking` / 기타 이벤트에 `shot_seq` 사용.

- [ ] **Step 2: Compile**

Run: `cargo check -p pingpong-bot --bin pingpong-bot`

Expected: success

- [ ] **Step 3: Commit**

```bash
git add src/real/estimator_worker.rs src/real/run.rs src/real/control_worker.rs
git commit -m "$(cat <<'EOF'
feat(real): gate commits on Ready and reset EKF when ball recedes

EOF
)"
```

---

### Task 6: 메인 세션 비종료 + timeout 재장전

**Files:**
- Modify: `src/real/run.rs`
- Modify: `src/real/shot_event.rs` — `ends_shot` 삭제

**Interfaces:**
- `ShotEvent::Done` + non-preview → 프로세스 종료 가능
- preview → ESC/`q`만 종료
- `Armed`마다 `wait_deadline` 재장전; timeout은 warn만 (세션 유지)

- [ ] **Step 1: Rewrite `main_loop` termination**

1. `ends_shot()`로 guard drop / `FINISH_GRACE` break **삭제**
2. preview result HUD는 최신 샷으로 덮어씀 (첫 샷 freeze 제거)
3. `ShotEvent::Armed { .. }`에서 `wait_deadline = now + timeout_secs`
4. timeout: `warn!`만, 프로세스 유지
5. `Done` && `!preview` → break
6. `FINISH_GRACE` / `finish_deadline` 제거
7. 요약용 `Outcome`에 `shots_seen` + 마지막 결과

- [ ] **Step 2: Delete `ends_shot`**

- [ ] **Step 3: Log `shot_seq` on every shot event**

```rust
info!(shot = shot_seq, duration_secs = f2(*duration_secs), ..., "real shot: swing commit");
```

- [ ] **Step 4: Test + check**

```bash
cargo test -p pingpong-bot --bin pingpong-bot ball_receding
cargo check -p pingpong-bot --bin pingpong-bot
```

Expected: PASS / success

- [ ] **Step 5: Commit**

```bash
git add src/real/run.rs src/real/shot_event.rs
git commit -m "$(cat <<'EOF'
feat(real): keep session alive across continuous-feed shots

EOF
)"
```

---

### Task 7: 문서

**Files:**
- Modify: `src/real/mod.rs`
- Modify: `src/real/README.md`
- Modify: `src/main.rs` (단발 문구)

- [ ] **Step 1: Update docs**

- `mod.rs`: 연속 급구(1·2차), spec/plan 링크, “커밋 래치” → “샷 루프 + Recovering 게이트”
- README: `Committed/Infeasible → Recovering → Ready`; 한계를 “진짜 랠리 미지원 / 연속 급구 지원”으로; §Future → spec
- `main.rs` 주석: 연속 급구 설명

- [ ] **Step 2: Commit**

```bash
git add src/real/mod.rs src/real/README.md src/main.rs
git commit -m "$(cat <<'EOF'
docs(real): document continuous feed and future rally hooks

EOF
)"
```

---

### Task 8: dry-run 스모크 (수동)

**Files:** 없음

- [ ] **Step 1: Build**

Run: `cargo build -p pingpong-bot --bin pingpong-bot`

- [ ] **Step 2: Manual dry-run**

```bash
cargo run -p pingpong-bot -- --mode real --dry-run --preview --timeout-secs 5
```

Expected:
- 첫 timeout에 프로세스가 죽지 않음
- ESC/`q`로 종료
- 가능하면 `shot`/`Ready` 로그 확인

- [ ] **Step 3: Commit only if smoke forced tweaks**

---

## Self-review (plan vs spec)

| Spec requirement | Task |
|------------------|------|
| 완주+센터 후 재무장 | Task 4 |
| 결선 NOTE | Task 4, 7 |
| 구조 루프 | Task 4 |
| 새 EKF / y 증가 | Task 1, 5 |
| 메인 세션 비종료 | Task 6 |
| Infeasible 후 다음 스윙 | Task 4 |
| Recovering 중 Attempt 금지 | Task 5 |
| 채널 drain | Task 4 |
| 히스테리시스 | Task 1 |
| shot_seq | Task 3–5 |
| 진짜 랠리 미구현·기록 | Task 7 + spec §Future |
| Hardware 소유권 | 유지 (Task 4) |

Type names consistent: `ControlStatus`, `BallReceding`, `CommitOutcome`, `shot_seq: u64`.
