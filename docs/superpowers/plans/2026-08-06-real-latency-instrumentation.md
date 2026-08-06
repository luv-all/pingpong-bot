# 실기 파이프라인 레이턴시 계측 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `--mode real`에서 공 하나당 파이프라인 구간별(카메라 캡처→비전 적합→제어 처리
큐잉→명령 전송→모터 실행) 소요 시간을 구조화된 필드로 계측하고, 콘솔과는 별개인
JSON Lines 파일에 남긴다.

**Architecture:** 새 데이터 구조·상태 머신 변경 없음. `src/real/control_worker.rs`에
이미 존재하는 두 `info!` 로그 호출에 계산된 지연 필드를 더하고, 명령 발행 후
`hardware.is_busy()`가 꺼지는 시점을 관찰하는 작은 free function을 추가한다.
`src/telemetry/init_tracing.rs`는 `tracing_subscriber::registry()` 기반 2-레이어
구독자로 바뀌어, `target: "latency"` 이벤트만 골라 실행마다 새로 여는 JSON Lines
파일에도 같이 남긴다.

**Tech Stack:** Rust, `tracing`/`tracing-subscriber` (0.3, `env-filter` + 새 `json` +
`registry` feature). 새 crate 의존성 없음.

## Global Constraints

- 제어 로직·상태 머신·`CommitRequest`/`vision::Trajectory` 데이터 계약은 바꾸지
  않는다 — 계측은 전부 덧붙이는 형태여야 한다 (설계 문서 "범위" 절).
  스펙: `docs/superpowers/specs/2026-08-06-real-latency-instrumentation-design.md`
- 기존 하드웨어 I/O 호출(`hardware.command`, `hardware.command_joints`,
  `hardware.read_pose`) 빈도를 늘리지 않는다. `hardware.is_busy()`는 실물에서
  atomic bool 읽기라 버스 I/O가 아니므로 추가 호출은 허용된다(설계 문서 참고).
- 파일 계측이 실패해도(디렉터리/파일 생성 실패) 콘솔 로그와 실기 제어는 그대로
  동작해야 한다.
- 새 crate 의존성 추가 금지 — `tracing-subscriber`에 이미 있는 feature만 켠다.
- 로그 필드는 기존 관례대로 `super::fmt::{f2, f4}`로 소수점을 고정한 `String`으로
  남긴다(기존 `request_age_secs` 등과 동일한 스타일).
- 주석·로그 문자열은 한국어로, 기존 파일의 어조(간결, WHY 위주)를 따른다.
- `cargo build -p pingpong-bot`, `cargo test -p pingpong-bot --lib`가 각 태스크
  끝에 통과해야 한다.

---

## 파일 개요

- **Modify:** `src/real/control_worker.rs` — 새 계산 함수 2개(`camera_to_fit_ms`,
  `log_motion_done_if_idle`) + 기존 두 `info!` 호출에 필드 추가 + 루프 로컬 상태
  `motion_watch` 추가.
- **Modify:** `src/telemetry/init_tracing.rs` — `init_tracing`에 `real_mode: bool`
  인자 추가, 2-레이어 subscriber, `open_latency_file` 헬퍼.
- **Modify:** `src/main.rs` — `init_tracing` 호출 두 곳에 `real_mode` 인자 전달.
- **Modify:** `Cargo.toml` — `tracing-subscriber` feature에 `"json"`, `"registry"`
  추가.
- **Modify:** `.gitignore` — `logs/` 추가.

---

### Task 1: `camera_to_fit_ms` — 카메라 캡처→비전 적합 완료 지연 계산

**Files:**
- Modify: `src/real/control_worker.rs` (새 함수는 `refined_prediction_ready` 함수
  바로 뒤, `select_alignment_target` 함수 바로 앞 — 106번째 줄 부근에 삽입)
- Test: `src/real/control_worker.rs`의 기존 `mod tests`(1585번째 줄부터) 안에 추가

**Interfaces:**
- Consumes: `CommitRequest { trajectory: vision::Trajectory, at: Instant }`
  (기존 타입, 변경 없음). `vision::Trajectory { origin: Instant, measured: Track, .. }`,
  `Track: Deref<Target = [vision::State]>`, `vision::State { t: Duration, .. }`
  (모두 기존 타입, `src/vision/contract.rs`).
- Produces: `fn camera_to_fit_ms(request: &CommitRequest) -> f64` — 다음 태스크와
  Task 3에서 로그 필드로 그대로 쓴다.

- [ ] **Step 1: 실패하는 테스트 작성**

`src/real/control_worker.rs`의 `mod tests` 블록 안, 기존 `vision_request` 헬퍼
함수(1603번째 줄) 바로 뒤에 추가한다:

```rust
    #[test]
    fn camera_to_fit_ms_reflects_capture_to_fit_gap() {
        // vision_request(age)는 origin = now-1s, measured[0].t = 0.20s(캡처 시각
        // = now-0.8s), at = now-age로 CommitRequest를 만든다. 따라서
        // camera_to_fit_ms ≈ 800 - age(ms)다.
        let request = vision_request(Duration::from_millis(20));
        let ms = camera_to_fit_ms(&request);
        assert!((ms - 780.0).abs() < 50.0, "camera_to_fit_ms={ms}");
    }

    #[test]
    fn camera_to_fit_ms_defensive_zero_when_measured_empty() {
        let mut request = vision_request(Duration::from_millis(20));
        request.trajectory.measured = Track(vec![]);
        assert_eq!(camera_to_fit_ms(&request), 0.0);
    }
```

- [ ] **Step 2: 컴파일 실패 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::camera_to_fit_ms`

Expected: FAIL — `cannot find function 'camera_to_fit_ms' in this scope`.

- [ ] **Step 3: 최소 구현 작성**

`src/real/control_worker.rs`의 `refined_prediction_ready` 함수(97~106번째 줄)
바로 뒤, `select_alignment_target` 함수(108번째 줄) 바로 앞에 삽입:

```rust
/// 카메라 캡처(마지막 채택 관측) → 비전 적합 완료까지 걸린 시간 [ms].
///
/// `select_alignment_target`이 이미 `measured.last()`의 존재를 요구하므로 이
/// 함수가 실제로 호출되는 시점(정렬 목표를 이미 고른 뒤)에는 항상 `Some`이다 —
/// 방어적으로만 빈 궤적에 0.0을 반환한다.
fn camera_to_fit_ms(request: &CommitRequest) -> f64 {
    let Some(last) = request.trajectory.measured.last() else {
        return 0.0;
    };
    let capture_at = request.trajectory.origin + last.t;
    return request.at.saturating_duration_since(capture_at).as_secs_f64() * 1e3;
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::camera_to_fit_ms`

Expected: PASS (2 tests).

- [ ] **Step 5: 커밋**

```bash
git add src/real/control_worker.rs
git commit -m "feat(real): add camera_to_fit_ms latency helper"
```

---

### Task 2: `log_motion_done_if_idle` — 명령 발행 → 실행기 유휴 전환 관찰

**Files:**
- Modify: `src/real/control_worker.rs` (새 함수는 `log_verification` 함수(967번째
  줄 끝) 바로 뒤, `initialize_pose` 함수(970번째 줄) 바로 앞에 삽입)
- Test: 같은 파일의 `mod tests` 안, `PoseApplyingHardware` 정의(1640~1659번째 줄)
  바로 뒤에 새 테스트 더블 추가

**Interfaces:**
- Consumes: `Hardware` trait(`src/hardware/hardware.rs`) — `is_busy(&mut self) -> bool`.
- Produces: `fn log_motion_done_if_idle(hardware: &mut dyn Hardware, motion_watch:
  &mut Option<(u64, Instant, &'static str)>)`. Task 3에서 루프 로컬 상태
  `motion_watch`를 이 시그니처 그대로 넘긴다. 튜플은 `(track_seq, issued_at,
  event_label)`.

- [ ] **Step 1: 실패하는 테스트 작성**

`mod tests` 안, `PoseApplyingHardware`의 `impl Hardware` 블록(1656~1659번째 줄)
바로 뒤에 추가:

```rust
    struct ToggleBusyHardware {
        /// 첫 `is_busy()` 호출은 `true`, 이후 전부 `false` — 실행기가 한 번
        /// "바쁨"을 보고한 뒤 다음 틱에 유휴로 바뀌는 상황을 흉내낸다.
        busy_then_idle: std::cell::Cell<bool>,
    }

    impl Hardware for ToggleBusyHardware {
        fn command(
            &mut self,
            _trajectory: &pingpong_bot::robot::motion::Trajectory,
        ) -> Result<(), HwError> {
            return Ok(());
        }

        fn read_pose(&mut self) -> Result<Pose, HwError> {
            return Ok(Pose::new(0.0, Joints::from_slice(&[0.0; 4])));
        }

        fn is_busy(&mut self) -> bool {
            return self.busy_then_idle.replace(false);
        }
    }

    #[test]
    fn log_motion_done_if_idle_keeps_watch_while_busy_then_clears_when_idle() {
        let mut hardware = ToggleBusyHardware {
            busy_then_idle: std::cell::Cell::new(true),
        };
        let mut watch: Option<(u64, Instant, &'static str)> =
            Some((7, Instant::now(), "primary_alignment"));

        log_motion_done_if_idle(&mut hardware, &mut watch);
        assert!(watch.is_some(), "여전히 busy인 동안은 워치를 유지");

        log_motion_done_if_idle(&mut hardware, &mut watch);
        assert!(watch.is_none(), "busy가 풀리면 워치를 비움");
    }
```

- [ ] **Step 2: 컴파일 실패 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::log_motion_done_if_idle`

Expected: FAIL — `cannot find function 'log_motion_done_if_idle' in this scope`.

- [ ] **Step 3: 최소 구현 작성**

`src/real/control_worker.rs`의 `log_verification` 함수 끝(967번째 줄) 바로 뒤,
`initialize_pose` 함수 바로 앞에 삽입:

```rust
/// `motion_watch`가 있고 하드웨어가 더 이상 바쁘지 않으면 명령 발행부터 걸린
/// 시간을 한 번 로그하고 워치를 비운다.
///
/// `is_busy()`는 실물에서 atomic bool 읽기(`RealHardware::is_busy`,
/// `src/hardware/real.rs:305`)라 버스 I/O가 아니다 — 이 호출로 하드웨어 부하가
/// 늘지 않는다. 다만 이 값은 소프트웨어 실행기가 계획한 `duration_secs`가 지났다는
/// 뜻일 뿐 엔코더로 확인한 실제 도달은 아니다(설계 문서의 "비목표" 참고).
fn log_motion_done_if_idle(
    hardware: &mut dyn Hardware,
    motion_watch: &mut Option<(u64, Instant, &'static str)>,
) {
    let Some((track_seq, issued_at, event)) = *motion_watch else {
        return;
    };
    if hardware.is_busy() {
        return;
    }
    info!(
        target: "latency",
        track_seq,
        event,
        command_to_motion_done_ms = f2(issued_at.elapsed().as_secs_f64() * 1e3),
        "명령 실행기 유휴 전환 — 소프트웨어 추정 소요 시간(엔코더 확인 아님)"
    );
    *motion_watch = None;
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::log_motion_done_if_idle`

Expected: PASS.

- [ ] **Step 5: 커밋**

```bash
git add src/real/control_worker.rs
git commit -m "feat(real): add log_motion_done_if_idle latency watch"
```

---

### Task 3: 루프에 두 헬퍼 배선 + 기존 로그에 필드 추가

**Files:**
- Modify: `src/real/control_worker.rs` — `spawn()` 함수 본문 4곳:
  1. 로컬 상태 선언부(259~269번째 줄)
  2. `verify_due_command` 호출 직후(335~355번째 줄)
  3. 고정 스윙 명령 블록(383~414번째 줄)
  4. 본 예측 정렬/팔 보정 명령 블록(679~805번째 줄)

**Interfaces:**
- Consumes: Task 1의 `camera_to_fit_ms(&CommitRequest) -> f64`, Task 2의
  `log_motion_done_if_idle(&mut dyn Hardware, &mut Option<(u64, Instant,
  &'static str)>)`.
- Produces: 없음(로그 필드 배선 — 이후 태스크가 참조하는 새 타입/함수 없음).

이 태스크는 순수 배선이라 새 단위 테스트를 추가하지 않는다. 기존 테스트 스위트가
회귀 없음을 보장한다(Step 마지막에 전체 `--lib` 테스트 실행).

- [ ] **Step 1: 로컬 상태에 `motion_watch` 추가**

`src/real/control_worker.rs:259-261`, 다음처럼 한 줄 추가:

```rust
        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        // Task 2의 log_motion_done_if_idle이 채우고 비운다 — (track_seq,
        // 명령 발행 시각, 이벤트 라벨).
        let mut motion_watch: Option<(u64, Instant, &'static str)> = None;
        let mut pending_verification: Option<PendingVerification> = None;
```

- [ ] **Step 2: 매 루프 틱마다 유휴 전환을 관찰**

`src/real/control_worker.rs:335-355`(`verify_due_command` 매치 블록) 바로 뒤,
`let due_swing = ...`(356번째 줄) 바로 앞에 한 줄 추가:

```rust
                VerificationResult::Pending => {}
            }
            log_motion_done_if_idle(hardware.as_mut(), &mut motion_watch);
            let due_swing = match &state {
```

- [ ] **Step 3: 고정 스윙 명령에 전송 시간 계측 + 워치 등록**

`src/real/control_worker.rs:387-414`(`let swing = &planned.trajectory;`부터
`Err(error) => warn!(...)` 분기 끝까지)을 다음으로 교체한다(변경 부분은
`command_send_started`/`command_send_ms` 추가, `motion_watch` 등록,
`info!`에 `target: "latency"`와 `command_send_ms` 필드 추가):

```rust
                                let swing = &planned.trajectory;
                                let command_send_started = Instant::now();
                                let command_result = hardware.command_joints(swing);
                                let command_send_ms =
                                    command_send_started.elapsed().as_secs_f64() * 1e3;
                                match command_result {
                                    Ok(()) => {
                                        if let BallControlState::Aligning { measurement, .. } =
                                            &mut state
                                        {
                                            measurement.rail_commanded_m = swing_start.rail_x;
                                            measurement.joints_commanded =
                                                swing.follow_through.clone();
                                        }
                                        motion_watch =
                                            Some((track_seq, command_send_started, "fixed_swing"));
                                        info!(
                                            target: "latency",
                                            track_seq,
                                            scheduled_lead_secs = FIXED_SWING_LEAD.as_secs_f64(),
                                            start_late_ms = f2(swing_due_at.elapsed().as_secs_f64() * 1e3),
                                            command_send_ms = f2(command_send_ms),
                                            swing_duration_secs = f4(swing.duration_secs),
                                            joints_start = %format!("{:?}", swing.start.values),
                                            joints_impact = %format!("{:?}", swing.end.values),
                                            joints_follow_through = %format!("{:?}", swing.follow_through.values),
                                            skipped_joint_indices = ?planned.skipped_joint_indices,
                                            "백스윙 없는 고정 관절 스윙 시작"
                                        );
                                    }
                                    Err(error) => warn!(
                                        track_seq,
                                        %error,
                                        "고정 스윙 명령 실패 — 스윙만 생략"
                                    ),
                                }
```

원래 코드는 `match hardware.command_joints(swing) { Ok(()) => { ... } Err(error)
=> ... }` 형태였다 — `match hardware.command_joints(swing)` 한 줄을
`command_send_started`/`command_result`/`command_send_ms` 세 줄로 바꾸고
`match hardware.command_joints(swing)`를 `match command_result`로 바꾼 것 외의
분기 로직은 그대로다.

- [ ] **Step 4: 본 예측 정렬/팔 보정 명령에 전송 시간 계측 + 워치 등록**

`src/real/control_worker.rs:726-736`을 다음으로 교체:

```rust
            let command_send_started = Instant::now();
            let command_result = match action {
                RefinedAction::PrimaryRailAndArm => hardware.command(&alignment),
                RefinedAction::ArmCorrection => hardware.command_joints(&alignment),
            };
            let command_send_ms = command_send_started.elapsed().as_secs_f64() * 1e3;
            if let Err(error) = command_result {
                let _ = event_tx.send(RuntimeEvent::Failed {
                    track_seq: Some(track_seq),
                    reason: format!("위치·방향 정렬 명령 실패: {error}"),
                });
                break;
            }
            motion_watch = Some((
                track_seq,
                command_send_started,
                match action {
                    RefinedAction::PrimaryRailAndArm => "primary_alignment",
                    RefinedAction::ArmCorrection => "arm_correction",
                },
            ));
```

(`motion_watch`의 발행 시각은 `command_send_started`를 쓴다 — `issued_at`
(679번째 줄)은 정렬 계획 계산이 시작되기 전 시각이라 IK·궤적 계획 시간까지
섞인다. `command_send_started`가 "하드웨어에 명령을 보내기 시작한 시각"에
더 가깝다 — 고정 스윙 블록(Step 3)과 같은 기준.)

- [ ] **Step 5: 기존 `info!`에 계측 필드 추가**

`src/real/control_worker.rs:780-805`의 `info!` 호출에서 `track_seq,` 바로 뒤에
`target: "latency",`를 추가하고, `request_age_secs = f4(request.age_secs()),`
바로 뒤에 두 줄을 추가한다:

```rust
            info!(
                target: "latency",
                track_seq,
                stage = ?PredictionStage::Refined,
                start_pose_source,
                request_age_secs = f4(request.age_secs()),
                camera_to_fit_ms = f2(camera_to_fit_ms(&request)),
                command_send_ms = f2(command_send_ms),
                target_time_secs = f4(target.t.as_secs_f64()),
```

(나머지 필드는 그대로 둔다.)

- [ ] **Step 6: 빌드 + 전체 테스트 확인**

Run: `cargo build -p pingpong-bot`

Expected: 경고 없이 성공.

Run: `cargo test -p pingpong-bot --lib`

Expected: 기존 통과 테스트 수 유지 + Task 1·2에서 추가한 3개 테스트 포함 전부 PASS.
(2026-08-01 기준 기존 스위트는 260 통과, 46 ignored, Dynamixel 매핑 테스트 1개
실패 이력이 있었다 — `docs/TODO.md` §1.4. 이 기존 실패가 그대로 남아 있다면
이번 변경과 무관하니 통과 개수만 "이전 + 4"인지 확인한다.)

- [ ] **Step 7: 커밋**

```bash
git add src/real/control_worker.rs
git commit -m "feat(real): wire latency fields into alignment and swing logs"
```

---

### Task 4: 전용 JSON Lines 파일로 `target: "latency"` 이벤트 미러링

**Files:**
- Modify: `Cargo.toml`
- Modify: `src/telemetry/init_tracing.rs`
- Modify: `src/main.rs`
- Modify: `.gitignore`
- Test: `src/telemetry/init_tracing.rs`에 `#[cfg(test)] mod tests` 추가(새 모듈)

**Interfaces:**
- Consumes: 없음(다른 태스크의 함수를 쓰지 않는다 — Task 1~3이 만든 `target:
  "latency"` 이벤트를 이 태스크가 만드는 필터가 받아간다).
- Produces: `pub fn init_tracing(debug: bool, debug_crates: &[&str], real_mode:
  bool)` — 시그니처가 기존 `init_tracing(debug, debug_crates)`에서 인자 하나
  늘어난다. 호출부는 `src/main.rs` 두 곳.

- [ ] **Step 1: `Cargo.toml`에 feature 추가**

`Cargo.toml:56`을 다음으로 교체:

```toml
tracing-subscriber = { version = "0.3.23", features = ["env-filter", "json", "registry"] }
```

Run: `cargo check -p pingpong-bot`

Expected: `Cargo.lock`이 갱신되며 성공(`serde_json`/`tracing-serde`가 새로
받아진다). feature 이름이 잘못됐다면 여기서 명확한 에러가 난다.

- [ ] **Step 2: 실패하는 테스트 작성**

`src/telemetry/init_tracing.rs` 끝에 추가:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_latency_file_creates_directory_and_unique_jsonl() {
        let base = std::env::temp_dir().join(format!(
            "pingpong_latency_test_{}",
            std::process::id()
        ));
        let base_str = base.to_str().expect("temp 경로는 유효한 UTF-8").to_owned();
        let _ = std::fs::remove_dir_all(&base);

        let file = open_latency_file(&base_str);
        assert!(file.is_some(), "파일을 열 수 있어야 한다");
        drop(file);

        let entries: Vec<_> = std::fs::read_dir(&base)
            .expect("디렉터리가 생성돼 있어야 한다")
            .filter_map(|entry| entry.ok())
            .collect();
        assert_eq!(entries.len(), 1, "파일이 정확히 하나 생성돼야 한다");
        let name = entries[0].file_name();
        let name = name.to_string_lossy();
        assert!(
            name.starts_with("latency-") && name.ends_with(".jsonl"),
            "예상치 못한 파일명: {name}"
        );

        std::fs::remove_dir_all(&base).expect("정리");
    }
}
```

- [ ] **Step 3: 컴파일 실패 확인**

Run: `cargo test -p pingpong-bot --lib telemetry::init_tracing`

Expected: FAIL — `cannot find function 'open_latency_file' in this scope`.

- [ ] **Step 4: `init_tracing.rs` 재작성**

`src/telemetry/init_tracing.rs` 전체를 다음으로 교체(맨 위 doc 주석부터
기존 `pub fn init_tracing` 끝까지 — 이후 Step 2에서 추가한 `mod tests`는
그대로 파일 끝에 남긴다):

```rust
//! CLI 바이너리용 tracing subscriber 초기화.
//!
//! Windows PowerShell 등에서 `RUST_LOG=… cargo …` 문법이 깨지므로
//! `--debug` 플래그만 쓴다.
//!
//! `real_mode`가 `true`면 `target: "latency"` 이벤트를 콘솔과 별개로
//! `logs/latency-<유닉스 초>.jsonl`에도 JSON Lines로 남긴다 — 실기 파이프라인
//! 구간별 소요 시간 진단용(`docs/superpowers/specs/2026-08-06-real-latency-instrumentation-design.md`).
//! 파일을 열지 못해도 콘솔 로그·실기 제어는 그대로 동작한다.

use std::fs::{self, File};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use tracing_subscriber::EnvFilter;
use tracing_subscriber::filter::Targets;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

/// tracing subscriber를 한 번 초기화한다.
///
/// - `debug == true` → `debug_crates`를 `=debug`로
/// - 아니면 기본 `info`
/// - `real_mode == true` → `target: "latency"` 이벤트를 파일 레이어로도 미러링
pub fn init_tracing(debug: bool, debug_crates: &[&str], real_mode: bool) {
    let filter = if debug {
        let directives = debug_crates
            .iter()
            .map(|name| format!("{name}=debug"))
            .collect::<Vec<_>>()
            .join(",");
        EnvFilter::new(if directives.is_empty() {
            "debug".to_owned()
        } else {
            directives
        })
    } else {
        EnvFilter::new("info")
    };
    let stdout_layer = tracing_subscriber::fmt::layer().with_filter(filter);

    let latency_layer = if real_mode {
        open_latency_file("logs").map(|file| {
            tracing_subscriber::fmt::layer()
                .json()
                .with_writer(file)
                .with_filter(Targets::new().with_target("latency", tracing::Level::INFO))
        })
    } else {
        None
    };

    tracing_subscriber::registry()
        .with(stdout_layer)
        .with(latency_layer)
        .init();
}

/// `<base_dir>/latency-<유닉스 초>.jsonl`을 새로 연다. 실패하면 `eprintln!`으로
/// 한 번만 경고하고 `None`을 돌려준다 — 이 시점엔 아직 tracing subscriber가 없어
/// 로그 매크로를 쓸 수 없고, 계측 실패가 실기 제어를 막아서도 안 된다.
fn open_latency_file(base_dir: &str) -> Option<Arc<File>> {
    if let Err(error) = fs::create_dir_all(base_dir) {
        eprintln!("경고: {base_dir} 디렉터리 생성 실패 — 레이턴시 파일 로그 없이 계속: {error}");
        return None;
    }
    let unix_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_secs());
    let path = format!("{base_dir}/latency-{unix_secs}.jsonl");
    return match File::create(&path) {
        Ok(file) => {
            println!("레이턴시 진단 로그: {path}");
            Some(Arc::new(file))
        }
        Err(error) => {
            eprintln!("경고: 레이턴시 로그 파일 생성 실패({path}) — 파일 로그 없이 계속: {error}");
            None
        }
    };
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot --lib telemetry::init_tracing`

Expected: PASS.

- [ ] **Step 6: `main.rs` 호출부 갱신**

`src/main.rs:37-41`(sim-child 분기)을 다음으로 교체:

```rust
    #[cfg(feature = "real")]
    if std::env::args().any(|arg| arg == real::SIM_CHILD_FLAG) {
        init_tracing(false, &["pingpong_bot"], false);
        return real::run_sim_child();
    }
```

`src/main.rs:43-44`를 다음으로 교체:

```rust
    let args = Args::parse();
    init_tracing(args.debug, &["pingpong_bot"], matches!(args.mode, ModeArg::Real));
```

(sim-child는 관전용 뷰어 자식 프로세스일 뿐 카메라·하드웨어 파이프라인을 돌리지
않으므로 `real_mode = false`다 — `src/real/sim_child.rs` 참고.)

- [ ] **Step 7: `.gitignore`에 `logs/` 추가**

`.gitignore`에 다음 줄을 `.worktrees/` 근처에 추가:

```
logs/
```

- [ ] **Step 8: 전체 빌드 확인**

Run: `cargo build -p pingpong-bot`

Expected: 경고 없이 성공.

Run: `cargo test -p pingpong-bot --lib`

Expected: 전체 통과(Task 1·2·4에서 추가한 4개 테스트 포함).

- [ ] **Step 9: 커밋**

```bash
git add Cargo.toml Cargo.lock src/telemetry/init_tracing.rs src/main.rs .gitignore
git commit -m "feat(telemetry): mirror latency events to a dedicated JSON Lines file"
```

- [ ] **Step 10: (하드웨어가 연결된 실기에서만) 수동 확인**

카메라·로봇이 연결된 실기 PC에서:

```bash
cargo run -p pingpong-bot -- --mode real --dry-run
```

몇 초 뒤 `logs/latency-<유닉스초>.jsonl`이 생성됐는지, 공을 하나 던져 정렬
명령이 나가면 그 파일에 `camera_to_fit_ms`, `command_send_ms`,
`request_age_secs` 필드가 있는 JSON 한 줄이 남는지 확인한다. 이 저장소가
있는 환경에 하드웨어가 없다면 이 단계는 건너뛰고 사용자에게 실기에서 확인해
달라고 알린다 — Step 8의 자동 테스트로 컴파일·단위 동작은 이미 검증됐다.

---

## 완료 후 남는 일 (이 플랜 범위 밖)

- 이 계측치를 근거로 한 레이턴시 보상 설계는 별도 spec으로 진행한다
  (`docs/superpowers/specs/2026-08-06-real-latency-instrumentation-design.md`
  "후속 작업" 절 참고).
