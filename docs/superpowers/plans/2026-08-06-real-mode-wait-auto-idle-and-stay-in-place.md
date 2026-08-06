# `--mode real` 수동 테스트 — 대기 3초 자동 idle · 스윙 후 제자리 유지 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 스윙(정렬→고정 스윙→중립 복귀) 뒤 자동으로 들어가는 `Waiting` 상태가 3초
뒤에는 운영자가 `n`을 누르지 않아도 자동으로 `Idle`(공을 받는 상태)로 돌아가게
하고, 그 복귀 동작에서 레일은 공을 친 위치에 그대로 두고 관절만 준비 자세로
되돌린다.

**Architecture:** 새 상태 머신 variant 추가 없음. `src/real/control_worker.rs`의
`spawn()` while 루프에 로컬 타이머(`Option<Instant>`) 하나를 추가해 "자동으로
들어간 Waiting"과 "`w`로 수동으로 들어간 Waiting"을 구분하고, 만료되면 기존
`ControlState::Idle` 이벤트를 그대로 재사용해 조용히 전이한다(하드웨어 이동 없음).
레일 정지 위치는 스윙 완료 후 이미 실행하던 `hardware.read_pose()` 실측값을
`move_to_ready`의 목표 레일 x로 재사용해, 홈 위치 대신 방금 있던 자리로 복귀
시킨다.

**Tech Stack:** Rust, `std::time::Instant`/`Duration`, 기존 `crossbeam-channel`
이벤트 배선. 새 의존성 없음.

## Global Constraints

- 범위는 `src/real/control_worker.rs` 하나다. `test_control.rs`의 `TestControl`/
  `TestZone` 타입, `run.rs`의 키 라우팅, `preview.rs`, `runtime_event.rs`는
  건드리지 않는다 (스펙: `docs/superpowers/specs/2026-08-06-real-mode-wait-auto-idle-and-stay-in-place-design.md`).
- `r`(ResetPosition), 수동 `w`(Wait), 존 변경(`1`/`2`/`3`/`4`)은 지금처럼
  `home_rail_x`로 완전 복귀한다 — "제자리 유지"는 스윙 직후 **자동** 복귀
  단계에만 적용된다.
- `BallControlState`, `ControlStateSnapshot`, `TestControl` 타입 자체는 바꾸지
  않는다. 외부에서 관찰되는 이벤트 시퀀스(`Waiting` → `Idle`)는 기존과 동일하게
  유지한다.
- `apply_test_control`/`apply_immediate_control` 함수 시그니처와 내부 로직은
  변경하지 않는다.
- 주석·로그 문자열은 한국어로, 기존 파일의 어조(간결, WHY 위주)를 따른다.
- 이 파일은 `main.rs` 바이너리 타깃(`required-features = ["gui"]`, 기본
  features에 이미 `gui`/`real` 포함)의 일부다 — 테스트는
  `cargo test -p pingpong-bot --bin pingpong-bot <filter>`로 실행한다
  (`--lib`가 아니다).
- 각 태스크 끝에 `cargo build -p pingpong-bot`과 해당 태스크의
  `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::<filter>`가
  통과해야 한다.

---

## 파일 개요

- **Modify:** `src/real/control_worker.rs`
  - 새 상수 `AUTO_IDLE_AFTER_WAIT`
  - `spawn()` 루프 로컬 변수 `waiting_auto_resume_at: Option<Instant>`
  - 새 free function `resume_waiting_in_place`
  - `spawn()` 루프의 3곳 수정: 대기 진입 시 타이머 설정(스윙 후 자동 진입에만),
    `w`/`n` 처리, 주기 만료 점검
  - 스윙 후 복귀 분기에서 `move_to_ready`에 넘기는 레일 x를 `home_rail_x`
    대신 실측값으로 교체
  - `mod tests`에 테스트 헬퍼 `wait_for_event_value`, mock `SharedPoseHardware`,
    신규 테스트 2개 추가

---

### Task 1: 대기 3초 경과 시 자동으로 idle 전환 (수동 `w`는 예외)

**Files:**
- Modify: `src/real/control_worker.rs`
  - 상수 블록: 32번째 줄(`const RECV_TIMEOUT`) 부근
  - 루프 로컬 선언: 284~286번째 줄 부근
  - `spawn()` 루프 상단 `test_control_rx` 드레인: 289~351번째 줄
  - `spawn()` 루프 본문: 373번째 줄(`log_motion_done_if_idle` 호출) 바로 뒤
  - `spawn()` 루프 본문: 560~566번째 줄(`state = BallControlState::Waiting;` 진입부)
  - 새 함수 `resume_waiting_in_place`: `apply_immediate_control` 함수(1455~1505번째
    줄) 바로 뒤에 삽입
- Test: `src/real/control_worker.rs`의 `mod tests`(1650번째 줄부터) 안, 기존
  `spawn_ignores_balls_while_waiting_and_resumes_after_next` 테스트(2216번째
  줄부터) 바로 뒤

**Interfaces:**
- Consumes: 기존 `BallControlState`, `ControlStateSnapshot`, `RuntimeEvent`,
  `Hardware` 트레이트(모두 변경 없음). `apply_immediate_control`(기존 시그니처
  그대로, 이 태스크에서 변경하지 않음).
- Produces: `const AUTO_IDLE_AFTER_WAIT: Duration` — Task 2는 이 상수를 쓰지
  않는다(Task 2는 완전히 별개 지점을 수정). `fn resume_waiting_in_place(hardware:
  &mut dyn Hardware, state: &mut BallControlState, cached_idle_pose: &mut
  Option<pingpong_bot::robot::Pose>, event_tx: &Sender<RuntimeEvent>)` — 이
  태스크 안에서만 3곳(스윙 후 자동 대기 만료 점검, `n` 처리)에서 호출된다.

- [ ] **Step 1: 실패하는 테스트 작성**

`src/real/control_worker.rs`의 `mod tests` 블록 안, 기존
`spawn_ignores_balls_while_waiting_and_resumes_after_next` 테스트(2299번째 줄,
닫는 `}` 다음) 바로 뒤에 추가한다:

```rust
    #[test]
    fn spawn_auto_resumes_from_waiting_after_three_seconds_without_next() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let hardware: Box<dyn Hardware> = Box::new(ReadCountingHardware {
            reads: 0,
            pose: Pose::new(rail.default_x(), robot.arm.default_joints.clone()),
        });

        let (commit_tx, commit_rx) = crossbeam_channel::unbounded();
        let (_test_control_tx, test_control_rx) = crossbeam_channel::unbounded();
        let (event_tx, event_rx) = crossbeam_channel::unbounded();
        let (guard, shutdown) = crate::real::shutdown_channel();

        let handle = spawn(
            hardware,
            Arc::clone(&robot.arm),
            commit_rx,
            test_control_rx,
            None,
            event_tx,
            shutdown,
        );

        let generous = Duration::from_secs(3);

        commit_tx
            .send(vision_request(Duration::ZERO))
            .expect("보낼 수 있음");
        assert!(
            wait_for_event(&event_rx, generous, |event| matches!(
                event,
                RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Waiting
                }
            )),
            "스윙 완료 후 대기 상태로 들어가야 한다"
        );

        assert!(
            wait_for_event(
                &event_rx,
                AUTO_IDLE_AFTER_WAIT + Duration::from_secs(1),
                |event| matches!(
                    event,
                    RuntimeEvent::ControlState {
                        state: ControlStateSnapshot::Idle
                    }
                )
            ),
            "n을 누르지 않아도 3초 뒤 자동으로 idle로 돌아와야 한다"
        );

        drop(guard);
        handle.join().expect("워커 스레드가 정상 종료해야 한다");
    }
```

이 테스트는 `AUTO_IDLE_AFTER_WAIT`가 아직 없어 컴파일이 안 된다 — 다음 스텝에서
확인한다.

- [ ] **Step 2: 컴파일 실패 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::spawn_auto_resumes_from_waiting_after_three_seconds_without_next`

Expected: FAIL — `cannot find value 'AUTO_IDLE_AFTER_WAIT' in this scope`.

- [ ] **Step 3: 상수·루프 로컬 상태 추가**

`src/real/control_worker.rs`의 상수 블록, `const RECV_TIMEOUT: Duration =
Duration::from_millis(100);`(32번째 줄) 바로 뒤에 추가:

```rust
const AUTO_IDLE_AFTER_WAIT: Duration = Duration::from_secs(3);
```

`spawn()` 안, 기존 루프 로컬 변수 선언부 — `let mut last_waiting_ignored_track_seq:
Option<u64> = None;`(286번째 줄) 바로 뒤에 추가:

```rust
        // 스윙 후 자동으로 들어간 Waiting에만 설정된다 — `w`로 수동 진입한
        // Waiting은 이 타이머 없이 계속 `n`을 기다린다.
        let mut waiting_auto_resume_at: Option<Instant> = None;
```

- [ ] **Step 4: `resume_waiting_in_place` 헬퍼 추가**

`apply_immediate_control` 함수(1455번째 줄에서 시작, 1505번째 줄 `}`로 끝)
바로 뒤에 추가:

```rust
/// 스윙 후 자동 대기가 3초 만료됐거나 `n`으로 건너뛴 경우 적용한다 — 하드웨어를
/// 다시 움직이지 않고 idle로만 전환한다. 레일·관절은 스윙 직후 복귀에서 이미
/// 준비 자세로 들어와 있다(Task 2).
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

- [ ] **Step 5: 스윙 후 Waiting 진입 시 타이머 설정**

`spawn()` 루프 본문에서 스윙 후 복귀가 성공한 뒤 `Waiting`으로 전이하는 부분
(560~566번째 줄):

```rust
                // 스윙(정렬→유지→중립 복귀)이 정상적으로 끝났다 — 운영자가 결과를
                // 확인하고 `n`을 누를 때까지 새 공을 받지 않는다.
                state = BallControlState::Waiting;
                pending_refined = None;
                let _ = event_tx.send(RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Waiting,
                });
```

를 아래로 교체:

```rust
                // 스윙(정렬→유지→중립 복귀)이 정상적으로 끝났다 — 3초 안에 `n`을
                // 누르면 곧바로, 안 눌러도 3초 뒤 자동으로 다음 공을 받는다.
                state = BallControlState::Waiting;
                waiting_auto_resume_at = Some(Instant::now() + AUTO_IDLE_AFTER_WAIT);
                pending_refined = None;
                let _ = event_tx.send(RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Waiting,
                });
```

- [ ] **Step 6: 주기 만료 점검 추가**

`log_motion_done_if_idle(hardware.as_mut(), &mut motion_watch);`(373번째 줄)
바로 뒤에 추가:

```rust
            if matches!(state, BallControlState::Waiting)
                && waiting_auto_resume_at.is_some_and(|deadline| Instant::now() >= deadline)
            {
                waiting_auto_resume_at = None;
                resume_waiting_in_place(hardware.as_mut(), &mut state, &mut cached_idle_pose, &event_tx);
                info!("대기 3초 경과 — 자동으로 idle 전환 (n 불필요)");
            }
```

- [ ] **Step 7: `w`/`n` 처리에 타이머 배선**

`spawn()` 루프 상단 `test_control_rx` 드레인 블록(289~351번째 줄) 전체를 아래로
교체:

```rust
            while let Ok(control) = test_control_rx.try_recv() {
                match control {
                    TestControl::ResetPosition | TestControl::Wait => {
                        pending_test_control = None;
                        waiting_auto_resume_at = None;
                        if hardware.is_busy() {
                            hardware.cancel();
                            while hardware.is_busy() && !shutdown.is_down() {
                                thread::sleep(BUSY_POLL);
                            }
                        }
                        if shutdown.is_down() {
                            break 'control;
                        }
                        pending_verification = None;
                        pending_refined = None;
                        consecutive_misses = 0;
                        if apply_immediate_control(
                            control,
                            hardware.as_mut(),
                            &arm,
                            &mut home_rail_x,
                            &mut current_zone,
                            &mut zone_filter,
                            &mut latch,
                            &mut state,
                            sim_tx.as_ref(),
                            &event_tx,
                            &mut cached_idle_pose,
                        )
                        .is_break()
                        {
                            break 'control;
                        }
                    }
                    TestControl::Next => {
                        if matches!(state, BallControlState::Waiting) {
                            pending_verification = None;
                            pending_refined = None;
                            consecutive_misses = 0;
                            if waiting_auto_resume_at.take().is_some() {
                                resume_waiting_in_place(
                                    hardware.as_mut(),
                                    &mut state,
                                    &mut cached_idle_pose,
                                    &event_tx,
                                );
                            } else if apply_immediate_control(
                                TestControl::Next,
                                hardware.as_mut(),
                                &arm,
                                &mut home_rail_x,
                                &mut current_zone,
                                &mut zone_filter,
                                &mut latch,
                                &mut state,
                                sim_tx.as_ref(),
                                &event_tx,
                                &mut cached_idle_pose,
                            )
                            .is_break()
                            {
                                break 'control;
                            }
                        } else {
                            debug!("대기 상태가 아닐 때 'n' 입력 — 무시");
                        }
                    }
                    other => pending_test_control = Some(other),
                }
            }
```

(바뀐 부분은 `ResetPosition | Wait` 팔의 `waiting_auto_resume_at = None;` 한 줄과
`Next` 팔 안의 `if waiting_auto_resume_at.take().is_some() { ... } else if
apply_immediate_control(...) ...` 분기뿐이다 — 나머지는 그대로.)

- [ ] **Step 8: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::spawn_auto_resumes_from_waiting_after_three_seconds_without_next`

Expected: PASS. (실행 시간이 약 3초 이상 걸린다 — 실제 타이머를 기다리는
테스트이기 때문에 정상이다.)

- [ ] **Step 9: 기존 회귀 테스트 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::spawn_ignores_balls_while_waiting_and_resumes_after_next`

Expected: PASS — `n`을 3초 안에 누르는 기존 시나리오는
`waiting_auto_resume_at.take().is_some()` 경로로 여전히 `Idle`로 전이하고, 세
번째 공도 다시 명령된다.

- [ ] **Step 10: 전체 빌드 확인**

Run: `cargo build -p pingpong-bot`

Expected: 경고 없이 컴파일 성공.

- [ ] **Step 11: 커밋**

```bash
git add src/real/control_worker.rs
git commit -m "feat(real): auto-idle 3s after post-swing wait without pressing n"
```

---

### Task 2: 스윙 후 복귀 — 레일은 친 위치에, 관절만 준비 자세로

**Files:**
- Modify: `src/real/control_worker.rs`
  - `spawn()` 루프의 `due_for_return` 분기(498~524번째 줄 부근,
    `move_to_ready(hardware.as_mut(), &arm, home_rail_x)` 호출 포함)
- Test: `src/real/control_worker.rs`의 `mod tests` 안, Task 1에서 추가한
  `spawn_auto_resumes_from_waiting_after_three_seconds_without_next` 테스트
  바로 뒤

**Interfaces:**
- Consumes: 기존 `move_to_ready(hardware: &mut dyn Hardware, arm: &Arm, rail_x:
  f64) -> Result<(), MoveError>`(시그니처 변경 없음 — 이 태스크는 호출 시점의
  인자 값만 바꾼다). 기존 `PoseApplyingHardware` mock 패턴을 참고해 새 mock을
  만든다.
- Produces: 새 테스트 mock `struct SharedPoseHardware { pose:
  Arc<std::sync::Mutex<Pose>> }`와 테스트 헬퍼 `fn wait_for_event_value<T>(...)
  -> Option<T>` — 이후 다른 태스크에서 재사용할 계획 없음(이 태스크 전용).

- [ ] **Step 1: 실패하는 테스트 작성**

`src/real/control_worker.rs`의 `mod tests` 블록 안, 기존 `wait_for_event`
함수(2197~2214번째 줄) 바로 뒤에 헬퍼를 추가한다:

```rust
    /// `wait_for_event`처럼 이벤트를 소진하지만, `extract`가 `Some`을 반환하는
    /// 첫 값을 돌려준다. `timeout` 안에 못 찾으면(채널 disconnect 포함) `None`.
    fn wait_for_event_value<T>(
        event_rx: &crossbeam_channel::Receiver<RuntimeEvent>,
        timeout: Duration,
        mut extract: impl FnMut(&RuntimeEvent) -> Option<T>,
    ) -> Option<T> {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return None;
            }
            match event_rx.recv_timeout(remaining) {
                Ok(event) => {
                    if let Some(value) = extract(&event) {
                        return Some(value);
                    }
                }
                Err(_) => return None,
            }
        }
    }
```

Task 1에서 추가한 `spawn_auto_resumes_from_waiting_after_three_seconds_without_next`
테스트 바로 뒤(파일 맨 끝, `mod tests`를 닫는 `}` 바로 앞)에 mock과 테스트를
추가한다:

```rust
    /// 여러 스레드에서 최신 포즈를 관찰할 수 있는 `PoseApplyingHardware`의
    /// 공유 버전 — 테스트가 워커 스레드 종료 없이도 최종 레일 위치를 읽는다.
    struct SharedPoseHardware {
        pose: Arc<std::sync::Mutex<Pose>>,
    }

    impl Hardware for SharedPoseHardware {
        fn command(
            &mut self,
            trajectory: &pingpong_bot::robot::motion::Trajectory,
        ) -> Result<(), HwError> {
            *self.pose.lock().expect("lock") = Pose::new(
                trajectory.follow_through_rail_x,
                trajectory.end_joints().clone(),
            );
            return Ok(());
        }

        fn read_pose(&mut self) -> Result<Pose, HwError> {
            return Ok(self.pose.lock().expect("lock").clone());
        }
    }

    #[test]
    fn spawn_keeps_rail_at_swing_position_after_return_to_ready() {
        let robot = pingpong_bot::defaults::robot().expect("robot");
        let rail = robot.arm.rail.expect("rail 있는 로봇");
        let shared_pose = Arc::new(std::sync::Mutex::new(Pose::new(
            rail.default_x(),
            robot.arm.default_joints.clone(),
        )));
        let hardware: Box<dyn Hardware> = Box::new(SharedPoseHardware {
            pose: Arc::clone(&shared_pose),
        });

        let (commit_tx, commit_rx) = crossbeam_channel::unbounded();
        let (_test_control_tx, test_control_rx) = crossbeam_channel::unbounded();
        let (event_tx, event_rx) = crossbeam_channel::unbounded();
        let (guard, shutdown) = crate::real::shutdown_channel();

        let handle = spawn(
            hardware,
            Arc::clone(&robot.arm),
            commit_rx,
            test_control_rx,
            None,
            event_tx,
            shutdown,
        );

        let generous = Duration::from_secs(3);

        commit_tx
            .send(vision_request(Duration::ZERO))
            .expect("보낼 수 있음");

        let commanded_rail_x = wait_for_event_value(&event_rx, generous, |event| match event {
            RuntimeEvent::Commanded { rail_x, .. } => Some(*rail_x),
            _ => None,
        })
        .expect("정렬 명령이 와야 한다");

        assert!(
            wait_for_event(&event_rx, generous, |event| matches!(
                event,
                RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Waiting
                }
            )),
            "스윙 완료 후 대기 상태로 들어가야 한다"
        );

        let final_rail_x = shared_pose.lock().expect("lock").rail_x;
        assert!(
            (final_rail_x - commanded_rail_x).abs() < 1e-6,
            "복귀 후 레일은 정렬 위치({commanded_rail_x})에 그대로 있어야 하는데 {final_rail_x}"
        );
        assert!(
            (final_rail_x - rail.default_x()).abs() > 1e-3,
            "정렬 위치가 홈과 달라야 검증 의미가 있다: final_rail_x={final_rail_x} default_x={}",
            rail.default_x()
        );

        drop(guard);
        handle.join().expect("워커 스레드가 정상 종료해야 한다");
    }
```

- [ ] **Step 2: 실패 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::spawn_keeps_rail_at_swing_position_after_return_to_ready`

Expected: FAIL — `final_rail_x`가 `commanded_rail_x`가 아니라 `rail.default_x()`
근처라 첫 번째 assert가 깨진다(현재 코드는 `move_to_ready`를 `home_rail_x`로
부르므로).

- [ ] **Step 3: 최소 구현 — 실측 레일 x를 복귀 목표로 재사용**

`spawn()` 루프의 `due_for_return` 분기(498~524번째 줄):

```rust
            } else if idle_ready && due_for_return {
                if let BallControlState::Aligning { measurement, .. } = &state {
                    match hardware.read_pose() {
                        Ok(measured) => {
                            let joint_errors: Vec<f64> = measurement
                                .joints_commanded
                                .values
                                .iter()
                                .zip(&measured.joints.values)
                                .map(|(commanded, measured)| commanded - measured)
                                .collect();
                            info!(
                                track_seq = measurement.track_seq,
                                rail_commanded_m = f4(measurement.rail_commanded_m),
                                rail_measured_m = f4(measured.rail_x),
                                rail_commanded_minus_measured_m =
                                    f4(measurement.rail_commanded_m - measured.rail_x),
                                joints_commanded = %format!("{:?}", measurement.joints_commanded.values),
                                joints_measured = %format!("{:?}", measured.joints.values),
                                joints_commanded_minus_measured = %format!("{joint_errors:?}"),
                                "공 위치·방향 정렬 완료 후 실측"
                            );
                        }
                        Err(error) => warn!(%error, "공 위치·방향 정렬 완료 후 포즈 읽기 실패"),
                    }
                }
                if let Err(error) = move_to_ready(hardware.as_mut(), &arm, home_rail_x) {
```

를 아래로 교체(추가된 줄은 `let mut swing_return_rail_x = home_rail_x;`,
`swing_return_rail_x = measured.rail_x;`, 그리고 `move_to_ready`에 넘기는 인자):

```rust
            } else if idle_ready && due_for_return {
                let mut swing_return_rail_x = home_rail_x;
                if let BallControlState::Aligning { measurement, .. } = &state {
                    match hardware.read_pose() {
                        Ok(measured) => {
                            swing_return_rail_x = measured.rail_x;
                            let joint_errors: Vec<f64> = measurement
                                .joints_commanded
                                .values
                                .iter()
                                .zip(&measured.joints.values)
                                .map(|(commanded, measured)| commanded - measured)
                                .collect();
                            info!(
                                track_seq = measurement.track_seq,
                                rail_commanded_m = f4(measurement.rail_commanded_m),
                                rail_measured_m = f4(measured.rail_x),
                                rail_commanded_minus_measured_m =
                                    f4(measurement.rail_commanded_m - measured.rail_x),
                                joints_commanded = %format!("{:?}", measurement.joints_commanded.values),
                                joints_measured = %format!("{:?}", measured.joints.values),
                                joints_commanded_minus_measured = %format!("{joint_errors:?}"),
                                "공 위치·방향 정렬 완료 후 실측"
                            );
                        }
                        Err(error) => warn!(%error, "공 위치·방향 정렬 완료 후 포즈 읽기 실패"),
                    }
                }
                if let Err(error) = move_to_ready(hardware.as_mut(), &arm, swing_return_rail_x) {
```

나머지(`if let Err(error) = move_to_ready(...)` 블록의 내부, 그리고 그 뒤
`match hardware.read_pose() { ... }` 블록)는 손대지 않는다 — `home_rail_x`를
쓰던 곳이 딱 한 줄뿐이었다. 실측 실패(`Err(error) => warn!(...)`) 시에는
`swing_return_rail_x`가 초기값 `home_rail_x` 그대로 남아, 위치를 모를 때는
안전하게 홈으로 복귀하는 기존 동작을 유지한다.

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::spawn_keeps_rail_at_swing_position_after_return_to_ready`

Expected: PASS.

- [ ] **Step 5: 기존 회귀 테스트 확인**

Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::tests::`

Expected: 이 파일의 모든 테스트가 PASS (Task 1의 신규 테스트 포함, 기존
`apply_test_control_*`/`spawn_ignores_balls_while_waiting_and_resumes_after_next`
등 회귀 없음).

- [ ] **Step 6: 전체 빌드 확인**

Run: `cargo build -p pingpong-bot`

Expected: 경고 없이 컴파일 성공.

- [ ] **Step 7: 커밋**

```bash
git add src/real/control_worker.rs
git commit -m "feat(real): keep rail at hit position after swing, only reset joints"
```

---

## 최종 확인

두 태스크 완료 후:

- [ ] Run: `cargo test -p pingpong-bot --bin pingpong-bot real::control_worker::`
      — 이 파일의 전체 테스트 스위트가 PASS.
- [ ] Run: `cargo build -p pingpong-bot --release` — 릴리스 빌드도 경고 없이
      통과하는지 확인(실기 배포 전 상시 확인 항목).
