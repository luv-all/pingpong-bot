# run_real 단발 타격 (1 hit) Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** `--mode real`에서 실캠 → 검출 → 삼각측량 → EKF → 플래너 → 실기 하드웨어까지
**공 1발에 스윙 한 번**을 커밋하고 종료한다. 랠리는 범위 밖.

**Architecture:** bin 전용 `src/real/` 모듈. 카메라 2 + 추정 1 + 제어 1 스레드를
**crossbeam 채널로만** 잇고 상태는 스레드별 단독 소유 — `Arc<Mutex<Hardware>>` 없음.
`read_pose → plan_best → command`가 전부 제어 스레드 안에서 일어나 jog의
"스냅샷으로 계획, 그대로 전송" 불변식이 구조적으로 성립한다.

**Tech Stack:** Rust, crossbeam-channel, 기존 `camera`/`detector`/`estimator`/`robot::motion`/
`hardware` 도메인, OpenCV highgui 프리뷰.

**Spec:** 이 문서에 설계 포함 (별도 spec 없음).

---

## Context

`--mode real`은 지금 **pose 스모크**까지다 (`src/main.rs:101-115`): Dynamixel/AXL 열고
`read_pose()` 한 번, `detector_for(cam0)` 한 번 만들고 버린 뒤 끝. 실캠 파이프라인이 실기에서
한 번도 돈 적이 없다.

`TODO.md` §4가 추적하는 항목(`run_real + 카메라·Pipeline`)이고 우선순위에도 올라와 있다.
1발이 되면 sim에서만 검증된 게이트·타이밍·IK 가정이 실물에서 처음 검증된다.

`src/pipeline/`에 카메라 N + 추정 1 + 제어 1 오케스트레이션이 **완성된 채로 죽어 있지만**
(`Pipeline::run` 호출부 0개), 랠리 전제라 종료 경로가 없고 커밋 래치·미드코트 게이트·포기 정책이
빠져 있다. 이번엔 건드리지 않고 단발 전용 경로를 새로 만든다.

## Global Constraints

- **공유 가변 상태 금지.** 스레드 간 전달은 채널 메시지만. `Arc<Mutex<RealHardware>>`를 쓰지 않는다
- `RealHardware`는 제어 스레드가 단독 소유 — 포즈 읽기·계획·전송이 한 스레드 안에서만
- 커밋은 **1회 래치**. 래치 후 들어오는 요청은 제어 스레드가 조용히 버린다
- 실시간 경로는 **drop-on-full**. 채널이 차면 오래된 프레임·예측을 버리고 카운트한다
- highgui는 메인 스레드에서만 (macOS 제약). 프리뷰가 핫패스를 막지 않는다
- 게이트 순서·상수는 sim `try_auto_swing`과 동일 — real 전용 튜닝 값을 새로 만들지 않는다
- 숫자 SSOT는 `src/defaults/`. CLI는 포트·플래그만 덮어쓴다
- 파일당 주 타입 1개 (repo 규약)

---

## 설계

### 동시성 — jog에서 가져온 불변식

`tools/jog`의 핵심은 **"동기화한 포즈 스냅샷 하나로 계획하고, 그 궤적을 그대로 보낸다"**
(`tools/jog/src/state/jog_app.rs` `sync → preview → apply`, `phase.rs`의 Phase 게이트).
계획과 전송 사이에 포즈가 바뀔 틈이 없다.

같은 보장을 채널 + 단일 소유권으로 강제한다. 상태를 공유하지 않으므로 race condition이
"안 생기게 조심하는" 게 아니라 **표현 불가능**해진다.

| 상태 | 유일한 소유자 |
| --- | --- |
| `FrameSource` + `Detector` | 카메라 스레드 (캠당 1개) |
| `Ekf` · `Calibration` · 게이트 | 추정 스레드 |
| `RealHardware` (버스 · 레일 · 커밋 래치) | 제어 스레드 |
| highgui 창 | 메인 스레드 |

추정 스레드는 로봇 포즈를 **볼 수 없다**. 그래서 "낡은 포즈로 계획"이 애초에 불가능하다.

### 스레드 · 채널

```text
  cam-left  ─┐
             ├─ VisionEvent ──►  estimator ──┬─ CommitRequest ──►  control
  cam-right ─┘   bounded(4)                  │     bounded(1)      (RealHardware 단독 소유)
                 drop-on-full                │     drop-on-full
                                             │
                                PreviewEvent │                  ShotEvent
                                  bounded(2) │                 unbounded
                                drop-on-full ▼                          ▼
                                          main (highgui + 로그 + 종료)
```

- **셧다운**: `AtomicBool` 대신 채널 파기 브로드캐스트. 메인이 `Sender<Never>` 하나를 들고
  각 워커는 `Receiver` 클론으로 매 루프 `try_recv()`가 `Disconnected`인지 본다. 메인이 sender를
  drop하면 전원 종료 — 공유 플래그가 없다
- **종료 순서**: 메인이 `Committed`/`Abandoned`/타임아웃 수신 → 셧다운 drop → 카메라·추정 종료.
  제어 스레드는 이미 래치된 상태로 스윙 완주 → 센터 복귀 → `ShotEvent::Done` → 종료. 메인이 join

### 단발 상태 기계

sim `SimWorld::try_auto_swing` (`src/sim/physics/world.rs:719`)의 게이트 **순서를 그대로** 옮긴다.
추정 스레드가 판정하고 제어 스레드가 커밋을 래치한다.

| 단계 | 위치 | 내용 |
| --- | --- | --- |
| 1. 추적 | estimator | `Ekf::is_tracking()` — velocity_seeded 전이면 대기 |
| 2. 예측 | estimator | `InterceptWindow::default().hit_planes()` → `ekf.predict_to(plane)` |
| 3. 미드코트 | estimator | `Planner::past_midcourt(ekf.position().y)` |
| 4. 너무 늦음 | estimator | `max(tti) < min_swing_secs` → **포기** |
| 5. 커밋 창 | estimator | `predictions.iter().any(Planner::in_commit_window)` |
| 6. 계획·전송 | control | `read_pose` → `Planner::plan_best` → `hardware.command` → 래치 |
| 7. 완주 | control | `while is_busy { sleep 5ms }` (jog `apply()`와 동일) |
| 8. 복귀 | control | `Planner::return_to_center` → `command` → 완주 대기 |

> **4단계는 `min`이 아니라 `max`다.** sim에서 `min`으로 썼다가 커밋률이 0%로 붕괴한 이력이
> `world.rs` 주석에 남아 있다. 단위 테스트로 잠근다.

제어 스레드의 `plan_best` 오류 처리도 sim과 동일:

- `JointOrTorqueLimit` → **영구 포기** (모터 보호), `ShotEvent::Abandoned`
- `InsufficientTime` → 조용히 버림 (다음 요청 대기)
- 그 외 → 1초 스로틀 `warn`

제어 스레드에도 `SWING_RETRY_THROTTLE_SECS`(20 ms) 스로틀을 둔다 — 57600 baud에서 `read_pose`는
sync_read 왕복이라 매 프레임 때리면 안 된다.

### 재사용 (새로 쓰지 않는 것)

| 필요 | 기존 것 |
| --- | --- |
| 실캠 2대 열기 | `CamCliArgs { cam: DEFAULT_STEREO_CAM_ROLES.to_vec(), stream: CamStreamArgs::default() }.open_sources()` — `src/camera/io/cam_cli/cam_cli_args.rs:52`. `threaded=true`라 캠당 grab 스레드까지 딸려온다 |
| 검출기 | `defaults::detector_for(camera::Id)` — `src/defaults/vision.rs:91` |
| 캘리브 | `Calibration::load_json(&defaults::calibration_path())` — `PipelineConfig`의 `Calibration::sim(3)`이 **아니다** |
| 왜곡 보정 | `Detector::undistort(&frame, &params)`. 단 `params.dist.is_empty()`면 스킵 (커밋된 calibration은 dist가 비어 있음 — 프레임당 remap 절약) |
| 삼각측량 | `Triangulate::pixels(&[(Id, Pixel)], &calibration)` |
| EKF | `Ekf::default()`, `update_position(p, t) -> GateOutcome` |
| 계획 | `Planner::plan_best` / `in_commit_window` / `past_midcourt` / `return_to_center` |
| 프리뷰 | `camera::io::preview::{show_bgr, hstack_bgr, draw_circle_px, draw_debug_lines, destroy_window}` + `PreviewAction` — `tools/verify_stereo/src/run.rs`가 쓰는 그대로 |
| 완주 대기 | jog `apply()`의 `while hw.is_busy() { sleep(10ms) }` |

### 로그 — sim과 용어 통일

`"real shot: armed"` / `"track"` / `"swing commit"` / `"포기 — …"` / `"end"`.

`swing commit`은 sim(`world.rs:961`)과 **같은 필드**(`duration_secs`, `rail_end`, `impact`,
`tti`, `peak_joint_speed`)를 찍어 sim ↔ real을 바로 비교할 수 있게 한다.

`end` 요약: 프레임 수 · 캠별 검출률 · 삼각측량 성공 수 · `GateOutcome` 분포 ·
스테레오 타임스탬프 skew p50/p95 · 채널 드롭 수 · 커밋 여부.

---

### Task 1: CLI 플래그

**Files:**

- Modify: `src/cli/args.rs`

```rust
/// 모터·레일을 실제로 움직이지 않고 전체 체인만 리허설 (`RealHardware::dry_run_with_arm`).
#[arg(long)]
pub dry_run: bool,
/// 좌/우 검출 오버레이 프리뷰 창. 끄려면 `--preview=false`
#[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
pub preview: bool,
/// 시작 시 센터(ready) 자세로 이동. 끄려면 `--home=false`
#[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
pub home: bool,
/// 공을 기다리는 최대 시간 [s].
#[arg(long, default_value_t = 60.0)]
pub timeout_secs: f64,
```

- [x] 4개 플래그 추가 (`--preview` / `--home`은 `ArgAction::Set` — `CamStreamArgs::threaded` 패턴)
- [x] `--preview` 기본 on — 왜 안 치는지 눈으로 봐야 한다
- [x] `--home` 기본 on — `plan_best`는 `arm.default_joints` 근처 시작 포즈를 전제한다.
      임의 자세에서 시작하면 IK가 대부분 실패. 이동 전에 목표를 크게 로그
- [x] `cargo build` + `--help` 확인

### Task 2: 메시지 타입 + Shutdown

**Files:**

- Create: `src/real/mod.rs`, `options.rs`, `shutdown.rs`, `vision_event.rs`,
  `commit_request.rs`, `preview_event.rs`, `shot_event.rs`
- Modify: `src/main.rs` (`mod real;`)

bin 전용 모듈. `src/cli/`와 같이 `main.rs`가 `mod real;`로 선언하고 `lib.rs`에는 넣지 않는다.
모드는 **경로**(`real::`)로 드러나고 타입명은 도메인 어휘를 그대로 쓴다.

- [x] `Options { dry_run, preview, home, timeout_secs }` — `Args`에서 변환
- [x] `Shutdown` — `Sender<Never>` 파기 브로드캐스트. `is_down()` + 메인이 쥐는 guard
- [x] `VisionEvent { frame: camera::Frame, pixel: Option<camera::Pixel> }` (cam → estimator)
- [x] `CommitRequest { predictions: Vec<Prediction>, at: Instant }` (estimator → control)
- [x] `PreviewEvent { frame, pixel, hud: Vec<String> }` (estimator → main)
- [x] `ShotEvent` — `Armed` / `Tracking` / `Committed` / `Abandoned` / `PlanFailed` / `Done`
- [x] `Frame`은 `Mat` 소유라 채널로 **이동**시킨다 (복사 없음)

### Task 3: decision.rs — 순수 게이트 + 단위 테스트

**Files:**

- Create: `src/real/decision.rs`

`Box<dyn Hardware>`만 쓰므로 하드웨어에 묶이지 않는다.

> **구현 중 변경:** `real`을 **기본 feature로 승격**했다 (`Cargo.toml` `default = ["gui", "real"]`).
> 빌드 feature와 런타임 모드를 둘 다 켜야 하는 마찰(`--features real -- --mode real`)이 불필요했다.
> 덕분에 `--mode real`만으로 돌고 `cargo test --workspace`가 real 테스트까지 잡는다.
> 모듈 전체는 `#[cfg(feature = "real")]`로 묶어서, feature를 끄면 dead code 경고가 아니라
> 코드 자체가 사라지게 했다.

- [x] `Decision { Wait(WaitReason), Abandon(&'static str), Attempt }`
- [x] `decide(tracking: bool, ball_y: Option<f64>, predictions: &[Prediction]) -> Decision`
      — sim `try_auto_swing` 게이트 순서 그대로
- [x] 테스트: 추적 전 → `Wait`
- [x] 테스트: 미드코트 이전 → `Wait`
- [x] 테스트: `max(tti) < min_swing_secs` → `Abandon` (**`min`이 아니라 `max`**)
- [x] 테스트: 커밋 창 밖 → `Wait`
- [x] 테스트: 창 안 → `Attempt`
- [x] `cargo test --workspace`

### Task 4: 워커 3종

**Files:**

- Create: `src/real/camera_worker.rs`, `estimator_worker.rs`, `control_worker.rs`

- [x] `camera_worker::spawn(source, detector, params, tx, shutdown)`
      — `next_frame` → (dist 있으면) `undistort` → `detect` → `VisionEvent` `try_send` drop-on-full
- [x] `estimator_worker::spawn(rx, commit_tx, preview_tx, event_tx, shutdown)`
      — 캠별 최신 관측 유지 → 양캠 확보 시 `Triangulate::pixels` → `ekf.update_position`
      → `hit_planes().filter_map(predict_to)` → `decide()` → `CommitRequest`
- [x] 스테레오 타임스탬프 skew 기록 (`Triangulate::pixels`는 타임스탬프를 무시한다)
- [x] `control_worker::spawn(hardware: Box<dyn Hardware>, arm, rx, event_tx)`
      — `--home`이면 부팅 시 `return_to_center` 후 `Armed`
- [x] 15 ms 넘은 `CommitRequest`는 버린다 (예측의 `tti`가 요청 시각 기준이라 계획 지연만큼 낡음)
- [x] 커밋 1회 래치 + `plan_best` 오류 3분기 (`JointOrTorqueLimit` 영구 포기 / `InsufficientTime`
      무시 / 그 외 1초 스로틀 warn)
- [x] 20 ms 계획 스로틀
- [x] 커밋 후 `is_busy` 완주 대기 → `return_to_center` → `Done`

### Task 5: preview + run + main 배선

**Files:**

- Create: `src/real/preview.rs`, `src/real/run.rs`
- Modify: `src/main.rs`

- [x] `preview.rs` — 좌/우 `hstack_bgr` + `draw_circle_px` + HUD `draw_debug_lines`,
      `PreviewAction::Quit`(ESC/q)이면 셧다운
- [x] `run.rs` — `#[cfg(feature = "real")] pub fn run(args: &Args) -> Result<()>`.
      `robot()` → 하드웨어(`dry_run_with_arm` / `new`) → 캠 2대 → 검출기 → `Calibration::load_json`
      → 채널·워커 조립 → 메인 프리뷰/이벤트 루프 → join → 요약 로그
- [x] `main.rs`: `mod real;`, `ModeArg::Real => real::run(&args)?`.
      `#[cfg(not(feature = "real"))]` bail 분기는 유지 (`--no-default-features` 빌드 대비)
- [x] 기존 `run_real_entry` 제거
- [x] `cargo build`

### Task 6: 문서

**Files:**

- Modify: `README.md`, `TODO.md`

- [x] `README.md` §real — "pose 스모크까지" 문장을 단발 타격 실행법으로 교체, 아키텍처
      다이어그램의 `real["… 실캠 예정"]`에서 "예정" 제거
- [x] `TODO.md` §4 `run_real + 카메라·Pipeline` 체크, 랠리는 후속 항목으로 남김

---

## 알려진 리스크 (구현 중 로그로 계측할 것)

1. **스테레오 타임스탬프 skew** — `Triangulate::pixels`는 타임스탬프를 무시한다. UVC 캠은
   하드웨어 동기가 없고 `TODO.md` §3이 "멀티캠 동기 — 비범위"로 못 박았다. 단발에선 `pixels`로
   가되 **skew를 매 프레임 기록**해 수치로 남긴다. 크게 나오면 `Triangulate::synced` 보간 검토
2. **커밋 요청 지연** — 예측의 `time_to_impact_secs`는 요청 시각 기준. `CommitRequest.at`으로
   낡은 요청을 버린다 (sim bang-bang worker의 경과 보정과 같은 문제)
3. **stale ready 포즈** — `src/defaults/robot.rs:73-90`에 기록된 대로 `READY_JOINTS_4DOF`와
   `mount_y`가 새 베이스 높이(0.935 m) 기준이 아니라 IK 도달률이 118/240 → 91/240으로 떨어져
   있다. 첫 실기에서 `plan_best` 실패가 잦으면 원인은 여기다
4. **EKF에 스핀 상태 없음** — `Ekf::predict_to`는 ω=0으로 예측한다 (Magnus 0). sim GT 경로는
   진짜 ω를 쓴다. 실기 예측 오차의 알려진 하한
5. **레일은 단일 절대 이동** — 관절은 200 Hz 스트리밍인데 레일은
   `command_abs_in_secs(follow_through_rail_x, duration_secs)` 한 번이다. 임팩트 시점 레일
   위치가 궤적과 어긋날 수 있다
6. **macOS에서 실기 불가** — `AxlRail::open`이 Windows 전용. `--dry-run`은 `AxlRail::dry_run`을
   타므로 macOS에서도 전 체인 리허설이 된다

## 빠른 검증

```bash
cargo test --workspace          # decision.rs·shutdown.rs 단위 테스트 8개 포함
cargo build

# macOS/Windows 리허설 — 실캠 2대만 있으면 검출→EKF→플래너까지 전부 돈다 (모터 정지)
cargo run -p pingpong-bot -- --mode real --dry-run --debug

# Windows 실기 — 먼저 jog로 배선 확인
cargo run -p jog -- --port COM8 --dry-run
cargo run -p jog -- --port COM8
cargo run -p pingpong-bot -- --mode real --dxl-port COM8 --debug
```

리허설 수동 확인:

- 프리뷰에 좌/우 공이 계속 잡히는가 (검출률 로그)
- `"real shot: track"`이 뜨는가 — EKF가 velocity_seeded까지 가는가
- `swing commit`의 `impact`가 테이블 위 상식적인 좌표인가
- skew p95가 프레임 간격(8.3 ms)보다 작은가

실기 전 레일 x 범위(`RailConfig::default()` 0 ~ 1.41 m)와 관절 리밋(`motor_angle_limits_deg`)이
벤치 실물과 맞는지 jog로 먼저 확인할 것.
