# `src/real` — 실기 단발 타격 (1 hit)

`--mode real` 런타임. 공 **하나**를 받아 스윙 **한 번**을 커밋하고 종료한다. 랠리는 아직 아니다.

bin 전용 모듈이라 `lib.rs`에 없다 ([`src/cli/`](../cli/)와 같은 방식). 도메인 로직은 전부
`camera` / `detector` / `estimator` / `robot::motion` / `hardware`에서 가져다 쓰고, 여기는
**오케스트레이션과 정책**만 담는다.

```bash
# 리허설 — 실캠·검출·EKF·플래너까지 다 돌리고 모터·레일만 안 움직인다 (macOS에서도 됨)
cargo run -p pingpong-bot -- --mode real --dry-run

# 실기 (Windows 벤치)
cargo run -p pingpong-bot -- --mode real --dxl-port COM8 --debug
```

| 플래그 | 기본 | 뜻 |
|--------|------|-----|
| `--dry-run` | off | `RealHardware::dry_run_with_arm` — 모터·레일 정지, 나머지 체인은 그대로 |
| `--preview` | on | 좌/우 검출 오버레이 창. ESC·`q`로 종료 |
| `--sim` | on | 관전용 3D 창 (테이블·로봇·예측 도달점·스윙 재생) |
| `--home` | on | 시작 시 센터(ready) 자세로 이동 |
| `--release-torque` | off | 종료 시 토크를 뺀다. 기본은 **켠 채로 둔다** (안 그러면 팔이 주저앉는다) |
| `--timeout-secs` | 60 | 공을 기다리는 최대 시간 |
| `--dxl-port` | `COM8` | `DynamixelConfig::default().port` 덮어쓰기 |

**샷이 끝나도 프로그램은 안 꺼진다** (`--preview`가 켜져 있을 때). 커밋했든 포기했든
동작만 멈추고 결과를 창에 띄운 채 기다린다 — 순식간에 끝나 아무것도 못 보는 걸 막기 위해서다.
종료는 ESC·`q`. `--preview=false`면 볼 것이 없으니 끝나는 즉시 종료한다.

**종료해도 토크는 유지된다.** `DynamixelBus::drop`이 토크를 끄면 프로그램이 끝나는 순간 팔이
중력으로 주저앉는다 (AXL 레일도 같은 이유로 서보를 켠 채 닫는다). 손으로 옮기려면
`--release-torque`를 쓰거나 전원을 내린다. 부작용으로 다음 실행의 EEPROM 설정(Operating Mode
11 · PWM Limit 36 · Current Limit 38)이 토크 ON 상태에서 거부될 수 있어,
`configure_position_mode_max_effort`가 **먼저 읽어보고 이미 맞으면 아무것도 쓰지 않는다** —
그래서 재실행할 때도 팔이 잠깐 늘어지지 않는다.

전제 파일: `data/calibration.json` (캠 2대) · `data/colormask.json`.

---

## 왜 채널인가 — jog에서 가져온 불변식

[`tools/jog`](../../tools/jog/)의 핵심은 **"동기화한 포즈 스냅샷 하나로 계획하고, 그 궤적을
그대로 보낸다"** 이다 (`sync → preview → apply` 단계 게이트). 계획과 전송 사이에 포즈가 바뀔
틈이 없어야 실기가 엉뚱한 데를 친다.

여기서는 같은 보장을 **단일 소유권 + 채널**로 강제한다. 공유 가변 상태가 없어서 race condition이
"안 생기게 조심하는" 게 아니라 **표현 불가능**하다.

| 상태 | 유일한 소유자 |
|------|--------------|
| `FrameSource` + `Detector` | `camera_worker` (캠당 1 스레드) |
| `Ekf` · `Calibration` · 게이트 | `estimator_worker` |
| `Hardware` (버스 · 레일 · 커밋 래치) | `control_worker` |
| highgui 창 | 메인 스레드 (`PreviewWindow`) |
| kiss3d 관전 창 | **자식 프로세스** (`--sim-child`) |

`Arc<Mutex<Hardware>>`가 없다. **`read_pose → plan_best → command`가 전부 `control_worker`
안에서만** 일어나고, 추정 스레드는 로봇 포즈를 볼 방법 자체가 없다.

---

## 파이프라인

kiss3d(3D 창)와 OpenCV highgui(프리뷰)가 **둘 다 메인 스레드를 요구**해서 한 프로세스에 같이
못 띄운다. `tools/verify_stereo`와 같이 자기 자신을 `--sim-child`로 띄우고 stdin 한 줄
JSON(`SimUpdate`)으로 먹인다. 자식이 죽어도 본 파이프라인은 그대로 간다.

```mermaid
flowchart LR
  subgraph cams ["카메라 스레드 × 2"]
    camL["cam-left<br/>capture → detect"]
    camR["cam-right<br/>capture → detect"]
  end

  est["estimator<br/>삼각측량 · EKF · 게이트"]
  ctl["control<br/>read_pose · plan_best · command"]
  main["main<br/>highgui · 로그 · 종료"]
  hw["RealHardware"]

  camL -->|"VisionEvent<br/>bounded(8) drop-on-full"| est
  camR -->|"VisionEvent"| est
  est -->|"CommitRequest<br/>bounded(1) drop-on-full"| ctl
  est -->|"PreviewEvent<br/>bounded(2) drop-on-full"| main
  est -->|"ShotEvent"| main
  ctl -->|"ShotEvent (unbounded)"| main
  ctl --> hw
  est -->|"SimUpdate"| sim["sim 자식 프로세스<br/>kiss3d 관전 창"]
  ctl -->|"SimUpdate (커밋 궤적)"| sim

  main -.->|"Shutdown 가드 drop"| cams
  main -.->|"Shutdown 가드 drop"| est
```

**드롭 정책** — 실시간 경로다. 채널이 차면 `try_send` 실패를 버리고 센다. 밀린 프레임·예측은
어차피 쓸모가 없다. 드롭 수는 종료 요약에 찍힌다.

**셧다운** — `AtomicBool` 대신 채널 파기 브로드캐스트 (`shutdown.rs`). 메인이 `ShutdownGuard`
하나를 들고 워커는 `Shutdown` 클론을 든다. 가드를 drop하면 모든 클론의 `try_recv`가
`Disconnected`가 되어 전원이 내려간다 — 공유 플래그가 없고 실수로 되켤 방법도 없다.

**프리뷰가 핫패스를 막지 않는다** — highgui는 macOS 제약상 메인 스레드여야 하는데,
`PreviewEvent`는 drop-on-full이라 창이 느려도 추정·제어는 그대로 돈다.

---

## 샷 라이프사이클

용어는 sim(`SimWorld::try_auto_swing`)과 맞춘다. 실기엔 슈터가 없어서 sim의 `launch` 대신
**`Tracking`**(EKF가 속도까지 시드한 시점)이 샷의 시작이다.

```mermaid
stateDiagram-v2
    [*] --> Homing: --home
    Homing --> Armed: return_to_center 완주
    Homing --> Failed: 포즈 읽기 실패

    Armed --> Tracking: EKF velocity_seeded
    Tracking --> Tracking: 게이트 Wait — 다음 관측

    Tracking --> Committed: plan_best Ok → command
    Tracking --> TooLate: max(tti) < min_swing_secs
    Tracking --> Infeasible: 관절·토크 한계
    Armed --> TimedOut: --timeout-secs 초과

    Committed --> Finishing: 스윙 완주 대기
    Finishing --> Done: 센터 복귀
    TooLate --> Done
    Infeasible --> Done
    TimedOut --> Done
    Failed --> Done
    Done --> Frozen: --preview
    Frozen --> [*]: ESC / q
    Done --> [*]: --preview=false
```

`ShotEvent`는 전부 메인으로 모여 한 곳에서만 로그된다 (워커가 중복으로 찍지 않는다).
`Committed`는 sim `"shot: swing commit"`과 **같은 필드**(`duration_secs` · `rail_end` ·
`impact` · `tti` · `peak_joint_speed`)를 실어서 sim ↔ real 로그를 그대로 비교할 수 있다.

포기 이벤트는 **근거 수치를 들고 온다** — `TooLate`는 `latest_tti_secs` · `min_swing_secs` ·
`candidates` · `ball_y`를, `Infeasible`은 IK 목표 좌표를 싣는다. 문자열 사유만 남기면 벤치에서
"얼마나 늦었길래 포기했나"를 되짚을 수 없다. 둘 다 **info** 레벨이라 `--debug` 없이 보인다.

`Done`은 항상 마지막이고 **제어 워커만** 보낸다. `--preview`면 `Done` 뒤에도 프로그램은 살아
있고 (Frozen) 화면만 갱신한다 — 제어 워커는 이미 래치돼 아무것도 하지 않는다.

---

## 화면

**프리뷰 창** (`real shot`) — 좌/우 프레임을 가로로 붙인다.

- 초록 원 = 검출한 공
- 빨간 원 = **예측 도달 위치**를 그 카메라로 재투영한 자리 (`camera::Params::project_world`)
- 좌상단 노란 HUD = 게이트 상태 · 공 위치·속력 · 예측 도달점·tti · EKF 게이트 d²
- 우하단 = 샷 결과 (커밋 요약 또는 포기 사유). 한 번 뜨면 창을 닫을 때까지 남는다

**sim 창** (`real shot sim`) — 아무것도 조작하지 않는 관전 전용.

- 주황 공 = EKF 추정 공 위치
- 반투명 공 = 예측 도달 위치
- 로봇 = 실기에서 읽은 포즈. 커밋하면 **그 궤적을 그대로 재생**한다

HUD 문자열은 ASCII만 쓴다 — Hershey 폰트가 유니코드를 못 그려 한글은 `??????`가 된다.
한글 사유는 로그로만 나간다.

## 커밋 결정 게이트

`decision.rs`의 순수 함수. sim `try_auto_swing`의 게이트 **순서를 그대로** 옮겼고,
상수는 전부 `defaults::ControlParams` / `robot::motion::Planner`에서 온다.
하드웨어 의존이 없어서 단위 테스트로 잠근다.

```mermaid
flowchart TD
    start(["관측 1건 처리 후"]) --> tracking{"ekf.is_tracking()?"}
    tracking -->|no| w1["Wait(NoTrack)"]
    tracking -->|yes| pred{"예측이 있나?<br/>hit_planes → predict_to"}
    pred -->|없음| w2["Wait(NoPrediction)"]
    pred -->|있음| mid{"past_midcourt(ball_y)?"}
    mid -->|no| w3["Wait(BeforeMidcourt)"]
    mid -->|yes| late{"max(tti) < min_swing_secs?"}
    late -->|yes| ab["Abandon<br/>너무 늦음"]
    late -->|no| win{"any in_commit_window?"}
    win -->|no| w4["Wait(OutOfWindow)"]
    win -->|yes| att["Attempt<br/>→ CommitRequest"]
```

> **`max`이지 `min`이 아니다.** `min`으로 쓰면 아직 여유 있는 후보가 늦은 후보 하나에 끌려가
> 통째로 포기된다. sim에서 이 실수로 커밋률이 0%가 된 이력이 `world.rs` 주석에 남아 있다.
> `abandons_only_when_every_candidate_is_too_late` 테스트가 이걸 잠근다.

`Attempt`가 나오면 제어 워커가 이어받는다:

```mermaid
flowchart TD
    req(["CommitRequest 수신"]) --> stale{"age > 15 ms?"}
    stale -->|yes| drop["버림 — 예측이 낡음"]
    stale -->|no| thr{"직전 시도 < 20 ms?"}
    thr -->|yes| drop2["버림 — 스로틀"]
    thr -->|no| pose["read_pose"]
    pose --> plan{"Planner::plan_best"}
    plan -->|Ok| cmd["command(그 궤적 그대로)<br/>→ Committed · 래치"]
    plan -->|JointOrTorqueLimit| abandon["Infeasible<br/>모터 보호 — 재시도 안 함"]
    plan -->|InsufficientTime| retry["조용히 버림 — 다음 요청"]
    plan -->|그 외| warn["PlanFailed<br/>1초 스로틀 warn → 다음 요청"]
```

`plan_best`에 넘긴 예측으로 만든 궤적을 **그대로** 보낸다. 사이에 포즈를 다시 읽지 않는다.

---

## 튜닝 상수

| 상수 | 값 | 위치 | 왜 |
|------|-----|------|-----|
| `MAX_STEREO_SKEW_SECS` | 20 ms | `estimator_worker` | 스테레오 쌍으로 인정할 타임스탬프 차. UVC는 하드웨어 동기가 없다 |
| `MAX_REQUEST_AGE_SECS` | 15 ms | `control_worker` | 예측의 `tti`는 요청 시각 기준 — 이보다 낡으면 임팩트 시점이 어긋난다 |
| `PLAN_THROTTLE_SECS` | 20 ms | `control_worker` | sim `SWING_RETRY_THROTTLE_SECS`와 같은 값. 57600 baud `read_pose`는 sync_read 왕복이다 |
| `FINISH_GRACE` | 15 s | `run` | 커밋 후 제어 워커가 스윙 완주 + 센터 복귀할 여유 |
| 게이트 임계 | — | `defaults::ControlParams` | `min_swing_secs` 0.20 · `swing_commit_max_secs` 0.60 · `swing_commit_max_ball_y_frac` 0.55 |
| 접수 창 | — | `InterceptWindow::default()` | y 0.08 ~ 0.35, step 0.03 (10 평면) |

real 전용 튜닝 값을 새로 만들지 않는다 — sim과 갈리면 sim에서 튜닝한 결과가 실기에 안 옮겨진다.

---

## 파일

| 파일 | 주 타입 | 역할 |
|------|--------|------|
| `run.rs` | — | 조립 · 메인 루프 · 종료 요약 |
| `options.rs` | `Options` | CLI 플래그 |
| `shutdown.rs` | `Shutdown` / `ShutdownGuard` | 채널 파기 종료 브로드캐스트 |
| `camera_worker.rs` | `CameraStats` | 캡처 → (왜곡 보정) → 검출 |
| `estimator_worker.rs` | `EstimatorStats` | 삼각측량 → EKF → 예측 → 게이트 |
| `control_worker.rs` | — | 하드웨어 단독 소유 · 계획 · 커밋 래치 |
| `decision.rs` | `Decision` / `WaitReason` | 순수 게이트 + 단위 테스트 |
| `preview.rs` | `PreviewWindow` | 메인 스레드 highgui |
| `sim_child.rs` | — | `--sim-child` kiss3d 관전 창 (자식 프로세스) |
| `sim_host.rs` | — | 부모 쪽 자식 관리 — 띄우고 stdin으로 먹인다 |
| `sim_update.rs` | `SimUpdate` | 부모 → sim 자식 한 줄 JSON (공·도달점·포즈·스윙) |
| `throttle.rs` | `Throttle` | 주기 로그 스로틀 |
| `fmt.rs` | — | 로그 숫자 소수점 2자리 |
| `vision_event.rs` | `VisionEvent` | cam → estimator |
| `commit_request.rs` | `CommitRequest` | estimator → control |
| `preview_event.rs` | `PreviewEvent` | estimator → main |
| `shot_event.rs` | `ShotEvent` | 워커 → main |

---

## 로그

`--debug` 없이는 **샷 라이프사이클(info)과 오류(warn)만** 나온다. 붙였을 때 추가되는 것:

| 로그 | 스레드 | 언제 | 왜 보나 |
|------|--------|------|---------|
| `real shot: 포기 — …` | main | 포기할 때 (**info** — `--debug` 불필요) |
| `real shot: 게이트 전이` | estimator | `Decision`이 **바뀔 때만** | "왜 안 쳤나"를 로그만으로 되짚는다. `decision` · `candidates` · `tti_min/max` · 공 위치·속력 |
| `측정 거부` | estimator | EKF가 측정을 버리거나 리셋할 때 | 마할라노비스 `d2` · `reject_streak` · 그 3D 점 · `skew_ms` |
| `추정 진척` | estimator | 1초마다 | triangulated·accepted·rejected 누적, 현재 게이트 |
| `카메라 진척` | cam × 2 | 1초마다 | 실측 `fps` · 최근 1초 `detection_rate` · 드롭 |
| `스윙 계획 실패 — 재시도` | control | 시도마다 | 실패 원인 + 후보 수 + 그때의 시작 포즈 |
| `InsufficientTime` | control | 늦은 예측을 버릴 때 | `tti` vs `min_swing_secs` |
| `커밋 요청 소비` | control | 계획 직전 | `request_age_secs` — 예측이 얼마나 낡았나 |

포기·커밋 로그는 판정 근거 **수치를 같이 싣는다** — `latest_tti` · `min_swing_secs` ·
`shortfall` · `candidates` · `ball_y`. 문자열 사유만 남으면 "얼마나 늦었길래"를 못 되짚는다.
로그의 실수는 전부 소수점 2자리다 (`fmt::f2`).

per-tick 로그는 없다. 120 fps × 2캠이면 초당 수백 줄이라 쓸 수 없어서, **전이 시점**과
**1초 주기**로만 묶는다 (`throttle.rs`). tracing span이 붙어 있어 `cam{id=0}` · `estimator` ·
`control` 중 어느 스레드인지 바로 보인다.

```text
DEBUG cam{id=0}: 카메라 진척 fps=118.0 detection_rate=0.86 frames=354 dropped=0
DEBUG estimator: real shot: 게이트 전이 decision=Wait(BeforeMidcourt) candidates=7
                 tti_min=0.31 tti_max=0.58 ball_x=0.71 ball_y=1.44 ball_z=0.35 speed=4.8
DEBUG estimator: real shot: 게이트 전이 decision=Attempt candidates=9 tti_min=0.22 tti_max=0.55
INFO  real shot: swing commit duration_secs=0.34 rail_end=0.81 impact_x=0.68 … tti=0.28
```

## 종료 요약 읽는 법

```text
real shot: end — 카메라  cam=0 frames=91 detections=0 detection_rate=0.0 dropped=0
real shot: end — 추정    triangulated=0 accepted=0 rejected=0 seeded=0 reset=0
                         skew_p50_ms=… skew_p95_ms=… commit_dropped=0 preview_dropped=3
real shot: end           outcome="타임아웃 - 공이 오지 않음"
```

- `detection_rate`가 0에 가까우면 → colormask/ROI 문제. `tune-colormask` · `detect-full`로 진단
- `triangulated`은 0인데 `detections`는 많으면 → 두 캠이 동시각에 못 잡는다. `skew_p95` 확인
- `rejected`가 많으면 → EKF 마할라노비스 게이트가 측정을 버리는 중 (오검출 또는 캘리브 오차)
- `skew_p95_ms`가 프레임 간격(120 fps ≈ 8.3 ms)보다 크면 → 스테레오 동기가 삼각측량 오차의 주범

---

## 알려진 한계

1. **스테레오 타임스탬프 동기 없음** — `Triangulate::pixels`는 타임스탬프를 무시한다
   (`TODO.md` §3 "멀티캠 동기 — 비범위"). skew를 계측해 남기는 것까지가 현재 범위다.
2. **EKF에 스핀 상태 없음** — `Ekf::predict_to`는 ω=0으로 예측한다 (Magnus 0). sim GT 경로는
   진짜 ω를 쓴다. 실기 예측 오차의 알려진 하한.
3. **stale ready 포즈** — [`defaults/robot.rs`](../defaults/robot.rs)에 기록된 대로
   `READY_JOINTS_4DOF`·`mount_y`가 새 베이스 높이(0.935 m) 기준이 아니라 IK 도달률이
   118/240 → 91/240으로 떨어져 있다. `plan_best` 실패가 잦으면 여기부터 본다.
4. **레일은 단일 절대 이동** — 관절은 200 Hz 스트리밍인데 레일은
   `command_abs_in_secs(follow_through_rail_x, duration_secs)` 한 번이다. 임팩트 시점 레일
   위치가 궤적과 어긋날 수 있다.
5. **macOS는 실기 불가** — `AxlRail::open`이 Windows 전용. `--dry-run`은 `AxlRail::dry_run`을
   타므로 macOS에서도 전 체인 리허설은 된다.
6. **랠리 미지원** — 커밋 래치가 1회고 샷 간 EKF 리셋·연속 포기 정책이 없다.
7. **죽은 카메라의 종료 지연** — `ThreadedCapture::next_frame`이 첫 프레임을 최대 8초 기다리는
   동안 카메라 워커는 셧다운 플래그를 못 본다. 장치 경합이나 device 인덱스 오류면 요약에
   `frames=0`으로 찍히고 `"프레임 소스 종료"` warn이 함께 나온다 — 그때는
   [`defaults::calib`](../defaults/calib.rs)의 `LEFT_DEVICE` / `RIGHT_DEVICE`를 의심한다
   (`cam-list`로 확인).

## 랠리로 넓힐 때

`control_worker`의 커밋 래치를 풀고, `estimator_worker`가 샷 경계에서 EKF를 리셋하면 된다.
`ShotEvent`에 샷 번호를 붙이고 메인의 종료 조건을 바꾸는 것도 필요하다.
sim의 `shot_seq` · `park_if_out_of_play` · `hard_fail_streak`가 참고 대상이다.
