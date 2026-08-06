# Design: 실기 파이프라인 레이턴시 계측 (latency instrumentation)

**작성일:** 2026-08-06
**상태:** 사용자 리뷰 대기
**범위:** `src/real/control_worker.rs`, `src/telemetry/init_tracing.rs`, `Cargo.toml`,
`.gitignore`. 제어 로직·상태 머신·데이터 계약은 바꾸지 않는다 — 기존 로그
호출에 필드를 더하고, 새 tracing 출력 레이어 하나를 추가할 뿐이다.

---

## 배경

`--mode real`에서 로봇이 느리게 움직이거나 공을 놓치는 것처럼 보인다는
관찰이 있었다. 다만 아직 실측은 없다 — 어느 구간(카메라 캡처, 비전 적합,
스레드 간 큐잉, 명령 전송, 실제 모터 이동)이 얼마나 걸리는지 로그로
남아 있지 않다.

먼저 파이프라인 각 구간의 소요 시간을 계측해 실제 병목을 확인하고, 그
수치를 근거로 레이턴시 보상(예: 목표 선택 시 물리 이동 소요 시간을
고려해 도달 불가능한 접수 평면을 거르는 것)을 별도 설계로 진행한다.
**이 spec은 계측만 다룬다 — 보상 로직은 범위 밖이다.**

코드를 살펴보니 `select_alignment_target`(`control_worker.rs:112`)이 이미
`CommitRequest::age_secs()`로 "비전이 궤적을 다 만든 시점 → 제어가 실제로
그 요청을 처리하는 시점" 사이의 큐잉·백프레셔 지연을 보정하고 있다. 이
설계는 그 옆에 있는, 지금까지 로그로 남지 않던 구간들을 채운다.

## 목표 / 비목표

**목표**

- real 모드에서 공 하나(`track_seq`)당 파이프라인 구간별 소요 시간을
  구조화된 필드로 남긴다.
- 이 로그를 콘솔과 별도인 파일에 JSON Lines로 남겨, 매번 터미널 로그를
  복사해 옮기지 않고도 파일을 직접 열어(또는 다음 세션에 그대로) 분석할
  수 있게 한다.
- 기존 제어 흐름·상태 머신·명령 판단 로직은 전혀 바꾸지 않는다.

**비목표 (다음 단계로 보류)**

- 레이턴시 보상 메커니즘 자체(look-ahead 확장, 도달 가능성 기준 접수
  평면 필터링 등) — 실측 수치를 본 뒤 별도 spec.
- 카메라 센서 노출~리드아웃 레벨의 하드웨어 레이턴시 — 하드웨어 트리거
  타임스탬프 없이는 측정 불가.
- 엔코더로 확인된 실제 물리 도달 시각. 지금 `PendingVerification`
  (`control_worker.rs:195`)이 정확히 이 값을 재는 코드지만, 현재 활성
  루프에서는 호출되지 않는 죽은 경로다(주석 참고). 이걸 되살리면
  `idle_ready` 판단 등 상태 머신 동작이 바뀌므로 "계측만" 범위를
  벗어난다. 대신 `hardware.is_busy()`가 꺼지는 시점(§측정 구간의 4번)을
  소프트웨어 추정치로 남긴다.

## 측정 구간

| # | 구간 | 계산식 | 새 계측 필요? |
|---|------|--------|----------------|
| 1 | 카메라 캡처 → 비전 적합 완료 | `request.at - (trajectory.origin + measured.last().t)` | 아니오 — `Trajectory::origin`(`vision/contract.rs:70`)과 `State::t`(같은 파일, "벽시계는 `origin + t`")가 이미 있음. 로그 지점에서 계산만 추가 |
| 2 | 비전 적합 완료 → 제어 처리 시작 | `request.age_secs()` | 아니오 — 이미 존재, 필드로 노출만 |
| 3 | 명령 전송(`hardware.command`/`command_joints` 블로킹 호출) | 호출 전후 `Instant::now()` 차 | 예 — 호출부 감싸기 |
| 4 | 명령 → `hardware.is_busy()`가 `false`로 바뀌는 시점 (소프트웨어 실행기 추정치, 엔코더 확인 아님) | 명령 발행 시각부터 다음 `is_busy() == false` 전이까지 | 예 — 이미 매 루프 틱 폴링되는 `is_busy()` 값에 사후 관찰 하나 추가 |
| 5 | 고정 스윙 타이밍 (`scheduled_lead_secs`, `start_late_ms`) | 이미 계산됨 (`control_worker.rs:399-401`) | 아니오 — 로그 태그만 추가 |

1+2 = "이 공에 대한 정보가 얼마나 오래됐는가"를 나타내는 단일 지표로도
쓸 수 있다(`camera_to_control_recv_ms` = 1번 + 2번).

## 코드 변경

### `src/real/control_worker.rs`

- 본 예측 정렬/팔 보정 명령을 로그하는 기존 `info!`
  (현재 `request_age_secs` 등을 남기는 지점, `control_worker.rs:780` 부근)에
  다음을 추가:
  - `target: "latency"` 태그
  - `camera_to_fit_ms` (구간 1, 위 계산식)
  - `command_send_ms` (구간 3)
  - 기존 `request_age_secs` 필드는 그대로 둔다(하위 호환) — 이것이 구간 2다.
- `hardware.command(&alignment)` / `hardware.command_joints(&alignment)`
  호출을 `Instant::now()`로 감싸 `command_send_ms`를 얻는다.
- 고정 스윙 명령을 로그하는 기존 `info!`(`control_worker.rs:397` 부근)에
  `target: "latency"` 태그만 추가한다 — 이미 필요한 필드
  (`scheduled_lead_secs`, `start_late_ms`)를 갖고 있다.
- 제어 루프 로컬 상태에 `motion_watch: Option<(u64, Instant, &'static str)>`를
  추가한다. 어떤 하드웨어 명령이든(본 정렬·팔 보정·고정 스윙) 발행 직후
  `(track_seq, issued_at, 이벤트 라벨)`로 채운다. 루프가 이미 매 틱
  읽는 `hardware.is_busy()` 값을 확인하는 기존 지점 중 하나에서, `Some`이고
  `is_busy() == false`로 바뀌었으면 `info!(target: "latency", track_seq,
  event, command_to_motion_done_ms, "...")`를 한 번 남기고 `motion_watch`를
  비운다. 새 명령이 이전 watch가 아직 안 끝난 채로 나가면(정상 흐름에서는
  `is_busy()`가 이미 새 명령을 막지만, 방어적으로) 이전 watch는 로그 없이
  버린다 — 없는 진단 한 줄이 틀린 진단 한 줄보다 낫다.
- 새 데이터 구조·에러 타입 없음. 위 값들은 지역 변수로만 존재한다.

### `src/telemetry/init_tracing.rs`

- 현재 `tracing_subscriber::fmt().with_env_filter(filter).init()` 단일
  빌더를, `tracing_subscriber::registry()`에 레이어 두 개를 얹는 형태로
  바꾼다:
  1. 기존 stdout 레이어 — 필터·출력 형식 동일, 동작 변화 없음.
  2. 새 파일 레이어 — `target: "latency"` 이벤트만 통과시키는 필터 +
     JSON Lines 포맷 + 파일 writer.
- `init_tracing`은 `real` 모드 여부를 새 인자로 받는다. `main.rs`는
  `Args::parse()` 직후 `args.mode`를 이미 알고 있으므로 호출부만 바꾸면
  된다. sim(GUI) 모드에서는 파일 레이어를 만들지 않는다 — 계측 대상은
  카메라·하드웨어 파이프라인이고 sim에는 해당하지 않는다.
- 파일을 열지 못하면(`std::io::Error`) `warn!`로 stdout에 한 번 남기고
  파일 레이어 없이 계속 진행한다 — 계측 실패가 실기 제어를 막아서는 안
  된다.

### `Cargo.toml`

- 이미 있는 `tracing-subscriber` 의존성에 `"json"` feature를 추가한다.
  새 crate 의존성은 없다(`serde_json`이 해당 feature를 통해 간접적으로
  딸려 온다).

### `.gitignore`

- `logs/` 추가 — 런타임 산출물이며 소스 관리 대상이 아니다.

## 파일 수명주기

- real 모드 시작 시 `logs/` 디렉터리를 없으면 생성한다
  (`std::fs::create_dir_all`).
- 실행마다 `logs/latency-<유닉스 초>.jsonl` 파일을 새로 연다(런 하나당
  파일 하나 — 이전 테스트 세션 데이터와 섞이지 않는다).
- append 모드는 필요 없다 — 파일명 자체가 실행마다 유일하다.

## 에러 처리

- 파일 열기 실패 → stdout에 `warn!` 한 번, 파일 레이어 없이 계속.
- 파일 쓰기 실패(디스크 풀 등) → `tracing_subscriber`의 파일 writer가
  기본적으로 쓰기 에러를 무시한다. 진단 한 줄을 잃는 것은 괜찮지만,
  그것 때문에 제어 루프가 패닉하거나 멈추면 안 된다.
- 이 설계는 제어 루프의 기존 처리 경로에 새로운 실패 가능 지점을
  추가하지 않는다 — 유일하게 감싸는 호출(`hardware.command`)은 이미
  실패 가능했고, 새로 추가되는 건 그 주위의 **측정**뿐이다.

## 테스트

- 구간 1 계산(`camera_to_fit_ms`)을 순수 함수로 분리하고, 합성
  `Instant`(`Instant::now() + Duration::from_millis(n)`)로 단위 테스트:
  정상 케이스, 지연 0인 케이스, `measured.last()`가 없는 케이스(이미
  상위에서 걸러지므로 패닉하지 않는지만 확인).
- `motion_watch` 하강 엣지 감지를 기존 `Hardware` 테스트 더블로 단위
  테스트: `is_busy()`가 `true → false`로 바뀌는 폴링 시퀀스를 흉내내
  `command_to_motion_done_ms`가 한 번만 로그되는지 확인.
- tracing 레이어·파일 I/O 배선 자체는 단위 테스트 대상이 아니다 — real
  모드(또는 `--dry-run`, 같은 `control_worker` 경로를 모의 하드웨어로
  실행)를 실제로 돌려 `logs/latency-*.jsonl`이 생성되고 기대한 필드가
  채워지는지 수동으로 확인한다.

## 후속 작업 (이 spec 범위 밖)

- 이 계측으로 얻은 실측치를 근거로 레이턴시 보상 설계(별도 spec):
  예를 들어 `select_alignment_target`이 후보 접수 평면을 고를 때 남은
  리드 타임이 예상 이동 소요 시간보다 충분히 큰지 확인하는 게이트.
- 엔코더 확인 기반 실제 도달 시각이 필요해지면 `PendingVerification`
  부활 여부를 별도로 결정한다(`docs/decisions.md`의 열린 과제 참고).
