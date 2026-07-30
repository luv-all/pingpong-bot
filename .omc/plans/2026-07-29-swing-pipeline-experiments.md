# 스윙 파이프라인 실험·개선 계획

**상태**: Phase 0·Phase 1·WP9(커밋 `42b8333`)·WP2b(커밋 `41d8b48`)·WP10(로컬
완료, §0.4 — 프로덕션 코드 변경 없음)·WP3(로컬 완료, §0.5 — 두 신규 공식
모두 기각, 프로덕션 코드 변경 없음)·WP11(로컬 완료, §0.6 — 신규 WP,
사용자 제안, 채택) 완료. Phase 2 잔여는 **WP4a 미착수**.
**기반 문서**: `docs/swing-diagnostic-report.md` (RC-1~RC-4)
**작성일**: 2026-07-29 (최종 갱신 2026-07-30)

---

## 0. 현재 상태 (2026-07-30 세션 종료 시점 — 새 세션 재개용 요약)

**브랜치/커밋**: `main` == `origin/main` @ `9282f62`(fast-forward, 로컬·원격
일치). `cargo build --lib --tests` 클린. `cargo test --lib` **220 passed, 1
failed**(아래 참고), **36 ignored**(진단 테스트 컨벤션).

**알려진 무관 실패 1건**: `hardware::dynamixel::tests::
motor_mapping_matches_python_reference` (`left: 1536, right: 2560`) — 이번
세션이 만든 게 아니라 origin의 `7f9f827`(다른 기여자, `nm_per_goal_current_unit`/
`joint_signs` 변경)에서 이미 깨져 있음을 격리 워크트리로 확인(순정
`origin/main`에서도 동일하게 실패). 이 계획 범위 밖 — 별도 이슈로 다룰 것.

**Phase 0 (전부 완료·머지됨)**: WP8(반사관성), WP5(coarse rate-limit,
레일 가속 제한 포함), WP4b(테이블 관통 — 재현 안 됨, WP5로 해소된 것으로
결론), WP6(접촉 타이밍 — `num_solver_iterations` 12→32로 해소).

**Phase 1 (전부 완료·머지됨)**: WP7(downscale 감사), WP2a(커밋 시간창 —
`min_swing_secs` 0.08→0.20, `swing_commit_max_secs` 0.35→0.60, 67샷 그리드
기준 회귀 없음), WP2c(접촉 오차 허용치 — 스윕 후 **현재값 0.005 유지 결정**,
근거는 `docs/wp2c-contact-tolerance.md`), WP1(y평면 범위 — 스윕 후 **현재값
유지 결정**, 근거는 `docs/wp1-hit-plane-window-sweep.md`), WP4c(base-우선
가중치 검증 — `READY_JOINTS_4DOF` 재계산으로 반영).

**Phase 2 (미착수 — 다음 세션 시작점)**: WP2b, WP4a, WP3. 아래 §4의 각 WP
상세는 원안 그대로 유효하다(계획 변경 없음). 단, WP1·WP2c 실험 중 **이
계획에 없던 두 가지 구조적 문제**가 새로 발견되었고 Phase 2 착수 전 먼저
검토할 가치가 있다 — §4.5 참고.

### 0.1 신규 발견 — Phase 2 착수 전 우선 검토 권장 (계획 밖, 이번 세션 발견)

1. **eval Right 존 커밋률 0%(10/10 실패)** — Left 10/10, Center 10/10인데
   Right만 전멸(`min_seen_error = inf`, IK/기구학 단계에서 이미 탈락).
   WP2c 스윕(14개 허용치 전부에서 동일)과 WP1 스윕 양쪽에서 독립적으로
   재현된 강한 신호. 좌우 비대칭이 이 정도로 크면 레일 도달범위·마운트
   기하 또는 `RailMotion` 실현성 판정(WP5가 지적한 "레일 가속을 플래너가
   못 봄"과 교차 가능)의 비대칭을 의심할 수 있다. WP2b/WP4a/WP3 중 어느
   것을 먼저 하든 이 비대칭이 그 실험 결과를 왜곡할 수 있으므로, **Phase 2
   착수 전 원인 격리를 먼저 하는 편이 안전**하다(근거:
   `docs/wp2c-contact-tolerance.md` §7-1, `docs/wp1-hit-plane-window-sweep.md`).
2. **커밋되는 샷이 전부 eval 1점(접촉은 하지만 네트를 못 넘김)** —
   RC-1/RC-4 계열 약한 리턴 증상. WP2b(치기쉬움+임팩트세기 복합 랭킹)로
   직접 다뤄질 항목이라 별도 조치보다는 **WP2b 우선순위를 올리는 근거**로
   본다(근거: `docs/wp2c-contact-tolerance.md` §7-2).

두 항목 모두 이 계획서에 새 WP 번호를 부여하지 않고 기존 WP(§4.5 참고)의
실행 순서·우선순위 조정 근거로만 반영했다 — 사용자 확인 후 정식 WP로
승격할지 결정.

### 0.2 WP9 — Right 존 커밋률 0% 원인 격리·수정 (2026-07-30, 정식 WP로 승격·완료)

사용자 승인으로 §0.1-1을 정식 WP로 승격해 Phase 2(WP2b/WP4a/WP3) 착수 전
먼저 처리했다.

**원인**: 플래너/기구학 버그가 아니라 **eval 하네스 부트 상태 아티팩트**였다.
`Arm::initial_state()`(`src/robot/arm.rs:192-199`)는 레일을 `home_x()`
(레일 끝단, x=0)에서 시작시키는데, `run_eval_shot`(`src/sim/eval/protocol.rs`)이
샷마다 새 `SimWorld`를 만들어 매번 이 위치에서 시작한다. Left(0.254m)·
Center(0.7625m)·Right(1.271m) 타겟까지 이동 거리가 5배 차이 나, WP5가
추가한 레일 가속 제한(`RAIL_ACCEL_M_S2=12.0`) 하에서 Right만 커밋 시간창
안에 레일이 도달하지 못하고 전부 `"tti < min_swing"`으로 탈락했다.
`Planner::feasibility()`(순수 IK, 이동시간 무관)는 Left·Right가 거의
동일(`peak_joint_speed_ratio≈4.87`)해 마운트 기하·IK 시드 편향(§0.1-1이
의심했던 후보)은 배제 — 실제 로봇은 랠리 사이 항상 `plan_return_to_center`로
`default_x()`(테이블 중앙)에 복귀해 대기하므로 프로덕션 스윙 파이프라인은
영향받지 않는다.

**수정 범위(사용자 결정)**: eval 하네스만 수정, `Arm::initial_state()`
자체는 유지. `run_eval_shot`이 매 샷 전에 로봇 상태를 `rail.default_x()` +
`default_joints`로 리셋하도록 변경(`src/sim/eval/protocol.rs`).

**검증**: `diag_eval_flags_deterministic`(`tests/diag_weak_return.rs`, eval
자신의 러너 `eval::Protocol::run_shot` 경유)로 30샷 전수 재실행 —
Left/Center/Right **전부 30/30 contact=true, cleared_net=true,
returned_in=true**(수정 전 Right는 0/10). `cargo test --lib` 전체 재실행 —
220 passed, 1 failed(§0의 기존 무관 실패와 동일), 36 ignored로 회귀 없음.

**후속 발견(범위 밖, 별도 처리 필요)**: `tests/diag_weak_return.rs`의
로컬 `run_shot` 헬퍼(104행)는 `eval::Protocol`을 거치지 않고 자체적으로
`SimWorld::with_physics`를 직접 호출해 같은 부트 아티팩트를 독립적으로
재현한다(이 진단 테스트 자체가 여전히 Right 10/10 MISS를 보고함) — 이
헬퍼도 같은 방식으로 리셋할지는 사용자 확인 필요. 또한 `diag_weak_return`
스크립트가 계측하는 "커밋 샷 전부 eval 1점" 현상은 `Flags::score()`가
`bounced_own_half`/`double_hit` 발생 시 `returned_in=true`여도 점수를 1로
캡핑하기 때문(`src/sim/eval/flags.rs:23-37`) — 이는 WP9 범위가 아니라
WP2b(치기쉬움+임팩트세기 복합 랭킹)가 다뤄야 할 RC-1 증상으로 그대로
남겨둔다(§0.1-2 근거 유지).

**파일**: `src/sim/eval/protocol.rs`(수정), `tests/diag_wp9_right_zone.rs`
(신규 진단 테스트, `#[ignore]`).

### 0.3 WP2b — 후보 랭킹에 치기쉬움+임팩트세기 반영 (2026-07-30, 로컬 완료)

**채택**: `plan_best_swing`의 타점 후보 정렬을 거리순 → 복합 점수
`score = |v_r·n| × min(1, 1/r)`(`r=peak_joint_speed_ratio`)로 변경. 가중치
없이 실제 축소 코드(`solve_impact_target`의 근특이점 사전축소,
`fit_end_velocity`의 이분탐색)에서 유도된 값. eval+랜덤 55샷 A/B에서 10샷
개선·0샷 회귀(네트통과·인코트율 86.7→100%, Center 존 60→100%), 커밋률·
접촉률은 전 존 100% 그대로(WP9 패리티 유지). IK 시드 랭킹은 실측 근거로
무변경 유지(같은 타점의 시드 간 `|v_r|` 산포가 최대 0.026%뿐이라 기존
`peak_joint_speed_ratio` 단독 기준이 이미 최적). `cargo test --lib` 227
passed/1 known-fail/38 ignored — 회귀 없음(베이스라인도 227/1/36, ignored
+2는 신규 진단 테스트).

**중요 — §0.1-2 가설은 반증됨**: 랭킹 변경으로 "커밋 샷 전부 eval 1점"
증상은 고쳐지지 않았다(`bounced_own_half` 100%→100%, eval 점수 30/90→30/90
동일). 판별 계측(`Impact::clears_net`) 결과: 플래너가 원하는 출사속도는
100% 네트를 직접 넘지만(조준은 정상 — WP3가 원인이 아님을 시사), **실측
출사속도는 0%가 직접 통과**한다. 방향은 그대로 두고 크기만 desired로
올리면 100% 통과 — 즉 원인은 순수한 **세기 부족(`|v_out|/desired≈0.67`,
약 1.5배)**이지 랭킹·조준 문제가 아니다. hit plane의 50~70%가 quintic
단계에서 "위치 이동 자체가 관절속도 예산을 다 써서" 탈락하고, 남는
후보들도 서로 비슷하게 약해 랭킹으로는 이 격차를 못 줄인다.

**후속 과제로 제시된 것**(정식 WP 승격 여부 사용자 확인 필요):
(1) 커밋 시점 Δq를 줄이는 coarse 추종 강화(WP4c의
`COARSE_TRACK_JOINT_FRACTION` 관절별 차등), (2) `min_swing_secs` 확대로
이동 시간 자체를 늘리는 방안, (3) `max_joint_speed`가 진짜 하드웨어 상한이면
랠리 리턴 타겟을 더 가깝게 잡아 필요 `|v_out|`을 낮추는 절충(WP3와 연계),
(4) 랭킹이 바뀌었으니 WP1 y평면 스윕 재실행(단 세기 병목이 남아있는 한
eval 점수 대신 네트통과율을 지표로 써야 함).

**파일**: `src/robot/motion/physics.rs`, `src/robot/motion/impact_target.rs`,
`src/robot/motion/impact_candidate.rs`(수정), `tests/diag_wp2b_composite_ranking.rs`
(신규). 상세: `docs/wp2b-composite-ranking.md`.

### 0.4 WP10 — coarse 추종 관절별 차등 검토 (2026-07-30, 기각·프로덕션 무변경)

WP2b §7-1이 제안한 첫 레버("커밋 시점 Δq를 줄이면 세기가 는다")를 실측으로
검증했고 **기각했다** — `COARSE_TRACK_JOINT_FRACTION`(현재 균일 0.80)은
그대로 유지, 프로덕션 코드 변경 없음(진단 계측 추가만).

**Δq 예산을 먹는 관절은 확인됨**: 라이브 eval 30샷 커밋 틱 계측
(`diag_wp10_commit_time_joint_speed_blame`)에서 탈락 90건 전부 q2(elbow)가
travel 최댓값(이동만으로 이미 한계의 1.206배), q0(base yaw)가 0.903배로
다음. q1·q3는 거의 안 씀.

**그런데 이 병목은 세기가 아니라 후보 생존 수만 정한다**: 통과 평면의
`fit_end_velocity` 실측 배율은 평균 0.981(거의 무손실)인 반면, **270개
후보 전부**가 `NEAR_SINGULARITY_SPEED_RATIO`(2.5)를 넘어(평균 r=4.114)
`impact_target_from_candidate`의 사전축소가 평균 **1/r=0.275**를 곱한다.
즉 세기 손실은 `사전축소 0.275 × quintic 0.981`로 분해되고, coarse
추종이 건드릴 수 있는 건 뒤쪽 0.981뿐 — **완벽한 선추종으로도 상한이
+1.9%**다(필요한 건 1.5배). 사전축소는 시작 자세(Δq)와 무관하게 임팩트
자세의 자코비안 조건수만으로 정해진다. 8개 스킴 A/B(eval 30+랜덤 5×5)도
`|v_out|/desired` 0.6681~0.6685(산포 0.06%)로 완전히 평평함을 확인.

**의미**: 세기 1.5배 격차의 진짜 레버는 **사전축소 1/r**, 즉 요구 임팩트
속도 대비 관절속도 한계의 비율이다 — WP2b §7의 나머지 항목(랠리 리턴
타겟을 가깝게 잡아 필요 `|v_out|` 낮추기 = WP3, 도달 가능 임팩트 자세
넓히기 = WP4a)로 우선순위가 좁혀졌다. `min_swing_secs` 확대(§7-2)는
우선순위 하향 — quintic이 이미 98%를 보존해 시간을 더 줘도 세기를 못 산다.

**파일**: `src/sim/physics/world.rs`(진단 계측 + doc comment, 상수 값
불변), `src/robot/motion/physics.rs`(`trajectory_with_follow_through`
가시성만 `pub(crate)`로), `tests/diag_wp10_coarse_track_per_joint.rs`(신규).
상세: `docs/wp10-coarse-track-per-joint.md`.

### 0.5 WP3 — 랠리 리턴 목표 거리·공식 재검토 (2026-07-30, 기각·프로덕션 무변경)

WP10 §7이 제안한 다른 레버("목표를 가깝게 잡아 필요 `|v_out|` 낮추기")와
사용자의 별도 지적("깊은 코트 지점이 아니라 네트를 넘기는 게 목표여야
함")을 각각 구현·실측했고 **둘 다 기각**했다 — `rally_return_velocity`는
원래 고정목표 공식으로 완전히 복귀(byte-identical), 프로덕션 경로 무변경.

**목표 거리 축소(y_frac 0.75→0.55 스윕)**: `r_mean` 2.076→2.729로 단조
악화, y_frac=0.55에서는 네트클리어 자체가 10%로 붕괴 — WP10의 가설과
정반대 결과.

**최소속도 네트클리어 공식(고전 탄도학 최소발사속도 공식으로 신규
구현)**: `|v_r|`(요구 라켓속도 크기)은 비슷하거나 살짝 낮췄지만(1.785→
1.846) `r`은 오히려 나빠졌다(2.076→2.721). 결정적 단일 사례 분석 —
두 공식의 `v_r`이 크기는 비슷한데 **y성분 부호가 반대**로 나와, 같은
임팩트 포즈의 자코비안에서 한쪽 방향은 base yaw(q0)에 유리하고 반대
방향은 불리했다(q0 −6.95→−10.53 rad/s, r 2.41→3.66).

**핵심 발견**: `r`을 좌우하는 건 요구 속도의 **크기가 아니라 방향**이고,
방향은 IK가 고르는 포즈의 자코비안 조건수와 상호작용한다. "v_out/v_r
공식을 바꿔 크기를 줄인다"는 레버 전체가 `r`에 대해 **잘못된 대리
지표를 최적화**하고 있었다 — WP3(구·신 양쪽 모두)뿐 아니라 애초에 WP10이
제안한 방향 자체가 이 발견으로 재평가돼야 한다. 남은 유효 레버는 포즈
탐색을 넓히는 WP4a와, 같은 포즈에서 관절 배분을 최적화하는 신규
WP11(§0.6 진행 중, 사용자 제안)이다.

**파일**: `src/estimator/impact.rs`(`rally_return_velocity_min_effort`
신규 함수 추가·기각된 채로 보존, `rally_return_velocity_fixed_point`로
기존 로직 이름 변경, 프로덕션 경로 무변경), `src/defaults/impact.rs`
(`rally_target_y_frac` 필드 추가, 스윕 파라미터). 상세:
`docs/wp3-rally-target-distance.md`.

### 0.6 WP11 — 임팩트 속도 IK 널스페이스 자체운동 재배분 (2026-07-30, 신규·채택)

사용자 질문("방향만 가지고 세기를 min(robot_max, required)로 클램프하면
되지 않아?")에서 시작 — 실제로 `impact_target_from_candidate`의 `1/r`
사전축소가 그 클램프였지만, 기준으로 삼는 `joint_velocities`가 4관절-3제약
널스페이스(1차원, 자체운동)를 전혀 안 쓰는 가중 최소노름 해 하나뿐이었다.
`Arm::linear_velocities_for_racket_velocity`에 자체운동 최적화(4차원
외적으로 널벡터 계산 + 삼분탐색으로 피크 관절속도 최소화, 레일은 그대로
두고 팔 관절 4개만) 추가.

**실측**: 원시 후보 풀 기준 `r_mean` 2.076→1.871(fixed@0.75), WP3에서
기각됐던 `min_effort`도 2.721→**1.871**(fixed@0.75와 소수점까지 일치 —
자체운동이 WP3가 발견한 "같은 크기, 반대 방향이 포즈에 불리했던" 효과를
정확히 상쇄). `diag_joint_utilization_at_impact_peak` 재실행 — 한 관절
포화·나머지 유휴이던 불균형이 사라지고, 여러 행에서 q0·q3가 정확히 같은
값에서 공동 최댓값(minimax 최적의 교과서적 신호). 라이브 eval 그리드
(WP2b 랭킹 이후)에서는 `\|v_out\|/desired` 0.669→0.673로 소폭 개선(회귀
없음, 커밋률·접촉률·네트통과율·인코트율 전부 100% 유지) — WP2b가 이미
낮은 `r` 후보를 골라놔서 최종 선택 후보에는 개선 여지가 적었다는 뜻으로,
WP2b(랭킹)와 WP11(같은 포즈 내 배분)이 서로 다른 축을 다루기 때문.
4개 단위테스트로 널벡터 공식·자체운동 정확성 검증(손계산 가능한 사례
포함).

**의미**: `r`을 낮추는 세 번째 독립 레버(WP3=요구크기 축소는 기각, 이건
"같은 포즈에서 배분 최적화") 확보. WP4a(포즈 자체를 더 찾기)와는 상보적 —
곱 효과 가능성 있음(§0.6 "후속 과제" 참고).

**파일**: `src/robot/arm.rs`(`linear_velocities_for_racket_velocity` 수정,
`arm_joint_null_vector`·`minimize_peak_via_self_motion` 신규, 단위테스트
4개). 상세: `docs/wp11-nullspace-self-motion.md`.

---

## 1. 요구사항 요약

사용자가 진단 보고서를 검토하고 8개 작업 항목을 지정했다. 각 항목을 진단 보고서의
근본원인(RC-1~RC-4)과 매핑하고, "실험으로 먼저 확인 후 필요시 수정"이 필요한 항목과
"이미 원인이 명확한 버그 수정" 항목을 구분했다.

| WP | 사용자 항목 | 관련 RC | 유형 |
|----|---|---|---|
| WP1 | 타점 y평면 범위 최적화 | RC-1 주변 | 실험 |
| WP2a | 커밋 시간창 적절성 검증 | RC-1 | 실험 |
| WP2b | 후보 랭킹에 "치기 쉬움"+"임팩트 세기" 반영 | RC-1 | 설계변경 |
| WP2c | 접촉점 오차 허용치(5mm) 재검토 | RC-1 인접 | 실험 |
| WP3 | 좌우 중앙 타겟팅 필요성 검토 | 신규 | 분석 |
| WP4a | IK를 elbow-up 단일 config로 제한 | RC-1 인접 | 설계변경(위험 존재) |
| WP4b | 테이블 관통 로직 수정 | 신규(관측됨) | 버그수정 |
| WP4c | 최소노름 base-우선 가중치 검증 | RC-1/RC-2 교차 | 진단 |
| WP5 | coarse 단계 sim rate-limit | **RC-2** (확인된 버그) | 버그수정 |
| WP6 | 접촉 타이밍 불일치 원인·해결 | **RC-3** | 진단+수정 |
| WP7 | downscale 반복 감사 | RC-1 | 감사+최적화 |
| WP8 | 토크 판단에 회전자 반사관성 포함 | **RC-4** | 모델확장 |

---

## 2. 사전 확인된 사실 (이번 계획 수립 중 코드 재확인)

- `MAX_CONTACT_ERROR = 0.005`(`src/planner/swing/physics.rs:363`) vs 라켓 반너비
  `RACKET_HALF_X=0.075m, RACKET_HALF_Y=0.08m`(`src/constants/geometry.rs:11-12`),
  공 반지름 `BALL_RADIUS=0.02m`(`src/constants/ball.rs:4`) — 현재 허용치는 반너비의
  **6.7%**에 불과해 사용자 지적이 타당하다.
- `linear_velocities_for_racket_velocity`(`src/robot/mod.rs:909-962`)에 이미
  `τ_limit⁴` 가중 최소노름(요·어깨가 더 많이 쓰이도록)이 구현돼 있다(`607790e`). 그런데
  진단 보고서 §3.6 실측은 shoulder 명령속도 0.000 rad/s, elbow 2.736 rad/s로 정반대
  결과를 보인다 — **가중치 자체가 아니라 다른 단계(RC-2 coarse 선추종)가 효과를
  상쇄했을 가능성**이 있다(WP4c로 분리).
- `robot_obbs`(`src/planner/collision.rs:96-97,103`)는 상완(mount→joint1) 구간을
  주석에 "테이블 끝 마운트에 붙어 겹칠 수 있어 제외"라고 명시하며 **의도적으로
  충돌 검사에서 뺀다** — 사용자가 관측한 관통의 유력 후보다.
- `.omc/research/dynamixel-specs.md`에는 기어비(MX-64 200:1)는 있지만 **회전자
  관성값은 없다** — WP8은 추가 조사가 선행돼야 한다.
- coarse 단계 레일("linear motor")은 `RobotState::step_commands`(`src/robot/state.rs:
  215-231`)에서 이미 `rail.max_speed`로 속도 제한은 되지만 **가속도 제한
  (`RAIL_ACCEL_M_S2=12.0`)은 적용되지 않는다** — 매 틱 순간적으로 최대속도로
  점프한다. 관절 쪽은 진단 보고서가 이미 밝힌 대로 **아무 제한이 없다**
  (`step_toward_targets`가 죽은 코드).

---

## 3. 실행 순서 (Phase)

여러 WP가 서로의 측정값에 영향을 준다. 순서를 지키지 않으면 나중에 다시 측정해야 한다.

```
Phase 0 — 구조적 결함 수정/모델 확장 (측정값을 바꾸는 것들, 먼저 처리)
  WP8  RNEA에 반사관성 추가        ← 모든 토크 관련 측정의 전제
  WP5  coarse rate-limit 복원      ← 모든 "치는 순간" 측정의 전제 (RC-2)
  WP4b 테이블 관통 로직 수정        ← WP5로 먼저 재현되는지 확인 후 필요시 착수
  WP6  접촉 타이밍 불일치 진단·수정  ← RC-3, 임팩트 시점 측정의 전제

Phase 1 — 재측정 (Phase 0 이후의 정상화된 시뮬 위에서 실험)
  WP7  downscale 감사
  WP2a 커밋 시간창 스윕
  WP2c 접촉 오차 허용치 스윕
  WP1  y평면 범위 스윕
  WP4c base-우선 가중치가 실제로 작동하는지 검증 (H1 기하 vs H2 coarse 상쇄)

Phase 2 — Phase 1 데이터로 설계 변경
  WP1  InterceptWindow 갱신 (또는 현재값 유지 근거 기록)
  WP2c MAX_CONTACT_ERROR 갱신
  WP2b 후보 랭킹 기준 변경 (치기 쉬움 + 임팩트 세기)
  WP4a elbow-up 단일 config 제한 (A/B 검증 필수)
  WP3  좌우 중앙 타겟팅 분석 (독립적 — 아무 때나 가능하나 여기 배치)
```

각 Phase는 순차 승인 대상이다. Phase 0 완료·검증 없이 Phase 1 실험을 시작하면
결과가 재작업 대상이 된다.

---

## 4. 작업 패키지 상세

### WP8 — 토크 실현 판단에 회전자/기어박스 반사관성 포함 (RC-4)

**현재 상태**: `peak_torque_utilization`(`physics.rs`)·`drive_arm_motors`
(`world.rs:1166-1191`)가 의존하는 `required_joint_torques_into`(`src/robot/dynamics.rs:
82-169`)는 강체 링크 관성만 쓰는 RNEA다. 모터 자체의 회전자·기어박스가 반사하는
관성(`I_reflected = I_rotor · gear_ratio²`)이 빠져 있다 — 기어비 200:1(MX-64)이면
작은 `I_rotor`도 `×40000`으로 증폭돼 무시 못 할 크기일 수 있다. 유일한 안전마진은
`CONTINUOUS_TORQUE_DERATE = 0.5`(`src/defaults/dxl_limits.rs:10`)이며 이것도
"실측 확인 필요"로 명시된 값이다.

**단계**:
1. MX-64/MX-28 회전자 관성값 조사 — Robotis e-manual에 직접 없으면 유사 클래스
   서보 스펙·모터 상수(kt)에서 역산 추정, 조사 결과와 근거를 `.omc/research/
   dynamixel-specs.md`에 추가(기존 컨벤션대로 출처 URL 명시, 없으면 "추정치·
   실측 필요" 명시).
2. `dynamics.rs`에 `τ_reflected,i = I_reflected,i · q̈_i` 항을 RNEA 결과에
   더하는 래퍼 추가(기존 `mass_matrix` 등 다른 소비처의 의미는 건드리지 않도록
   피드백게이트 전용 함수로 분리).
3. 단위테스트: 반사관성 항이 `gear_ratio²`에 비례해 스케일하는지 검증.
4. `CONTINUOUS_TORQUE_DERATE` 재검토 — 실제 물리항이 반영됐으니 임의 마진을
   줄일 근거가 생기는지 확인(마찰은 여전히 미포함이므로 완전히 없애지는 않음).

**수용 기준**: 회전자 관성값 출처 명시(또는 "추정" 플래그) / 대표 스윙에서
반사관성 항 추가 전후 `peak_torque_utilization` 비교 기록 / derate 재산정 근거
`docs/measure-physics.md`에 기록.

**파일**: `src/robot/dynamics.rs`, `src/defaults/dxl_limits.rs`,
`.omc/research/dynamixel-specs.md`

---

### WP5 — coarse 단계 sim rate-limit (관절 + 레일)

**버그**: `try_auto_swing`의 coarse 분기(`world.rs:637-655`)가 `set_targets`를
직접 불러 Rapier 모터에 **무제한 스텝 입력**을 준다. 관절 rate-limit을 가진
`step_toward_targets`(`src/robot/state.rs:335-364`)는 프로덕션에서 호출되지
않는 죽은 코드이며, `world.rs:624-626, 641-644`의 주석("실제 이동은
rate-limited 추종 루프가 처리")은 사실이 아니다. 레일은 `state.rs:215-231`에서
속도 제한(`rail.max_speed`)만 있고 가속도 제한(`RAIL_ACCEL_M_S2`)은 없다.

**변경**:
1. (관절) coarse 목표 갱신에 `arm.max_joint_speed` 기반 슬루 제한을 적용 —
   `step_toward_targets`를 실제로 배선하거나 coarse 분기에 동등한 제한을
   인라인 추가.
2. (레일) `state.rs:215-231`의 rail-only 분기에 `RAIL_ACCEL_M_S2` 기반
   가속도 제한 슬루잉 추가(사다리꼴 프로파일 근사).
3. `world.rs:624-626, 641-644`의 잘못된 주석 수정.
4. `COARSE_TRACK_JOINT_FRACTION`(`world.rs:167`) 스윕을 반드시 재실행 —
   기존 스윕(`world.rs:134-166` doc comment)은 무제한 스텝 입력 기준으로
   측정된 값이라 rate-limit 적용 후에는 무효.

**수용 기준**: `diag_motor_tracking`의 `peak_err_all`이 465 mrad → 스윙 중
오차(≤10 mrad)와 같은 자릿수로 감소 / 커밋률(`diag_swing_commit_rate_across_
shot_grid`)이 현재 대비 회귀하지 않음(또는 트레이드오프를 문서화하고
`COARSE_TRACK_JOINT_FRACTION` 재튜닝으로 상쇄) / 잘못된 주석 제거.

**파일**: `src/sim/physics/world.rs`, `src/robot/state.rs`

**결과 (2026-07-29, worker-wp5)**: `peak_err_all` 465→16 mrad(29배 개선,
q0/q1/q2/q3 전부 스윙 중 오차와 같은 자릿수). 커밋률은 91%→**76%**로 저하—
4-way ablation으로 원인을 정확히 특정: 관절 슬루(RC-2 버그 수정 본체)는
100%→99%(−1샷)만 깎고, `clamp_above_table`은 0pp, **레일 가속 제한
(`RAIL_ACCEL_M_S2`) 단독이 −23pp 전부를 설명**한다. 근본 원인은 하드웨어
스펙 자체의 모순: `RAIL_MAX_SPEED=5.0` m/s·`RAIL_ACCEL_M_S2=12.0`이면
최고속 도달에 왕복 2.08 m가 필요한데 레일 전장(`WIDTH_X`)은 1.525 m —
**레일은 어떤 이동에서도 최고속에 닿을 수 없다.** 예전 시뮬은 한 틱 만에
5 m/s로 순간이동해 낙관적이었을 뿐, 76%는 "시뮬이 정직해진 결과"다.
`COARSE_TRACK_JOINT_FRACTION`은 재스윕 결과 0.65~1.00 구간이 완전히
평평해(전부 76%) 더 이상 커밋률의 지배 인자가 아님을 확인, 0.80 유지.

**사용자 결정 (2026-07-29)**: 정직한 76% 커밋률을 유지한다(레일 가속 제한을
빼고 예전의 비현실적 순간이동으로 되돌리지 않음) — 시뮬이 실제 AXL 레일
하드웨어 한계를 반영하는 쪽을 택했다. 부수 발견(WP2a와 연계 필요):
`kinematic_limit_violation`(`physics.rs:805-820`)이 레일 **속도**만 검사하고
**가속도**는 검사하지 않아, 플래너가 여전히 레일이 실제로 못 내는 이동을
계획하고 시뮬이 커밋 시점에 뒤늦게 거부하는 구조적 불일치가 있다. Phase 1
후속 과제로 편입:
1. 실물 AXL 레일에서 `RAIL_ACCEL_M_S2`/`RAIL_MAX_SPEED` 재실측 (둘 중 하나가
   틀렸을 가능성 — 서로 모순되는 스펙).
2. 커밋 시간창(`swing_commit_max_secs`) 확대 검토 — **WP2a에 편입**.
3. 플래너의 `RailMotion` 자체에 가속 한계를 반영해 애초에 실행 불가능한
   레일 이동을 계획하지 않게 함 — WP2a 실험과 함께 검토.

상세 측정 기록: `.omc/notepads/2026-07-29-swing-pipeline-experiments/wp5-coarse-rate-limit.md`.

---

### WP4b — 테이블 관통 자세 회피 로직 수정

**후보 원인**(우선순위순, 순서대로 검증):
1. **WP5 미적용 상태의 coarse 무제한 스텝**이 `clamp_above_table`을 거치지
   않고 팔을 테이블 관통 자세로 밀어넣었을 가능성 — **WP5 완료 후 재현되는지
   먼저 확인**.
2. `robot_obbs`(`collision.rs:96-97,103`)가 상완 구간을 통째로 제외 —
   마운트 인근이 아닌 개방된 테이블 공간으로 상완이 들어가는 자세에서는
   관통이 검사되지 않는다.
3. `trajectory_collision_free`(`physics.rs`)가 5ms 간격 샘플이라 빠른 스윙
   중 순간적 관통을 놓칠 수 있음.

**단계**: (1) WP5 이후 재현 시나리오로 재검증 → 여전히 발생하면 (2) 상완
구간을 마운트 인근만 제외하도록 검사 범위를 좁히거나 별도 "마운트 클리어런스
존"을 도입 → (3) 그래도 남으면 충돌 샘플링 해상도 상향 또는 샘플 사이 해석적
경계 추가.

**수용 기준**: 관통이 재현되는 시나리오(시드 고정)를 저장 → 수정 후 해당
시나리오 및 전체 eval+랜덤 그리드에서 `table_penetration ≤ 1e-4` / 기존
`deep_racket_penetrates_table`, `default_pose_clears_table`(`collision.rs:
207-240`) 테스트 유지 통과.

**파일**: `src/planner/collision.rs`, `src/sim/physics/world.rs`

**결과 (2026-07-30, team-lead)**: 후보 1을 재검증한 결과 **재현되지 않는다.**
`tests/diag_table_penetration_live.rs`를 신설해 WP5 적용 후 eval 30샷 전수 +
좌우(±0.5m)×yaw(±15°) 5×5=25 랜덤 그리드를, **명령된 목표가 아니라 Rapier가
실제로 추종한 관절각**을 매 물리 틱(1ms)마다 샘플링해 `table_penetration`을
측정 — 55개 시나리오 전부 `worst_depth = 0.00000`. 계측 자체의 정확성은
양성 대조군(`diag_table_penetration_live_positive_control`, 테이블 아래
지점으로 강제 IK한 자세에서 depth=0.058m 정상 검출)으로 확인했다. 즉
사용자가 관찰한 관통은 WP5가 고친 RC-2(coarse 무제한 스텝 입력)가 원인이었을
가능성이 높고, WP5로 이미 해소된 것으로 보인다.

후보 2(`robot_obbs`의 상완 구간 제외)는 여전히 코드에 남은 구조적 가정이지만,
위 55개 시나리오 전 구간·전 틱에서 관통이 전혀 발생하지 않아 **현재 이 가정이
실제로 문제를 일으킨다는 근거가 없다** — 억지 수정은 하지 않고 계획서·진단
보고서에 "미확인 잔여 가정"으로만 남긴다. 후보 3(5ms 샘플링 해상도)도 이번
검증이 1ms(매 틱) 해상도로 이미 확인했으므로 기각.

기존 `deep_racket_penetrates_table`, `default_pose_clears_table` 포함 전체
218개 lib 테스트 유지 통과.

---

### WP6 — 접촉 타이밍 불일치 (RC-3) 원인 분석·해결

**현상**: `swing_bench --sim-verify` 실측에서 실제 Rapier 접촉이 계획된
`impact_time_secs`보다 4.7ms 이르다.

**가설**:
- (a) 기하: `predict_hit_plane`(`estimator/ballistics.rs`)이 라켓을 점으로
  취급해 y평면 교차 시각을 구하지만, 실제 라켓 콜라이더는 15×16cm 박스이고
  **움직이며** 접근한다 — 스윙 가속 구간에서 면이 빠르게 다가올수록 "면의
  가장자리가 먼저 닿는" 시점이 점-평면 교차보다 이르다.
- (b) 예측기 적분(`ballistics.rs`의 반암시적 오일러, `est.integrate_dt`)과
  Rapier의 실제 적분(dt=1ms, 12 solver iteration) 사이의 누적 오차.
- (c) `best_impact_candidate`의 `racket_center` 오프셋(`physics.rs:156-160`)이
  법선 방향 오프셋(공 반지름+면 반두께)만 고려하고, 이동 방향의 면 스윕은
  고려하지 않음.

**단계**: 대표 스윙 여러 건에서 매 틱 (라켓 중심 위치/속도, 공 위치, 실제
`ContactPair` 발동 틱)을 계측해 오차가 (a)/(c) 기하 스윕(속도에 비례해야
함)과 (b) 적분 드리프트(속도와 무관해야 함) 중 어느 쪽 패턴을 보이는지
구분. 원인에 따라: 기하형이면 접촉 판정 로직을 실제 Rapier `ContactPair`
기준으로 재정의하거나 면 반경만큼 리드타임 보정, 적분형이면 예측기·Rapier
적분 파라미터 정합.

**수용 기준**: 근거 데이터로 원인 확정 / 대표 그리드에서 `|실제접촉 −
계획impact_time|`을 스윙 지속시간의 1% 이내(또는 1ms 이내)로 축소 /
`swing_bench --sim-verify`로 재검증.

**파일**: `src/estimator/ballistics.rs`, `src/planner/swing/physics.rs`,
`src/sim/physics/world.rs`

---

### WP7 — downscale 반복 감사 (RC-1 보조)

**대상**: (1) `NEAR_SINGULARITY_SPEED_RATIO`(2.5) 초과 시 `1/ratio` 1회
균일 축소(`physics.rs:241-274`), (2) `fit_end_velocity`의 최대 32회 반복,
매 반복 `min(speed_scale,accel_scale,torque_scale)×0.95` 균일 축소
(`physics.rs:829-886`). 두 단계가 같은 스윙에 중첩 적용될 수 있고, 반복
횟수가 늘수록 `0.95ⁿ` 마진이 누적돼 불필요하게 보수적일 수 있다.

**단계**: (Phase 0 완료 후) eval+랜덤 그리드 전수에 대해 계측 추가 —
(i) NEAR_SINGULARITY 스케일 발동 여부·계수, (ii) `fit_end_velocity` 수렴까지
반복 횟수·최종 스케일, (iii) 두 단계 동시 발동 빈도·중첩 배율. 반복 횟수가
평균적으로 많다면(예: >3회) 토크/속도 이용률로부터 스케일을 해석적으로
한 번에 계산하는 방식으로 교체 검토, 또는 두 단계가 사실상 같은 제약(관절
속도 한계)을 이중 적용하고 있다면 단일 스케일 계산으로 통합.

**수용 기준**: 계측 테이블(스케일 발동률, 평균 반복수, 중첩 배율) 확보 /
불필요한 보수성이 확인되면 수정 후 A/B로 달성 속도(v_r·n) 개선 검증, 확인
안 되면 "현재 방식이 병목이 아님"을 데이터로 기록하고 종료.

**파일**: `src/planner/swing/physics.rs`

---

### WP2a — 커밋 시간창(`min_swing_secs=0.08`, `swing_commit_max_secs=0.35`) 검증

**질문**: 이 경계가 "그 이상이면 어떤 방법으로도 실행 불가능"한 실측 경계인지,
아니면 과거 추정치인지.

**실험**: (Phase 1, WP5/WP6/WP8 이후) 대표 임팩트 목표를 고정하고
time-to-impact를 0.03~0.6초 범위에서 세밀히 스윕, 각 값에서 `plan_swing` +
`swing_bench --sim-verify`로 IK/궤적 성공 여부, 추종 오차, 달성
`v_r·n`, 토크 여유를 기록(`bang_bang.rs`의 `diag_ball_speed_feasibility_
sweep` 패턴을 quintic 경로용으로 재구현).

**수용 기준**: time-to-impact 대 실행가능성/스윙품질 표·그래프 확보 /
현재 경계값을 유지하거나 데이터 근거로 갱신, `docs/`에 기록.

**파일**: `src/planner/swing/physics.rs`(신규 진단 테스트), `src/defaults/control.rs`

**범위 추가 (WP5 핸드오프, 2026-07-29)**: WP5가 레일 가속 제한(정직한 76%
커밋률로 확정, 사용자 승인)을 적용하며 발견한 두 항목을 이 실험에 포함한다.
(1) `kinematic_limit_violation`(`physics.rs:805-820`)이 레일 **속도**만
검사하고 **가속도**는 검사하지 않아, 플래너가 여전히 레일이 실제로 못 내는
이동을 계획하고 시뮬이 커밋 시점에야 거부하는 구조적 불일치가 있다 —
time-to-impact 스윕 결과에 이 항목을 반영해, 레일 가속 한계를
`kinematic_limit_violation`(또는 `RailMotion` 생성 자체)에 넣는 것이
커밋률에 미치는 영향을 함께 측정한다. (2) `RAIL_MAX_SPEED=5.0`·
`RAIL_ACCEL_M_S2=12.0`이 레일 전장(1.525 m)과 모순되는 스펙이라는
사용자 결정 보류 항목 — 실측 전까지는 두 상수 중 하나가 틀렸다는 전제로
민감도만 확인(재실측은 이 실험 범위 밖, 하드웨어 접근 필요).

---

### WP2c — 접촉점 오차 허용치(`MAX_CONTACT_ERROR=0.005`) 재검토

**근거**: 라켓 반너비 `0.075/0.08m` 대비 5mm는 6.7% — 라켓 면 안에 들어오는
많은 후보를 불필요하게 기각할 가능성.

**실험**: (Phase 1) `MAX_CONTACT_ERROR`를 0.005~0.04m 범위로 스윕하며
eval+랜덤 그리드에서 커밋률·점수·(참고용으로) 실제 Rapier 접촉 성공률·법선
편차를 측정. 라켓 반너비−공 반지름(약 0.055~0.06m)을 상한으로 잡는다.

**수용 기준**: 스윕 데이터로 값 선정(예: "라켓 반너비 최솟값 − 공 반지름의
1/2" 같은 명시적 공식) 및 대표성 확인(과도하게 벗어난 접촉이 실제 Rapier
콜라이더에서도 유효 접촉으로 이어지는지) / `docs/`에 근거 기록.

**파일**: `src/planner/swing/physics.rs:363`

---

### WP1 — 타점 y평면 범위(`InterceptWindow::default()`) 최적화

**현재**: `y_min=0.08, y_max=0.35, sample_step=0.03`(`src/defaults/planner.rs:
27-36`) → 10개 평면.

**실험**: (Phase 1, 반드시 WP5/WP6/WP8 이후 — 그 전 데이터는 무효) `y_min ∈
{0.05, 0.08, 0.12}`, `y_max ∈ {0.25, 0.35, 0.45}`, `step ∈ {0.02, 0.03,
0.05}`의 조합(또는 축소된 부분집합)을 eval 30샷 그리드 + 랜덤 슈터 그리드로
평가, 커밋률·접촉률·달성 `v_r·n`·eval 점수 기록.

**수용 기준**: 최소 3×3×2 조합 이상 스윕 / 현재값 대비 개선이 확인되면
`InterceptWindow::default()` 갱신, 아니면 현재값이 최적임을 데이터로 기록 /
`docs/`에 표로 기록.

**파일**: `src/defaults/planner.rs`, 신규 진단 테스트

---

### WP4c — 최소노름 base-우선 가중치가 실제로 작동하는지 검증

**배경**: 가중치(`τ_limit⁴`)는 이미 구현돼 있으나(`robot/mod.rs:909-962`)
실측(진단 보고서 §3.6)에서 shoulder=0, elbow 지배. 두 가설: **H1**(기하) —
해당 스윙의 목표 방향에 대해 shoulder 자코비안 열이 원래 기여가 거의 0이라
가중치로도 못 살림. **H2**(RC-2 교차) — coarse 선추종이 커밋 전에 이미
shoulder를 목표 근처로 옮겨놔 커밋 시점 Δq 자체가 없음.

**실험**: (Phase 1, WP5 이후 — coarse 동작이 바뀌므로) 대표 시나리오 ≥5개에서
(i) `linear_velocities_for_racket_velocity`의 원시 관절속도 출력, (ii) 커밋
시점 실제 필요 Δq(코스 추종이 이미 옮겨놓은 만큼 제외)를 함께 기록.

**수용 기준**: H1/H2/혼합 여부를 데이터로 확정 / H2 확인 시 관절별
`COARSE_TRACK_JOINT_FRACTION` 차등 적용(예: base/shoulder는 낮게, wrist는
높게) 등 후속안 제시 및 A/B 검증 / H1이면 "가중치는 정상 작동, 기하학적
한계"로 문서화 후 종료.

**파일**: `src/robot/mod.rs`, `src/sim/physics/world.rs`

---

### WP2b — 후보 랭킹에 "치기 쉬움" + "임팩트 세기" 반영

**현재**: `plan_best_swing`은 현재 위치와의 거리로만 예측(타점)을 정렬
(`physics.rs:376-382`); `best_impact_candidate`는 IK 시드를
`peak_joint_speed_ratio`(치기 쉬움에 해당) **단독**으로만 랭킹
(`physics.rs:210-213`) — "임팩트 세기"는 반영되지 않는다.

**설계**: 복합 점수 도입 — (a) 치기 쉬움 = 기존 `peak_joint_speed_ratio`
(낮을수록 좋음), (b) 임팩트 세기 = 달성 가능한 `v_r·n`(또는 근특이점
다운스케일 배율의 역수 — 이미 "얼마나 스윙이 약해졌는지"를 직접 나타내는
값). 두 층(타점 간 랭킹, IK 시드 간 랭킹) 모두에 적용할지 결정 필요.

**실험**: 현재(거리/조작성 단일기준) vs 복합기준 A/B를 eval+랜덤 그리드로
비교, 달성 `v_r·n`·점수 변화 측정.

**수용 기준**: 복합 점수 공식 확정 및 구현 / A/B에서 회귀 없음(이상적으로
개선) 확인 / `docs/`에 기록.

**파일**: `src/planner/swing/physics.rs`

---

### WP4a — IK를 elbow-up 단일 config로 제한

**선행 확인 필요**: 현재 기본 자세(`READY_JOINTS_4DOF`, `src/defaults/robot.rs`)가
실제로 elbow-up인지, 이 URDF의 부호 규약에서 "elbow-up"이 관절 2의 어느
부호에 대응하는지 먼저 검증(단순 가정 금지).

**위험**: `robot/mod.rs:898-908`의 실측 기록 — 반사 시드(다른 config
탐색)로 조작성이 최대 7배 차이 났다. 단일 config 고정은 특정 타점 기하에서
도달 가능한 최고 속도를 낮출 수 있다.

**실험**: (Phase 2, 다른 Phase 1 데이터 확보 후) 단일 시드(elbow-up만) vs
현재 다중 시드(최대 4개 반사) A/B를 eval+랜덤 그리드로 비교 — 커밋률,
달성 `v_r·n`, 계획 소요시간(다중 IK 호출 비용) 측정.

**수용 기준**: elbow-up 부호 규약 문서화 / A/B에서 회귀 없으면 단일 config로
전환(계획 속도 이득), 회귀 있으면 "elbow-up을 1순위 시드로 먼저 시도하고
실패시에만 대안 시드 폴백"으로 절충.

**파일**: `src/planner/swing/physics.rs:107-231`

---

### WP3 — 상대 코트 좌우 중앙 타겟팅 필요성 분석

**현재**: `rally_return_velocity`가 항상 `(WIDTH_X*0.5, LENGTH_Y*0.75, ...)`
고정 타겟(`impact.rs:17-21`)을 향해 `v_out`을 역산한다.

**분석**(실험이라기보단 계측+절충 판단):
1. `v_r`의 x성분이 전체 크기·관절 예산에서 차지하는 비중을 eval 그리드
   전수에서 계측(`required_racket_velocity_parts` 주변에 로깅 추가).
2. Ablation: 고정 중앙 타겟 vs 대안(예: 입사 x를 단순 미러링, 또는 x
   타겟팅 제거하고 IK가 자연히 내는 대로 둠)을 A/B, 커밋률·인코트율·
   `v_r·n` 비교.
3. 아웃오브바운즈 위험(좌우 조준을 없애면 x가 코트 밖으로 나갈 위험) 여부
   확인.

**수용 기준**: x성분의 예산 비중 수치화 / A/B 결과 근거로 유지·단순화 중
결정 및 문서화 / 변경 시 인코트율 회귀 없음 확인.

**파일**: `src/planner/impact.rs`

---

### 4.5 Phase 2 권장 착수 순서 (2026-07-30 갱신)

원안(§3)은 WP2b → WP4a → WP3 순서를 명시하지 않았으나, §0.1의 신규 발견을
반영해 다음 순서를 권장한다:

1. **Right 존 전멸(§0.1-1) 원인 격리부터.** 새 진단 테스트로 Right 존 샷의
   IK 실패 지점을 확인(`best_impact_candidate`가 후보를 아예 못 내는지,
   `RailMotion` 실현성 판정에서 떨어지는지, 아니면 다른 단계인지). 이 자체는
   계획에 없던 작업이라 우선 사용자에게 정식 WP로 승격할지 확인 필요 —
   원인이 WP2b/WP4a와 무관한 별개 버그라면 독립적으로 고치고, WP2b/WP4a
   설계와 얽혀 있다면 그 안에 흡수.
2. **WP2b(복합 랭킹).** §0.1-2(전부 1점)의 직접적 해법 후보이자, WP1
   재스윕의 전제조건(§ WP1 결과 "후속 과제" 참고 — 랭킹이 바뀌면 y_max·
   sample_step이 비로소 유효 파라미터가 됨).
3. **WP4a(elbow-up 단일 config).** WP2b 이후 착수(원안 그대로) — 조작성
   저하 A/B가 새 랭킹 기준으로 이뤄져야 결과가 최종 설계와 일치.
4. **WP3(좌우 타겟팅).** 독립적이므로 언제든 가능하나, Right 존 비대칭
   원인이 좌우 조준 로직과 얽혀 있을 수 있어 1번 이후 착수 권장.

---

## 5. 리스크 및 완화

| 리스크 | 완화 |
|---|---|
| Phase 순서를 어기고 Phase 1 실험을 Phase 0 이전에 실행 → 데이터 무효 | 각 Phase 완료를 체크리스트로 명시, Phase 1 착수 전 WP5/WP6/WP8 완료·검증 확인 |
| WP4a(단일 config)가 조작성 저하로 커밋률 회귀 | A/B 필수, 회귀 시 폴백 전략(1순위 시드+대안 폴백)으로 완화 |
| WP1/WP2a/WP2c 스윕이 Phase 0 변경으로 여러 번 무효화·재실행 필요 | Phase 0을 먼저 전부 끝내고 한 번에 Phase 1 착수 |
| WP8 회전자 관성 실측 데이터 부재 | 추정치+명시적 "미실측" 플래그로 우선 진행, 후속 벤치 측정 항목으로 별도 기록(`docs/measure-physics.md`) |
| WP5(coarse rate-limit)가 커밋률을 떨어뜨림 | `COARSE_TRACK_JOINT_FRACTION` 재튜닝으로 상쇄, 회귀 시 트레이드오프 명시 후 사용자 판단 요청 |
| 각 실험이 새 진단 테스트/계측 코드를 늘려 유지보수 부담 증가 | 기존 `tests/diag_weak_return.rs`·`world.rs`의 `#[ignore]` 진단 테스트 컨벤션을 따르고, 결과는 `docs/` 또는 `docs/measure-physics.md`류 문서에 정리 후 테스트 자체는 남겨 재현 가능하게 유지 |

---

## 6. 검증 절차 (전체)

1. Phase 0 각 WP: 관련 기존 테스트(`diag_motor_tracking`,
   `every_joint_reaches_commanded_pose_at_real_ball_contact`,
   `deep_racket_penetrates_table`, `swing_bench --sim-verify`) 재실행해
   회귀 없음 확인.
2. Phase 0 완료 후 `docs/swing-diagnostic-report.md`의 §0 라이브 측정
   (eval 30샷, `diag_weak_return`)을 재실행해 새 베이스라인 기록 —
   Phase 1 실험은 이 새 베이스라인 위에서 진행.
3. Phase 1 각 실험: 결과를 표/그래프로 `docs/`에 기록(새 문서 또는
   `docs/measure-physics.md` 확장).
4. Phase 2 각 설계변경: A/B 비교(변경 전/후 eval 점수, 커밋률, `v_r·n`)를
   반드시 기록 후 적용.
5. 전체 완료 후 `docs/swing-diagnostic-report.md`를 갱신하거나 후속
   보고서를 작성해 최종 상태 기록.

---

## 7. 다음 단계

Phase 0·Phase 1·WP9·WP2b·WP10 완료(§0 참고). WP10이 세기 1.5배 격차의
레버를 **사전축소 `1/r`**(요구 임팩트속도 대 관절속도한계 비율) 하나로
좁혔다. **다음 세션은 여기서 시작**:

1. `r`을 낮추는 두 방향 중 어느 쪽을 먼저 볼지 확인 — (a) WP3: 랠리
   리턴 타겟을 가깝게 당겨 필요 `|v_out|` 자체를 낮춤(조준 로직은 이미
   정상 확인됨, §0.3), (b) WP4a: elbow-up 단일 config든 다중 시드든 도달
   가능 임팩트 자세 집합을 넓혀 `r`이 낮은 자세를 더 찾음. 정식 WP 승격
   여부·순서 사용자 확인 필요.
2. WP4a A/B는 이제 세기 병목 원인이 좁혀졌으니 원안대로 진행해도 결과
   왜곡 위험이 적다(WP10이 coarse 추종 쪽은 무관함을 확인).
3. `min_swing_secs` 추가 확대(WP2a 후속, WP2b §7-2)는 우선순위 하향 —
   WP10이 quintic 단계는 이미 98% 무손실임을 확인해 시간을 더 줘도 세기가
   늘지 않는다.
4. 실행 방식(순차 vs `team`)은 착수 시 별도 확인.
