# WP2b — 후보 랭킹에 "치기 쉬움 + 임팩트 세기" 반영

**날짜**: 2026-07-30
**대상**: `plan_best_swing`(`src/robot/motion/physics.rs`)의 타점 후보 정렬 기준
**기반**: WP9 완료 커밋 `42b8333` (eval 레일 리셋 포함)
**진단 테스트**: `tests/diag_wp2b_composite_ranking.rs`,
`impact_candidate.rs::diag_wp2b_ik_seed_spread`,
`physics.rs::diag_wp2b_score_vs_achieved` (모두 `#[ignore]` 관례 유지)

---

## 1. 결론 요약

**복합 랭킹을 채택한다.** 다만 **WP2b가 겨냥했던 증상은 해결되지 않았고,
진짜 병목을 데이터로 특정했다.** 두 결론을 분리해서 읽어야 한다.

### 채택하는 이유 (실측 개선)

| 지표 (접촉 샷 기준) | OLD 거리순 | NEW 복합 |
|---|---|---|
| eval 네트 통과율 | 86.7% | **100.0%** |
| eval 인코트율 | 86.7% | **100.0%** |
| eval Center 존 네트 통과율 | 60.0% | **100.0%** |
| 랜덤 5×5 네트 통과율 | 71.4% | **100.0%** |
| 랜덤 5×5 인코트율 | 71.4% | **100.0%** |
| 커밋률·접촉률 (전 존) | 100 / 100 % | 100 / 100 % (동일) |

55샷 중 **10샷이 개선, 0샷이 회귀**했다. 커밋률·접촉률은 전 존에서 완전
동일해 WP9가 고친 Right 존 패리티도 유지된다.

### 그런데 해결되지 않은 것 (계획 §0.1-2의 가설 반증)

`bounced_own_half`는 **100% → 100%로 그대로**고, eval 점수도 **30/90 →
30/90으로 완전히 동일**하다. "복합 랭킹이 약한 리턴(전 샷 1점)을 고칠
것"이라는 §0.1-2의 기대는 **반증됐다.** 오히려 원시 세기 지표는 약간
나빠졌다(`|v_out|/desired` 0.705 → 0.669).

랭킹은 **후보들 중에서 고르는 것**뿐인데, 아래 §4가 보이듯 **모든 후보가
필요 세기의 절반 이하**라 순서를 어떻게 바꿔도 임계값을 못 넘는다.

---

## 2. 복합 점수 공식

```
score = |v_r · n| × retained(r),    retained(r) = min(1, 1/r)
```

`r` = `peak_joint_speed_ratio`, `n` = IK 해가 실제로 만드는 라켓 면 법선.
클수록 좋은 점수이며 **단위가 m/s**다 — "이 타점에 커밋하면 임팩트 순간
라켓이 실제로 낼 법선속도"의 추정치다.

### 두 항이 각각 사용자 요구에 대응한다

- **임팩트 세기** = `|v_r · n|`. 임팩트 모델
  (`required_racket_velocity_parts`)에서 출사 법선속도는
  `(1+e)·v_r·n − e·v_in·n`이라 **리턴 세기를 지배하는 건 법선 성분뿐**이다.
  접선 lift 성분은 네트 클리어용이므로 세기 비교에서 뺀다.
- **치기 쉬움** = `retained(r)`. 파이프라인이 끝속도를 깎는 곳이 두
  군데인데 **둘 다 같은 `1/r` 꼴로 수렴**한다:
  1. `solve_impact_target`의 사전 축소는 `r > NEAR_SINGULARITY_SPEED_RATIO`
     (2.5)일 때 정확히 `1/r`을 곱한다(`impact_target.rs`).
  2. 그 아래 구간에서도 `fit_end_velocity`가 quintic 첨두 관절속도를
     `max_joint_speed` 안으로 이분탐색한다 — 끝속도 기여분은 배율에
     선형이고 무축소 상태의 비율이 곧 `r`이므로 가능한 최대 배율의 상한이
     `1/r`이다.

### 가중치가 없다 — 유도된 값이다

`min(1, 1/r)`은 손으로 고른 가중치가 아니라 **실제 축소 코드에서 유도한**
값이다. 그래서 이 점수에는 튜닝 파라미터가 하나도 없다.

**버린 대안: `w_e·ease + w_s·strength` 2항 가중합.**
`diag_wp2b_ik_seed_spread` 실측에서 `InterceptWindow` 전 평면의 `|v_r|`은
1.746~1.837 m/s(산포 5%)로 사실상 상수인 반면 `r`은 1.45~3.56(2.4배)로
움직인다 — **두 항 모두 `r`에 단조라 가중치가 식별되지 않는다**(어떤
`w_e`/`w_s`를 넣어도 순서가 같다). 유도된 단일 항이 더 단순하면서
`|v_r|`이 실제로 갈리는 기하에서도 옳게 동작한다.

### 공식의 예측력 검증

`diag_wp2b_score_vs_achieved`는 후보마다 점수와, `plan_swing`을 끝까지 돌려
얻은 궤적의 **임팩트 시점 실측 라켓 법선속도**(FK 유한차분)를 나란히 잰다.

| 공 | 점수 1위 타점 | 그 타점 실제 달성 | 실제 1위 달성 | 놓친 배율 |
|---|---|---|---|---|
| 5 m/s | y=0.26 | 0.3894 | 0.4064 (y=0.29) | 1.04× |
| 7 m/s | y=0.32 | 0.1308 | 0.1308 (y=0.32) | 1.00× |

빠른 공에서는 점수와 실측이 **소수점 4자리까지 일치**한다(0.0321/0.0650/
0.0979/0.1308). 공식은 정확하다.

---

## 3. IK 시드 랭킹은 바꾸지 않았다 (데이터 근거)

`best_impact_candidate`(`impact_candidate.rs`)의 시드 간 랭킹은
`peak_joint_speed_ratio` **단독**을 유지한다.

`diag_wp2b_ik_seed_spread` 실측: `InterceptWindow` 전 평면 × 3개 임팩트 x ×
2개 입사속도 = 60개 조합에서, **같은 타점**의 IK 시드 4개가 요구하는
`|v_r|`은 서로 **최대 0.026%**밖에 다르지 않다.

이유는 구조적이다 — `v_r`은 **타점 기하가 정하고**(입사속도·목표 출사속도·
법선), 시드가 바꾸는 건 그걸 내는 관절 조합뿐이다. 시드마다 갈리는 건 IK
수렴 오차가 만드는 법선 차이(`NORMAL_TOLERANCE = 1e-3`)뿐이다.

따라서 `score = |v_r| × min(1, 1/r)`에서 `|v_r|`이 상수면 **`r` 최소화가 곧
세기 최대화**다. 시드 레벨에서 복합 점수는 같은 순서를 더 비싸게 계산하는
것에 불과하다. 계획서가 제기한 "두 층 모두에 적용할지" 질문의 답은
**타점 층에만**이다.

---

## 4. 진짜 병목 — 랭킹이 아니라 세기 1.5배 부족

`tests/diag_wp2b_composite_ranking.rs`에 세 개의 판별 계측을 넣었다.
`Impact::clears_net`으로 **"바운스 없이 네트를 직접 넘는가"**를 직접 묻는다.

| 질문 | OLD | NEW |
|---|---|---|
| 플래너가 **원한** `v_out`이 네트를 직접 넘는가 | **100%** | **100%** |
| **실측** `v_out`이 네트를 직접 넘는가 | **0%** | **0%** |
| 실측 `v_out`을 **방향 그대로** desired 크기로 늘리면 | **100%** | **100%** |

읽는 법:

1. **조준은 맞다.** 플래너가 원하는 출사속도는 모든 임팩트점에서 네트를
   직접 넘는다(100%). `rally_return` 타겟팅 자체는 정상이다 — WP3(좌우
   중앙 타겟팅)가 이 증상의 원인이 아니다.
2. **달성은 전멸이다.** 실측 출사속도로는 **단 한 발도** 네트를 직접
   못 넘는다(0%). 그래서 전부 자기 코트에 먼저 바운스하고
   (`bounced_own_half` 100%), `Flags::score()`가 1점으로 캡핑한다.
3. **원인은 크기지 방향이 아니다.** 실측 출사속도의 **방향은 그대로 두고
   크기만** desired로 올리면 **100%가 네트를 직접 넘는다.**

즉 병목은 **`|v_out|/desired ≈ 0.67`, 약 1.5배의 순수한 세기 부족**이고,
네트 직접 통과 임계값이 그 사이에 있다. 방향·조준·후보선택이 아니다.

### 왜 랭킹으로는 못 고치는가

`diag_wp2b_score_vs_achieved`에서 10개 hit plane 중 **5~7개가
`plan_swing` 단계에서 아예 실패**한다 — 전부 같은 사유다:

```
임팩트 자세는 도달 가능하나 quintic 궤적이 중간에 [관절 속도] 한계를 벗어남
```

`fit_end_velocity`가 끝속도를 0으로 깎아도 실현 불가라는 뜻으로, **위치
이동(Δq) 자체가 관절속도 예산을 다 쓴다**. 살아남는 3~4개 후보는 서로
비슷하게 약하다(달성 0.32~0.41 m/s vs 필요 0.47~0.65 m/s). 랭킹은 이
3~4개의 순서만 바꾼다 — 1.5배 격차를 만들 여지가 없다.

---

## 5. A/B 전체 데이터

동일 하네스를 코드 변경 전/후로 두 번 돌린 짝지은 비교다(`git stash`로
`src/robot/motion/{physics,impact_candidate,impact_target}.rs`만 격리).
eval 그리드는 고정 시드(`0x5741_5031`) 지터를 켠다 — 끄면
`settings_for_zone_shot`이 `index_in_zone`을 버려 존당 10발이 전부 같아져
실질 표본이 3발이 된다(WP1 §5-a).

`*` = 접촉 샷 기준 비율.

### OLD (거리순 랭킹)

| grid | shots | commit% | contact% | score | own_half%* | cleared%* | in%* | v_r·n | \|v_r\| | v_r·n/req | \|v_out\| | \|v_out\|/des |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| eval 30 (all) | 30 | 100.0 | 100.0 | 30 | 100.0 | 86.7 | 86.7 | 0.100 | 0.397 | 0.099 | 2.908 | 0.705 |
| eval Left | 10 | 100.0 | 100.0 | 10 | 100.0 | 100.0 | 100.0 | 0.072 | 0.442 | 0.070 | 2.876 | 0.695 |
| eval Center | 10 | 100.0 | 100.0 | 10 | 100.0 | 60.0 | 60.0 | 0.153 | 0.305 | 0.155 | 2.983 | 0.728 |
| eval Right | 10 | 100.0 | 100.0 | 10 | 100.0 | 100.0 | 100.0 | 0.074 | 0.445 | 0.073 | 2.864 | 0.692 |
| random 5×5 | 25 | 100.0 | 84.0 | 21 | 100.0 | 71.4 | 71.4 | 0.122 | 0.395 | 0.123 | 2.966 | 0.721 |

### NEW (복합 랭킹)

| grid | shots | commit% | contact% | score | own_half%* | cleared%* | in%* | v_r·n | \|v_r\| | v_r·n/req | \|v_out\| | \|v_out\|/des |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| eval 30 (all) | 30 | 100.0 | 100.0 | 30 | 100.0 | **100.0** | **100.0** | 0.098 | 0.333 | 0.088 | 2.841 | 0.669 |
| eval Left | 10 | 100.0 | 100.0 | 10 | 100.0 | 100.0 | 100.0 | **0.089** | 0.395 | 0.078 | 2.828 | 0.660 |
| eval Center | 10 | 100.0 | 100.0 | 10 | 100.0 | **100.0** | **100.0** | 0.116 | 0.207 | 0.108 | 2.881 | 0.688 |
| eval Right | 10 | 100.0 | 100.0 | 10 | 100.0 | 100.0 | 100.0 | **0.090** | 0.397 | 0.079 | 2.814 | 0.657 |
| random 5×5 | 25 | 100.0 | 84.0 | 21 | 100.0 | **100.0** | **100.0** | 0.099 | 0.296 | 0.092 | 2.873 | 0.681 |

`double_hit`은 양쪽 모두 전 그리드 0.0%다(표에서 생략).

### 읽는 법 — 세기를 약간 내주고 일관성을 크게 샀다

- **Left·Right 존은 `v_r·n`이 올랐다** (0.072→0.089, 0.074→0.090, 각 +24%).
  복합 점수가 의도대로 작동한 구간이다.
- **Center 존은 `v_r·n`이 내렸지만**(0.153→0.116) **네트 통과율이
  60%→100%로 올랐다.** 거리순 랭킹이 Center에서 고르던 타점은 순간
  법선속도는 컸지만 4/10 확률로 네트를 못 넘겼다 — 조건수가 나쁜(높은 `r`)
  자세라 명령 법선을 제대로 추종하지 못한 것으로 보인다.
- 전체 평균 `v_r·n`이 0.100→0.098로 미세하게 내린 건 Center의 하락이
  Left/Right 상승보다 컸기 때문이다. **점수 캡핑(§4) 때문에 이 지표는
  현재 판별력이 낮다** — 세기 1.5배 부족이 해소되기 전까지는
  네트 통과율·인코트율이 더 신뢰할 만한 지표다.

---

## 6. 실제 변경 내용

`src/robot/motion/physics.rs`
- `candidate_score()` 신규 — §2의 공식.
- `plan_best_swing()` 2단계 구조로 변경:
  1. **IK 전용 채점 패스** — in-window 후보마다 `best_impact_candidate`를
     한 번씩 부른다. quintic/토크 적합은 **돌리지 않는다**.
  2. 점수 내림차순으로 `plan_swing_with_target`을 돌려 **첫 성공**을 채택.
     실패 시 다음 후보로 넘어가는 폴백은 예전과 동일 — **바뀐 건 순서뿐**이다.
- `accept_if_contact_within_tolerance()`, `distance_ranked()` 추출.
  후자는 채점 패스가 통째로 실패했을 때만 쓰는 안전망(예전 거리순 랭킹)이라
  채점이 고장 나도 후보가 0개로 줄지 않는다.
- `plan_swing_with_target()` 분리 — `plan_swing`의 후반부.

`src/robot/motion/impact_target.rs`
- `impact_target_from_candidate()` 분리, `NEAR_SINGULARITY_SPEED_RATIO`를
  `pub(crate)`로. 채점 패스가 이미 푼 IK 결과를 재사용해 **같은 IK를 두 번
  풀지 않게** 한다.

`src/robot/motion/impact_candidate.rs`
- `ImpactCandidate.impact_normal` 필드 추가(점수의 `v_r·n` 계산용).
- 시드 랭킹 로직은 **무변경** — §3의 근거를 doc comment에 기록.

### 계산 비용

채점 패스는 후보당 IK 1회를 추가한다. 이 경로는 매 물리 틱이 아니라
`SWING_RETRY_THROTTLE_SECS = 0.02`(50 Hz)로 스로틀된 커밋 시도에서만 돌고
(`world.rs::try_auto_swing`), 채택 후보의 IK는 `impact_target_from_candidate`로
재사용된다. 실측 `--lib` 스위트 소요시간은 변경 전후 동일(9.6초).

---

## 7. 후속 과제 — 다음 WP의 방향

§4가 특정한 병목은 **랭킹도 조준도 아닌 달성 세기 1.5배 부족**이다.
레버리지가 있는 곳:

1. **관절속도 예산이 "이동"에 다 쓰인다.** hit plane의 50~70%가
   `[관절 속도] 한계` 하나로 탈락한다. `min_swing_secs`를 늘려 이동 시간을
   더 주거나, 커밋 전 coarse 추종을 강화해 커밋 시점 Δq를 줄이는 쪽
   (WP4c의 `COARSE_TRACK_JOINT_FRACTION` 관절별 차등)이 직접적이다.
2. **WP7 재방문.** `fit_end_velocity` 이분탐색은 이미 최적 배율을 찾지만,
   그 배율이 낮은 건 Δq가 예산을 먹기 때문이다 — 위치·속도 예산 배분 자체가
   설계 대상이다.
3. **`max_joint_speed`(~2.88 rad/s, 실기 Dynamixel 스펙) 자체가 상한.**
   1.5배가 하드웨어 한계라면 랠리 리턴 타겟(`LENGTH_Y*0.75`)을 더 가깝게
   잡아 필요 `|v_out|`을 낮추는 게 현실적 절충이다 — WP3와 함께 검토.
4. **WP1 y평면 스윕 재실행.** WP1 문서 §6이 예고한 대로 랭킹이 바뀌었으니
   `y_max`·`sample_step`이 이제 유효 파라미터다. 다만 §4의 세기 병목이
   남아 있는 한 eval 점수는 여전히 판별력이 없으므로, 네트 통과율을 지표로
   써야 한다.

---

## 8. 재현 절차

```bash
# A/B 계측 (eval 30샷 + 랜덤 5×5, 약 15초)
cargo test --release --test diag_wp2b_composite_ranking -- --ignored --nocapture

# OLD 쪽을 다시 재려면 랭킹 변경분만 격리해서 stash
git stash push src/robot/motion/physics.rs src/robot/motion/impact_candidate.rs \
               src/robot/motion/impact_target.rs
cargo test --release --test diag_wp2b_composite_ranking -- --ignored --nocapture
git stash pop

# IK 시드 간 |v_r| 산포 (§3의 근거)
cargo test --release --lib diag_wp2b_ik_seed_spread -- --ignored --nocapture

# 점수 vs 실제 달성 세기 (§2 검증, §4 근거)
cargo test --release --lib diag_wp2b_score_vs_achieved -- --ignored --nocapture
```

---

## 9. 검증

- `cargo build --lib --tests` 클린.
- `cargo test --lib` — **227 passed / 1 failed / 38 ignored**.
  베이스라인(`42b8333`)도 **227 passed / 1 failed / 36 ignored**로 동일하며,
  ignored 증가분 2개는 이번에 추가한 `#[ignore]` 진단 테스트다.
  유일한 실패는 기존 무관 실패
  `hardware::dynamixel::tests::motor_mapping_matches_python_reference`
  (origin `7f9f827`에서 이미 깨져 있음, 이 계획 범위 밖).
  참고: `bang_bang_swing_planning_does_not_block_physics_step`은 wall-clock
  가드라 병렬 부하에서 간헐적으로 실패한다 — 베이스라인에서도 동일하게
  관측됐다(226/2 ↔ 227/1).
- **WP9 회귀 없음**: `diag_wp9_right_zone_commits_from_centered_start`가
  Right #1~#3 전부 `commit=true contact=true`. A/B 표에서도 세 존 모두
  커밋률·접촉률 100%로 완전 동일하다.
