# 테이블 바운스 SSOT (예측기 · Rapier)

작성: 2026-07-26.  
범위: brainstorm **B** + 접근 **2** (공유 bounce 커널 + Rapier 재료 정렬 + 회귀).  
실측 e/μ/drag 잠금·drag 켜기·ω EKF는 **비범위** (보드/후속).

## 목적

테이블 바운스 정의를 **하나**로 두고:

1. ballistics 예측기 (`predict_hit_plane` / `semi_implicit_euler`)가 그 정의를 쓰고  
2. Rapier 시뮬의 실효 \(e,\mu\)(및 가능하면 스핀 결합)가 같은 정의에 맞게 정렬되어  
3. 발사 직후→바운스 후 재예측 시 hit-plane 마커 **점프**와 네트 게이트 슬랙(~1 cm) 원인을 줄인다.

맞출 대상은 **현실 물리 의미**이며, Rapier에 예측기를 끼워 맞추는 것이 아니다. Rapier는 같은 SSOT를 재료/관성으로 구현한다.

## 현재 갭 (근거)

| | Ballistics | Rapier |
|--|------------|--------|
| 법선 | \(v_z'=-e\,v_z\), \(e=\)`physics.restitution` | Average combine → 실효 \(e≈0.88\) (숫자는 같음, 솔버 오차) |
| 접선 | \(v_t'=(1-\mu)v_t\), \(\mu=\)`friction`(**0.4**), **ω 무시** | Average(`friction`,`ball_friction`)→\(\mu_{\mathrm{eff}}≈0.3\) + Coulomb + **ω↔v_t** |
| 비행 | `aero_accel` 공유, `drag=0` | 동일 |

코드 주석: ballistics 바운스가 Rapier보다 ~1 cm 낮음 → `NET_GATE_SLACK_M`; Rapier 바운스 ω가 Magnus로 과할 수 있음 → `ANGULAR_DAMPING` / `MAGNUS_OMEGA_MAX`.

## 경계

| 포함 | 제외 |
|------|------|
| 공유 `table_bounce` 커널 (선형 ± 스핀) | Rapier 접촉 후 속도 **강제 덮어쓰기** |
| Rapier 테이블/공 마찰·combine을 커널 \(\mu\)와 정합 | 실측으로 defaults 숫자 교체 |
| ballistics가 커널 호출 | drag 기본값 변경, ω EKF |
| 회귀: 발사 직후 vs 바운스 후 / Rapier 교차 | 라켓 `e_eff` 재설계 (이미 Min SSOT) |

## 커널 (SSOT)

새 모듈 후보: `src/estimator/bounce.rs` (또는 `planner`/`physics` 공용에 두되 예측기·문서에서 bounce SSOT로 명시).

입력: 접촉 직전 \(v\), \(\omega\) (선택), `PhysicsParams`, 공 `R`, `I=SHELL_INERTIA`.  
출력: 접촉 직후 \(v'\), \(\omega'\).

### 최소 (필수)

현 ballistics와 동일 의미, 한곳으로 모음:

\[
v_n' = -e\, v_n,\quad
v_t' = (1-\mu)\, v_t,\quad
\omega' = \omega\ \text{(그대로)}
\]

- \(e =\) `physics.restitution`  
- \(\mu =\) **테이블–공 실효 마찰** (아래 Rapier 정렬과 같은 식)

### 스핀 확장 (권장, 같은 커널 안에)

중공 셸 \(I=(2/3)mR^2\)로 접선–스핀을 한 스텝 결합 (문헌/단순 Coulomb 맵).  
목표가 “Rapier와 비트 단위 동일”이 아니라 **같은 물리 의미·비슷한 실효 감쇠**.  
구현 후 Rapier 재료만으로 부족하면 damping/`MAGNUS_OMEGA_MAX`는 **유지하되 주석에 “밴드에이드, 커널 정합 후 재평가”**.

`semi_implicit_euler`는 바닥 침투 시에만 이 커널을 호출. ω를 갱신하면 `predict_hit_plane` 적분 루프에 \(\omega\)를 넘겨 이후 Magnus가 바뀐다.

## Rapier 정렬

목표: 테이블–공 접촉의 **실효** \(e,\mu\)가 커널과 같다.

1. **\(\mu\) SSOT**  
   - 구현 현황 (2026-07-26): 커널 μ = `friction`(0.4). Rapier는 기본 Average → `rapier_table_ball_mu`≈0.3.  
     `ball_friction`을 올리면 라켓–공 Average도 바뀌고, 테이블 Max combine/Average=0.4 정렬은  
     랠리·랜덤샷 그리드를 깨므로 **그리드 재튜닝과 함께** 후속.  
   - 후속 옵션: `ball_friction = friction` **또는** 테이블 `friction_combine_rule=Max`로 실효 μ = `friction`.  
2. **\(e\)**  
   - 테이블·공 모두 `restitution`, Average 유지 (이미 실효 = `restitution`).  
3. **관성**  
   - 공 `SHELL_INERTIA` 유지 (이미 중공 셸).  
4. **하지 않음**  
   - 접촉 이벤트에서 linvel/angvel을 커널로 덮어쓰지 않음 (접근 3 거절).

랜덤 샷·네트 게이트 회귀가 깨지면 μ는 `friction` 단일값으로 그리드를 재맞추고, decisions/TODO에 “시뮬 그리드 재튜닝” 한 줄.

## 예측기

- `semi_implicit_euler` → 공유 커널.  
- `NET_GATE_SLACK_M`: 정합 후 회귀로 **축소 가능하면 줄이고**, 남으면 이유를 주석/스펙에 남김 (솔버 잔차).  
- GT `predict_impact`는 계속 Rapier `(p,v,ω)` 입력 + 같은 커널 적분 (진실 상태 + 현실 SSOT 예측).

## 성공 기준 (테스트)

1. **커널 단위**: 합성 \(v_n,v_t\)에 대해 \(e,\mu\) 정의와 identify 공식 일치.  
2. **μ 정합**: Rapier 재료로부터 계산한 실효 μ == 커널 μ (헬퍼 테스트).  
3. **점프 완화**: 기본 슈터 샷에서  
   - 발사 직후 `predict_hit_plane` 교차점 vs  
   - 첫 테이블 바운스 직후 같은 평면 재예측  
   XY 또는 Z 어긋남이 **현행보다 개선**되고, 문서에 수치 상한(예: Z ≤ 3 cm 등)을 회귀로 고정.  
4. 기존: `rapier_hit_plane_z_matches_predict_within_5cm` (또는 더 타이트하게) 유지/갱신.  
5. 랜덤 샷/네트 게이트 관련 기존 테스트 green (필요 시 그리드만 조정).

## 파일

| 경로 | 역할 |
|------|------|
| `src/estimator/bounce.rs` (신) | `table_bounce(v, ω, physics) -> (v', ω')` |
| `src/estimator/ballistics.rs` | 커널 호출 |
| `src/defaults/physics.rs` | μ 헬퍼·`ball_friction` 정렬, 주석 |
| `src/sim/physics/world.rs` | Rapier 공/테이블 friction·combine |
| `docs/decisions.md` | E/A 절 한 줄: bounce SSOT |
| `TODO.md` | 실측 잠금은 여전히 후속 |

## 비목표 / 후속

- 보드 실측으로 e/μ/k 숫자 교체  
- `physics.drag > 0` + EKF drag 동기  
- EKF ω 상태  
- 라켓 바운스 커널화 (이미 e_eff Min)
