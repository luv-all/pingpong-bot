# 2단계 구현 로드맵

1단계 sim E2E(Z-up·URDF·슈터 GUI) 위에, **스윙·임팩트·sim 접수**까지 들어왔다.  
남은 2단계는 **관측·추정 스텁을 실구현으로 교체**하고 sim↔real 경계를 연다.

애매 상수·폴백 결정은 [`decisions.md`](decisions.md)를 본다.

의존 순서대로 진행한다. 앞 단계가 없으면 뒤 단계가 의미 없다.

---

## 현재 진도 (2026-07)

| 영역 | 상태 | 비고 |
|------|------|------|
| 접수 평면 `HitPlane { y }` | ✅ | 기본 `DEFAULT_HIT_PLANE_Y = 0.30` |
| 3DOF IK + 리니어 레일 | ✅ | `Arm::competition`, `domain::rail` |
| quintic 스윙 + 관절 한계 | ✅ | `domain::trajectory`, `plan_swing` |
| 임팩트 역산 \(v_{out},n,e \to v_r\) | ✅ | `domain::impact` |
| 네트 통과 로프트 \(v_{out}\) | ✅ | `loft_return_velocity` |
| sim 자동 스윙 (commit-once) | ✅ | `world::try_auto_swing` — 재계획·contact 폴백 없음 |
| sim 예측 (적분+바운스) | 🔶 | `sim/estimator` — EKF 아님, 진실 위치 기반 |
| DLT / OpenCV 검출 / EKF | ⏳ | 마일스톤 1–2 |
| 토크 피드포워드 §7.4 | ⏳ | 속도 BC만 (`decisions` F4) |
| real 카메라·Dynamixel | ⏳ | 마일스톤 5 |

---

## 마일스톤 0 — 기준 고정 (선행)

| 항목 | 상태 | 비고 |
|------|------|------|
| Z-up 월드 좌표 (꼭짓점 원점) | ✅ | `WIDTH_X`, `LENGTH_Y`, `SURFACE_Z` |
| `HitPlane { y }` 접수 평면 | ✅ | 기본 `DEFAULT_HIT_PLANE_Y = 0.30` |
| `plan.md` §6 운동 모델 (g, drag, bounce) | 📖 | `domain::physics::accel` 이미 Z-up |
| 스윙·\(v_{out}\) 정책 | ✅ | `decisions.md` A–C |

---

## 마일스톤 1 — 관측 파이프라인

**목표:** 카메라 픽셀 → 월드 3D 위치

| # | 작업 | crate | 산출물 | 상태 |
|---|------|-------|--------|------|
| 1.1 | ChArUco 보정 CLI | `tools/calib_charuco` | `Calibration` YAML/JSON | ⏳ |
| 1.2 | DLT 삼각측량 | `domain::triangulation` | `triangulate_synced` 본체 | ⏳ 스텁 |
| 1.3 | OpenCV 원 검출 | `infra::detector` | HSV/contour 어댑터 | ⏳ 패스스루 |
| 1.4 | sim 카메라 → 실제 투영 | `infra::sim` | Rapier 공 + extrinsics | 🔶 부분 |
| 1.5 | `--config` TOML | `bin` | 카메라·extrinsics·ROI | ⏳ |

**완료 기준:** sim에서 공 3D 위치 RMSE < 5 cm (고정 카메라 3대).

---

## 마일스톤 2 — 궤적 추정 (EKF + RK4)

**목표:** 3D 관측 → `predict_to(HitPlane { y })` 실구현

| # | 작업 | crate | 산출물 | 상태 |
|---|------|-------|--------|------|
| 2.1 | `EkfState` (p, v) + 예측/보정 | `domain` (`ekf.rs`) | `Estimator` 구현체 | ⏳ |
| 2.2 | RK4 forward + **y 평면 교차** | `domain::ekf` | `time_to_impact`, `(x, z)` | ⏳ |
| 2.3 | 테이블 바운스 (e, μ) | `domain` / sim | 적분 중 반사 | 🔶 sim estimator만 |
| 2.4 | `PassThroughEstimator` 교체 | `app` DI | 파이프라인 연동 | ⏳ |
| 2.5 | k (drag) 튜닝 | `tools/measure_*` | `ball` 상수 보정 | ⏳ |

**완료 기준:** 슈터 발사 후 `Prediction.impact_position.y ≈ hit_plane.y`, x/z 오차 합리적.  
**임시:** sim은 진실 공 상태로 `predict_impact` — 관측 노이즈·EKF 없이 스윙 검증용.

---

## 마일스톤 3 — 스윙 계획 (IK + 임팩트)

**목표:** `Prediction` → 관절 궤적 → sim 접촉·리턴

| # | 작업 | crate | 산출물 | 상태 |
|---|------|-------|--------|------|
| 3.1 | 3DOF 역기구학 + 레일 | `domain::robot` | `inverse_kinematics*` | ✅ |
| 3.2 | `plan_swing` 본체 | `domain::physics` | quintic + joint limits | ✅ |
| 3.3 | 임팩트 역산 + 로프트 \(v_{out}\) | `domain::impact` | \(v_r\), 면 법선 | ✅ |
| 3.4 | sim 라켓 연동·자동 스윙 | `infra::sim` | commit-once 타격 | ✅ |
| 3.5 | 토크/동역학 피드포워드 | `domain::physics` | \(\tau = M\ddot q+\cdots\) | ⏳ |
| 3.6 | 면 법선·URDF↔제어 정합 | `robot` / URDF | `decisions` D | ⏳ |

**완료 기준 (1차):** sim에서 예측 지점으로 라켓이 이동하고 공을 네트 너머로 띄움 — **달성**.  
**완료 기준 (2차):** 관측 기반 예측 + 토크 한계 검증 + 상수 측정값 고정.

---

## 마일스톤 4 — 관측·디버깅

| # | 작업 | crate | 상태 |
|---|------|-------|------|
| 4.1 | Rerun `Telemetry` 어댑터 | `infra` | ⏳ |
| 4.2 | span 지연 (capture→control) | `app` + `tracing` | 🔶 부분 |
| 4.3 | 실패 run 리플레이 | Rerun recording | ⏳ |

---

## 마일스톤 5 — 실물 (Windows)

| # | 작업 | crate | 상태 |
|---|------|-------|------|
| 5.1 | UVC 카메라 `CameraSource` | `infra` | ⏳ |
| 5.2 | Dynamixel `Hardware` | `infra/real` | ⏳ |
| 5.3 | `--mode real` | `bin` | ⏳ |

---

## 다음 착수 (권장 순서)

스윙 1차가 돌아갔으니, **관측→추정**으로 파이프라인을 닫는다.

1. **마일스톤 1.2 — DLT 삼각측량**  
   - 파일: `crates/domain/src/triangulation.rs`  
   - 입력: N대 `(CameraId, PixelPoint)` + `Calibration`  
   - 출력: `Point3<World>`  
   - 테스트: 합성 extrinsics + 알려진 3D 점 → 복원 오차

2. **마일스톤 2.1–2.2 — EKF + hit-plane 예측**  
   - sim `estimator`의 적분/바운스를 domain으로 올리고 `PassThroughEstimator` 교체

3. **`decisions.md` D/E 잔여**  
   - 면 법선 정책, hit plane·restitution 단일화, URDF↔제어

4. **마일스톤 3.5 — 토크 피드포워드** (모터 한계가 보이기 시작할 때)

각 마일스톤마다 `cargo test --workspace` + sim GUI 스모크.
