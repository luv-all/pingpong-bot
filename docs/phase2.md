# 2단계 구현 로드맵

1단계(현재)에서 sim E2E 파이프라인·Z-up 좌표계·URDF·슈터 GUI까지 완료했다.  
2단계는 **스텁을 실제 알고리즘으로 교체**하고 sim↔real 경계를 연다.

의존 순서대로 진행한다. 앞 단계가 없으면 뒤 단계가 의미 없다.

---

## 마일스톤 0 — 기준 고정 (선행)

| 항목 | 상태 | 비고 |
|------|------|------|
| Z-up 월드 좌표 (꼭짓점 원점) | ✅ | `WIDTH_X`, `LENGTH_Y`, `SURFACE_Z` |
| `HitPlane { y }` 접수 평면 | ✅ | 기본 `DEFAULT_HIT_PLANE_Y = 1.0` |
| `plan.md` §6 운동 모델 (g, drag, bounce) | 📖 | `domain::physics::accel` 이미 Z-up |

---

## 마일스톤 1 — 관측 파이프라인

**목표:** 카메라 픽셀 → 월드 3D 위치

| # | 작업 | crate | 산출물 |
|---|------|-------|--------|
| 1.1 | ChArUco 보정 CLI | `tools/calib_charuco` | `Calibration` YAML/JSON |
| 1.2 | DLT 삼각측량 | `domain::triangulation` | `triangulate_synced` 본체 |
| 1.3 | OpenCV 원 검출 | `infra::detector` | HSV/contour 어댑터 |
| 1.4 | sim 카메라 → 실제 투영 | `infra::sim` | Rapier 공 위치 + extrinsics |
| 1.5 | `--config` TOML | `bin` | 카메라 대수·extrinsics·ROI 로드 |

**완료 기준:** sim에서 공 3D 위치 RMSE < 5 cm (고정 카메라 3대).

---

## 마일스톤 2 — 궤적 추정 (EKF + RK4)

**목표:** 3D 관측 → `predict_to(HitPlane { y })` 실구현

| # | 작업 | crate | 산출물 |
|---|------|-------|--------|
| 2.1 | `EkfState` (p, v) + 예측/보정 | `domain` (신규 `ekf.rs`) | `Estimator` 구현체 |
| 2.2 | RK4 forward + **y 평면 교차** | `domain::ekf` | `time_to_impact`, `(x, z)` |
| 2.3 | 테이블 바운스 (e, μ) | `domain::physics` | 적분 중 z=0.76 반사 |
| 2.4 | `PassThroughEstimator` 교체 | `app` DI | sim 파이프라인 연동 |
| 2.5 | k (drag) 튜닝 | `tools/measure_*` | `ball` 상수 보정 |

**완료 기준:** 슈터 발사 후 `Prediction.impact_position.y ≈ hit_plane.y`, x/z 오차 합리적.

---

## 마일스톤 3 — 스윙 계획 (IK + 임팩트)

**목표:** `Prediction` → 관절 궤적

| # | 작업 | crate | 산출물 |
|---|------|-------|--------|
| 3.1 | 3DOF 역기구학 | `domain::robot` | `inverse_kinematics(p)` |
| 3.2 | `plan_swing` 본체 | `domain::physics` | quintic + joint limits |
| 3.3 | 임팩트 역산 (e, 원하는 v_out) | `domain::physics` | 라켓 속도·면 법선 |
| 3.4 | sim 라켓 연동 검증 | `infra::sim` | FK 목표 = Rapier 라켓 |

**완료 기준:** sim에서 예측 지점으로 라켓이 이동하고 공과 접촉.

---

## 마일스톤 4 — 관측·디버깅

| # | 작업 | crate |
|---|------|-------|
| 4.1 | Rerun `Telemetry` 어댑터 | `infra` |
| 4.2 | span 지연 (capture→control) | `app` + `tracing` |
| 4.3 | 실패 run 리플레이 | Rerun recording |

---

## 마일스톤 5 — 실물 (Windows)

| # | 작업 | crate |
|---|------|-------|
| 5.1 | UVC 카메라 `CameraSource` | `infra` |
| 5.2 | Dynamixel `Hardware` | `infra/real` |
| 5.3 | `--mode real` | `bin` |

---

## 권장 브랜치 전략

```
main          — 1단계 안정 (sim E2E)
phase2/1-obs  — 마일스톤 1
phase2/2-ekf  — 마일스톤 2
phase2/3-swing — 마일스톤 3
```

각 마일스톤마다 `cargo test --workspace` + sim GUI 스모크.

---

## 다음 착수 (즉시)

**마일스톤 1.2 — DLT 삼각측량**

- 파일: `crates/domain/src/triangulation.rs`
- 입력: N대 `(CameraId, PixelPoint)` + `Calibration`
- 출력: `Point3<World>`
- 테스트: 합성 extrinsics + 알려진 3D 점 → 복원 오차
