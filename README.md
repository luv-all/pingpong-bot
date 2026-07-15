# pingpong-bot

사람과 오래 협력 랠리를 이어가는 핑퐁 로봇 런타임.  
Rust 경연 바이너리. 크레이트는 domain / app / infra / bin으로 나누되, **비전은 OpenCV SSOT로 infra**에 두고 헥사 포트로 감싸지 않는다.

상세 설계는 [`plan.md`](plan.md)를 본다.

---

## 요구 사항

- [Rust](https://rustup.rs/) (edition 2024 — 최신 stable 권장)
- Cargo (workspace)

macOS·Linux에서 **sim 모드**로 end-to-end 파이프라인을 돌릴 수 있다.  
**real 모드**(실 카메라·모터)는 Windows + `pingpong-infra/real` feature — 2단계 예정.

---

## 빠른 시작

```bash
# 전체 workspace 빌드·검증
cargo check --workspace
cargo test --workspace

# sim 파이프라인 실행 (`config/default.toml`)
cargo run -p pingpong-bin

# 다른 실험 설정
cargo run -p pingpong-bin -- config/experiment.toml
```

실행하면 **Rapier3d 디지털 트윈**(탁구대·공·로봇 팔) 위에서 가상 카메라가 공을 촬영하고, 제어 루프가 라켓을 구동한다.  
로그는 `tracing`으로 stdout에 출력된다.

---

## 런타임 설정 (`pingpong-bot`)

```bash
cargo run -p pingpong-bin                       # config/default.toml
cargo run -p pingpong-bin -- config/test.toml  # 지정한 TOML
```

CLI는 선택적인 TOML 경로 하나만 받는다. 모드·로봇·카메라·sim·물리 값은
모두 TOML이 SSOT이며 CLI override는 없다. `calibration_path`와 `urdf_path`의
상대 경로는 해당 TOML 파일의 디렉터리를 기준으로 해석한다. 전체 필드 설명은
[`config/example.toml`](config/example.toml)을 참고한다.

### 예시

```bash
# GUI sim (기본) — 슈터에서 「발사」로 공 쏘기
cargo run -p pingpong-bin

# 설명이 포함된 예시를 복사해 robot, [sim], urdf_path 등을 수정
cp config/example.toml config/experiment.toml
cargo run -p pingpong-bin -- config/experiment.toml

# 로그 레벨 조정
RUST_LOG=debug cargo run -p pingpong-bin
RUST_LOG=pingpong_app=debug,info cargo run -p pingpong-bin -- config/experiment.toml
```

---

## 프로젝트 구조

```
crates/
  domain/   추정·제어 — 타입, `Arm`/FK/EKF, `constants/`, Hardware 등 포트
  app/      파이프라인 오케스트레이션 — 카메라·추정·제어 스레드, 채널
  infra/    비전(`vision`: Calibration·삼각측량·FrameSource) + Rapier sim + 텔레메트리
  bin/      CLI 진입점 — sim/real 모드 DI

tools/      실험·캘리브·검증용 독립 바이너리
plan.md     기술 마스터 플랜
TODO.md     실행 체크리스트
```

의존 방향: `bin` → `app` → `infra` → `domain` (비전은 app이 infra를 직접 호출)

**로봇**
- 기구학·제어 `Arm` 타입은 `domain/robot/`에만 있다. 부팅 시 `Arc<Arm>`으로 sim·real·제어가 같은 불변 객체를 공유한다.
- **프리셋 목록**은 [`crates/app/src/arm.rs`](crates/app/src/arm.rs)의 `ROBOTS`다. id·URDF 경로·EE 링크·최대 속도는 여기서 고친다.
- URDF 프리셋은 `origin xyz/rpy`, 축, 한계, EE 고정 변환을 보존한 일반 직렬 체인으로 변환한다. 제어·FK·수치 IK·충돌 검사·mesh 뷰어가 같은 URDF 관절 순서를 쓴다.
- `competition`만 URDF가 없으므로 기존 4DOF primitive 빌더를 사용한다. URDF 로드/변환 실패는 다른 모델로 대체하지 않고 시작 오류로 반환한다.

| id | 모델 | 제어·FK·IK |
|----|------|------------|
| `competition` | 없음 (빌더만) | `4-dof` URDF의 축·offset·한계를 보존한 단순화 체인 |
| `urdf-test` | `assets/robots/urdf-test/.../urdf-test.urdf` | 해당 URDF |
| `competition-urdf` | `assets/robots/competition_arm.urdf` | 해당 URDF |
| `4-dof` | `assets/robots/4-dof/urdf/all-4-export.urdf` | 해당 URDF |

```toml
# config/default.toml
robot = "competition"
```

### sim 모드 — Rapier3d 디지털 트윈 (plan §9)

```
SimSession (물리 스레드 @ physics-hz, CCD)
  ├─ SimWorld: ITTF 탁구대 + 슈터(+y) + 로봇 라켓(y≈0) + 공
  ├─ SimCamera × N: 공 3D 위치 → 핀홀 픽셀 투영
  └─ SimHardware: plan_swing → 관절 목표 → 라켓 이동

슈터(+y) ──발사──► 테이블 ──► 로봇(y≈0) 라켓
         ▲ GUI 「발사」 버튼으로 트리거
```

```bash
cargo run -p pingpong-bin
cargo run -p pingpong-bin -- config/experiment.toml
```

- 좌표계: **Z-up**, 원점 = 탁구대 로봇 쪽 꼭짓점 (바닥)
  - **+X** = 너비 1.525 m, **+Y** = 길이 2.74 m, **+Z** = 고도 (테이블 면 `z = 0.76 m`)
- **로봇** `y ≈ 0` 쪽, **슈터** `+y` 끝 (상대편)
- 공: 슈터에 **주차** → GUI 「발사」 시에만 비행, 이탈 시 자동 회수
- GUI: yaw/pitch/roll 조준·속도·top/side/drill spin·시간배율 + 발사/회수 버튼
- **kiss3d 3D + egui 패널** (단일 창 — macOS EventLoop 제약)

제어 루프는 100 Hz. `Prediction` 슬롯은 1칸(최신 예측만 유지).

---

## 실험 도구 (`tools/`)

각 도구는 `domain`/`infra`와 같은 타입을 공유한다. 비전 산출물은 `pingpong_infra::Calibration`.

| crate | 바이너리 | 상태 | 용도 |
|-------|----------|------|------|
| `calib-charuco` | `calib_charuco` | ✅ | ChArUco (`--emit-sim` / `--validate` / `--features opencv --from-images`) |
| `measure-restitution` | `measure_restitution` | ✅ | 반발계수 \(e\), 항력 \(k\) |
| `measure-friction` | `measure_friction` | ✅ | 마찰계수 \(\mu\) |
| `jog-axis` | `jog_axis` | ⏳ 스텁 | 축 수동 조그 |
| `capture-flying-ball` | `capture_flying_ball` | ⏳ 스텁 | 비행 공 캡처 데이터셋 |
| `detect-bgsub` | `detect_bgsub` | ⏳ 스텁 | 배경 차분 검출 |
| `detect-colormask` | `detect_colormask` | ⏳ 스텁 | 색상 마스크 검출 |
| `detect-contour` | `detect_contour` | ⏳ 스텁 | contour·원형도 검출 |
| `detect-roi` | `detect_roi` | ⏳ 스텁 | ROI 추적 검출 |

### 캘리브레이션

```bash
cargo run -p calib-charuco -- --emit-sim 3 -o calibration.json
cargo run -p calib-charuco -- --validate calibration.json
# 시스템 OpenCV 필요:
# cargo run -p calib-charuco --features opencv -- --from-images ./boards -o calibration.json
cargo run -p pingpong-bin
```

infra 삼각측량에 OpenCV를 쓰려면:

```bash
cargo test -p pingpong-infra --features opencv
```

### 물리 계수 측정 (`measure_*`)

공식은 `domain::estimator::identify`, 기본 상수는 `domain::constants::ball` / `physics`다.  
측정 결과는 TOML 스니펫으로 찍히며 `-o`로 저장한 뒤 constants에 반영한다.

**반발계수 \(e\)** — \(e \approx \sqrt{h_{i+1}/h_i}\) 또는 \(e = |v_n'|/|v_n|\)

측정값은 기본으로 [`config/default.toml`](config/default.toml)의 `[physics]`에 **merge**된다.
다른 파일을 쓰려면 `--config path`, 쓰기 없이 확인만 하려면 `--dry-run`.

```bash
# 연속 바운스 정점 높이 [m] → config [physics].restitution
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21

# 법선 속력 쌍 |vin|:|vout|
cargo run -p measure-restitution -- --vz-pairs 2.0:1.7,1.9:1.61

# 탄도 적분으로 설정된 TABLE_BOUNCE_RESTITUTION 검증 (모델 SSOT)
cargo run -p measure-restitution -- --sim-ballistics --dry-run

# Rapier 낙하 — 솔버 실효 e
cargo run -p measure-restitution -- --sim

# 다른 config 파일에 쓰기
cargo run -p measure-restitution -- --heights 0.40,0.29 --config config/my.toml
```

**마찰 \(\mu\)** — \(v_t' = (1-\mu)\,v_t\) → \(\mu = 1 - |v_t'|/|v_t|\)

```bash
cargo run -p measure-friction -- --vt-pairs 2.0:1.4,1.5:1.05
cargo run -p measure-friction -- --sim
cargo run -p measure-friction -- --sim --dry-run
```

**항력 \(k\)** — 비행 궤적 CSV `t,x,y,z` 최소제곱 (마일스톤 2.5)

```bash
cargo run -p measure-restitution -- --drag-csv traj.csv
```

런타임 반영:

```bash
cargo run -p pingpong-bin
# → Rapier 반발 + BallEkf drag/friction/restitution 예측에 [physics] 사용
```

런타임 필드는 TOML에서 명시한다. 누락되거나 타입이 틀리면 시작 전에 실패한다.
---

## 개발

```bash
# 특정 crate만
cargo check -p pingpong-domain
cargo test -p pingpong-domain

# 릴리스 빌드
cargo build -p pingpong-bin --release
# → target/release/pingpong-bot
```

---

## 현재 구현 상태

| 영역 | 상태 |
|------|------|
| workspace 스캐폴딩, sim E2E 파이프라인 | ✅ |
| **Rapier3d 디지털 트윈** (탁구대·슈터·로봇·공·SimCamera) | ✅ |
| **egui 슈터 GUI** (발사 트리거·파라미터) | ✅ |
| **kiss3d 3D 뷰** (탁구대·로봇·슈터·공) | ✅ |
| **로봇 프리셋** (TOML `robot`, `app/arm.rs` `ROBOTS`) | ✅ |
| **URDF mesh** (TOML `urdf_path` 또는 프리셋 `urdf_rel`) + 제어→URDF 관절 매핑 | ✅ |
| Z-up 좌표계 + `HitPlane { y }` 접수 평면 | ✅ |
| **BallScript** (시간·위치·속도·임펄스 스케줄) | ✅ |
| **RobotBuilder** (URDF mesh + sim 마운트) | ✅ |
| `sample_at` 타임스탬프 보간 | ✅ |
| DLT 삼각측량 (`infra::vision`, `opencv` feature 시 triangulatePoints) | ✅ |
| ChArUco (`calib_charuco --emit-sim` / `--features opencv --from-images`) | ✅ 초안 |
| EKF / 궤적 추정 | ✅ (sim; 기본은 `sim.use_ground_truth=true`) |
| `measure_restitution` / `measure_friction` (e·μ·k) | ✅ |
| TOML 단일 설정 (`config/default.toml`) | ✅ |
| OpenCV 원/공 검출, 실 카메라 `VideoCapture` | ⏳ |
| Rerun 시각화, Dynamixel/AXL real | ⏳ |
| TOML `mode = "real"` | ⏳ 2단계 |

sim에서는 **실제 3D 물리**로 공이 날고, ground truth 또는 EKF control로 라켓이 움직인다.

**로드맵:** [`docs/phase2.md`](docs/phase2.md) · 잔여 작업 [`TODO.md`](TODO.md) · 결정 [`docs/decisions.md`](docs/decisions.md)

---

## 라이선스

(미정)
