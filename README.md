# pingpong-bot

사람과 오래 협력 랠리를 이어가는 핑퐁 로봇 런타임.  
Rust 경연용 단일 애플리케이션 크레이트다. 카메라·검출·추정·로봇·시뮬레이션·
계획을 `src/` 아래 기능별 모듈로 나눈다. OpenCV는 필수 의존성이며,
Rapier·실물 하드웨어 경계는 feature와 모듈로 격리한다.

상세 설계는 [`plan.md`](plan.md)를 본다.

---

## 요구 사항

- [Rust](https://rustup.rs/) (edition 2024 — 최신 stable 권장)
- Cargo (workspace)
- 시스템 **OpenCV 4.x** + `libclang` (`opencv` crate `0.98.2+` 빌드 시 필요)

macOS·Linux에서 **sim 모드**로 end-to-end 파이프라인을 돌릴 수 있다.  
**real 모드**(실 카메라·모터)는 Windows + `real` feature — 2단계 예정.

> OpenCV 5.x(`brew install opencv`)는 쓰지 말고 **`opencv@4`** 를 쓴다.  
> Homebrew LLVM 22와 맞추려면 Rust `opencv` crate도 **0.98.2 이상**이어야 한다
> (그 이하면 `as_raw_*` 메서드 누락 에러가 대량으로 난다).

### OpenCV · libclang (macOS)

```bash
brew install llvm opencv@4 pkgconf
# 이미 OpenCV 5를 깔았다면:
# brew uninstall opencv && brew install opencv@4
```

환경 변수는 프로젝트 [`.envrc`](.envrc)에 둔다 (`direnv` 권장). `~/.zshrc`에
넣지 않는다.

```bash
brew install direnv
# ~/.zshrc 에 한 줄: eval "$(direnv hook zsh)"
cd /path/to/pingpong-bot
direnv allow .
```

수동으로 넣을 때 (Apple Silicon / Intel 공통 — `brew --prefix` 사용):

```bash
export LIBCLANG_PATH="$(brew --prefix llvm)/lib"
export PKG_CONFIG_PATH="$(brew --prefix opencv@4)/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export DYLD_FALLBACK_LIBRARY_PATH="$(brew --prefix llvm)/lib:$(brew --prefix opencv@4)/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
```

확인 (이 셸에 direnv가 켜져 있어야 함 — `echo $LIBCLANG_PATH`가 비면 안 됨):

```bash
echo "$LIBCLANG_PATH"
test -f "$LIBCLANG_PATH/libclang.dylib" && echo "libclang ok"
pkg-config --modversion opencv4   # 4.x 여야 함
cargo clean
cargo check --workspace
```

### OpenCV · libclang (Windows)

로컬에서 실행 검증은 못 했고, [`opencv` crate INSTALL](https://docs.rs/crate/opencv/0.98.2/source/INSTALL.md) ·
[vcpkg `opencv4`](https://vcpkg.io/en/package/opencv4.html) ·
이슈 가이드를 기준으로 정리했다.

필요 조건:

- [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/) (C++ 워크로드) 또는 Visual Studio
- LLVM (`libclang.dll`) — [공식 설치](https://github.com/llvm/llvm-project/releases) 또는 `choco install llvm`
- **OpenCV 4.x** (OpenCV 5 금지)

#### 권장: vcpkg (자동 탐색)

공식 INSTALL의 Windows 경로다. `VCPKG_ROOT` + `VCPKGRS_DYNAMIC=1`이면
`OPENCV_LINK_*`를 수동으로 안 잡아도 된다.

```powershell
# vcpkg 루트에서
# Charuco/ArUco는 OpenCV 4.7+ 메인 objdetect에 있음 → contrib 불필요
vcpkg install opencv4:x64-windows

$env:VCPKG_ROOT = "C:\path\to\vcpkg"   # 실제 vcpkg 클론 경로
$env:VCPKGRS_DYNAMIC = "1"             # 동적 링크 — 없으면 링크 에러가 흔함
# LLVM은 PATH에 bin이 잡혀 있어야 한다 (설치 시 옵션 또는 수동 추가)
$env:LIBCLANG_PATH = "C:\Program Files\LLVM\bin"
$env:PATH = "C:\Program Files\LLVM\bin;$env:VCPKG_ROOT\installed\x64-windows\bin;$env:PATH"
```

`VCPKG_ROOT`는 vcpkg 저장소 루트(예: `C:\src\vcpkg`)여야 한다.
`installed\x64-windows`가 아니라 **vcpkg 루트**다.

확인:

```powershell
Test-Path "$env:LIBCLANG_PATH\libclang.dll"
cargo clean
cargo check --workspace
```

실행 시 `STATUS_DLL_NOT_FOUND`(0xc0000135)가 나면 OpenCV DLL 경로가 PATH에
없거나, 바이너리 옆에 DLL이 없는 경우다. `installed\x64-windows\bin`을 PATH에
넣었는지 다시 본다.

#### 대안: 공식 OpenCV 바이너리 (수동 링크)

[OpenCV Releases](https://opencv.org/releases/)의 Windows 패키지를 쓸 때는
라이브러리 이름이 버전에 묶인다. 예: OpenCV **4.12.0** → `opencv_world4120`
(`opencv_world4`가 아님).

```powershell
$env:LIBCLANG_PATH = "C:\Program Files\LLVM\bin"
$env:OPENCV_INCLUDE_PATHS = "C:\opencv\build\include"
$env:OPENCV_LINK_PATHS = "C:\opencv\build\x64\vc16\lib"
$env:OPENCV_LINK_LIBS = "opencv_world4120"   # 설치한 버전에 맞게 수정
$env:PATH = "C:\Program Files\LLVM\bin;C:\opencv\build\x64\vc16\bin;$env:PATH"
```

`include` 아래 `opencv2\core\version.hpp`가 보여야 버전 탐지가 된다.

#### 주의

- vcpkg 기본 `opencv4`에는 **`world` feature가 꺼져 있다**.
  그래서 README에 `OPENCV_LINK_LIBS=opencv_world4`만 적으면 대개 실패한다.
  vcpkg를 쓸 때는 위처럼 `VCPKG_ROOT`/`VCPKGRS_DYNAMIC` 자동 탐색을 권장한다.
- `world`를 쓰려면 `vcpkg install "opencv4[world]:x64-windows"` 후
  `OPENCV_LINK_LIBS=opencv_world4`(Debug는 `opencv_world4d`)처럼 수동 지정한다.
- Chocolatey `choco install llvm opencv`도 INSTALL에 나오지만, 그 경우에도
  `OPENCV_INCLUDE_PATHS` / `OPENCV_LINK_PATHS` / `OPENCV_LINK_LIBS`를
  설치 경로·버전명에 맞게 잡아야 한다.
- 영구 설정은 사용자/시스템 환경 변수에 등록한다.

---

## 빠른 시작

```bash
# 전체 workspace 빌드·검증
cargo check --workspace
cargo test --workspace

# sim 파이프라인 실행 (`config/default.toml`)
cargo run -p pingpong-bot

# 다른 실험 설정
cargo run -p pingpong-bot -- config/experiment.toml
```

실행하면 **Rapier3d 디지털 트윈**(탁구대·공·로봇 팔) 위에서 가상 카메라가 공을 촬영하고, 제어 루프가 라켓을 구동한다.  
로그는 `tracing`으로 stdout에 출력된다.

---

## 런타임 설정 (`pingpong-bot`)

```bash
cargo run -p pingpong-bot                       # config/default.toml
cargo run -p pingpong-bot -- config/test.toml  # 지정한 TOML
```

CLI는 선택적인 TOML 경로 하나만 받는다. 모드·로봇·카메라·sim·물리 값은
모두 TOML이 SSOT이며 CLI override는 없다. `calibration_path`와 `urdf_path`의
상대 경로는 해당 TOML 파일의 디렉터리를 기준으로 해석한다. 전체 필드 설명은
[`config/example.toml`](config/example.toml)을 참고한다.
`[intercept]`의 `y_min..=y_max`를 `sample_step` 간격으로 예측해, 현재 로봇
포즈에서 접촉 오차·관절/레일 한계·테이블 충돌을 통과하는 타격점만 선택한다.

### 예시

```bash
# GUI sim (기본) — 슈터에서 「발사」로 공 쏘기
cargo run -p pingpong-bot

# 설명이 포함된 예시를 복사해 robot, [sim], urdf_path 등을 수정
cp config/example.toml config/experiment.toml
cargo run -p pingpong-bot -- config/experiment.toml

# 로그 레벨 조정
RUST_LOG=debug cargo run -p pingpong-bot
RUST_LOG=pingpong_bot=debug,info cargo run -p pingpong-bot -- config/experiment.toml
```

---

## 프로젝트 구조

```
src/
  camera/     캡처·캘리브레이션·삼각측량·가상 카메라
  detector/   공 검출
  estimator/  EKF·탄도·물리계수 식별
  robot/      Arm·FK/IK·URDF·프리셋
  sim/        Rapier 월드·슈터·뷰어·sim 어댑터
  planner/    충돌·임팩트·스윙 궤적
  constants/  공·탁구대·로봇·제어 상수
  pipeline.rs 카메라→추정→제어 오케스트레이션
  config.rs   TOML 런타임 설정
  main.rs     CLI와 sim/real 조립

tools/      실험·캘리브·검증용 독립 바이너리
plan.md     기술 마스터 플랜
TODO.md     실행 체크리스트
```

단일 크레이트이지만 `planner`·`estimator`·로봇 기구학은 OpenCV와 Rapier를
직접 참조하지 않는다. 외부 구현은 `camera`, `sim`, `hardware`에 둔다.

**로봇**
- 기구학·제어 `Arm` 타입은 `src/robot/`에만 있다. 부팅 시 `Arc<Arm>`으로 sim·real·제어가 같은 불변 객체를 공유한다.
- **프리셋 목록**은 [`src/robot/catalog.rs`](src/robot/catalog.rs)의 `ROBOTS`다. id·URDF 경로·EE 링크·최대 속도는 여기서 고친다.
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
cargo run -p pingpong-bot
cargo run -p pingpong-bot -- config/experiment.toml
```

- 좌표계: **Z-up**, 원점 = 탁구대 로봇 쪽 꼭짓점 (바닥)
  - **+X** = 너비 1.525 m, **+Y** = 길이 2.74 m, **+Z** = 고도 (테이블 면 `z = 0.76 m`)
- **로봇** `y ≈ 0` 쪽, **슈터** `+y` 끝 (상대편)
- 공: 슈터에 **주차** → GUI 「발사」 시에만 비행, 이탈 시 자동 회수
- 타격: 동적 인터셉트 → 레일+관절 pose IK(라켓 중심·면법선) → 임팩트 knot
  → 팔로스루 순으로 실행하며, 실제 Rapier 접촉·네트 통과·상대 코트 중앙
  바운스를 회귀 테스트한다.
- GUI: yaw/pitch/roll 조준·속도·top/side/drill spin·시간배율 + 발사/회수 버튼
- **kiss3d 3D + egui 패널** (단일 창 — macOS EventLoop 제약)

제어 루프는 100 Hz. `Prediction` 슬롯은 1칸(최신 예측만 유지).

---

## 실험 도구 (`tools/`)

각 도구는 루트 `pingpong_bot` 라이브러리와 같은 타입을 공유한다. 카메라
캘리브레이션 산출물은 `pingpong_bot::Calibration`이다.

| crate | 바이너리 | 상태 | 용도 |
|-------|----------|------|------|
| `calib-charuco` | `calib_charuco` | ✅ | ChArUco (`--emit-sim` / `--validate` / `--from-images`) |
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
# cargo run -p calib-charuco -- --from-images ./boards -o calibration.json
cargo run -p pingpong-bot
```

OpenCV는 모든 빌드에서 필수다:

```bash
cargo test -p pingpong-bot --lib
```

### 물리 계수 측정 (`measure_*`)

공식은 `estimator::identify`, 기본 상수는 `constants::ball` / `physics`다.
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
cargo run -p pingpong-bot
# → Rapier 반발 + BallEkf drag/friction/restitution 예측에 [physics] 사용
```

런타임 필드는 TOML에서 명시한다. 누락되거나 타입이 틀리면 시작 전에 실패한다.
---

## 개발

```bash
# 특정 crate만
cargo check -p pingpong-bot --lib
cargo test -p pingpong-bot --lib

# 릴리스 빌드
cargo build -p pingpong-bot --release
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
| **로봇 프리셋** (TOML `robot`, `robot/catalog.rs` `ROBOTS`) | ✅ |
| **URDF mesh** (TOML `urdf_path` 또는 프리셋 `urdf_rel`) + 제어→URDF 관절 매핑 | ✅ |
| Z-up 좌표계 + `HitPlane { y }` 접수 평면 | ✅ |
| **BallScript** (시간·위치·속도·임펄스 스케줄) | ✅ |
| **RobotBuilder** (URDF mesh + sim 마운트) | ✅ |
| `sample_at` 타임스탬프 보간 | ✅ |
| DLT/OpenCV 삼각측량 (`camera`, 2뷰는 `triangulatePoints`) | ✅ |
| ChArUco (`calib_charuco --emit-sim` / `--from-images`) | ✅ 초안 |
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
