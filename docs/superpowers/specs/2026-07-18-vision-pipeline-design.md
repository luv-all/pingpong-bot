# 관측 파이프라인 — OpenCV 캡처·검출·ChArUco 인트린식

날짜: 2026-07-18  
상태: 디자인 승인됨 (구현 전)

## 목표

실물 관측 본선을 닫는다.

1. 오프라인 `calib_charuco`로 카메라별 인트린식+왜곡 JSON 저장
2. `BallDetector` 4종 + `detect_*` 툴이 동일 구현을 공유
3. `VideoCapture` + 검출기가 런타임 파이프라인에 연결되어 `BallObservation` 생성

보정은 **툴에서만** 수행한다. 런타임은 JSON 로드만 한다.

## 비범위

- 멀티캠 하드웨어 동기(트리거·공통 타임스탬프 하드 동기)
- ChArUco **외부** pose 자동 피팅 (R\|t는 후속 툴/수동)
- Magnus / 스핀 추정
- 별도 `Roi` 타입 도입
- 구 Calibration JSON 호환 (스키마 깨짐 허용)

## 아키텍처

```text
[오프라인]
  이미지 디렉터리 ──► calib_charuco ──► calibration.json
       │                                    │
       │              detect_* 4툴          │ (intrinsics + dist;
       └──────────► BallDetector 구현들      │  extrinsics = 후속/수동/sim)
                         │                  │
[런타임]                 ▼                  ▼
  VideoCapture ──► Frame ──► BallDetector ──► BallObservation
       │                              ▲
       │                              │ TOML detector 선택
  Calibration 로드
       │
       └──► (optional undistort) → triangulate_synced → EKF → …
```

### 모듈 경계

| 단위 | 역할 |
|------|------|
| `CameraParams` | `dist: Vec<f64>` **필수**. 외부 R\|t는 이번 사이클에서 피팅하지 않음 |
| `calib_charuco` | 카메라 **1대** `calibrateCameraCharuco` → 인트린식+dist JSON |
| `Frame` / 캡처 | `VideoCapture`·파일 → `(CameraId, Mat, Instant)` |
| `BallDetector` | `detect(&Frame) -> Option<PixelPoint>` |
| 파이프라인 | capture → undistort(dist 비면 no-op) → detect → send |

접근: **프레임 버스 + 검출 포트** (캡처와 검출 분리). 실험 툴과 런타임이 같은 detector 구현을 쓴다.

## Calibration 스키마

`CameraParams`에 OpenCV 관례 `dist: Vec<f64>`를 필수 필드로 추가한다.

- sim / `--emit-sim`: 항상 `dist: []` (왜곡 없음)를 **명시**
- `dist` 필드 없는 JSON → **로드 실패** (호환 레이어 없음)
- `--from-images` 출력: 피팅된 `fx,fy,cx,cy,width,height,dist`
- `rotation` / `translation`: 이번엔 피팅하지 않음. sim look-at 자리표시자 또는 CLI로 기존/sim extrinsics 시드
- 멀티캠: 카메라마다 폴더·JSON을 만든 뒤 `cameras[]` 병합(수동 또는 후속). 자동 번들 외부 보정은 후속

삼각측량: 왜곡 보정된 픽셀 + 기존 `P = K[R|t]`. 왜곡을 P에 넣지 않는다.

### `calib_charuco` 흐름

1. 디렉터리 이미지에서 ChArUco 코너 수집  
   - 보드 기본: 현 초안과 동일 `DICT_4X4_50`, squares `5×7`, 0.04 / 0.02 m — CLI로 덮어쓰기 가능
2. `calibrateCameraCharuco` → `K`, `dist`, RMS
3. JSON 기록 + RMS·사용 프레임 수 로그
4. 코너 부족 → 툴 실패

## 프레임·캡처

`FrameSource`를 힌트 픽셀 API에서 **프레임 소스**로 재정의한다.

```text
next() -> Option<Frame>
Frame { camera_id, image /* BGR Mat */, timestamp }
```

구현:

| 구현 | 용도 |
|------|------|
| `OpenCvCapture` | 장치 인덱스 / 경로 (`VideoCapture`), Windows UVC |
| 이미지 디렉터리 / 동영상 파일 소스 | `detect_*`, 보정 입력 |
| sim | 아래 “sim 경로” |

## 검출

```rust
trait BallDetector: Send {
    fn detect(&mut self, frame: &Frame) -> Option<PixelPoint>;
}
```

| 구현 | 툴 |
|------|-----|
| `ColormaskDetector` | `detect_colormask` |
| `BgSubDetector` | `detect_bgsub` |
| `ContourDetector` | `detect_contour` |
| `RoiDetector` | `detect_roi` |

- 구현체는 `src/detector/`에 둔다. `tools/detect_*`는 CLI 래퍼만.
- 런타임 기본 검출기: **colormask**. 4툴 비교 후 TOML 기본값만 교체.
- 툴은 동일 `BallDetector`에 파일/동영상 입력을 넣고 픽셀(및 선택적 오버레이)을 출력.

## 파이프라인

카메라 스레드:

```text
frame = source.next()?
frame = undistort(frame, calib[camera_id])  // dist empty → no-op
pixel = detector.detect(&frame)?            // None → send 안 함
send BallObservation { pixel, camera_id, timestamp }
```

### sim 경로

`mode = sim`에서는 기존처럼 투영 픽셀을 곧바로 observation으로 넣을 수 있다 (`SimHintSource` 등 **명시적** 우회). 실물·`detect_*`는 반드시 `BallDetector` 경로.

### 에러

| 상황 | 동작 |
|------|------|
| Calibration 파싱 실패 / `dist` 누락 | 기동 실패 |
| `mode=real`인데 `calibration_path` 또는 vision 카메라 설정 누락 | 기동 실패 |
| VideoCapture open 실패 | 해당 캠 기동 실패 (명확한 메시지) |
| 프레임마다 검출 실패 | 관측 스킵 (정상) |
| 캡처 끊김 | 해당 캠 스레드 종료; 채널 종료 패턴은 기존과 동일 |

## 설정 (TOML)

```toml
calibration_path = "calibration.json"

[vision]
detector = "colormask"   # bgsub | colormask | contour | roi

[[vision.cameras]]
id = 0
device = 0               # 또는 path = "..."
```

- `mode = "sim"`: sim 카메라/힌트 경로; `calibration_path` 없으면 `Calibration::sim` (`dist: []`)
- `mode = "real"`: `calibration_path` 필수, `vision.cameras`로 `OpenCvCapture` × N + 공통 detector

### `[vision.colormask]` 기본값

탁구공(주황/흰색) 실험용 출발점. 현장 조명에 맞게 TOML로 덮어쓴다.

| 키 | 기본 | 의미 |
|----|------|------|
| `space` | `"ycrcb"` | `ycrcb` \| `hsv` |
| `y_min`/`y_max` 또는 `h_min`… | YCrCb: Y 0–255, Cr 133–173, Cb 77–127 | inRange |
| `min_area_px` | `20` | 너무 작은 blob 제거 |
| `max_area_px` | `20000` | 너무 큰 blob 제거 |

## 문서 (필수 산출물)

구현과 **같은 커밋 사이클**에서 아래를 실제 명령·경로로 맞춘다. “예정”만 적지 않는다 — 동작하는 플래그·예시를 쓴다.

| 문서 | 내용 |
|------|------|
| `README.md` | 오프라인 보정 → JSON → `detect_*` 비교 → `mode=real`+`[vision]` 런타임 흐름, 상태 표, 도구 표 |
| `config/example.toml` | `calibration_path`, `[vision]`·`[[vision.cameras]]`·`[vision.colormask]` 주석 예시 |
| `docs/phase2.md` | 마일스톤 1/5 관측 항목 진도 동기 |
| `TODO.md` | §3 관측 체크박스 갱신 + 스펙 링크 |
| `assets/` 또는 `docs/` 짧은 메모 (선택) | ChArUco 보드 규격·촬영 팁 |

## 테스트

- 합성 단색 원 이미지 → colormask/contour가 중심 근처
- `CameraParams` serde round-trip (`dist` 필수)
- ChArUco: fixture 이미지가 있으면 RMS 상한; 없으면 calibrate API 스모크/단위 테스트로 코너 수집 경로 검증
- 파이프라인: mock `FrameSource` + stub detector → `BallObservation` 전달

## 완료 기준

1. `calib_charuco --from-images` → 인트린식+`dist` JSON, RMS 출력
2. `detect_*` 4툴이 공유 `BallDetector`로 이미지/동영상에서 픽셀 출력
3. `OpenCvCapture` + 기본 colormask가 파이프라인에 연결되어 observation 생성
4. `dist` 필수 스키마; sim emit도 `dist` 포함
5. README · `example.toml` · `phase2.md` · `TODO.md`가 위 흐름을 **복붙 가능한 명령**으로 설명
6. 위 비범위 항목은 이 스펙에 포함하지 않음

## 구현 순서 (초안)

1. `CameraParams.dist` 필수 + serde/sim/`--emit-sim` 갱신
2. `calibrate_charuco` 실피팅으로 draft 교체 + 툴 CLI
3. `Frame` / `FrameSource` 재정의 + `OpenCvCapture` + 파일 소스
4. `BallDetector` + colormask 먼저, 나머지 3종, `detect_*` 래퍼
5. 파이프라인 연결 + `vision` TOML + undistort
6. README · example · phase2 · TODO를 동작에 맞게 작성
