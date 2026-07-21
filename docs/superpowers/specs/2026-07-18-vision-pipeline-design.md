# 관측 파이프라인 — OpenCV 캡처·검출·ChArUco 인트린식

날짜: 2026-07-18 (검출 모델 2026-07-21 fuse로 갱신)  
상태: 구현 반영

## 목표

실물 관측 본선.

1. 오프라인 `calib_charuco` → 카메라별 인트린식+왜곡 JSON
2. 런타임 검출은 항상 **fuse**: appearance generators → Scorer → MotionPrior
3. `VideoCapture` + fuse → `BallObservation`

보정은 **툴에서만**. 런타임은 JSON 로드만.

## 비범위

- 멀티캠 하드웨어 동기
- ChArUco **외부** pose 자동 피팅
- Magnus / 스핀 추정

## 아키텍처

```text
[오프라인]
  스냅/이미지 ──► calib_charuco ──► calibration.json
  detect-appearance ──► colormask | contour 좌우
  detect-full ──► fuse_from_vision + ROI(`r`) 토글

[런타임]
  VideoCapture ──► Frame ──► fuse_from_vision ──► BallObservation
       │                         ▲
       │                         │ vision.appearance|scorer|motion
  Calibration 로드
       └──► (optional undistort) → triangulate_synced → EKF → …
```

| 레이어 | 역할 | 코드 / TOML |
|--------|------|-------------|
| Appearance | 후보 생성 | `detector/appearance/` · `vision.appearance.*` · `generators` |
| Scorer | area · circularity · motion soft | `detector/scorer.rs` · `vision.scorer` |
| MotionPrior | 움직임 마스크 | `detector/motion/` · `vision.motion.weight` |
| ROI | 탐색 범위 | `track(fuse, roi_half_px)` · detect-full `r` |

툴은 **레이어 디버그**만. peer “방법 선택”이 아니다.

## 완료 기준

- [x] `fuse_from_vision` + `[vision]` nested SSOT
- [x] `detect-appearance` / `detect-full`
- [x] real 경로 `CameraFeed::Detect`에 fuse 연결
