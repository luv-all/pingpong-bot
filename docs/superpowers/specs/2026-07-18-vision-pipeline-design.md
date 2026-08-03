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
  스냅/이미지 ──► calib_table_pnp ──► calibration.json
  detect-full ──► fuse + ROI(`r`) 토글
  verify-stereo ──► 스테레오 격자·삼각·SimScene 공

[런타임]
  VideoCapture ──► Frame ──► detector_for ──► BallObservation
       │                         ▲
       │                         │ defaults / colormask.json
  Calibration 로드
       └──► (optional undistort) → triangulate_synced → EKF → …
```

| 레이어 | 역할 | 코드 / TOML |
|--------|------|-------------|
| Appearance | 후보 생성 | `detector/appearance/` · `generators` |
| Scorer | area · circularity · motion soft | `detector/scorer.rs` |
| MotionPrior | 움직임 마스크 | `detector/motion/` |
| ROI | 탐색 범위 | `track(fuse, …)` · detect-full `r` |

툴은 **레이어 디버그**만. peer “방법 선택”이 아니다.

## 완료 기준

- [x] fuse + defaults SSOT (`detector_for`)
- [x] `detect-full`
- [x] `verify-stereo` (스테레오 검증)
- [x] real 경로 `CameraFeed::Detect`에 fuse 연결