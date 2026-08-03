# Design: verify-proj (월드 그리드 투영 검증)

> **Superseded (2026-07-24):** 격자 오버레이는 [`calib-table-pnp`](../../../tools/calib_table_pnp/README.md) 인터랙티브 루프에 흡수됨. 독립 크레이트 `tools/verify_proj`는 제거.

**Date:** 2026-07-24  
**Status:** superseded — absorbed into calib-table-pnp  
**Goal:** table-PnP `Calibration`의 `project_world`(= DLT `P = K[R|t]`)를 라이브 멀티캠에 XY×Z 격자로 투영해 눈으로 검증한다.

## 운영 캘리브 전제

- Charuco 인트린식/`dist`는 **운영에서 하지 않음** (렌즈 왜곡이 작음).
- 카메라 파라미터는 `calib-table-pnp`만: FOV로 `K` 근사 + 탁구대 8점 `solvePnP` → `R|t`, `dist=[]`.
- 그 JSON으로 멀티뷰 DLT (`triangulate_*`) 한다.
- `verify-proj`는 **PnP 직후 · DLT 직전 게이트**. PnP RMSE는 Z=0만 보므로 XY 격자 + Z 상승 기둥으로 높이·전 FOV를 막는다.

`calib-charuco` 코드는 유지하되 운영 경로에는 넣지 않는다.

## Scope

| In | Out |
|----|-----|
| 라이브 캠, `--calibration` + `--device` | 영상/폴더 입력 |
| 가로 타일 한 창, 키로 그리드 조절 | Charuco/`dist` 토글 |
| `undistort_frame` 호출 (운영 JSON이면 no-op) | PnP 클릭 UI, DLT RMSE UI |
| `draw_world_grid` in `camera/io/preview` | |

## Flow

```
탁구대 8점 → calib-table-pnp → Calibration JSON (dist=[])
                                    ↓
                              verify-proj (본 툴)
                                    ↓
                              DLT triangulate → detect / pipeline
```

툴 내부: `OpenCvCapture` → `undistort_frame` → `project_world` 격자 → `hstack_bgr` → `show_bgr`.

## Keys

| 키 | 동작 |
|----|------|
| `=` / `+` | XY 간격 ↑ |
| `-` | XY 간격 ↓ |
| `]` / `[` | Z 층 수 ↑ / ↓ |
| `.` / `,` | Z 간격 ↑ / ↓ |
| `Space` | 동결/해제 |
| `q` / ESC | 종료 |

초기: `xy_step=0.10`, `z_step=0.05`, `z_layers=6`.

## Success

- 다중 캠 한 창에 동일 월드 격자 투영
- Z=0(빨강) ↔ 물리 테이프 교차
- Z 기둥 수직·정렬 → DLT 투입 전 OK
