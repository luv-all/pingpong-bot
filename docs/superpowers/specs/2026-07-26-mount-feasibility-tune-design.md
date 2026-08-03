# Mount feasibility tune (weak swing)

**날짜:** 2026-07-26  
**상태:** approved → implementing

## Goal

실기 Dynamixel 관절속도 한계(~2.88 rad/s)는 유지한 채, 마운트 기하로
`peak_joint_speed_ratio`를 낮춰 임팩트 끝속도가 덜 스케일다운되게 한다.

## Non-goals

- `DYNAMIXEL_MAX_JOINT_SPEED_RAD_S` 상향 (시뮬만 관대)
- `solve_impact_target` / `fit_end_velocity` 스케일 완화

## Procedure (C)

1. `tools/mount_search` — `base_y` × `height_offset` 스윕, ratio≤1 비율·mean/worst로 순위
2. 상위 후보를 `defaults::rail_frame` (+ URDF mount가 같은 상수를 쓰면 함께) 반영
3. 같은 마운트로 `tools/shot_tune` Rapier 랠리 성공률 확인; 필요 시 슈터만 미세 조정
4. 성공률이 부족할 때만 `--rest-pose-search`

## Success

- (A) 현 마운트 대비 실현가능(ratio≤1) 시나리오 비율 상승 — **달성**
  - `behind=0.02`: 0/150, mean≈3.79
  - `behind=0.10`: 10/150, mean≈2.48
- (B) 그 마운트에서 랠리 성공률 유지 또는 상승 — **달성**
  - −0.02: 17/216 (7.9%), avg median peak 2.38
  - −0.10: 29/216 (13.4%), avg median peak 1.79

## Applied

`defaults::rail_frame`: `behind_table_end=0.10`, `above_table=0.05`
