# 비전 성적표 (새 리그, 2026-08-05)

`cargo run --release -p clip-review -- --all` 이 만든 것 중 PR 근거로 남긴 사본이다.
전체는 `data/review/`에 있고 그건 커밋하지 않는다 (언제든 다시 만든다).

`_sim.png` — 위(x-y)·옆(y-z) 직교 투영. 두 축 축척이 같다.

| 색 | 무엇 |
|---|---|
| 초록 | 생 삼각측량 — 필터 밖의 값 |
| 하늘색 | 적합 궤적 (`Trajectory::measured`) |
| 자홍 | 예측 궤적 (`Trajectory::predicted`), 트리거 순간에 얼림 |

`_cam.png` — 트리거 순간의 두 카메라와 그 위 오버레이.

`summary.txt` — 클립별 지표. 열 뜻은 파일 끝에 적혀 있다.
