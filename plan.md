# 핑퐁 로봇 — 계획 안내

이 문서는 더 이상 상세 설계의 기준이 아니다. 과거의 `domain`/`infra`/`app`
크레이트 분할, 비전 포트, `pin_to_p_core`, peer detector 설계는 폐기됐다.

현재 기준 문서는 다음과 같다.

- 구조와 실행법: [`README.md`](README.md)
- 확정된 설계 결정과 이유: [`docs/decisions.md`](docs/decisions.md)
- 남은 작업과 우선순위: [`TODO.md`](TODO.md)
- 앱 기본값과 조립: [`src/defaults/`](src/defaults/)

## 유지하는 목표

사람을 이기는 것이 아니라, 안전하고 일관된 리턴으로 사람과 가능한 오래 랠리한다.
따라서 공격성보다 리턴 성공률, 변동성, 복구 시간, 장시간 안정성을 우선한다.

현재 작업 순서는 다음과 같다.

1. `BallTrajectory`(`observed`/`predicted`, `N×7`)를 real 추정 워커의
   최종 출력으로 연결한다.
2. `HitTargetSelector`가 예측 궤적의 행 또는 두 행 사이 보간점에서
   시간·위치 목표를 선택하게 한다.
3. 제어 입력을 `Target { position, arrival_time_secs }`로 바꾸고,
   스윙 없이 목표 위치로 이동·대기하는 경로를 구현한다.
4. 영상 → 7열 궤적 → 목표 선택 → sim/real dry-run 위치 이동을
   통합 테스트한다.
5. 위치 이동이 안정되면 `Prediction`/`HitPlane`/`Impact`에 묶인
   기존 스윙 경로를 제어 계층에서 순서대로 제거한다.

현재 1번의 도메인 API와 예측 샘플러는 구현됐으며,
real `CommitRequest` 이후는 아직 기존 `Vec<Prediction>` 제어 경로다.
