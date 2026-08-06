# 핑퐁 로봇 — 계획 안내

이 문서는 더 이상 상세 설계의 기준이 아니다. 과거의 `domain`/`infra`/`app`
크레이트 분할, 비전 포트, `pin_to_p_core`, peer detector 설계는 폐기됐다.

현재 기준 문서는 다음과 같다.

- 구조와 실행법: [`README.md`](README.md)
- 확정된 설계 결정과 이유: [`docs/decisions.md`](docs/decisions.md)
- 남은 작업과 우선순위: [`TODO.md`](TODO.md)
- 현재 레일·라켓 조준 제어: [`docs/two-stage-position-control.md`](docs/two-stage-position-control.md)
- 앱 기본값과 조립: [`src/defaults/`](src/defaults/)

## 유지하는 목표

사람을 이기는 것이 아니라, 안전하고 일관된 리턴으로 사람과 가능한 오래 랠리한다.
따라서 공격성보다 리턴 성공률, 변동성, 복구 시간, 장시간 안정성을 우선한다.

현재 활성 경로는 다음과 같다.

```text
vision::Trajectory
    → CommitRequest (track_seq)
    → 제어 워커의 인터셉트 평면 선택
    → 첫 명령: Planner::ball_alignment → Hardware::command
    → 같은 트랙 보정: Planner::ball_alignment_fixed_rail
                         → Hardware::command_joints
```

현재 실기 제어는 전체 스윙이 아니라 예측 불확실성 기준을 통과한 궤적으로
접촉 위치와 라켓 방향을 맞춘다. 첫 명령은 레일과 관절을 함께 움직이고, 같은
트랙의 후속 예측은 레일을 고정한 채 관절만 보정한다. 라켓 면은 상대 코트
절반의 중심을 향하며, 예측 도착 시각 0.5초 뒤 현재 모드의 준비 자세로
복귀한다. 명령 완료 시 읽기와 로그는 한 번 수행하지만 주기적
`PendingVerification` 수렴 판정은 현재 런타임에서 사용하지 않는다.

다음 우선순위는 Windows 실물 장비 통합 검증과 비활성 검증 경로 및 보존 중인
구형 `PositionController`·스윙 계획기의 향후 사용 여부 결정이다.
