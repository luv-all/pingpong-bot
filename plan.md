# 핑퐁 로봇 — 계획 안내

이 문서는 더 이상 상세 설계의 기준이 아니다. 과거의 `domain`/`infra`/`app`
크레이트 분할, 비전 포트, `pin_to_p_core`, peer detector 설계는 폐기됐다.

현재 기준 문서는 다음과 같다.

- 구조와 실행법: [`README.md`](README.md)
- 확정된 설계 결정과 이유: [`docs/decisions.md`](docs/decisions.md)
- 남은 작업과 우선순위: [`TODO.md`](TODO.md)
- 현재 레일·손목 제어: [`docs/two-stage-position-control.md`](docs/two-stage-position-control.md)
- 앱 기본값과 조립: [`src/defaults/`](src/defaults/)

## 유지하는 목표

사람을 이기는 것이 아니라, 안전하고 일관된 리턴으로 사람과 가능한 오래 랠리한다.
따라서 공격성보다 리턴 성공률, 변동성, 복구 시간, 장시간 안정성을 우선한다.

현재 활성 경로는 다음과 같다.

```text
BallTrajectory
    → CommitRequest (track_seq, Provisional | Refined)
    → DirectController
    → DirectControlCommand
    → real Hardware / GUI sim robot::State
```

현재 실기 제어는 전체 스윙이 아니라 리니어 레일과 손목 ID 5의 2단계 장비
시험이다. 명령 후에는 실제 적용값을 기준으로 레일·손목을 다시 읽어 수렴 여부를
판정한다. 다음 우선순위는 Windows 실물 장비 통합 검증과 보존 중인 구형
`PositionController`·스윙 계획기의 향후 사용 여부 결정이다.
