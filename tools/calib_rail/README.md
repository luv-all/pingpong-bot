# calib-rail

리니어 레일을 물리적 엔드스톱까지 저속(`RAIL_HOMING_VELOCITY_M_S`) 이동시켜 AXL 알람으로
도달을 감지하고, 그 지점을 기준으로 `board_zero_domain_m`을 다시 계산해
`data/rail_calibration.json`에 저장한다.

이 독립 도구는 재조립 후, 레일을 다른 마운트로 옮긴 후 등 영점을 별도로
다시 잡을 때 쓴다. 정상적인 `--mode real`은 기본 기동 과정에서 같은 방식의
+X(max) 엔드스톱 홈잉·영점 저장·중앙 복귀를 자동으로 실행한다.

Windows + AXL 보드가 실제로 연결된 벤치에서만 동작한다(`feature = "real"`이 없는
빌드나 비Windows에서는 하드웨어 초기화 단계에서 에러로 종료한다).

`AxlRail::open`만 사용한다 — Dynamixel 팔은 열지 않는다. 팔이 정렬 안 됐거나 연결
안 돼 있어도 레일 홈잉은 그와 무관하게 실행할 수 있다.

## 사용

```bash
# 기본: 왼쪽(min) 엔드스톱까지 홈잉
cargo run -p calib-rail

# 오른쪽(max) 엔드스톱
cargo run -p calib-rail -- --end max

# DLL 경로 오버라이드
cargo run -p calib-rail -- --dll-path "C:/path/to/AXL.dll"
```

## 안전

- 실제로 저속이지만 물리적 엔드스톱까지 이동한다 — 레일 경로에 장애물이 없는지,
  비상정지 스위치 위치를 확인한 뒤 실행할 것.
- 처음 실행할 때는 사람이 옆에서 지켜보며 1회 확인하는 것을 권장한다.
- 자세한 절차는 `docs/rail-calibration.md` 참고.
