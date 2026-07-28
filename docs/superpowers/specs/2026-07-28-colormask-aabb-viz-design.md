# tune-colormask AABB 시각화 (1단계)

날짜: 2026-07-28  
상태: approved (대화 승인 → 구현)

## 문제

하단 min→max **대각 보간 띠**는 AABB 대각선만 보여 샘플(주황)과 무관한 갈/초록이 나온다.  
`inRange`는 박스 전체를 통과시키므로 띠는 검출 의미를 오해하게 한다.

## 범위 (1단계)

- `tools/tune_colormask` UI만 변경
- 검출기·`colormask.json`·퍼센타일/`--trim` 변경 없음
- 2단계(축 재정의·타원 실험 하네스)는 이 스펙 밖

## UI

```
[ original | mask ]
[ sample swatch — 실제 BGR ]
[ scatter c0-c1 | c0-c2 | c1-c2 | isometric AABB wire + points ]
```

- 산점도: 현재 space 채널 쌍, 점은 샘플 BGR, AABB는 해당 2축 사각형
- 아이소메트릭: 고정각 와이어 큐브(AABB) + 샘플 점 (직관용; 깊이 손실 있음)
- 대각선 그라디언트 띠 **삭제**

## 성공 기준

픽한 주황 점이 산점도에 보이고, AABB 면과 비교 가능하다. 가짜 갈→초록 그라디언트가 없다.
