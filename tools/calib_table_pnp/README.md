# calib-table-pnp

탁구대 **규격 랜드마크 8점**을 클릭해 OpenCV `solvePnP`(IPPE)로 카메라 외참 `R|t`를 잡고, 같은 창에서 **월드 XY×Z 무지개 격자**로 투영을 확인한 뒤 `Calibration` JSON을 쓴다. Charuco 없이 FOV로 `K`만 근사 (`dist=[]`).

`--cam left|right` **필수** (한 대씩 캘리브 — 생략 시 어느 쪽인지 모호하므로 막음).

라이브 스트림·FOV 기본은 **Arducam B0332** datasheet SSOT (`arducam_b0332`: 1280×800@120 MJPG, HFOV70°→VFOV≈47.3°).

저장/로드 기본 경로는 **`data/calibration.json`** (`defaults::calib::DEFAULT_CALIBRATION_PATH`). left/right 각각 실행해도 같은 번들에 upsert.

| 파일 | 역할 |
|------|------|
| `interactive.rs` | Space 스냅 · LMB 클릭 · 자동 PnP · 격자 · s 저장 |
| `adjust.rs` | 8점 미세조정 상태·순수로직 (선택 · 이동 · undo · bounded refine) |
| `overlay.rs` | 패딩 캔버스 · 클릭 · 재투영 메시 · 잔차 그리기 |
| `cli.rs` | `--from-pixels` / `--validate` / merge |
| `args.rs` | clap |

월드 격자는 lib `draw_world_grid` (점+선).

## 흐름

1. 기존 `calibration.json`에 이 카메라가 있으면 **라이브부터** 격자 오버레이 (첫 클릭까지)
2. `Space` — 스냅 (Review). 클릭 0개면 계속 기존 격자
3. LMB — 첫 클릭부터 **recalib** (baseline 숨김) · 8점 → **자동 PnP**
4. RMSE OK → pending 사이드카 자동 저장 + 무지개 격자. FAIL여도 **초록(클릭) vs 마젠타(이상 재투영)** + 노란 잔차선
5. 8/8이면 **조정 모드** — 지우지 말고 잔차 큰 점을 `1`–`8`로 골라 1px씩 밀면서 재-PnP (아래 "조정" 절)
6. `s` — 본파일 upsert 후 pending 삭제. `q`해도 pending은 남음 (재실행 후 `s`만으로도 promote 가능)

## 사용

```bash
# -o 생략 → data/calibration.json (카메라별 upsert)
cargo run -p calib-table-pnp -- --cam left
cargo run -p calib-table-pnp -- --cam right

cargo run -p calib-table-pnp -- --cam left --backend dshow
cargo run -p calib-table-pnp -- --path capture.mp4 --cam left

# 다른 파일로 쓰거나 합치기
cargo run -p calib-table-pnp -- --cam left -o other.json
cargo run -p calib-table-pnp -- --cam right --merge other.json -o data/calibration.json
```

| 키 | 동작 |
|----|------|
| `Space` | 스냅 |
| `LMB` / `Enter` | 랜드마크 클릭 (aim 위치) |
| `←↑→↓` / `hjkl` | aim 1px (마우스 이동 시 재동기화; nudge 중 loupe 유지) |
| `Shift`+이동 | 8× loupe |
| `z` / `c` | 마지막 클릭 취소 / 전부 비우기 |
| `s` | 본파일 promote (세션 accepted 또는 디스크 pending) |
| `n` | live |
| `q` | 종료 (pending 유지) |

`--pad N` (기본 16): Review 캔버스 외곽에 Npx 회색 체크 패딩. 프레임에 잘린 랜드마크를 이미지 좌표(음수·폭 초과)로 찍을 수 있다. `--pad 0`이면 패딩 없음.

오버레이: **초록○** = 클릭, **시안◎** = 선택된 점, **마젠타×·마젠타 메시** = PnP 이상 재투영(= 투영된 탁구대 평면), **노란선** = 잔차(px), **회색 원** = 선택된 점의 refine 반경.

pending: 공유 `data/calibration.pending.json`에 `cameras[]` upsert. `s`는 현재 cam만 본파일로 promote하고 pending에서 해당 항목만 제거. 조정 중에는 쓰지 않고 **마지막 조정 후 0.4초 조용해지면** 한 번 쓴다 (키 하나마다 JSON 쓰기 방지).

## 조정 — 8점을 지우지 않고 고치기

8점이 다 찍히면 `z`로 되돌려 다시 찍는 대신 **그 자리에서 점을 골라 미세 이동**한다. 이동마다 PnP를 다시 풀어 마젠타 평면·잔차가 실시간으로 갱신된다.

| 키 | 동작 |
|----|------|
| `1`–`8` | 그 랜드마크 선택 |
| `Tab` | 선택 순회 |
| `0` | 선택 해제 (방향키가 다시 aim을 움직임) |
| `←↑→↓` / `hjkl` | **선택된 점** 1px 이동 (선택 없으면 기존대로 aim) |
| `H` `J` `K` `L` | 선택된 점 5px 이동 |
| `Enter` | 선택된 점을 현재 aim 위치로 |
| `LMB` | 12px 안의 점 잡기(선택). 반경 밖이면 선택된 점을 그 자리로 |
| `u` | 조정 1단계 되돌리기 (`fov_y`도 함께, 최대 64단계) |
| `r` | 자동 미세탐색 — `--refine-radius` 이내로 RMSE 최소화 |
| `f` / `F` | `fov_y` −0.5° / +0.5° |

선택 중에는 loupe가 마우스 대신 **그 점**에 붙고, HUD에 `SEL 3:c11 res=4.2px d=(+2,-1)`(원래 클릭 대비 이동량)이 나온다.

### `f`/`F` — 같은 RMSE에서 더 그럴듯한 투영

랜드마크 8점은 전부 `z = SURFACE_Z` **한 평면**이고 `K`는 FOV 추정(`dist=[]`)이다. 동일평면 대응에서는 pose가 호모그래피로 결정되므로 **초점거리 오차가 평면 기울기 오차로 흡수된다** — RMSE는 거의 그대로인데 외삽된 마젠타 평면·무지개 격자만 틀어진다. 그래서 클릭을 안 건드리고 `f`/`F`로 `fov_y`를 흔들어 눈으로 맞추는 게 이 경우의 정답이다.

같은 이유로 **자동 FOV 피팅은 넣지 않았다** (RMSE로 focal이 거의 관측되지 않아 조건화가 나쁘다). 맞춘 값은 종료·promote 시 `--fov-y 45.5` 형태로 출력되니 다음 실행에 넘기면 된다.

### `r` — 경계 있는 자동 미세탐색

`--refine-radius N` (기본 3px). 각 점을 **8점을 다 찍은 순간의 원래 위치**에서 `N`px 이내로만 0.5px씩 옮겨 RMSE를 최소화한다 (좌표하강).

**반경이 유일한 안전장치다.** pose가 클릭에서 매번 재적합되므로 경계가 없으면 클릭을 실제 영상 특징에서 떼어내 서로 완벽히 일관된 배치로 옮겨버린다 — RMSE는 0으로 가지만 캘리브는 무의미해진다. 기본값을 작게 두고, 선택된 점의 반경을 화면에 회색 원으로 그린다. 결과가 마음에 안 들면 `u`. `--refine-radius 0`이면 `r`은 no-op.
