# calib-table-pnp

탁구대 **규격 랜드마크 8점**을 클릭해 OpenCV `solvePnP`(IPPE)로 카메라 외참 `R|t`를 잡고, 기존과 같은 `Calibration` JSON을 쓴다. Charuco 없이 FOV로 `K`만 근사 (`dist=[]`).

| 파일 | 역할 |
|------|------|
| `interactive.rs` | Space 스냅 · LMB 클릭 · s 저장 |
| `cli.rs` | `--from-pixels` / `--validate` / merge |
| `args.rs` | clap |

라이브러리 SSOT: `pingpong_bot::camera::calib` (`table_landmarks` / `calibrate_table_pnp`).

## 랜드마크 순서 (고정)

1. 로봇쪽 왼쪽 `(0, 0, SURFACE_Z)`
2. 로봇쪽 오른쪽 `(W, 0, SURFACE_Z)`
3. 상대쪽 오른쪽 `(W, L, SURFACE_Z)`
4. 상대쪽 왼쪽 `(0, L, SURFACE_Z)`
5. 테이블 중앙 `(W/2, L/2, SURFACE_Z)`
6. 로봇 반쪽 내부 `(W/2, L/4, SURFACE_Z)`
7. 상대 반쪽 내부 `(W/2, 3L/4, SURFACE_Z)`
8. 로봇쪽 변 중점 `(W/2, 0, SURFACE_Z)`

재투영 RMSE **≤ 3 px**(기본 `--max-rmse`)일 때만 저장.

## 사용

```bash
# 캠 0
cargo run -p calib-table-pnp -- --device 0 -o calibration.json

# 캠 1을 같은 JSON에 합치기
cargo run -p calib-table-pnp -- --device 1 --camera-id 1 \
  --merge calibration.json -o calibration.json

# 녹화본
cargo run -p calib-table-pnp -- --path capture.mp4 --camera-id 0 -o cam0.json
```

| 키 | 동작 |
|----|------|
| `Space` | 스냅 (심사 모드) |
| `LMB` | 다음 랜드마크 클릭 |
| `z` | 마지막 클릭 취소 |
| `c` | 클릭 전부 삭제 |
| `s` | PnP → RMSE 검사 → JSON |
| `n` | 다시 라이브 |
| `q` | 종료 |

```bash
# 픽셀만으로 (테스트/자동화)
cargo run -p calib-table-pnp -- --from-pixels pixels.json --fov-y 55 -o out.json

cargo run -p calib-table-pnp -- --validate calibration.json
```

`pixels.json` 예:

```json
{
  "width": 640,
  "height": 480,
  "pixels": [[100,200],[500,200],[520,400],[80,400],[300,300],[300,250],[300,350],[300,200]]
}
```

멀티캠 JSON은 `measure-restitution` / 파이프라인 `calibration` 로드 경로와 동일하다.
