# detect-compare

같은 프레임에 `colormask` / `bgsub` / `contour` / `roi`를 돌리고 **2×2 한 창**으로 비교한다.

## 프리뷰·디버그

창 이름 `detect:compare`.

| 표시 | 의미 |
|------|------|
| 패널별 색 원 | 해당 검출기 hit |
| 좌상단 텍스트 | frame, 검출기별 px 또는 miss, 누적 hits |

stdout은 TSV (`frame	colormask	bgsub	contour	roi`). `q`/`ESC` 종료.

## 사용

```bash
cargo run -p detect-compare -- --path clip.mp4
cargo run -p detect-compare -- --device 0
cargo run -p detect-compare -- --images ./frames -o out/
cargo run -p detect-compare -- --path clip.mp4 --no-preview -o out/
```

| 플래그 | 설명 |
|--------|------|
| `--images` / `--path` / `--device` | 입력 (택1) |
| `-o DIR` | 2×2 오버레이 PNG |
| `--max-frames` | 상한 (기본 300) |
| `--no-preview` | 창 끄기 |
| `--wait-ms` | waitKey ms |

개별 툴: [colormask](../detect_colormask/README.md) · [bgsub](../detect_bgsub/README.md) · [contour](../detect_contour/README.md) · [roi](../detect_roi/README.md)
