# detect-bgsub

연속 프레임 차분으로 공을 찾는다. 런타임 `BgSubDetector`와 동일.

## 프리뷰·디버그

창 이름 `detect:bgsub`. hit 원·직전 hit·frame/miss/hit율 텍스트. `q`/`ESC` 종료.

## 사용

```bash
cargo run -p detect-bgsub -- --path clip.mp4
cargo run -p detect-bgsub -- --device 0
cargo run -p detect-bgsub -- --images ./frames -o out/
```

입력: `--images` \| `--path` \| `--device`  
기타: `-o`, `--max-frames`, `--no-preview`, `--wait-ms`  
상세 표: [detect-colormask/README](../detect_colormask/README.md)
