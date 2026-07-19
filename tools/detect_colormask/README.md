# detect-colormask

YCrCb/HSV 색 마스크로 공 픽셀을 찾는다. 런타임 `ColormaskDetector`와 동일.

## 프리뷰·디버그

실행하면 **캡처 창** `detect:colormask`가 뜬다.

| 표시 | 의미 |
|------|------|
| 초록 원 | 현재 프레임 hit |
| 주황 원 | 직전 hit |
| 텍스트 | frame / hit·miss·px / 누적 hit율 |

`q` · ESC 종료. `--no-preview`면 창 없이 콘솔·`-o`만.

## 사용

```bash
cargo run -p detect-colormask -- --path clip.mp4
cargo run -p detect-colormask -- --device 0
cargo run -p detect-colormask -- --images ./frames -o out/
cargo run -p detect-colormask -- --path clip.mp4 --no-preview -o out/
```

| 플래그 | 설명 |
|--------|------|
| `--images DIR` | 이미지 시퀀스 |
| `--path FILE` | 동영상 |
| `--device N` | 웹캠 |
| `-o DIR` | 오버레이 PNG |
| `--max-frames` | 라이브/영상 상한 (기본 300) |
| `--no-preview` | highgui 끄기 |
| `--wait-ms` | waitKey ms |

공통 비교: [detect-compare](../detect_compare/README.md)
