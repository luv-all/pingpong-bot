# cam-preview

다중 웹캠을 **가로 한 창**으로 보는 프리뷰.

카메라 인자는 공용 SSOT (`StereoCamCliArgs`): `--cam left|right` + 스트림.
USB 장치 번호는 CLI에 없고 [`CamRigConfig`](`pingpong_bot`)가 역할별로 부여한다 (기본 left→0, right→1).

```bash
# 기본: left+right, B0332 full, recommended 백엔드, grab 스레드 on
cargo run -p cam-preview

# Windows 듀얼: MSMF가 recommended. 대역 실험은 --preset
cargo run -p cam-preview -- --preset mid
cargo run -p cam-preview -- --preset low
cargo run -p cam-preview -- --cam left
cargo run -p cam-preview -- --threaded=false
```

시작 시 콘솔에 `backend=` / `fourcc=` / `fps=` / `size=` 를 찍고, 읽을 수 있는 FOURCC·size만 mismatch WARN을 낸다 (`????`·`CAP_PROP_FPS` 허위는 무시 → **meas FPS** 신뢰).

기본값은 [Arducam B0332 datasheet](https://cdn.robotshop.com/media/A/Adu/RB-Adu-256/pdf/arducam-1mp-ov9281-usb-camera-120fps-global-shutter-uvc-low-distortion-m12-lens-datasheet.pdf) SSOT (`pingpong_bot::arducam_b0332`):

- **full** 1280×800 MJPG **120fps** (YUY2는 10fps만 — 고FPS 불가)
- **mid** 960×600 / **low** 640×400 — 듀얼 USB 대역 트레이드오프
- 렌즈 HFOV **70°** (EFL 2.8mm)
- Windows `recommended` → **MSMF** (DSHOW는 YUY2 함정)

| 키 | 동작 |
|----|------|
| `Space` | 동결 / 해제 |
| `e` | 짧은 노출 재시도 (macOS AVFoundation에선 대개 무시; `--threaded`에선 N/A) |
| `q` / ESC | 종료 |
