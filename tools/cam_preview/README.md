# cam-preview

다중 웹캠을 **가로 한 창**으로 보는 프리뷰.

카메라 인자는 공용 SSOT (`StereoCamCliArgs`): `--cam left|right` + 스트림.
USB 장치 번호는 CLI에 없고 [`CamRigConfig`](`pingpong_bot`)가 역할별로 부여한다 (기본 left→0, right→1).

```bash
# 기본: left+right, B0332 SSOT (1280×800@120 MJPG, HFOV70°)
cargo run -p cam-preview

cargo run -p cam-preview -- --cam left
cargo run -p cam-preview -- --cam left,right --backend dshow
cargo run -p cam-preview -- --threaded
```

시작 시 콘솔에 `backend=` / `fourcc=` / `fps=` / `size=` 를 찍고, 요청과 다르면 `WARN stream mismatch`를 낸다.

기본값은 [Arducam B0332 datasheet](https://cdn.robotshop.com/media/A/Adu/RB-Adu-256/pdf/arducam-1mp-ov9281-usb-camera-120fps-global-shutter-uvc-low-distortion-m12-lens-datasheet.pdf) SSOT (`pingpong_bot::arducam_b0332`):

- **1280×800** MJPG **120fps** (YUY2는 10fps만 — 고FPS 불가)
- 렌즈 HFOV **70°** (EFL 2.8mm)

| 키 | 동작 |
|----|------|
| `Space` | 동결 / 해제 |
| `e` | 짧은 노출 재시도 (macOS AVFoundation에선 대개 무시; `--threaded`에선 N/A) |
| `q` / ESC | 종료 |
