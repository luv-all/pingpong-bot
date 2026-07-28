# cam-list

OpenCV가 보는 **카메라 device 인덱스**를 프로브한다.

USB를 다시 꽂거나 OBS/다른 앱과 섞이면 번호가 바뀐다. OBS에 보여도 OpenCV 인덱스는 다를 수 있으니, `defaults::calib`의 `LEFT_DEVICE` / `RIGHT_DEVICE`를 맞출 때 쓴다.

hinguri 시절 Python [`cv2_enumerate_cameras`](https://github.com/lukehugh/cv2_enumerate_cameras)와 같은 역할이다. (Rust OpenCV에는 이름 열거 API가 없어 **인덱스·백엔드·한 프레임 성공**만 본다.)

```bash
# Windows 기본 recommended(=MSMF)로 0..7 프로브
cargo run -p cam-list

# DSHOW 인덱스도 따로 (백엔드마다 번호가 다를 수 있음)
cargo run -p cam-list -- --backend dshow
cargo run -p cam-list -- --all-backends

# 열린 장치마다 프리뷰 (q/ESC → 다음)
cargo run -p cam-list -- --preview
```

출력 예:

```
defaults::calib 현재 매핑: LEFT_DEVICE=0  RIGHT_DEVICE=1

=== backend=msmf (api=1400) ===
  device 0: OPEN  frame=ok grabbed=1280x800 | backend=MSMF fourcc=???? fps=30 size=1280x800
  device 1: OPEN  frame=ok grabbed=1280x800 | backend=MSMF fourcc=???? fps=30 size=1280x800
  device 2: —  (...)
  → 2 device(s) opened
```

그다음 `src/defaults/calib.rs`에서 left/right에 해당하는 숫자를 넣고:

```bash
cargo run -p cam-preview -- --cam left
cargo run -p cam-preview -- --cam right
```
