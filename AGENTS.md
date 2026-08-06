# AGENTS.md

## Cursor Cloud specific instructions

Durable, non-obvious notes for running this Rust `pingpong-bot` crate in the Cloud VM.
Standard build/test/run commands live in `README.md` (`## 빠른 시작`, `## 개발`); this
section only records the gotchas that are not obvious from the repo.

### System dependencies (baked into the VM snapshot — not in the update script)
- **Rust stable** (edition 2024 needs rustc ≥ 1.85). The repo's `mise.toml` requests
  `rust = "latest"`, but mise is not used here; `rustup default stable` is set instead.
- **OpenCV 4.13.0 is built from source and installed to `/usr/local`.** This is required:
  Ubuntu's apt package is only 4.6, but the code uses APIs added in OpenCV 4.7+
  (`core::AlgorithmHint`, Charuco types in the main `objdetect` module, the extra
  `cvtColor` hint arg). `pkg-config --modversion opencv4` must report `4.13.0`
  (it resolves via `/usr/local/lib/pkgconfig` before the apt `.pc`). Do **not** rely on
  the apt OpenCV — the crate will fail to compile against 4.6.
- **libclang** (llvm-18) is present; `clang-sys` finds it automatically, so
  `LIBCLANG_PATH` does **not** need to be set on Linux. `libstdc++-14-dev` must be
  installed or the OpenCV binding generator fails with `fatal error: 'memory' file not found`.

### Running the GUI sim headless (the core product)
- The default binary is a GUI sim (kiss3d + egui, `gui` feature, on by default) that uses
  **wgpu**. On the headless VM you must use the desktop display and force software rendering:
  ```bash
  DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 WGPU_BACKEND=gl cargo run -p pingpong-bot -- --debug
  ```
  Without `LIBGL_ALWAYS_SOFTWARE=1` wgpu fails with "Failed to find an appropriate adapter".
- The `--mode real` / `--features real` path is Windows + physical Dynamixel/AXL hardware
  only; it cannot run in this VM. Sim mode is the only end-to-end path here.

### Verifying the rally loop (fire ball → robot returns it)
- Software rendering (llvmpipe) is **low FPS**, so the robot's return swing (~0.3 s of sim
  time) is very hard to catch in a single rendered frame or screenshot, even in slow motion.
  The ball trajectory is drawn as a persistent trail so it stays visible, but the transient
  arm pose usually falls between rendered frames.
- To confirm the robot actually plans and commits swings, run with `--debug` and watch stdout
  for lines like `shot: swing commit ... committed=true peak_joint_speed=...` and
  `shot: end — park`. In the GUI, the `Status` panel's `Robot` section shows `스윙 확정`
  (swing confirmed) and the `Impact` section shows the predicted impact point while a ball is
  in flight. Use the `View` panel's `배속` (time-scale) slider to slow motion down.

### Known pre-existing issues (code-level, not environment)
Under current stable Rust + OpenCV 4.13 these fail but are unrelated to environment setup:
- `cargo test -p pingpong-bot --lib`: 1 failure,
  `hardware::dynamixel::tests::motor_mapping_matches_python_reference` (joint-sign mapping
  assertion). The other 240 tests pass.
- `cargo clippy`: 2 denied `clippy::approx_constant` errors (hand-written `FRAC_PI_6`), plus
  many warnings. `cargo fmt --check` also reports diffs. Lint tooling itself works.
