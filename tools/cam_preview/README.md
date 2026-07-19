# cam-preview

순수 OpenCV 다중 웹캠 프리뷰. CLI 없음.

`src/main.rs`의 `DEVICES` 배열만 바꾼다. 프레임을 **가로로 이어 붙인 한 창**.

```rust
const DEVICES: &[i32] = &[0, 1];
```

```bash
cargo run -p cam-preview
```

`q` / ESC 종료.
