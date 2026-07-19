# measure-friction

테이블 위 **롤** 구간에서 접선 감속으로 마찰 $\mu$를 구하고 `[physics].friction`에 넣는다.

모델: $\mu = 1 - |v_t'|/|v_t|$

## 영상 (권장)

멀티캠 + 캘리브 → 검출 → 삼각측량 → 테이블 위 저 $|v_z|$ 구간에서 $v_t$ 감속.

창 이름 `measure:friction` — 카메라 패널 **가로 한 창**.

| 표시 | 의미 |
|------|------|
| 초록 원 | 검출 |
| 노랑 / 시안 원 | 롤 구간 시작·끝 |
| (바운스 있으면) | $v_\mathrm{in}$/$v_\mathrm{out}$·접촉 원도 표시 |
| 텍스트 | $\mu$, $v_t$ in/out, $p_0$/$p_1$ |

```bash
cargo run -p measure-friction -- \
  --calibration calib.json \
  --video cam0.mp4 --video cam1.mp4

cargo run -p measure-friction -- \
  --calibration calib.json \
  --device 0 --device 1 --dry-run
```

`q`/`ESC` · `--no-preview` · `--fps` · `--detector` · `--wait-ms` · `--max-frames`

## 수동 / sim

```bash
cargo run -p measure-friction -- --vt-pairs 2.0:1.4,1.5:1.05
cargo run -p measure-friction -- --sim --dry-run
```

기본 `--config`는 `config/default.toml`.
