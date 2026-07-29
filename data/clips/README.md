# data/clips

연구실에서 `record-stereo`로 찍은 **오프라인 재생 클립**.

`data/calibration.json` · `data/colormask.json`과 같은 비전 트리.

```
{scene}_{nn}/
  left.avi
  right.avi
  meta.json
```

장면: `fly` | `roll` | `drop`

```bash
cargo run -p verify-stereo -- --clip fly_01
cargo run -p measure-restitution -- --clip drop_02
cargo run -p measure-friction -- --clip roll_01
cargo run -p detect-full -- --cam left --clip fly_01
cargo run -p tune-colormask -- --cam left --clip fly_01
```

녹화: [tools/record_stereo/README.md](../../tools/record_stereo/README.md)
