# label-stills

클립 타임라인을 등분해 뽑은 스틸에 **공 중심 한 번 클릭**(또는 무공)으로 GT를 만든다.
`eval-colormask`가 이 GT로 색·검출 조합을 hit/miss/FP/TN 채점한다.

**비디오 전 프레임 GT는 비범위.** 캠·클립당 ~10장, 그중 **2~3장은 무공**으로 남긴다
(무공이 있어야 FP/TN을 잰다).

## 산출물

```
data/detect_stills/
  manifest.json
  fly_01_left_t0000.png
  fly_01_left_t0047.png
  ...
```

```json
{
  "hit_radius_px": 20.0,
  "items": [
    { "path": "fly_01_left_t0047.png", "camera_id": 0, "clip": "fly_01", "frame": 47, "pixel": [812.0, 340.5] },
    { "path": "fly_01_left_t0400.png", "camera_id": 0, "clip": "fly_01", "frame": 400, "pixel": null }
  ]
}
```

`pixel: null` = 무공 — 검출되면 **FP**, 없으면 **TN**.

스키마 SSOT: `detector::StillsManifest` / `detector::StillItem`.
라벨은 한 장 찍을 때마다 즉시 저장되므로 중간에 `q`로 나가도 남는다. 같은 `path`는 덮어쓴다.

## 사용

```bash
cargo run -p label-stills -- --cam left  --clip fly_01 --count 10
cargo run -p label-stills -- --cam right --clip fly_01 --count 10
cargo run -p label-stills -- --cam left  --clip drop_02 --count 10
```

## 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--cam left\|right` | **필수** | 어느 쪽 카메라인지 |
| `--clip NAME\|DIR` | **필수** | `data/clips/<name>`의 해당 캠 영상 |
| `--count N` | 10 | 타임라인 등분해 뽑을 장 수 |
| `--hit-radius PX` | 20 | eval hit 판정 반경 (manifest에 저장) |

## 키

| 키 | 동작 |
|----|------|
| LMB / Enter | 공 중심 확정 → 저장 후 다음 장 |
| 화살표 | 조준점 1px 이동 |
| Shift + 이동 | 확대(loupe) |
| `n` | **무공** (`pixel: null`) → 저장 후 다음 장 |
| `z` | 이전 장으로 (다시 라벨) |
| `q` / ESC | 저장 후 종료 |
