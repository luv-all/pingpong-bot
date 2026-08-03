#!/usr/bin/env bash
set -euo pipefail

# Homebrew의 최신 opencv(현재 5.x)가 아니라 Rust opencv 0.98.2와 호환되는
# 버전 고정 formula를 선택한다. 추가 인자는 그대로 pingpong-bot에 전달한다.
opencv4_prefix="$(brew --prefix opencv@4)"
exec env PKG_CONFIG_PATH="${opencv4_prefix}/lib/pkgconfig" \
  cargo run -p pingpong-bot -- --mode sim "$@"
