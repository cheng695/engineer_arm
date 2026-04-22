#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

echo "[watch-j6] Watching /rosout for J6 debug logs..."
echo "[watch-j6] Press Ctrl+C to stop."

ros2 topic echo /rosout --field msg | grep --line-buffered "\\[J6\\]"
