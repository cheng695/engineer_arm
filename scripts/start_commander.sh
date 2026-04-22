#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/Desktop/engineer_whc}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
BUILD_WS=0

print_help() {
  cat <<'EOF'
Usage:
  start_commander.sh [options]

Options:
  --ws-dir <path>          Workspace path (default: ~/Desktop/engineer_whc)
  --ros-distro <name>      ROS distro (default: humble)
  --use-sim-time <bool>    true | false (default: false)
  --build                  Run colcon build before launch
  -h, --help               Show this help

Examples:
  bash scripts/start_commander.sh
  bash scripts/start_commander.sh --build
EOF
}

log() {
  echo "[commander] $*"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws-dir)
      WS_DIR="$2"
      shift 2
      ;;
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --use-sim-time)
      USE_SIM_TIME="$2"
      shift 2
      ;;
    --build)
      BUILD_WS=1
      shift
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      print_help
      exit 1
      ;;
  esac
done

WS_DIR="${WS_DIR/#\~/$HOME}"

case "$USE_SIM_TIME" in
  true|false)
    ;;
  *)
    echo "Invalid use_sim_time: $USE_SIM_TIME" >&2
    echo "Valid values: true, false" >&2
    exit 1
    ;;
esac

require_cmd bash
require_cmd colcon

if [[ ! -d "$WS_DIR/src" ]]; then
  echo "Workspace src not found: $WS_DIR/src" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

cd "$WS_DIR"

# ROS setup scripts may reference optional environment variables that are unset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if (( BUILD_WS == 1 )); then
  log "Building workspace..."
  colcon build --symlink-install --packages-select my_robot_commander_cpp
fi

set +u
source "$WS_DIR/install/setup.bash"
set -u

log "Launching commander..."
exec ros2 launch my_robot_commander_cpp commander.launch.py use_sim_time:="${USE_SIM_TIME}"
