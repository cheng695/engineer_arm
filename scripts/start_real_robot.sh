#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/Desktop/engineer_whc}"
CAN0_NAME="${CAN0_NAME:-can0}"
CAN1_NAME="${CAN1_NAME:-can1}"
CAN_DRIVER="${CAN_DRIVER:-}"
BITRATE="${BITRATE:-1000000}"
GRAVITY_MODE="${GRAVITY_MODE:-off}"
ACTIVE_REAL_JOINTS="${ACTIVE_REAL_JOINTS:-}"
BUILD_WS=0
SKIP_CAN=0

print_help() {
  cat <<'EOF'
Usage:
  start_real_robot.sh [options]

Options:
  --ws-dir <path>          Workspace path (default: ~/Desktop/engineer_whc)
  --ros-distro <name>      ROS distro (default: humble)
  --bitrate <value>        CAN bitrate for can0/can1 (default: 1000000)
  --gravity-mode <mode>    off | assist | gravity_only (default: off)
  --can-driver <module>    Optional CAN kernel module to load (default: empty)
  --can0 <name>            CAN interface name for joints 1-4 (default: can0)
  --can1 <name>            CAN interface name for joints 5-7 (default: can1)
  --active-real-joints <csv>
                           Comma-separated real joints to keep on CAN.
                           Other joints use local fake feedback for full-stack debugging.
  --build                  Run colcon build before launch
  --skip-can               Do not touch CAN interfaces, only launch ROS
  -h, --help               Show this help

Examples:
  bash scripts/start_real_robot.sh --bitrate 1000000
  bash scripts/start_real_robot.sh --build --gravity-mode assist
  bash scripts/start_real_robot.sh --bitrate 1000000 --can-driver gs_usb
  bash scripts/start_real_robot.sh --active-real-joints joint6
EOF
}

log() {
  echo "[real-robot] $*"
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
    --bitrate)
      BITRATE="$2"
      shift 2
      ;;
    --gravity-mode)
      GRAVITY_MODE="$2"
      shift 2
      ;;
    --can-driver)
      CAN_DRIVER="$2"
      shift 2
      ;;
    --can0)
      CAN0_NAME="$2"
      shift 2
      ;;
    --can1)
      CAN1_NAME="$2"
      shift 2
      ;;
    --active-real-joints)
      ACTIVE_REAL_JOINTS="$2"
      shift 2
      ;;
    --build)
      BUILD_WS=1
      shift
      ;;
    --skip-can)
      SKIP_CAN=1
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

case "$GRAVITY_MODE" in
  off|assist|gravity_only)
    ;;
  *)
    echo "Invalid gravity mode: $GRAVITY_MODE" >&2
    echo "Valid values: off, assist, gravity_only" >&2
    exit 1
    ;;
esac

require_cmd bash
require_cmd sudo
require_cmd ip
require_cmd modprobe
require_cmd colcon

if [[ ! -d "$WS_DIR/src" ]]; then
  echo "Workspace src not found: $WS_DIR/src" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if (( SKIP_CAN == 0 )); then
  log "Loading CAN kernel modules..."
  sudo modprobe can
  sudo modprobe can_raw
  if [[ -n "$CAN_DRIVER" ]]; then
    if ! sudo modprobe "$CAN_DRIVER"; then
      log "Warning: failed to load CAN driver module '$CAN_DRIVER'. Continuing anyway."
      log "If your interface is already provided by another driver, this is OK."
    fi
  else
    log "No extra CAN driver module requested. Assuming the interface driver is already available."
  fi

  log "Configuring ${CAN0_NAME} and ${CAN1_NAME} with bitrate ${BITRATE}..."
  sudo ip link set "$CAN0_NAME" down 2>/dev/null || true
  sudo ip link set "$CAN1_NAME" down 2>/dev/null || true
  sudo ip link set "$CAN0_NAME" type can bitrate "$BITRATE"
  sudo ip link set "$CAN1_NAME" type can bitrate "$BITRATE"
  sudo ip link set "$CAN0_NAME" up
  sudo ip link set "$CAN1_NAME" up

  log "CAN status:"
  ip -details link show "$CAN0_NAME"
  ip -details link show "$CAN1_NAME"
fi

cd "$WS_DIR"

# ROS setup scripts may reference optional environment variables that are unset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if (( BUILD_WS == 1 )); then
  log "Building workspace..."
  colcon build --symlink-install
fi

set +u
source "$WS_DIR/install/setup.bash"
set -u

log "Launching real robot bringup..."
log "use_mock_hardware:=false gravity_compensation_mode:=${GRAVITY_MODE} active_real_joints:=${ACTIVE_REAL_JOINTS:-<all>}"
exec ros2 launch my_robot_bringup robot.launch.py \
  use_mock_hardware:=false \
  gravity_compensation_mode:="${GRAVITY_MODE}" \
  active_real_joints:="${ACTIVE_REAL_JOINTS}"
