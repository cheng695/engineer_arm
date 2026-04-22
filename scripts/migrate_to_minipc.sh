#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${WS_DIR:-$HOME/engineer_whc}"
SKIP_APT=0
SKIP_ROSDEP=0
SKIP_BASHRC=0
CLEAN_BUILD=0

print_help() {
  cat <<'EOF'
Usage:
  migrate_to_minipc.sh [options]

Options:
  --ros-distro <name>   ROS distro (default: humble)
  --ws-dir <path>       Workspace path (default: ~/engineer_whc)
  --skip-apt            Skip apt dependency install
  --skip-rosdep         Skip rosdep install
  --skip-bashrc         Do not append source lines to ~/.bashrc
  --clean-build         Remove build/install/log before build
  -h, --help            Show this help

Example:
  bash scripts/migrate_to_minipc.sh --ros-distro humble --ws-dir ~/engineer_whc --clean-build
EOF
}

log() {
  echo "[migrate] $*"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --ws-dir)
      WS_DIR="$2"
      shift 2
      ;;
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --skip-rosdep)
      SKIP_ROSDEP=1
      shift
      ;;
    --skip-bashrc)
      SKIP_BASHRC=1
      shift
      ;;
    --clean-build)
      CLEAN_BUILD=1
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

require_cmd bash
require_cmd colcon

if [[ ! -d "$WS_DIR/src" ]]; then
  echo "Workspace src not found: $WS_DIR/src" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  echo "Please install ROS ${ROS_DISTRO} first." >&2
  exit 1
fi

if (( SKIP_APT == 0 )); then
  require_cmd sudo
  require_cmd apt-get
  log "Installing base tools and rosdep via apt..."
  sudo apt-get update
  sudo apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential
fi

if (( SKIP_ROSDEP == 0 )); then
  require_cmd rosdep
  log "Initializing rosdep (safe if already initialized)..."
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
  fi
  rosdep update
fi

log "Entering workspace: $WS_DIR"
cd "$WS_DIR"

if (( CLEAN_BUILD == 1 )); then
  log "Cleaning build/install/log..."
  rm -rf build install log
fi

if (( SKIP_ROSDEP == 0 )); then
  log "Installing workspace dependencies..."
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  rosdep install --from-paths src --ignore-src -r -y
fi

log "Building workspace with colcon..."
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install

if (( SKIP_BASHRC == 0 )); then
  ROS_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
  WS_LINE="source ${WS_DIR}/install/setup.bash"

  if ! grep -Fqx "$ROS_LINE" "$HOME/.bashrc"; then
    echo "$ROS_LINE" >> "$HOME/.bashrc"
    log "Added ROS setup line to ~/.bashrc"
  fi

  if ! grep -Fqx "$WS_LINE" "$HOME/.bashrc"; then
    echo "$WS_LINE" >> "$HOME/.bashrc"
    log "Added workspace setup line to ~/.bashrc"
  fi
fi

log "Done. Run: source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS_DIR}/install/setup.bash"
