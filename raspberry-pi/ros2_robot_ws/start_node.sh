#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-all}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-ros2}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
SELECTED_VIDEO_DEVICE=""

PIDS=()

find_conda_sh() {
  local candidates=(
    "$HOME/miniforge3/etc/profile.d/conda.sh"
    "$HOME/miniconda3/etc/profile.d/conda.sh"
    "$HOME/anaconda3/etc/profile.d/conda.sh"
  )

  for path in "${candidates[@]}"; do
    if [[ -f "$path" ]]; then
      echo "$path"
      return 0
    fi
  done

  return 1
}

start_node() {
  local name="$1"
  shift
  local log_file="$SCRIPT_DIR/logs/${name}.log"

  echo "Starting ${name}..."
  "$@" >"$log_file" 2>&1 &
  PIDS+=("$!")
  echo "  pid=${PIDS[-1]} log=${log_file}"
}

cleanup() {
  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo ""
    echo "Stopping nodes..."
    for pid in "${PIDS[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    wait || true
  fi
}

trap cleanup INT TERM EXIT

mkdir -p "$SCRIPT_DIR/logs"

resolve_video_device() {
  if [[ -e "$VIDEO_DEVICE" ]]; then
    SELECTED_VIDEO_DEVICE="$VIDEO_DEVICE"
    return 0
  fi

  shopt -s nullglob
  local devices=(/dev/video*)
  shopt -u nullglob

  if [[ ${#devices[@]} -gt 0 ]]; then
    SELECTED_VIDEO_DEVICE="${devices[0]}"
    echo "WARN: Requested device $VIDEO_DEVICE not found. Using ${SELECTED_VIDEO_DEVICE} instead."
    return 0
  fi

  echo "ERROR: No V4L2 camera device found (no /dev/video*)."
  echo "Fix options:"
  echo "  1) Connect/enable a USB or CSI camera"
  echo "  2) For Raspberry Pi CSI camera, enable V4L2 bridge: sudo modprobe bcm2835-v4l2"
  echo "  3) Re-run with VIDEO_DEVICE=/dev/videoX if your device path is different"
  return 1
}

CONDA_SH_PATH=""
if ! CONDA_SH_PATH="$(find_conda_sh)"; then
  echo "ERROR: conda.sh not found (checked miniforge3/miniconda3/anaconda3)."
  exit 1
fi

# Some conda activation hooks reference optional variables (e.g. CONDA_BUILD)
# and can fail when nounset is enabled.
set +u
# shellcheck disable=SC1090
source "$CONDA_SH_PATH"
conda activate "$CONDA_ENV_NAME"

# Colcon-generated setup scripts may reference optional variables that are unset.
# Source them with nounset disabled, then restore strict mode.
source "$SCRIPT_DIR/install/setup.sh"

# Ensure dynamic linker can resolve ROS2 libs from conda (e.g. librclcpp.so).
if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
set -u

echo "Workspace: $SCRIPT_DIR"
echo "Conda env: $CONDA_ENV_NAME"
echo "Mode: $MODE"

if ! resolve_video_device; then
  exit 1
fi
echo "Video device: $SELECTED_VIDEO_DEVICE"

action_camera_only() {
  ros2 run camera_sensor camera_node
}

action_all_nodes() {
  start_node motor_driver ros2 run motor_control motor_driver
  start_node imu_node ros2 run imu_sensor imu_node
  start_node encoder_node ros2 run encoder_odometry encoder_node
  start_node robot_controller ros2 run robot_controller robot_controller
  start_node camera_node ros2 run camera_sensor camera_node

  echo ""
  echo "All nodes started. Press Ctrl+C to stop all."
  echo "Camera topic: /camera/raw"
  echo ""
  wait
}

case "$MODE" in
  camera)
    echo "Starting camera node only..."
    action_camera_only
    ;;
  all)
    action_all_nodes
    ;;
  *)
    echo "Usage: ./start_node.sh [camera|all]"
    echo "Examples:"
    echo "  ./start_node.sh camera"
    echo "  VIDEO_DEVICE=/dev/video1 ./start_node.sh camera"
    echo "  ./start_node.sh all"
    exit 1
    ;;
esac
