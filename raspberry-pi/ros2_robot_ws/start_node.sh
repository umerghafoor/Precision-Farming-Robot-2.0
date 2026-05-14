#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-all}"
VIDEO_DEVICE_ARG="${2:-}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-ros2}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
MQTT_HOST="${MQTT_HOST:-sanilinux.mullet-bull.ts.net}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_KEEPALIVE="${MQTT_KEEPALIVE:-60}"
ODOM_RATE_HZ="${ODOM_RATE_HZ:-1.0}"
SELECTED_VIDEO_DEVICE=""
WEBCAM_DEVICE_INDEX="${WEBCAM_DEVICE_INDEX:-}"
SELECTED_WEBCAM_DEVICE_INDEX=""

if [[ -n "$VIDEO_DEVICE_ARG" ]]; then
  VIDEO_DEVICE="$VIDEO_DEVICE_ARG"
fi

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

device_path_to_index() {
  local device_path="$1"
  if [[ "$device_path" =~ ^/dev/video([0-9]+)$ ]]; then
    echo "${BASH_REMATCH[1]}"
    return 0
  fi

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
echo "MQTT broker: $MQTT_HOST:$MQTT_PORT"

if [[ "$MODE" == "webcam" ]]; then
  if ! resolve_video_device; then
    exit 1
  fi
  echo "Video device: $SELECTED_VIDEO_DEVICE"

  if [[ -z "$WEBCAM_DEVICE_INDEX" ]]; then
    if ! SELECTED_WEBCAM_DEVICE_INDEX="$(device_path_to_index "$SELECTED_VIDEO_DEVICE")"; then
      echo "ERROR: Could not extract webcam index from $SELECTED_VIDEO_DEVICE"
      echo "Set WEBCAM_DEVICE_INDEX explicitly (example: WEBCAM_DEVICE_INDEX=1)"
      exit 1
    fi
  else
    SELECTED_WEBCAM_DEVICE_INDEX="$WEBCAM_DEVICE_INDEX"
  fi

  echo "Webcam index: $SELECTED_WEBCAM_DEVICE_INDEX"
fi

action_camera_only() {
  ros2 run camera_sensor camera_node
}

action_webcam_only() {
  ros2 run camera_sensor webcam_node --ros-args \
    -p device_index:="$SELECTED_WEBCAM_DEVICE_INDEX" \
    -p camera_topic:=/camera/raw
}

action_all_nodes() {
  if ! resolve_video_device; then
    exit 1
  fi
  local device_index
  if ! device_index="$(device_path_to_index "$SELECTED_VIDEO_DEVICE")"; then
    echo "ERROR: Could not extract device index from $SELECTED_VIDEO_DEVICE"
    echo "Set WEBCAM_DEVICE_INDEX explicitly (example: WEBCAM_DEVICE_INDEX=1)"
    exit 1
  fi

  # spi_controller_bridge requires /dev/spidev0.0 (enable SPI via raspi-config if needed)
  if [[ -e /dev/spidev0.0 ]]; then
    start_node spi_controller_bridge ros2 run motor_control spi_controller_bridge --ros-args \
      -p cmd_vel_topic:=/cmd_vel \
      -p servo1_topic:=/servo1/angle \
      -p servo2_topic:=/servo2/angle \
      -p spi_device:=/dev/spidev0.0 \
      -p spi_mode:=0 \
      -p spi_bits_per_word:=8 \
      -p spi_speed_hz:=500000 \
      -p wheel_base:=0.2 \
      -p max_linear_velocity:=1.0 \
      -p cmd_timeout_sec:=0.5 \
      -p tx_rate_hz:=20.0 \
      -p default_servo_angle:=90
  else
    echo "  SKIP spi_controller_bridge: /dev/spidev0.0 not found (SPI disabled)"
  fi
  start_node imu_node ros2 run imu_sensor imu_node --ros-args \
    -p update_rate:=50.0 \
    -p i2c_bus:=1 \
    -p i2c_address:=0x68
  start_node webcam_node ros2 run camera_sensor webcam_node --ros-args \
    -p device_index:="$device_index" \
    -p image_width:=640 \
    -p image_height:=480 \
    -p publish_rate:=10.0 \
    -p frame_id:=camera_link \
    -p camera_topic:=/camera/raw \
    -p color_jpeg_topic:=/camera/color_jpeg \
    -p jpeg_quality:=80
  start_node yolo_detection_node ros2 run yolo_detection yolo_detection_node --ros-args \
    -p confidence_threshold:=0.25 \
    -p iou_threshold:=0.45 \
    -p input_size:=640 \
    -p camera_topic:=/camera/color_jpeg \
    -p annotated_topic:=/camera/detection \
    -p results_topic:=/detections/results \
    -p enable_visualization:=true
  start_node encoder_node ros2 run encoder_odometry encoder_node --ros-args \
    -p wheel_radius:=0.05 \
    -p wheel_base:=0.2 \
    -p counts_per_rev:=20 \
    -p update_rate:=20.0
  start_node robot_controller ros2 run robot_controller robot_controller --ros-args \
    -p control_loop_rate:=20.0 \
    -p emergency_stop_enabled:=true \
    -p max_linear_velocity:=1.0 \
    -p max_angular_velocity:=2.0 \
    -p min_battery_voltage:=7.0
  start_node mqtt_bridge ros2 run mqtt_bridge mqtt_bridge_node --ros-args \
    -p mqtt_host:="$MQTT_HOST" \
    -p mqtt_port:="$MQTT_PORT" \
    -p mqtt_keepalive:="$MQTT_KEEPALIVE" \
    -p odom_rate_hz:="$ODOM_RATE_HZ" \
    -p image_rate_hz:=0.5 \
    -p imu_rate_hz:=1.0 \
    -p image_format:=jpeg \
    -p image_quality:=80 \
    -p camera_detection_transport:=compressed

  echo ""
  echo "All nodes started. Press Ctrl+C to stop all."
  echo "Camera topics: /camera/raw (gray compressed) | /camera/color_jpeg (JPEG)"
  echo ""
  wait
}

action_yolo_only() {
  ros2 run yolo_detection yolo_detection_node
}

action_mqtt_only() {
  ros2 run mqtt_bridge mqtt_bridge_node --ros-args \
    -p mqtt_host:="$MQTT_HOST" \
    -p mqtt_port:="$MQTT_PORT" \
    -p mqtt_keepalive:="$MQTT_KEEPALIVE" \
    -p odom_rate_hz:="$ODOM_RATE_HZ" \
    -p camera_detection_transport:=compressed \
    -p image_format:=jpeg \
    -p image_quality:=80
}

case "$MODE" in
  camera)
    echo "Starting camera node only..."
    action_camera_only
    ;;
  webcam)
    echo "Starting webcam node only..."
    action_webcam_only
    ;;
  yolo)
    echo "Starting YOLO node only..."
    action_yolo_only
    ;;
  mqtt)
    echo "Starting MQTT bridge node only..."
    action_mqtt_only
    ;;
  all)
    action_all_nodes
    ;;
  *)
    echo "Usage: ./start_node.sh [camera|webcam|yolo|mqtt|all] [video_device]"
    echo "Examples:"
    echo "  ./start_node.sh camera"
    echo "  ./start_node.sh webcam"
    echo "  ./start_node.sh yolo"
    echo "  ./start_node.sh mqtt"
    echo "  ./start_node.sh webcam /dev/video0"
    echo "  VIDEO_DEVICE=/dev/video0 ./start_node.sh camera"
    echo "  VIDEO_DEVICE=/dev/video0 ./start_node.sh webcam"
    echo "  WEBCAM_DEVICE_INDEX=1 ./start_node.sh webcam"
    echo "  MQTT_HOST=192.168.1.10 MQTT_PORT=1883 ./start_node.sh mqtt"
    echo "  ./start_node.sh all"
    exit 1
    ;;
esac
