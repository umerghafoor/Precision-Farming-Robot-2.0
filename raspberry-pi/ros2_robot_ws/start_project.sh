#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DROIDCAM_PID=""

cleanup() {
  echo ""
  echo "Shutting down..."
  if [[ -n "$DROIDCAM_PID" ]]; then
    kill "$DROIDCAM_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

# --- ADB camera setup ---
echo "Checking ADB device..."
adb devices

echo "Forwarding ADB port 4747..."
adb forward tcp:4747 tcp:4747

echo "Starting droidcam-cli..."
droidcam-cli 127.0.0.1 4747 &
DROIDCAM_PID="$!"
echo "  droidcam-cli pid=$DROIDCAM_PID"

# Give droidcam a moment to open the video device before nodes start
sleep 2

# --- ROS2 nodes ---
exec "$SCRIPT_DIR/start_node.sh" all
