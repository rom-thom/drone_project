#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/dev/drone/PX4-Autopilot}"
TARGET="${1:-gz_x500}"

RESTART_DELAY=2

if [[ ! -d "$PX4_DIR" ]]; then
echo "PX4_DIR not found: $PX4_DIR"
exit 1
fi

echo "==> PX4 auto-restart launcher"
echo "==> Dir: $PX4_DIR"
echo "==> Target: $TARGET"

cd "$PX4_DIR"

while true; do
echo "======================================"
echo "Starting PX4 SITL..."
echo "======================================"

# Optional: clean Gazebo each run

if [[ "${CLEAN_GZ:-0}" == "1" ]]; then
echo "Cleaning Gazebo processes..."
pkill -f "gz sim" || true
pkill -f "gz gui" || true
pkill -f gazebo || true
sleep 1
fi

# Run PX4 (this blocks until exit/crash)

make px4_sitl "$TARGET" || true

echo "======================================"
echo "PX4 exited. Restarting in ${RESTART_DELAY}s..."
echo "Press Ctrl+C to stop."
echo "======================================"

sleep "$RESTART_DELAY"
done
