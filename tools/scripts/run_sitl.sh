#!/usr/bin/env bash
set -euo pipefail

# --- Config: set this to your PX4-Autopilot path ---
PX4_DIR="${PX4_DIR:-$HOME/dev/drone/PX4-Autopilot}"

# Default target/model
TARGET="${1:-gz_x500}"

if [[ ! -d "$PX4_DIR" ]]; then
  echo "PX4_DIR not found: $PX4_DIR"
  echo "Set PX4_DIR env var, e.g.: PX4_DIR=~/dev/drone/PX4-Autopilot $0"
  exit 1
fi

echo "==> Starting PX4 SITL in: $PX4_DIR"
echo "==> Target: $TARGET"
echo "Tip: export PX4_DIR=/path/to/PX4-Autopilot to avoid editing this file."

cd "$PX4_DIR"

# Optional: build first (uncomment if you want)
# make clean

make px4_sitl "$TARGET"
