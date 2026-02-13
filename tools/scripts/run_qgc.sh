#!/usr/bin/env bash
set -euo pipefail

# --- Config: where your QGC AppImage is stored ---
QGC_APPIMAGE="${QGC_APPIMAGE:-$HOME/Apps/QGC/QGroundControl-x86_64.AppImage}"

if [[ ! -f "$QGC_APPIMAGE" ]]; then
  echo "QGC AppImage not found: $QGC_APPIMAGE"
  echo "Set QGC_APPIMAGE env var, e.g.:"
  echo "  QGC_APPIMAGE=~/Downloads/QGroundControl-x86_64.AppImage $0"
  exit 1
fi

chmod +x "$QGC_APPIMAGE" 2>/dev/null || true

echo "==> Starting QGroundControl: $QGC_APPIMAGE"
# Run it in background, keep logs in terminal.
"$QGC_APPIMAGE" &
disown

echo "==> QGC started."
