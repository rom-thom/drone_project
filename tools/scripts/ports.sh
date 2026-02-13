#!/usr/bin/env bash
set -euo pipefail

# Common ports you’ll see in PX4 SITL setups
PORTS=(
  14540
  14550
  14560
  14570
  18570
  14580
  14280
  13030
)

echo "==> UDP listeners on common PX4/QGC ports"
echo "    (If a port is already in use, it will show up here)"
echo

for p in "${PORTS[@]}"; do
  echo "--- Port $p ---"
  # -u UDP, -l listening, -p process, -n numeric
  ss -u -lpn | grep -E "[:.]$p\b" || echo "(no listener)"
  echo
done

echo "==> All UDP listeners (filtered to ss -u -lpn)"
echo "Tip: run 'ss -u -lpn | less' to browse everything."
