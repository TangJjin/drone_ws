#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PORT=${PORT:-5004}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
BACKEND=${BACKEND:-cpp}

exec "$SCRIPT_DIR/rdk_udp_vpu_display.sh" \
  --port "$PORT" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --backend "$BACKEND" \
  "$@"
