#!/usr/bin/env bash
set -Eeuo pipefail

PORT=${PORT:-5004}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-360}
DURATION=${DURATION:-0}
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CV_DISPLAY=${CV_DISPLAY:-"$SCRIPT_DIR/rdk_mjpeg_rtp_cv_display.py"}

usage() {
  echo "Usage: $0 [--port N] [--width N] [--height N] [--duration SEC]"
}

while (($#)); do
  case "$1" in
    --port) PORT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

[[ -f "$CV_DISPLAY" ]] || { echo "cv display not found: $CV_DISPLAY" >&2; exit 1; }

if ((DURATION > 0)); then
  exec timeout --signal=TERM --kill-after=2 "$DURATION" \
    python3 "$CV_DISPLAY" --port "$PORT" --width "$WIDTH" --height "$HEIGHT"
fi

exec python3 "$CV_DISPLAY" --port "$PORT" --width "$WIDTH" --height "$HEIGHT"
