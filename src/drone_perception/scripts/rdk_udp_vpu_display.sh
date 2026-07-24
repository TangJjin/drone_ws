#!/usr/bin/env bash
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
[[ -f /opt/tros/humble/setup.bash ]] && source /opt/tros/humble/setup.bash

set -Eeuo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BRIDGE=${BRIDGE:-"$SCRIPT_DIR/rdk_udp_vpu_bridge.sh"}
DISPLAY_NODE=${DISPLAY_NODE:-"$SCRIPT_DIR/rdk_nv12_display.py"}
DISPLAY_CPP=${DISPLAY_CPP:-"$SCRIPT_DIR/rdk_nv12_display_cpp"}
BACKEND=${BACKEND:-python}
PORT=${PORT:-5004}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-360}
FRAME_HEIGHT=${FRAME_HEIGHT:-}
DURATION=${DURATION:-0}
TOPIC=${TOPIC:-/rdk_video/image}

usage() {
  echo "Usage: $0 [--port N] [--width N] [--height N] [--duration SEC] [--topic TOPIC] [--backend python|cpp]"
}

while (($#)); do
  case "$1" in
    --port) PORT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    --topic) TOPIC=$2; shift 2;;
    --backend) BACKEND=$2; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

if [[ -z "$FRAME_HEIGHT" ]]; then
  FRAME_HEIGHT=$(( (HEIGHT + 15) / 16 * 16 ))
fi

[[ -x "$BRIDGE" ]] || { echo "bridge not executable: $BRIDGE" >&2; exit 1; }
case "$BACKEND" in
  python) [[ -f "$DISPLAY_NODE" ]] || { echo "display node not found: $DISPLAY_NODE" >&2; exit 1; };;
  cpp) [[ -x "$DISPLAY_CPP" ]] || { echo "display node not executable: $DISPLAY_CPP" >&2; exit 1; };;
  *) echo "Unknown backend: $BACKEND" >&2; usage >&2; exit 2;;
esac

WORK_DIR=$(mktemp -d /tmp/rdk_udp_vpu_display.XXXXXX)
OUTPUT_FIFO="$WORK_DIR/output.nv12"
mkfifo "$OUTPUT_FIFO"
DISPLAY_PID=""

cleanup() {
  trap - EXIT INT TERM
  [[ -n "$DISPLAY_PID" ]] && kill "$DISPLAY_PID" 2>/dev/null || true
  wait 2>/dev/null || true
  rm -rf "$WORK_DIR"
}
trap cleanup EXIT INT TERM

if [[ "$BACKEND" == "cpp" ]]; then
  "$DISPLAY_CPP" \
    --input "$OUTPUT_FIFO" \
    --width "$WIDTH" \
    --height "$FRAME_HEIGHT" \
    --visible-height "$HEIGHT" &
else
  python3 "$DISPLAY_NODE" \
    --input "$OUTPUT_FIFO" \
    --width "$WIDTH" \
    --height "$FRAME_HEIGHT" \
    --visible-height "$HEIGHT" \
    --topic "$TOPIC" &
fi
DISPLAY_PID=$!

"$BRIDGE" \
  --port "$PORT" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --duration "$DURATION" \
  --output "$OUTPUT_FIFO"
