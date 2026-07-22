#!/usr/bin/env bash
set -Eeuo pipefail

[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
[[ -f /opt/tros/humble/setup.bash ]] && source /opt/tros/humble/setup.bash

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BRIDGE=${BRIDGE:-"$SCRIPT_DIR/rdk_udp_vpu_bridge.sh"}
DISPLAY_NODE=${DISPLAY_NODE:-"$SCRIPT_DIR/rdk_nv12_display.py"}
PORT=${PORT:-5004}
WIDTH=${WIDTH:-1280}
HEIGHT=${HEIGHT:-720}
DURATION=${DURATION:-0}
TOPIC=${TOPIC:-/rdk_video/image}

usage() {
  echo "Usage: $0 [--port N] [--width N] [--height N] [--duration SEC] [--topic TOPIC]"
}

while (($#)); do
  case "$1" in
    --port) PORT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    --topic) TOPIC=$2; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

[[ -x "$BRIDGE" ]] || { echo "bridge not executable: $BRIDGE" >&2; exit 1; }
[[ -f "$DISPLAY_NODE" ]] || { echo "display node not found: $DISPLAY_NODE" >&2; exit 1; }

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

python3 "$DISPLAY_NODE" \
  --input "$OUTPUT_FIFO" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --topic "$TOPIC" &
DISPLAY_PID=$!

"$BRIDGE" \
  --port "$PORT" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --duration "$DURATION" \
  --output "$OUTPUT_FIFO"
