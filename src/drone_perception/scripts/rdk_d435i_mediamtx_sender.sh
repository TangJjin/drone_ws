#!/usr/bin/env bash
set -Eeuo pipefail

STREAM_CONFIG=${STREAM_CONFIG:-}
MEDIAMTX_BIN=${MEDIAMTX_BIN:-/home/sunrise/mediamtx/mediamtx}
MEDIAMTX_CONFIG=${MEDIAMTX_CONFIG:-}
ENCODER_BIN=${ENCODER_BIN:-}
package_prefix=""

if command -v ros2 >/dev/null; then
  package_prefix=$(ros2 pkg prefix drone_perception 2>/dev/null || true)
fi

if [[ -z "$STREAM_CONFIG" && -n "$package_prefix" ]]; then
  installed_stream_config="$package_prefix/share/drone_perception/config/rdk_d435i_stream.yaml"
  if [[ -f "$installed_stream_config" ]]; then
    STREAM_CONFIG="$installed_stream_config"
  fi
fi

if [[ -n "$STREAM_CONFIG" ]]; then
  [[ -f "$STREAM_CONFIG" ]] || { echo "stream config not found: $STREAM_CONFIG" >&2; exit 1; }
  command -v python3 >/dev/null || { echo "python3 not found" >&2; exit 127; }
  if ! config_output=$(python3 - "$STREAM_CONFIG" <<'PY'
import re
import sys

import yaml

config_path = sys.argv[1]
with open(config_path, "r", encoding="utf-8") as config_file:
    root = yaml.safe_load(config_file) or {}

camera = root.get("camera", {})
encoder = root.get("encoder", {})
rtsp = root.get("rtsp", {})

values = {
    "device": camera.get("device", "/dev/d435i_color"),
    "width": camera.get("width", 640),
    "height": camera.get("height", 480),
    "fps": camera.get("fps", 30),
    "bitrate_kbps": encoder.get("bitrate_kbps", 5000),
    "gop": encoder.get("gop", 15),
    "port": rtsp.get("port", 8554),
    "path": rtsp.get("path", "d435i"),
}
transports = rtsp.get("transports", ["udp", "tcp"])

for name in ("width", "height", "fps", "bitrate_kbps", "gop", "port"):
    value = values[name]
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
if values["port"] > 65535:
    raise ValueError("port must be at most 65535")
if not isinstance(values["device"], str) or not values["device"]:
    raise ValueError("device must be a non-empty string")
if not isinstance(values["path"], str) or not re.fullmatch(r"[A-Za-z0-9_.~-]+", values["path"]):
    raise ValueError("path may contain only letters, digits, dot, underscore, tilde, and hyphen")
if not isinstance(transports, list) or not transports:
    raise ValueError("transports must be a non-empty list")
if any(item not in ("udp", "tcp") for item in transports):
    raise ValueError("transports may contain only udp and tcp")

for name in ("device", "width", "height", "fps", "bitrate_kbps", "gop", "port", "path"):
    print(values[name])
print(",".join(transports))
PY
  ); then
    echo "invalid stream config: $STREAM_CONFIG" >&2
    exit 1
  fi
  mapfile -t config_values <<< "$config_output"
  [[ ${#config_values[@]} -eq 9 ]] || { echo "incomplete stream config: $STREAM_CONFIG" >&2; exit 1; }
else
  config_values=(/dev/d435i_color 640 480 30 5000 15 8554 d435i udp,tcp)
fi

DEVICE=${DEVICE:-${config_values[0]}}
WIDTH=${WIDTH:-${config_values[1]}}
HEIGHT=${HEIGHT:-${config_values[2]}}
FPS=${FPS:-${config_values[3]}}
BITRATE_KBPS=${BITRATE_KBPS:-${config_values[4]}}
GOP=${GOP:-${config_values[5]}}
RTSP_PORT=${RTSP_PORT:-${config_values[6]}}
RTSP_PATH=${RTSP_PATH:-${config_values[7]}}
RTSP_TRANSPORTS=${RTSP_TRANSPORTS:-${config_values[8]}}
RTSP_PUBLISH_URL=${RTSP_PUBLISH_URL:-rtsp://127.0.0.1:${RTSP_PORT}/${RTSP_PATH}}

command -v ffmpeg >/dev/null || { echo "ffmpeg not found" >&2; exit 127; }
[[ -e "$DEVICE" ]] || { echo "camera device not found: $DEVICE" >&2; exit 1; }
[[ -x "$MEDIAMTX_BIN" ]] || { echo "MediaMTX not executable: $MEDIAMTX_BIN" >&2; exit 1; }

if [[ -z "$ENCODER_BIN" ]]; then
  if command -v rdk_d435i_h264_sender >/dev/null; then
    ENCODER_BIN=$(command -v rdk_d435i_h264_sender)
  elif [[ -n "$package_prefix" ]]; then
    ENCODER_BIN="$package_prefix/lib/drone_perception/rdk_d435i_h264_sender"
  fi
fi
[[ -n "$ENCODER_BIN" && -x "$ENCODER_BIN" ]] || {
  echo "rdk_d435i_h264_sender not found; set ENCODER_BIN" >&2
  exit 1
}

mediamtx_pid=""
runtime_mediamtx_config=""
cleanup()
{
  trap - EXIT INT TERM
  if [[ -n "$mediamtx_pid" ]]; then
    kill "$mediamtx_pid" 2>/dev/null || true
    wait "$mediamtx_pid" 2>/dev/null || true
  fi
  if [[ -n "$runtime_mediamtx_config" ]]; then
    rm -f "$runtime_mediamtx_config"
  fi
}
trap cleanup EXIT INT TERM

if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${RTSP_PORT}$"; then
  echo "TCP port ${RTSP_PORT} is already in use; stop the existing RTSP server first" >&2
  exit 1
fi

if [[ -z "$MEDIAMTX_CONFIG" ]]; then
  runtime_mediamtx_config=$(mktemp /tmp/d435i-mediamtx.XXXXXX.yml)
  transports_yaml=${RTSP_TRANSPORTS//,/, }
  printf '%s\n' \
    'logLevel: info' \
    "rtspAddress: :${RTSP_PORT}" \
    "rtspTransports: [${transports_yaml}]" \
    '' \
    'paths:' \
    "  ${RTSP_PATH}:" \
    '    source: publisher' > "$runtime_mediamtx_config"
  MEDIAMTX_CONFIG="$runtime_mediamtx_config"
else
  [[ -f "$MEDIAMTX_CONFIG" ]] || { echo "MediaMTX config not found: $MEDIAMTX_CONFIG" >&2; exit 1; }
fi
"$MEDIAMTX_BIN" "$MEDIAMTX_CONFIG" &
mediamtx_pid=$!

for _ in $(seq 1 50); do
  if ! kill -0 "$mediamtx_pid" 2>/dev/null; then
    wait "$mediamtx_pid"
    exit 1
  fi
  if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${RTSP_PORT}$"; then
    break
  fi
  sleep 0.1
done
if ! ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${RTSP_PORT}$"; then
  echo "MediaMTX did not open TCP port ${RTSP_PORT}" >&2
  exit 1
fi

echo "Publishing ${WIDTH}x${HEIGHT}@${FPS} H.264 at ${BITRATE_KBPS} kbit/s" >&2
echo "Stream config: ${STREAM_CONFIG:-built-in defaults}" >&2
echo "Local publish URL: $RTSP_PUBLISH_URL" >&2

"$ENCODER_BIN" \
  --device "$DEVICE" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --fps "$FPS" \
  --bitrate-kbps "$BITRATE_KBPS" \
  --gop "$GOP" | \
ffmpeg \
  -hide_banner \
  -loglevel warning \
  -fflags +genpts \
  -r "$FPS" \
  -f h264 \
  -i pipe:0 \
  -map 0:v:0 \
  -c:v copy \
  -an \
  -f rtsp \
  -rtsp_transport tcp \
  "$RTSP_PUBLISH_URL"
