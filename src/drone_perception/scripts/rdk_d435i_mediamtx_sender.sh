#!/usr/bin/env bash
set -Eeuo pipefail

DEVICE=${DEVICE:-/dev/d435i_color}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-30}
BITRATE_KBPS=${BITRATE_KBPS:-5000}
GOP=${GOP:-15}
RTSP_PATH=${RTSP_PATH:-d435i}
RTSP_PUBLISH_URL=${RTSP_PUBLISH_URL:-rtsp://127.0.0.1:8554/${RTSP_PATH}}
MEDIAMTX_BIN=${MEDIAMTX_BIN:-/home/sunrise/mediamtx/mediamtx}
MEDIAMTX_CONFIG=${MEDIAMTX_CONFIG:-}
ENCODER_BIN=${ENCODER_BIN:-}
package_prefix=""

command -v ffmpeg >/dev/null || { echo "ffmpeg not found" >&2; exit 127; }
[[ -e "$DEVICE" ]] || { echo "camera device not found: $DEVICE" >&2; exit 1; }
[[ -x "$MEDIAMTX_BIN" ]] || { echo "MediaMTX not executable: $MEDIAMTX_BIN" >&2; exit 1; }

if command -v ros2 >/dev/null; then
  package_prefix=$(ros2 pkg prefix drone_perception 2>/dev/null || true)
fi

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

if [[ -z "$MEDIAMTX_CONFIG" && -n "$package_prefix" ]]; then
  installed_config="$package_prefix/share/drone_perception/config/mediamtx_d435i.yml"
  if [[ -f "$installed_config" ]]; then
    MEDIAMTX_CONFIG="$installed_config"
  fi
fi

mediamtx_pid=""
cleanup()
{
  trap - EXIT INT TERM
  if [[ -n "$mediamtx_pid" ]]; then
    kill "$mediamtx_pid" 2>/dev/null || true
    wait "$mediamtx_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE '(^|:)8554$'; then
  echo "TCP port 8554 is already in use; stop the existing RTSP server first" >&2
  exit 1
fi

mediamtx_args=()
if [[ -n "$MEDIAMTX_CONFIG" ]]; then
  [[ -f "$MEDIAMTX_CONFIG" ]] || { echo "MediaMTX config not found: $MEDIAMTX_CONFIG" >&2; exit 1; }
  mediamtx_args+=("$MEDIAMTX_CONFIG")
fi
"$MEDIAMTX_BIN" "${mediamtx_args[@]}" &
mediamtx_pid=$!

for _ in $(seq 1 50); do
  if ! kill -0 "$mediamtx_pid" 2>/dev/null; then
    wait "$mediamtx_pid"
    exit 1
  fi
  if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE '(^|:)8554$'; then
    break
  fi
  sleep 0.1
done
if ! ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE '(^|:)8554$'; then
  echo "MediaMTX did not open TCP port 8554" >&2
  exit 1
fi

echo "Publishing ${WIDTH}x${HEIGHT}@${FPS} H.264 at ${BITRATE_KBPS} kbit/s" >&2
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
