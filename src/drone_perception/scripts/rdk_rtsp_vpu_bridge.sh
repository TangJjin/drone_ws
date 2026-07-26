#!/usr/bin/env bash
set -Eeuo pipefail

# RTSP/RTP transport is handled by FFmpeg without decoding.  The resulting
# Annex-B H.264 elementary stream is decoded by the RDK X5 sample_codec VPU.

URL=""
SAMPLE=/app/multimedia_samples/sample_codec/sample_codec
BASE_CONFIG=/app/multimedia_samples/sample_codec/codec_config.ini
OUTPUT=/dev/null
WIDTH=1280
HEIGHT=720
DURATION=0
TRANSPORT=udp
RECONNECT_DELAY=2
ONCE=0

usage() {
  cat <<'EOF'
Usage: rdk_rtsp_vpu_bridge.sh --url RTSP_URL [options]

  --url URL          K230 RTSP URL, e.g. rtsp://192.168.1.20:8554/live
  --sample PATH      RDK sample_codec executable
  --config PATH      sample_codec codec_config.ini
  --output PATH      NV12 output FIFO/file (default: /dev/null)
  --width N          decoded width (default: 1280)
  --height N         decoded height (default: 720)
  --transport MODE   udp or tcp (default: udp)
  --duration SEC     total runtime; 0 means until Ctrl-C
  --reconnect SEC    restart delay after disconnect (default: 2)
  --once             do not reconnect
EOF
}

while (($#)); do
  case "$1" in
    --url) URL=$2; shift 2;;
    --sample) SAMPLE=$2; shift 2;;
    --config) BASE_CONFIG=$2; shift 2;;
    --output) OUTPUT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --transport) TRANSPORT=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    --reconnect) RECONNECT_DELAY=$2; shift 2;;
    --once) ONCE=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "[CONFIG][ERROR] unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

[[ -n "$URL" ]] || { echo "[CONFIG][ERROR] --url is required" >&2; exit 2; }
[[ "$TRANSPORT" == udp || "$TRANSPORT" == tcp ]] || {
  echo "[CONFIG][ERROR] transport must be udp or tcp" >&2; exit 2;
}
command -v ffmpeg >/dev/null || { echo "[DEPENDENCY][ERROR] ffmpeg not found" >&2; exit 127; }
[[ -x "$SAMPLE" ]] || { echo "[DEPENDENCY][ERROR] sample_codec not executable: $SAMPLE" >&2; exit 1; }
[[ -f "$BASE_CONFIG" ]] || { echo "[DEPENDENCY][ERROR] codec config not found: $BASE_CONFIG" >&2; exit 1; }

WORK_DIR=$(mktemp -d /tmp/rdk_rtsp_vpu_bridge.XXXXXX)
INPUT_FIFO="$WORK_DIR/video.h264"
CONFIG="$WORK_DIR/codec_config.ini"
CODEC_LOG="$WORK_DIR/sample_codec.log"
FFMPEG_LOG="$WORK_DIR/ffmpeg.log"
mkfifo "$INPUT_FIFO"

CODEC_PID=""
FFMPEG_PID=""
STOP=0

stop_children() {
  [[ -n "$FFMPEG_PID" ]] && kill "$FFMPEG_PID" 2>/dev/null || true
  [[ -n "$CODEC_PID" ]] && kill "$CODEC_PID" 2>/dev/null || true
  for _ in {1..20}; do
    { [[ -z "$FFMPEG_PID" ]] || ! kill -0 "$FFMPEG_PID" 2>/dev/null; } && \
      { [[ -z "$CODEC_PID" ]] || ! kill -0 "$CODEC_PID" 2>/dev/null; } && break
    sleep 0.1
  done
  [[ -n "$FFMPEG_PID" ]] && kill -KILL "$FFMPEG_PID" 2>/dev/null || true
  [[ -n "$CODEC_PID" ]] && kill -KILL "$CODEC_PID" 2>/dev/null || true
  [[ -n "$FFMPEG_PID" ]] && wait "$FFMPEG_PID" 2>/dev/null || true
  [[ -n "$CODEC_PID" ]] && wait "$CODEC_PID" 2>/dev/null || true
  FFMPEG_PID=""
  CODEC_PID=""
}

on_signal() {
  STOP=1
  stop_children
}

cleanup() {
  trap - EXIT INT TERM
  stop_children
  cp "$CODEC_LOG" /tmp/rdk_rtsp_vpu_bridge.codec.last.log 2>/dev/null || true
  cp "$FFMPEG_LOG" /tmp/rdk_rtsp_vpu_bridge.ffmpeg.last.log 2>/dev/null || true
  rm -rf "$WORK_DIR"
}
trap on_signal INT TERM
trap cleanup EXIT

awk -v input="$INPUT_FIFO" -v output="$OUTPUT" -v width="$WIDTH" -v height="$HEIGHT" '
  /^encode_streams[[:space:]]*=/ { print "encode_streams = 0x0"; next }
  /^decode_streams[[:space:]]*=/ { print "decode_streams = 0x1"; next }
  /^\[vdec_stream1\]/ { in_decoder=1; print; next }
  /^\[/ && $0 != "[vdec_stream1]" { in_decoder=0 }
  in_decoder && /^codec_type[[:space:]]*=/ { print "codec_type = 0"; next }
  in_decoder && /^width[[:space:]]*=/ { print "width = " width; next }
  in_decoder && /^height[[:space:]]*=/ { print "height = " height; next }
  in_decoder && /^input[[:space:]]*=/ { print "input = " input; next }
  in_decoder && /^output[[:space:]]*=/ { print "output = " output; next }
  { print }
' "$BASE_CONFIG" > "$CONFIG"

echo "[CONFIG] url=$URL transport=$TRANSPORT resolution=${WIDTH}x${HEIGHT} output=$OUTPUT"
echo "[CONFIG] decoder=$SAMPLE reconnect_delay=${RECONNECT_DELAY}s once=$ONCE"

started_epoch=$(date +%s)
attempt=0
while ((STOP == 0)); do
  if ((DURATION > 0)) && (( $(date +%s) - started_epoch >= DURATION )); then
    echo "[STOP] duration reached"
    break
  fi

  attempt=$((attempt + 1))
  : > "$CODEC_LOG"
  : > "$FFMPEG_LOG"
  echo "[CONNECT] attempt=$attempt"

  stdbuf -oL "$SAMPLE" -f "$CONFIG" -d 0x1 -v >"$CODEC_LOG" 2>&1 &
  CODEC_PID=$!

  ffmpeg \
    -y -hide_banner -loglevel warning \
    -rtsp_transport "$TRANSPORT" \
    -rw_timeout 5000000 \
    -fflags nobuffer -flags low_delay \
    -analyzeduration 0 -probesize 32768 \
    -i "$URL" \
    -map 0:v:0 -an -c:v copy -bsf:v h264_mp4toannexb \
    -flush_packets 1 -f h264 "$INPUT_FIFO" >"$FFMPEG_LOG" 2>&1 &
  FFMPEG_PID=$!

  set +e
  wait "$FFMPEG_PID"
  relay_rc=$?
  set -e
  FFMPEG_PID=""
  [[ -n "$CODEC_PID" ]] && kill "$CODEC_PID" 2>/dev/null || true
  [[ -n "$CODEC_PID" ]] && wait "$CODEC_PID" 2>/dev/null || true
  CODEC_PID=""

  echo "[DISCONNECT] attempt=$attempt ffmpeg_rc=$relay_rc"
  grep -Ei "error|failed|invalid|timeout|refused|severed" "$FFMPEG_LOG" | tail -n 12 || true
  grep -Ei "Decode idx:|get frame size|error|failed" "$CODEC_LOG" | tail -n 12 || true

  ((ONCE == 0)) || break
  ((STOP == 0)) || break
  sleep "$RECONNECT_DELAY"
done

echo "[STOP] bridge exiting attempts=$attempt"
