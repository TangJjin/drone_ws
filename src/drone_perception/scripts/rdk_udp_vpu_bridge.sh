#!/usr/bin/env bash
set -Eeuo pipefail

# Feed an MPEG-TS/UDP H.264 stream into the RDK sample_codec VPU decoder.
# FFmpeg is used only as a transport demux/relay; it does not decode video.

PORT=5004
SAMPLE=/app/multimedia_samples/sample_codec/sample_codec
BASE_CONFIG=/app/multimedia_samples/sample_codec/codec_config.ini
OUTPUT=/dev/null
WIDTH=640
HEIGHT=360
DURATION=0
LOG_LEVEL=warn

usage() {
  cat <<'EOF'
Usage: rdk_udp_vpu_bridge.sh [options]

  --port N          UDP MPEG-TS port (default: 5004)
  --sample PATH     RDK sample_codec executable
  --config PATH     Base sample_codec codec_config.ini
  --output PATH     NV12 output file, or /dev/null (default: /dev/null)
  --width N         H.264 width (default: 640)
  --height N        H.264 height (default: 360)
  --duration SEC    Stop after SEC seconds; 0 means run until Ctrl-C
  --verbose         Keep sample_codec verbose logging
EOF
}

while (($#)); do
  case "$1" in
    --port) PORT=$2; shift 2;;
    --sample) SAMPLE=$2; shift 2;;
    --config) BASE_CONFIG=$2; shift 2;;
    --output) OUTPUT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    --verbose) LOG_LEVEL=verbose; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

command -v ffmpeg >/dev/null || { echo "ffmpeg not found" >&2; exit 127; }
[[ -x "$SAMPLE" ]] || { echo "sample_codec not executable: $SAMPLE" >&2; exit 1; }
[[ -f "$BASE_CONFIG" ]] || { echo "codec config not found: $BASE_CONFIG" >&2; exit 1; }

WORK_DIR=$(mktemp -d /tmp/rdk_udp_vpu_bridge.XXXXXX)
FIFO="$WORK_DIR/video.ts"
CONFIG="$WORK_DIR/codec_config.ini"
LOG="$WORK_DIR/sample_codec.log"
mkfifo "$FIFO"

cleanup() {
  trap - EXIT INT TERM
  [[ -n "${RELAY_PID:-}" ]] && kill "$RELAY_PID" 2>/dev/null || true
  [[ -n "${CODEC_PID:-}" ]] && kill "$CODEC_PID" 2>/dev/null || true
  wait 2>/dev/null || true
  rm -rf "$WORK_DIR"
}
trap cleanup EXIT INT TERM

awk -v input="$FIFO" -v output="$OUTPUT" -v width="$WIDTH" -v height="$HEIGHT" '
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

echo "[bridge] UDP port: $PORT"
echo "[bridge] resolution: ${WIDTH}x${HEIGHT}"
echo "[bridge] output: $OUTPUT"
echo "[bridge] hardware decoder: $SAMPLE"

stdbuf -oL "$SAMPLE" -f "$CONFIG" -d 0x1 -v >"$LOG" 2>&1 &
CODEC_PID=$!

ffmpeg_args=(
  -y -hide_banner -loglevel warning
  -fflags nobuffer -flags low_delay
  -probesize 32768 -analyzeduration 100000
  -i "udp://0.0.0.0:${PORT}?fifo_size=1000000&buffer_size=1048576&overrun_nonfatal=1"
  -map 0:v:0 -c copy -f mpegts "$FIFO"
)

if ((DURATION > 0)); then
  timeout --signal=TERM --kill-after=2 "$DURATION" ffmpeg "${ffmpeg_args[@]}" &
else
  ffmpeg "${ffmpeg_args[@]}" &
fi
RELAY_PID=$!

set +e
wait "$RELAY_PID"
RELAY_RC=$?
set -e
kill "$CODEC_PID" 2>/dev/null || true
sleep 1
kill -KILL "$CODEC_PID" 2>/dev/null || true
wait "$CODEC_PID" 2>/dev/null || true

cp "$LOG" /tmp/rdk_udp_vpu_bridge.last.log 2>/dev/null || true

echo "[bridge] relay exit: $RELAY_RC"
echo "[bridge] decoder log: $LOG"
grep -E "get frame size|Decode idx:|successful|error|failed" "$LOG" | tail -20 || true
