#!/usr/bin/env bash
set -Eeuo pipefail

PORT=${PORT:-5004}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-360}
DURATION=${DURATION:-0}
SINK=${SINK:-autovideosink}

usage() {
  echo "Usage: $0 [--port N] [--width N] [--height N] [--duration SEC] [--sink SINK]"
}

while (($#)); do
  case "$1" in
    --port) PORT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --duration) DURATION=$2; shift 2;;
    --sink) SINK=$2; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

pipeline=(
  gst-launch-1.0 -v
  udpsrc port="$PORT" buffer-size=1048576
    caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)JPEG,payload=(int)26,width=(int)$WIDTH,height=(int)$HEIGHT"
  !
  rtpjpegdepay
  !
  jpegdec
  !
  videoconvert
  !
  "$SINK" sync=false
)

if ((DURATION > 0)); then
  exec timeout --signal=TERM --kill-after=2 "$DURATION" "${pipeline[@]}"
fi

exec "${pipeline[@]}"
