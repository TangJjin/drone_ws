#!/usr/bin/env bash
set -Eeuo pipefail

# RealSense D435i color node exposed by the librealsense V4L2 backend.
DEVICE=${DEVICE:-/dev/video5}
HOST=${HOST:-192.168.46.114}
PORT=${PORT:-5004}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-30}
BITRATE=${BITRATE:-1200000}
GOP=${GOP:-10}

while (($#)); do
  case "$1" in
    --device) DEVICE=$2; shift 2;;
    --host) HOST=$2; shift 2;;
    --port) PORT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --fps) FPS=$2; shift 2;;
    --bitrate) BITRATE=$2; shift 2;;
    --gop) GOP=$2; shift 2;;
    -h|--help)
      echo "Usage: $0 [--device DEV] [--host IP] [--port N] [--width N] [--height N] [--fps N] [--bitrate BPS] [--gop N]"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 2;;
  esac
done

[[ -e "$DEVICE" ]] || { echo "RealSense V4L2 device not found: $DEVICE" >&2; exit 1; }

exec gst-launch-1.0 -v \
  v4l2src device="$DEVICE" io-mode=2 do-timestamp=true ! \
  video/x-raw,format=YUY2,width="$WIDTH",height="$HEIGHT",framerate="$FPS"/1 ! \
  videoconvert ! video/x-raw,format=NV12 ! \
  queue max-size-buffers=1 leaky=downstream ! \
  mpph264enc \
    rc-mode=cbr \
    bps="$BITRATE" \
    bps-min="$BITRATE" \
    bps-max="$BITRATE" \
    gop="$GOP" \
    profile=baseline \
    header-mode=each-idr \
    zero-copy-pkt=true ! \
  h264parse config-interval=-1 ! \
  mpegtsmux alignment=7 ! \
  udpsink host="$HOST" port="$PORT" sync=false async=false
