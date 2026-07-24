#!/usr/bin/env bash
set -Eeuo pipefail

DEVICE=${DEVICE:-/dev/video1}
HOST=${HOST:-192.168.50.2}
PORT=${PORT:-5004}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-360}
FPS=${FPS:-15}
MTU=${MTU:-1200}

while (($#)); do
  case "$1" in
    --device) DEVICE=$2; shift 2;;
    --host) HOST=$2; shift 2;;
    --port) PORT=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --fps) FPS=$2; shift 2;;
    --mtu) MTU=$2; shift 2;;
    -h|--help)
      echo "Usage: $0 [--device DEV] [--host IP] [--port N] [--width N] [--height N] [--fps N] [--mtu N]"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 2;;
  esac
done

exec gst-launch-1.0 -v \
  v4l2src device="$DEVICE" io-mode=2 do-timestamp=true ! \
  image/jpeg,width="$WIDTH",height="$HEIGHT",framerate="$FPS"/1 ! \
  jpegparse ! \
  rtpjpegpay mtu="$MTU" ! \
  udpsink host="$HOST" port="$PORT" sync=false async=false
