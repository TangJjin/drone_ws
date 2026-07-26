#!/usr/bin/env bash
set -Eeo pipefail

[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
[[ -f /opt/tros/humble/setup.bash ]] && source /opt/tros/humble/setup.bash
[[ -f "$HOME/drone_ws/install/setup.bash" ]] && source "$HOME/drone_ws/install/setup.bash"
set -u

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BRIDGE=${BRIDGE:-"$SCRIPT_DIR/rdk_rtsp_vpu_bridge.sh"}
PUBLISHER=${PUBLISHER:-"$SCRIPT_DIR/rdk_nv12_display.py"}
URL=""
WIDTH=1280
HEIGHT=720
TOPIC=/rdk_video/image
MODEL=/home/sunrise/drone_ws/src/drone_perception/new_640x640_nv12.bin
TRANSPORT=udp
SHOW_VIDEO=0
DEBUG_VIEW=0

usage() {
  cat <<'EOF'
Usage: rdk_k230_video_vision.sh --url RTSP_URL [options]

  --width N           stream width (default: 1280)
  --height N          stream height (default: 720)
  --topic TOPIC       ROS color topic (default: /rdk_video/image)
  --model PATH        RDK X5 BPU model
  --transport udp|tcp RTSP RTP transport (default: udp)
  --show-video        open an OpenCV preview window
  --debug-view        enable qr_vision_node debug window
EOF
}

while (($#)); do
  case "$1" in
    --url) URL=$2; shift 2;;
    --width) WIDTH=$2; shift 2;;
    --height) HEIGHT=$2; shift 2;;
    --topic) TOPIC=$2; shift 2;;
    --model) MODEL=$2; shift 2;;
    --transport) TRANSPORT=$2; shift 2;;
    --show-video) SHOW_VIDEO=1; shift;;
    --debug-view) DEBUG_VIEW=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "[CONFIG][ERROR] unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done

[[ -n "$URL" ]] || { echo "[CONFIG][ERROR] --url is required" >&2; exit 2; }
[[ -x "$BRIDGE" ]] || { echo "[DEPENDENCY][ERROR] bridge not executable: $BRIDGE" >&2; exit 1; }
[[ -f "$PUBLISHER" ]] || { echo "[DEPENDENCY][ERROR] publisher missing: $PUBLISHER" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "[DEPENDENCY][ERROR] BPU model missing: $MODEL" >&2; exit 1; }
command -v ros2 >/dev/null || { echo "[DEPENDENCY][ERROR] ros2 not found or environment not sourced" >&2; exit 127; }

WORK_DIR=$(mktemp -d /tmp/rdk_k230_video_vision.XXXXXX)
NV12_FIFO="$WORK_DIR/k230.nv12"
mkfifo "$NV12_FIFO"
BRIDGE_PID=""
PUBLISHER_PID=""
VISION_PID=""

cleanup() {
  trap - EXIT INT TERM
  [[ -n "$VISION_PID" ]] && kill "$VISION_PID" 2>/dev/null || true
  [[ -n "$BRIDGE_PID" ]] && kill "$BRIDGE_PID" 2>/dev/null || true
  [[ -n "$PUBLISHER_PID" ]] && kill "$PUBLISHER_PID" 2>/dev/null || true
  wait 2>/dev/null || true
  rm -rf "$WORK_DIR"
}
trap cleanup EXIT INT TERM

publisher_args=(
  --input "$NV12_FIFO"
  --width "$WIDTH"
  --height "$HEIGHT"
  --topic "$TOPIC"
  --frame-id k230_gc2093
)
((SHOW_VIDEO == 1)) || publisher_args+=(--no-display)

echo "[START] publisher topic=$TOPIC"
python3 "$PUBLISHER" "${publisher_args[@]}" &
PUBLISHER_PID=$!

echo "[START] RTSP/VPU bridge url=$URL"
"$BRIDGE" \
  --url "$URL" \
  --transport "$TRANSPORT" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --output "$NV12_FIFO" &
BRIDGE_PID=$!

sleep 2
kill -0 "$PUBLISHER_PID" 2>/dev/null || { echo "[START][ERROR] publisher exited" >&2; exit 1; }
kill -0 "$BRIDGE_PID" 2>/dev/null || { echo "[START][ERROR] bridge exited" >&2; exit 1; }

echo "[START] qr_vision_node color_topic=$TOPIC model=$MODEL"
ros2 run drone_perception qr_vision_node --ros-args \
  -p color_topic:="$TOPIC" \
  -p require_depth:=false \
  -p require_camera_info:=false \
  -p use_rgbd:=false \
  -p enable_rknn:=false \
  -p enable_bpu:=true \
  -p enable_bpu_ocr:=false \
  -p bpu_model_path:="$MODEL" \
  -p debug_view:=$([[ "$DEBUG_VIEW" == 1 ]] && echo true || echo false) &
VISION_PID=$!

set +e
wait "$VISION_PID"
vision_rc=$?
set -e
echo "[STOP] qr_vision_node exited rc=$vision_rc"
exit "$vision_rc"
