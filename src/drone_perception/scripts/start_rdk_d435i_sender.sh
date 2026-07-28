#!/usr/bin/env bash
set -Eeuo pipefail

DRONE_WS=${DRONE_WS:-/home/sunrise/drone_ws}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
MEDIAMTX_BIN=${MEDIAMTX_BIN:-/home/sunrise/mediamtx/mediamtx}

[[ -f "$ROS_SETUP" ]] || { echo "ROS setup not found: $ROS_SETUP" >&2; exit 1; }
[[ -f "$DRONE_WS/install/setup.bash" ]] || {
  echo "workspace is not built: $DRONE_WS" >&2
  exit 1
}

source "$ROS_SETUP"
source "$DRONE_WS/install/setup.bash"

package_prefix=$(ros2 pkg prefix drone_perception)
STREAM_CONFIG=${STREAM_CONFIG:-$package_prefix/share/drone_perception/config/rdk_d435i_stream.yaml}
ENCODER_BIN=${ENCODER_BIN:-$package_prefix/lib/drone_perception/rdk_d435i_h264_sender}
sender_script=$package_prefix/lib/drone_perception/rdk_d435i_mediamtx_sender.sh

exec env \
  STREAM_CONFIG="$STREAM_CONFIG" \
  ENCODER_BIN="$ENCODER_BIN" \
  MEDIAMTX_BIN="$MEDIAMTX_BIN" \
  "$sender_script"
