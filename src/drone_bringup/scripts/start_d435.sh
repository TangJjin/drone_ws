#!/usr/bin/env bash
set -e

sleep 10

source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  serial_no:=_327122074056 \
  enable_color:=true \
  rgb_camera.color_profile:=640x480x30 \
  rgb_camera.power_line_frequency:=1 \
  enable_depth:=true \
  depth_module.depth_profile:=640x480x30 \
  enable_sync:=true \
  enable_rgbd:=true \
  align_depth.enable:=true
