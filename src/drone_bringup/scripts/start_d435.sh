#!/usr/bin/env bash
set -e

sleep 10

source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  rgb_camera.color_profile:=640,480,30 \
  enable_depth:=false \
  enable_rgbd:=false \
  enable_sync:=false \
  align_depth.enable:=false
