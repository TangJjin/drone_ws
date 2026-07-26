#!/usr/bin/env bash
# ==============================================================================
# 相机标定采图脚本（MATLAB Camera Calibrator 用）
#
# 在香橙派上运行，从工业相机（USB 32e4:6577）按固定间隔抓取 1280x720 JPEG，
# 供 MATLAB 标定 camera_fx/fy/cx/cy。完整流程见
# docs/camera_fx_matlab_calibration.md。
#
# 用法：
#   ./src/drone_perception/scripts/calib_capture.sh                  # 40 张、间隔 3s
#   ./src/drone_perception/scripts/calib_capture.sh 30 2             # 30 张、间隔 2s
#   ./src/drone_perception/scripts/calib_capture.sh 40 3 /dev/video1 # 手动指定设备
#
# ⚠ 拍摄前把变焦调到飞行档位并固定；本组图拍完前不许再碰焦距（fx 随焦距变）。
# ⚠ 必须先停视觉节点，否则相机被占用。
# ==============================================================================
set -euo pipefail

COUNT="${1:-40}"
INTERVAL="${2:-3}"
DEV="${3:-}"
WIDTH=1280
HEIGHT=720
VID=32e4
PID=6577

if pgrep -f industrial_animal_vision_node >/dev/null 2>&1; then
  echo "错误：industrial_animal_vision_node 正在运行（占用相机）。先停节点："
  echo "  pkill -f 'ros2 launch drone_perception'; pkill -f industrial_animal_vision_node"
  exit 1
fi

command -v v4l2-ctl >/dev/null 2>&1 || { echo "缺少 v4l2-ctl：sudo apt install v4l-utils"; exit 1; }

# 按 USB VID:PID 自动定位支持 MJPG 的采集节点
if [[ -z "$DEV" ]]; then
  for v in /dev/video*; do
    info=$(udevadm info -q property -n "$v" 2>/dev/null) || continue
    grep -q "ID_VENDOR_ID=$VID" <<<"$info" || continue
    grep -q "ID_MODEL_ID=$PID" <<<"$info" || continue
    v4l2-ctl -d "$v" --list-formats 2>/dev/null | grep -q MJPG || continue
    DEV="$v"
    break
  done
fi
[[ -n "$DEV" ]] || { echo "错误：没找到相机 $VID:$PID（拔插 USB 重试，或手动传第 3 个参数）"; exit 1; }

OUT="$HOME/calib_imgs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

cat <<EOF
设备: $DEV   分辨率: ${WIDTH}x${HEIGHT}   共 $COUNT 张，间隔 ${INTERVAL}s
输出: $OUT

拍摄要领（两张之间挪动棋盘换姿态，倒计时到点前保持棋盘完全静止）：
  - 远近各拍几张：棋盘占画面 1/3 ~ 2/3
  - 位置覆盖：画面中心 + 四个角落都要出现过
  - 姿态覆盖：前后/左右各倾斜 30~45 度，再加几张平面内旋转的
Ctrl+C 可提前结束。3 秒后开始……
EOF
sleep 3

ok=0
for i in $(seq -w 1 "$COUNT"); do
  f="$OUT/img_$i.jpg"
  # 每张跳过前 60 帧等自动曝光收敛，存第 61 帧
  if v4l2-ctl -d "$DEV" \
       --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=MJPG \
       --stream-mmap=4 --stream-skip=60 --stream-count=1 \
       --stream-to="$f" >/dev/null 2>&1 \
     && [[ -s "$f" ]] && [[ "$(stat -c%s "$f")" -gt 10000 ]]; then
    ok=$((ok + 1))
    echo "[$i/$COUNT] √ img_$i.jpg"
  else
    rm -f "$f"
    echo "[$i/$COUNT] × 抓取失败（相机若卡死见 docs/camera_usb_wedge_bringup_recovery.md）"
  fi
  sleep "$INTERVAL"
done

echo
echo "完成：$ok/$COUNT 张 → $OUT"
echo "在 PC（PowerShell）上取回："
echo "  scp -r orangepi@192.168.46.168:$OUT D:/calib_imgs/"
