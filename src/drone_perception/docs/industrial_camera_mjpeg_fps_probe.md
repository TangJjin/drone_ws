# 工业相机 MJPEG 参数与真实 FPS 探测

> 目标：在修改 RKNN 动物识别链路前，先确认工业相机的真实设备节点、USB 协商速率、MJPEG 模式、实际采集 FPS，以及 Orange Pi 上 MPP/RGA 硬件链路能跑到的 FPS。
>
> 本文只做相机与图像前端探测，不启动 D435i、ROS 图像链路或 RKNN 推理。

## 1. 测试原则

1. 不根据相机宣传参数直接认定 `1920x1080@230` 可用，以 `v4l2-ctl` 枚举和持续采集结果为准。
2. `/dev/video*` 编号会随设备和启动顺序变化，必须先枚举；不能沿用 D435i 或其他相机的节点号。
3. 先测“只取 MJPEG”，再测“MPP 解码”，最后测“MPP + RGA 预处理”，避免把解码瓶颈误判成相机帧率不足。
4. 每次只运行一个相机消费者。遇到 `Device or resource busy` 时先查占用进程，不使用宽泛的 `pkill`。
5. 性能测试默认不显示窗口，显示和 `cv::imshow` 会降低高帧率链路的吞吐。

## 2. 插接要求

- 工业相机直接连接 Orange Pi 的 USB 3.x/SuperSpeed 端口。
- 优先使用原装或确认支持 SuperSpeed 的短线，不经过 USB 2.0 Hub。
- 如果 `lsusb -t` 显示相机只有 `480M`，说明当前仍是 USB 2.0 链路；应先更换端口或线材，再讨论 230 FPS。

检查 USB 拓扑：

```bash
lsusb
lsusb -t
```

预期 USB 3.x 设备所在分支显示 `5000M`、`10000M` 或更高，而不是 `480M`。

## 3. 找到正确的视频节点

先列出相机和全部视频节点：

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ 2>/dev/null
```

逐个检查节点能力：

```bash
for camera_node in /dev/video*; do
  [ -e "$camera_node" ] || continue
  echo
  echo "===== $camera_node ====="
  v4l2-ctl -d "$camera_node" --all 2>&1 | sed -n '1,45p'
done
```

选择同时具备以下条件的节点：

- `Device Caps` 包含 `Video Capture` 和 `Streaming`；
- `--list-formats-ext` 能枚举 `MJPG`；
- 不是只包含 `Metadata Capture` 的元数据节点。

设置本轮测试变量。下面的 `/dev/video0` 只是占位，必须换成刚刚确认的节点：

```bash
export CAMERA_DEVICE=/dev/video0
export CAMERA_WIDTH=1920
export CAMERA_HEIGHT=1080
export REQUESTED_FPS=230
export TEST_FPS=230
export TEST_SECONDS=15
export CAMERA_PROBE_DIR=/tmp/industrial_camera_probe_$(date +%Y%m%d_%H%M%S)
mkdir -p "$CAMERA_PROBE_DIR"
```

记录设备身份和驱动信息：

```bash
udevadm info -q property -n "$CAMERA_DEVICE" | sort | tee "$CAMERA_PROBE_DIR/udev.txt"
v4l2-ctl -d "$CAMERA_DEVICE" --all 2>&1 | tee "$CAMERA_PROBE_DIR/device_all.txt"
```

## 4. 读取相机声明的 MJPEG 参数

完整枚举格式、分辨率和帧率：

```bash
v4l2-ctl -d "$CAMERA_DEVICE" --list-formats-ext 2>&1 \
  | tee "$CAMERA_PROBE_DIR/formats.txt"
```

重点查找：

```text
'MJPG'
  Size: Discrete 1920x1080
    Interval: ... (230.000 fps)
```

如果 `1920x1080 MJPG` 下没有 230 FPS：

1. 将 `TEST_FPS` 改为该分辨率下枚举出的最高 FPS；
2. 不降低分辨率来伪装成满足 1080p 要求；
3. 在测试结论中同时记录“请求 230 FPS”和“设备最高可用 FPS”。

例如设备最高只支持 120 FPS：

```bash
export TEST_FPS=120
```

## 5. 设置并回读实际模式

确认没有其他进程占用相机：

```bash
fuser -v "$CAMERA_DEVICE" 2>&1 || true
```

设置 MJPEG、分辨率和测试帧率，然后立即回读：

```bash
v4l2-ctl -d "$CAMERA_DEVICE" \
  --set-fmt-video=width="$CAMERA_WIDTH",height="$CAMERA_HEIGHT",pixelformat=MJPG \
  --set-parm="$TEST_FPS"

v4l2-ctl -d "$CAMERA_DEVICE" --get-fmt-video --get-parm 2>&1 \
  | tee "$CAMERA_PROBE_DIR/negotiated_mode.txt"
```

必须检查回读值。驱动可能接受 230 的设置命令，但最终回退到 30、60 或其他帧率。

## 6. 测试一：只采集 MJPEG，不解码

这一项用于判断相机、USB、UVC 驱动本身能否持续输出目标帧率。

```bash
set -o pipefail
timeout --signal=INT "${TEST_SECONDS}s" \
  v4l2-ctl -d "$CAMERA_DEVICE" \
    --set-fmt-video=width="$CAMERA_WIDTH",height="$CAMERA_HEIGHT",pixelformat=MJPG \
    --set-parm="$TEST_FPS" \
    --stream-mmap=8 \
    --stream-poll \
    --stream-count=100000 \
    --stream-to=/dev/null \
    --verbose 2>&1 \
  | tee "$CAMERA_PROBE_DIR/v4l2_capture.txt"
```

`timeout` 到时返回 124 属于正常结束方式。记录输出中的稳定 FPS、错误、丢帧或超时信息。

再用 GStreamer 复测压缩帧吞吐，不做 JPEG 解码：

```bash
timeout --signal=INT "${TEST_SECONDS}s" \
  gst-launch-1.0 -e \
    v4l2src device="$CAMERA_DEVICE" io-mode=2 do-timestamp=true ! \
    image/jpeg,width="$CAMERA_WIDTH",height="$CAMERA_HEIGHT",framerate="$TEST_FPS"/1 ! \
    queue max-size-buffers=1 leaky=downstream ! \
    jpegparse ! \
    fpsdisplaysink video-sink=fakesink text-overlay=false sync=false \
      fps-update-interval=1000 2>&1 \
  | tee "$CAMERA_PROBE_DIR/gst_mjpeg_capture.txt"
```

判断：

- 两项都接近 `TEST_FPS`：相机和 USB 输入基本合格；
- V4L2 高、GStreamer 低：检查 GStreamer caps、队列和插件；
- 两项都低：优先检查 USB 速率、线材、端口、相机曝光和驱动声明。

## 7. 测试二：MPP 硬件 JPEG 解码

该测试只在 Orange Pi/RK3588 上进行。先确认插件：

```bash
gst-inspect-1.0 mppjpegdec
```

插件不存在时停止本项并保存错误，不用 `jpegdec` 的软件结果冒充 MPP 硬解。

测试 MJPEG 到 DMABUF NV12 的硬解吞吐：

```bash
timeout --signal=INT "${TEST_SECONDS}s" \
  gst-launch-1.0 -e \
    v4l2src device="$CAMERA_DEVICE" io-mode=2 do-timestamp=true ! \
    image/jpeg,width="$CAMERA_WIDTH",height="$CAMERA_HEIGHT",framerate="$TEST_FPS"/1 ! \
    queue max-size-buffers=1 leaky=downstream ! \
    jpegparse ! \
    mppjpegdec format=NV12 dma-feature=true ! \
    fpsdisplaysink video-sink=fakesink text-overlay=false sync=false \
      fps-update-interval=1000 2>&1 \
  | tee "$CAMERA_PROBE_DIR/gst_mpp_decode.txt"
```

日志中应能看到类似以下 caps，才算硬件路径证据之一：

```text
video/x-raw(memory:DMABuf), format=NV12
```

仅仅 `gst-inspect-1.0` 能找到插件，不代表相机流已经真正完成硬解；必须同时保留运行时 caps、帧数和 FPS。

## 8. 测试三：MPP + RGA 到模型预处理尺寸

先确认 RGA 插件：

```bash
gst-inspect-1.0 rgaconvert
```

工业相机为 16:9，YOLO 模型输入为 640x640。本测试让 RGA 完成 `1920x1080 -> 640x360 RGB`，后续推理代码只需补上下黑边到 640x640。

```bash
timeout --signal=INT "${TEST_SECONDS}s" \
  gst-launch-1.0 -e \
    v4l2src device="$CAMERA_DEVICE" io-mode=2 do-timestamp=true ! \
    image/jpeg,width="$CAMERA_WIDTH",height="$CAMERA_HEIGHT",framerate="$TEST_FPS"/1 ! \
    queue max-size-buffers=1 leaky=downstream ! \
    jpegparse ! \
    mppjpegdec format=NV12 dma-feature=true ! \
    queue max-size-buffers=1 leaky=downstream ! \
    rgaconvert ! \
    video/x-raw,format=RGB,width=640,height=360 ! \
    fpsdisplaysink video-sink=fakesink text-overlay=false sync=false \
      fps-update-interval=1000 2>&1 \
  | tee "$CAMERA_PROBE_DIR/gst_mpp_rga_640x360.txt"
```

如果 RGA 协商失败，完整保留错误和 caps；不要先加多个 `videoconvert` 掩盖格式问题。

## 9. 同时观察 CPU、USB 和温度

运行任一 GStreamer 测试时，在另一个终端执行：

```bash
gst_pid=$(pgrep -n gst-launch-1.0)
pidstat -u -r -p "$gst_pid" 1
```

Orange Pi 另外记录：

```bash
cat /sys/kernel/debug/rknpu/load 2>/dev/null || true
for thermal_zone in /sys/class/thermal/thermal_zone*/temp; do
  printf '%s ' "$thermal_zone"
  cat "$thermal_zone"
done
```

本阶段尚未运行 RKNN，所以 NPU 负载应接近空闲；这可以避免把其他残留 NPU 进程混进后续基线。

## 10. 30 秒和 5 分钟验证

首次探测先把 `TEST_SECONDS` 设为 30：

```bash
export TEST_SECONDS=30
```

模式和硬件链路确认后，再进行 5 分钟无显示测试：

```bash
export TEST_SECONDS=300
```

5 分钟测试重点观察：

- FPS 是否从高值逐渐下降；
- 是否出现 `lost`, `drop`, `timeout`, `corrupt` 或协商错误；
- CPU 和温度是否持续上升并触发降频；
- 队列是否始终保持丢旧而不是形成不断增长的延迟。

## 11. 需要保留的结果

测试完成后保留整个日志目录：

```bash
echo "$CAMERA_PROBE_DIR"
find "$CAMERA_PROBE_DIR" -maxdepth 1 -type f -printf '%f\n' | sort
```

最终结论至少包含：

| 项目 | 记录值 |
| --- | --- |
| 相机厂商、VID:PID、序列号 | 待测 |
| 实际 Capture 节点 | 待测 |
| USB 协商速率 | 待测 |
| 1920x1080 MJPEG 声明最高 FPS | 待测 |
| 驱动最终协商 FPS | 待测 |
| V4L2 只采集实际 FPS | 待测 |
| GStreamer 只采集实际 FPS | 待测 |
| MPP 解码实际 FPS | 待测 |
| MPP + RGA 640x360 实际 FPS | 待测 |
| CPU 占用、温度、错误和丢帧 | 待测 |

## 12. 当前已知对照信息

- 2026-07-24 在当前电脑上枚举到的 Sonix USB Camera 为 USB 2.0 `480M`，`1920x1080 MJPEG` 最高只有 30 FPS；它不能作为工业相机 230 FPS 能力的证明。
- 之前另一台 `12MP U3 Camera` 曾在 Orange Pi 上走通 `MJPEG -> mppjpegdec -> DMABUF NV12`，但当时验证的是 `1280x720@30`；更换相机、分辨率和帧率后必须重新测量。
- 后续 RKNN 动物识别启动参数只采用本轮真实枚举和持续测试得到的值。
