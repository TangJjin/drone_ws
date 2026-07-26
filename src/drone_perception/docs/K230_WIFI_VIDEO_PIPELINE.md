# K230 GC2093 无线图传与 RDK X5 视觉链路

## 目标链路

```text
GC2093 CSI1
  -> K230 VICAP
  -> K230 VENC H.264
  -> RTSP/RTP over Wi-Fi
  -> RDK X5 FFmpeg 解封装（不解码）
  -> RDK X5 sample_codec / VPU 硬件解码
  -> NV12
  -> /rdk_video/image
  -> qr_vision_node / BPU
```

先验收 1280x720@30，再尝试 1280x720@60。不要在链路尚未稳定时同时提高帧率、码率和分辨率。

## 第一、二步：MicroPython 联调

K230 SDK 内的诊断目录：

```text
/home/gjl/Projects/K230-SDK/src/reference/my_project/k230_wifi_video_diag
```

1. 将 `wifi_secrets.example.py` 复制到 K230 的 `/sdcard/wifi_secrets.py`。
2. 在板上填写热点 SSID、密码以及 RDK X5 的热点内网 IP；不要将真实密码提交到 Git。
3. 在 RDK X5 上先启动 UDP 探针：

```bash
python3 scripts/rdk_k230_link_probe.py --host 0.0.0.0 --port 5601
```

4. 用 CanMV IDE 运行 `wifi_gc2093_diag.py`。

验收标准：

- 出现 `[WIFI] connected` 和有效 IPv4 地址；
- RDK X5 持续出现 `[RX]`，`lost` 不持续增长；
- K230 LCD 有实时画面；
- K230 持续出现 `[STATS]`，相机 FPS 稳定且无持续掉线；
- 停止程序后出现 `[STOP] cleanup complete`。

## 第三、四步：K230 SDK H.264/RTSP 发送

在 WSL 中构建：

```bash
cd /home/gjl/Projects/K230-SDK/src/common/cdk/user
source ./build_env.sh
make -C samples/k230_video_rtsp -j2
```

输出文件：

```text
/home/gjl/Projects/K230-SDK/src/common/cdk/user/out/little/k230_video_rtsp
```

将该文件以及 `samples/k230_video_rtsp/wifi_sta_watchdog.sh` 复制到 K230 little Linux。创建仅 root 可读的 `/etc/k230_wifi.conf`：

```sh
WIFI_SSID='你的热点名称'
WIFI_PASSWORD='你的热点密码'
```

```sh
chmod 600 /etc/k230_wifi.conf
chmod +x /usr/bin/wifi_sta_watchdog.sh /usr/bin/k230_video_rtsp
/usr/bin/wifi_sta_watchdog.sh
```

另开一个 K230 终端启动 720p30：

```sh
/usr/bin/k230_video_rtsp -s 63 -w 1280 -h 720 -f 30 -g 15 -b 4000 -p 8554 -u live
```

RDK X5 访问地址：

```text
rtsp://<K230热点内网IP>:8554/live
```

720p30 稳定后才测试 720p60：

```sh
/usr/bin/k230_video_rtsp -s 64 -w 1280 -h 720 -f 60 -g 30 -b 6000 -p 8554 -u live
```

## 第五步：RDK X5 VPU 解码与视觉处理

在 RDK X5 上构建当前 ROS 2 包：

```bash
cd ~/drone_ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --packages-select drone_perception
source install/setup.bash
```

启动完整链路：

```bash
ros2 pkg prefix drone_perception
~/drone_ws/src/drone_perception/scripts/rdk_k230_video_vision.sh \
  --url rtsp://<K230热点内网IP>:8554/live \
  --width 1280 --height 720 --transport udp
```

如果 Wi-Fi 丢包导致马赛克或恢复慢，先改用 RTP over TCP 对照：

```bash
~/drone_ws/src/drone_perception/scripts/rdk_k230_video_vision.sh \
  --url rtsp://<K230热点内网IP>:8554/live \
  --width 1280 --height 720 --transport tcp
```

仅验证硬件解码和图像发布时，可先不开视觉节点，分别运行：

```bash
mkfifo /tmp/k230.nv12
python3 scripts/rdk_nv12_display.py \
  --input /tmp/k230.nv12 --width 1280 --height 720 \
  --topic /rdk_video/image --no-display
```

```bash
scripts/rdk_rtsp_vpu_bridge.sh \
  --url rtsp://<K230热点内网IP>:8554/live \
  --width 1280 --height 720 --transport udp \
  --output /tmp/k230.nv12
```

## 日志定位

| 日志 | 含义 | 优先检查 |
|---|---|---|
| `[WIFI][ERROR]` | 热点关联或 DHCP 失败 | 热点频段、密码、信号、`wlan0` |
| `[STATS] fps` 很低 | K230 采集/编码端未达到目标 | sensor 枚举、输入 FPS、温度与供电 |
| `[RTSP][WARN]` | RTSP 发送失败 | 客户端连接、K230 IP、端口 8554 |
| `[DISCONNECT]` | RDK 解封装断开并准备重连 | FFmpeg 最后日志、热点丢包 |
| `sample_codec` error | VPU 初始化、码流或尺寸不匹配 | `/dev/vpu`、H.264、宽高参数 |
| `[INPUT][WARN]` | VPU NV12 写端关闭 | 同时查看 bridge 和 codec 日志 |
| ROS 有图但识别慢 | 解码成功，视觉处理成为瓶颈 | BPU模型、图像转换、调试窗口 |

保留的最后一次接收端日志：

```text
/tmp/rdk_rtsp_vpu_bridge.ffmpeg.last.log
/tmp/rdk_rtsp_vpu_bridge.codec.last.log
```

## 当前验证边界

- K230 C++ 发送程序已完成交叉编译验证。
- MicroPython 脚本、Wi-Fi 驱动、RDK X5 `sample_codec` 连续 FIFO 输出和端到端帧率仍必须在真实板卡上验证。
- FFmpeg 在本方案中只负责 RTSP/RTP 解封装和 H.264 原样转发；H.264 解码由 RDK X5 VPU 完成。
