# D435i 发送端到 Qt 地面站图传方案

## 1. 文档目标

本文说明 D435i 彩色视频如何由发送端编码并发布为 RTSP，以及后续如何在 Qt 地面站中接收、解码和显示。

本文重点是：

- 当前已经可用的 D435i、RDK H.264 硬编码和 MediaMTX 发送链路。
- RTSP 地址如何传给 Qt 地面站。
- Qt 地面站后续需要实现的 FFmpeg 解码线程、帧缓冲、自动重连和运行接口。
- 当前手机热点测试地址如何替换为未来香橙派 AP 网络中的实际地址。

本阶段只编写方案，不修改 `warehouse_ws`，因此 Qt 地面站当前还不能直接显示该 RTSP 视频。

> 发送端启动脚本只负责相机采集、编码和 MediaMTX，不会停止 LightDM、占用接收端 HDMI 或影响 Qt 地面站运行。

## 2. 设备角色与当前测试地址

本文统一使用角色名称：

| 角色 | 当前手机热点测试 IP | 职责 |
|---|---|---|
| 发送端 | `192.168.46.253` | 连接 D435i，采集 RGB，硬编码 H.264，并运行 MediaMTX |
| 接收端/Qt 地面站 | `192.168.46.114` | 连接发送端 RTSP 服务，在 Qt 中解码并显示视频 |

当前测试 RTSP 地址为：

```text
rtsp://192.168.46.253:8554/d435i
```

各字段含义：

| 字段 | 含义 |
|---|---|
| `rtsp://` | 使用 RTSP 协议访问实时视频 |
| `192.168.46.253` | 发送端在当前手机热点中的测试 IP，不是永久地址 |
| `8554` | 发送端 MediaMTX 的 RTSP 服务端口 |
| `d435i` | MediaMTX 中的视频流路径 |

`192.168.46.253` 是 IP 地址，`8554` 是端口。更换 Wi-Fi 或 AP 后，通常改变的是 IP，而不是 RTSP 端口。

发送端不需要知道接收端 IP。Qt 地面站作为客户端，只需要知道完整 RTSP 地址。

## 3. 图传数据链路

本方案只传输 D435i 的 RGB 彩色视频，不传输深度、红外、IMU 或点云：

```text
D435i RGB（USB 3.0 / YUYV）
  -> 发送端采集 320x240@30
  -> 转换为 NV12
  -> RDK 硬件 H.264 编码
  -> FFmpeg 复制 H.264 码流并发布
  -> MediaMTX
  -> rtsp://<发送端IP>:8554/d435i
  -> Qt 地面站 FFmpeg 工作线程
  -> H.264 软件解码
  -> RGB888
  -> QImage
  -> Qt 视频控件
```

当前发送参数：

| 参数 | 当前值 |
|---|---:|
| D435i 彩色输入 | YUYV |
| 分辨率 | `320x240` |
| 帧率 | `30 fps` |
| 编码格式 | H.264 Main |
| 目标码率 | `5000 kbit/s`，约 `5 Mbit/s` |
| GOP | `15` 帧 |
| RTSP 端口 | `8554` |
| 流路径 | `d435i` |

## 4. 发送端当前实现

### 4.1 发送端组件

发送端工作空间：

```text
/home/sunrise/drone_ws
```

主要组件：

| 组件 | 作用 |
|---|---|
| `/dev/d435i_color` | D435i RGB 彩色节点的稳定设备别名 |
| `rdk_d435i_h264_sender` | 采集 YUYV、转换 NV12并调用RDK硬件H.264编码器 |
| `rdk_d435i_mediamtx_sender.sh` | 组织编码器、FFmpeg和MediaMTX发布链路 |
| `start_rdk_d435i_sender.sh` | 标准部署的一键启动入口 |
| `rdk_d435i_stream.yaml` | 相机、编码器和RTSP发送参数 |
| MediaMTX | 向一个或多个客户端提供RTSP视频流 |

### 4.2 首次构建

代码首次部署或 C++、构建文件发生更新后执行：

```bash
cd /home/sunrise/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_perception --symlink-install
```

检查发送程序、启动脚本和 MediaMTX：

```bash
test -x /home/sunrise/drone_ws/install/drone_perception/lib/drone_perception/rdk_d435i_h264_sender
test -x /home/sunrise/drone_ws/install/drone_perception/lib/drone_perception/start_rdk_d435i_sender.sh
test -x /home/sunrise/mediamtx/mediamtx
```

### 4.3 检查 D435i RGB 节点

确认 D435i 通过 USB 3.0 连接：

```bash
lsusb
lsusb -t
v4l2-ctl --list-devices
```

确认稳定别名指向 YUYV 彩色节点：

```bash
ls -l /dev/d435i_color
readlink -f /dev/d435i_color
v4l2-ctl --device=/dev/d435i_color --get-fmt-video
```

正确结果应包含：

```text
Pixel Format      : 'YUYV'
```

如果显示 `Z16`，该节点是深度图，不是本方案需要的 RGB 彩色节点。

### 4.4 启动发送端

当前标准启动命令只有一个：

```bash
/home/sunrise/drone_ws/install/drone_perception/lib/drone_perception/start_rdk_d435i_sender.sh
```

该脚本会自动：

1. 加载 ROS 2 和 `drone_ws` 环境。
2. 读取发送端 YAML。
3. 启动 MediaMTX。
4. 启动 D435i 采集和 RDK H.264 硬编码器。
5. 将编码结果发布到本机 MediaMTX。
6. 对局域网提供 `rtsp://<发送端IP>:8554/d435i`。

正常日志应包含：

```text
Publishing 320x240@30 H.264 at 5000 kbit/s
Local publish URL: rtsp://127.0.0.1:8554/d435i
stream is available and online
```

脚本以前台方式运行。按 `Ctrl+C` 会结束编码器和由该脚本启动的 MediaMTX。

### 4.5 发送端 YAML

发送配置位于：

```text
/home/sunrise/drone_ws/src/drone_perception/config/rdk_d435i_stream.yaml
```

当前内容对应：

```yaml
camera:
  device: /dev/d435i_color
  width: 320
  height: 240
  fps: 30

encoder:
  bitrate_kbps: 5000
  gop: 15

rtsp:
  port: 8554
  path: d435i
  transports: [udp, tcp]
```

使用 `--symlink-install` 构建后，只修改这些运行参数不需要重新编译；停止并重新运行发送脚本即可生效。

该 YAML 只属于发送端，不包含接收端 IP。发送端可以同时服务 Qt 地面站、FFprobe 或其他 RTSP 客户端。

## 5. 当前如何验证 RTSP 接口

Qt 功能尚未实现前，可以在与发送端处于同一网络的设备上先验证接口。

### 5.1 网络连通

```bash
ping -c 4 192.168.46.253
```

### 5.2 检查视频编码信息

```bash
ffprobe -v error \
  -rtsp_transport tcp \
  -select_streams v:0 \
  -show_entries stream=codec_name,profile,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 \
  rtsp://192.168.46.253:8554/d435i
```

当前期望结果：

```text
codec_name=h264
profile=Main
width=320
height=240
r_frame_rate=30/1
```

### 5.3 临时播放器验证

```bash
ffplay -rtsp_transport tcp rtsp://192.168.46.253:8554/d435i
```

`ffplay` 只用于验证 RTSP 链路，不等于 Qt 地面站已经实现视频显示。

## 6. Qt 地面站当前状态

Qt 地面站工程位于：

```text
/home/sunrise/warehouse_ws
```

对应源码包为：

```text
warehouse_ws/src/drone_qt
```

当前 `ground_station` 使用 Qt Widgets、Qt SerialPort 和 ROS 2。现有图像显示只处理离散的条码图片，不具备以下连续视频能力：

- 打开 RTSP 地址。
- 持续读取 H.264 视频包。
- 解码 H.264。
- 按视频帧率刷新 Qt 控件。
- 网络断开后自动重连。

因此当前发送端可以正常提供 RTSP，但现有 Qt 地面站还不能直接显示该视频。发送端启动不会影响 Qt；缺少的是 Qt 内部的接收和解码模块。

## 7. Qt 地面站后续实现方案

> **本节是后续开发设计，当前尚未实现。**

### 7.1 设计目标

未来 Qt 地面站应保持 LightDM 和 Ubuntu 桌面正常运行：

```text
启动 Qt 地面站
  -> 后台线程自动连接 RTSP
  -> FFmpeg 软件解码 H.264
  -> 转换为 QImage
  -> Qt 主线程显示最新视频帧
  -> 断线后每 2 秒自动重连
```

视频接收失败不能影响已有的任务控制、状态显示、串口和 ROS 功能。

### 7.2 模块边界

在 `warehouse_ws/src/drone_qt` 中增加独立的 `RtspVideoReceiver`。它只负责：

- RTSP 网络连接。
- 选择 H.264 视频流。
- H.264 软件解码。
- YUV 到 RGB888 的格式转换。
- 保存最新 `QImage`。
- 上报连接、断开和错误状态。
- 自动重连和资源释放。

它不能直接操作 Qt 界面控件。Qt 主线程负责定期获取最新帧并更新视频控件。

### 7.3 为什么首版使用 FFmpeg 软件解码

当前视频只有 `320x240@30`，软件解码负担相对较低。FFmpeg 软件解码可以直接获得 `AVFrame`，再转换为 Qt 可使用的 RGB 图像，实现路径清晰。

RDK 硬件解码更适合更高分辨率或 CPU 性能不足的情况，但将厂商解码缓冲区重新取回、处理 NV12 缓冲区生命周期并转换为 `QImage` 更复杂。首版完成后应实测 CPU、延迟和稳定性，再决定是否增加硬件解码后端。

### 7.4 FFmpeg 解码流程

计划采用：

```text
avformat_network_init
  -> avformat_open_input
  -> avformat_find_stream_info
  -> av_find_best_stream
  -> avcodec_find_decoder
  -> avcodec_alloc_context3
  -> avcodec_parameters_to_context
  -> avcodec_open2
  -> av_read_frame
  -> avcodec_send_packet
  -> avcodec_receive_frame
  -> sws_scale(RGB888)
  -> QImage::copy
```

打开 RTSP 时设置：

```text
rtsp_transport=tcp
stimeout=<有限超时>
```

TCP 是当前热点测试的默认选择。未来可以保留 `udp` 参数用于专用无线链路对比，但不应在源码中写死。

### 7.5 线程模型

网络和解码循环必须位于独立工作线程：

```text
Qt 主线程
  ├── 控制、状态、ROS 和界面事件
  └── 每约 33 ms 读取并显示最新帧

RTSP 工作线程
  ├── 连接 MediaMTX
  ├── 读取 H.264 包
  ├── 解码和颜色转换
  ├── 覆盖最新帧缓冲区
  └── 断线后释放资源并重连
```

不能在 Qt 主线程调用 `avformat_open_input` 或 `av_read_frame`，否则网络延迟会让整个地面站界面卡死。

### 7.6 最新帧缓冲

地面站是实时显示系统，应优先显示最新画面，而不是完整播放所有历史帧。

采用单帧缓冲：

1. 工作线程解码出新帧。
2. 用新帧覆盖旧帧。
3. Qt 主线程按固定周期取出当前最新帧。
4. Qt 暂时变慢时直接丢弃中间旧帧。

不要为每一帧无限发送 queued signal，否则事件队列积压后会出现“画面流畅但延迟越来越大”。

### 7.7 QImage 内存安全

FFmpeg 的 `AVFrame` 和转换缓冲区会在后续解码中重复使用。构造 `QImage` 后必须进行深拷贝，例如通过 `QImage::copy()` 让 Qt 图像拥有自己的内存。

工作线程写入最新帧、主线程读取最新帧时，需要使用互斥锁或等效同步方式保护帧对象。持锁期间只交换图像引用，不执行缩放和界面刷新。

### 7.8 自动重连

以下情况都应进入重连流程：

- 发送端尚未启动。
- 发送端重启。
- Wi-Fi 短时中断。
- 发送端 IP 错误。
- `av_read_frame` 返回网络错误或流结束。

重连步骤：

1. 标记视频状态为“连接中断”。
2. 释放 `AVPacket`、`AVFrame`、`SwsContext`、`AVCodecContext` 和 `AVFormatContext`。
3. 等待 `2000 ms`，期间持续检查程序退出标志。
4. 使用同一 RTSP 地址重新连接。
5. 恢复后替换最新视频帧并更新状态。

FFmpeg 应配置 interrupt callback。Qt 退出时设置原子退出标志，以便打断阻塞的连接或读取操作，然后等待工作线程结束。

### 7.9 后续构建依赖

未来 `drone_qt` 需要链接：

```text
libavformat
libavcodec
libavutil
libswscale
```

Ubuntu 对应开发包通常为：

```text
libavformat-dev
libavcodec-dev
libavutil-dev
libswscale-dev
pkg-config
```

后续实现时需要同步修改 `drone_qt/CMakeLists.txt` 和 `drone_qt/package.xml`。本阶段不修改这些文件。

### 7.10 RTSP 地址如何传给 Qt 地面站

不把测试 IP 写入 C++ 源码，也暂不增加接收端 YAML。建议由 `run_ground_station.launch.py` 提供运行参数：

```text
video_url
video_transport
```

未来计划启动命令：

```bash
cd /home/sunrise/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch drone_bringup run_ground_station.launch.py \
  video_url:=rtsp://<发送端实际IP>:8554/d435i \
  video_transport:=tcp
```

预期行为：

- `video_url` 非空：启动后自动连接，断线每 2 秒重试。
- `video_url` 为空：不启动视频线程，地面站其他功能正常使用。
- 修改发送端 IP 时只改启动参数，不重新编译 Qt。

**这些 Launch 参数当前尚不存在，上述命令目前不能直接用于显示视频。** 必须先完成本节描述的 Qt 解码代码和构建配置。

## 8. 未来香橙派 AP 组网

未来香橙派只负责提供 Wi-Fi AP：

```text
香橙派 AP
  ├── 发送端 RDK：D435i、编码器、MediaMTX
  └── 接收端/Qt 地面站：RTSP 客户端
```

香橙派不负责 D435i 采集、H.264 编码或 MediaMTX。当前地址属于手机热点网络，换成香橙派 AP 后不保证继续有效。

两端连接香橙派 AP 后分别执行：

```bash
ip -4 -brief address
ip route
```

确认发送端在香橙派 AP 网络中的实际 IPv4 地址，然后从接收端测试：

```bash
ping -c 4 <发送端在香橙派AP中的实际IP>
```

最终 RTSP 地址：

```text
rtsp://<发送端在香橙派AP中的实际IP>:8554/d435i
```

地址变化规则：

- `192.168.46.253`：当前手机热点测试 IP，未来替换。
- `8554`：MediaMTX RTSP 端口，除非发送端 YAML 主动修改，否则保持不变。
- `d435i`：流路径，除非发送端 YAML 主动修改，否则保持不变。

香橙派系统版本、AP 软件、DHCP 网段和网卡接口确定后，再通过 DHCP 地址保留固定发送端 IP。不能直接沿用或猜测当前手机热点地址。

## 9. 验证与排障

### 9.1 发送端没有 D435i RGB 节点

```bash
lsusb
lsusb -t
ls -l /dev/d435i_color
readlink -f /dev/d435i_color
v4l2-ctl --device=/dev/d435i_color --get-fmt-video
```

- 没有 D435i：检查 USB 供电、线缆和 USB 3.0 连接。
- 没有 `/dev/d435i_color`：检查 udev 规则和相机序列号。
- 显示 `Z16`：当前别名指向深度节点，需要重新确认 RGB 接口。

### 9.2 MediaMTX 没有监听

在发送端执行：

```bash
ss -lntp | grep :8554
ps -ef | grep -E '[m]ediamtx|[r]dk_d435i_h264_sender'
```

如果提示 `TCP port 8554 is already in use`，应先识别具体占用进程。不要无条件结束系统中所有 FFmpeg 或媒体进程。

### 9.3 地面站不能访问 RTSP

```bash
ping -c 4 192.168.46.253

ffprobe -v error \
  -rtsp_transport tcp \
  -show_entries stream=codec_name,width,height,r_frame_rate \
  -of default=noprint_wrappers=1 \
  rtsp://192.168.46.253:8554/d435i
```

检查顺序：

1. 发送端启动脚本是否仍在运行。
2. 地面站使用的是否为发送端当前 IP。
3. URL 端口是否与发送端 YAML 一致。
4. URL 路径是否与发送端 YAML 一致。
5. TCP 连接失败时先排查网络和端口，不要直接归因于 Qt 或解码器。

### 9.4 Qt 后续实现的验收标准

Qt 解码功能完成后至少验证：

1. LightDM 保持运行，Qt 地面站和视频可以同时显示。
2. RTSP 识别为 H.264 `320x240@30`。
3. 连续运行 30 分钟，CPU 和内存稳定。
4. 停止发送端时 Qt 界面不卡死，其他功能仍可操作。
5. 重新启动发送端后，视频在重连周期内自动恢复。
6. 输入错误 IP 时持续重试，但不影响 ROS 和任务控制。
7. Qt 退出后不遗留视频线程或 RTSP 连接。
8. 画面延迟不会因运行时间增长而持续累积。

## 10. 当前状态总结

| 项目 | 状态 |
|---|---|
| D435i RGB `320x240@30` 采集 | 已实现 |
| RDK H.264 硬编码，5 Mbit/s，GOP 15 | 已实现 |
| MediaMTX RTSP 发布 | 已实现 |
| `rtsp://192.168.46.253:8554/d435i` 测试接口 | 已验证 |
| 发送端一键启动脚本 | 已实现 |
| Qt 地面站 RTSP 连接 | 未实现 |
| Qt 地面站 H.264 解码 | 未实现 |
| Qt 地面站自动重连 | 未实现 |
| 香橙派 AP 最终 IP 规划 | 等待实际组网后确定 |

当前可以启动发送端并用 FFprobe 或 FFplay 验证视频流。下一开发阶段需要在 `warehouse_ws/src/drone_qt` 中实现第 7 节的接收和解码模块，Qt 地面站才能直接显示实时图传。
