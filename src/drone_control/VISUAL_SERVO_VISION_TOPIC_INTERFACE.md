# 视觉伺服话题接口

## 视觉端输出

```text
Topic: /air_ground_servo/target_point
Type:  geometry_msgs/msg/PointStamped
QoS:   SensorDataQoS (best effort)
```

坐标单位为米，坐标系来自 `header.frame_id`，当前视觉节点输出
`camera_color_optical_frame`：

- `point.x`：靶标相对相机光轴向画面右侧的偏移，向右为正。
- `point.y`：靶标相对相机光轴向画面下方的偏移，向下为正。
- `point.z`：相机到靶标、沿相机光轴向前的深度。

视觉节点只在靶标识别、深度取样和 CameraInfo 反投影都有效时发布，
不会发布无效占位消息。控制端以消息到达时间判断数据新鲜度；停止收到新消息后，
按视觉伺服的 `lost_timeout_s` 进入失目标保护。

## 控制端用法

控制端使用 `point.x/y` 作为米制横向偏差，不使用 `point.z` 控制前后距离。
偏差不会再按 `[-1, 1]` 截断。现有 `image_x_axis/image_y_axis` 和对应 sign
负责把相机画面方向映射到机体系方向，实飞前必须确认相机安装方向与配置一致。

PID 输出为机体系速度，随后按控制周期换算为位置步长：

```text
body_step = clamp(PID(metric_offset), max_body_speed_mps) * dt
```

默认 `max_body_speed_mps: 0.20`；控制频率为 20 Hz 时，单轴单次位置步长最大约
`0.01 m`。默认进入/退出容差 `0.04/0.07` 分别表示 4 cm 和 7 cm。

## 控制状态输出

```text
Topic: /control/vision_servo/status
Type:  drone_msgs/msg/VisionServoStatus
QoS:   reliable + transient_local
```

该话题仍由控制端发布，用于观察视觉伺服是否处于等待、跟踪、对准、成功或超时状态。
新版 `air_ground_servo_node` 不依赖该状态话题才发布坐标。
