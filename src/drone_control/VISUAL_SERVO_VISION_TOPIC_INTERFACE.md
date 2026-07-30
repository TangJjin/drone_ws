# 视觉伺服话题接口

## 视觉端输出

```text
Topic: /vision/servo/target
Type:  drone_msgs/msg/VisionServoTarget
QoS:   SensorDataQoS (best effort)
```

新版 `air_ground_servo_node` 每帧发布一次消息。控制端只读取以下字段：

- `valid`：视觉、深度和反投影结果是否有效。
- `confirmed`：当前有效帧是否已确认。
- `error_x`：相机画面向右为正的米制偏差。
- `error_y`：相机画面向下为正的米制偏差。

无效帧中 `valid/confirmed` 均为 `false`。控制端不会使用该帧的误差，
也不会复用之前的有效坐标。`target_id`、`sequence` 及其他兼容字段不参与
当前 D435i 视觉伺服目标筛选。

## 控制端步长

`error_x/error_y` 直接作为米制偏差，不进行 `[-1, 1]` 截断。
现有 `image_x_axis/image_y_axis` 和对应 sign 负责把画面方向映射到机体系方向。

PID 输出为机体系速度，再按照控制周期换算为位置步长：

```text
body_step = clamp(PID(metric_offset), max_body_speed_mps) * dt
```

默认最大速度为 `0.20 m/s`；控制频率为 20 Hz 时，单轴单次步长最大约
`0.01 m`。默认进入/退出容差 `0.04/0.07` 分别表示 4 cm 和 7 cm。

## 控制状态输出

```text
Topic: /control/vision_servo/status
Type:  drone_msgs/msg/VisionServoStatus
QoS:   reliable + transient_local
```
