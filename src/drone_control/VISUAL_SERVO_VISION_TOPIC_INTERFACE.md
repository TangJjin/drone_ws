# 视觉伺服视觉端 Topic 接口

## 1. 视觉端发布目标信息

```text
Topic: /vision/servo/target
Type:  drone_msgs/msg/VisionServoTarget
QoS:   SensorDataQoS（best effort）
频率:  建议 20-30 Hz，最低 15 Hz
```

消息内容：

| 字段 | 类型 | 填写内容 |
|---|---|---|
| `stamp` | Time | 当前图像采集或推理完成时间 |
| `sequence` | uint32 | 递增帧号 |
| `target_id` | string | 稳定目标 ID，一次伺服期间不能随意改变 |
| `valid` | bool | 检测到可用目标时为 `true` |
| `confirmed` | bool | 目标经过多帧确认后为 `true` |
| `confidence` | float64 | 检测置信度，建议范围 `[0,1]` |
| `error_x` | float64 | 目标中心的水平归一化误差 |
| `error_y` | float64 | 目标中心的垂直归一化误差 |
| `image_width` | uint32 | 图像宽度，单位 px |
| `image_height` | uint32 | 图像高度，单位 px |
| `center_x` | float64 | 目标中心横坐标，单位 px |
| `center_y` | float64 | 目标中心纵坐标，单位 px |

控制端实际使用以下字段进行视觉伺服：

```text
target_id
valid
confirmed
error_x
error_y
```

### 归一化误差计算

不能直接把像素误差填入 `error_x/error_y`，必须使用：

```text
error_x = (center_x - image_width  / 2) / (image_width  / 2)
error_y = (center_y - image_height / 2) / (image_height / 2)
```

结果限制在 `[-1,1]`，方向定义为：

```text
error_x > 0：目标在图像右侧
error_x < 0：目标在图像左侧
error_y > 0：目标在图像下方
error_y < 0：目标在图像上方
```

### 检测到目标

```yaml
sequence: 101
target_id: "target_1"
valid: true
confirmed: true
confidence: 0.92
error_x: 0.18
error_y: -0.06
image_width: 640
image_height: 480
center_x: 377.6
center_y: 225.6
```

### 没有检测到目标

不要重复上一帧有效坐标，也不要停止发布。继续按原频率发布：

```yaml
sequence: 102
target_id: "target_1"
valid: false
confirmed: false
confidence: 0.0
error_x: 0.0
error_y: 0.0
image_width: 640
image_height: 480
center_x: 0.0
center_y: 0.0
```

## 2. 视觉端订阅控制状态

```text
Topic: /control/vision_servo/status
Type:  drone_msgs/msg/VisionServoStatus
QoS:   reliable + transient_local
```

视觉端主要读取：

| 字段 | 处理方式 |
|---|---|
| `active` | `true` 时开始发布伺服目标，`false` 时结束本次伺服输出 |
| `requested_target_id` | 非空时选择指定目标；为空时自行选择一个稳定目标 |
| `tracked_target_id` | 控制端锁定的目标 ID；非空后持续发布同一目标 |

## 3. 配合流程

```text
1. 收到 active=true。
2. 根据 requested_target_id 选择目标。
3. 持续发布 /vision/servo/target。
4. tracked_target_id 非空后，保持发布同一目标 ID。
5. 目标丢失时发布 valid=false，不要重复旧坐标。
6. 收到 active=false 后停止本次视觉伺服输出。
```

注意：控制端状态为 `aligned` 时仍要继续发布目标，直到 `active=false`。
