# Air-Ground CameraInfo 与手工像素 XYZ 设计

## 目标

在现有 `air_ground_servo_node` 的 RGBD 调试链路上，使用手工配置像素
`(center_x, center_y)`、该位置的稳健深度中值和 RGB 相机内参，计算并显示
RGB 光学坐标系下的三维点 `(X, Y, Z)`。本阶段不加入靶标检测、YOLO、光流、
滤波、机体外参或控制消息。

## 数据来源

- 图像、对齐深度和相机内参均取自同一个
  `realsense2_camera_msgs/msg/RGBD` 消息。
- 只使用消息中的 `rgb_camera_info`，不得使用 `depth_camera_info` 反投影
  RGB 像素。
- 继续使用当前 10x10 深度窗口中值作为轴向深度 `Z`。
- `rgb_camera_info.width/height` 必须与 RGB 图像尺寸一致。

直接使用 RGBD 内置内参可以保证内参与当前 RGB 帧来自同一个组合消息，避免
增加独立 CameraInfo 订阅和额外的同步状态。

## 坐标定义与计算

使用 `image_geometry::PinholeCameraModel` 读取 `rgb_camera_info` 并反投影：

```text
ray = projectPixelTo3dRay(u, v)
point = ray * Z
```

等价针孔模型为：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth_m
```

坐标遵循 RGB optical frame：图像右侧为 `+X`，图像下方为 `+Y`，相机前方
为 `+Z`。发布消息的 `frame_id` 使用 `rgb_camera_info.header.frame_id`。

## 有效性规则

只有全部满足以下条件时，XYZ才有效：

1. RGB与aligned depth尺寸一致且深度编码受支持。
2. RGB CameraInfo宽高与RGB图像一致。
3. `fx`、`fy`为有限正数，`cx`、`cy`为有限数。
4. 手工像素位于图像内。
5. 深度窗口存在至少一个范围内有效样本。
6. 反投影结果三个分量均为有限数。

任一条件失败时，HUD显示 `XYZ: invalid`，日志给出限流原因，并且不发布
`PointStamped`，禁止用 `(0, 0, 0)`代替无效测量。

## 输出与HUD

新增参数：

- `point_topic`：有效三维点输出话题，默认
  `/air_ground_servo/manual_point`。

发布 `geometry_msgs/msg/PointStamped`，时间戳使用 RGB 图像时间戳。

调试界面保留FPS、分辨率、采样框和深度，并增加以下关键参数：

- `Pixel: u, v`与采样窗口尺寸。
- `Intrinsics: fx, fy, cx, cy`。
- `CameraInfo: width x height`和状态。
- `Frame: rgb optical frame_id`。
- `XYZ: X, Y, Z m`或`XYZ: invalid`。
- `Depth valid: 有效样本数/总样本数`。

HUD使用紧凑字号和扩展背景区域，不遮挡图像中心采样框。长 `frame_id`允许截断
显示，但发布消息保留完整内容。

## 测试

新增可独立测试的投影函数，先编写失败测试，再实现：

- 主点像素在1米深度时得到 `(0, 0, 1)`。
- 像素向右、向下偏移时，`X`、`Y`分别为正。
- 偏移到左、上时，`X`、`Y`分别为负。
- 无效焦距、尺寸不匹配、无效深度返回无效结果。
- 结果不得包含NaN或Inf。
- YAML缺失或错误类型的 `point_topic`在launch阶段明确失败。

## Orange Pi 验收

1. HUD持续显示CameraInfo尺寸、内参、frame_id和XYZ，不闪烁为未初始化。
2. `ros2 topic hz /air_ground_servo/manual_point`在有效深度下接近RGBD帧率。
3. 相机正前方主点附近，`X/Y`接近0，`Z`接近卷尺距离。
4. 在同一深度把采样点移到画面右/左/下/上，XYZ符号分别正确。
5. 使用0、正负10 cm、正负20 cm真值点比较，记录误差而不是仅凭画面判断。
6. 遮挡深度或制造无效深度后HUD显示invalid，输出话题停止发布新点。
7. 恢复有效深度后无需重启即可恢复发布。
