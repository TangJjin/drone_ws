# 工业相机 Animal Vision 控制端交付

## 接口

工业相机节点保持原有启动方式，新增唯一的结构化检测输出：

```text
topic: /animal_vision/detections
type:  drone_msgs/msg/AnimalDetections
QoS:   reliable, keep last 10
```

该接口与 `/k230/animals/*` 无关；控制端接入时不要同时把两个来源当成同一套目标数据。

## 消息语义

`AnimalDetections` 表示一帧工业相机的全部有效 YOLO 检测结果：

- `stamp`：视觉节点发布该帧的 ROS 时间。
- `frame_seq`：工业相机节点内部单调递增的采集帧号。并行推理导致较旧帧晚返回时，节点会按帧号顺序发布已完成结果。
- `image_width`、`image_height`：所有像素坐标的源图尺寸。
- `target_count`：必须等于 `targets.size()`；`0` 表示本帧没有通过置信度阈值和 NMS 的目标。

每个 `AnimalDetection` 包含：

- `label`：YOLO 类别名称。
- `track_id`：视觉节点运行期间不复用的稳定目标 ID。控制端应以该字段区分同类目标，不能按数组下标或类别名称区分。
- `score`：YOLO 置信度。
- `cx`、`cy`：目标框中心的源图像像素坐标，图像左上角为 `(0, 0)`。
- `x1`、`y1`、`x2`、`y2`：目标框边界；`x2/y2` 为右/下排他边界，因此 `bbox_w = x2 - x1`、`bbox_h = y2 - y1`。
- `err_x = cx - image_width / 2`、`err_y = cy - image_height / 2`，单位为像素。`err_x > 0` 表示目标在相机中心右侧，`err_y > 0` 表示目标在相机中心下方。
- `norm_x`、`norm_y`：分别为 `err_x / (image_width / 2)` 和 `err_y / (image_height / 2)`。

## 同类目标 ID 规则

视觉节点仅在相同类别内按检测框 IoU 关联前后帧：

1. 当前检测框与历史轨迹 IoU 不小于 `track_iou_threshold` 时，沿用历史 `track_id`。
2. 未匹配检测创建新轨迹，获取递增的 `track_id`。
3. 目标在一帧未检出时不会进入该帧 `targets`，但轨迹会在 `track_max_missed_frames` 内保留；在窗口内重新匹配时继续使用原 ID。
4. 超过该窗口的轨迹删除；之后出现的目标获得新的 ID。已删除 ID 在节点重启前不会重新分配。

默认参数：`track_iou_threshold=0.3`，`track_max_missed_frames=15`。可通过工业相机节点参数覆盖。

## 控制端接入要求

当前版本只负责检测结果发布，不包含区域门控、航线扫描、拍照命令或记录结果闭环。当前仓库控制端尚未订阅该话题，需后续迁移后才会实际消费消息。

接入时应：

1. 订阅 `/animal_vision/detections`，先校验 `target_count == targets.size()`。
2. 在同一帧内要求 `track_id` 唯一；使用 `track_id` 维护目标状态。
3. 使用 `err_x/err_y` 或 `norm_x/norm_y` 做相机对准时，遵循上述正负方向。
4. 目标暂时不在数组中不等同于新目标；在轨迹保留窗口内重新出现时仍会携带原 ID。

## 代码改造清单

本次实际修改范围：

~~~text
src/drone_msgs/msg/AnimalDetection.msg
src/drone_msgs/msg/AnimalDetections.msg
src/drone_msgs/CMakeLists.txt
src/drone_perception/CMakeLists.txt
src/drone_perception/src/industrial_animal_vision_node.cpp
src/drone_perception/config/industrial_default.yaml
src/drone_perception/launch/industrial_animal_vision.launch.py
~~~

本次没有修改：

~~~text
drone_control 的 ActionExecutor
工业相机节点名 industrial_animal_vision
工业相机可执行文件 industrial_animal_vision_node
工业相机 launch 和 YAML 文件名
GStreamer 相机采集管线
RknnYoloDetector 的 YOLO 推理、NMS 和坐标反变换
K230 节点、K230 话题和 K230 消息
~~~

AnimalDetection 表示单个目标，AnimalDetections 表示一帧目标列表。两个新消息已经在 drone_msgs 的 rosidl_generate_interfaces 中注册。工业相机 CMake target 也已增加 drone_msgs 依赖。

## 工业相机节点完整运行流程

### 1. 节点初始化

入口保持不变：

~~~text
main.cpp
  -> makeIndustrialAnimalVisionNode()
  -> IndustrialAnimalVisionNode
~~~

构造函数按以下顺序执行：

~~~text
1. declareParameters：声明相机、模型、显示、检测发布和跟踪参数
2. readParameters：读取 launch/YAML 参数
3. validateParameters：检查参数合法性
4. configureProcessAffinity：设置 CPU 亲和性
5. configureCameraControls：写入 V4L2 相机控制参数
6. startPipeline：启动 v4l2src -> MPP JPEG 解码 -> appsink
7. initializeDetectors：初始化三个 RKNN detector，分别使用 NPU Core0/1/2
8. 创建 /animal_vision/detections 发布器
9. 创建检测发布线程
10. 创建三个 RKNN worker
11. 创建相机采集线程
12. display_enabled=true 时创建 OpenCV 显示线程
~~~

发布器类型是 rclcpp::Publisher<drone_msgs::msg::AnimalDetections>，QoS 是 KeepLast(10) + reliable。默认话题是 /animal_vision/detections，可通过 detections_topic 参数覆盖。

### 2. 相机采集

采集线程是 captureLoop，实际数据流：

~~~text
工业相机 /dev/videoX
  -> v4l2src 输出 MJPEG
  -> jpegparse
  -> mppjpegdec 解码为 RGB
  -> appsink
  -> captureLoop 获取 GstSample
  -> 分配 frame_id
  -> 放入 RKNN 任务队列
~~~

每个有效采集帧分配从 1 开始递增的 frame_id，最终写入 AnimalDetections.frame_seq。它是采集帧序号，不是目标数量，也不是目标 ID。

RKNN 任务队列容量固定为 3。采集速度高于推理速度时，采集线程丢弃最旧待推理帧以降低延迟，并将该 frame_id 标记为 skipped，避免后续发布线程永远等待不存在的结果。

### 3. 三个 RKNN worker 推理

三个 worker 在 workerLoop 中并行调用 processTask：

~~~text
FrameTask
  -> GstVideoFrame 映射
  -> RGB cv::Mat
  -> RknnYoloDetector::infer(rgb)
  -> std::vector<Detection>
  -> 检测结果队列
~~~

RknnYoloDetector 已经完成：

~~~text
RGB 预处理和 letterbox
  -> RKNN 推理
  -> 置信度筛选
  -> 按类别 NMS
  -> 模型坐标反变换到 RGB 原图
  -> 生成 Detection.class_id / score / box / center
~~~

本次不改变这些 YOLO 结果。发布侧直接使用 Detection.box 和 Detection.center，它们已经是 decode_width x decode_height 的 RGB 原图坐标。

worker 只把 frame_id、图像宽高、Detection 数组和 class label 放入检测结果队列。worker 不直接更新 track_id，因为三个 RKNN worker 的完成顺序可能与相机采集顺序不同。

### 4. 帧排序和单线程发布

detectionPublishLoop 是唯一进行目标跟踪和 ROS 发布的线程。它维护：

~~~text
pending_detection_frames_
  已完成推理、等待按顺序发布的结果：
  frame_id -> DetectionFrame

skipped_detection_frames_
  已知不会产生结果的帧：
  队列丢弃、RGB 信息无效、Gst 映射失败或推理异常

next_detection_frame_id_
  当前等待处理的最小 frame_id
~~~

规则：

1. 下一个帧号已推理完成：取出并发布。
2. 下一个帧号属于 skipped：跳过并推进序号。
3. 较新的帧先完成，但较旧帧既未完成也未 skipped：等待较旧帧。
4. 对取出的帧执行 IoU 跟踪、生成 AnimalDetections 并发布。

所以已发布 frame_seq 按采集帧号递增；任务队列丢帧或推理失败时，序号可以有间隔。

空检测帧与 skipped 帧不同：

~~~text
YOLO 正常执行但无目标：
  仍发布 AnimalDetections
  target_count = 0
  targets = []

帧被丢弃或推理失败：
  不发布该帧消息
  frame_seq 直接跳到后续帧
~~~

### 5. 同类目标 track_id 分配

单线程发布阶段维护 tracked_detections。每条内部轨迹包含：

~~~text
track_id
class_id
上一帧匹配成功的 bbox
last_seen_frame_id
~~~

关联步骤：

1. 只在相同 class_id 的当前框和历史框之间计算 IoU。
2. IoU 小于 track_iou_threshold 的候选直接丢弃，默认阈值 0.3。
3. 候选按 IoU 从高到低排序。
4. 贪心匹配，一个当前框只对应一个历史轨迹，一个历史轨迹也只对应一个当前框。
5. 匹配成功：更新历史框和 last_seen_frame_id，沿用历史 track_id。
6. 未匹配当前框：分配 next_track_id，然后计数器递增。
7. 历史轨迹超过 track_max_missed_frames 未匹配时删除，默认是 15 个采集帧。

两个同类物体会有不同 track_id。例如：

~~~text
elephant A -> track_id: 1
elephant B -> track_id: 2
~~~

当它们在相邻帧正常移动且框 IoU 满足阈值时，数组内先后顺序可以变化，但 ID 应保持不变。

这是基于检测框 IoU 的轻量跟踪，不是 ReID 外观重识别。两个同类目标长时间遮挡、快速交叉或大幅跳动时，ID 仍可能交换；控制端需要保留连续帧判断和超时处理。

### 6. 短暂漏检和 ID 生命周期

保留只保留内部轨迹，不发布虚假的目标：

~~~text
帧 100：target 7 被检测到并发布
帧 101：target 7 漏检
  -> 本帧 targets 中没有 target 7
  -> 内部还保留 track_id=7 的轨迹
帧 105：同一物体重新检测到，IoU 达标
  -> 恢复使用 track_id=7
帧 116：仍没有重新检测到，超过默认窗口
  -> 删除 track_id=7 的内部轨迹
帧 130：该物体再次出现
  -> 分配新的、更大的 track_id
~~~

窗口按 frame_id 差值计算，不按墙上时间计算。控制端若需要目标丢失超时，应使用 stamp 或 frame_seq 自行维护，感知端不会在漏检期间发布预测坐标。

next_track_id 从 1 递增。删除后的 ID 不回收，节点重启前不复用；节点重启后从 1 重新开始。因此 track_id 只保证单次节点运行期间有效，不是跨重启的永久数据库 ID。

### 7. 消息组装

每个当前检测转换为 AnimalDetection：

~~~text
label      = detector 类别名称
track_id   = IoU 跟踪结果
score      = Detection.score

cx         = Detection.center.x
cy         = Detection.center.y

x1         = Detection.box.x
y1         = Detection.box.y
bbox_w     = Detection.box.width
bbox_h     = Detection.box.height
x2         = x1 + bbox_w
y2         = y1 + bbox_h
bbox_area  = bbox_w * bbox_h
~~~

误差计算：

~~~text
image_center_x = image_width / 2
image_center_y = image_height / 2

err_x = cx - image_center_x
err_y = cy - image_center_y

norm_x = err_x / (image_width / 2)
norm_y = err_y / (image_height / 2)
~~~

方向约定：

~~~text
err_x > 0：目标在图像中心右侧
err_x < 0：目标在图像中心左侧
err_y > 0：目标在图像中心下方
err_y < 0：目标在图像中心上方
~~~

外层 AnimalDetections 组装规则：

~~~text
stamp        = 发布时的 ROS 时间
frame_seq    = 采集 frame_id
image_width  = RGB 源图宽度
image_height = RGB 源图高度
target_count = targets.size()
targets      = 当前帧全部通过现有置信度阈值和 NMS 的目标
~~~

## 参数说明

~~~text
detections_topic
  类型：string
  默认：/animal_vision/detections
  含义：检测列表发布话题。

track_iou_threshold
  类型：double
  默认：0.3
  范围：[0, 1]
  含义：同类当前检测框与历史框的最小 IoU 匹配阈值。
  提高：ID 更谨慎，但快速移动时更容易生成新 ID。
  降低：ID 更容易延续，但相邻同类目标更容易错误匹配。

track_max_missed_frames
  类型：int
  默认：15
  范围：>= 0
  含义：轨迹允许未匹配的最大采集帧数。
  设为 0：下一帧不匹配就删除旧轨迹。
  增大：更能应对短暂遮挡，但旧轨迹保留更久。
~~~

可通过现有 launch 覆盖，例如：

~~~bash
ros2 launch drone_perception industrial_animal_vision.launch.py \
  track_iou_threshold:=0.35 \
  track_max_missed_frames:=20
~~~

## 控制端接入建议

控制端未来订阅 /animal_vision/detections 后，每帧建议：

1. 检查 image_width 和 image_height 大于 0。
2. 检查 target_count 等于 targets.size。
3. 检查当前帧内 track_id 不重复。
4. 检查 bbox_w 等于 x2 - x1，bbox_h 等于 y2 - y1，且宽高大于 0。
5. 使用 track_id 建立控制端目标状态，不能用数组下标或 label 作为唯一键。
6. 对准时使用当前帧 err_x/err_y 或 norm_x/norm_y。
7. 目标不在当前数组时，不应把误差当成 0；应按控制端自身超时规则等待、保持或结束控制。

当前没有实现：

~~~text
ActionExecutor 订阅新话题
相机对准 PID
无人机 setpoint 控制
capture_ready / record_result
区域或航线扫描点门控
深度采样、相机距离、三维世界坐标
~~~

因此，新接口只回答：

~~~text
当前 RGB 图像识别到什么
每个目标的类别、置信度和像素框在哪里
同类多个目标如何区分
每个目标相对图像中心的像素偏差是多少
~~~

它不回答目标离相机多少米、目标世界坐标、目标所属区域或无人机是否已经对准。

## 验收和构建状态

查看接口：

~~~bash
source install/setup.bash
ros2 interface show drone_msgs/msg/AnimalDetection
ros2 interface show drone_msgs/msg/AnimalDetections
~~~

运行工业相机节点后查看话题：

~~~bash
source install/setup.bash
ros2 topic list | rg animal_vision
ros2 topic info /animal_vision/detections -v
ros2 topic echo /animal_vision/detections
~~~

预期：

~~~text
话题类型：drone_msgs/msg/AnimalDetections
target_count 始终等于 targets 数量
同类目标在同一帧拥有不同 track_id
无目标有效推理帧发布 target_count=0
队列丢帧或推理失败时 frame_seq 可能不连续
短暂遮挡恢复的目标可能沿用原 track_id
超出丢失窗口后重新出现的目标得到新的、更大的 track_id
~~~

已执行构建：

~~~bash
colcon build --packages-select drone_msgs drone_perception --symlink-install \
  --event-handlers console_direct+
~~~

drone_msgs 和非 RKNN 的 drone_perception 目标已构建成功。当前机器缺少 RKNN_INCLUDE、RKNN_LIB/librknnrt.so 或完整 GStreamer 开发环境，CMake 因而跳过 industrial_animal_vision_node 的实际编译。请在目标 Orange Pi/RK3588 环境补齐依赖后，重新构建并进行相机实机验证。
