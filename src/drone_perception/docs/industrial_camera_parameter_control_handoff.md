# 工业相机地面站参数控制交接说明

本文是给地面站开发人员的接口说明。视觉端只订阅一条完整的相机参数消息并写入
工业相机；视觉端不保存参数、不恢复默认、不发布相机状态。

## 1. 最重要的约定

只有一个控制话题：

```text
话题名：/industrial_camera/params
消息类型：drone_msgs/msg/IndustrialCameraParams
方向：地面站发布 -> industrial_animal_vision_node 订阅 -> 工业相机
QoS：reliable + transient_local，深度 1
```

通信流程只有一条：

```text
地面站界面修改参数
  -> 地面站更新一套完整参数
  -> 地面站发布完整 IndustrialCameraParams
  -> 视觉端写入工业相机
```

没有 `command`、`state`、`capabilities` 话题，也没有 `request_id`、`update_mask`。
视觉端不会把相机实际值发回地面站。地面站界面显示的值由地面站自己维护。

## 2. 地面站必须维护的两套数据

地面站内部必须保存两份**完整的** `IndustrialCameraParams` 数据。两份数据的存储
方式由地面站决定，可以写在程序中，也可以使用地面站自己的配置文件；视觉端不会
读取、写入或管理它们。

| 数据 | 用途 | 什么时候使用 |
| --- | --- | --- |
| 默认参数 | 出厂/项目推荐的一整套值 | 用户点击“恢复默认”时，复制到当前参数并发布。 |
| 已保存参数 | 用户最后一次点击“保存”时的整套值 | 地面站启动时加载；用户点击“保存”时覆盖。 |

地面站运行时还应有一份“当前参数”。用户修改任意一个滑块或圆点时，只改当前参数
中的对应字段，然后发布**完整当前参数**。不能只发一个 `gain` 或只发一个
`exposure_absolute`。

建议启动流程：

```text
地面站启动
  -> 读取已保存参数；没有已保存参数时使用默认参数
  -> 用该完整参数刷新全部界面控件
  -> 发布一次 /industrial_camera/params
```

`transient_local` 的作用是：地面站先发布过一次后，即使视觉节点稍后启动，视觉节点
也会立刻收到最后一条完整参数。视觉端启动时仍会暂时使用它自己的
`industrial_default.yaml` 参数；收到地面站消息后会被整套覆盖。

## 3. 消息定义

消息类型：`drone_msgs/msg/IndustrialCameraParams`

```text
bool auto_exposure
int32 exposure_absolute
bool auto_exposure_priority
int32 gain
int32 brightness
int32 contrast
int32 saturation
int32 gamma
int32 sharpness
int32 backlight_compensation
bool auto_white_balance
int32 white_balance_temperature
uint8 power_line_frequency
bool auto_focus
int32 focus_absolute
int32 zoom_absolute
```

每个字段必须填写，不能依赖上一条消息中的字段值。

## 4. 参数表、控件和范围

以下范围来自当前 Orange Pi 上连接的工业相机。地面站应按此表创建滑块和圆点。
视觉端也会再次按照相机驱动实际范围限制数值，防止异常输入损坏相机设置。

| 消息字段 | ROS 类型 | 地面站控件 | 可选值或滑块范围 | 作用 |
| --- | --- | --- | --- | --- |
| `auto_exposure` | `bool` | 自动/手动两个圆点 | `true`=自动，`false`=手动 | 曝光由相机自动计算，或使用手动曝光时间。 |
| `exposure_absolute` | `int32` | 滑块 | `1..10000`，步长 `1` | 手动曝光时间。 |
| `auto_exposure_priority` | `bool` | 开关或两个圆点 | `false/true` | 自动曝光时，是否允许相机降低帧率换取更亮图像。 |
| `gain` | `int32` | 滑块 | `0..190`，步长 `1` | 图像增益；过高会增加噪点。 |
| `brightness` | `int32` | 滑块 | `0..255`，步长 `1` | 图像亮度处理值。 |
| `contrast` | `int32` | 滑块 | `0..128`，步长 `1` | 图像明暗反差。 |
| `saturation` | `int32` | 滑块 | `0..128`，步长 `1` | 图像色彩饱和度。 |
| `gamma` | `int32` | 滑块 | `0..255`，步长 `1` | 中间亮度校正，不等于曝光。 |
| `sharpness` | `int32` | 滑块 | `0..255`，步长 `1` | 图像锐化强度。 |
| `backlight_compensation` | `int32` | 滑块 | `16..160`，步长 `1` | 逆光补偿。 |
| `auto_white_balance` | `bool` | 自动/手动两个圆点 | `true`=自动，`false`=手动 | 白平衡由相机自动计算，或使用手动色温。 |
| `white_balance_temperature` | `int32` | 滑块 | `2800..6500`，步长 `1` | 手动白平衡色温。 |
| `power_line_frequency` | `uint8` | 三个圆点 | `0`=关闭，`1`=50Hz，`2`=60Hz | 防止灯光造成画面闪烁。 |
| `auto_focus` | `bool` | 自动/手动两个圆点 | `true`=自动，`false`=手动 | 对焦由相机自动完成，或使用手动焦点位置。 |
| `focus_absolute` | `int32` | 滑块 | `0..1023`，步长 `1` | 手动焦点位置。 |
| `zoom_absolute` | `int32` | 滑块 | `100..200`，步长 `1` | 变焦值。 |

`pan`、`tilt` 没有放入消息。它们属于镜头/云台物理运动，不属于成像参数调节。

## 5. 自动和手动互斥规则

地面站负责界面灰显和禁用交互。只要自动模式为 `true`，对应手动滑块就必须灰显，
不可拖动；自动模式关闭后，手动滑块恢复可用。

| 自动模式字段 | 值为 `true` 时灰显的滑块 | 值为 `false` 时 |
| --- | --- | --- |
| `auto_exposure` | `exposure_absolute` | 曝光时间可调。 |
| `auto_white_balance` | `white_balance_temperature` | 色温可调。 |
| `auto_focus` | `focus_absolute` | 焦点位置可调。 |

自动模式开启时，地面站仍要在整条消息中带上对应手动值，但视觉端会跳过写入该手动
值。这样地面站可以保留用户上次手动设置的数值；用户切回手动后，只要发布完整消息，
该数值就会写入相机。

## 6. 用户操作流程

### 6.1 调整普通滑块

以增益为例，用户把增益调为 `80`：

```text
当前参数.gain = 80
界面显示 80
用户松开滑块或停止拖动约 100 ms
  -> 发布完整当前参数
```

建议不要每移动一个像素就发布一次消息。拖动期间可以仅更新界面；用户松开滑块后，
或连续 100 ms 未移动时再发布一次。

### 6.2 切换手动曝光

用户选择“手动曝光”并把曝光时间设为 `500`：

```text
当前参数.auto_exposure = false
当前参数.exposure_absolute = 500
启用曝光时间滑块
发布完整当前参数
```

视觉端会先切到手动曝光，再写 `exposure_absolute=500`。

用户选择“自动曝光”时：

```text
当前参数.auto_exposure = true
灰显 exposure_absolute 滑块
发布完整当前参数
```

同样的规则适用于自动白平衡/色温、自动对焦/焦点位置。

### 6.3 保存参数

“保存”按钮只由地面站实现：

```text
当前参数 -> 覆盖地面站自己的已保存参数
```

保存不需要向视觉端发送特殊命令。若用户刚修改过界面但还未触发发布，则保存后应立刻
发布一次完整当前参数。

### 6.4 恢复默认

“恢复默认”按钮只由地面站实现：

```text
地面站默认参数 -> 当前参数 -> 刷新所有界面控件 -> 发布完整当前参数
```

恢复默认不会修改视觉端的 `industrial_default.yaml`，也不会修改视觉端任何文件。是否把
默认参数同时写入地面站“已保存参数”，由地面站产品逻辑决定；推荐只改变当前参数，
用户再次点击“保存”后才覆盖已保存参数。

## 7. 视觉端内部处理逻辑

视觉端收到每条完整消息后，按固定顺序执行：

```text
收到 IndustrialCameraParams
  -> 将三个 bool 自动模式转换为相机控制值
  -> 查询相机是否支持各参数及其实际范围
  -> 先写自动曝光、自动白平衡、自动对焦
  -> 写入其余普通参数
  -> 仅在对应自动模式关闭时写手动曝光、色温、焦点
  -> 每个成功写入值都从相机读回确认
```

视觉端使用的 Linux V4L2 接口只在视觉端内部运行：

| 接口 | 作用 |
| --- | --- |
| `VIDIOC_QUERYCTRL` | 查询相机是否支持该参数、最小值、最大值、步长和当前是否可写。 |
| `VIDIOC_S_CTRL` | 把参数写入相机。 |
| `VIDIOC_G_CTRL` | 从相机读回写入后的实际值，用于确认驱动接受了该值。 |

视觉端对整数滑块会进行范围限制。例如地面站错误发布 `gain=300` 时，视觉端会按相机
当前最大值写入 `190` 并记录日志。但视觉端不回传 `190`；地面站应按照参数表限制
控件，避免发布超范围值。

如果相机不支持某个字段、该字段只读，或驱动写入失败，视觉端只记录日志，不回传
任何 ROS 消息。地面站不应等待确认消息。

## 8. 命令行联调例子

下面是一整条消息，切换为手动曝光、曝光时间 `500`、增益 `80`。未修改的字段也必须
完整填写。

```bash
ros2 topic pub --once /industrial_camera/params \
  drone_msgs/msg/IndustrialCameraParams \
  "{auto_exposure: false, exposure_absolute: 500, auto_exposure_priority: false, gain: 80, brightness: 128, contrast: 65, saturation: 90, gamma: 130, sharpness: 128, backlight_compensation: 16, auto_white_balance: true, white_balance_temperature: 4650, power_line_frequency: 1, auto_focus: true, focus_absolute: 0, zoom_absolute: 120}"
```

验证话题及 QoS：

```bash
ros2 topic info -v /industrial_camera/params
```

视觉端不会提供可订阅的相机状态话题；检查写入失败或范围裁剪时，请查看
`industrial_animal_vision_node` 的日志。
