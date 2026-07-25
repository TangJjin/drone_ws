# 工业相机地面站调参交付说明

本文只说明地面站如何控制视觉端的 `industrial_animal_vision_node`。视觉端已经
负责相机驱动、参数保存和互斥判断；地面站不需要调用 V4L2，也不需要修改视觉端
的配置文件。

## 1. 先记住这三件事

地面站只需要做两件事：

1. 订阅视觉端发出的相机信息。
2. 用户操作界面时，向视觉端发送修改请求。

三个话题的分工如下：

| 话题 | 谁发布 | 谁订阅 | 一句话作用 |
| --- | --- | --- | --- |
| `/industrial_camera/control/capabilities` | 视觉端 | 地面站 | 告诉地面站：这台相机有哪些参数，每个滑块的最小值、最大值和步进是多少。 |
| `/industrial_camera/control/state` | 视觉端 | 地面站 | 告诉地面站：相机现在实际的参数值是什么，哪些控件当前应变灰。 |
| `/industrial_camera/control/command` | 地面站 | 视觉端 | 地面站告诉视觉端：用户刚刚要修改哪个参数、保存参数，或恢复默认。 |

地面站**永远不发布** `state` 和 `capabilities`。它只发布 `command`。

完整方向如下：

```text
工业相机 <-> 视觉端 industrial_animal_vision_node <-> 地面站

地面站 -- command --> 视觉端 -- 写入相机 --> 工业相机
地面站 <-- state --- 视觉端 <-- 读取相机 <-- 工业相机
地面站 <-- capabilities --- 视觉端
```

## 2. 地面站启动时应该怎么做

### 2.1 订阅两个视觉端输出

地面站启动后立刻订阅：

```text
/industrial_camera/control/capabilities
/industrial_camera/control/state
```

视觉端使用 `transient_local` 发布这两个话题，所以即使视觉端已经运行很久，
地面站刚打开也会马上收到最后一份参数信息，不需要重启相机。

### 2.2 收到 capabilities 后创建界面

`capabilities.controls` 是一个控制项列表。每个列表项包含：

| 字段 | 地面站用途 |
| --- | --- |
| `name` | 控制项名称，例如 `gain`。 |
| `update_mask` | 用户改这个控制项时，`command.update_mask` 要使用的位。 |
| `control_type` | `INTEGER` 用滑块；`BOOLEAN` 用开关/两个圆点；`MENU` 用多个圆点。 |
| `minimum`、`maximum`、`step` | 滑块的最小值、最大值、单步值。 |
| `menu_values`、`menu_labels` | 菜单或单选圆点的真实值和显示文字。 |
| `available` | `false` 表示相机不支持此项，地面站隐藏它或显示灰色“不支持”。 |
| `writable` | `false` 表示驱动只允许读取，不能让用户修改。 |
| `active` | `false` 表示该参数被当前自动模式暂时禁用，应显示灰色。 |
| `current_value` | 此时驱动读取到的值，可作为界面初始显示值。 |

地面站不能把范围写死。例如当前相机的增益是 `0..190`，但以后换相机后可能是
`0..100`；必须按 `minimum/maximum/step` 创建滑块。

### 2.3 收到 state 后刷新界面

`state` 是相机的**实际当前状态**，不是地面站发出的命令回显。

收到 `state` 后地面站应：

1. 用 `state.gain`、`state.brightness` 等字段更新全部滑块当前位置。
2. 用 `state.exposure_auto`、`state.white_balance_auto`、`state.focus_auto` 更新圆点或开关选中状态。
3. 判断每个控件是否能操作；不能操作时显示灰色并禁用交互。
4. 若 `state.request_id` 与自己刚发送的命令相同，则这是该操作的结果；查看
   `success` 和 `message` 决定是否提示失败。

`state.request_id=0` 是视觉端启动后主动发出的初始状态，不对应任何用户操作。

## 3. 用户操作时的完整流程

### 3.1 拖动“增益”滑块

用户把增益滑块拖到 `80` 后，地面站执行：

```text
1. 生成新的 request_id，例如 1001。
2. 发布一条 command：
   command = APPLY
   update_mask = UPDATE_GAIN
   gain = 80
3. 等待视觉端的 state。
4. 收到 request_id=1001 的 state 后，用 state.gain 更新滑块。
```

实际通信如下：

```text
用户拖动 gain 到 80
    -> 地面站发布 command(gain=80)
    -> 视觉端检查范围、写入相机、再读回相机
    -> 视觉端发布 state(request_id=1001, gain=80)
    -> 地面站把 gain 滑块显示为 80
```

如果用户请求 `gain=300`，而相机最大增益是 `190`，视觉端会实际写入 `190`，
然后发布 `state.gain=190`。地面站最终必须显示 `190`，而不是仍显示 `300`。

建议地面站在用户松开滑块后发送一次命令，或停止拖动约 100 ms 后发送一次；
不要每移动一个像素就连续发送命令。

### 3.2 用户点击“自动曝光”

用户选择自动曝光时，地面站发布：

```text
command = APPLY
update_mask = UPDATE_EXPOSURE_AUTO
exposure_auto = 3
```

视觉端写入自动曝光，随后发布最新 `state`。这时：

```text
state.exposure_auto = 3
state.active_mask 中不再有 UPDATE_EXPOSURE_ABSOLUTE
```

地面站收到该状态后，将“手动曝光时间”滑块变灰并禁止拖动。相机仍会保留该滑块
的旧数值，但自动曝光期间不使用它。

### 3.3 用户切换手动曝光并设置曝光时间

建议地面站把模式和数值合成**一条**命令发送：

```text
command = APPLY
update_mask = UPDATE_EXPOSURE_AUTO | UPDATE_EXPOSURE_ABSOLUTE
exposure_auto = 1
exposure_absolute = 500
```

视觉端内部先切换成手动曝光，再写入曝光时间 `500`，最后读取真实值并发布 state。
不能先单独发送曝光时间、再发送手动模式，因为自动曝光下相机不允许写手动曝光时间。

### 3.4 自动白平衡和自动对焦

逻辑与曝光完全相同：

| 自动模式开启 | 地面站应变灰的手动项 | 关闭自动模式后可调 |
| --- | --- | --- |
| `white_balance_auto=1` | `white_balance_temperature` 色温滑块 | 色温 |
| `focus_auto=1` | `focus_absolute` 焦距滑块 | 焦距 |
| `exposure_auto=3` | `exposure_absolute` 曝光时间滑块 | 曝光时间 |

地面站无需自己猜测互斥关系。只要每次收到 state 后按掩码重新判断控件是否可用：

```text
某控制项可交互 =
  对应位在 available_mask 中
  且对应位在 writable_mask 中
  且对应位在 active_mask 中
```

三者任何一个条件不满足，就灰显该控件。

### 3.5 用户点击“保存参数”

地面站只发送：

```text
command = SAVE_CURRENT
request_id = 新编号
update_mask = 0
```

视觉端先从相机读取全部实际值，再保存到：

```text
~/.ros/drone_perception/industrial_camera_saved.yaml
```

视觉端随后发布 state。`success=true` 表示保存成功。下次视觉节点启动时，会优先
加载这个保存文件。

### 3.6 用户点击“恢复默认”

地面站只发送：

```text
command = RESTORE_PROJECT_DEFAULTS
request_id = 新编号
update_mask = 0
```

视觉端执行：

```text
industrial_default.yaml
    -> 写入相机
    -> 用同样默认值覆盖 industrial_camera_saved.yaml
    -> 读取相机真实值
    -> 发布最新 state
```

因此“恢复默认”之后，当前相机和下次启动都会使用 `industrial_default.yaml` 的默认值，
不会恢复以前保存的用户参数。

## 4. 相机参数和控件类型

下表是当前 Orange Pi `/dev/video1` 的实机范围。地面站仍应优先使用
capabilities 中收到的范围，因为换相机后可能不同。

| 参数名 | 用户界面 | 当前范围/选项 | 作用 |
| --- | --- | --- | --- |
| `exposure_auto` | 手动/自动两个圆点 | 手动=`1`，自动=`3` | 选择曝光由用户固定，还是由相机自己调整。 |
| `exposure_absolute` | 滑块 | `1..10000` | 手动曝光时间；自动曝光时灰色。 |
| `exposure_auto_priority` | 开关 | `0/1` | 自动曝光时是否允许相机降低帧率来获得更亮图像。 |
| `gain` | 滑块 | `0..190` | 增益，越大通常越亮，也可能噪点更多。 |
| `brightness` | 滑块 | `0..255` | 图像亮度处理值。 |
| `contrast` | 滑块 | `0..128` | 图像明暗反差。 |
| `saturation` | 滑块 | `0..128` | 色彩饱和度。 |
| `gamma` | 滑块 | `0..255` | 中间亮度校正，不等于曝光。 |
| `sharpness` | 滑块 | `0..255` | 图像锐化。 |
| `backlight_compensation` | 滑块 | `16..160` | 逆光补偿。 |
| `white_balance_auto` | 自动/手动圆点 | `0/1` | 自动白平衡开关。 |
| `white_balance_temperature` | 滑块 | `2800..6500` | 手动色温；自动白平衡时灰色。 |
| `power_line_frequency` | 关闭/50 Hz/60 Hz 圆点 | `0/1/2` | 防止灯光造成画面闪烁。 |
| `focus_auto` | 自动/手动圆点 | `0/1` | 自动对焦开关。 |
| `focus_absolute` | 滑块 | `0..1023` | 手动焦距；自动对焦时灰色。 |
| `zoom_absolute` | 滑块 | `100..200` | 变焦。 |

`pan_absolute` 和 `tilt_absolute` 没有纳入接口。它们会让镜头物理左右/上下转动，
属于云台运动控制，不属于相机成像调参。

## 5. Command 消息每个字段的作用

消息类型：`drone_msgs/msg/IndustrialCameraControlCommand`

| 字段 | 地面站填写规则 |
| --- | --- |
| `request_id` | 每次发送递增，例如 1、2、3。用来在 state 中找到这次操作结果。 |
| `command` | `COMMAND_APPLY`=修改参数；`COMMAND_SAVE_CURRENT`=保存；`COMMAND_RESTORE_PROJECT_DEFAULTS`=恢复默认。 |
| `update_mask` | 说明本次要改哪些字段。只改增益就只放 `UPDATE_GAIN`，其余字段即使带了数值也会被视觉端忽略。 |
| 所有参数字段 | 仅当自己对应的 `UPDATE_*` 位已写入 `update_mask` 时才生效。 |

`update_mask` 不能直接填写参数值，它是要修改的字段标记。常用值：

| 要修改什么 | `update_mask` |
| --- | ---: |
| 自动/手动曝光 | `UPDATE_EXPOSURE_AUTO` = `1` |
| 手动曝光时间 | `UPDATE_EXPOSURE_ABSOLUTE` = `2` |
| 同时切手动曝光并设置时间 | `1 | 2` = `3` |
| 增益 | `UPDATE_GAIN` = `8` |
| 亮度 | `UPDATE_BRIGHTNESS` = `16` |
| 自动白平衡 | `UPDATE_WHITE_BALANCE_AUTO` = `1024` |
| 手动色温 | `UPDATE_WHITE_BALANCE_TEMPERATURE` = `2048` |
| 防闪烁频率 | `UPDATE_POWER_LINE_FREQUENCY` = `4096` |
| 自动对焦 | `UPDATE_FOCUS_AUTO` = `8192` |
| 手动焦距 | `UPDATE_FOCUS_ABSOLUTE` = `16384` |
| 变焦 | `UPDATE_ZOOM_ABSOLUTE` = `32768` |

其他图像滑块按顺序为：`contrast=32`、`saturation=64`、`gamma=128`、
`sharpness=256`、`backlight_compensation=512`、`exposure_auto_priority=4`。

## 6. State 消息每个字段的作用

消息类型：`drone_msgs/msg/IndustrialCameraControlState`

| 字段 | 地面站应如何使用 |
| --- | --- |
| `request_id` | 等于自己发送的编号，表示是这次操作的结果；为 `0` 表示启动初始状态。 |
| `success` | `true` 表示整条操作成功；`false` 表示至少有一项失败。 |
| `message` | 可显示给用户的失败原因或提示文字。 |
| `applied_mask` | 本次实际成功修改的字段。 |
| `rejected_mask` | 本次被拒绝的字段，例如自动模式下修改了手动值。 |
| `available_mask` | 当前相机真正支持的控制项。 |
| `writable_mask` | 当前相机允许写入的控制项。 |
| `active_mask` | 当前模式下没有被自动模式禁用的控制项。 |
| `exposure_auto` 到 `zoom_absolute` | 全部是视觉端从相机回读的真实值，用来刷新对应滑块和圆点。 |

## 7. 视觉端内部到底做了什么

地面站发布一条 `command` 后，视觉端按下面顺序处理：

```text
收到 command
  -> 检查 command 类型和 update_mask
  -> 查询相机是否支持该参数、范围和当前是否可修改
  -> 若本次同时切自动/手动模式和手动值，先切模式
  -> 将参数写入相机
  -> 从相机读回真实值
  -> 更新 capabilities（当前哪些参数变灰可能已改变）
  -> 发布 state 给地面站
```

这里的三个 Linux 相机接口仅由视觉端使用：

| 接口 | 视觉端用途 |
| --- | --- |
| `VIDIOC_QUERYCTRL` | 查询相机是否支持该参数、范围、步进、是否只读或暂时不可用。 |
| `VIDIOC_S_CTRL` | 把地面站请求的值写给相机。 |
| `VIDIOC_G_CTRL` | 从相机读回最终真实值。 |

地面站只需要发 ROS 2 command、收 ROS 2 state，不需要知道如何打开 `/dev/video1`。

## 8. ROS 2 命令行联调例子

切为手动曝光并设为 500：

```bash
ros2 topic pub --once /industrial_camera/control/command \
  drone_msgs/msg/IndustrialCameraControlCommand \
  "{request_id: 1001, command: 0, update_mask: 3, exposure_auto: 1, exposure_absolute: 500}"
```

只设置增益为 80：

```bash
ros2 topic pub --once /industrial_camera/control/command \
  drone_msgs/msg/IndustrialCameraControlCommand \
  "{request_id: 1002, command: 0, update_mask: 8, gain: 80}"
```

保存当前参数：

```bash
ros2 topic pub --once /industrial_camera/control/command \
  drone_msgs/msg/IndustrialCameraControlCommand \
  "{request_id: 1003, command: 1, update_mask: 0}"
```

恢复项目默认参数：

```bash
ros2 topic pub --once /industrial_camera/control/command \
  drone_msgs/msg/IndustrialCameraControlCommand \
  "{request_id: 1004, command: 2, update_mask: 0}"
```

查看视觉端返回结果：

```bash
ros2 topic echo /industrial_camera/control/state \
  drone_msgs/msg/IndustrialCameraControlState
```

## 9. 地面站最小实现清单

1. 建立一个 command 发布器，和 capabilities/state 订阅器。
2. 收到 capabilities 后按实际范围创建滑块和圆点。
3. 收到 state 后更新所有控件数值，并按三个 mask 灰显互斥项。
4. 用户拖动或点击时发布一条 command；不要发布 state。
5. 保存和恢复默认各自发布一条 command，并在收到对应 request_id 的 state 后刷新整个面板。
