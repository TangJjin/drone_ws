# 工业相机 USB 死机问题与自启动恢复方案（交接给 bringup 维护者）

> 2026-07-27 板端实测记录。视觉端（drone_perception）已确认问题根因，
> 恢复逻辑按分工放在 startup_supervisor（自启动总开关）实现，本文档
> 给出问题全貌、根因分析与建议实现，供 drone_bringup 维护者落地。

## 1. 现象（板端两次实拍复现）

### 现象 A：开机自启动 / 手动 launch 时节点直接退出

```text
[FATAL] [industrial_animal_vision]: GStreamer pipeline produced no camera frame within 2 seconds
[ERROR] process has died [exit code 1]
```

管线创建成功、MPP 解码器配置成功，但相机 2 秒内一帧都不产出，
节点按设计主动退出（宁可失败也不带病运行）。

### 现象 B：运行中途视觉流悄悄冻结

```text
ANIMAL_SERVO      seq=2916 frame=6075 ... valid=0 age_ms=1597.8   ← frame 卡死不再前进
ANIMAL_SERVO_RATE pub_hz=25.00 snapshot_hz=0.00 stale=378         ← 推理快照速率归零
```

节点进程仍然存活，伺服话题仍以 25Hz 发布 valid=false 心跳
（降级保护按设计工作，控制端安全），但相机已不再产出任何帧。

### 关键事实

- 此状态下 **重新 launch 无效**（现象 B 之后手动重启节点，立刻变成现象 A）
- **拔插相机 USB 后立即恢复正常**
- 正常启动的参照：首帧约 1.1 秒内就绪，日志出现
  `GStreamer first camera frame ready: RGB 1280x720`

## 2. 根因

相机模组（12MP U3 Camera，USB ID `32e4:6577`）内部固件死机。

- `/dev/v4l/...` 设备节点只是内核的入口，相机固件死机后它依然存在、
  依然可以 open/配置参数，但永远不出帧——"门牌还在，屋里没人"
- 软件重启节点只是重新打开设备，**不会给相机断电**，救不了死机的固件
- 拔插 USB 的本质是给相机断电重启，固件从头开机，所以立竿见影

诱因（按嫌疑排序）：

1. 前一个节点实例被强杀（自启动失败清场、手动 kill），相机取流会话
   没有正常关闭，部分廉价固件在下一次重新配置时锁死
2. 上电瞬间 USB3 供电/信号毛刺（USB3 辐射频段与 2.4GHz 相邻，机上
   电磁环境差时是知名问题）
3. 固件在 60fps MJPG 持续负载下自身的稳定性缺陷

## 3. 手动救急命令（不用拔线）

Linux 可以软件模拟拔插（USB 总线级 reset，等效断电重启相机）：

```bash
sudo usbreset 32e4:6577
```

- `usbreset` 来自 usbutils 包：`sudo apt install usbutils`
- `32e4:6577` 是相机的厂商号:产品号，拔插/换口/重启都不变，
  只有换不同型号相机才需要重查（`lsusb`）

## 4. 建议的自启动恢复方案（待 bringup 维护者实现）

### 4.1 设计

在 `startup_supervisor.py` 的 `qr_vision_node` 步骤（工业相机视觉）
加入"探针失败 → USB reset → 重试一次"的恢复流程：

```text
启动步骤 → 就绪探针（/vision/servo/target 有消息）
   ├─ 通过 → 继续下一步骤（现状不变）
   └─ 超时失败：
        1. 杀掉本步骤已启动的进程（清场）
        2. usbreset 32e4:6577（给相机断电重启）
        3. 等待 2~3 秒（相机重新枚举）
        4. 重新启动本步骤，探针重新计时
        5. 第二次仍失败 → 走现有的整机失败流程（保持 fail-stop 兜底）
```

### 4.2 参考实现（示意，接在现有 StartupStep 结构上）

```python
CAMERA_USB_ID = '32e4:6577'

def reset_camera_usb() -> bool:
    """软件模拟拔插相机。需要 usbutils 与 udev 授权（见 4.3）。"""
    result = subprocess.run(
        ['usbreset', CAMERA_USB_ID],
        capture_output=True, text=True, timeout=10)
    print(f'[recover] usbreset {CAMERA_USB_ID}: rc={result.returncode} '
          f'{result.stdout.strip()} {result.stderr.strip()}')
    return result.returncode == 0

# 在步骤执行循环中，对 name == 'qr_vision_node' 的步骤：
#   第一次 ready 检查失败时不立即 fail_stop，而是：
#     stop_step_process(step)      # 杀掉本步骤进程
#     reset_camera_usb()
#     time.sleep(3.0)              # 等相机重新枚举
#     retry = start_step(step)     # 原样重启 + 重新探针
#   retry 仍失败 → 原有 stop_all_processes() + sys.exit(1)
```

建议把"是否重试过"做成局部标记而不是全局配置，只对相机步骤生效，
避免影响 mavros / 雷达等其他步骤的失败语义。

### 4.3 权限准备（一次性系统配置）

supervisor 以普通用户运行，USB reset 默认需要 root。两种任选：

**方案一（推荐）：udev 授权该相机设备**

```bash
sudo tee /etc/udev/rules.d/99-industrial-camera.rules <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="32e4", ATTR{idProduct}=="6577", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

之后普通用户可直接 `usbreset 32e4:6577`（免 sudo）。

**方案二：sudoers 放行单条命令**

```bash
echo 'orangepi ALL=(root) NOPASSWD: /usr/bin/usbreset' | \
  sudo tee /etc/sudoers.d/usbreset-camera
```

代码里相应改为 `['sudo', 'usbreset', CAMERA_USB_ID]`。

## 5. 本方案的覆盖范围与已知残余风险

| 场景 | 是否覆盖 |
|---|---|
| 开机时相机死机（现象 A） | ✅ 自启动自动恢复 |
| 手动 `ros2 launch` 调试时死机 | ❌ 不经过 supervisor，手动跑第 3 节命令 |
| **飞行/运行中途死机（现象 B）** | ❌ **不覆盖**，见下 |

现象 B 时节点进程存活、心跳照发，supervisor 的探针早已通过，
**结构上无法感知**。此时的系统行为是：视觉端持续发 valid=false，
控制端按目标丢失超时处理（安全，但当次任务失败）。

若后续实测中现象 B 发生频率不可接受，备选方案是在视觉节点内部
加"断粮看门狗"（连续 N 秒无帧 → 自行 reset + 重建管线），该改动
属 drone_perception 侧，与本方案不冲突，可叠加。

## 6. 验收方法

1. 正常开机一次，确认视觉步骤照常通过（不回归）
2. 人为制造死机复现恢复：节点运行中执行
   `sudo usbreset 32e4:6577` 有小概率使相机进入坏状态；更可靠的
   办法是开机前反复上下电几次直到复现现象 A
3. 观察 supervisor 日志应出现 `[recover] usbreset ...` 且第二次
   启动成功、整机继续后续步骤
4. 连续重启整机 5 次，视觉步骤全部通过即验收
