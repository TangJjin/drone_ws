# industrial_animal_vision 绑核与 60fps 板端基线（2026-07-26）

> 目标：记录视觉节点从"120fps + 节点内自绑 CPU4-7"切换到"60fps + launch 层 taskset 绑 CPU6-7"后的首组板端实测数据，补上此前性能文档中动物模型与工业相机链路的"待测"空档，作为后续换模型/换相机/整机联调的对比基线。
>
> 本次为单节点测试（仅视觉节点 + 旁路监控脚本），display_enabled=true（默认值）；整机共存负载见文末待测项。

## 1. 变更内容与动机

| 项目 | 旧值 | 新值 | 动机 |
|------|------|------|------|
| `camera_fps` | 120 | 60 | 伺服输出恒 25Hz，120fps 过配约 4 倍；且请求 120 时板端实测采集率漂移 127-370fps 不受控 |
| 绑核方式 | 节点构造函数内 `sched_setaffinity(0, CPU4-7)` | launch 的 `Node(prefix="taskset -c 6,7")`（`cpu_set` 参数，默认 `6,7`） | `sched_setaffinity(0,...)` 只作用于调用线程及之后创建的线程；DDS/rmw 线程在 `rclcpp::Node` 基类构造时已创建，掩码仍是全核 0-7 会漂到任意核。taskset 在 exec 时生效，能圈住全部线程 |
| `cpu_affinity_enabled` | true | false | 置 true 时节点自绑 CPU4-7、worker 经 `pthread_setaffinity_np` 绑 CPU5-7——均在 taskset 掩码之外，会静默越界占用留给外部的核 |

worker 说明：3 个推理 worker 不再与 CPU 一一绑定，改为在掩码内浮动。2 核 <3 worker 时一一绑定在数学上不成立；worker 每周期约 18ms 阻塞在 NPU、仅 2-3ms 用 CPU，浮动由调度器挑空闲核反而更优。NPU 侧三上下文各绑 NPU_CORE_0/1/2 不受 CPU 掩码影响。

## 2. 测试环境

- 板卡：Orange Pi 5 Ultra（RK3588：CPU0-3=A55，CPU4-7=A76，NPU 3 核）
- 节点：`industrial_animal_vision_node`，模型 `animal_clean200_yolo11n_int8.rknn`（INT8）
- 相机：12MP U3 Camera，1280x720 MJPEG，请求 60fps
- 启动：`ros2 launch drone_perception industrial_animal_vision.launch.py`（全默认参数）
- 监控：`monitor_yolo_npu_perf.sh --process industrial_animal_vision_node --interval 1`（sudo）
- 采样时间：2026-07-26 22:29

## 3. 实测数据

### 3.1 线程亲和（全部通过）

```bash
for t in /proc/$(pgrep -f industrial_animal_vision_node)/task/*; do taskset -pc ${t##*/}; done
```

27 个线程（含 DDS/rmw、GStreamer 内部线程）掩码全部为 `6,7`，无一漏网。

### 3.2 负载与温度（单次采样，display_enabled=true）

| 指标 | 实测值 |
|------|--------|
| 进程 CPU | 103.7%（2 核预算 200% 的约 52%） |
| RSS | 149.8 MB |
| 线程数 | 26 |
| 系统 CPU | 21.2% |
| NPU debugfs 三核 | core0=34.0% / core1=34.0% / core2=35.0% |
| NPU devfreq | load=100%（口径为"有活即计满"，以 debugfs 三核为准）freq=1.00GHz governor=rknpu_ondemand |
| 温度 | soc/bigcore0/bigcore1/littlecore ≈ 49.9°C，center/gpu ≈ 49.0°C |

## 4. 数据解读

1. **动物 INT8 模型单帧推理 ≈18ms（首次实测佐证）**：由三核占用反推，0.345 × 3000ms ÷ 18ms ≈ 57.5fps，与相机 60fps 输入吻合。此前 18ms 只有 qr/package 模型（rknn_cpu_npu_efficiency_full_report.md）的数据，动物模型一直缺基线，本次补上。
2. **帧率协商问题消失**：处理率 ≈57-58fps ≈ 输入 60fps，说明相机在 60 档协商稳定，请求 120 时的 127-370fps 漂移不再出现，且几乎每帧都被处理、丢帧极少。
3. **NPU 占用与公式吻合**：预测 60×18/3000≈36%/核，实测 34-35%/核，占用公式（full_report 5.7 节）对动物链路同样适用；NPU 余量约 65%。
4. **2 核足够，第 3 核无收益**：`cpu_set:=5-7` 与默认 `6,7` 对比无可观测差异——瓶颈在相机入口帧率（60fps）而非 CPU（预算才用一半、NPU 34%），加核属于给吃不饱的进程加饭。**推荐保持默认 `6,7`，CPU0-5 完整让给雷达/定位/控制等外部节点。**
5. **display 开销已计入本组数据**：103.7% 是开显示的数字，飞行剖面 `display_enabled:=false` 后进程 CPU 会进一步下降（显示路径为纯 CPU 的 cvtColor+画框+imshow）。

## 5. 结论与推荐默认值

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `cpu_set`（launch 参数） | `6,7` | 2 核实测足够；换核只需 `cpu_set:=...`，传空串关闭绑核 |
| `camera_fps` | 60 | 协商稳定、NPU 34%、覆盖伺服 25Hz 需求 |
| `cpu_affinity_enabled` | false | 必须保持，防节点自绑/worker 越界到 CPU4-5 |
| `display_enabled` | 台架 true / 飞行 false | 显示是最大单项可省 CPU 开销 |

## 6. 复测命令

```bash
# 线程掩码（期望每行都是 6,7）
for t in /proc/$(pgrep -f industrial_animal_vision_node)/task/*; do taskset -pc ${t##*/}; done

# 伺服频率（期望 ~25Hz）
ros2 topic hz /vision/servo/target

# 负载/NPU/温度旁路监控
sudo ./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --process industrial_animal_vision_node --interval 1
```

## 7. 遗留待测项

1. **整机共存负载**：supervisor 全链（mavros + livox + FAST-LIO + 视觉等）同时运行时的视觉指标回归与 `/Odometry` 频率稳定性——历史上从未实测。
2. **飞行剖面数据**：`display_enabled:=false` 下的进程 CPU 基线。
3. **长稳/热衰减**：本组温度为短测约 50°C，10-30 分钟连续运行的降频风险仍未验证（full_report 8.2 待补充项）。
4. 其余非视觉节点（FAST-LIO/mavros/任务链）仍未绑核，全核浮动；若后续观测到与视觉在 CPU6-7 上竞争，再评估 supervisor 层 taskset 方案。
