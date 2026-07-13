# P0：当前 9 输出 YOLO11 FP16 基线采集说明

> 对应优化报告第一步：先建立当前 9 输出模型真实基线，再做 INT8 等后续优化。  
> 模型：`package_qrcode_shelf_tag_fp16.rknn`（路径 B，9 输出 DFL）  
> 架构：3 Worker × 3 NPU Context

## 1. 日志关键字

节点每约 1 秒打印一行：

```text
RKNN_BASELINE model=package_qrcode_shelf_tag_fp16 path=9out_fp16_3w ...
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `input_fps` | Color 入队频率（收到并尝试 enqueue 的帧/秒） |
| `process_fps` | 三 Worker 合计完成推理的帧/秒 |
| `consume_fps` | Timer 消费“最新结果”的频率 |
| `publish_fps` | 业务路径完成次数/秒（含 ZBar/发布尝试） |
| `npu_capacity_fps` | `sum(1000 / 各核 latest rknn_run_ms)` |
| `frame_age_ms` | Color 时间戳到消费时刻的端到端年龄 |
| `queue_drop_delta` / `stale_delta` | 本秒队列丢旧 / stale 增量 |
| `core_latest_ms` | 各核最近一次 `rknn_run_ms` |
| `core_med_ms` / `core_p95_ms` / `core_avg_ms` | 各核滚动窗口（最多 256 样本） |
| `pre/in/run/out/post/total` | 最近一帧 detector 分段计时（ms） |

## 2. 板端编译

```bash
cd ~/drone_ws
git fetch origin
git checkout feature/drone_perception
git pull --ff-only origin feature/drone_perception

source /opt/ros/humble/setup.bash
colcon build \
  --packages-select drone_perception \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 3. 测试运行（建议性能口径）

先确认相机 topic：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

性能模式（关调试窗，减少 UI 开销）：

```bash
ros2 run drone_perception qr_vision_node --ros-args \
  -p debug_view:=false \
  -p enable_rknn:=true
```

另开终端抓基线日志 ≥ 60 秒：

```bash
ros2 run drone_perception qr_vision_node --ros-args \
  -p debug_view:=false \
  -p enable_rknn:=true 2>&1 | tee /tmp/rknn_p0_baseline_$(date +%Y%m%d_%H%M%S).log
```

或在已运行节点时：

```bash
# 若节点已在别处启动，可从 journal / 终端过滤
grep RKNN_BASELINE /tmp/rknn_p0_baseline_*.log | tail -n 80
```

旁路 NPU/系统监控（若脚本存在）：

```bash
sudo ~/drone_ws/src/drone_perception/scripts/monitor_yolo_npu_perf.sh \
  --duration 60 \
  --interval 0.5 \
  --no-clear \
  --show-raw
```

## 4. 验收清单（本步完成标准）

- [ ] 连续 ≥ 60 s 有稳定 `RKNN_BASELINE` 行
- [ ] 记录 `input/process/consume/publish` 与 `core_med/p95`
- [ ] 记录 `drop/stale` 是否持续快速增长
- [ ] 记录 `frame_age_ms` 量级
- [ ] 记录 `run` 占 `total` 比例（判断后续是否优先 INT8）
- [ ] 固定提交 SHA 与模型文件名

## 5. 注意

- 旧文档 `rknn_runtime_performance_optimization_log.md` 基于单输出 `qr_rk3588_FP16.rknn`，**不可与本 9 输出基线混用**。
- `debug_view:=true` 会抬高 CPU/DDR 开销，对比 INT8 时应用同一 `debug_view` 设置。
- 检测召回/误检需人工或录包对照，本日志只覆盖吞吐与延迟口径。
