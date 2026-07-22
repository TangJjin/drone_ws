# RK3588 YOLO 实时链路：CPU / NPU 效率优化全记录

> 工程：`drone_perception`  
> 平台：Orange Pi 5 Ultra（RK3588）+ Intel RealSense D435i  
> 文档性质：已落地技术 + 板上实测证据 + 后续优化路线  
> 最新 6 组分辨率×精度实测日期：2026-07-14  
> 板端原始日志：`/tmp/rknn_res_model_compare_20260714_014417/`  
> 相关文档：  
> - [`rknn_runtime_performance_optimization_log.md`](./rknn_runtime_performance_optimization_log.md)  
> - [`yolo_npu_fps_optimization.md`](./yolo_npu_fps_optimization.md)  
> - Desktop：`RK3588_YOLO11_NPU性能优化对比报告.md`

---

## 0. 一页结论

### 0.1 我们已经做到什么

| 维度 | 当前状态（2026-07-14 实测） | 证据 |
| --- | --- | --- |
| 吞吐 | 30 FPS 相机下 **Input ≈ Process**，无队列无限积压 | 6 组测试 drop/stale 全程为 0 |
| 并行 | **3 Context × 3 NPU 核**，多帧并行 | 日志 `path=9out_3w`，core mask 1/2/4 |
| NPU 算力利用率（相对输入） | 容量远高于输入；INT8 三核约 **16%**，FP16 约 **30%** | debugfs `rknpu/load` |
| 单帧延迟主因 | 几乎全部在 **`rknn_run`** | INT8 run≈18ms / det≈20ms；FP16 run≈33–35ms / det≈36–40ms |
| 量化收益 | INT8 比 FP16 **快约 1.8–2.0×** | 同分辨率 A/B 实测 |
| 分辨率影响 | 相机 640→1280 **几乎不改变** `rknn_run`（模型固定 640×640 letterbox） | 6 组 run 对比 |

### 0.2 一句话路线图

```text
阶段 A 架构：回调与推理解耦 + 三核三 Context + 最新帧队列
阶段 B CPU：绑大核、去重复色彩转换、缓冲复用、减日志、Depth 分流
阶段 C 输入：native tensor zero-copy（FP16/INT8 各自匹配）
阶段 D 模型：INT8 9 输出路径落地，rknn_run 从约 40ms 级降到约 18ms
阶段 E 验证：分辨率×精度 6 组实测，确认“输入喂满 vs 算力空闲”的真实关系
```

**当前不建议优先做**：盲目加到 6/12 Context。  
**原因**：Input≈Process 且 NPU 仍有大量空闲 → 瓶颈是“没有更多帧可喂 / 单帧模型耗时”，不是“并行度不够”。

---

## 1. 问题背景与指标定义

### 1.1 早期误判

早期常见现象：

- 监控里 **NPU load 接近 100%**，但画面只有十几 FPS；
- 或反过来 **NPU 只有二十来个百分点**，怀疑“没用上 NPU”。

正确理解：

| 现象 | 真实含义 |
| --- | --- |
| `devfreq .../npu/load ≈ 100` | NPU 设备整体忙，**不等于** core0/1/2 都满 |
| debugfs 三核 16% | 在 30 FPS 输入 + 单核 18ms 下是**数学上合理**的空闲 |
| Process FPS ≈ Input FPS | 链路跟得上输入，再加 Context **抬不高有效帧率** |
| Process FPS ≪ Input FPS 且 drop 涨 | 才值得考虑更多 Context / 更轻模型 |

### 1.2 指标口径（必须统一）

| 指标 | 含义 | 用来判断什么 |
| --- | --- | --- |
| **Input FPS** | Color 入队频率 | 相机 / ROS 是否限速 |
| **Process FPS** | 三 Worker 合计完成帧率 | 并行推理吞吐 |
| **Consume / Publish FPS** | 最新结果消费 / 业务路径频率 | 是否跟得上处理 |
| **NPU Capacity FPS** | `sum(1000 / 各核 rknn_run_ms)` | 三核理论满载吞吐 |
| **frame_age_ms** | Color 时间戳到结果消费的年龄 | 控制闭环感知延迟 |
| **pre / in / run / out / post** | 单帧 detector 分段 | 瓶颈在 CPU 还是 NPU |
| **queue_drop / stale** | 丢旧帧 / 过期结果 | 是否积压、是否乱序覆盖 |
| **NPU Core0/1/2 %** | `/sys/kernel/debug/rknpu/load` | 三核真实占用 |

**铁律：**

1. 吞吐容量 ≠ 单帧延迟。三核 160 FPS 容量 **不表示** 一帧只要 6 ms。  
2. 三 Context 是 **同时处理三帧**，每帧仍在 **一个 NPU 核** 上跑完整模型。  
3. 任何 A/B 必须固定：模型、分辨率、debug_view、频率/温度、统计时长。

### 1.3 当前硬件与软件基线

| 项目 | 值 |
| --- | --- |
| 板卡 | Orange Pi 5 Ultra |
| SoC | RK3588，3× NPU |
| RKNN Runtime | 2.3.2 |
| RKNPU 驱动 | 0.9.6 |
| 相机 | D435i Color + Depth（软对齐，50 ms 窗） |
| 检测模型 | `package_qrcode_shelf_tag_{i8,fp16}.rknn` |
| 网络输入 | 640×640 letterbox，NHWC |
| 输出 | 9 输出 YOLO11 DFL（box/class/sum 三尺度） |
| 类别 | qrcode / package / shelf_tag |
| 并行 | 3 Worker + 3 Context + Core0/1/2 |
| CPU 亲和 | 进程绑 **CPU4–7** 大核 |

---

## 2. 当前完整数据流（优化后）

```text
D435i
  |-- Color raw  ------------------> ROS Color 回调（只组帧/入队，不 rknn_run）
  |-- Depth raw  --> latest Depth --+
  |-- CameraInfo --> intrinsics ----+
  `-- Extrinsics --> extrinsics ----+
                                    |
                                    v
                      有界队列 capacity=3（满则丢最旧）
                         /          |          \
                        v           v           v
                   Worker0      Worker1      Worker2
                   Context0     Context1     Context2
                   NPU Core0    NPU Core1    NPU Core2
                        \           |           /
                         v          v          v
                      Frame ID / STALE 过滤（旧结果不覆盖新结果）
                                    |
                                    v
                      Depth 投影 / 2D 降级 / 业务发布 / 聚合日志
```

性能模式建议：

```bash
ros2 run drone_perception qr_vision_node --ros-args \
  -p debug_view:=false \
  -p enable_rknn:=true \
  -p rknn_model_path:=/path/to/package_qrcode_shelf_tag_i8.rknn
```

---

## 3. 已落地优化技术总表

按 **“主要提升 CPU 效率 / 主要提升 NPU 效率 / 同时改善稳定性”** 分类。  
“有效”均指在本工程路径上有代码与（或）实测支撑。

### 3.1 架构层（先解决“能不能喂满、会不会堵死”）

| 编号 | 技术 | 主要对象 | 做法 | 为什么有效 | 证据 / 边界 |
| --- | --- | --- | --- | --- | --- |
| A1 | ROS 回调与推理解耦 | CPU + 吞吐 | 回调只入队；`rknn_run` 在 Worker | 避免 20–40 ms 推理堵死相机回调 | 日志：pipeline ready + 持续 Input≈30 |
| A2 | 三 Context 绑三 NPU 核 | NPU 吞吐 | 每 Worker 独立 detector，mask 0/1/2 | 三帧并行，容量 > 30 FPS | Capacity：INT8≈167，FP16≈84–92 |
| A3 | 有界队列丢旧 | 延迟 / 内存 | queue_cap=3，满删最旧 | 禁止无限积压旧帧 | 6 组 drop=0（跟得上时） |
| A4 | Frame ID + STALE | 控制时效 | 乱序完成时丢旧结果 | 慢帧不覆盖新帧 | stale 计数器；实时控制语义 |
| A5 | Color 驱动 + Depth 软对齐 | CPU / 鲁棒 | 绕过 RGBD align+sync 硬同步 | 单路抖动不卡死整条链路 | soft_depth_match_ms=50；过期降级 2D |
| A6 | 不采用严格重排序 | 延迟 | 最新完成优先 | 严格排序会把长尾放大到控制端 | 文档明确否决 FIFO future |

### 3.2 CPU 效率层（让大核做有效功，少做无效功）

| 编号 | 技术 | 做法 | 收益类型 | 证据 / 边界 |
| --- | --- | --- | --- | --- |
| C1 | 绑定 CPU4–7 | `sched_setaffinity` 进程级 | 降调度抖动与长尾，**不缩短 NPU 算子** | `main.cpp`；日志 `CPU affinity : cores 4-7` |
| C2 | 直接 `rgb8`，避免 BGR↔RGB 往返 | cv_bridge 出 RGB，仅 UI 转 BGR | 减 CPU 像素遍历 | 相对旧路径少一次全图转换 |
| C3 | 预处理缓冲复用 | `resized_buffer_` / `letterbox_buffer_` 成员复用 | 减分配抖动 | pre 在 640 下约 0.3 ms 且稳定 |
| C4 | 精简逐框日志 | 每秒 `RKNN_BASELINE` 聚合 | 减格式化 / 锁 / 终端 I/O | 性能基线必须关刷屏日志 |
| C5 | 关 debug_view 测性能 | `debug_view:=false` | 减 clone / 画框 / 显示 | 6 组均在 false 下采集 |
| C6 | Depth 软快照而非硬对齐 | 最近 depth + 时间窗 | 减同步等待与对齐算力 | 深度过期只做 2D |

**CPU 效率的正确目标：**

```text
不是“把 CPU 占用率拉满”，
而是“单位有效帧的 CPU 工作量最小、抖动最小，
      并把时间片留给预处理/后处理/ROS，而不是空转与锁竞争”。
```

### 3.3 NPU 效率层（让每毫秒 NPU 做更多有效推理）

| 编号 | 技术 | 做法 | 收益类型 | 证据 / 边界 |
| --- | --- | --- | --- | --- |
| N1 | 每 Context 单核绑定 | `rknn_set_core_mask` CORE_0/1/2 | 多帧并行，避免单 Context 假三核 | 否决单 Context `CORE_0_1_2` 降单帧延迟的幻想 |
| N2 | `RKNN_FLAG_PRIOR_HIGH` | init 高优先级 | 多任务时减 NPU 调度等待 | 单节点时收益有限，仍保留 |
| N3 | Native 输入 zero-copy | `create_mem` + `set_io_mem` + `pass_through` | 去掉 `inputs_set` 内部转换/拷贝 | 旧 FP16：Input Prepare 约 11→0.7 ms |
| N4 | Native 类型必须匹配 | INT8 模型写 int8；FP16 写 fp16 | 避免“假 zero-copy”（格式转换挪到 run） | 实测 zc=int8 / zc=fp16 |
| N5 | INT8 模型落地 | `package_qrcode_shelf_tag_i8.rknn` | **最大单帧加速** | run 18ms vs FP16 33–35ms |
| N6 | 三核并行容量设计 | Capacity > Input | 保证 30 FPS 不 backlog | INT8 容量约 167 ≫ 30 |

**NPU 效率的正确目标：**

```text
1) 单帧：降低 rknn_run_ms（量化、算子、输入尺寸、模型结构）
2) 多帧：在输入足够高时用并行填满三核空档
3) 不为“占用率数字好看”而制造无效在途帧
```

### 3.4 测量闭环（没有测量就没有优化）

| 编号 | 技术 | 作用 |
| --- | --- | --- |
| M1 | 分段计时 pre/in/run/out/post | 判断瓶颈在 CPU 还是 NPU |
| M2 | `RKNN_BASELINE` 每秒一行 | Input/Process/age/drop/stale/core 统计 |
| M3 | debugfs 三核 load | 区分“总体忙”与“三核空闲” |
| M4 | 同场景 A/B | 一次只改一个变量 |

---

## 4. 我们如何一步一步提高 CPU / NPU 效率

下面按 **时间逻辑 + 因果链** 叙述，而不是技术清单堆叠。

### 步骤 1：先分清“堵在输入”还是“堵在推理”

**问题：** 回调里直接跑 `rknn_run` 时，相机回调被 30–40 ms 卡住，Input 抖动、队列语义混乱。

**动作：** A1 解耦 + A3 有界队列 + A4 STALE。

**对 CPU：** 回调线程变轻，ROS 调度更稳。  
**对 NPU：** Worker 可以连续取帧，避免“等回调 → NPU 空转 → 又突发”。

**判据：** 出现稳定的 Input FPS 与 Process FPS，而不是偶发卡顿。

---

### 步骤 2：用三核并行抬吞吐容量（不是砍单帧延迟）

**问题：** 单 Context 即使用 `CORE_0_1_2`，也很难把单帧延迟砍成 1/3。

**动作：** A2 三 Context 各绑一核。

**对 NPU：** 同时跑 3 帧 → Capacity 从“单核 1000/run_ms”变为“三核之和”。  
**对 CPU：** 三个 Worker 分摊预处理/后处理，但共享 CPU4–7，需要 C1 亲和与 C3 缓冲复用控制抖动。

**证据（历史 FP16 单输出路径）：** 单核 run≈41 ms 时，三核 Capacity≈70 FPS > 30。  
**证据（当前 INT8 9 输出）：** Capacity≈167 FPS。

**边界：** 这只保证“跟得上 30 FPS”，**不**把 frame_age 变成 10 ms。

---

### 步骤 3：CPU 侧做“减负”，不是“占满”

**动作：** C1–C6。

| 优化 | 直接效果 | 间接对 NPU 的帮助 |
| --- | --- | --- |
| 绑大核 | 预/后处理更稳 | 减少 Worker 在 NPU 前后的 CPU 空窗抖动 |
| 去重复色彩转换 | 每帧少一次全图像素写 | 略减 pre，把时间让给喂 NPU |
| 缓冲复用 | 分配器抖动↓ | pre 更平滑 |
| 减日志 / 关 debug_view | CPU/DDR 干扰↓ | 基准可重复 |

**关键认知：**  
在当前链路里，CPU 段合计往往只有 **1–3 ms**，而 `rknn_run` 是 **18–35 ms**。  
所以 CPU 优化的主价值是 **稳定性与可扩展性**，不是再抠出 2× 端到端 FPS。

---

### 步骤 4：输入路径匹配 native tensor（真正的 zero-copy）

**问题：** 用 `rknn_inputs_set` 提交 FP16 时，Input Prepare 可达约 **11–12 ms**。

**动作：** N3 + N4：

1. `RKNN_QUERY_NATIVE_INPUT_ATTR`  
2. `rknn_create_mem` / `rknn_set_io_mem` / `pass_through=1`  
3. CPU 直接写 `virt_addr` + `rknn_mem_sync(TO_DEVICE)`  

**历史证据（FP16）：**

| 指标 | 优化前 | zero-copy 后 |
| --- | ---: | ---: |
| Input Prepare | ~11 ms | ~0.7 ms |
| Detector Total | ~53 ms | ~43 ms |

**反例（我们测过并否决）：**

- 对 FP16 模型做 UINT8 zero-copy → run 变长，端到端无收益（转换被挪进 Runtime）。  
- 说明：**zero-copy 的关键是类型/layout 匹配，不是 API 名字好听。**

**当前 6 组证据：**

- INT8 日志：`zero_copy=int8`  
- FP16 日志：`zero_copy=fp16`  
- INT8 的 in≈0.17–0.24 ms；FP16 的 in≈0.71–0.77 ms（含 FP16 写入成本）

---

### 步骤 5：模型量化 —— 本轮最大的 NPU 单帧效率跃迁

**动作：** 从 FP16 9 输出模型切到 INT8 9 输出模型，并保持三核架构。

**2026-07-14 同板 A/B（见第 5 节）：**

| 分辨率 | INT8 run | FP16 run | FP16/INT8 |
| --- | ---: | ---: | ---: |
| 640×480 | 17.97 ms | 32.54 ms | **1.81×** |
| 848×480 | 17.60 ms | 33.48 ms | **1.90×** |
| 1280×720 | 18.11 ms | 35.47 ms | **1.96×** |

**对 NPU 效率：** 同样跑完一帧，INT8 占用核时间约为 FP16 的一半 → 相同输入下 NPU% 从 ~30% 降到 ~16%（不是变差，是 **单帧更省算力**）。  
**对控制：** frame_age 从约 77–89 ms 降到约 61–67 ms。

---

### 步骤 6：用 6 组实测回答“要不要继续加 Context / 分辨率”

**决策树（实测验证）：**

```text
若 Input ≈ Process 且 drop 很少：
  → 没有更多帧可喂，加 Context 收益小   ✅ 当前 6 组全部命中

若 Input ≫ Process 且 NPU 仍低、drop 涨：
  → 值得测 3→6 Context

若目标是更低单帧延迟：
  → 模型/INT8/输入尺寸/算子，而不是加 Context   ✅ 与实测一致
```

**分辨率结论：**

- 模型输入固定 640×640 → **run 几乎不随相机分辨率变**。  
- 升高分辨率主要增加 **preprocess** 与 **frame_age**，以及 USB/DDR 压力。  
- 实时控制默认推荐：**Color/Depth 640×480 + INT8**。

---

### 步骤 7：效率提升的“账本”总结

| 阶段 | CPU 效率如何变好 | NPU 效率如何变好 |
| --- | --- | --- |
| 解耦 + 队列 | 回调不堵、线程职责清晰 | 可持续供数，减少空等 |
| 三核并行 | Worker 分摊预/后处理 | 多帧并行，容量 > 输入 |
| 绑大核 + 减负 | 抖动↓、无效像素/日志↓ | 喂核更平滑 |
| native zero-copy | Input Prepare 大降 | NPU 前等待缩短，有效 run 占比↑ |
| INT8 | 后处理仍轻 | **run 近乎减半**，同输入下更省核时 |
| 分辨率管控 | 高分辨率 pre 上涨可控 | 不误以为“分辨率= NPU 更重” |

---

## 5. 六组实测：分辨率 × INT8/FP16

### 5.1 测试方法

| 项 | 设置 |
| --- | --- |
| 板卡 | Orange Pi 5 Ultra @ `192.168.31.45` |
| Color profile | 与 Depth 同步切换 |
| 分辨率 | 640×480 / 848×480 / 1280×720，标称 30 FPS |
| 模型 | `package_qrcode_shelf_tag_i8.rknn` / `_fp16.rknn` |
| 节点参数 | `debug_view:=false`，`enable_rknn:=true` |
| 每组时长 | 约 45 s 有效推理 + 预热剔除前 5 个样本 |
| 有效样本 | 约 58–59 行 `RKNN_BASELINE` / 组 |
| NPU 占用 | 每秒读 `/sys/kernel/debug/rknpu/load` |
| 脚本 | `/tmp/rknn_res_model_bench.sh` |

### 5.2 INT8 三分辨率

| 分辨率 | Input | Process | run (ms) | det合计 (ms) | pre (ms) | frame_age (ms) | Capacity | NPU% c0/c1/c2 | drop | stale |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: |
| 640×480 | 29.82 | 29.82 | **17.97** | 19.78 | 0.28 | 61.1 | 167.5 | 16/16/16 | 0 | 0 |
| 848×480 | 28.57 | 28.55 | **17.60** | 20.75 | 1.71 | 63.4 | 169.1 | 15/15/15 | 0 | 0 |
| 1280×720 | 28.96 | 28.98 | **18.11** | 20.48 | 1.01 | 67.3 | 166.2 | 15/16/16 | 0 | 0 |

三核 `core_med_ms`：

| 分辨率 | med Core0/1/2 | p95 Core0/1/2 |
| --- | --- | --- |
| 640×480 | 17.24 / 17.63 / 17.63 | 20.47 / 19.70 / 19.27 |
| 848×480 | 17.16 / 17.60 / 17.63 | 20.98 / 20.04 / 19.51 |
| 1280×720 | 17.58 / 17.91 / 17.97 | 19.06 / 19.40 / 19.35 |

### 5.3 FP16 三分辨率

| 分辨率 | Input | Process | run (ms) | det合计 (ms) | pre (ms) | frame_age (ms) | Capacity | NPU% c0/c1/c2 | drop | stale |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: |
| 640×480 | 29.06 | 29.08 | **32.54** | 35.68 | 0.31 | 77.1 | 91.9 | 29/29/30 | 0 | 0 |
| 848×480 | 29.46 | 29.46 | **33.48** | 38.11 | 1.80 | 80.0 | 89.4 | 29/30/30 | 0 | 0 |
| 1280×720 | 28.94 | 28.94 | **35.47** | 39.80 | 1.27 | 89.0 | 84.0 | 30/31/31 | 0 | 0 |

三核 `core_med_ms`：

| 分辨率 | med Core0/1/2 | p95 Core0/1/2 |
| --- | --- | --- |
| 640×480 | 32.23 / 32.67 / 32.63 | 34.13 / 34.72 / 34.87 |
| 848×480 | 32.61 / 33.74 / 33.66 | 35.07 / 36.88 / 36.49 |
| 1280×720 | 35.15 / 35.27 / 35.47 | 38.08 / 38.92 / 39.54 |

### 5.4 INT8 vs FP16（同分辨率）

| 分辨率 | INT8 run | FP16 run | 比值 | INT8 age | FP16 age | INT8 NPU均 | FP16 NPU均 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 640×480 | 17.97 | 32.54 | 1.81× | 61.1 | 77.1 | 16.0% | 29.5% |
| 848×480 | 17.60 | 33.48 | 1.90× | 63.4 | 80.0 | 15.2% | 29.9% |
| 1280×720 | 18.11 | 35.47 | 1.96× | 67.3 | 89.0 | 15.6% | 30.7% |

### 5.5 内存占用（启动日志）

| 模型 | weight | internal | dma | zero-copy |
| --- | ---: | ---: | ---: | --- |
| INT8 | 7.6 MiB | 12.9 MiB | 24.4 MiB | int8 |
| FP16 | 14.8 MiB | 49.2 MiB | 68.7 MiB | fp16 |

> 以上为三 Context 汇总量级（日志打印值），INT8 内存明显更省。

### 5.6 本轮实测结论

1. **量化是第一杠杆**：INT8 全面优于 FP16（延迟、容量、内存）。  
2. **分辨率是第二杠杆（弱）**：主要影响 pre 与 age，不显著改变 run。  
3. **并行度已足够当前 30 FPS**：六组皆 Input≈Process、drop=stale=0。  
4. **NPU 低占用 ≠ 失败**：INT8 16% 是“输入只有 30、单帧 18ms”的正常结果。  
5. **默认工程推荐**：`640×480 + INT8 + debug_view=false`。

### 5.7 粗算：为什么 INT8 三核约 16%

```text
有效 NPU 时间占比 ≈ (Input_FPS × rknn_run_ms) / (1000 × 3)
INT8 @640: 29.82 × 17.97 / 3000 ≈ 17.9%  → 与实测 16% 同量级
FP16 @640: 29.06 × 32.54 / 3000 ≈ 31.5% → 与实测 29–30% 同量级
```

---

## 6. 测过但未采用 / 明确否决的方向

| 方向 | 结果 | 原因 |
| --- | --- | --- |
| `RKNN_FLAG_ENABLE_SRAM` | 几乎无收益 | 内存报告 SRAM=0，中位数仅 ~0.38% |
| 对 FP16 模型 UINT8 zero-copy | 端到端无收益 | 格式转换转移到 run |
| 单 Context `CORE_0_1_2` 想砍单帧延迟 | 不成立 | Runtime 不会 magically 把一帧均分三核 |
| 直接 12 Worker | 不适合当前业务 | 参考工程离线视频压测；我们是 30 FPS 实时控制 |
| 当前阶段 RGA | 暂缓 | 640 下 pre≈0.3ms，ROI 太小 |
| 严格按 Frame ID 重排序 | 否决 | 放大长尾，伤害控制时效 |
| 当前阶段 3→6 Context | 不优先 | 6 组证明输入侧已饱和匹配 |

---

## 7. 后期还能怎么提高性能

按 **收益期望 × 风险 × 前置条件** 排序。

### 7.1 模型与算子层（最高优先级）

#### （1）算子筛选与 NPU 友好结构

目标：减少 **CPU fallback**、多余 layout convert、低效算子。

建议流程：

```text
1. 导出 ONNX 时固定 opset，避免动态 shape
2. RKNN 转换日志检查：
   - 哪些层落到 CPU
   - 是否出现频繁 transpose / reshape / cast
3. 用 Toolkit2 perf_debug / eval_perf 看层耗时
4. 替换不友好结构：
   - 某些激活 / 注意力 / 自定义插件
   - 过大的 Detect head / 过多输出分支
5. 重新量化与回归检测精度
```

对当前 YOLO11 9 输出路径的特别点：

- 三组 box/class/sum 是否都必要；未使用输出能否在导出阶段裁掉；  
- DFL decode 是否可简化或部分下沉；  
- 确认 INT8 校准集覆盖现场光照、距离、货架角度。

#### （2）量化策略

| 策略 | 说明 | 风险 |
| --- | --- | --- |
| 全 INT8 | 当前主路径 | 小目标召回需验证 |
| 混合精度（敏感层 FP16） | 保精度 | run 可能回升 |
| per-channel / 更好校准 | 提精度 | 转换时间与工具链版本 |
| 输入输出量化一致性 | 与 zero-copy 布局绑定 | 错 zp/scale 会静默错检 |

#### （3）输入尺寸与模型规格

| 选项 | 预期 | 注意 |
| --- | --- | --- |
| 640 → 512/416/320 | run 显著下降 | 远距离二维码/货架标签召回 |
| 更小 backbone（n/s） | 延迟↓ | mAP |
| 剪枝 / 蒸馏 | 中长期 | 训练成本 |

### 7.2 推理策略层（业务可接受时）

| 策略 | 思路 | 适用 |
| --- | --- | --- |
| **ROI 推理** | 只在感兴趣区域跑检测 | 已知货架区域 / 跟踪框扩大 |
| **隔帧推理** | 检测 15 Hz，跟踪补间 | 控制允许稍低检测率 |
| **级联** | 小模型粗检 → 大模型精检 | 稀目标场景 |
| **类别门控** | 只要 qrcode 时关掉 package 分支（若可导出） | 任务单一阶段 |
| **动态分辨率** | 远距离升分辨率，近距离降 | 需状态机 |
| **结果复用** | 静止悬停时降频 | 配合 hover 话题 |

这些策略提升的是 **系统有效效率**（单位任务耗时），不一定提高 NPU%。

### 7.3 运行时与并行层（有条件再做）

| 技术 | 进入条件 | 预期 |
| --- | --- | --- |
| Context 3→6 | Input≫Process 且三核有空档、drop 涨 | 高输入吞吐；单帧延迟未必降 |
| `rknn_dup_context` | 内存吃紧 | 减权重副本，不直接提 FPS |
| 输出 zero-copy / raw INT8 后处理 | out+post 成为明显占比 | 当前 out+post 仅约 1–2 ms，优先级低 |
| RGA letterbox | pre 稳定 >3–5 ms 或升到高分辨率高帧率 | 释放 CPU |
| CPU/NPU/DDR 定频 | 做可重复 benchmark | 降抖动；注意温升 |

### 7.4 系统与工程层

- 温度 / 降频长稳测试（10–30 min），避免短测乐观。  
- USB3 带宽：1280p 时相机本身可能抖，先看 `ros2 topic hz`。  
- ZBar / JPEG / 调试预览与检测线程隔离或降频。  
- 业务路径：无目标时跳过深度投影与部分日志。

### 7.5 推荐的下一阶段实施顺序

```text
P0  固定默认：640x480 + INT8 + debug_view=false
P1  INT8 精度回归（召回/误检/距离分层）
P2  转换日志 + 层耗时，清 CPU fallback / 冗余输出
P3  评估 640→416/512 的精度-延迟帕累托
P4  业务策略：ROI / 隔帧 / 悬停降频
P5  仅当输入压力上升：再评估 3→6 Context 与 RGA
```

---

## 8. 还可以补充进本报告的内容（建议清单）

下列内容当前文档未完全展开，后续版本建议追加：

### 8.1 精度与业务指标

- [ ] 同场景 INT8 vs FP16 的 **召回率 / 误检率 / 漏检距离曲线**  
- [ ] 二维码解码成功率（检测框有了 ≠ ZBar 成功）  
- [ ] Depth 有效率（soft_ok 比例、过期降级比例）

### 8.2 更完整的性能画像

- [ ] 每组的 **P50/P95/P99** `frame_age` 与 `rknn_run` 分布（不仅均值）  
- [ ] CPU 占用（整进程 + 各线程）、DDR 带宽、温度曲线  
- [ ] 功耗（若板端可测）  
- [ ] 长稳 10/30 分钟热衰减

### 8.3 版本与可复现

- [ ] git commit / 模型 SHA256 / Runtime 与驱动版本表  
- [ ] 一键复现脚本入库（现板端 `/tmp/rknn_res_model_bench.sh`）  
- [ ] 固定测试挂图 / bag，避免现场画面变化污染 A/B

### 8.4 模型侧附件

- [ ] RKNN 转换配置（mean/std、quant_img_RGB、optimization_level）  
- [ ] 校准集构成说明  
- [ ] 层耗时 top10 表（perf_debug）  
- [ ] CPU fallback 层列表

### 8.5 对照实验

- [ ] 单 Worker vs 三 Worker（同模型）  
- [ ] zero-copy on/off  
- [ ] debug_view on/off  
- [ ] 不同 conf/nms 对 post 与业务召回的影响  

### 8.6 架构图与故障手册

- [ ] 一张“延迟构成饼图”（run vs 其他）  
- [ ] 常见故障：NPU 0%、drop 狂涨、age 飙升、三核不均  
- [ ] 决策树海报版（给现场同学）

---

## 9. 关键代码与日志索引

| 主题 | 位置 |
| --- | --- |
| CPU4–7 亲和 | `src/main.cpp` |
| 三 Worker / 队列 / STALE | `include/drone_perception/qr_vision_node.hpp`，`src/qr_vision_node.cpp` |
| RKNN detector / zero-copy | `src/rknn_yolo_detector.cpp` |
| 基线日志 `RKNN_BASELINE` | `src/qr_vision_node.cpp` |
| 性能监控脚本 | `scripts/monitor_yolo_npu_perf.sh` |
| 历史技术日志 | `docs/rknn_runtime_performance_optimization_log.md` |
| 本次 6 组原始数据 | 板端 `/tmp/rknn_res_model_compare_20260714_014417/` |

---

## 10. 最终推荐配置（工程默认）

```text
相机 Color/Depth : 640x480@30
模型             : package_qrcode_shelf_tag_i8.rknn
并行             : 3 Worker / 3 Context / Core0-1-2
CPU              : affinity 4-7
输入             : native INT8 zero-copy
显示             : debug_view=false（性能与控制）
队列             : capacity=3，丢旧 + STALE
Depth            : 软对齐 50ms，过期降级 2D
```

**预期量级（本轮实测）：**

```text
Input/Process ≈ 29–30 FPS
rknn_run      ≈ 18 ms
frame_age     ≈ 60–65 ms
NPU 三核      ≈ 15–16%
drop/stale    ≈ 0
Capacity      ≈ 160+ FPS
```

---

## 11. 结语

我们提升 CPU / NPU 效率的路径不是“把占用率拉满”，而是：

1. **先保证实时语义正确**（最新帧、不堵回调、不积压）；  
2. **再让 CPU 少做无效功**（亲和、转换、日志、缓冲）；  
3. **再让 NPU 少做无效搬运**（native zero-copy）；  
4. **最后动模型本身**（INT8 / 算子 / 尺寸）——这才是单帧延迟的主战场；  
5. **用 6 组实测证明**：当前阶段继续堆 Context 收益有限，应把精力转向精度回归与模型/策略层。

后续若输入帧率提高、或单帧模型再变重，再回头评估更多 Context 与 RGA；在那之前，**量化质量、算子映射、推理策略** 是更划算的优化面。
