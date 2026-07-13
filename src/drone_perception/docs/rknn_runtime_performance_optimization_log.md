# RKNN 实时推理性能优化技术日志

## 1. 文档定位

本文记录 Orange Pi 5 Ultra 上 D435i + RKNN 实时检测链路的最终优化结果，重点说明采用了什么技术、为什么有效、实测收益和适用边界。本文不是逐行代码说明，也不替代早期调研文档 [`yolo_npu_fps_optimization.md`](./yolo_npu_fps_optimization.md)。

早期文档包含不同模型、不同运行路径和不同优化阶段的测量值；本文以当前 `qr_rk3588_FP16.rknn`、三 RKNN Context、原生 FP16 zero-copy 实现为最终基线。不同测试轮次的绝对耗时不能直接横向相减，优化判断应以同一设备、模型、场景和统计口径下的 A/B 数据为准。

## 2. 硬件、软件与测量口径

### 2.1 基线环境

| 项目 | 基线 |
| --- | --- |
| 主机 | Orange Pi 5 Ultra |
| SoC | Rockchip RK3588，3 个 NPU 核心 |
| RKNN Runtime | 2.3.2 |
| RKNPU 驱动 | 0.9.6 |
| 相机 | Intel RealSense D435i |
| Color / Depth | 640 x 480 @ 30 FPS |
| 当前模型 | `qr_rk3588_FP16.rknn` |
| 模型输入 | 640 x 640，原生 FP16，NHWC |
| 模型输出 | 单输出 `[1, 6, 8400]` |
| 并行方式 | 3 个 RKNN Context，分别绑定 NPU Core0、Core1、Core2 |

### 2.2 FPS 与延迟不能混为一谈

| 指标 | 含义 | 当前链路中的用途 |
| --- | --- | --- |
| Input FPS | Color topic 实际到达频率 | 判断相机或 ROS 输入是否限速 |
| Process FPS | 所有 Worker 每秒完成的帧数总和 | 判断整条并行推理链路实际处理吞吐 |
| Display FPS | UI 每秒展示的新结果数 | 受 UI、结果覆盖、相机帧率共同限制 |
| NPU Capacity FPS | `sum(1000 / 各核心 rknn_run_ms)` | 估计三核心满负载时的模型吞吐上限 |
| 单帧延迟 | 一帧从预处理到检测结果完成的时间 | 决定实时控制拿到结果的最早时间 |

三核心约 70 FPS 的容量不代表单帧只需约 14 ms。三个 Context 是同时处理三帧，每帧仍需在一个 NPU 核心上执行约 41 ms。因此它提高的是多帧吞吐，不会把一帧平均拆到三个核心来降低单帧延迟。

## 3. 已采用且有效的技术

### 3.1 ROS 回调与推理解耦

相机回调只接收消息、组合当前可用的 Color/Depth/标定信息并投递任务，不在回调线程中执行 `rknn_run()`。三个独立 Worker 从有界队列取帧推理，UI 另有独立线程。

这种结构的收益不是让单次 NPU 运算变快，而是避免约 42 ms 的推理阻塞 ROS 相机回调。输入持续到达时，回调仍可及时接收新帧，推理线程也能持续喂满 NPU。

适用条件：输入速率接近或高于单 Context 的处理能力，并且业务更重视“拿到较新的结果”而不是“每一帧必须处理”。

### 3.2 原始 Color/Depth 分流

当前节点分别订阅：

- `/camera/camera/color/image_raw`
- `/camera/camera/depth/image_rect_raw`
- Color/Depth `camera_info`
- `/camera/camera/extrinsics/depth_to_color`

Color 到达即触发检测任务，Depth 使用最近一帧并检查时间差；节点根据内参和外参把彩色像素投影到原始深度图取值。这样绕过了 `RGBD + align + sync` 聚合路径，避免其中任一流短暂停顿时整组消息无法交付。

边界：原始流分流不等于忽略同步。代码仍以 50 ms 为最大 Color/Depth 时间偏差，标定缺失或 Depth 过期时只继续 2D 检测，不发布伪造或旧的深度结果。

### 3.3 绑定 CPU4-7 大核

进程启动时通过 `sched_setaffinity()` 绑定 CPU4-7。RK3588 的这些性能核心承担 ROS 消息转换、OpenCV 预处理、后处理、深度投影和 UI 工作，可减少线程在大小核之间迁移造成的调度抖动。

该措施主要改善稳定性和长尾延迟，不应被解释为 NPU 算子本身加速。若系统中 CPU4-7 同时运行高负载任务，仍可能发生竞争。

### 3.4 三 Context 分别绑定三个 NPU 核心

节点创建三个独立 `RknnYoloDetector`，其 Core Mask 分别为：

```text
Context 0 -> RKNN_NPU_CORE_0
Context 1 -> RKNN_NPU_CORE_1
Context 2 -> RKNN_NPU_CORE_2
```

每个 Context 只由对应 Worker 使用，避免同一个 RKNN Context 被多线程并发调用。三帧可在三个 NPU 核心上并行执行，实测总 NPU Capacity 约 70 FPS，高于 D435i 的 30 FPS 输入，因此当前吞吐能力足够。

代价是模型权重、内部区和 DMA 内存按 Context 分配。当前代码会汇总并显示三 Context 的内存占用，后续可评估 `rknn_dup_context()` 降低权重重复占用。

### 3.5 容量 3 的有界队列与 STALE 保护

任务队列容量固定为 3。队列已满时先删除最旧任务，再插入新帧。这是一种“保留最新数据”的有界策略，可避免 NPU 短时落后时形成不断增长的旧帧积压。

每个任务携带单调递增的 Frame ID。由于三核心完成顺序不确定，结果写入和 UI 取出时均检查 Frame ID：

- 不晚于已显示帧的结果计为 `stale_result` 并丢弃；
- 不晚于当前待显示结果的结果也丢弃；
- UI 始终消费当前最新的已完成结果。

这保证“慢帧不会覆盖新帧”。代价是 Process FPS 可以高于 Display FPS，部分已完成推理会因过期而不展示。这是低延迟实时链路的主动取舍。

### 3.6 精简逐检测框日志

检测框级别的高频日志会产生字符串格式化、日志锁竞争和终端 I/O，目标多时尤其明显。当前链路保留每秒一次的聚合性能日志和画面状态，避免为每个框持续输出日志。

该优化主要减少 CPU 抖动和终端阻塞。诊断单个检测结果时可以临时增加日志，但不应作为长期性能基线配置。

### 3.7 去除重复 RGB/BGR 转换

相机 Color 消息通过 `cv_bridge` 直接转换为模型需要的 `rgb8`，RGB 数据直接进入 letterbox 和 RKNN 输入准备。只有 UI 展示前才执行一次 RGB 到 BGR 转换。

这避免了“相机 BGR -> 模型 RGB -> 显示 BGR”路径中的无效重复转换，同时保持 OpenCV UI 的颜色正确。

### 3.8 复用预处理缓冲

`resized_buffer_` 和 `letterbox_buffer_` 作为 Detector 成员复用；每帧使用 `cv::Mat::create()` 按需调整容量，而不是反复创建临时大缓冲。letterbox 的 640 x 640 输出同样复用。

这降低了动态内存分配次数和分配器抖动。它不会消除 resize 和填充本身的计算，但可让预处理耗时更稳定。

### 3.9 原生 FP16 zero-copy 输入

当前模型的 native input 是 FP16/NHWC。初始化阶段执行：

1. `RKNN_QUERY_NATIVE_INPUT_ATTR` 获取原生输入属性和 stride；
2. `rknn_create_mem()` 创建 RKNN 共享输入内存；
3. 设置 `pass_through = 1`；
4. `rknn_set_io_mem()` 将该内存绑定为 Context 输入；
5. 用带 stride 的 `cv::Mat` 直接映射 `virt_addr`；
6. 每帧把 letterbox 图像归一化并转换为 FP16，直接写入映射内存；
7. 调用 `rknn_mem_sync(..., RKNN_MEMORY_SYNC_TO_DEVICE)` 后执行 `rknn_run()`。

这里的 zero-copy 是指输入张量不再通过 `rknn_inputs_set()` 进行额外提交、格式转换或内部复制，并不表示相机图像到 FP16 张量的 resize、letterbox、归一化和类型转换全部消失。

若 Runtime 报告的 native input 不是 4 维 FP16/NHWC、末维不是 3，代码会明确回退到 UINT8 `rknn_inputs_set()`，而不是按错误布局写内存。

### 3.10 关键实测收益

在 Orange Pi 5 Ultra 最终 FP16 链路的同条件对比中：

| 指标 | 优化前 | 原生 FP16 zero-copy 后 | 变化 |
| --- | ---: | ---: | ---: |
| Input Prepare | 约 11 ms | 约 0.7 ms | 减少约 10 ms |
| Detector Total | 约 53 ms | 约 43 ms | 减少约 10 ms |

这说明当 FP16 模型使用普通输入提交时，Runtime 侧转换或复制可能成为显著成本；绑定 native FP16 内存后，该成本基本被移除。收益与模型 native input 类型、Runtime 版本、stride 和输入准备方式绑定，不能直接外推到所有 RKNN 模型。

## 4. 测试过但未采用的技术

### 4.1 `RKNN_FLAG_ENABLE_SRAM`

开启后输出与基线完全一致，但 `RKNN_QUERY_MEM_SIZE` 报告的 SRAM 使用量为 0，推理中位数仅改善约 0.38%。这个量级小于运行抖动，不能证明模型实际使用了 SRAM，因此未纳入最终配置。

结论：API 返回成功不等于模型已把有效工作集放入 SRAM，必须同时检查内存报告和重复测量结果。

### 4.2 UINT8 zero-copy

UINT8 共享输入内存的 CPU 复制和 cache sync 较快，但随后 `rknn_run()` 增长，端到端没有收益。对于 native FP16 模型，UINT8 并不是原生输入格式，格式转换只是从输入提交阶段转移到 Runtime 执行阶段。

结论：zero-copy 的关键是匹配 native tensor，而不仅是使用 `rknn_create_mem()`。

### 4.3 FP16 `rknn_inputs_set()`

把 FP16 buffer 交给 `rknn_inputs_set()` 的输入准备仍约 12 ms，表明该 API 路径仍有转换或复制。它提供的是普通输入提交接口，不属于本项目最终采用的 I/O memory zero-copy 路径。

### 4.4 单 Context 使用 `RKNN_NPU_CORE_0_1_2`

该 Core Mask 允许 Runtime 使用三个 NPU 核心，但不能假设 Runtime 会把任意单帧模型平均切分到三核并将延迟缩短为三分之一。本模型实测中，它没有解决单帧延迟问题。

最终采用三个独立 Context 分别绑定单核，以多帧并行换取总吞吐。

### 4.5 RGA 预处理

当前 resize/letterbox 约 0.3 ms，已经不是主要瓶颈。即使 RGA 把这部分完全消除，理论收益上限也只有约 0.3 ms，却会引入 RGA buffer、stride、格式转换、同步和额外依赖的维护成本，因此暂不采用。

### 4.6 结果严格重排序

严格按 Frame ID 输出需要等待先提交但较慢的帧。一旦某核心出现长尾，更新的结果也会被阻塞，直接增加控制感知延迟。

当前保留“最新完成结果优先 + 旧结果丢弃”，不保证每帧展示，也不保证完成结果按提交顺序发布。

## 5. 尚未采用但可能有价值的技术

### 5.1 模型层优化

- **INT8 全量化**：使用与现场光照、尺度和二维码/条码分布一致的代表性数据集校准，并同时验证精度、召回率和 `rknn_run()`。
- **更小输入或更轻模型**：降低输入尺寸，或调整 Backbone/Head 和算子，通常比继续压缩亚毫秒 CPU 阶段更可能显著降低延迟。
- **检查 CPU fallback**：审查 RKNN 转换日志和逐层性能，确认是否存在未映射到 NPU 的算子、额外 layout 转换或 CPU 执行节点。
- **Runtime/驱动升级 A/B**：在相同模型、频率、温度和数据集下比较兼容性、正确性、延迟分布和内存；版本更新不等于必然加速。

### 5.2 数据搬运与后处理

- **RGA / Rockchip 2D**：未来相机分辨率或帧率提高、预处理重新成为瓶颈时，再用于 resize、颜色转换和填充。
- **`rknn_dup_context()`**：复用权重，降低三个 Context 的模型内存开销；采用前需验证 Runtime 2.3.2 下的线程隔离和销毁顺序。
- **输出 tensor zero-copy**：仅在 `rknn_outputs_get()` 明显成为瓶颈时评估。
- **raw INT8 后处理和 SIMD NMS**：仅在输出反量化或 NMS 成为显著耗时时采用，并先验证量化参数、输出布局和数值一致性。

### 5.3 系统稳定性

- NPU、CPU 和 DDR 定频，用于可重复基准和降低频率切换抖动；需评估功耗与温度。
- 持续监控温度、降频和供电，避免短时测试结论掩盖长时间热衰减。
- 在真正需要更低长尾时，再评估线程优先级、内存锁定和系统服务隔离；实时调度配置错误可能反而饿死 ROS 或 UI 线程。

## 6. 最终瓶颈判断

最终基线的典型分解为：

| 阶段 | 典型量级 |
| --- | ---: |
| resize / letterbox | 约 0.3 ms |
| FP16 Input Prepare + cache sync | 约 0.7 ms |
| output get + 后处理 | 亚毫秒级 |
| 除 `rknn_run()` 外合计 | 约 1-2 ms |
| `rknn_run()` | 约 41 ms |
| Detector Total | 约 42-43 ms |

`rknn_run()` 约占单帧检测时间的 96%，说明瓶颈已经从 ROS 回调、图像聚合、内存分配和输入提交转移到模型执行层。继续优化当前 CPU 预处理，即使做到完全免费，单帧延迟也只能减少约 1-2 ms。

三核心总吞吐约 70 FPS，高于 D435i 的 30 FPS，所以输入吞吐已足够；但任意一帧仍需约 42 ms 才能形成检测结果。后续若目标是降低控制闭环延迟，应优先优化模型计算量、量化和算子映射，而不是继续增加 Context 数量。

## 7. 当前完整数据流

```text
D435i 640x480@30 FPS
  |-- Color raw -----------------------> ROS Color callback
  |                                         |
  |                                         | 组合最新 Depth/标定快照
  |-- Depth raw ------> latest Depth -------+
  |-- CameraInfo -----> latest intrinsics --+
  `-- Extrinsics -----> latest extrinsics --+
                                            |
                                            v
                              容量 3 的有界任务队列
                              满时丢弃最旧 Frame ID
                                  /         |         \
                                 v          v          v
                            Worker 0    Worker 1    Worker 2
                            Context 0   Context 1   Context 2
                            NPU Core0   NPU Core1   NPU Core2
                                 \          |          /
                                  v         v         v
                              Frame ID / STALE 过滤
                                        |
                                        v
                         原始 Depth 投影取值与 3D 计算
                                        |
                                        v
                            最新结果 -> UI / 聚合日志
```

关键语义：Depth 是带时间差检查的最近快照；任务队列保留较新输入；三个 Worker 可乱序完成；STALE 过滤保证旧结果不能覆盖新结果；UI 和日志反映的是最新完成结果，而不是逐帧严格重放。

## 8. 复测工作流与判定标准

1. 固定模型、测试画面、相机分辨率和帧率，记录 Runtime、驱动、CPU/NPU/DDR 频率及温度。
2. 预热后连续采样，不使用单帧或单次运行值下结论；至少比较中位数和长尾。
3. 同时记录 Input FPS、Process FPS、Display FPS、NPU Capacity FPS，以及各阶段耗时。
4. 验证输出数值和检测结果一致性，性能提升不能以错误结果、旧深度或乱序覆盖为代价。
5. 检查 `queue_drop` 和 `stale_result`：少量丢弃是最新结果策略的预期现象，持续快速增长则说明处理能力或系统调度出现退化。
6. 任何 Runtime、驱动、模型或 Core Mask 变化都单独做 A/B，避免一次修改多个变量后无法归因。

当前验收基准是：30 FPS 相机输入下无无限积压；三 Context 总容量高于输入速率；旧结果不覆盖新结果；Depth 过期时降级为 2D；典型单帧 Detector Total 约 42-43 ms，其中 `rknn_run()` 约 41 ms。

## 9. 技术边界总结

- 当前架构解决了吞吐、输入停顿和旧帧积压问题，没有消除约 41 ms 的模型执行延迟。
- “三核约 70 FPS”是并行容量，不是单帧 70 FPS，也不是 Display FPS。
- “FP16 zero-copy”消除了普通输入提交路径的额外转换/复制，不消除图像预处理和 FP16 写入。
- 原始 Depth 分流提高了输入鲁棒性，但仍必须检查时间戳和标定，不能把任意最近深度当作同步深度。
- 当前下一阶段的主要优化空间在模型、量化和 NPU 算子映射，而不在亚毫秒级的 resize、输出或 NMS。
