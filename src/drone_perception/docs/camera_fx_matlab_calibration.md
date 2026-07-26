# 相机内参 MATLAB 标定流程（camera_fx/fy/cx/cy）

> 目标：用 MATLAB Camera Calibrator 把 `config/industrial_default.yaml` 中按水平视场角 70° 估算的名义内参（`camera_fx/fy=914`、`cx/cy=640/360`）替换为实测值。fx 直接决定反投影 `ground=()` 的缩放精度：估算值误差可达 10~20%，标定后可进 1% 以内。
>
> ⚠ 关键约束：本相机（12MP U3 Camera，32e4:6577）可调变焦，**fx/fy 随焦距变化**——每个焦距档位对应一套内参，调焦后必须重新标定。旧飞行数据若跨了焦距档位，不可混用。

## 1. 原理一句话

fx 的物理含义是"镜头放大率的像素表达"：距离 Z 处宽 X 米的物体，在画面上占 `fx·X/Z` 个像素。标定就是把一块**尺寸精确已知**的棋盘格摆出多个距离和倾角拍下来，让 MATLAB 反推出唯一能解释所有照片的 fx/fy/cx/cy（顺带给出畸变系数 k1/k2）。

## 2. 材料清单

| 物品 | 说明 |
|------|------|
| 棋盘格 PDF | PC 桌面 `标定棋盘格_A4_25mm.pdf`：7×10 方格、名义边长 25mm、内角点 6×9（MATLAB 要求一边奇数格、一边偶数格，此板已满足） |
| 硬质平板 | 剪贴板 / 亚克力板 / 硬纸板均可，棋盘必须完全平整无褶皱 |
| 直尺 | 校验打印缩放 + 实测方格真实边长 |
| MATLAB | 需 Computer Vision Toolbox（App 列表里有 **Camera Calibrator** 即可） |
| 采图脚本 | 本包 `scripts/calib_capture.sh`，在香橙派上运行 |

## 3. 打印与制板

1. 打印时**必须选"实际大小 / 100% 缩放"**，严禁"适应页面"。
2. 用直尺量页面底部的校验线：必须恰好 **100mm**，否则这张纸作废、改设置重打。
3. 再横量 4 个连续方格取平均：得到**实测边长 S**（例如量出 99.6mm/4 → S=24.9mm）。后面 MATLAB 里填的就是这个实测值，**不是名义的 25**。
4. 把纸平贴在硬板上（四角胶带或整面胶棒），确认无气泡。

## 4. 板端采图

1. **停掉视觉节点**（含自启动拉起的实例），否则相机被占用：

   ```bash
   pkill -f 'ros2 launch drone_perception'; pkill -f industrial_animal_vision_node
   ```

2. **把变焦调到飞行档位**，贴胶带固定焦距环，并记下档位名（写进第 8 节档案表）。
3. 运行采图脚本（默认 40 张、间隔 3 秒、1280x720——与节点运行分辨率一致，这点不能改）：

   ```bash
   cd ~/drone_ws
   chmod +x src/drone_perception/scripts/calib_capture.sh   # 首次
   ./src/drone_perception/scripts/calib_capture.sh
   ```

   若 `git pull` 后报 `bad interpreter: ^M`，是 Windows 换行符问题：`sed -i 's/\r$//' src/drone_perception/scripts/calib_capture.sh`。

4. 拍摄姿态覆盖清单（每两张之间挪动棋盘，**倒计时到点前保持静止**，避免运动模糊）：
   - 距离：近、中、远各几张（棋盘占画面 1/3 ~ 2/3）；
   - 位置：画面中心 + 四个角落都要出现过（角落姿态是标 cx/cy 和畸变的关键）；
   - 倾角：前后、左右各倾斜 30~45°，再加几张平面内旋转的。
5. 拍完在 PC（PowerShell）上取回，脚本结束时会打印现成命令：

   ```powershell
   scp -r orangepi@192.168.46.168:~/calib_imgs/<时间戳目录> D:/calib_imgs/
   ```

## 5. MATLAB 标定步骤

1. MATLAB → 顶部 **APP** 选项卡 → **Camera Calibrator**。
2. **Add Images** → 全选取回的 `img_*.jpg` → 弹窗里 Pattern 选 **Checkerboard**，**Square Size 填第 3 节量出的实测边长 S**（单位 mm）。
3. 自动角点检测：有图被拒（rejected）是正常的，可用图 **≥15 张**即可继续；不足就回板上补拍。
4. 工具栏 Options：Radial Distortion 选 **2 Coefficients**，Skew 和 Tangential Distortion 不勾（本用途够用，参数越少越稳）。
5. 点 **Calibrate** → 看右下 **Reprojection Errors** 柱状图：
   - 平均误差 **< 0.5 px** 合格；
   - 个别图 > 1 px 的，选中删除后点 **Recalibrate**。
6. 点 **Export Camera Parameters** → 工作区得到变量 `cameraParams`，命令行读数：

   ```matlab
   cameraParams.Intrinsics.FocalLength       % [fx fy]，单位像素
   cameraParams.Intrinsics.PrincipalPoint    % [cx cy]（注意：1 基坐标）
   cameraParams.Intrinsics.ImageSize         % 必须是 [720 1280]，否则采图分辨率错了，作废重拍
   cameraParams.Intrinsics.RadialDistortion  % [k1 k2]，记入档案表
   ```

   MATLAB 像素坐标从 1 数起，填 yaml 时 cx/cy 各**减 1**（差 1 像素对地面换算影响可忽略，但按规矩来）。

## 6. 写回与生效

1. 改 `config/industrial_default.yaml` 的 `camera_fx / camera_fy / camera_cx / camera_cy`（101~105 行一带），并把上方"名义内参（未标定）"的警告注释更新为：`MATLAB 标定 @ <焦距档位> <日期>，重投影误差 <x.xx>px`。
2. 提交推送后板上生效（yaml 要装进 install，必须重新 build）：

   ```bash
   cd ~/drone_ws && git pull && colcon build --packages-select drone_perception && source install/setup.bash
   ```

3. 快速验证（任选其一）：
   - 静态：相机固定在已知高度 H 俯视，地面两记号实距 D、画面像素距 d，应有 `fx ≈ d·H/D`，与标定值对比；
   - 飞行：升降高度时观察周期日志 `ground=()`，标定正确则地面坐标不再随高度漂移。

## 7. 常见坑

| 坑 | 后果 | 预防 |
|----|------|------|
| 打印选了"适应页面" | 方格实际不是 25mm，fx 整体偏比例 | 100mm 校验线 + 实测 S |
| 快门瞬间棋盘在动 | 运动模糊，角点检测被拒或误差大 | 间隔倒计时内摆好后**停稳** |
| 调焦后没重标 | fx 直接作废，ground 缩放漂移 | 焦距环贴胶带；档案表按档位记录 |
| 光线太暗 / 逆光 | AE 拉长曝光加剧模糊 | 白天顺光环境拍 |
| \|k1\| > 0.1 | 节点是纯针孔模型不修畸变，画面边缘换算偏差放大 | 伺服目标尽量在画面中部；偏差不可接受时再考虑节点内加去畸变 |
| 用别的分辨率采图 | fx 与运行分辨率不匹配 | 脚本已固定 1280x720，别改；`ImageSize` 读数兜底检查 |
| 标定与飞行的 `zoom_absolute` 不一致 | UVC 数字变焦直接乘进有效 fx（yaml 当前 120 → ×1.2） | 采图前把 zoom_absolute 设成飞行值并记入档案表；范围 100~200 |

## 8. 标定档案（每标一次填一行）

| 日期 | 焦距档位 | fx | fy | cx | cy | k1 | k2 | 重投影误差(px) | 可用图数 |
|------|----------|----|----|----|----|----|----|----------------|----------|
| 2026-07-26 | —（估算） | 914 | 914 | 640 | 360 | — | — | — | 名义值，HFOV 70° 假设 |
|  |  |  |  |  |  |  |  |  |  |

## 附录：出厂规格与 fx 理论核对（2026-07-27 补充）

出厂规格（RER-U3CAM12MP01）关键项：

| 项目 | 值 |
|------|-----|
| 传感器 | SONY IMX577，1/2.3"，4000×3000，像元 1.55μm（感光面 6.2×4.65mm） |
| 快门 | 电子卷帘（rolling shutter），拍标定图时棋盘务必静止 |
| 镜头 | ZONTOP M12 **5-50mm 手动变焦**，f/1.4，像面 1/2.7"，M12×0.5。原厂"无畸变、对角约 100°"定焦已拆下 |
| 720p 帧率上限 | USB3.0 下 MJPEG 230fps（当前节点按功耗/稳定性取 60fps，勿因此改） |

镜头三个注意点：

1. **双环联动**：变焦环动过之后焦点会跑，正确顺序是"定变焦 → 重新对焦 → 两个环都贴胶带"。标定绑定的是这一对环位置的组合。
2. **像面圆偏小**：镜头按 1/2.7"（对角约 6.7mm）设计，传感器是 1/2.3"（对角约 7.8mm）——广角端四角出现暗角/画质下降属正常。标定图角落若频繁检测失败，多拍"棋盘在画面中带偏内"的姿态补齐即可。
3. **标称"58°-20°"不直接适用**：那是按 1/2.7" 传感器标的视场角，本机实际视场取决于 720p 读出口径，以标定实测为准。

**fx 理论核对公式**（用于给 MATLAB 结果做交叉验证，不能代替标定）：

若 1280×720 输出是全幅 4000 宽等比缩放，则等效像元 = 1.55μm × 4000/1280 ≈ 4.84μm：

```text
fx ≈ 镜头焦距 f(mm) / 0.00484mm ≈ 206.5 × f(mm)
本镜头范围：f=5mm → fx≈1033    f=10mm → fx≈2065    f=25mm → fx≈5163    f=50mm → fx≈10326
```

节点侧 `camera_fx/fy` 校验上限已从 10000 放宽到 50000（2026-07-27），长焦端与裁切口径的实测值都容得下。

⚠ 高帧率模式可能不是全幅缩放而是 binning/中心裁切，等效像元随之不同。判别方法：
标定后算 `fx实测 ÷ 镜头标称焦距`——结果 ≈207 说明全幅缩放；明显更大（400~650）说明
存在裁切，此时实际视场比镜头标称更窄，属正常现象，以实测 fx 为准即可。

推论：原厂镜头对角 100°（水平约 87°）对应 fx≈670；换长焦后真实 fx 大概率是名义值
914 的数倍——**标定前 `ground=()` 的偏移量会被等比例放大数倍，不可用于决策**。
