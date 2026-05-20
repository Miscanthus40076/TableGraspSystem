# 手眼标定说明

这个目录用于做当前项目的**眼在手外手眼标定**：

- 相机固定在外部
- 标定板固定在机械臂末端
- 采集多组：
  - `T_base_ee`
  - `T_camera_board`
- 最终求解：
  - `T_base_camera`

当前主流程已经收敛为**手动采集 + 离线求解 + 一致性验证**，不再依赖深度自动对正或自动移动。

## 目录里的主要脚本

- `start_handeye_session.py`
  - 连续手动采样主入口
- `collect_handeye_sample.py`
  - 单条样本采集
- `solve_eye_to_hand.py`
  - 用采样结果求手眼矩阵
- `validate_eye_to_hand.py`
  - 验证求出的矩阵是否自洽

## 1. 采集前准备

### 1.1 启动 runtime bridge

```bash
source /opt/ros/humble/setup.zsh
source ~/starai_ws/install/setup.zsh
/usr/bin/python3 /home/misca/TabletopGraspSystem/robot/runtime_bridge.py
```

如果 bridge 已经在运行，就不要重复启动。

### 1.2 准备标定板

- 标定板要刚性固定在末端
- 固定后不要相对末端滑动
- 当前物理板是**裁下来的 6x6 子板**
- 只有完整识别到 `25/25` 个内角点时，才视为有效样本

### 1.3 Python 环境

采集和求解默认都在 `tgs` 环境里运行：

```bash
source /home/misca/miniconda3/etc/profile.d/conda.sh
conda activate tgs
```

## 2. 连续手动采样

### 2.1 推荐命令

```bash
cd /home/misca/TabletopGraspSystem
source /home/misca/miniconda3/etc/profile.d/conda.sh
conda activate tgs
python calibrate_by_handeye/start_handeye_session.py \
  --serial 419522072950 \
  --start-full-stack \
  --color-width 1280 \
  --color-height 720 \
  --color-fps 15 \
  --depth-width 640 \
  --depth-height 480 \
  --depth-fps 15 \
  --dictionary DICT_4X4_50 \
  --squares-x 11 \
  --squares-y 8 \
  --square-length-mm 15 \
  --marker-length-mm 11 \
  --save-overlay
```

### 2.2 如果要失能舵机手动拖动

```bash
cd /home/misca/TabletopGraspSystem
source /home/misca/miniconda3/etc/profile.d/conda.sh
conda activate tgs
python calibrate_by_handeye/start_handeye_session.py \
  --serial 419522072950 \
  --disable-servos \
  --color-width 1280 \
  --color-height 720 \
  --color-fps 15 \
  --depth-width 640 \
  --depth-height 480 \
  --depth-fps 15 \
  --dictionary DICT_4X4_50 \
  --squares-x 11 \
  --squares-y 8 \
  --square-length-mm 15 \
  --marker-length-mm 11 \
  --save-overlay
```

这个模式会切到 `driver_free`，适合你手动摆位采样。

### 2.3 会话中的控制

- `s`
  - 保存当前样本
- `q`
  - 退出会话

### 2.4 保存规则

只有满足下面条件，按 `s` 才会落盘：

- `pose_ok = true`
- `pose_source = charuco`
- `charuco_corner_count = 25`

也就是说：

- 必须完整看到这块 `6x6` 板
- 不接受部分角点样本

### 2.5 样本保存位置

样本保存在：

- `calibrate_by_handeye/samples/session_xxxxxx/`

每条样本目录里一般有：

- `color.png`
- `overlay.png`（如果加了 `--save-overlay`）
- `sample.json`

## 3. 单条样本采集

如果你只想手工保存一条样本：

```bash
cd /home/misca/TabletopGraspSystem
source /home/misca/miniconda3/etc/profile.d/conda.sh
conda activate tgs
python calibrate_by_handeye/collect_handeye_sample.py \
  --serial 419522072950 \
  --start-full-stack \
  --color-width 1280 \
  --color-height 720 \
  --color-fps 15 \
  --depth-width 640 \
  --depth-height 480 \
  --depth-fps 15 \
  --dictionary DICT_4X4_50 \
  --squares-x 11 \
  --squares-y 8 \
  --square-length-mm 15 \
  --marker-length-mm 11 \
  --save-overlay
```

## 4. 求解标定矩阵

### 4.1 先看有哪些 session

```bash
cd /home/misca/TabletopGraspSystem
ls -lt calibrate_by_handeye/samples
```

### 4.2 默认使用最新 session 求解

如果不显式指定 `--samples-dir`，求解脚本现在会自动选择：

- `calibrate_by_handeye/samples/` 下面**时间最新的 `session_*` 目录**

直接运行：

```bash
cd /home/misca/TabletopGraspSystem
python calibrate_by_handeye/solve_eye_to_hand.py \
  --output calibrate_by_handeye/extrinsics_eye_to_hand.json
```

### 4.3 如果你想指定某个 session

```bash
cd /home/misca/TabletopGraspSystem
python calibrate_by_handeye/solve_eye_to_hand.py \
  --samples-dir calibrate_by_handeye/samples/session_20260327_150851 \
  --output calibrate_by_handeye/extrinsics_eye_to_hand.json
```

### 4.4 默认覆盖输出

默认输出文件：

- `calibrate_by_handeye/extrinsics_eye_to_hand.json`

里面包含：

- `T_base_camera`
- `translation_xyz_m`
- `quaternion_xyzw`
- `method`
- `sample_count`

## 5. 验证标定矩阵

求解后建议立刻做一致性验证：

```bash
cd /home/misca/TabletopGraspSystem
python calibrate_by_handeye/validate_eye_to_hand.py \
  --samples-dir calibrate_by_handeye/samples/session_20260327_150851 \
  --extrinsics calibrate_by_handeye/extrinsics_eye_to_hand.json \
  --output calibrate_by_handeye/validation_eye_to_hand.json
```

### 5.1 验证在检查什么

验证脚本会反推出每条样本对应的：

- `T_ee_board`

如果标定正确、板相对末端确实固定，那么所有样本反推出的 `T_ee_board` 应该接近常量。

### 5.2 当前可接受的经验判断

通常可以先用下面这个量级做直觉判断：

- 平移误差均值在厘米级以内，通常就比较像是对的
- 旋转误差均值在几度以内，通常说明一致性不错

如果误差很大，优先怀疑：

- 板子相对末端有滑动
- 样本里有非 `25/25` 质量边缘帧
- 采集时机械臂没停稳
- 用错了 session 或 extrinsics 文件

## 6. 当前坐标变换约定

当前这条链采用的约定是：

- `T_base_camera`
  - 把相机坐标系下的点变到机械臂基座坐标系
- `T_camera_board`
  - 把标定板坐标系下的点变到相机坐标系
- `T_base_ee`
  - 把末端坐标系下的点变到基座坐标系

因此验证时使用：

```text
T_ee_board = inv(T_base_ee) @ T_base_camera @ T_camera_board
```

如果这组结果在不同样本中接近不变，就说明手眼矩阵是自洽的。

## 7. 当前推荐工作流

1. 启动 `runtime_bridge`
2. 手动采集一批 `25/25` 完整样本
3. 用该批样本求 `extrinsics_eye_to_hand.json`
4. 用 `validate_eye_to_hand.py` 验证一致性
5. 通过后再把这份矩阵用于抓取或三维目标变换

当前用于 hover 的默认参考外参是：

- `calibrate_by_handeye/extrinsics_eye_to_hand_prev.json`

它对应：

- `session_20260327_150851`

这版验证结果明显好于更新后的最新 session，更适合作为当前 hover 默认值。

## 8. 常见命令汇总

### 连续采样

```bash
python calibrate_by_handeye/start_handeye_session.py \
  --serial 419522072950 \
  --start-full-stack \
  --color-width 1280 \
  --color-height 720 \
  --color-fps 15 \
  --depth-width 640 \
  --depth-height 480 \
  --depth-fps 15 \
  --dictionary DICT_4X4_50 \
  --squares-x 11 \
  --squares-y 8 \
  --square-length-mm 15 \
  --marker-length-mm 11 \
  --save-overlay
```

### 求解最新 session

```bash
python calibrate_by_handeye/solve_eye_to_hand.py \
  --samples-dir calibrate_by_handeye/samples/session_你的目录 \
  --output calibrate_by_handeye/extrinsics_eye_to_hand.json
```

### 验证

```bash
python calibrate_by_handeye/validate_eye_to_hand.py \
  --samples-dir calibrate_by_handeye/samples/session_你的目录 \
  --extrinsics calibrate_by_handeye/extrinsics_eye_to_hand.json \
  --output calibrate_by_handeye/validation_eye_to_hand.json
```

### 当前 hover 默认外参

```bash
/home/misca/TabletopGraspSystem/calibrate_by_handeye/extrinsics_eye_to_hand_prev.json
```
