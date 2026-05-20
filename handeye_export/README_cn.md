# 手眼标定迁移包

这个目录是从当前项目中提取出来的眼在手外手眼标定代码，适合搬到另一个项目继续用。

## 标定场景

- 相机固定在环境中。
- ChArUco 标定板固定在机械臂末端。
- 每个样本保存 `T_base_ee` 和 `T_camera_board`。
- 求解结果是 `T_base_camera`，语义是把相机坐标系中的点变换到机械臂基座坐标系。

坐标约定：

```text
T_x_y 表示 p_x = T_x_y * p_y

T_ee_board = inv(T_base_ee) @ T_base_camera @ T_camera_board
```

## 文件说明

- `calibrate_by_handeye/start_handeye_session.py`
  - 连续手动采样入口。
  - 依赖 RealSense、ChArUco 检测、机械臂 joint_state 获取、URDF 正运动学。
- `calibrate_by_handeye/collect_handeye_sample.py`
  - 单条样本采集入口。
- `calibrate_by_handeye/solve_eye_to_hand.py`
  - 离线读取 `sample.json`，调用 OpenCV `calibrateHandEye` 求 `T_base_camera`。
  - 这是最容易独立复用的核心脚本。
- `calibrate_by_handeye/validate_eye_to_hand.py`
  - 验证外参是否自洽，检查反推的 `T_ee_board` 是否在样本间近似恒定。
- `calibration/detect_charuco.py`
  - ChArUco 检测和 `board->camera` 位姿估计。
- `calibration/robot_fk.py`
  - 从 URDF 和 joint positions 计算 `T_base_ee`。
- `perception/tabletopseg3d/src/camera/realsense_capture.py`
  - RealSense 枚举、取流、内参转换。
- `robot/runtime_bridge_client.py`
  - 当前项目读取机械臂状态的 socket 客户端。迁移时通常需要替换或适配。

## 依赖

见 `requirements.txt`。其中 `pyrealsense2` 只在采样时需要；如果你已经能生成样本，只跑求解和验证时主要需要 `numpy`、`opencv-contrib-python`。

## 新项目需要适配的地方

采样脚本默认通过 `RuntimeBridgeClient.get_joint_state()` 获取机械臂状态，返回结构大致是：

```json
{
  "ok": true,
  "joint_state": {
    "joint_names": ["joint1", "joint2"],
    "joint_positions": [0.0, 0.0]
  }
}
```

迁移到另一个项目时，优先改这里：

- 如果新项目已有 ROS/SDK joint state，替换 `robot/runtime_bridge_client.py` 或在 `start_handeye_session.py` 中改 `bridge.get_joint_state()` 的来源。
- 修改 `--urdf-path` 默认值，指向新机械臂 URDF。
- 修改 `--ee-link` 默认值，指向新机械臂末端 link。
- 按实际标定板修改 `--dictionary`、`--squares-x`、`--squares-y`、`--square-length-mm`、`--marker-length-mm`。
- 当前采样质量门槛写死为完整 6x6 子板的 `25/25` 个 ChArUco 内角点；如果你的板不同，需要改 `REQUIRED_CHARUCO_6X6_CORNERS` 和相关判断。

## 最小复用方式

如果新项目已经能自己采集样本，只需要保证 `sample.json` 里至少有：

```json
{
  "sample_name": "sample_000",
  "board_detection": {
    "pose_ok": true,
    "rvec": [0.0, 0.0, 0.0],
    "tvec_m": [0.0, 0.0, 0.5]
  },
  "ee_reference": {
    "T_base_ee": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
  }
}
```

然后运行：

```bash
python calibrate_by_handeye/solve_eye_to_hand.py \
  --samples-dir calibrate_by_handeye/samples/session_你的目录 \
  --output calibrate_by_handeye/extrinsics_eye_to_hand.json

python calibrate_by_handeye/validate_eye_to_hand.py \
  --samples-dir calibrate_by_handeye/samples/session_你的目录 \
  --extrinsics calibrate_by_handeye/extrinsics_eye_to_hand.json \
  --output calibrate_by_handeye/validation_eye_to_hand.json
```

## 推荐迁移顺序

1. 先只迁移 `solve_eye_to_hand.py` 和 `validate_eye_to_hand.py`，用新项目生成的样本跑通离线求解。
2. 再接入 `detect_charuco.py`，确认新相机图像能稳定输出 `rvec/tvec_m`。
3. 最后改采样脚本里的机械臂状态来源和 URDF 正运动学。

