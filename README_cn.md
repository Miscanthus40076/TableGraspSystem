# TabletopGraspSystem

English documentation: [README.md](./README.md)

这是一个面向桌面抓取任务的总系统仓库，包含：

- 来自 `TabletopSeg3D` 的视觉感知模块
- 来自独立仓库的机械臂控制模块
- 相机到机械臂的标定与坐标变换
- 抓取位姿规划与系统编排

## 目录结构

```text
TabletopGraspSystem/
├── README.md
├── README_cn.md
├── .gitignore
├── app/
├── calibration/
├── planning/
├── robot/
└── perception/
    └── tabletopseg3d/
```

## 设计思路

这个仓库用于把视觉、标定、规划和机械臂控制整合到一起。

- `perception/tabletopseg3d/`
  - 保留当前已经完成的视觉模块
  - 输出类别、中心点、尺寸和桌面对齐 `yaw`
- `robot/`
  - 对接独立维护的机械臂仓库
- `calibration/`
  - 完成相机坐标到机械臂坐标的变换
- `planning/`
  - 生成抓取位姿
- `app/`
  - 串联整个抓取流程

## 官方 StarAI 机械臂 ROS2 功能包安装

当前使用的官方机械臂 ROS2 仓库是：

- `fashionstar-starai-arm-ros2`
- 地址：`https://github.com/Seeed-Projects/fashionstar-starai-arm-ros2.git`

推荐安装步骤如下。

1. 创建工作空间并克隆仓库

```bash
mkdir -p ~/starai_ws/src
cd ~/starai_ws/src
git clone https://github.com/Seeed-Projects/fashionstar-starai-arm-ros2.git
```

2. 先 source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

3. 编译工作空间

```bash
cd ~/starai_ws
colcon build
```

4. 加入 shell 启动项

```bash
echo "source ~/starai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

5. 安装驱动节点依赖到系统 Python

注意：`robo_driver` 是 ROS 2 节点，会使用系统 Python，而不是 conda 环境里的 Python。
因此 `pyserial` 和 `fashionstar-uart-sdk` 需要安装到系统 Python：

```bash
/usr/bin/python3 -m pip install --user pyserial fashionstar-uart-sdk
```

## Conda 用户注意事项

如果你正在使用 conda，`python3` 很可能会被 conda 环境覆盖，导致 ROS 2 构建失败。

在这台机器上，能够稳定编译通过的方式是：

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
cd ~/starai_ws
colcon build
```

原因是：

- ROS 2 Humble 这里依赖系统 Python `3.10`
- 如果误用了 conda 的 Python，构建过程里可能会找不到 `ament_cmake` 或 `em`
- 即使视觉侧已经在 conda 里装好了 `pyserial` 和 `fashionstar-uart-sdk`，ROS 驱动节点仍然可能因为系统 Python 缺包而启动失败

## 当前安装结果

我已经在这台机器上完成了安装验证：

- ROS 版本：`Humble`
- 工作空间：`~/starai_ws`
- 编译结果：`14 packages finished`

这说明官方 StarAI 机械臂 ROS2 功能包已经在当前机器上安装成功。

## 当前桥接层

第一版机械臂桥接层已经写在：

- [robot/adapter.py](./robot/adapter.py)

当前行为是：

- 向 `/position_orientation_topic` 发布末端位姿
- 向 `/gripper_command_topic` 发布夹爪开闭命令
- 通过 `ros2 topic pub --once` 调用 ROS2
- 避开在 conda 主程序里直接导入 `rclpy` 造成的环境冲突
