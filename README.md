# Bot Description ROS2 仿真示例 Demo

## 项目概述

本仓库提供一个基于 ROS 2 的差分驱动机器人仿真示例，使用 Gazebo Classic 进行物理模拟，并在 RViz 中可视化机器人模型。包括机器人描述（URDF/Xacro）、启动脚本、控制器配置以及自定义仿真场景。

## 目录结构

```text
.
├── CMakeLists.txt             # Ament CMake 构建配置
├── package.xml                # 包元数据及依赖声明
├── urdf/                      # URDF/Xacro 机器人模型
│   ├── bot/bot.urdf.xacro     # 顶层机器人描述
│   ├── actuator/              # 车轮和万向轮宏定义
│   ├── sensor/                # 摄像头、IMU、激光传感器宏定义
│   └── plugins/               # Gazebo 插件集成配置
├── config/                    # 配置文件
│   ├── bot_ros2_controller.yaml  # ros2_control 控制器参数
│   └── display_robot_model.rviz  # RViz 预设场景
├── launch/                    # 启动文件
│   ├── display_robot.launch.py   # 发布 TF 并启动 RViz
│   └── gazebo_sim.launch.py      # 启动 Gazebo 仿真并加载控制器
├── world/                     # 自定义 Gazebo 场景
│   ├── custom_room.world      # 室内场景描述
│   └── room/                  # 自定义房间模型 (SDF + config)
└── README.md                  # 本 README 文件
```

## 依赖项

- ROS 2（如 Humble 或更新版本）
- Gazebo Classic (v11)
- `ament_cmake`
- `gazebo_ros2_control`
- `ros2_control` 及控制器包：
  - `joint_state_broadcaster`
  - `effort_controllers`（如 JointGroupEffortController）
- TF2 与 `robot_state_publisher`

## 启动示例

### 可视化机器人模型

```bash
ros2 launch bot_description display_robot.launch.py
```

- 启动 `robot_state_publisher` 节点发布 TF
- 打开 RViz，显示机器人模型

### 全功能仿真

```bash
ros2 launch bot_description gazebo_sim.launch.py
```

- 启动 Gazebo 服务端（gzserver）和客户端（gzclient）
- 加载 `custom_room.world` 场景
- 发布机器人 URDF 至 `/robot_description`
- 自动加载并激活以下控制器：
  - `bot_joint_broadcaster` (JointStateBroadcaster)
  - `bot_effort_controller` (JointGroupEffortController)

## 机器人描述 (URDF/Xacro)

- 通过 Xacro 模块化描述机器人：
  - `actuator/`：车轮和万向轮关节与几何
  - `sensor/`：摄像头、IMU、激光分类传感器
  - `plugins/`：Gazebo 插件（`gazebo_ros2_control`、传感器插件）

## 控制器配置

在 `config/bot_ros2_controller.yaml` 中指定：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100    # Hz
    use_sim_time: true

    bot_joint_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    bot_effort_controller:
      type: effort_controllers/JointGroupEffortController

bot_effort_controller:
  ros__parameters:
    joints:
      - left_wheel_joint
      - right_wheel_joint
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
      - effort
```

## 仿真场景

- `world/custom_room.world`：简单室内环境
- 自定义房间模型位于 `world/room/`，包含 SDF 与模型配置

## 使用技术

- **ROS 2 Launch**：基于 Python 的启动脚本
- **Gazebo Classic**：物理仿真环境
- **ros2_control**：硬件抽象与控制器管理
- **Xacro**：可复用 URDF 宏
- **RViz**：机器人状态及 TF 可视化
- **Ament CMake**：ROS 2 构建系统

## 构建与运行

```bash
# 编译
colcon build --packages-select bot_description
source install/setup.bash

# 可视化
ros2 launch bot_description display_robot.launch.py

# 完整仿真
ros2 launch bot_description gazebo_sim.launch.py
```

## 许可证

本项目采用 Apache-2.0 许可证，详情见 LICENSE 文件。
