# pb2025_sentry_behavior

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

> 开发中，不考虑向前兼容性，仅供参考，请谨慎使用。

基于 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 和 [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) 的哨兵行为树。

## 1. Usage

### 1.1 Install

- Ubuntu 22.04
- ROS2 Humble
- BehaviorTree.CPP (Developed with release [4.6.2](https://github.com/BehaviorTree/BehaviorTree.CPP/releases/tag/4.6.2))
- BehaviorTree.ROS2 (Developed with commit [cc31ea7](https://github.com/BehaviorTree/BehaviorTree.ROS2/commit/cc31ea7b97947f1aac6e8c37df6cec379c84a7d9))

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior.git
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb_rm_interfaces.git -b protocol-v1.7.0

cd ~/ros2_ws
```

```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 1.2 Run

```bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
```

To call the Action Server from the command line:

```bash
ros2 action send_goal /pb2025_sentry_behavior btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: dev_rm}"
```

## 2. Behaviors

### 2.1 Action

#### SendNav2Goal

创建 Client，以 `nav2_msgs/action/NavigateToPose` 的形式发送 Navigation2 目标点。

input_port：`goal_x` (float), `goal_y` (float), `goal_yaw` (float), `action_name` (string)

### 2.2 Condition

#### IsGameStatus

通过 GlobalBlackboard 获取实时 `pb_rm_interfaces::msg::GameStatus` 类型数据，判断当前比赛状态是否在输入的时间范围内且处于预期的比赛阶段。

input_port：`max_remain_time` (int), `min_remain_time` (int), `expected_game_progress` (int), `key_port` (pb_rm_interfaces::msg::GameStatus)
