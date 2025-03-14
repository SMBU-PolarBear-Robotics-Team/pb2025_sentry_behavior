# pb2025_sentry_behavior

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

> 开发中，不考虑向前兼容性，仅供参考，请谨慎使用。文档可能不会及时更新以反映代码的最新变化。

基于 [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 和 [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) 的行为树框架与插件，用于 [RoboMaster](https://www.robomaster.com) 2025 赛季哨兵机器人。

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS2 Humble
- BehaviorTree.CPP (Developed with release [4.6.2](https://github.com/BehaviorTree/BehaviorTree.CPP/releases/tag/4.6.2))

### 2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
pip install vcstool2
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior.git src/pb2025_sentry_behavior
```

```bash
vcs import --recursive src < src/pb2025_sentry_behavior/dependencies.repos
```

### 2.3 Build

```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.4 Running

```bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
```

## 3. Behaviors

### 3.1 Action

#### CalculateAttackPose

![attack_behavior](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/attack_behavior.99th7cowh5.gif)

`CalculateAttackPose` 动作节点用于计算并输出进攻坐标，该节点基于敌方坐标和全局代价地图生成候选点，过滤出可行点，并选择最佳攻击点。

Input Ports:

- `costmap_port`：类型为 `nav_msgs::msg::OccupancyGrid`，表示全局代价地图。
- `tracker_port`：类型为 `auto_aim_interfaces::msg::Target`，表示视觉估计得到的目标状态信息。Related repo: [armor_tracker](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision/tree/main/armor_tracker)

Output Ports:

- `goal`：类型为 `geometry_msgs::msg::PoseStamped`，表示发送给导航系统的目标姿态。

Parameters:

- `attack_radius`：以目标位置为圆心的半径，default: 3.0。
- `num_sectors`：划分扇区数量，default: 36。
- `cost_threshold`：代价阈值，default: 50。
- `robot_base_frame`：机器人基坐标系，default: "chassis"。
- `transform_tolerance`：查询 tf 超时时间，default: 0.5。
- `visualize`：是否启用可视化，default: false。

##### 功能描述

1. **transformToCostmapFrame**：将敌方位置转换到代价地图框架。
2. **generateCandidatePoints**：生成候选攻击点。
3. **filterFeasiblePoints**：过滤出可行的攻击点。
4. **selectBestPoint**：选择最佳攻击点。
5. **createAttackPose**：创建攻击姿态。
6. **createVisualizationMarkers**：创建可视化标记。

#### PubTwist

以 `geometry_msgs/msg/Twist` 的形式发布速度，用于控制底盘运动。

#### SendNav2Goal

创建 Client，以 `nav2_msgs/action/NavigateToPose` 的形式发送 Navigation2 目标点。

> [!CAUTION]
> BehaviorTree.ROS2 中存在 bug，导致继承自 [RosActionNode](https://github.com/BehaviorTree/BehaviorTree.ROS2/blob/cc31ea7b97947f1aac6e8c37df6cec379c84a7d9/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp#L80) 的节点无法被正确 halt，最终导致行为树被 shutdown。因此，下面提供了一个以 topic 代替 action 发布 goal_pose 的临时方案 `PubNav2Goal`，坏处是无法实时获取到 action 的 feedback 和 result。
>
> Related Issue:  [#18 Error when canceling action during halt()](https://github.com/BehaviorTree/BehaviorTree.ROS2/issues/18)

#### PubNav2Goal

以 `geometry_msgs/msg/pose_stamped` 的形式发布 Navigation2 目标点。

### 3.2 Condition

#### IsAttacked

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RobotStatus` 类型数据，判断机器人是否受到攻击，并根据裁判系统装甲模块反馈的信息输出敌方可能的角度位置。该条件节点会根据输入端口的配置，检查以下几个条件：

- `key_port`：从 GlobalBlackboard 获取 `RobotStatus` 消息
- `gimbal_pitch`：输出固定的云台俯仰角度（0.0）
- `gimbal_yaw`：输出敌方可能的角度位置

如果检测到装甲板被击中，则返回 `SUCCESS`，并输出相应的云台角度；否则返回 `FAILURE`。

#### IsDetectEnemy

通过视觉模块判断是否感知到敌方。

- `armor_id`：预期的装甲 ID 列表。可以是多个数字，用 `;` 分隔
- `max_distance`：敌方目标的最大距离

如果视觉模块感知到的敌方在 `armor_id` 列表中，且距离小于 `max_distance`，则返回 SUCCESS，否则返回 FAILURE。

#### IsGameStatus

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::GameStatus` 类型数据，判断当前比赛状态是否在输入的时间范围内且处于预期的比赛阶段。该条件节点会根据输入端口的配置，检查以下几个条件：

- `key_port`：从 GlobalBlackboard 获取 `GameStatus` 消息
- `expected_game_progress`：预期的比赛阶段
- `min_remain_time`：最小剩余时间（秒）
- `max_remain_time`：最大剩余时间（秒）

如果比赛阶段和剩余时间都符合预期，则返回 `SUCCESS`，否则返回 `FAILURE`。

#### IsRfidDetected

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RfidStatus` 类型数据，判断机器人是否检测到指定的 RFID 标签。该条件节点会根据输入端口的配置，检查以下几个位置的 RFID 状态：

- `key_port`：从 GlobalBlackboard 获取 `RfidStatus` 消息
- `friendly_fortress_gain_point`：己方堡垒增益点
- `friendly_supply_zone_non_exchange`：己方与兑换区不重叠的补给区 / RMUL 补给区
- `friendly_supply_zone_exchange`：己方与兑换区重叠的补给区
- `center_gain_point`：中心增益点（仅 RMUL 适用）

如果任意一个配置为 `true` 的位置检测到 RFID 标签，则返回 `SUCCESS`，否则返回 `FAILURE`。

#### IsStatusOK

通过 GlobalBlackboard 获取实时的 `pb_rm_interfaces::msg::RobotStatus` 类型数据，判断机器人的状态是否正常。该条件节点会根据输入端口的配置，检查以下几个条件：

- `key_port`：从 GlobalBlackboard 获取 `RobotStatus` 消息
- `hp_min`：最低血量
- `heat_max`：最大发射机构的射击热量
- `ammo_min`：最小弹丸允许发弹量

如果机器人的 HP、热量和弹药量都在预期范围内，则返回 `SUCCESS`，否则返回 `FAILURE`。
