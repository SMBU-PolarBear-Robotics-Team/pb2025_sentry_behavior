// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb2025_sentry_behavior/plugins/condition/is_attacked.hpp"

#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<sensor_msgs::msg::JointState>(name, conf, params)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool IsAttackedCondition::setMessage(sensor_msgs::msg::JointState & goal)
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(node_->get_logger(), "RobotStatus message is not available");
    return false;
  }

  float hit_armor_pose = 0.0;
  double cur_roll, cur_pitch, cur_yaw;
  if (msg->is_hp_deduced && msg->hp_deduction_reason == msg->ARMOR_HIT) {
    RCLCPP_DEBUG(node_->get_logger(), "Armor hit detected");
    switch (msg->armor_id) {
      // Anticlockwise from armor 0
      case 0:
        hit_armor_pose = 0.0;
        break;
      case 1:
        hit_armor_pose = M_PI_2;
        break;
      case 2:
        hit_armor_pose = M_PI;
        break;
      case 3:
        hit_armor_pose = -M_PI_2;
        break;
      default:
        RCLCPP_WARN(node_->get_logger(), "Invalid armor id: %d", msg->armor_id);
        break;
    }
    tf2::Transform tf_chassis_to_gimbal = getTransform("chassis", "gimbal_yaw", node_->now());
    tf2::Matrix3x3(tf_chassis_to_gimbal.getRotation()).getRPY(cur_roll, cur_pitch, cur_yaw);
    RCLCPP_INFO(node_->get_logger(), "cur_yaw: %f", cur_yaw);

    int yaw_rounds = static_cast<int>(cur_yaw / (2 * M_PI));

    goal.header.stamp = node_->now();
    goal.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
    goal.position = {0.0, hit_armor_pose + yaw_rounds * (2 * M_PI)};
  }

  return true;
}

BT::PortsList IsAttackedCondition::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@robot_status}", "RobotStatus port on blackboard"),
  };
  return providedBasicPorts(additional_ports);
}

tf2::Transform IsAttackedCondition::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::IsAttackedCondition, "IsAttacked");
