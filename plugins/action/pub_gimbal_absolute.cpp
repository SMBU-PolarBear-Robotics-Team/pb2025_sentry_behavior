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

#include "pb2025_sentry_behavior/plugins/action/pub_gimbal_absolute.hpp"

namespace pb2025_sentry_behavior
{

PublishGimbalAbsolute::PublishGimbalAbsolute(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishGimbalAbsolute::providedPorts()
{
  return {
    BT::InputPort<float>("gimbal_pitch", "Expected Pitch angle (rad)"),
    BT::InputPort<float>("gimbal_yaw", "Expected Yaw angle (rad)"),
    BT::InputPort<std::chrono::milliseconds>("duration", "Publish duration"),
    BT::InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name")};
}

bool PublishGimbalAbsolute::setMessage(pb_rm_interfaces::msg::GimbalCmd & msg)
{
  msg.header.stamp = node_->now();
  msg.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  msg.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;

  float pitch, yaw;
  if (!getInput("gimbal_pitch", pitch) || !getInput("gimbal_yaw", yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing angle parameters");
    return false;
  }

  msg.position.pitch = pitch;
  msg.position.yaw = yaw;
  msg.velocity = pb_rm_interfaces::msg::Gimbal();

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishGimbalAbsolute, "PublishGimbalAbsolute");
