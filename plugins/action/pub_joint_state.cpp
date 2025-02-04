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

#include "pb2025_sentry_behavior/plugins/action/pub_joint_state.hpp"

namespace pb2025_sentry_behavior
{

PublishJointStateAction::PublishJointStateAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishJointStateAction::providedPorts()
{
  return providedBasicPorts(
    {BT::InputPort<float>(
       "gimbal_pitch", "{gimbal_pitch}", "Expected Pitch angle (rad) of the gimbal"),
     BT::InputPort<float>("gimbal_yaw", "{gimbal_yaw}", "Expected Yaw angle (rad) of the gimbal")});
}

bool PublishJointStateAction::setMessage(sensor_msgs::msg::JointState & msg)
{
  double gimbal_pitch = 0.0, gimbal_yaw = 0.0;
  getInput("gimbal_pitch", gimbal_pitch);
  getInput("gimbal_yaw", gimbal_yaw);

  msg.header.stamp = node_->now();
  msg.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  msg.position = {gimbal_pitch, gimbal_yaw};

  return true;
}

bool PublishJointStateAction::setHaltMessage(sensor_msgs::msg::JointState & /*msg*/)
{
  // Not sending any message
  return false;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishJointStateAction, "PubJointState");
