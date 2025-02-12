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

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");

  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose = goal->pose;
  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
