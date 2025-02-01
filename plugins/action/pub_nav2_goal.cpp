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

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & goal)
{
  auto goal_x = getInput<float>("goal_x");
  auto goal_y = getInput<float>("goal_y");
  auto goal_yaw = getInput<float>("goal_yaw");
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, goal_yaw.value());

  goal.header.stamp = now();
  goal.header.frame_id = "map";
  goal.pose.position.x = goal_x.value();
  goal.pose.position.y = goal_y.value();
  goal.pose.orientation = tf2::toMsg(quaternion);

  RCLCPP_DEBUG(
    logger(), "Setting goal to (%.2f, %.2f, %.2f)", goal_x.value(), goal_y.value(),
    goal_yaw.value());

  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<float>("goal_x", 0.0, "Goal x coordinate"),
    BT::InputPort<float>("goal_y", 0.0, "Goal y coordinate"),
    BT::InputPort<float>("goal_yaw", 0.0, "Goal orientation (yaw)"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
