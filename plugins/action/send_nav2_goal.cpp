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

#include "pb2025_sentry_behavior/plugins/action/send_nav2_goal.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto goal_x = getInput<float>("goal_x");
  auto goal_y = getInput<float>("goal_y");
  auto goal_yaw = getInput<float>("goal_yaw");
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, goal_yaw.value());

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = goal_x.value();
  goal.pose.pose.position.y = goal_y.value();
  goal.pose.pose.orientation = tf2::toMsg(quaternion);

  RCLCPP_DEBUG(
    logger(), "Setting goal to (%.2f, %.2f, %.2f)", goal_x.value(), goal_y.value(),
    goal_yaw.value());

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown navigation result code: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Distance remaining: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt() { RCLCPP_INFO(logger(), "SendNav2GoalAction has been halted."); }

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendNav2GoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
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
CreateRosNodePlugin(pb2025_sentry_behavior::SendNav2GoalAction, "SendNav2Goal");
