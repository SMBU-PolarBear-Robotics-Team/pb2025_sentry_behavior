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

#include "pb2025_sentry_behavior/plugins/action/pub_twist.hpp"

namespace pb2025_sentry_behavior
{

PublishTwistAction::PublishTwistAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishTwistAction::providedPorts()
{
  return providedBasicPorts(
    {BT::InputPort<double>("v_x", 0.0, "Linear X velocity (m/s)"),
     BT::InputPort<double>("v_y", 0.0, "Linear Y velocity (m/s)"),
     BT::InputPort<double>("v_yaw", 0.0, "Angular Z velocity (rad/s)")});
}

bool PublishTwistAction::setMessage(geometry_msgs::msg::Twist & msg)
{
  double vx = 0.0, vy = 0.0, vyaw = 0.0;
  getInput("v_x", vx);
  getInput("v_y", vy);
  getInput("v_yaw", vyaw);

  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = vyaw;

  return true;
}

bool PublishTwistAction::setHaltMessage(geometry_msgs::msg::Twist & msg)
{
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.angular.z = 0;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishTwistAction, "PublishTwist");
