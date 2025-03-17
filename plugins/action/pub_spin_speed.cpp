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

#include "pb2025_sentry_behavior/plugins/action/pub_spin_speed.hpp"

namespace pb2025_sentry_behavior
{

PublishSpinSpeedAction::PublishSpinSpeedAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishSpinSpeedAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("spin_speed", 0.0, "Angular Z velocity (rad/s)"),
  });
}

bool PublishSpinSpeedAction::setMessage(example_interfaces::msg::Float32 & msg)
{
  double spin_speed = 0.0;
  getInput("spin_speed", spin_speed);

  msg.data = spin_speed;

  return true;
}

bool PublishSpinSpeedAction::setHaltMessage(example_interfaces::msg::Float32 & msg)
{
  msg.data = 0;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishSpinSpeedAction, "PublishSpinSpeed");
