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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GIMBAL_VELOCITY_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GIMBAL_VELOCITY_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"

namespace pb2025_sentry_behavior
{

class PublishGimbalVelocity
: public BT::RosTopicPubStatefulActionNode<pb_rm_interfaces::msg::GimbalCmd>
{
public:
  PublishGimbalVelocity(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

protected:
  bool setMessage(pb_rm_interfaces::msg::GimbalCmd & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GIMBAL_VELOCITY_HPP_
