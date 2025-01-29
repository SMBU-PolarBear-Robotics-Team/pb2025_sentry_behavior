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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace pb2025_sentry_behavior
{
class SendNav2GoalAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendNav2GoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
