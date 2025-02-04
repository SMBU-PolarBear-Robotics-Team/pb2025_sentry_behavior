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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsAttackedCondition : public BT::SimpleConditionNode
{
public:
  IsAttackedCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for robot_status ports
   */
  BT::NodeStatus checkIsAttacked();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsAttackedCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
