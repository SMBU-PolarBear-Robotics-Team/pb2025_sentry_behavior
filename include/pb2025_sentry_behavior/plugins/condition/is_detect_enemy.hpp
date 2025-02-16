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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_

#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that checks for the presence of an enemy target
 * returns SUCCESS if an enemy is detected
 */
class IsDetectEnemyCondition : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkEnemy();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsDetectEnemyCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
