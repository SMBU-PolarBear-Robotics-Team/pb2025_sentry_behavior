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

#include "pb2025_sentry_behavior/plugins/condition/is_detect_enemy.hpp"

namespace pb2025_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemy, this), config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    BT::InputPort<auto_aim_interfaces::msg::Armors>(
      "key_port", "{@detector_armors}", "Vision detector port on blackboard"),
    BT::InputPort<std::vector<int>>(
      "armor_id", "1;2;3;4;5;7",
      "Expected id of armors. Multiple numbers should be separated by the character `;` in Groot2"),
    BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),
  };
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemy()
{
  std::vector<int> expected_armor_ids;
  float max_distance;
  auto msg = getInput<auto_aim_interfaces::msg::Armors>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "Detector message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("armor_id", expected_armor_ids);
  getInput("max_distance", max_distance);

  for (const auto & armor : msg->armors) {
    float distance_to_enemy = std::hypot(armor.pose.position.x, armor.pose.position.y);

    if (armor.number.empty()) {
      continue;
    }
    int armor_id = std::stoi(armor.number);
    const bool is_armor_id_match =
      std::find(expected_armor_ids.begin(), expected_armor_ids.end(), armor_id) !=
      expected_armor_ids.end();

    const bool is_within_distance = (distance_to_enemy <= max_distance);

    if (is_armor_id_match && is_within_distance) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}
