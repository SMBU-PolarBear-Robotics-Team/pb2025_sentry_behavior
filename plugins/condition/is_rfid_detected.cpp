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

#include "pb2025_sentry_behavior/plugins/condition/is_rfid_detected.hpp"

namespace pb2025_sentry_behavior
{

IsRfidDetectedCondition::IsRfidDetectedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRfidDetectedCondition::checkRfidStatus, this), config)
{
}

BT::NodeStatus IsRfidDetectedCondition::checkRfidStatus()
{
  bool friendly_fortress_gain_point, friendly_supply_zone_non_exchange,
    friendly_supply_zone_exchange, center_gain_point;
  auto msg = getInput<pb_rm_interfaces::msg::RfidStatus>("key_port");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
    RCLCPP_ERROR(logger_, "RfidStatus message is not available");
  }

  getInput("friendly_fortress_gain_point", friendly_fortress_gain_point);
  getInput("friendly_supply_zone_non_exchange", friendly_supply_zone_non_exchange);
  getInput("friendly_supply_zone_exchange", friendly_supply_zone_exchange);
  getInput("center_gain_point", center_gain_point);

  if (
    (friendly_fortress_gain_point && msg->friendly_fortress_gain_point == msg->DETECTED) ||
    (friendly_supply_zone_non_exchange &&
     msg->friendly_supply_zone_non_exchange == msg->DETECTED) ||
    (friendly_supply_zone_exchange && msg->friendly_supply_zone_exchange == msg->DETECTED) ||
    (center_gain_point && msg->center_gain_point == msg->DETECTED)) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList IsRfidDetectedCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RfidStatus>(
      "key_port", "{@referee_rfidStatus}", "RfidStatus port on blackboard"),
    BT::InputPort<bool>("friendly_fortress_gain_point", false, "己方堡垒增益点"),
    BT::InputPort<bool>(
      "friendly_supply_zone_non_exchange", false, "己方与兑换区不重叠的补给区 / RMUL 补给区"),
    BT::InputPort<bool>("friendly_supply_zone_exchange", false, "己方与兑换区重叠的补给区"),
    BT::InputPort<bool>("center_gain_point", false, "中心增益点（仅 RMUL 适用）"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsRfidDetectedCondition>("IsRfidDetected");
}
