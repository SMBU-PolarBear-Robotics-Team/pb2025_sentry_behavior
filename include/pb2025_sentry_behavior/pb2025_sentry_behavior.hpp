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

#ifndef PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_HPP_
#define PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SentryDecisonServer : public BT::TreeExecutionServer
{
public:
  explicit SentryDecisonServer(const rclcpp::NodeOptions & options);

  void onTreeCreated(BT::Tree & tree) override;

  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override;

private:
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
  rclcpp::Subscription<pb_rm_interfaces::msg::EventData>::SharedPtr event_data_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::Buff>::SharedPtr buff_sub_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_HPP_
