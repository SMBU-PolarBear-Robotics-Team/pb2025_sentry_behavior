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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior.hpp"

#include <fstream>

#include "behaviortree_cpp/xml_parsing.h"

namespace pb2025_sentry_behavior
{

SentryDecisonServer::SentryDecisonServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  // Subscribe ros topic and update the global blackboard
  event_data_sub_ = node()->create_subscription<pb_rm_interfaces::msg::EventData>(
    "referee/event_data", 10, [this](const pb_rm_interfaces::msg::EventData::SharedPtr msg) {
      globalBlackboard()->set("event_data", *msg);
    });
  all_robot_hp_sub_ = node()->create_subscription<pb_rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10, [this](const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg) {
      globalBlackboard()->set("all_robot_hp", *msg);
    });
  game_status_sub_ = node()->create_subscription<pb_rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10, [this](const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
      globalBlackboard()->set("game_status", *msg);
    });
  ground_robot_position_sub_ =
    node()->create_subscription<pb_rm_interfaces::msg::GroundRobotPosition>(
      "referee/ground_robot_position", 10,
      [this](const pb_rm_interfaces::msg::GroundRobotPosition::SharedPtr msg) {
        globalBlackboard()->set("ground_robot_position", *msg);
      });
  rfid_status_sub_ = node()->create_subscription<pb_rm_interfaces::msg::RfidStatus>(
    "referee/rfid_status", 10, [this](const pb_rm_interfaces::msg::RfidStatus::SharedPtr msg) {
      // Update the global blackboard
      globalBlackboard()->set("rfid_status", *msg);
    });
  robot_status_sub_ = node()->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10, [this](const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
      globalBlackboard()->set("robot_status", *msg);
    });
  buff_sub_ = node()->create_subscription<pb_rm_interfaces::msg::Buff>(
    "referee/buff", 10, [this](const pb_rm_interfaces::msg::Buff::SharedPtr msg) {
      globalBlackboard()->set("buff", *msg);
    });
  // Note that the callback above and the execution of the tree accessing the
  // global blackboard happen in two different threads.
  // The former runs in the MultiThreadedExecutor, while the latter in the
  // thread created by TreeExecutionServer. But this is OK because the
  // blackboard is thread-safe.
}

void SentryDecisonServer::onTreeCreated(BT::Tree & tree)
{
  logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
}

std::optional<std::string> SentryDecisonServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  // NOT really needed, even if logger_cout_ may contain a dangling pointer of
  // the tree at this point
  logger_cout_.reset();
  return std::nullopt;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryDecisonServer>(options);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

  // Save the XML models to a file
  std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
  file << xml_models;

  rclcpp::shutdown();
}
