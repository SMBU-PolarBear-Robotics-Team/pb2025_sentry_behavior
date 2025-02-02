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

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{
class IsAttackedCondition : public BT::RosTopicPubNode<sensor_msgs::msg::JointState>
{
public:
  IsAttackedCondition(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(sensor_msgs::msg::JointState & goal) override;

private:
  rclcpp::Time now() { return node_->now(); }

  tf2::Transform getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
