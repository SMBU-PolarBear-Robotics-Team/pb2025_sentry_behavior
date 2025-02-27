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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using PointStamped = geometry_msgs::msg::PointStamped;
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;

namespace pb2025_sentry_behavior
{

class CalculateAttackPoseAction : public BT::RosTopicPubNode<visualization_msgs::msg::MarkerArray>
{
public:
  CalculateAttackPoseAction(
    const std::string & instance_name, const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  bool setMessage(visualization_msgs::msg::MarkerArray & msg) override;

  static BT::PortsList providedPorts();

private:
  struct Parameters
  {
    double attack_radius;
    int num_sectors;
    int cost_threshold;
    std::string robot_base_frame;
    double transform_tolerance;
    double max_visualization_distance;
    double marker_scale_base;
    bool visualize;
  };

  std::vector<Point> generateCandidatePoints(const Point & enemy_position);

  std::vector<Point> filterFeasiblePoints(
    const std::vector<Point> & candidates, const nav_msgs::msg::OccupancyGrid & costmap);

  Point selectBestPoint(const std::vector<Point> & feasible_points, const Point & robot_position);

  PoseStamped createAttackPose(const Point & attack_point, const PointStamped & enemy_position);

  void createVisualizationMarkers(
    visualization_msgs::msg::MarkerArray & msg, const Point & enemy_position,
    const std::vector<Point> & candidates, const std::vector<Point> & feasible_points,
    const Point & robot_position, const nav_msgs::msg::OccupancyGrid & costmap);

  bool transformPoseInTargetFrame(
    const PointStamped & input_pose, PointStamped & transformed_pose, tf2_ros::Buffer & tf_buffer,
    const std::string target_frame, const double transform_timeout);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Parameters params_;

  PointStamped enemy_on_costmap_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__CALCULATE_ATTACK_POSE_HPP_
