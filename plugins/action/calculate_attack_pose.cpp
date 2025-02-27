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

#include "pb2025_sentry_behavior/plugins/action/calculate_attack_pose.hpp"

#include "auto_aim_interfaces/msg/target.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

using nav2_util::declare_parameter_if_not_declared;
namespace pb2025_sentry_behavior
{

CalculateAttackPoseAction::CalculateAttackPoseAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubNode(name, config, params)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter_if_not_declared(node_, name + ".attack_radius", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node_, name + ".num_sectors", rclcpp::ParameterValue(36));
  declare_parameter_if_not_declared(node_, name + ".cost_threshold", rclcpp::ParameterValue(50));
  declare_parameter_if_not_declared(
    node_, name + ".robot_base_frame", rclcpp::ParameterValue("chassis"));
  declare_parameter_if_not_declared(
    node_, name + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node_, name + ".max_visualization_distance", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(
    node_, name + ".marker_scale_base", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node_, name + ".visualize", rclcpp::ParameterValue(false));

  node_->get_parameter(name + ".attack_radius", params_.attack_radius);
  node_->get_parameter(name + ".num_sectors", params_.num_sectors);
  node_->get_parameter(name + ".cost_threshold", params_.cost_threshold);
  node_->get_parameter(name + ".robot_base_frame", params_.robot_base_frame);
  node_->get_parameter(name + ".transform_tolerance", params_.transform_tolerance);
  node_->get_parameter(name + ".max_visualization_distance", params_.max_visualization_distance);
  node_->get_parameter(name + ".marker_scale_base", params_.marker_scale_base);
  node_->get_parameter(name + ".visualize", params_.visualize);
}

BT::PortsList CalculateAttackPoseAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<nav_msgs::msg::OccupancyGrid>(
      "costmap_port", "{@nav_globalCostmap}", "GlobalCostmap port on blackboard"),
    BT::InputPort<auto_aim_interfaces::msg::Target>(
      "tracker_port", "{@tracker_target}", "Vision target port on blackboard"),
    BT::OutputPort<PoseStamped>(
      "goal", "{attack_pose}", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  });
}

bool CalculateAttackPoseAction::setMessage(visualization_msgs::msg::MarkerArray & msg)
{
  auto global_costmap = getInput<nav_msgs::msg::OccupancyGrid>("costmap_port");
  auto tracker_target = getInput<auto_aim_interfaces::msg::Target>("tracker_port");

  if (!global_costmap) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: costmap_port");
    return false;
  }
  if (!tracker_target) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: tracker_port");
    return false;
  }

  std::vector<Point> candidates;
  std::vector<Point> feasible_points;
  PoseStamped robot_pose;

  // If tracker lost target, use last known center position
  if (!tracker_target->tracking && tracker_target->id == "") {
    RCLCPP_INFO(
      node_->get_logger(),
      "Tracker target is not currently being tracked. Directing to last known position.");
    if (enemy_on_costmap_.point.x == 0 && enemy_on_costmap_.point.y == 0) {
      RCLCPP_WARN(node_->get_logger(), "No last known position to direct to.");
      return false;
    }
    PoseStamped pose;
    pose.pose.position = enemy_on_costmap_.point;
    setOutput("goal", pose);
  } else {
    // Transform enemy position
    PointStamped enemy_point;
    enemy_point.header = tracker_target->header;
    enemy_point.point = tracker_target->position;
    if (!transformPoseInTargetFrame(
          enemy_point, enemy_on_costmap_, *tf_buffer_, global_costmap->header.frame_id,
          params_.transform_tolerance)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to transform enemy position");
      return false;
    }

    // Generate candidate points
    candidates = generateCandidatePoints(enemy_on_costmap_.point);

    // Filter feasible points
    feasible_points = filterFeasiblePoints(candidates, global_costmap.value());
    if (feasible_points.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No feasible attack points found");
      return false;
    }

    // Get robot position
    if (!nav2_util::getCurrentPose(
          robot_pose, *tf_buffer_, global_costmap->header.frame_id, params_.robot_base_frame,
          params_.transform_tolerance)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get robot pose");
      return false;
    }

    // Select best point
    const auto best_point = selectBestPoint(feasible_points, robot_pose.pose.position);

    // Create attack pose
    const auto attack_pose = createAttackPose(best_point, enemy_on_costmap_);
    setOutput("goal", attack_pose);

    // Create visualization
    if (params_.visualize) {
      createVisualizationMarkers(
        msg, enemy_on_costmap_.point, candidates, feasible_points, robot_pose.pose.position,
        global_costmap.value());
    }
  }
  return true;
}

bool CalculateAttackPoseAction::transformPoseInTargetFrame(
  const PointStamped & input_pose, PointStamped & transformed_pose, tf2_ros::Buffer & tf_buffer,
  const std::string target_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose =
      tf_buffer.transform(input_pose, target_frame, tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(logger, "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(logger, "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger, "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(logger, "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s", input_pose.header.frame_id.c_str(),
      target_frame.c_str());
  }

  return false;
}

std::vector<Point> CalculateAttackPoseAction::generateCandidatePoints(const Point & enemy_position)
{
  std::vector<Point> candidates;
  candidates.reserve(params_.num_sectors);

  for (int i = 0; i < params_.num_sectors; ++i) {
    const double angle = i * 2 * M_PI / params_.num_sectors;
    Point p;
    p.x = enemy_position.x + params_.attack_radius * cos(angle);
    p.y = enemy_position.y + params_.attack_radius * sin(angle);
    p.z = 0.0;
    candidates.push_back(p);
  }
  return candidates;
}

std::vector<Point> CalculateAttackPoseAction::filterFeasiblePoints(
  const std::vector<Point> & candidates, const nav_msgs::msg::OccupancyGrid & costmap)
{
  std::vector<Point> feasible_points;
  const auto & info = costmap.info;

  for (const auto & p : candidates) {
    const int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
    const int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

    if (
      cell_x < 0 || cell_x >= static_cast<int>(info.width) || cell_y < 0 ||
      cell_y >= static_cast<int>(info.height)) {
      continue;
    }

    const int index = cell_y * info.width + cell_x;
    const int8_t cost = costmap.data[index];
    if (cost >= 0 && cost <= params_.cost_threshold) {
      feasible_points.push_back(p);
    }
  }
  return feasible_points;
}

Point CalculateAttackPoseAction::selectBestPoint(
  const std::vector<Point> & feasible_points, const Point & robot_position)
{
  auto compare = [&](const auto & a, const auto & b) {
    const double dx1 = a.x - robot_position.x;
    const double dy1 = a.y - robot_position.y;
    const double dx2 = b.x - robot_position.x;
    const double dy2 = b.y - robot_position.y;
    return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
  };
  return *std::min_element(feasible_points.begin(), feasible_points.end(), compare);
}

PoseStamped CalculateAttackPoseAction::createAttackPose(
  const Point & attack_point, const PointStamped & enemy_position)
{
  PoseStamped pose;
  pose.header.frame_id = enemy_position.header.frame_id;
  pose.header.stamp = node_->now();
  pose.pose.position = attack_point;

  const double dx = enemy_position.point.x - attack_point.x;
  const double dy = enemy_position.point.y - attack_point.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(dy, dx));
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

void CalculateAttackPoseAction::createVisualizationMarkers(
  visualization_msgs::msg::MarkerArray & msg, const Point & enemy_position,
  const std::vector<Point> & candidates, const std::vector<Point> & feasible_points,
  const Point & robot_position, const nav_msgs::msg::OccupancyGrid & costmap)
{
  msg.markers.clear();

  // Enemy marker
  visualization_msgs::msg::Marker enemy_marker;
  enemy_marker.header.frame_id = costmap.header.frame_id;
  enemy_marker.ns = "enemy";
  enemy_marker.id = 0;
  enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
  enemy_marker.pose.position = enemy_position;
  enemy_marker.scale.x = enemy_marker.scale.y = enemy_marker.scale.z = 0.3;
  enemy_marker.color.b = 1.0;
  enemy_marker.color.a = 1.0;
  msg.markers.push_back(enemy_marker);

  // Best attack marker
  const auto best_point = selectBestPoint(feasible_points, robot_position);
  visualization_msgs::msg::Marker best_marker;
  best_marker.header.frame_id = costmap.header.frame_id;
  best_marker.ns = "best";
  best_marker.id = 0;
  best_marker.type = visualization_msgs::msg::Marker::SPHERE;
  best_marker.pose.position = best_point;
  best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = 0.4;
  best_marker.color.g = 1.0;
  best_marker.color.a = 1.0;
  msg.markers.push_back(best_marker);

  // Candidate markers
  const auto & info = costmap.info;
  for (size_t i = 0; i < candidates.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap.header.frame_id;
    marker.ns = "candidates";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position = candidates[i];

    // Calculate cost
    const int cell_x = (candidates[i].x - info.origin.position.x) / info.resolution;
    const int cell_y = (candidates[i].y - info.origin.position.y) / info.resolution;
    int8_t cost = -1;
    if (
      cell_x >= 0 && cell_x < static_cast<int>(info.width) && cell_y >= 0 &&
      cell_y < static_cast<int>(info.height)) {
      cost = costmap.data[cell_y * info.width + cell_x];
    }

    // Visual properties
    marker.scale.x = marker.scale.y = marker.scale.z =
      params_.marker_scale_base + (cost / 100.0) * 0.3;

    const double distance =
      std::hypot(candidates[i].x - robot_position.x, candidates[i].y - robot_position.y);
    const float alpha =
      0.5 + 0.5 * (1.0 - std::min(distance / params_.max_visualization_distance, 1.0));

    marker.color.r = 1.0;
    marker.color.a = alpha;

    // Check feasibility
    const bool is_feasible = std::any_of(
      feasible_points.begin(), feasible_points.end(),
      [&](const auto & p) { return p.x == candidates[i].x && p.y == candidates[i].y; });

    if (!is_feasible) {
      marker.color.a *= 0.3;
    }

    msg.markers.push_back(marker);
  }

  // Range circle
  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = costmap.header.frame_id;
  circle.ns = "range";
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.pose.position.z = 0.05;
  circle.scale.x = 0.05;
  circle.color.b = 1.0;
  circle.color.a = 0.5;

  constexpr int circle_points = 36;
  for (int i = 0; i <= circle_points; ++i) {
    const double angle = i * 2 * M_PI / circle_points;
    Point p;
    p.x = enemy_position.x + params_.attack_radius * cos(angle);
    p.y = enemy_position.y + params_.attack_radius * sin(angle);
    circle.points.push_back(p);
  }
  msg.markers.push_back(circle);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::CalculateAttackPoseAction, "CalculateAttackPose");
