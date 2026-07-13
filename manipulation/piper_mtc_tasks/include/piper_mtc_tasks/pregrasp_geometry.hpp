#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "piper_mtc_tasks/task_parameters.hpp"

namespace piper_mtc_tasks
{

inline bool is_complete_cartesian_path_fraction(double fraction)
{
  return std::isfinite(fraction) && fraction >= 1.0 - 1e-6;
}

bool has_valid_adjacent_joint_position_samples(
  const std::vector<std::vector<double>> & joint_position_samples,
  double max_delta_rad);

bool has_valid_total_joint_range_samples(
  const std::vector<std::vector<double>> & joint_position_samples,
  double max_range_rad);

std::vector<double> make_adaptive_release_backoff_amounts(
  double overshoot,
  double step,
  double max_backoff);

std::optional<geometry_msgs::msg::PoseStamped> make_planning_frame_pregrasp_pose(
  const geometry_msgs::msg::PoseStamped & grasp_pose,
  const XYZ & approach_direction,
  double distance);

std::optional<geometry_msgs::msg::PoseStamped> make_planning_frame_postgrasp_escape_pose(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const XYZ & approach_direction,
  double retreat_distance,
  double vertical_lift_distance);

}  // namespace piper_mtc_tasks
