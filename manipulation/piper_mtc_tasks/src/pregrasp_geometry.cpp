#include "piper_mtc_tasks/pregrasp_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace piper_mtc_tasks
{

std::optional<geometry_msgs::msg::PoseStamped> make_planning_frame_pregrasp_pose(
  const geometry_msgs::msg::PoseStamped & grasp_pose,
  const XYZ & approach_direction,
  double distance)
{
  if (!std::isfinite(distance)) {
    return std::nullopt;
  }

  if (!std::isfinite(approach_direction.x) ||
    !std::isfinite(approach_direction.y) ||
    !std::isfinite(approach_direction.z))
  {
    return std::nullopt;
  }

  const double direction_norm = std::sqrt(
    approach_direction.x * approach_direction.x +
    approach_direction.y * approach_direction.y +
    approach_direction.z * approach_direction.z);
  if (!std::isfinite(direction_norm) || direction_norm <= 1e-6) {
    return std::nullopt;
  }

  const double offset = std::max(distance, 0.0);
  auto pregrasp_pose = grasp_pose;
  pregrasp_pose.pose.position.x -= offset * approach_direction.x / direction_norm;
  pregrasp_pose.pose.position.y -= offset * approach_direction.y / direction_norm;
  pregrasp_pose.pose.position.z -= offset * approach_direction.z / direction_norm;
  return pregrasp_pose;
}

std::optional<geometry_msgs::msg::PoseStamped> make_planning_frame_postgrasp_escape_pose(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const XYZ & approach_direction,
  double retreat_distance,
  double vertical_lift_distance)
{
  if (!std::isfinite(retreat_distance) || retreat_distance <= 0.0 ||
    !std::isfinite(vertical_lift_distance) || vertical_lift_distance <= 0.0 ||
    !std::isfinite(approach_direction.x) || !std::isfinite(approach_direction.y) ||
    !std::isfinite(approach_direction.z) || std::abs(approach_direction.z) > 1e-6)
  {
    return std::nullopt;
  }

  const double direction_norm = std::sqrt(
    approach_direction.x * approach_direction.x +
    approach_direction.y * approach_direction.y +
    approach_direction.z * approach_direction.z);
  if (!std::isfinite(direction_norm) || direction_norm <= 1e-6) {
    return std::nullopt;
  }

  auto escape_pose = current_pose;
  escape_pose.pose.position.x -= retreat_distance * approach_direction.x / direction_norm;
  escape_pose.pose.position.y -= retreat_distance * approach_direction.y / direction_norm;
  escape_pose.pose.position.z += vertical_lift_distance;
  return escape_pose;
}

}  // namespace piper_mtc_tasks
