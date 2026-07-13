#include "piper_mtc_tasks/pregrasp_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace piper_mtc_tasks
{

bool has_valid_adjacent_joint_position_samples(
  const std::vector<std::vector<double>> & joint_position_samples,
  double max_delta_rad)
{
  if (!std::isfinite(max_delta_rad) || max_delta_rad <= 0.0 ||
    joint_position_samples.size() < 2)
  {
    return false;
  }

  const std::size_t joint_count = joint_position_samples.front().size();
  if (joint_count == 0) {
    return false;
  }

  for (std::size_t sample_index = 0; sample_index < joint_position_samples.size(); ++sample_index) {
    const auto & sample = joint_position_samples[sample_index];
    if (sample.size() != joint_count) {
      return false;
    }

    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      if (!std::isfinite(sample[joint_index]) ||
        (sample_index > 0 &&
        std::abs(sample[joint_index] - joint_position_samples[sample_index - 1][joint_index]) >
        max_delta_rad))
      {
        return false;
      }
    }
  }

  return true;
}

bool has_valid_total_joint_range_samples(
  const std::vector<std::vector<double>> & joint_position_samples,
  double max_range_rad)
{
  if (!std::isfinite(max_range_rad) || max_range_rad <= 0.0 ||
    joint_position_samples.size() < 2)
  {
    return false;
  }

  const std::size_t joint_count = joint_position_samples.front().size();
  if (joint_count == 0) {
    return false;
  }

  std::vector<double> minimum_positions = joint_position_samples.front();
  std::vector<double> maximum_positions = joint_position_samples.front();
  for (const auto & sample : joint_position_samples) {
    if (sample.size() != joint_count) {
      return false;
    }

    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      if (!std::isfinite(sample[joint_index])) {
        return false;
      }
      minimum_positions[joint_index] = std::min(minimum_positions[joint_index], sample[joint_index]);
      maximum_positions[joint_index] = std::max(maximum_positions[joint_index], sample[joint_index]);
      if (maximum_positions[joint_index] - minimum_positions[joint_index] > max_range_rad) {
        return false;
      }
    }
  }

  return true;
}

std::vector<double> make_adaptive_release_backoff_amounts(
  double overshoot,
  double step,
  double max_backoff)
{
  constexpr double kEpsilon = 1e-6;
  if (!std::isfinite(overshoot) || !std::isfinite(step) || !std::isfinite(max_backoff) ||
    overshoot <= kEpsilon || max_backoff <= kEpsilon)
  {
    return {};
  }

  std::vector<double> amounts{
    std::min(max_backoff, overshoot)};
  const double backoff_step = std::max(0.0, step);
  if (backoff_step > kEpsilon) {
    amounts.push_back(std::min(max_backoff, overshoot + backoff_step));
    amounts.push_back(std::min(max_backoff, overshoot + 2.0 * backoff_step));

    for (double amount = backoff_step; amount < max_backoff - kEpsilon;
      amount += backoff_step)
    {
      amounts.push_back(amount);
    }
    amounts.push_back(max_backoff);
  }

  std::sort(amounts.begin(), amounts.end());
  amounts.erase(
    std::unique(
      amounts.begin(), amounts.end(),
      [](double lhs, double rhs) {
        return std::abs(lhs - rhs) < kEpsilon;
      }),
    amounts.end());
  return amounts;
}

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
