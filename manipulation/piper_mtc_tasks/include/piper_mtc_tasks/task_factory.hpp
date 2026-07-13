#pragma once

#include <memory>
#include <string>
#include <vector>

#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

#include "piper_mtc_tasks/pick_task.hpp"
#include "piper_mtc_tasks/task_parameters.hpp"

namespace piper_mtc_tasks
{

namespace mtc = moveit::task_constructor;

bool validate_classification_observe_joint_positions(
  const std::vector<double> & joint_positions,
  std::string & error_message);

bool validate_classification_release_corrections(
  const std::vector<XYZ> & release_corrections,
  std::size_t box_count,
  std::string & error_message);

class TaskFactory
{
public:
  TaskFactory(
    const rclcpp::Node::SharedPtr & node,
    const TaskParameters & parameters);

  mtc::Task create_pick_task() const;
  mtc::Task create_place_task() const;
  mtc::Task create_grasp_ik_probe_task() const;
  mtc::Task create_move_home_task() const;

private:
  PlannerBundle create_planners() const;
  void configure_task_properties(mtc::Task & task) const;

  rclcpp::Node::SharedPtr node_;
  TaskParameters parameters_;
};

}  // namespace piper_mtc_tasks
