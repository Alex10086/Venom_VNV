#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/task.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#if defined(HAVE_LINKATTACHER_MSGS)
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <venom_manipulation_interfaces/action/execute_task.hpp>
#include <venom_manipulation_interfaces/msg/grasp_target.hpp>

#include "piper_mtc_tasks/scene_manager.hpp"
#include "piper_mtc_tasks/pregrasp_geometry.hpp"
#include "piper_mtc_tasks/stage_builders.hpp"
#include "piper_mtc_tasks/task_factory.hpp"

namespace piper_mtc_tasks
{

namespace
{

enum class GripperCloseOutcome
{
  kFailed,
  kContact,
  kFullyClosed
};

enum class VisualStylePickOutcome
{
  kSuccess,
  kExhausted,
  kFailed,
  kRefreshTarget
};

struct FixedPlacePoseCandidate
{
  int64_t index{0};
  XYZ position;
};

double choose_nearest_bounded_angle(
  double current_angle,
  double desired_angle,
  const moveit::core::VariableBounds & bounds)
{
  double best_angle = desired_angle;
  double best_distance = std::numeric_limits<double>::infinity();

  for (int wraps = -1; wraps <= 1; ++wraps) {
    const double candidate = desired_angle + static_cast<double>(wraps) * 2.0 * M_PI;
    if (bounds.position_bounded_) {
      if (candidate < bounds.min_position_ || candidate > bounds.max_position_) {
        continue;
      }
    }

    const double distance = std::abs(candidate - current_angle);
    if (distance < best_distance) {
      best_distance = distance;
      best_angle = candidate;
    }
  }

  if (!std::isfinite(best_distance)) {
    best_angle = std::min(std::max(desired_angle, bounds.min_position_), bounds.max_position_);
  }

  return best_angle;
}

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double positive_or(double value, double fallback)
{
  return value > 1e-6 ? value : fallback;
}

bool has_cartesian_distance(double min_distance, double max_distance)
{
  constexpr double kDistanceEpsilon = 1e-6;
  return std::abs(min_distance) > kDistanceEpsilon ||
         std::abs(max_distance) > kDistanceEpsilon;
}

std::vector<double> make_single_candidate(double value)
{
  return std::vector<double>{value};
}

void append_unique_angle_candidate(std::vector<double> & candidates, double angle)
{
  constexpr double kAngleTolerance = 1e-6;
  const double normalized_angle = normalize_angle(angle);
  const auto existing = std::find_if(
    candidates.begin(),
    candidates.end(),
    [normalized_angle](double candidate) {
      return std::abs(normalize_angle(candidate - normalized_angle)) < kAngleTolerance;
    });
  if (existing == candidates.end()) {
    candidates.push_back(normalized_angle);
  }
}

double choose_good_enough_object_angle(
  double current_object_angle,
  double object_radius_from_base,
  const SceneOpenTopBin & bin,
  double object_radius)
{
  (void)current_object_angle;
  (void)object_radius_from_base;
  (void)object_radius;

  // For coarse pre-place rotation, prioritize pointing the carried object toward the
  // bin center direction instead of rotating all the way to an "inside-bin optimal"
  // angle. This keeps joint1 from overshooting when a simple release is enough.
  return std::atan2(bin.center.y, bin.center.x);
}

std::string normalize_class_name(const std::string & value)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const char character : value) {
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
  return normalized;
}

std::string describe_bounds_violations(
  const moveit::core::RobotState & state,
  const std::string & label)
{
  constexpr double kTolerance = 1e-9;
  std::ostringstream out;
  bool any_violation = false;

  const auto & variable_names = state.getVariableNames();
  const auto & robot_model = state.getRobotModel();
  for (const auto & variable_name : variable_names) {
    const auto & bounds = robot_model->getVariableBounds(variable_name);
    if (!bounds.position_bounded_) {
      continue;
    }

    const double value = state.getVariablePosition(variable_name);
    const bool below = value < bounds.min_position_ - kTolerance;
    const bool above = value > bounds.max_position_ + kTolerance;
    if (!below && !above) {
      continue;
    }

    if (!any_violation) {
      out << label << " bounds violations:";
      any_violation = true;
    }

    out << "\n  " << variable_name << " = " << std::fixed << std::setprecision(9) << value
        << " outside [" << bounds.min_position_ << ", " << bounds.max_position_ << "]";
  }

  if (!any_violation) {
    out << label << " has no strict position bounds violations.";
  }
  return out.str();
}

bool has_bounds_violation(const moveit::core::RobotState & state)
{
  constexpr double kTolerance = 1e-9;
  const auto & variable_names = state.getVariableNames();
  const auto & robot_model = state.getRobotModel();
  for (const auto & variable_name : variable_names) {
    const auto & bounds = robot_model->getVariableBounds(variable_name);
    if (!bounds.position_bounded_) {
      continue;
    }

    const double value = state.getVariablePosition(variable_name);
    if (value < bounds.min_position_ - kTolerance ||
      value > bounds.max_position_ + kTolerance)
    {
      return true;
    }
  }
  return false;
}

std::string describe_solution_state_bounds(
  const mtc::SolutionBase & solution,
  const std::string & prefix)
{
  std::ostringstream out;
  if (solution.start() != nullptr && solution.start()->scene()) {
    out << describe_bounds_violations(
      solution.start()->scene()->getCurrentState(),
      prefix + " start");
  } else {
    out << prefix << " start scene unavailable.";
  }

  out << '\n';

  if (solution.end() != nullptr && solution.end()->scene()) {
    out << describe_bounds_violations(
      solution.end()->scene()->getCurrentState(),
      prefix + " end");
  } else {
    out << prefix << " end scene unavailable.";
  }
  return out.str();
}

std::string describe_collisions(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state,
  const std::string & label)
{
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.contacts = true;
  request.max_contacts = 20;
  request.max_contacts_per_pair = 3;

  scene.checkCollision(request, result, state);
  if (!result.collision) {
    return label + " has no collision contacts.";
  }

  std::ostringstream out;
  out << label << " collision contacts:";
  for (const auto & contact_pair : result.contacts) {
    out << "\n  " << contact_pair.first.first << " <-> " << contact_pair.first.second
        << " count=" << contact_pair.second.size();
  }
  return out.str();
}

std::string describe_solution_state_collisions(
  const mtc::SolutionBase & solution,
  const std::string & prefix)
{
  std::ostringstream out;
  if (solution.start() != nullptr && solution.start()->scene()) {
    const auto & scene = *solution.start()->scene();
    out << describe_collisions(scene, scene.getCurrentState(), prefix + " start");
  } else {
    out << prefix << " start scene unavailable.";
  }

  out << '\n';

  if (solution.end() != nullptr && solution.end()->scene()) {
    const auto & scene = *solution.end()->scene();
    out << describe_collisions(scene, scene.getCurrentState(), prefix + " end");
  } else {
    out << prefix << " end scene unavailable.";
  }
  return out.str();
}

void append_solution_tree(
  const mtc::SolutionBase & solution,
  std::ostringstream & out,
  std::size_t depth)
{
  const std::string indent(depth * 2, ' ');
  const auto * creator = solution.creator();
  out << indent
      << "stage='" << (creator != nullptr ? creator->name() : "<unknown>") << "'";
  if (!solution.comment().empty()) {
    out << " comment='" << solution.comment() << "'";
  }
  out << '\n';

  if (const auto * sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution)) {
    for (const auto * child_solution : sequence->solutions()) {
      if (child_solution == nullptr) {
        continue;
      }
      append_solution_tree(*child_solution, out, depth + 1);
    }
  } else if (const auto * wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution)) {
    if (wrapped->wrapped() != nullptr) {
      append_solution_tree(*wrapped->wrapped(), out, depth + 1);
    }
  }
}

std::string describe_solution_tree(const mtc::SolutionBase & solution)
{
  std::ostringstream out;
  append_solution_tree(solution, out, 0);
  return out.str();
}

void log_stage_solution_bounds(
  const rclcpp::Logger & logger,
  const mtc::Task & task)
{
  task.stages()->traverseRecursively(
    [&logger](const mtc::Stage & stage, unsigned int) {
      std::size_t solution_index = 0;
      for (const auto & solution : stage.solutions()) {
        if (!solution) {
          continue;
        }

        const bool start_invalid =
          solution->start() != nullptr &&
          solution->start()->scene() &&
          has_bounds_violation(solution->start()->scene()->getCurrentState());
        const bool end_invalid =
          solution->end() != nullptr &&
          solution->end()->scene() &&
          has_bounds_violation(solution->end()->scene()->getCurrentState());
        if (start_invalid || end_invalid) {
          RCLCPP_ERROR_STREAM(
            logger,
            "MTC stage solution bounds issue: stage='" << stage.name()
                                                       << "' solution=" << solution_index
                                                       << '\n'
                                                       << describe_solution_state_bounds(
                                                         *solution,
                                                         stage.name() + " solution " +
                                                         std::to_string(solution_index)));
        }
        ++solution_index;
      }

      std::size_t failure_index = 0;
      for (const auto & failure : stage.failures()) {
        if (!failure) {
          continue;
        }

        RCLCPP_ERROR_STREAM(
          logger,
          "MTC stored failure: stage='" << stage.name()
                                       << "' failure=" << failure_index
                                       << " comment='" << failure->comment() << "'\n"
                                       << describe_solution_state_bounds(
                                         *failure,
                                         stage.name() + " failure " +
                                         std::to_string(failure_index))
                                       << '\n'
                                       << describe_solution_state_collisions(
                                         *failure,
                                         stage.name() + " failure " +
                                         std::to_string(failure_index)));
        ++failure_index;
      }

      return true;
    });
}

double duration_to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

void log_solution_trajectory_timing(
  const rclcpp::Logger & logger,
  const mtc::SolutionBase & solution,
  mtc::Introspection * introspection)
{
  moveit_task_constructor_msgs::msg::Solution message;
  solution.toMsg(message, introspection);

  std::size_t index = 0;
  for (const auto & sub_trajectory : message.sub_trajectory) {
    const auto & joint_trajectory = sub_trajectory.trajectory.joint_trajectory;
    const auto point_count = joint_trajectory.points.size();
    if (point_count == 0) {
      RCLCPP_INFO_STREAM(
        logger,
        "MTC trajectory timing: sub=" << index
                                      << " stage_id=" << sub_trajectory.info.stage_id
                                      << " planner='" << sub_trajectory.info.planner_id
                                      << "' joints=[] points=0");
      ++index;
      continue;
    }

    const double first_time =
      duration_to_seconds(joint_trajectory.points.front().time_from_start);
    const double last_time =
      duration_to_seconds(joint_trajectory.points.back().time_from_start);

    bool strictly_increasing = true;
    double previous_time = first_time;
    for (std::size_t point_index = 1; point_index < point_count; ++point_index) {
      const double time =
        duration_to_seconds(joint_trajectory.points[point_index].time_from_start);
      if (time <= previous_time) {
        strictly_increasing = false;
        break;
      }
      previous_time = time;
    }

    RCLCPP_INFO_STREAM(
      logger,
      "MTC trajectory timing: sub=" << index
                                    << " stage_id=" << sub_trajectory.info.stage_id
                                    << " planner='" << sub_trajectory.info.planner_id
                                    << "' joints="
                                    << joint_trajectory.joint_names.size()
                                    << " points=" << point_count
                                    << " first=" << first_time
                                    << " last=" << last_time
                                    << " increasing="
                                    << (strictly_increasing ? "true" : "false"));
    ++index;
  }
}

bool trajectory_has_strictly_increasing_timing(
  const robot_trajectory::RobotTrajectory & trajectory)
{
  const std::size_t waypoint_count = trajectory.getWayPointCount();
  if (waypoint_count < 2) {
    return true;
  }

  double previous_time = trajectory.getWayPointDurationFromStart(0);
  for (std::size_t waypoint_index = 1; waypoint_index < waypoint_count; ++waypoint_index) {
    const double time = trajectory.getWayPointDurationFromStart(waypoint_index);
    if (time <= previous_time) {
      return false;
    }
    previous_time = time;
  }
  return true;
}

std::size_t retime_solution_trajectories(
  mtc::SolutionBase & solution,
  const trajectory_processing::TimeParameterization & time_parameterization,
  double velocity_scaling,
  double acceleration_scaling)
{
  if (auto * sub_trajectory = dynamic_cast<mtc::SubTrajectory *>(&solution)) {
    const auto original_trajectory = sub_trajectory->trajectory();
    if (
      !original_trajectory ||
      original_trajectory->getWayPointCount() < 2 ||
      trajectory_has_strictly_increasing_timing(*original_trajectory))
    {
      return 0;
    }

    auto retimed_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(*original_trajectory, true);
    if (!time_parameterization.computeTimeStamps(
        *retimed_trajectory,
        velocity_scaling,
        acceleration_scaling))
    {
      return 0;
    }

    sub_trajectory->setTrajectory(retimed_trajectory);
    return 1;
  }

  std::size_t retimed_count = 0;
  if (auto * sequence = dynamic_cast<mtc::SolutionSequence *>(&solution)) {
    for (const auto * child_solution : sequence->solutions()) {
      if (child_solution == nullptr) {
        continue;
      }
      retimed_count += retime_solution_trajectories(
        *const_cast<mtc::SolutionBase *>(child_solution),
        time_parameterization,
        velocity_scaling,
        acceleration_scaling);
    }
  } else if (auto * wrapped = dynamic_cast<mtc::WrappedSolution *>(&solution)) {
    if (wrapped->wrapped() != nullptr) {
      retimed_count += retime_solution_trajectories(
        *const_cast<mtc::SolutionBase *>(wrapped->wrapped()),
        time_parameterization,
        velocity_scaling,
        acceleration_scaling);
    }
  }

  return retimed_count;
}

struct ExecutableTrajectoryStep
{
  std::string stage_name;
  moveit_msgs::msg::RobotTrajectory trajectory;
};

void collect_executable_trajectory_steps(
  const mtc::SolutionBase & solution,
  std::vector<ExecutableTrajectoryStep> & steps)
{
  if (const auto * sub_trajectory = dynamic_cast<const mtc::SubTrajectory *>(&solution)) {
    const auto trajectory = sub_trajectory->trajectory();
    if (trajectory && trajectory->getWayPointCount() > 0) {
      ExecutableTrajectoryStep step;
      step.stage_name = sub_trajectory->creator() != nullptr ?
        sub_trajectory->creator()->name() :
        "<unknown>";
      trajectory->getRobotTrajectoryMsg(step.trajectory);
      steps.push_back(std::move(step));
    }
    return;
  }

  if (const auto * sequence = dynamic_cast<const mtc::SolutionSequence *>(&solution)) {
    for (const auto * child_solution : sequence->solutions()) {
      if (child_solution != nullptr) {
        collect_executable_trajectory_steps(*child_solution, steps);
      }
    }
    return;
  }

  if (const auto * wrapped = dynamic_cast<const mtc::WrappedSolution *>(&solution)) {
    if (wrapped->wrapped() != nullptr) {
      collect_executable_trajectory_steps(*wrapped->wrapped(), steps);
    }
  }
}

bool stage_name_contains(const std::string & stage_name, const std::string & token)
{
  return stage_name.find(token) != std::string::npos;
}

bool trajectory_targets_gripper(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  for (const auto & joint_name : trajectory.joint_trajectory.joint_names) {
    if (joint_name == "joint7" || joint_name == "joint8") {
      return true;
    }
  }
  return false;
}

uint8_t feedback_stage_for_step(const std::string & stage_name)
{
  if (stage_name_contains(stage_name, "home")) {
    return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_MOVING_HOME;
  }
  if (stage_name_contains(stage_name, "place")) {
    return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_MOVING_PLACE;
  }
  if (stage_name_contains(stage_name, "open gripper")) {
    return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_OPENING_GRIPPER;
  }
  if (stage_name_contains(stage_name, "close gripper")) {
    return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_CLOSING_GRIPPER;
  }
  if (stage_name_contains(stage_name, "lift")) {
    return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_LIFTING;
  }
  if (stage_name_contains(stage_name, "grasp")) {
    return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_MOVING_GRASP;
  }
  return venom_manipulation_interfaces::action::ExecuteTask::Goal::STAGE_MOVING_PREGRASP;
}

// Keep these task ids aligned with ExecuteTask.action while installed interface headers are stale.
constexpr uint8_t kTaskRepeatVisualPickToPayload = 6u;
constexpr uint8_t kTaskStartFlameTracking = 7u;
constexpr uint8_t kTaskStopFlameTracking = 8u;

}  // namespace

class PickPlaceServer : public rclcpp::Node
{
public:
  using ExecuteTask = venom_manipulation_interfaces::action::ExecuteTask;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GraspTarget = venom_manipulation_interfaces::msg::GraspTarget;
  using SetBool = std_srvs::srv::SetBool;
#if defined(HAVE_LINKATTACHER_MSGS)
  using AttachLink = linkattacher_msgs::srv::AttachLink;
  using DetachLink = linkattacher_msgs::srv::DetachLink;
#endif
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  PickPlaceServer()
  : Node("pick_place_server")
  {
    declare_task_parameters(*this);
    parameters_ = load_task_parameters(*this);
    auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
    factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
    scene_manager_ = std::make_unique<SceneManager>(get_logger());
    target_fusion_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this,
      parameters_.classification_place.target_fusion_node_name);
    arm_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_handle, parameters_.arm_group_name);
    gripper_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_handle, parameters_.gripper_group_name);
    arm_move_group_->setPoseReferenceFrame(parameters_.planning_frame);
    arm_move_group_->setPlanningTime(parameters_.plan_timeout_sec);
    arm_move_group_->setMaxVelocityScalingFactor(parameters_.cartesian_velocity_scaling);
    arm_move_group_->setMaxAccelerationScalingFactor(parameters_.cartesian_acceleration_scaling);
    if (!parameters_.hand_frame.empty() && !arm_move_group_->setEndEffectorLink(parameters_.hand_frame)) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to set arm end effector link to '%s'; explicit lift will query the link by name instead",
        parameters_.hand_frame.c_str());
    }
    action_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gazebo_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    arm_direct_trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      "/arm_controller/follow_joint_trajectory",
      action_callback_group_);

    if (parameters_.enable_gazebo_attachment) {
#if defined(HAVE_LINKATTACHER_MSGS)
      gazebo_attach_link_client_ = create_client<AttachLink>(
        "/ATTACHLINK",
        rmw_qos_profile_services_default,
        gazebo_callback_group_);
      gazebo_detach_link_client_ = create_client<DetachLink>(
        "/DETACHLINK",
        rmw_qos_profile_services_default,
        gazebo_callback_group_);
#else
      RCLCPP_WARN(
        get_logger(),
        "Gazebo attachment requested, but linkattacher_msgs is unavailable. Disabling gazebo attachment support.");
      parameters_.enable_gazebo_attachment = false;
#endif
    }

    rclcpp::SubscriptionOptions joint_state_options;
    joint_state_options.callback_group = gazebo_callback_group_;
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(&PickPlaceServer::handle_joint_state, this, std::placeholders::_1),
      joint_state_options);
    grasp_target_subscription_ = create_subscription<GraspTarget>(
      parameters_.vision_target.grasp_target_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PickPlaceServer::handle_grasp_target, this, std::placeholders::_1));
    target_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      parameters_.vision_target.target_valid_topic,
      10,
      std::bind(&PickPlaceServer::handle_target_valid, this, std::placeholders::_1));
    classification_grasp_target_subscription_ = create_subscription<GraspTarget>(
      parameters_.classification_place.grasp_target_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PickPlaceServer::handle_classification_grasp_target, this, std::placeholders::_1));
    classification_target_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      parameters_.classification_place.target_valid_topic,
      10,
      std::bind(&PickPlaceServer::handle_classification_target_valid, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this,
      parameters_.action_name,
      std::bind(&PickPlaceServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceServer::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      action_callback_group_);

    if (parameters_.autostart_task_type > 0) {
      autostart_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&PickPlaceServer::run_autostart_task, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "MTC pick_place_server ready on '%s' for arm group '%s' and gripper group '%s'",
      parameters_.action_name.c_str(),
      parameters_.arm_group_name.c_str(),
      parameters_.gripper_group_name.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Vision target topics: pick='%s', classify='%s'",
      parameters_.vision_target.grasp_target_topic.c_str(),
      parameters_.classification_place.grasp_target_topic.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTask::Goal> goal)
  {
    if (goal->task_type != ExecuteTask::Goal::PICK_AND_PLACE_FIXED &&
      goal->task_type != ExecuteTask::Goal::MOVE_HOME &&
      goal->task_type != ExecuteTask::Goal::MOVE_OBSERVE &&
      goal->task_type != ExecuteTask::Goal::PICK_AND_PLACE_LATEST_TARGET &&
      goal->task_type != ExecuteTask::Goal::CLASSIFY_PLATFORM_TO_COLOR_BOXES &&
      goal->task_type != kTaskRepeatVisualPickToPayload &&
      goal->task_type != kTaskStartFlameTracking &&
      goal->task_type != kTaskStopFlameTracking)
    {
      RCLCPP_WARN(get_logger(), "Rejecting unsupported task type %u", goal->task_type);
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(current_task_mutex_);
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "Rejecting task type %u because another MTC task is active", goal->task_type);
      return rclcpp_action::GoalResponse::REJECT;
    }
    active_goal_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTask>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    std::lock_guard<std::mutex> lock(current_task_mutex_);
    if (current_task_ != nullptr) {
      current_task_->preempt();
      RCLCPP_INFO(get_logger(), "Forwarded cancel request to current MTC task");
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    std::thread(
      [this, goal_handle]() {
        execute_goal(goal_handle);
      }).detach();
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    uint8_t stage,
    const std::string & message)
  {
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->current_stage = stage;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void finish_result(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    bool success,
    uint8_t stage_reached,
    int32_t error_code,
    const std::string & message)
  {
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = success;
    result->stage_reached = stage_reached;
    result->error_code = error_code;
    result->message = message;

    if (success) {
      goal_handle->succeed(result);
    } else if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void execute_goal(const std::shared_ptr<GoalHandleExecuteTask> & goal_handle)
  {
    const auto task_type = goal_handle->get_goal()->task_type;
    const bool move_home_task = task_type == ExecuteTask::Goal::MOVE_HOME;
    const bool observe_task = task_type == ExecuteTask::Goal::MOVE_OBSERVE;
    const bool vision_pick_task = task_type == ExecuteTask::Goal::PICK_AND_PLACE_LATEST_TARGET;
    const bool classification_task =
      task_type == ExecuteTask::Goal::CLASSIFY_PLATFORM_TO_COLOR_BOXES;
    const bool repeat_visual_pick_task =
      task_type == kTaskRepeatVisualPickToPayload;
    const bool flame_tracking_start_task =
      task_type == kTaskStartFlameTracking;
    const bool flame_tracking_stop_task =
      task_type == kTaskStopFlameTracking;
    const bool pick_task =
      task_type == ExecuteTask::Goal::PICK_AND_PLACE_FIXED || vision_pick_task;
    const bool diagnostic_ik_only = pick_task && parameters_.diagnostic_ik_only;
    const bool arm_motion_task =
      move_home_task || observe_task || pick_task || classification_task || repeat_visual_pick_task;

    try {
      if (arm_motion_task && parameters_.require_fresh_joint_states) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_IDLE,
          "Checking Piper joint state freshness");

        std::string joint_state_error_message;
        if (!wait_for_fresh_arm_joint_state(
            parameters_.joint_state_wait_timeout_sec,
            parameters_.joint_state_max_age_sec,
            joint_state_error_message))
        {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_IDLE,
            ExecuteTask::Result::ERROR_NOT_READY,
            joint_state_error_message);
          clear_current_task(nullptr);
          return;
        }
      }

      if (observe_task) {
        execute_observe_goal(goal_handle);
        return;
      }

      if (classification_task) {
        execute_classification_place_goal(goal_handle);
        return;
      }

      if (repeat_visual_pick_task) {
        execute_repeat_visual_pick_goal(goal_handle);
        return;
      }

      if (flame_tracking_start_task || flame_tracking_stop_task) {
        execute_flame_tracking_goal(goal_handle, flame_tracking_start_task);
        return;
      }

      const auto baseline_parameters = parameters_;
      if (pick_task && parameters_.move_home_before_pick) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_HOME,
          "Moving arm home before pick");

        std::string home_error_message;
        if (!execute_named_arm_target(
            parameters_.arm_home_named_target,
            "home before pick",
            home_error_message))
        {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_MOVING_HOME,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            home_error_message.empty() ?
            "Failed to move arm home before pick." :
            home_error_message);
          clear_current_task(nullptr);
          return;
        }

        if (parameters_.move_home_open_gripper) {
          publish_feedback(
            goal_handle,
            ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
            "Opening gripper fully at home");
          if (parameters_.gripper_open_joint7 >= -0.5) {
            publish_gripper_target(parameters_.gripper_open_joint7, 0.40);
          } else {
            std::string open_gripper_error_message;
            if (!execute_named_gripper_target(
                parameters_.gripper_open_named_target,
                "opening gripper at home",
                open_gripper_error_message))
            {
              finish_result(
                goal_handle,
                false,
                ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
                ExecuteTask::Result::ERROR_EXECUTION_FAILED,
                open_gripper_error_message.empty() ?
                "Failed to open gripper fully at home." :
                open_gripper_error_message);
              clear_current_task(nullptr);
              return;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(600));
        }

        if (vision_pick_task && parameters_.vision_target.wait_after_home_timeout_sec > 0.0) {
          publish_feedback(
            goal_handle,
            ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
            "Waiting for a fresh visual target after home");

          std::string fresh_target_error_message;
          if (!wait_for_fresh_visual_target_after(
              now(),
              parameters_.vision_target.wait_after_home_timeout_sec,
              fresh_target_error_message))
          {
            finish_result(
              goal_handle,
              false,
              ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
              ExecuteTask::Result::ERROR_NOT_READY,
              fresh_target_error_message);
            clear_current_task(nullptr);
            return;
          }
        }
      }

      if (vision_pick_task) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
          "Resolving latest visual grasp target");

        std::string target_error_message;
        if (!prepare_visual_pick_target(target_error_message)) {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
            ExecuteTask::Result::ERROR_NOT_READY,
            target_error_message);
          clear_current_task(nullptr);
          return;
        }

        if (parameters_.pick_only && parameters_.use_direct_visual_pick_fallback) {
          std::string direct_pick_error_message;
          if (execute_direct_visual_pick(goal_handle, direct_pick_error_message)) {
            parameters_ = baseline_parameters;
            auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
            factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
            clear_current_task(nullptr);
            finish_result(
              goal_handle,
              true,
              ExecuteTask::Goal::STAGE_DONE,
              ExecuteTask::Result::ERROR_NONE,
              "Direct visual pick completed.");
          } else {
            parameters_ = baseline_parameters;
            auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
            factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
            clear_current_task(nullptr);
            finish_result(
              goal_handle,
              false,
              ExecuteTask::Goal::STAGE_FAILED,
              ExecuteTask::Result::ERROR_EXECUTION_FAILED,
              direct_pick_error_message.empty() ?
              "Direct visual pick failed." :
              direct_pick_error_message);
          }
          return;
        }
      }

      publish_feedback(
        goal_handle,
        move_home_task ?
        ExecuteTask::Goal::STAGE_MOVING_HOME :
        ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
        "Building MTC task");

      mtc::Task task =
        move_home_task ?
        factory_->create_move_home_task() :
        diagnostic_ik_only ?
        factory_->create_grasp_ik_probe_task() :
        factory_->create_pick_task();
      set_current_task(&task);

      if (pick_task) {
        detach_pick_object_from_moveit();
      }

      if (pick_task && !parameters_.disable_scene_objects && !scene_manager_->sync_pick_scene(parameters_))
      {
        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          false,
          ExecuteTask::Goal::STAGE_FAILED,
          ExecuteTask::Result::ERROR_EXECUTION_FAILED,
          "Failed to synchronize pick scene before MTC planning.");
        return;
      }

      try {
        task.init();
      } catch (const mtc::InitStageException & exception) {
        std::ostringstream message;
        message << exception;
        RCLCPP_ERROR_STREAM(get_logger(), "MTC init failed:\n" << message.str());
        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          false,
          ExecuteTask::Goal::STAGE_FAILED,
          ExecuteTask::Result::ERROR_EXECUTION_FAILED,
          "MTC init failed: " + message.str());
        return;
      }

      publish_feedback(
        goal_handle,
        move_home_task ?
        ExecuteTask::Goal::STAGE_MOVING_HOME :
        ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
        "Planning MTC task");

      try {
        task.plan(static_cast<std::size_t>(parameters_.max_solutions));
      } catch (const mtc::InitStageException & exception) {
        std::ostringstream message;
        message << exception;
        RCLCPP_ERROR_STREAM(get_logger(), "MTC plan initialization failed:\n" << message.str());
        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          false,
          ExecuteTask::Goal::STAGE_FAILED,
          ExecuteTask::Result::ERROR_EXECUTION_FAILED,
          "MTC plan initialization failed: " + message.str());
        return;
      }
      if (task.solutions().empty()) {
        std::ostringstream state;
        task.printState(state);
        RCLCPP_ERROR_STREAM(get_logger(), "MTC planning state:\n" << state.str());

        std::ostringstream failure_explanation;
        task.explainFailure(failure_explanation);
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "MTC planning failure explanation:\n" << failure_explanation.str());
        log_stage_solution_bounds(get_logger(), task);

        std::size_t failure_index = 0;
        for (const auto & failure : task.failures()) {
          if (!failure) {
            continue;
          }

          const auto * creator = failure->creator();
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "MTC failure #" << failure_index
                            << " stage='" << (creator != nullptr ? creator->name() : "<unknown>")
                            << "' comment='" << failure->comment() << "'\n"
                            << describe_solution_state_bounds(
                              *failure,
                              "failure #" + std::to_string(failure_index)));
          ++failure_index;
        }

        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          false,
          ExecuteTask::Goal::STAGE_FAILED,
          ExecuteTask::Result::ERROR_PLAN_FAILED,
          "MTC planning failed. Inspect the task in RViz for the failing stage.");
        return;
      }

      auto & solutions = task.solutions();
      trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
      const std::size_t retimed_count = retime_solution_trajectories(
        *const_cast<mtc::SolutionBase *>(solutions.front().get()),
        time_parameterization,
        parameters_.cartesian_velocity_scaling,
        parameters_.cartesian_acceleration_scaling);
      if (retimed_count > 0) {
        RCLCPP_WARN(
          get_logger(),
          "Retimed %zu MTC sub-trajectories with non-increasing timestamps before execution",
          retimed_count);
      }

      task.introspection().publishSolution(*solutions.front());
      RCLCPP_INFO_STREAM(
        get_logger(),
        "MTC chosen solution tree:\n" << describe_solution_tree(*solutions.front()));
      log_solution_trajectory_timing(
        get_logger(),
        *solutions.front(),
        &task.introspection());

      if (diagnostic_ik_only) {
        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          true,
          ExecuteTask::Goal::STAGE_DONE,
          ExecuteTask::Result::ERROR_NONE,
          "Grasp IK probe generated at least one solution. Execution is intentionally skipped.");
        return;
      }

      if (!parameters_.execute_on_plan) {
        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          true,
          ExecuteTask::Goal::STAGE_DONE,
          ExecuteTask::Result::ERROR_NONE,
          "MTC plan generated successfully. Execution was disabled by parameter.");
        return;
      }

      publish_feedback(
        goal_handle,
        move_home_task ?
        ExecuteTask::Goal::STAGE_MOVING_HOME :
        ExecuteTask::Goal::STAGE_CLOSING_GRIPPER,
        pick_task ? "Executing pick MTC solution" : "Executing MTC solution");

      if (pick_task) {
        detach_pick_object_from_gazebo_link_attacher();
      }
      stop_gripper_hold();

      moveit::core::MoveItErrorCode execution_result(moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      std::string execution_error_message;
      const bool use_contact_aware_close =
        pick_task &&
        parameters_.use_contact_aware_gripper_close &&
        parameters_.gripper_close_joint7 >= 0.0;

      if (use_contact_aware_close) {
        if (!execute_solution_with_contact_aware_gripper_close(
            goal_handle,
            *solutions.front(),
            execution_error_message))
        {
          execution_result.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
        }
      } else {
        execution_result = task.execute(*solutions.front());
      }

      if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        if (pick_task) {
          detach_pick_object_from_gazebo_link_attacher();
          detach_pick_object_from_moveit();
        }
        stop_gripper_hold();
        parameters_ = baseline_parameters;
        auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
        factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
        clear_current_task(&task);
        finish_result(
          goal_handle,
          false,
          ExecuteTask::Goal::STAGE_FAILED,
          ExecuteTask::Result::ERROR_EXECUTION_FAILED,
          execution_error_message.empty() ?
          "MTC execution failed with MoveIt error code " +
          std::to_string(execution_result.val) :
          execution_error_message);
        return;
      }

      if (pick_task && parameters_.enable_gazebo_attachment && !gazebo_link_attached()) {
        RCLCPP_WARN(
          get_logger(),
          "MTC execution succeeded, but the official Gazebo link attacher never latched '%s'",
          parameters_.pickup_object.id.c_str());
      }

      if (pick_task) {
        if (parameters_.pick_only) {
          parameters_ = baseline_parameters;
          auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
          factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
          clear_current_task(&task);
          finish_result(
            goal_handle,
            true,
            ExecuteTask::Goal::STAGE_DONE,
            ExecuteTask::Result::ERROR_NONE,
            "Pick-only task completed.");
          return;
        }

        if (!execute_pre_place_alignment(goal_handle, execution_error_message)) {
          detach_pick_object_from_gazebo_link_attacher();
          detach_pick_object_from_moveit();
          stop_gripper_hold();
          parameters_ = baseline_parameters;
          auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
          factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
          clear_current_task(&task);
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            execution_error_message);
          return;
        }

        if (parameters_.place_target.release_after_pre_place) {
          publish_feedback(
            goal_handle,
            ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
            "Releasing object from pre-place pose");

          const bool release_success =
            execute_direct_release_fallback(goal_handle, execution_error_message);
          stop_gripper_hold();
          parameters_ = baseline_parameters;
          auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
          factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);

          if (release_success) {
            finish_result(
              goal_handle,
              true,
              ExecuteTask::Goal::STAGE_DONE,
              ExecuteTask::Result::ERROR_NONE,
              "Released object from pre-place pose.");
          } else {
            finish_result(
              goal_handle,
              false,
              ExecuteTask::Goal::STAGE_FAILED,
              ExecuteTask::Result::ERROR_EXECUTION_FAILED,
              execution_error_message.empty() ?
              "Failed to release object from pre-place pose." :
              execution_error_message);
          }
          clear_current_task(nullptr);
          return;
        }

        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          "Planning place MTC task");

        mtc::Task place_task = factory_->create_place_task();
        set_current_task(&place_task);

        try {
          place_task.init();
        } catch (const mtc::InitStageException & exception) {
          std::ostringstream message;
          message << exception;
          RCLCPP_ERROR_STREAM(get_logger(), "Place MTC init failed:\n" << message.str());
          stop_gripper_hold();
          parameters_ = baseline_parameters;
          auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
          factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
          clear_current_task(&place_task);
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            "Place MTC init failed: " + message.str());
          return;
        }

        place_task.plan(static_cast<std::size_t>(parameters_.max_solutions));
        if (place_task.solutions().empty()) {
          std::ostringstream state;
          place_task.printState(state);
          RCLCPP_ERROR_STREAM(get_logger(), "Place MTC planning state:\n" << state.str());

          std::ostringstream failure_explanation;
          place_task.explainFailure(failure_explanation);
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Place MTC planning failure explanation:\n" << failure_explanation.str());
          log_stage_solution_bounds(get_logger(), place_task);

          if (parameters_.place_target.allow_direct_release_fallback) {
            publish_feedback(
              goal_handle,
              ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
              "Place planning failed, releasing object with direct fallback");

            const bool fallback_success =
              execute_direct_release_fallback(goal_handle, execution_error_message);
            clear_current_task(&place_task);
            stop_gripper_hold();
            parameters_ = baseline_parameters;
            auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
            factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);

            if (fallback_success) {
              finish_result(
                goal_handle,
                true,
                ExecuteTask::Goal::STAGE_DONE,
                ExecuteTask::Result::ERROR_NONE,
                "Place planning failed, but direct release fallback completed.");
            } else {
              finish_result(
                goal_handle,
                false,
                ExecuteTask::Goal::STAGE_FAILED,
                ExecuteTask::Result::ERROR_EXECUTION_FAILED,
                execution_error_message.empty() ?
                "Place planning failed and direct release fallback also failed." :
                execution_error_message);
            }
            return;
          }

          stop_gripper_hold();
          parameters_ = baseline_parameters;
          auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
          factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
          clear_current_task(&place_task);
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_PLAN_FAILED,
            "Place MTC planning failed. Inspect the task in RViz for the failing stage.");
          return;
        }

        auto & place_solutions = place_task.solutions();
        trajectory_processing::IterativeParabolicTimeParameterization place_time_parameterization;
        const std::size_t place_retimed_count = retime_solution_trajectories(
          *const_cast<mtc::SolutionBase *>(place_solutions.front().get()),
          place_time_parameterization,
          parameters_.cartesian_velocity_scaling,
          parameters_.cartesian_acceleration_scaling);
        if (place_retimed_count > 0) {
          RCLCPP_WARN(
            get_logger(),
            "Retimed %zu place MTC sub-trajectories with non-increasing timestamps before execution",
            place_retimed_count);
        }

        place_task.introspection().publishSolution(*place_solutions.front());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "MTC chosen place solution tree:\n" << describe_solution_tree(*place_solutions.front()));
        log_solution_trajectory_timing(
          get_logger(),
          *place_solutions.front(),
          &place_task.introspection());

        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          "Executing place MTC solution");

        const auto place_execution_result =
          execute_solution_with_contact_aware_gripper_close(
          goal_handle,
          *place_solutions.front(),
          execution_error_message,
          false);
        stop_gripper_hold();
        clear_current_task(&place_task);

        if (!place_execution_result) {
          if (parameters_.place_target.allow_direct_release_fallback) {
            RCLCPP_WARN(
              get_logger(),
              "Place MTC execution failed after the object was already picked. "
              "Attempting direct release fallback from the current pose.");
            std::string fallback_error_message;
            publish_feedback(
              goal_handle,
              ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
              "Place execution failed, releasing object with direct fallback");
            const bool fallback_success =
              execute_direct_release_fallback(goal_handle, fallback_error_message);
            detach_pick_object_from_gazebo_link_attacher();
            detach_pick_object_from_moveit();
            parameters_ = baseline_parameters;
            auto fallback_node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
            factory_ = std::make_unique<TaskFactory>(fallback_node_handle, parameters_);

            if (fallback_success) {
              finish_result(
                goal_handle,
                true,
                ExecuteTask::Goal::STAGE_DONE,
                ExecuteTask::Result::ERROR_NONE,
                "Place execution failed, but direct release fallback completed.");
            } else {
              finish_result(
                goal_handle,
                false,
                ExecuteTask::Goal::STAGE_FAILED,
                ExecuteTask::Result::ERROR_EXECUTION_FAILED,
                fallback_error_message.empty() ?
                "Place execution failed and direct release fallback also failed." :
                fallback_error_message);
            }
            return;
          }

          detach_pick_object_from_gazebo_link_attacher();
          detach_pick_object_from_moveit();
          parameters_ = baseline_parameters;
          auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
          factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            execution_error_message.empty() ?
            "Place MTC execution failed." :
            execution_error_message);
          return;
        }
      }

      stop_gripper_hold();
      parameters_ = baseline_parameters;
      auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
      factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
      finish_result(
        goal_handle,
        true,
        ExecuteTask::Goal::STAGE_DONE,
        ExecuteTask::Result::ERROR_NONE,
        move_home_task ?
        "Move-home MTC task completed." :
        "Pick-and-place MTC task completed.");
      clear_current_task(nullptr);
    } catch (const std::exception & exception) {
      detach_pick_object_from_gazebo_link_attacher();
      detach_pick_object_from_moveit();
      stop_gripper_hold();
      auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
      factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
      clear_current_task(nullptr);
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_EXECUTION_FAILED,
        std::string("MTC task failed: ") + exception.what());
    }
  }

  bool execute_observe_pose_move(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const std::string & feedback_message,
    std::string & error_message)
  {
    if (!parameters_.observe_pose.enabled) {
      error_message = "Observe pose is not enabled in configuration.";
      return false;
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
      feedback_message);

    geometry_msgs::msg::PoseStamped observe_pose;
    observe_pose.header.frame_id = parameters_.planning_frame;
    observe_pose.pose.position.x = parameters_.observe_pose.position.x;
    observe_pose.pose.position.y = parameters_.observe_pose.position.y;
    observe_pose.pose.position.z = parameters_.observe_pose.position.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(
      parameters_.observe_pose.orientation.roll,
      parameters_.observe_pose.orientation.pitch,
      parameters_.observe_pose.orientation.yaw);
    observe_pose.pose.orientation.x = quaternion.x();
    observe_pose.pose.orientation.y = quaternion.y();
    observe_pose.pose.orientation.z = quaternion.z();
    observe_pose.pose.orientation.w = quaternion.w();

    arm_move_group_->clearPoseTargets();
    arm_move_group_->setStartStateToCurrentState();
    arm_move_group_->setPoseTarget(observe_pose, parameters_.hand_frame);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(arm_move_group_->plan(plan))) {
      arm_move_group_->clearPoseTargets();
      error_message = "Failed to plan move to observe pose.";
      return false;
    }

    const bool execute_success = execute_arm_plan(
      plan,
      "move to observe pose",
      error_message);
    arm_move_group_->clearPoseTargets();
    if (!execute_success) {
      if (error_message.empty()) {
        error_message = "Failed to execute move to observe pose.";
      }
      return false;
    }

    return true;
  }

  void execute_observe_goal(const std::shared_ptr<GoalHandleExecuteTask> & goal_handle)
  {
    std::string error_message;
    if (!execute_observe_pose_move(goal_handle, "Moving arm to observe pose", error_message)) {
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_EXECUTION_FAILED,
        error_message);
      clear_current_task(nullptr);
      return;
    }

    finish_result(
      goal_handle,
      true,
      ExecuteTask::Goal::STAGE_DONE,
      ExecuteTask::Result::ERROR_NONE,
      "Observe pose reached.");
    clear_current_task(nullptr);
  }

  XYZ add_xyz(const XYZ & lhs, const XYZ & rhs) const
  {
    return XYZ{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
  }

  geometry_msgs::msg::PoseStamped make_pose_stamped(const XYZ & position, const RPY & orientation) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = parameters_.planning_frame;
    pose.pose.position.x = position.x;
    pose.pose.position.y = position.y;
    pose.pose.position.z = position.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(orientation.roll, orientation.pitch, orientation.yaw);
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    return pose;
  }

  XYZ clamp_xyz(const XYZ & value, const XYZ & min_value, const XYZ & max_value) const
  {
    return XYZ{
      std::clamp(value.x, min_value.x, max_value.x),
      std::clamp(value.y, min_value.y, max_value.y),
      std::clamp(value.z, min_value.z, max_value.z)};
  }

  bool contains_near_xyz(const std::vector<XYZ> & values, const XYZ & candidate) const
  {
    constexpr double kEpsilon = 1e-6;
    return std::any_of(values.begin(), values.end(), [&](const XYZ & value) {
      return std::abs(value.x - candidate.x) < kEpsilon &&
             std::abs(value.y - candidate.y) < kEpsilon &&
             std::abs(value.z - candidate.z) < kEpsilon;
    });
  }

  std::vector<XYZ> xyz_candidates_from_flat(
    const std::vector<double> & configured,
    const XYZ & fallback) const
  {
    std::vector<XYZ> candidates;
    candidates.push_back(fallback);
    if (configured.size() % 3 != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring malformed XYZ candidate list with %zu values; expected a multiple of 3.",
        configured.size());
      return candidates;
    }

    for (std::size_t i = 0; i < configured.size(); i += 3) {
      const XYZ candidate{configured[i], configured[i + 1], configured[i + 2]};
      if (!contains_near_xyz(candidates, candidate)) {
        candidates.push_back(candidate);
      }
    }
    return candidates;
  }

  std::vector<RPY> rpy_candidates_from_flat(
    const std::vector<double> & configured,
    const RPY & fallback) const
  {
    std::vector<RPY> candidates;
    candidates.push_back(fallback);
    if (configured.size() % 3 != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring malformed RPY candidate list with %zu values; expected a multiple of 3.",
        configured.size());
      return candidates;
    }

    constexpr double kEpsilon = 1e-6;
    for (std::size_t i = 0; i < configured.size(); i += 3) {
      const RPY candidate{
        configured[i],
        configured[i + 1],
        configured[i + 2]};
      const auto existing = std::find_if(
        candidates.begin(),
        candidates.end(),
        [&](const RPY & value) {
          return std::abs(value.roll - candidate.roll) < kEpsilon &&
                 std::abs(value.pitch - candidate.pitch) < kEpsilon &&
                 std::abs(normalize_angle(value.yaw - candidate.yaw)) < kEpsilon;
        });
      if (existing == candidates.end()) {
        candidates.push_back(candidate);
      }
    }
    return candidates;
  }

  std::vector<double> scalar_candidates(
    const std::vector<double> & configured,
    double fallback) const
  {
    std::vector<double> candidates;
    candidates.push_back(fallback);
    constexpr double kEpsilon = 1e-6;
    for (const double value : configured) {
      const auto existing = std::find_if(
        candidates.begin(),
        candidates.end(),
        [value](double candidate) {
          return std::abs(candidate - value) < kEpsilon;
        });
      if (existing == candidates.end()) {
        candidates.push_back(value);
      }
    }
    return candidates;
  }

  bool has_visual_style_ik(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & label)
  {
    arm_move_group_->clearPoseTargets();
    arm_move_group_->clearPathConstraints();
    arm_move_group_->setStartStateToCurrentState();
    const bool solved = arm_move_group_->setJointValueTarget(pose, parameters_.hand_frame);
    arm_move_group_->clearPoseTargets();
    if (!solved) {
      RCLCPP_INFO(
        get_logger(),
        "Skipping visual-style pick %s: no IK for pose in %s position=(%.4f, %.4f, %.4f) "
        "orientation=(%.4f, %.4f, %.4f, %.4f)",
        label.c_str(),
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
    }
    return solved;
  }

  void rebuild_factory_from_parameters()
  {
    auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
    factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
  }

  bool solve_arm_pose_ik_state(
    const geometry_msgs::msg::PoseStamped & pose,
    moveit::core::RobotStatePtr & ik_state,
    std::string & error_message)
  {
    auto current_state = arm_move_group_->getCurrentState(5.0);
    if (!current_state) {
      error_message = "Failed to query current robot state for IK preview.";
      return false;
    }

    const auto * joint_model_group =
      current_state->getJointModelGroup(parameters_.arm_group_name);
    if (!joint_model_group) {
      error_message =
        "Failed to find joint model group '" + parameters_.arm_group_name + "' for IK preview.";
      return false;
    }

    auto candidate_state = std::make_shared<moveit::core::RobotState>(*current_state);
    if (!candidate_state->setFromIK(
        joint_model_group,
        pose.pose,
        parameters_.hand_frame,
        0.1))
    {
      error_message = "Failed to solve IK state preview for pose goal.";
      return false;
    }

    candidate_state->update();
    ik_state = candidate_state;
    return true;
  }

  bool plan_arm_pose_goal(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & stage_name,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    std::string & error_message,
    const moveit::core::RobotState * start_state = nullptr)
  {
    arm_move_group_->clearPoseTargets();
    if (start_state != nullptr) {
      arm_move_group_->setStartState(*start_state);
    } else {
      arm_move_group_->setStartStateToCurrentState();
    }
    arm_move_group_->setPoseTarget(pose, parameters_.hand_frame);

    const bool plan_success = static_cast<bool>(arm_move_group_->plan(plan));
    arm_move_group_->clearPoseTargets();
    if (!plan_success) {
      error_message = "Failed to plan pose goal for stage '" + stage_name + "'.";
      return false;
    }
    return true;
  }

  bool execute_arm_pose_goal(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & stage_name,
    uint8_t feedback_stage,
    std::string & error_message)
  {
    if (goal_handle->is_canceling()) {
      error_message = "Task canceled";
      return false;
    }

    publish_feedback(goal_handle, feedback_stage, stage_name);
    RCLCPP_INFO(
      get_logger(),
      "%s pose in %s: position=(%.4f, %.4f, %.4f) orientation=(%.4f, %.4f, %.4f, %.4f)",
      stage_name.c_str(),
      pose.header.frame_id.c_str(),
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z,
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!plan_arm_pose_goal(pose, stage_name, plan, error_message)) {
      return false;
    }

    const bool execute_success = execute_arm_plan(plan, stage_name, error_message);
    if (!execute_success) {
      return false;
    }
    return true;
  }

  TaskParameters build_visual_style_pick_parameters_for_center(
    const TaskParameters & seed_parameters,
    const XYZ & object_center) const
  {
    TaskParameters local_parameters = seed_parameters;
    local_parameters.pickup_object.center = object_center;

    const auto grasp_strategy =
      normalize_class_name(local_parameters.vision_target.grasp_strategy);
    if (local_parameters.vision_target.compute_grasp_offsets && grasp_strategy == "radial_side") {
      const double center_norm_xy = std::hypot(
        local_parameters.pickup_object.center.x,
        local_parameters.pickup_object.center.y);
      XYZ approach_direction{1.0, 0.0, 0.0};
      if (center_norm_xy > 1e-6) {
        approach_direction.x = local_parameters.pickup_object.center.x / center_norm_xy;
        approach_direction.y = local_parameters.pickup_object.center.y / center_norm_xy;
        approach_direction.z = 0.0;
      }
      if (local_parameters.vision_target.lock_lateral_offsets_to_zero) {
        approach_direction.x = approach_direction.x >= 0.0 ? 1.0 : -1.0;
        approach_direction.y = 0.0;
        approach_direction.z = 0.0;
      }
      local_parameters.approach_direction = approach_direction;
      local_parameters.grasp_orientation.yaw =
        normalize_angle(std::atan2(approach_direction.y, approach_direction.x));

      const double clearance = std::max(0.0, local_parameters.vision_target.grasp_clearance);
      const double pregrasp_distance =
        std::max(0.0, local_parameters.vision_target.pregrasp_distance);
      const double grasp_radius = local_parameters.pickup_object.radius + clearance;
      const double grasp_z =
        0.5 * local_parameters.pickup_object.height + local_parameters.vision_target.grasp_z_offset;

      local_parameters.grasp_target_offset = {
        -approach_direction.x * grasp_radius,
        -approach_direction.y * grasp_radius,
        grasp_z};
      local_parameters.pregrasp_offset = {
        local_parameters.grasp_target_offset.x - approach_direction.x * pregrasp_distance,
        local_parameters.grasp_target_offset.y - approach_direction.y * pregrasp_distance,
        grasp_z};

      local_parameters.top_down_grasp_target_x_candidates =
        make_single_candidate(local_parameters.grasp_target_offset.x);
      local_parameters.top_down_grasp_target_y_candidates =
        make_single_candidate(local_parameters.grasp_target_offset.y);
      local_parameters.top_down_grasp_target_z_candidates =
        make_single_candidate(local_parameters.grasp_target_offset.z);
      local_parameters.top_down_pregrasp_x_candidates =
        make_single_candidate(local_parameters.pregrasp_offset.x);
      local_parameters.top_down_pregrasp_y_candidates =
        make_single_candidate(local_parameters.pregrasp_offset.y);
      local_parameters.top_down_pregrasp_z_candidates =
        make_single_candidate(local_parameters.pregrasp_offset.z);
      local_parameters.top_down_approach_direction_candidates = {
        approach_direction.x,
        approach_direction.y,
        approach_direction.z};
      local_parameters.top_down_roll_candidates =
        make_single_candidate(local_parameters.grasp_orientation.roll);
      local_parameters.top_down_pitch_candidates =
        make_single_candidate(local_parameters.grasp_orientation.pitch);
      local_parameters.top_down_yaw_candidates.clear();
      append_unique_angle_candidate(
        local_parameters.top_down_yaw_candidates,
        local_parameters.grasp_orientation.yaw);
      for (const double offset : local_parameters.vision_target.yaw_candidate_offsets) {
        append_unique_angle_candidate(
          local_parameters.top_down_yaw_candidates,
          local_parameters.grasp_orientation.yaw + offset);
      }

      if (!has_cartesian_distance(
          local_parameters.approach_min_distance,
          local_parameters.approach_max_distance) &&
        pregrasp_distance > 1e-6)
      {
        local_parameters.approach_min_distance = 0.75 * pregrasp_distance;
        local_parameters.approach_max_distance = pregrasp_distance;
      }
    }

    return local_parameters;
  }

  TaskParameters apply_visual_grasp_forward_probe(
    const TaskParameters & seed_parameters,
    double forward_probe_distance) const
  {
    TaskParameters local_parameters = seed_parameters;
    const double bounded_forward_probe = std::max(0.0, forward_probe_distance);
    if (bounded_forward_probe <= 1e-6) {
      return local_parameters;
    }

    const XYZ approach_direction = local_parameters.approach_direction;
    const double approach_direction_norm = std::sqrt(
      approach_direction.x * approach_direction.x +
      approach_direction.y * approach_direction.y +
      approach_direction.z * approach_direction.z);
    XYZ unit_direction{1.0, 0.0, 0.0};
    if (approach_direction_norm > 1e-6) {
      unit_direction = {
        approach_direction.x / approach_direction_norm,
        approach_direction.y / approach_direction_norm,
        approach_direction.z / approach_direction_norm};
    }

    local_parameters.grasp_target_offset = {
      local_parameters.grasp_target_offset.x + unit_direction.x * bounded_forward_probe,
      local_parameters.grasp_target_offset.y + unit_direction.y * bounded_forward_probe,
      local_parameters.grasp_target_offset.z + unit_direction.z * bounded_forward_probe};
    local_parameters.pregrasp_offset = {
      local_parameters.pregrasp_offset.x + unit_direction.x * bounded_forward_probe,
      local_parameters.pregrasp_offset.y + unit_direction.y * bounded_forward_probe,
      local_parameters.pregrasp_offset.z + unit_direction.z * bounded_forward_probe};

    local_parameters.top_down_grasp_target_x_candidates =
      make_single_candidate(local_parameters.grasp_target_offset.x);
    local_parameters.top_down_grasp_target_y_candidates =
      make_single_candidate(local_parameters.grasp_target_offset.y);
    local_parameters.top_down_grasp_target_z_candidates =
      make_single_candidate(local_parameters.grasp_target_offset.z);
    local_parameters.top_down_pregrasp_x_candidates =
      make_single_candidate(local_parameters.pregrasp_offset.x);
    local_parameters.top_down_pregrasp_y_candidates =
      make_single_candidate(local_parameters.pregrasp_offset.y);
    local_parameters.top_down_pregrasp_z_candidates =
      make_single_candidate(local_parameters.pregrasp_offset.z);
    return local_parameters;
  }

  std::vector<TaskParameters> build_visual_style_pick_candidates(
    const TaskParameters & base_parameters) const
  {
    std::vector<TaskParameters> candidates;
    const int64_t max_candidates = std::max<int64_t>(
      1,
      base_parameters.vision_target.max_visual_pick_candidates);
    std::vector<std::pair<double, TaskParameters>> ranked_candidates;
    const auto add_candidate = [&](const TaskParameters & candidate_parameters) {
      if (static_cast<int64_t>(candidates.size()) >= max_candidates) {
        return;
      }
      TaskParameters candidate = candidate_parameters;
      candidate.grasp_orientation.yaw = normalize_angle(candidate.grasp_orientation.yaw);
      const auto existing = std::find_if(
        candidates.begin(),
        candidates.end(),
        [&candidate](const TaskParameters & existing_candidate) {
          return std::abs(
            normalize_angle(
              existing_candidate.grasp_orientation.yaw - candidate.grasp_orientation.yaw)) < 1e-6 &&
                 std::abs(existing_candidate.grasp_orientation.roll -
                 candidate.grasp_orientation.roll) < 1e-6 &&
                 std::abs(existing_candidate.grasp_orientation.pitch -
                 candidate.grasp_orientation.pitch) < 1e-6 &&
                 std::abs(existing_candidate.pickup_object.center.x -
                 candidate.pickup_object.center.x) < 1e-6 &&
                 std::abs(existing_candidate.pickup_object.center.y -
                 candidate.pickup_object.center.y) < 1e-6 &&
                 std::abs(existing_candidate.pickup_object.center.z -
                 candidate.pickup_object.center.z) < 1e-6 &&
                 std::abs(existing_candidate.grasp_target_offset.x -
                 candidate.grasp_target_offset.x) < 1e-6 &&
                 std::abs(existing_candidate.grasp_target_offset.y -
                 candidate.grasp_target_offset.y) < 1e-6 &&
                 std::abs(existing_candidate.grasp_target_offset.z -
                 candidate.grasp_target_offset.z) < 1e-6 &&
                 std::abs(existing_candidate.pregrasp_offset.x -
                 candidate.pregrasp_offset.x) < 1e-6 &&
                 std::abs(existing_candidate.pregrasp_offset.y -
                 candidate.pregrasp_offset.y) < 1e-6 &&
                 std::abs(existing_candidate.pregrasp_offset.z -
                 candidate.pregrasp_offset.z) < 1e-6;
        });
      if (existing != candidates.end()) {
        return;
      }
      candidates.push_back(candidate);
    };
    const auto queue_candidate =
      [&](double priority, const TaskParameters & candidate_parameters) {
        ranked_candidates.emplace_back(priority, candidate_parameters);
      };
    const auto center_offset_priority = [](const XYZ & center_offset) {
        const double lateral = std::hypot(center_offset.x, center_offset.y);
        const double upward =
          center_offset.z > 0.0 ? center_offset.z * 0.5 : 0.0;
        const double downward =
          center_offset.z < 0.0 ? -center_offset.z * 2.0 : 0.0;
        return lateral * 2.5 + upward + downward;
      };

    std::vector<double> yaws;
    append_unique_angle_candidate(yaws, base_parameters.grasp_orientation.yaw);
    for (const double yaw : base_parameters.top_down_yaw_candidates) {
      append_unique_angle_candidate(yaws, yaw);
    }

    const auto grasp_strategy =
      normalize_class_name(base_parameters.vision_target.grasp_strategy);
    const bool robust_enabled =
      base_parameters.vision_target.robust_candidate_enabled &&
      base_parameters.vision_target.compute_grasp_offsets &&
      grasp_strategy == "radial_side";

    std::vector<XYZ> center_offsets{{0.0, 0.0, 0.0}};
    std::vector<RPY> orientation_offsets{{0.0, 0.0, 0.0}};
    std::vector<double> grasp_forward_probe_distances{0.0};
    std::vector<double> pregrasp_distances;
    double base_pregrasp_distance = 0.0;
    if (base_parameters.vision_target.compute_grasp_offsets && grasp_strategy == "radial_side") {
      base_pregrasp_distance = std::max(
        0.0,
        -(
          (base_parameters.pregrasp_offset.x - base_parameters.grasp_target_offset.x) *
          base_parameters.approach_direction.x +
          (base_parameters.pregrasp_offset.y - base_parameters.grasp_target_offset.y) *
          base_parameters.approach_direction.y +
          (base_parameters.pregrasp_offset.z - base_parameters.grasp_target_offset.z) *
          base_parameters.approach_direction.z));
      pregrasp_distances = robust_enabled ?
        scalar_candidates(
        base_parameters.vision_target.pregrasp_distance_candidates,
        base_pregrasp_distance) :
        std::vector<double>{base_pregrasp_distance};
      grasp_forward_probe_distances = robust_enabled ?
        scalar_candidates(
        base_parameters.vision_target.grasp_forward_probe_distance_candidates,
        0.0) :
        std::vector<double>{0.0};
    } else {
      pregrasp_distances = {0.0};
      grasp_forward_probe_distances = {0.0};
    }

    if (robust_enabled) {
      center_offsets = xyz_candidates_from_flat(
        base_parameters.vision_target.target_position_candidate_offsets,
        XYZ{0.0, 0.0, 0.0});
      orientation_offsets = rpy_candidates_from_flat(
        base_parameters.vision_target.orientation_candidate_offsets,
        RPY{0.0, 0.0, 0.0});
    }

    for (const auto & center_offset : center_offsets) {
      if (static_cast<int64_t>(candidates.size()) >= max_candidates) {
        break;
      }
      const XYZ requested_candidate_center =
        add_xyz(base_parameters.pickup_object.center, center_offset);
      const XYZ candidate_center = clamp_xyz(
        requested_candidate_center,
        base_parameters.vision_target.workspace_min,
        base_parameters.vision_target.workspace_max);
      constexpr double kClampEpsilon = 1e-6;
      if (std::abs(requested_candidate_center.x - candidate_center.x) > kClampEpsilon ||
        std::abs(requested_candidate_center.y - candidate_center.y) > kClampEpsilon ||
        std::abs(requested_candidate_center.z - candidate_center.z) > kClampEpsilon)
      {
        RCLCPP_WARN(
          get_logger(),
          "Clamped visual pick candidate center from (%.4f, %.4f, %.4f) to "
          "(%.4f, %.4f, %.4f) within workspace min=(%.4f, %.4f, %.4f) "
          "max=(%.4f, %.4f, %.4f).",
          requested_candidate_center.x,
          requested_candidate_center.y,
          requested_candidate_center.z,
          candidate_center.x,
          candidate_center.y,
          candidate_center.z,
          base_parameters.vision_target.workspace_min.x,
          base_parameters.vision_target.workspace_min.y,
          base_parameters.vision_target.workspace_min.z,
          base_parameters.vision_target.workspace_max.x,
          base_parameters.vision_target.workspace_max.y,
          base_parameters.vision_target.workspace_max.z);
      }
      TaskParameters center_parameters =
        build_visual_style_pick_parameters_for_center(base_parameters, candidate_center);

      std::vector<double> candidate_yaws;
      append_unique_angle_candidate(candidate_yaws, center_parameters.grasp_orientation.yaw);
      for (const double yaw : center_parameters.top_down_yaw_candidates) {
        append_unique_angle_candidate(candidate_yaws, yaw);
      }
      for (const double yaw : yaws) {
        append_unique_angle_candidate(candidate_yaws, yaw);
      }

      const XYZ & direction = center_parameters.approach_direction;
      const double direction_norm =
        std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
      XYZ unit_direction{1.0, 0.0, 0.0};
      if (direction_norm > 1e-6) {
        unit_direction = {
          direction.x / direction_norm,
          direction.y / direction_norm,
          direction.z / direction_norm};
      }

      for (const auto & orientation_offset : orientation_offsets) {
        if (static_cast<int64_t>(candidates.size()) >= max_candidates) {
          break;
        }
        for (const double yaw : candidate_yaws) {
          if (static_cast<int64_t>(candidates.size()) >= max_candidates) {
            break;
          }
          for (const double pregrasp_distance : pregrasp_distances) {
            for (const double forward_probe_distance : grasp_forward_probe_distances) {
              TaskParameters candidate = center_parameters;
              candidate.grasp_orientation.roll += orientation_offset.roll;
              candidate.grasp_orientation.pitch += orientation_offset.pitch;
              candidate.grasp_orientation.yaw = normalize_angle(yaw + orientation_offset.yaw);
              const double distance = std::max(0.0, pregrasp_distance);
              const double forward_probe = forward_probe_distance;
              candidate.grasp_target_offset = {
                center_parameters.grasp_target_offset.x + unit_direction.x * forward_probe,
                center_parameters.grasp_target_offset.y + unit_direction.y * forward_probe,
                center_parameters.grasp_target_offset.z + unit_direction.z * forward_probe};
              candidate.pregrasp_offset = {
                candidate.grasp_target_offset.x - unit_direction.x * distance,
                candidate.grasp_target_offset.y - unit_direction.y * distance,
                candidate.grasp_target_offset.z - unit_direction.z * distance};
              const double yaw_priority =
                std::abs(normalize_angle(yaw - center_parameters.grasp_orientation.yaw));
              const double orientation_priority =
                std::abs(orientation_offset.roll) +
                std::abs(orientation_offset.pitch) +
                std::abs(orientation_offset.yaw);
              const double distance_priority =
                std::abs(distance - base_pregrasp_distance);
              const double forward_probe_priority = std::abs(forward_probe);
              const double priority =
                center_offset_priority(center_offset) +
                orientation_priority +
                yaw_priority +
                distance_priority * 0.6 +
                forward_probe_priority * 0.8;
              queue_candidate(priority, candidate);
            }
          }
        }
      }
    }

    std::stable_sort(
      ranked_candidates.begin(),
      ranked_candidates.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.first < rhs.first;
      });
    for (const auto & ranked_candidate : ranked_candidates) {
      add_candidate(ranked_candidate.second);
      if (static_cast<int64_t>(candidates.size()) >= max_candidates) {
        break;
      }
    }

    if (candidates.empty()) {
      candidates.push_back(base_parameters);
    }
    RCLCPP_INFO(
      get_logger(),
      "Built %zu visual-style pick candidates from %zu center offsets, %zu orientation offsets, "
      "%zu yaw candidates, %zu forward probes, and %zu pregrasp distances (limit=%ld, robust=%s).",
      candidates.size(),
      center_offsets.size(),
      orientation_offsets.size(),
      yaws.size(),
      grasp_forward_probe_distances.size(),
      pregrasp_distances.size(),
      max_candidates,
      robust_enabled ? "true" : "false");
    return candidates;
  }

  geometry_msgs::msg::PoseStamped make_visual_style_pick_pose(
    const XYZ & object_center,
    const TaskParameters & candidate_parameters,
    const XYZ & target_offset) const
  {
    TaskParameters local_parameters = candidate_parameters;
    local_parameters.grasp_target_offset = target_offset;

    Eigen::Isometry3d object_transform = Eigen::Isometry3d::Identity();
    object_transform.translation().x() = object_center.x;
    object_transform.translation().y() = object_center.y;
    object_transform.translation().z() = object_center.z;

    const Eigen::Isometry3d grasp_frame_in_hand =
      make_grasp_frame_transform(local_parameters);
    const Eigen::Isometry3d hand_transform =
      object_transform * grasp_frame_in_hand.inverse();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = candidate_parameters.planning_frame;
    pose.pose.position.x = hand_transform.translation().x();
    pose.pose.position.y = hand_transform.translation().y();
    pose.pose.position.z = hand_transform.translation().z();
    const Eigen::Quaterniond hand_orientation(hand_transform.rotation());
    pose.pose.orientation.x = hand_orientation.x();
    pose.pose.orientation.y = hand_orientation.y();
    pose.pose.orientation.z = hand_orientation.z();
    pose.pose.orientation.w = hand_orientation.w();
    return pose;
  }

  bool execute_explicit_pose_offset(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const XYZ & offset,
    const std::string & stage_name,
    uint8_t feedback_stage,
    std::string & error_message)
  {
    if (goal_handle->is_canceling()) {
      error_message = "Task canceled";
      return false;
    }

    const double offset_distance = std::sqrt(
      offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
    if (offset_distance <= 1e-6) {
      return true;
    }

    publish_feedback(goal_handle, feedback_stage, stage_name);

    auto current_state = arm_move_group_->getCurrentState(5.0);
    if (!current_state) {
      error_message = "Failed to query current arm state before Cartesian offset move.";
      return false;
    }

    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    target_pose.position.x += offset.x;
    target_pose.position.y += offset.y;
    target_pose.position.z += offset.z;

    moveit_msgs::msg::RobotTrajectory trajectory_message;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double achieved_fraction = arm_move_group_->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target_pose},
      std::max(parameters_.cartesian_step_size, 0.001),
      parameters_.cartesian_jump_threshold,
      trajectory_message,
      false,
      &error_code);
    if (achieved_fraction < 0.99) {
      error_message =
        "Cartesian offset move only achieved fraction " + std::to_string(achieved_fraction) + ".";
      return false;
    }

    robot_trajectory::RobotTrajectory trajectory(
      arm_move_group_->getRobotModel(),
      parameters_.arm_group_name);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory,
        parameters_.cartesian_velocity_scaling,
        parameters_.cartesian_acceleration_scaling))
    {
      error_message = "Failed to time-parameterize Cartesian offset trajectory.";
      return false;
    }

    trajectory.getRobotTrajectoryMsg(trajectory_message);
    if (!execute_arm_robot_trajectory(
        trajectory_message,
        "Cartesian offset move",
        error_message))
    {
      return false;
    }
    return true;
  }

  bool execute_cartesian_pose_goal(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & stage_name,
    uint8_t feedback_stage,
    std::string & error_message)
  {
    if (goal_handle->is_canceling()) {
      error_message = "Task canceled";
      return false;
    }

    publish_feedback(goal_handle, feedback_stage, stage_name);

    auto current_state = arm_move_group_->getCurrentState(5.0);
    if (!current_state) {
      error_message = "Failed to query current arm state before Cartesian pose move.";
      return false;
    }

    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    const double dx = pose.pose.position.x - current_pose.pose.position.x;
    const double dy = pose.pose.position.y - current_pose.pose.position.y;
    const double dz = pose.pose.position.z - current_pose.pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance <= 1e-6) {
      return true;
    }

    moveit_msgs::msg::RobotTrajectory trajectory_message;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double achieved_fraction = arm_move_group_->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{pose.pose},
      std::max(parameters_.cartesian_step_size, 0.001),
      parameters_.cartesian_jump_threshold,
      trajectory_message,
      false,
      &error_code);
    if (achieved_fraction < 0.99) {
      error_message =
        "Cartesian pose move for stage '" + stage_name + "' only achieved fraction " +
        std::to_string(achieved_fraction) + ".";
      return false;
    }

    robot_trajectory::RobotTrajectory trajectory(
      arm_move_group_->getRobotModel(),
      parameters_.arm_group_name);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        trajectory,
        parameters_.cartesian_velocity_scaling,
        parameters_.cartesian_acceleration_scaling))
    {
      error_message = "Failed to time-parameterize Cartesian pose trajectory.";
      return false;
    }

    trajectory.getRobotTrajectoryMsg(trajectory_message);
    return execute_arm_robot_trajectory(trajectory_message, stage_name, error_message);
  }

  VisualStylePickOutcome execute_visual_style_pick_to_lift(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const std::vector<TaskParameters> & pick_candidates,
    const std::string & pick_label,
    bool allow_target_refresh_after_empty_grasp,
    std::string & error_message)
  {
    stop_gripper_hold();
    detach_pick_object_from_gazebo_link_attacher();
    detach_pick_object_from_moveit();

    if (!parameters_.skip_open_gripper_stage) {
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
        "Opening gripper for " + pick_label);
      if (parameters_.gripper_open_joint7 >= -0.5) {
        publish_gripper_target(parameters_.gripper_open_joint7, 0.40);
      } else if (!execute_named_gripper_target(
          parameters_.gripper_open_named_target,
          "opening for " + pick_label,
          error_message))
      {
        return VisualStylePickOutcome::kFailed;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }

    for (std::size_t candidate_index = 0; candidate_index < pick_candidates.size(); ++candidate_index) {
      const auto & candidate = pick_candidates[candidate_index];
      const XYZ & object_center = candidate.pickup_object.center;
      const XYZ approach_direction = candidate.approach_direction;
      const bool uses_radial_side_pregrasp =
        normalize_class_name(candidate.vision_target.grasp_strategy) == "radial_side";
      const double approach_direction_norm = std::sqrt(
        approach_direction.x * approach_direction.x +
        approach_direction.y * approach_direction.y +
        approach_direction.z * approach_direction.z);
      XYZ unit_direction{1.0, 0.0, 0.0};
      if (approach_direction_norm > 1e-6) {
        unit_direction = {
          approach_direction.x / approach_direction_norm,
          approach_direction.y / approach_direction_norm,
          approach_direction.z / approach_direction_norm};
      }
      double pregrasp_distance = 0.0;
      if (uses_radial_side_pregrasp) {
        if (!std::isfinite(approach_direction.x) ||
          !std::isfinite(approach_direction.y) ||
          !std::isfinite(approach_direction.z) ||
          !std::isfinite(approach_direction_norm) || approach_direction_norm <= 1e-6)
        {
          RCLCPP_WARN(
            get_logger(),
            "Skipping visual-style pick %s candidate %zu: invalid approach direction=(%.6f, %.6f, %.6f)",
            pick_label.c_str(), candidate_index + 1,
            approach_direction.x, approach_direction.y, approach_direction.z);
          continue;
        }
        pregrasp_distance =
          (candidate.grasp_target_offset.x - candidate.pregrasp_offset.x) * unit_direction.x +
          (candidate.grasp_target_offset.y - candidate.pregrasp_offset.y) * unit_direction.y +
          (candidate.grasp_target_offset.z - candidate.pregrasp_offset.z) * unit_direction.z;
      }
      std::ostringstream candidate_label;
      candidate_label << pick_label << " candidate " << (candidate_index + 1) << "/" <<
        pick_candidates.size() << " rpy=(" << std::fixed << std::setprecision(4) <<
        candidate.grasp_orientation.roll << ", " <<
        candidate.grasp_orientation.pitch << ", " <<
        candidate.grasp_orientation.yaw << ") target=(" <<
        object_center.x << ", " << object_center.y << ", " << object_center.z << ") "
        "grasp_offset=(" << candidate.grasp_target_offset.x << ", " <<
        candidate.grasp_target_offset.y << ", " << candidate.grasp_target_offset.z << ") "
        "pregrasp_offset=(" << candidate.pregrasp_offset.x << ", " <<
        candidate.pregrasp_offset.y << ", " << candidate.pregrasp_offset.z << ") "
        "pregrasp_distance=" << pregrasp_distance;
      RCLCPP_INFO(
        get_logger(),
        "Trying visual-style pick %s",
        candidate_label.str().c_str());

      const auto grasp_pose =
        make_visual_style_pick_pose(object_center, candidate, candidate.grasp_target_offset);
      RCLCPP_INFO(
        get_logger(),
        "Commanded grasp pose for %s: world_position=(%.4f, %.4f, %.4f) "
        "world_orientation=(%.4f, %.4f, %.4f, %.4f)",
        candidate_label.str().c_str(),
        grasp_pose.pose.position.x,
        grasp_pose.pose.position.y,
        grasp_pose.pose.position.z,
        grasp_pose.pose.orientation.x,
        grasp_pose.pose.orientation.y,
        grasp_pose.pose.orientation.z,
        grasp_pose.pose.orientation.w);
      if (!has_visual_style_ik(grasp_pose, candidate_label.str() + " grasp")) {
        continue;
      }

      bool reached_pregrasp = false;
      if (candidate.approach_max_distance > 1e-6 || candidate.approach_min_distance > 1e-6) {
        std::optional<geometry_msgs::msg::PoseStamped> pregrasp_pose;
        if (uses_radial_side_pregrasp) {
          pregrasp_pose = make_planning_frame_pregrasp_pose(
            grasp_pose, approach_direction, pregrasp_distance);
        } else {
          pregrasp_pose = make_visual_style_pick_pose(
            object_center, candidate, candidate.pregrasp_offset);
        }
        if (!pregrasp_pose) {
          RCLCPP_WARN(
            get_logger(),
            "Skipping visual-style pick %s: invalid approach direction for pregrasp.",
            candidate_label.str().c_str());
          continue;
        }
        if (!has_visual_style_ik(*pregrasp_pose, candidate_label.str() + " pregrasp")) {
          RCLCPP_INFO(
            get_logger(),
            "Pregrasp IK failed for visual-style pick %s, trying direct grasp.",
            candidate_label.str().c_str());
        } else
        if (!execute_arm_pose_goal(
            goal_handle,
            *pregrasp_pose,
            "Moving to pregrasp " + candidate_label.str(),
            ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
            error_message))
        {
          RCLCPP_WARN(
            get_logger(),
            "Pregrasp planning failed for visual-style pick %s, trying direct grasp.",
            candidate_label.str().c_str());
        } else {
          reached_pregrasp = true;
        }
      }

      const std::string grasp_stage_name = "Moving to grasp " + candidate_label.str();
      const bool reached_grasp =
        reached_pregrasp ?
        execute_cartesian_pose_goal(
          goal_handle,
          grasp_pose,
          grasp_stage_name,
          ExecuteTask::Goal::STAGE_MOVING_GRASP,
          error_message) :
        execute_arm_pose_goal(
          goal_handle,
          grasp_pose,
          grasp_stage_name,
          ExecuteTask::Goal::STAGE_MOVING_GRASP,
          error_message);
      if (reached_grasp)
      {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_CLOSING_GRIPPER,
          "Closing gripper on " + candidate_label.str());
        const auto close_outcome = execute_visual_style_gripper_close(
          goal_handle,
          "Closing gripper on " + candidate_label.str(),
          "closing for " + candidate_label.str(),
          error_message);
        if (close_outcome == GripperCloseOutcome::kFailed) {
          return VisualStylePickOutcome::kFailed;
        }
        if (close_outcome == GripperCloseOutcome::kFullyClosed) {
          RCLCPP_WARN(
            get_logger(),
            "Visual-style pick %s likely missed the object; retreating and trying next candidate.",
            candidate_label.str().c_str());
          const double retreat_backoff = std::max(
            0.0,
            parameters_.vision_target.empty_grasp_retreat_backoff);
          const double retreat_up = std::max(
            0.0,
            parameters_.vision_target.empty_grasp_retreat_up);
          const XYZ retreat_offset{
            -unit_direction.x * retreat_backoff,
            -unit_direction.y * retreat_backoff,
            retreat_up - unit_direction.z * retreat_backoff};
          std::string retreat_error_message;
          if (!execute_explicit_pose_offset(
              goal_handle,
              retreat_offset,
              "Retreating after empty visual grasp",
              ExecuteTask::Goal::STAGE_LIFTING,
              retreat_error_message))
          {
            RCLCPP_WARN(
              get_logger(),
              "Best-effort retreat after empty visual grasp failed for %s: %s",
              candidate_label.str().c_str(),
              retreat_error_message.c_str());
          }
          stop_gripper_hold();
          detach_pick_object_from_gazebo_link_attacher();
          detach_pick_object_from_moveit();
          if (parameters_.gripper_open_joint7 >= -0.5) {
            publish_gripper_target(parameters_.gripper_open_joint7, 0.30);
          } else {
            std::string open_error_message;
            execute_named_gripper_target(
              parameters_.gripper_open_named_target,
              "opening after empty visual grasp",
              open_error_message);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(300));
          if (allow_target_refresh_after_empty_grasp &&
            parameters_.vision_target.refresh_target_after_empty_grasp)
          {
            const XYZ previous_target_center = parameters_.pickup_object.center;
            std::optional<GraspTarget> previous_visual_target;
            {
              std::lock_guard<std::mutex> lock(vision_target_mutex_);
              previous_visual_target = latest_grasp_target_;
            }
            const auto refresh_not_before = now();
            publish_feedback(
              goal_handle,
              ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
              "Refreshing visual target after empty grasp from retreat pose");
            std::string refresh_error_message;
            if (!wait_for_changed_visual_target_after(
                refresh_not_before,
                previous_visual_target,
                parameters_.vision_target.empty_grasp_target_refresh_timeout_sec,
                refresh_error_message))
            {
              error_message =
                "Empty visual grasp detected, but no fresh visual target arrived after reobserve: " +
                refresh_error_message;
              return VisualStylePickOutcome::kFailed;
            }
            if (!prepare_visual_pick_target(refresh_error_message)) {
              error_message =
                "Received a post-empty-grasp visual target, but it was not usable: " +
                refresh_error_message;
              return VisualStylePickOutcome::kFailed;
            }
            std::optional<GraspTarget> refreshed_visual_target;
            {
              std::lock_guard<std::mutex> lock(vision_target_mutex_);
              refreshed_visual_target = latest_grasp_target_;
            }
            const XYZ refreshed_target_center = parameters_.pickup_object.center;
            const auto yaw_value = (refreshed_visual_target && refreshed_visual_target->has_yaw) ?
              static_cast<double>(refreshed_visual_target->yaw) :
              std::numeric_limits<double>::quiet_NaN();
            RCLCPP_INFO(
              get_logger(),
              "Refreshed visual target after empty grasp for %s: raw_target=(%.4f, %.4f, %.4f) "
              "class=%s conf=%.3f yaw=%s%.4f size=(%.4f, %.4f, %.4f) "
              "biased_center=(%.4f, %.4f, %.4f) prev_center=(%.4f, %.4f, %.4f) "
              "delta=(%.4f, %.4f, %.4f). Rebuilding candidates.",
              candidate_label.str().c_str(),
              refreshed_visual_target ? refreshed_visual_target->pose.position.x : std::numeric_limits<double>::quiet_NaN(),
              refreshed_visual_target ? refreshed_visual_target->pose.position.y : std::numeric_limits<double>::quiet_NaN(),
              refreshed_visual_target ? refreshed_visual_target->pose.position.z : std::numeric_limits<double>::quiet_NaN(),
              refreshed_visual_target ? refreshed_visual_target->class_name.c_str() : "<none>",
              refreshed_visual_target ? refreshed_visual_target->confidence : 0.0,
              (refreshed_visual_target && refreshed_visual_target->has_yaw) ? "" : "n/a:",
              yaw_value,
              refreshed_visual_target ? refreshed_visual_target->size.x : std::numeric_limits<double>::quiet_NaN(),
              refreshed_visual_target ? refreshed_visual_target->size.y : std::numeric_limits<double>::quiet_NaN(),
              refreshed_visual_target ? refreshed_visual_target->size.z : std::numeric_limits<double>::quiet_NaN(),
              refreshed_target_center.x,
              refreshed_target_center.y,
              refreshed_target_center.z,
              previous_target_center.x,
              previous_target_center.y,
              previous_target_center.z,
              refreshed_target_center.x - previous_target_center.x,
              refreshed_target_center.y - previous_target_center.y,
              refreshed_target_center.z - previous_target_center.z);
            error_message.clear();
            return VisualStylePickOutcome::kRefreshTarget;
          }
          continue;
        }

        if (!attach_pick_object_to_moveit(error_message)) {
          return VisualStylePickOutcome::kFailed;
        }
        if (parameters_.enable_gazebo_attachment &&
          !attach_pick_object_with_gazebo_link_attacher(error_message))
        {
          return VisualStylePickOutcome::kFailed;
        }

        if (has_cartesian_distance(parameters_.lift_min_distance, parameters_.lift_max_distance)) {
          publish_feedback(
            goal_handle,
            ExecuteTask::Goal::STAGE_LIFTING,
            "Lifting " + candidate_label.str());
          const std::optional<XYZ> escape_approach_direction =
            uses_radial_side_pregrasp && candidate.vision_target.compute_grasp_offsets ?
            std::optional<XYZ>(approach_direction) : std::nullopt;
          if (!execute_explicit_lift(goal_handle, error_message, escape_approach_direction)) {
            return VisualStylePickOutcome::kFailed;
          }
        }

        return VisualStylePickOutcome::kSuccess;
      }

      RCLCPP_WARN(
        get_logger(),
        "Visual-style pick %s failed before gripper close.",
        candidate_label.str().c_str());
    }

    if (error_message.empty()) {
      error_message = "Failed to pick with all visual-style pick candidates.";
    }
    return VisualStylePickOutcome::kExhausted;
  }

  bool validate_classification_place_config(std::string & error_message) const
  {
    const auto & config = parameters_.classification_place;
    if (!config.enabled) {
      error_message = "Classification place task is not enabled in configuration.";
      return false;
    }
    if (config.platform_slots.empty()) {
      error_message = "classification_place.platform_slots_xyz is empty.";
      return false;
    }
    if (config.platform_slots.size() != config.platform_slot_classes.size() ||
      config.platform_slots.size() != config.box_classes.size())
    {
      error_message =
        "classification_place platform_slots, platform_slot_classes, and box_classes must have the same length.";
      return false;
    }
    return true;
  }

  bool set_fusion_target_class(
    const std::string & target_class,
    const std::string & target_fusion_node_name,
    bool set_enabled,
    double settle_sec,
    std::string & error_message)
  {
    if (!set_enabled) {
      return true;
    }
    const std::string node_name =
      target_fusion_node_name.empty() ? "/grasp_target_fusion" : target_fusion_node_name;
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
    const auto service_timeout = std::chrono::seconds(5);
    const auto response_timeout = std::chrono::seconds(5);
    constexpr int max_attempts = 3;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
      if (!client->wait_for_service(service_timeout)) {
        error_message =
          "Timed out waiting for parameter service on " + node_name +
          " while setting target_class to '" + target_class + "' (attempt " +
          std::to_string(attempt) + "/" + std::to_string(max_attempts) + ").";
        continue;
      }

      auto future = client->set_parameters(
        {rclcpp::Parameter("target_class", target_class)});
      if (future.wait_for(response_timeout) != std::future_status::ready) {
        error_message =
          "Timed out setting grasp_target_fusion target_class to '" + target_class +
          "' on " + node_name + " (attempt " + std::to_string(attempt) + "/" +
          std::to_string(max_attempts) + ").";
        continue;
      }

      const auto results = future.get();
      bool all_successful = true;
      for (const auto & result : results) {
        if (!result.successful) {
          error_message =
            "Failed to set grasp_target_fusion target_class to '" + target_class +
            "' on " + node_name + ": " + result.reason;
          all_successful = false;
          break;
        }
      }
      if (all_successful) {
        error_message.clear();
        break;
      }
      if (attempt == max_attempts) {
        return false;
      }
    }
    if (!error_message.empty()) {
      return false;
    }

    const double bounded_settle_sec = std::max(0.0, settle_sec);
    if (bounded_settle_sec > 1e-6) {
      std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(bounded_settle_sec)));
    }
    return true;
  }

  bool set_fusion_target_class(const std::string & target_class, std::string & error_message)
  {
    const auto & config = parameters_.classification_place;
    return set_fusion_target_class(
      target_class,
      config.target_fusion_node_name,
      config.set_fusion_target_class,
      config.target_switch_settle_sec,
      error_message);
  }

  bool set_fixed_place_index(int64_t index, std::string & error_message)
  {
    const auto result = set_parameter(rclcpp::Parameter("place_target.fixed_pose_index", index));
    if (!result.successful) {
      error_message =
        "Failed to set place_target.fixed_pose_index to " + std::to_string(index) +
        ": " + result.reason;
      return false;
    }
    return true;
  }

  bool execute_visual_pick_once_for_repeat(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::size_t pick_index,
    std::size_t total_picks,
    std::string & error_message)
  {
    const std::string progress =
      std::to_string(pick_index + 1) + "/" + std::to_string(total_picks);

    if (!parameters_.pick_only || !parameters_.use_direct_visual_pick_fallback) {
      error_message =
        "Repeat visual pick task currently requires pick_only=true and "
        "use_direct_visual_pick_fallback=true in configuration.";
      return false;
    }

    if (parameters_.move_home_before_pick) {
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_MOVING_HOME,
        "Repeat visual pick " + progress + ": moving arm home");

      if (!execute_named_arm_target(
          parameters_.arm_home_named_target,
          "home before repeat visual pick",
          error_message))
      {
        if (error_message.empty()) {
          error_message = "Failed to move arm home before repeat visual pick.";
        }
        return false;
      }

      if (parameters_.move_home_open_gripper) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
          "Repeat visual pick " + progress + ": opening gripper at home");
        if (parameters_.gripper_open_joint7 >= -0.5) {
          publish_gripper_target(parameters_.gripper_open_joint7, 0.40);
        } else if (!execute_named_gripper_target(
            parameters_.gripper_open_named_target,
            "opening gripper at home",
            error_message))
        {
          if (error_message.empty()) {
            error_message = "Failed to open gripper fully at home.";
          }
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
      }

      if (parameters_.vision_target.wait_after_home_timeout_sec > 0.0) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
          "Repeat visual pick " + progress + ": waiting for a fresh visual target");
        if (!wait_for_fresh_visual_target_after(
            now(),
            parameters_.vision_target.wait_after_home_timeout_sec,
            error_message))
        {
          return false;
        }
      }
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
      "Repeat visual pick " + progress + ": resolving latest visual grasp target");
    if (!prepare_visual_pick_target(error_message)) {
      return false;
    }

    return execute_direct_visual_pick(goal_handle, error_message);
  }

  void execute_repeat_visual_pick_goal(const std::shared_ptr<GoalHandleExecuteTask> & goal_handle)
  {
    const auto baseline_parameters = parameters_;
    const int64_t original_fixed_place_index =
      get_parameter("place_target.fixed_pose_index").as_int();
    const auto & config = baseline_parameters.repeat_visual_pick;
    std::string error_message;

    const auto restore_baseline = [&]() {
      parameters_ = baseline_parameters;
      rebuild_factory_from_parameters();
      std::string restore_error_message;
      if (!set_fixed_place_index(original_fixed_place_index, restore_error_message)) {
        RCLCPP_WARN(get_logger(), "%s", restore_error_message.c_str());
      }
    };

    const auto finish_failure = [&](uint8_t stage, int32_t error_code, const std::string & message) {
      stop_gripper_hold();
      restore_baseline();
      clear_current_task(nullptr);
      finish_result(goal_handle, false, stage, error_code, message);
    };

    if (config.place_indices.empty()) {
      finish_failure(
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_NOT_READY,
        "repeat_visual_pick.place_indices is empty.");
      return;
    }
    if (config.target_class.empty() && config.target_classes.empty()) {
      finish_failure(
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_NOT_READY,
        "repeat_visual_pick.target_class and repeat_visual_pick.target_classes are empty.");
      return;
    }
    if (!config.target_classes.empty() &&
      config.target_classes.size() != 1 &&
      config.target_classes.size() != config.place_indices.size())
    {
      finish_failure(
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_NOT_READY,
        "repeat_visual_pick.target_classes must contain one class or match place_indices length.");
      return;
    }

    try {
      for (std::size_t index = 0; index < config.place_indices.size(); ++index) {
        if (goal_handle->is_canceling()) {
          finish_failure(
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_CANCELED,
            "Repeat visual pick task canceled.");
          return;
        }

        const std::string target_class =
          config.target_classes.empty() ?
          config.target_class :
          config.target_classes[config.target_classes.size() == 1 ? 0 : index];
        if (target_class.empty()) {
          finish_failure(
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_NOT_READY,
            "repeat_visual_pick target class for index " + std::to_string(index) + " is empty.");
          return;
        }
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
          "Repeat visual pick " + std::to_string(index + 1) + "/" +
          std::to_string(config.place_indices.size()) +
          ": setting target class to " + target_class);
        if (!set_fusion_target_class(
            target_class,
            config.target_fusion_node_name,
            config.set_fusion_target_class,
            config.target_switch_settle_sec,
            error_message))
        {
          finish_failure(
            ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
            ExecuteTask::Result::ERROR_NOT_READY,
            error_message);
          return;
        }

        const int64_t place_index = config.place_indices[index];
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          "Repeat visual pick " + std::to_string(index + 1) + "/" +
          std::to_string(config.place_indices.size()) +
          ": selecting fixed place index " + std::to_string(place_index));
        if (!set_fixed_place_index(place_index, error_message)) {
          finish_failure(
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_NOT_READY,
            error_message);
          return;
        }

        if (!execute_visual_pick_once_for_repeat(
            goal_handle,
            index,
            config.place_indices.size(),
            error_message))
        {
          finish_failure(
            goal_handle->is_canceling() ?
            ExecuteTask::Goal::STAGE_FAILED :
            ExecuteTask::Goal::STAGE_FAILED,
            goal_handle->is_canceling() ?
            ExecuteTask::Result::ERROR_CANCELED :
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            error_message.empty() ? "Repeat visual pick failed." : error_message);
          return;
        }

        parameters_ = baseline_parameters;
        rebuild_factory_from_parameters();

        if (index + 1 < config.place_indices.size()) {
          const double delay_sec = std::max(0.0, config.delay_between_picks_sec);
          if (delay_sec > 1e-6) {
            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(delay_sec)));
          }
        }
      }

      stop_gripper_hold();
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_MOVING_HOME,
        "Repeat visual pick completed: moving arm home");
      if (!execute_named_arm_target(
          parameters_.arm_home_named_target,
          "home after repeat visual pick",
          error_message))
      {
        finish_failure(
          ExecuteTask::Goal::STAGE_MOVING_HOME,
          ExecuteTask::Result::ERROR_EXECUTION_FAILED,
          error_message.empty() ?
          "Failed to move arm home after repeat visual pick." :
          error_message);
        return;
      }

      restore_baseline();
      clear_current_task(nullptr);
      finish_result(
        goal_handle,
        true,
        ExecuteTask::Goal::STAGE_DONE,
        ExecuteTask::Result::ERROR_NONE,
        "Repeat visual pick to payload completed.");
    } catch (const std::exception & exception) {
      detach_pick_object_from_gazebo_link_attacher();
      detach_pick_object_from_moveit();
      finish_failure(
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_EXECUTION_FAILED,
        std::string("Repeat visual pick task failed: ") + exception.what());
    }
  }

  void execute_flame_tracking_goal(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    bool enable)
  {
    const auto & config = parameters_.flame_tracking;
    const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(std::max(0.1, config.service_timeout_sec)));
    std::string action_label = enable ? "start" : "stop";

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
      "Calling flame tracking " + action_label + " service");

    auto client = create_client<SetBool>(
      config.set_enabled_service,
      rmw_qos_profile_services_default,
      action_callback_group_);
    if (!client->wait_for_service(timeout)) {
      clear_current_task(nullptr);
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_NOT_READY,
        "Timed out waiting for flame tracking service " + config.set_enabled_service + ".");
      return;
    }

    auto request = std::make_shared<SetBool::Request>();
    request->data = enable;
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      clear_current_task(nullptr);
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_TIMEOUT,
        "Timed out calling flame tracking service " + config.set_enabled_service + ".");
      return;
    }

    const auto response = future.get();
    if (!response->success) {
      clear_current_task(nullptr);
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_NOT_READY,
        response->message.empty() ?
        "Flame tracking service rejected the request." :
        response->message);
      return;
    }

    clear_current_task(nullptr);
    finish_result(
      goal_handle,
      true,
      ExecuteTask::Goal::STAGE_DONE,
      ExecuteTask::Result::ERROR_NONE,
      response->message.empty() ?
      (enable ? "Flame tracking started." : "Flame tracking stopped.") :
      response->message);
  }

  bool wait_for_box_target(
    const std::string & box_class,
    const rclcpp::Time & not_before,
    GraspTarget & target,
    std::string & error_message)
  {
    const auto & config = parameters_.classification_place;
    const auto deadline = now() + rclcpp::Duration::from_seconds(
      std::max(0.0, config.box_target_timeout_sec));
    const std::string normalized_box_class = normalize_class_name(box_class);

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(classification_target_mutex_);
        if (!latest_classification_grasp_target_) {
          error_message = "No box target has been received yet.";
        } else if ((latest_classification_grasp_target_received_at_ - not_before).seconds() < 0.0) {
          error_message = "Waiting for a fresh box target.";
        } else if (latest_classification_grasp_target_->header.frame_id != parameters_.planning_frame) {
          error_message = "Latest box target is not expressed in the planning frame.";
        } else if (normalize_class_name(latest_classification_grasp_target_->class_name) != normalized_box_class) {
          error_message =
            "Latest box target class is '" + latest_classification_grasp_target_->class_name +
            "', waiting for '" + box_class + "'.";
        } else if (latest_classification_grasp_target_->confidence < config.min_box_confidence) {
          error_message = "Latest box target confidence is below threshold.";
        } else if (parameters_.vision_target.require_valid_signal && !latest_classification_target_valid_) {
          error_message = "Latest box target is currently marked invalid.";
        } else if (
          latest_classification_grasp_target_->pose.position.x < parameters_.vision_target.workspace_min.x ||
          latest_classification_grasp_target_->pose.position.x > parameters_.vision_target.workspace_max.x ||
          latest_classification_grasp_target_->pose.position.y < parameters_.vision_target.workspace_min.y ||
          latest_classification_grasp_target_->pose.position.y > parameters_.vision_target.workspace_max.y ||
          latest_classification_grasp_target_->pose.position.z < parameters_.vision_target.workspace_min.z ||
          latest_classification_grasp_target_->pose.position.z > parameters_.vision_target.workspace_max.z)
        {
          error_message = "Latest box target position is outside workspace bounds.";
        } else {
          target = *latest_classification_grasp_target_;
          return true;
        }
      }

      if (now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (error_message.empty()) {
      error_message = "Timed out waiting for box target '" + box_class + "'.";
    }
    return false;
  }

  bool execute_classification_pick_from_slot(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::size_t slot_index,
    std::string & error_message)
  {
    struct RankedSlotPickCandidate
    {
      XYZ grasp_position;
      XYZ pregrasp_position;
      std::optional<XYZ> approach_position;
      XYZ offset;
      std::vector<double> approach_joint_positions;
      std::vector<double> pregrasp_joint_positions;
      double current_delta{0.0};
      double offset_norm{0.0};
      std::size_t source_index{0};
    };

    const auto & config = parameters_.classification_place;
    const auto slot = config.platform_slots[slot_index];
    const auto block_class = config.platform_slot_classes[slot_index];

    stop_gripper_hold();
    detach_pick_object_from_gazebo_link_attacher();
    detach_pick_object_from_moveit();

    if (config.move_home_before_platform_pick) {
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_MOVING_HOME,
        "Moving arm home before picking " + block_class + " from platform");
      if (!execute_named_arm_target(
          parameters_.arm_home_named_target,
          "home before classification platform pick",
          error_message))
      {
        return false;
      }
    }

    std::vector<RankedSlotPickCandidate> ranked_candidates;
    ranked_candidates.reserve(config.pick_candidate_offsets.size());
    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    for (std::size_t candidate_index = 0; candidate_index < config.pick_candidate_offsets.size();
      ++candidate_index)
    {
      const auto & offset = config.pick_candidate_offsets[candidate_index];
      const XYZ grasp_position = add_xyz(slot, offset);
      const XYZ pregrasp_position = add_xyz(grasp_position, config.pregrasp_offset);
      const auto grasp_pose = make_pose_stamped(grasp_position, config.grasp_orientation);
      const auto pregrasp_pose = make_pose_stamped(pregrasp_position, config.grasp_orientation);

      if (!has_visual_style_ik(
          grasp_pose,
          "classification slot grasp candidate " + std::to_string(candidate_index)))
      {
        continue;
      }
      if (!has_visual_style_ik(
          pregrasp_pose,
          "classification slot pregrasp candidate " + std::to_string(candidate_index)))
      {
        continue;
      }

      const double current_dx = pregrasp_position.x - current_pose.pose.position.x;
      const double current_dy = pregrasp_position.y - current_pose.pose.position.y;
      const double current_dz = pregrasp_position.z - current_pose.pose.position.z;
      const double offset_norm =
        std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);

      moveit::planning_interface::MoveGroupInterface::Plan pregrasp_preview_plan;
      std::string preview_error;
      if (!plan_arm_pose_goal(
          pregrasp_pose,
          block_class + " platform pregrasp preview candidate " + std::to_string(candidate_index),
          pregrasp_preview_plan,
          preview_error))
      {
        RCLCPP_INFO(
          get_logger(),
          "Skipping classification slot pregrasp candidate %zu for %s: %s",
          candidate_index,
          block_class.c_str(),
          preview_error.c_str());
        continue;
      }

      moveit::core::RobotStatePtr pregrasp_ik_state;
      if (!solve_arm_pose_ik_state(pregrasp_pose, pregrasp_ik_state, preview_error)) {
        RCLCPP_INFO(
          get_logger(),
          "Skipping classification slot grasp preview candidate %zu for %s: %s",
          candidate_index,
          block_class.c_str(),
          preview_error.c_str());
        continue;
      }

      std::vector<double> pregrasp_joint_positions;
      const auto * pregrasp_joint_model_group =
        pregrasp_ik_state->getJointModelGroup(parameters_.arm_group_name);
      if (!pregrasp_joint_model_group) {
        RCLCPP_INFO(
          get_logger(),
          "Skipping classification slot pregrasp candidate %zu for %s: missing joint model group "
          "'%s' while caching IK solution.",
          candidate_index,
          block_class.c_str(),
          parameters_.arm_group_name.c_str());
        continue;
      }
      pregrasp_ik_state->copyJointGroupPositions(
        pregrasp_joint_model_group, pregrasp_joint_positions);
      if (pregrasp_joint_positions.size() != 6U) {
        RCLCPP_INFO(
          get_logger(),
          "Skipping classification slot pregrasp candidate %zu for %s: expected 6 arm joints "
          "from IK solution, got %zu.",
          candidate_index,
          block_class.c_str(),
          pregrasp_joint_positions.size());
        continue;
      }

      std::optional<XYZ> approach_position;
      std::vector<double> approach_joint_positions;
      const double direct_approach_lift_z =
        std::max(0.0, config.direct_pregrasp_approach_lift_z);
      if (config.use_direct_pregrasp_joint_approach && direct_approach_lift_z > 1e-6) {
        approach_position = add_xyz(pregrasp_position, XYZ{0.0, 0.0, direct_approach_lift_z});
        const auto approach_pose =
          make_pose_stamped(*approach_position, config.grasp_orientation);
        moveit::core::RobotStatePtr approach_ik_state;
        std::string approach_preview_error;
        if (!solve_arm_pose_ik_state(approach_pose, approach_ik_state, approach_preview_error)) {
          RCLCPP_INFO(
            get_logger(),
            "Classification slot pregrasp approach candidate %zu for %s has no lifted IK "
            "solution at z+%.4f: %s",
            candidate_index,
            block_class.c_str(),
            direct_approach_lift_z,
            approach_preview_error.c_str());
          approach_position.reset();
        } else {
          const auto * approach_joint_model_group =
            approach_ik_state->getJointModelGroup(parameters_.arm_group_name);
          if (!approach_joint_model_group) {
            RCLCPP_INFO(
              get_logger(),
              "Classification slot pregrasp approach candidate %zu for %s is missing joint "
              "model group '%s'.",
              candidate_index,
              block_class.c_str(),
              parameters_.arm_group_name.c_str());
            approach_position.reset();
          } else {
            approach_ik_state->copyJointGroupPositions(
              approach_joint_model_group, approach_joint_positions);
            if (approach_joint_positions.size() != 6U) {
              RCLCPP_INFO(
                get_logger(),
                "Classification slot pregrasp approach candidate %zu for %s expected 6 arm "
                "joints from lifted IK solution, got %zu.",
                candidate_index,
                block_class.c_str(),
                approach_joint_positions.size());
              approach_position.reset();
              approach_joint_positions.clear();
            }
          }
        }
      }

      moveit::planning_interface::MoveGroupInterface::Plan grasp_preview_plan;
      if (!plan_arm_pose_goal(
          grasp_pose,
          block_class + " platform grasp preview candidate " + std::to_string(candidate_index),
          grasp_preview_plan,
          preview_error,
          pregrasp_ik_state.get()))
      {
        RCLCPP_INFO(
          get_logger(),
          "Skipping classification slot grasp candidate %zu for %s after pregrasp preview: %s",
          candidate_index,
          block_class.c_str(),
          preview_error.c_str());
        continue;
      }

      ranked_candidates.push_back(RankedSlotPickCandidate{
        grasp_position,
        pregrasp_position,
        approach_position,
        offset,
        approach_joint_positions,
        pregrasp_joint_positions,
        std::sqrt(
          current_dx * current_dx +
          current_dy * current_dy +
          current_dz * current_dz),
        offset_norm,
        candidate_index});
    }

    std::stable_sort(
      ranked_candidates.begin(),
      ranked_candidates.end(),
      [](const RankedSlotPickCandidate & lhs, const RankedSlotPickCandidate & rhs) {
        constexpr double kEpsilon = 1e-6;
        if (std::abs(lhs.offset_norm - rhs.offset_norm) > kEpsilon) {
          return lhs.offset_norm < rhs.offset_norm;
        }
        if (std::abs(lhs.current_delta - rhs.current_delta) > kEpsilon) {
          return lhs.current_delta < rhs.current_delta;
        }
        return lhs.source_index < rhs.source_index;
      });

    if (ranked_candidates.empty()) {
      error_message =
        "No classification platform pick candidate passed quick IK precheck for " + block_class + ".";
      return false;
    }

    {
      std::ostringstream candidate_summary;
      for (std::size_t i = 0; i < ranked_candidates.size(); ++i) {
        const auto & candidate = ranked_candidates[i];
        if (i > 0) {
          candidate_summary << " -> ";
        }
        candidate_summary << "#" << candidate.source_index
                          << " grasp=(" << std::fixed << std::setprecision(3)
                          << candidate.grasp_position.x << ", "
                          << candidate.grasp_position.y << ", "
                          << candidate.grasp_position.z << ")"
                          << " offset=("
                          << candidate.offset.x << ", "
                          << candidate.offset.y << ", "
                          << candidate.offset.z << ")";
      }
      RCLCPP_INFO(
        get_logger(),
        "Classification fixed-slot pick candidate order for %s: %s",
        block_class.c_str(),
        candidate_summary.str().c_str());
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
      "Opening gripper for " + block_class);
    if (parameters_.gripper_open_joint7 >= -0.5) {
      publish_gripper_target(parameters_.gripper_open_joint7, 0.40);
    } else if (!execute_named_gripper_target(
        parameters_.gripper_open_named_target,
        "opening for classification pick",
        error_message))
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    std::string last_candidate_error;
    bool reached_pick_candidate = false;
    for (std::size_t ranked_index = 0; ranked_index < ranked_candidates.size(); ++ranked_index) {
      const auto & candidate = ranked_candidates[ranked_index];
      std::string candidate_error;
      const auto pregrasp_stage_name =
        "Moving to " + block_class + " platform pregrasp candidate " +
        std::to_string(ranked_index + 1) + "/" + std::to_string(ranked_candidates.size()) +
        " (source " + std::to_string(candidate.source_index) + ")";
      bool reached_pregrasp = false;
      bool direct_pregrasp_attempted = false;
      std::string direct_pregrasp_error;
      if (config.use_direct_pregrasp_joint_approach &&
        candidate.pregrasp_joint_positions.size() == 6U)
      {
        if (candidate.approach_position.has_value() &&
          candidate.approach_joint_positions.size() == 6U)
        {
          const auto approach_stage_name =
            "Moving above " + block_class + " platform pregrasp candidate " +
            std::to_string(ranked_index + 1) + "/" + std::to_string(ranked_candidates.size()) +
            " (source " + std::to_string(candidate.source_index) + ") via direct joint approach";
          std::string direct_approach_error;
          publish_feedback(
            goal_handle,
            ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
            approach_stage_name);
          if (!execute_direct_arm_joint_target(
              candidate.approach_joint_positions,
              config.direct_pregrasp_joint_duration_sec,
              config.direct_pregrasp_joint_goal_time_tolerance_sec,
              approach_stage_name,
              direct_approach_error))
          {
            RCLCPP_WARN(
              get_logger(),
              "Direct lifted approach failed for %s platform pregrasp candidate %zu/%zu "
              "(source %zu): %s. Continuing with direct pregrasp target.",
              block_class.c_str(),
              ranked_index + 1,
              ranked_candidates.size(),
              candidate.source_index,
              direct_approach_error.c_str());
          }
        }

        direct_pregrasp_attempted = true;
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
          pregrasp_stage_name + " via direct joint approach");
        if (execute_direct_arm_joint_target(
            candidate.pregrasp_joint_positions,
            config.direct_pregrasp_joint_duration_sec,
            config.direct_pregrasp_joint_goal_time_tolerance_sec,
            pregrasp_stage_name + " via direct joint approach",
            direct_pregrasp_error))
        {
          RCLCPP_INFO(
            get_logger(),
            "Reached %s platform pregrasp candidate %zu/%zu (source %zu) via direct joint "
            "approach.",
            block_class.c_str(),
            ranked_index + 1,
            ranked_candidates.size(),
            candidate.source_index);
          reached_pregrasp = true;
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Direct joint approach failed for %s platform pregrasp candidate %zu/%zu "
            "(source %zu): %s. Skipping this candidate instead of falling back to a long pose "
            "plan from home.",
            block_class.c_str(),
            ranked_index + 1,
            ranked_candidates.size(),
            candidate.source_index,
            direct_pregrasp_error.c_str());
        }
      }

      if (direct_pregrasp_attempted && !reached_pregrasp) {
        last_candidate_error = direct_pregrasp_error.empty() ?
          ("Failed direct pregrasp approach for " + block_class + ".") :
          direct_pregrasp_error;
        continue;
      }

      if (!reached_pregrasp &&
        !execute_arm_pose_goal(
          goal_handle,
          make_pose_stamped(candidate.pregrasp_position, config.grasp_orientation),
          pregrasp_stage_name,
          ExecuteTask::Goal::STAGE_MOVING_PREGRASP,
          candidate_error))
      {
        last_candidate_error = candidate_error;
        RCLCPP_WARN(
          get_logger(),
          "Failed %s platform pregrasp candidate %zu/%zu (source %zu): %s",
          block_class.c_str(),
          ranked_index + 1,
          ranked_candidates.size(),
          candidate.source_index,
          candidate_error.c_str());
        continue;
      }

      const auto grasp_stage_name =
        "Moving to " + block_class + " platform grasp candidate " +
        std::to_string(ranked_index + 1) + "/" + std::to_string(ranked_candidates.size()) +
        " (source " + std::to_string(candidate.source_index) + ")";
      if (!execute_arm_pose_goal(
          goal_handle,
          make_pose_stamped(candidate.grasp_position, config.grasp_orientation),
          grasp_stage_name,
          ExecuteTask::Goal::STAGE_MOVING_GRASP,
          candidate_error))
      {
        last_candidate_error = candidate_error;
        RCLCPP_WARN(
          get_logger(),
          "Failed %s platform grasp candidate %zu/%zu (source %zu): %s",
          block_class.c_str(),
          ranked_index + 1,
          ranked_candidates.size(),
          candidate.source_index,
          candidate_error.c_str());
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "Selected classification platform pick candidate %zu/%zu (source %zu) for %s, "
        "offset=(%.4f, %.4f, %.4f)",
        ranked_index + 1,
        ranked_candidates.size(),
        candidate.source_index,
        block_class.c_str(),
        candidate.offset.x,
        candidate.offset.y,
        candidate.offset.z);
      reached_pick_candidate = true;
      break;
    }

    if (!reached_pick_candidate) {
      error_message = last_candidate_error.empty() ?
        ("Failed to reach any classification platform pick candidate for " + block_class + ".") :
        last_candidate_error;
      return false;
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_CLOSING_GRIPPER,
      "Closing gripper on " + block_class);
    const auto close_outcome = execute_contact_aware_gripper_close(goal_handle, error_message);
    if (close_outcome == GripperCloseOutcome::kFailed) {
      return false;
    }

    if (!attach_pick_object_to_moveit(error_message)) {
      return false;
    }
    if (parameters_.enable_gazebo_attachment &&
      !attach_pick_object_with_gazebo_link_attacher(error_message))
    {
      return false;
    }

    return execute_explicit_pose_offset(
      goal_handle,
      config.lift_offset,
      "Lifting " + block_class + " from platform",
      ExecuteTask::Goal::STAGE_LIFTING,
      error_message);
  }

  bool execute_classification_release_to_box(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const GraspTarget & box_target,
    std::string & error_message)
  {
    struct RankedReleaseCandidate
    {
      XYZ position;
      RPY orientation;
      std::vector<double> joint_positions;
      double projected_delta{0.0};
      double ideal_delta{0.0};
      double current_delta{0.0};
      double orientation_offset_norm{0.0};
      std::size_t original_index{0};
      std::size_t orientation_index{0};
      bool preview_passed{false};
    };

    const auto & config = parameters_.classification_place;
    const XYZ ideal_release_position{
      box_target.pose.position.x + config.release_offset.x,
      box_target.pose.position.y + config.release_offset.y,
      box_target.pose.position.z + config.release_offset.z};
    const XYZ projected_release_position = clamp_xyz(
      ideal_release_position,
      config.release_workspace_min,
      config.release_workspace_max);

    std::vector<XYZ> release_candidates;
    release_candidates.push_back(projected_release_position);
    if (!contains_near_xyz(release_candidates, ideal_release_position)) {
      release_candidates.push_back(ideal_release_position);
    }
    for (const auto & offset : config.release_candidate_offsets) {
      const auto candidate = clamp_xyz(
        add_xyz(projected_release_position, offset),
        config.release_workspace_min,
        config.release_workspace_max);
      if (!contains_near_xyz(release_candidates, candidate)) {
        release_candidates.push_back(candidate);
      }
    }
    for (const auto & offset : config.release_candidate_offsets) {
      const auto candidate = add_xyz(ideal_release_position, offset);
      if (!contains_near_xyz(release_candidates, candidate)) {
        release_candidates.push_back(candidate);
      }
    }

    if (config.adaptive_release_candidates_enabled) {
      const double xy_radius = std::hypot(
        ideal_release_position.x,
        ideal_release_position.y);
      const double nominal_radius = std::max(0.0, config.adaptive_release_nominal_xy_radius);
      const double backoff_step = std::max(0.0, config.adaptive_release_backoff_step);
      const double max_backoff = std::max(0.0, config.adaptive_release_max_backoff);
      const double lift_step = std::max(0.0, config.adaptive_release_lift_step);
      const double max_lift = std::max(0.0, config.adaptive_release_max_lift);
      const double overshoot = std::max(0.0, xy_radius - nominal_radius);

      if (overshoot > 1e-6 && max_backoff > 1e-6) {
        const double radial_x = xy_radius > 1e-6 ? ideal_release_position.x / xy_radius : 1.0;
        const double radial_y = xy_radius > 1e-6 ? ideal_release_position.y / xy_radius : 0.0;
        std::vector<double> backoff_amounts;
        backoff_amounts.push_back(std::min(max_backoff, overshoot));
        if (backoff_step > 1e-6) {
          backoff_amounts.push_back(std::min(max_backoff, overshoot + backoff_step));
          backoff_amounts.push_back(std::min(max_backoff, overshoot + 2.0 * backoff_step));
        }

        std::sort(backoff_amounts.begin(), backoff_amounts.end());
        backoff_amounts.erase(
          std::unique(
            backoff_amounts.begin(),
            backoff_amounts.end(),
            [](double lhs, double rhs) {
              return std::abs(lhs - rhs) < 1e-6;
            }),
          backoff_amounts.end());

        for (std::size_t i = 0; i < backoff_amounts.size(); ++i) {
          const double backoff = backoff_amounts[i];
          if (backoff <= 1e-6) {
            continue;
          }

          XYZ adaptive_candidate{
            ideal_release_position.x - radial_x * backoff,
            ideal_release_position.y - radial_y * backoff,
            ideal_release_position.z};
          adaptive_candidate = clamp_xyz(
            adaptive_candidate,
            config.release_workspace_min,
            config.release_workspace_max);
          if (!contains_near_xyz(release_candidates, adaptive_candidate)) {
            release_candidates.push_back(adaptive_candidate);
          }

          if (lift_step > 1e-6 && max_lift > 1e-6) {
            const double lift = std::min(
              max_lift,
              std::max(lift_step, static_cast<double>(i + 1) * lift_step));
            XYZ lifted_candidate = adaptive_candidate;
            lifted_candidate.z = std::min(
              config.release_workspace_max.z,
              lifted_candidate.z + lift);
            lifted_candidate = clamp_xyz(
              lifted_candidate,
              config.release_workspace_min,
              config.release_workspace_max);
            if (!contains_near_xyz(release_candidates, lifted_candidate)) {
              release_candidates.push_back(lifted_candidate);
            }
          }
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Classification release for %s: detected=(%.4f, %.4f, %.4f), ideal=(%.4f, %.4f, %.4f), fallback_projected=(%.4f, %.4f, %.4f), xy_radius=%.4f, candidates=%zu",
      box_target.class_name.c_str(),
      box_target.pose.position.x,
      box_target.pose.position.y,
      box_target.pose.position.z,
      ideal_release_position.x,
      ideal_release_position.y,
      ideal_release_position.z,
      projected_release_position.x,
      projected_release_position.y,
      projected_release_position.z,
      std::hypot(ideal_release_position.x, ideal_release_position.y),
      release_candidates.size());

    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    std::vector<RankedReleaseCandidate> ranked_candidates;
    ranked_candidates.reserve(
      release_candidates.size() * config.release_orientation_candidate_offsets.size());
    for (std::size_t candidate_index = 0; candidate_index < release_candidates.size(); ++candidate_index) {
      const auto & release_position = release_candidates[candidate_index];
      for (std::size_t orientation_index = 0;
        orientation_index < config.release_orientation_candidate_offsets.size();
        ++orientation_index)
      {
        const auto & orientation_offset =
          config.release_orientation_candidate_offsets[orientation_index];
        const RPY release_orientation{
          config.release_orientation.roll + orientation_offset.roll,
          config.release_orientation.pitch + orientation_offset.pitch,
          config.release_orientation.yaw + orientation_offset.yaw};
        const auto release_pose = make_pose_stamped(release_position, release_orientation);
        if (!has_visual_style_ik(
            release_pose,
            "classification release candidate " + std::to_string(candidate_index) +
            " orientation " + std::to_string(orientation_index)))
        {
          continue;
        }

        moveit::planning_interface::MoveGroupInterface::Plan release_preview_plan;
        std::string preview_error;
        if (!plan_arm_pose_goal(
            release_pose,
            box_target.class_name + " release preview candidate " +
            std::to_string(candidate_index) + " orientation " +
            std::to_string(orientation_index),
            release_preview_plan,
            preview_error))
        {
          RCLCPP_INFO(
            get_logger(),
            "Skipping classification release candidate %zu orientation %zu for %s after planning preview: %s",
            candidate_index,
            orientation_index,
            box_target.class_name.c_str(),
            preview_error.c_str());
          continue;
        }

        moveit::core::RobotStatePtr release_ik_state;
        if (!solve_arm_pose_ik_state(release_pose, release_ik_state, preview_error)) {
          RCLCPP_INFO(
            get_logger(),
            "Skipping classification release candidate %zu orientation %zu for %s after IK solve: %s",
            candidate_index,
            orientation_index,
            box_target.class_name.c_str(),
            preview_error.c_str());
          continue;
        }

        std::vector<double> release_joint_positions;
        const auto * release_joint_model_group =
          release_ik_state->getJointModelGroup(parameters_.arm_group_name);
        if (!release_joint_model_group) {
          RCLCPP_INFO(
            get_logger(),
            "Skipping classification release candidate %zu orientation %zu for %s: missing joint "
            "model group '%s' while caching IK solution.",
            candidate_index,
            orientation_index,
            box_target.class_name.c_str(),
            parameters_.arm_group_name.c_str());
          continue;
        }
        release_ik_state->copyJointGroupPositions(
          release_joint_model_group, release_joint_positions);
        if (release_joint_positions.size() != 6U) {
          RCLCPP_INFO(
            get_logger(),
            "Skipping classification release candidate %zu orientation %zu for %s: expected 6 arm "
            "joints from IK solution, got %zu.",
            candidate_index,
            orientation_index,
            box_target.class_name.c_str(),
            release_joint_positions.size());
          continue;
        }

        const double projected_dx = release_position.x - projected_release_position.x;
        const double projected_dy = release_position.y - projected_release_position.y;
        const double projected_dz = release_position.z - projected_release_position.z;
        const double ideal_dx = release_position.x - ideal_release_position.x;
        const double ideal_dy = release_position.y - ideal_release_position.y;
        const double ideal_dz = release_position.z - ideal_release_position.z;
        const double current_dx = release_position.x - current_pose.pose.position.x;
        const double current_dy = release_position.y - current_pose.pose.position.y;
        const double current_dz = release_position.z - current_pose.pose.position.z;
        ranked_candidates.push_back(RankedReleaseCandidate{
          release_position,
          release_orientation,
          release_joint_positions,
          std::sqrt(
            projected_dx * projected_dx +
            projected_dy * projected_dy +
            projected_dz * projected_dz),
          std::sqrt(ideal_dx * ideal_dx + ideal_dy * ideal_dy + ideal_dz * ideal_dz),
          std::sqrt(current_dx * current_dx + current_dy * current_dy + current_dz * current_dz),
          std::sqrt(
            orientation_offset.roll * orientation_offset.roll +
            orientation_offset.pitch * orientation_offset.pitch +
            orientation_offset.yaw * orientation_offset.yaw),
          candidate_index,
          orientation_index,
          true});
      }
    }

    std::stable_sort(
      ranked_candidates.begin(),
      ranked_candidates.end(),
      [](const RankedReleaseCandidate & lhs, const RankedReleaseCandidate & rhs) {
        constexpr double kScoreEpsilon = 1e-6;
        if (std::abs(lhs.projected_delta - rhs.projected_delta) > kScoreEpsilon) {
          return lhs.projected_delta < rhs.projected_delta;
        }
        if (std::abs(lhs.ideal_delta - rhs.ideal_delta) > kScoreEpsilon) {
          return lhs.ideal_delta < rhs.ideal_delta;
        }
        if (std::abs(lhs.current_delta - rhs.current_delta) > kScoreEpsilon) {
          return lhs.current_delta < rhs.current_delta;
        }
        if (std::abs(lhs.orientation_offset_norm - rhs.orientation_offset_norm) > kScoreEpsilon) {
          return lhs.orientation_offset_norm < rhs.orientation_offset_norm;
        }
        return lhs.original_index < rhs.original_index;
      });

    if (!ranked_candidates.empty()) {
      std::ostringstream candidate_summary;
      for (std::size_t i = 0; i < ranked_candidates.size(); ++i) {
        const auto & candidate = ranked_candidates[i];
        if (i > 0) {
          candidate_summary << " -> ";
        }
        candidate_summary << "#" << candidate.original_index
                          << "=(" << std::fixed << std::setprecision(3)
                          << candidate.position.x << ", "
                          << candidate.position.y << ", "
                          << candidate.position.z << ")"
                          << "@ori" << candidate.orientation_index;
      }
      RCLCPP_INFO(
        get_logger(),
        "Classification release IK-prechecked candidate order for %s: %s",
        box_target.class_name.c_str(),
        candidate_summary.str().c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No classification release candidates passed IK/planning preview for %s. "
        "Falling back to legacy pose-goal retries in original order.",
        box_target.class_name.c_str());
      for (std::size_t candidate_index = 0; candidate_index < release_candidates.size(); ++candidate_index) {
        ranked_candidates.push_back(RankedReleaseCandidate{
          release_candidates[candidate_index],
          config.release_orientation,
          {},
          0.0,
          0.0,
          0.0,
          0.0,
          candidate_index,
          0,
          false});
      }
    }

    bool release_reached = false;
    std::string last_error;
    for (std::size_t ranked_index = 0; ranked_index < ranked_candidates.size(); ++ranked_index) {
      const auto & ranked_candidate = ranked_candidates[ranked_index];
      const auto & release_position = ranked_candidate.position;
      std::string candidate_error;
      const auto stage_name =
        "Moving above " + box_target.class_name + " for release candidate " +
        std::to_string(ranked_index + 1) + "/" + std::to_string(ranked_candidates.size()) +
        " (source " + std::to_string(ranked_candidate.original_index) + ")";
      if (!ranked_candidate.preview_passed) {
        RCLCPP_WARN(
          get_logger(),
          "Release candidate %zu/%zu for %s is using legacy pose-goal fallback "
          "because no preview candidate survived IK/planning filtering.",
          ranked_index + 1,
          ranked_candidates.size(),
          box_target.class_name.c_str());
      }
      bool execute_success = false;
      if (ranked_candidate.preview_passed && ranked_candidate.joint_positions.size() == 6U) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          stage_name + " via direct joint approach");
        execute_success = execute_direct_arm_joint_target(
          ranked_candidate.joint_positions,
          config.direct_pregrasp_joint_duration_sec,
          config.direct_pregrasp_joint_goal_time_tolerance_sec,
          stage_name + " via direct joint approach",
          candidate_error);
        if (!execute_success) {
          RCLCPP_WARN(
            get_logger(),
            "Direct joint approach failed for classification release candidate %zu/%zu "
            "(source %zu, ori %zu) for %s: %s. Skipping this candidate.",
            ranked_index + 1,
            ranked_candidates.size(),
            ranked_candidate.original_index,
            ranked_candidate.orientation_index,
            box_target.class_name.c_str(),
            candidate_error.c_str());
        }
      } else {
        execute_success = execute_arm_pose_goal(
          goal_handle,
          make_pose_stamped(release_position, ranked_candidate.orientation),
          stage_name,
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          candidate_error);
      }
      if (execute_success)
      {
        RCLCPP_INFO(
          get_logger(),
          "Selected classification release candidate %zu/%zu (source %zu, ori %zu) for %s at (%.4f, %.4f, %.4f), "
          "delta_from_ideal=(%.4f, %.4f, %.4f)",
          ranked_index + 1,
          ranked_candidates.size(),
          ranked_candidate.original_index,
          ranked_candidate.orientation_index,
          box_target.class_name.c_str(),
          release_position.x,
          release_position.y,
          release_position.z,
          release_position.x - ideal_release_position.x,
          release_position.y - ideal_release_position.y,
          release_position.z - ideal_release_position.z);
        error_message.clear();
        release_reached = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "Failed classification release candidate %zu/%zu (source %zu, ori %zu) for %s: %s",
        ranked_index + 1,
        ranked_candidates.size(),
        ranked_candidate.original_index,
        ranked_candidate.orientation_index,
        box_target.class_name.c_str(),
        candidate_error.c_str());
      last_error = candidate_error;
    }

    if (!release_reached) {
      error_message =
        last_error.empty() ?
        ("Failed to plan any release candidate for " + box_target.class_name + ".") :
        last_error;
      return false;
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
      "Releasing block into " + box_target.class_name);
    return execute_direct_release_fallback(goal_handle, error_message);
  }

  void execute_classification_place_goal(const std::shared_ptr<GoalHandleExecuteTask> & goal_handle)
  {
    std::string error_message;
    if (!validate_classification_place_config(error_message)) {
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_NOT_READY,
        error_message);
      clear_current_task(nullptr);
      return;
    }

    try {
      stop_gripper_hold();
      if (parameters_.observe_pose.enabled) {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_HOME,
          "Moving to classification observe pose at zero");
        if (!execute_named_arm_target(
            parameters_.arm_home_named_target,
            "classification observe pose at zero",
            error_message))
        {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            error_message);
          clear_current_task(nullptr);
          return;
        }
      }

      const auto & config = parameters_.classification_place;
      std::vector<GraspTarget> box_targets(config.box_classes.size());
      for (std::size_t box_index = 0; box_index < config.box_classes.size(); ++box_index) {
        const auto & box_class = config.box_classes[box_index];
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
          "Locating " + box_class + " target");
        if (!set_fusion_target_class(box_class, error_message)) {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
            ExecuteTask::Result::ERROR_NOT_READY,
            error_message);
          clear_current_task(nullptr);
          return;
        }

        {
          std::lock_guard<std::mutex> lock(classification_target_mutex_);
          latest_classification_grasp_target_.reset();
          latest_classification_target_valid_ = false;
          latest_classification_grasp_target_received_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        const auto target_switch_started_at = now();

        if (!wait_for_box_target(box_class, target_switch_started_at, box_targets[box_index], error_message)) {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_WAITING_FOR_TARGET,
            ExecuteTask::Result::ERROR_TIMEOUT,
            error_message);
          clear_current_task(nullptr);
          return;
        }
      }

      for (std::size_t slot_index = 0; slot_index < config.platform_slots.size(); ++slot_index) {
        if (goal_handle->is_canceling()) {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_CANCELED,
            "Classification place task canceled.");
          clear_current_task(nullptr);
          return;
        }

        const auto & block_class = config.platform_slot_classes[slot_index];
        const auto & box_class = config.box_classes[slot_index];
        RCLCPP_INFO(
          get_logger(),
          "Classification step %zu/%zu: pick '%s' from platform slot, place into '%s'",
          slot_index + 1,
          config.platform_slots.size(),
          block_class.c_str(),
          box_class.c_str());

        if (!execute_classification_pick_from_slot(goal_handle, slot_index, error_message)) {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            error_message);
          clear_current_task(nullptr);
          return;
        }

        if (!execute_classification_release_to_box(goal_handle, box_targets[slot_index], error_message)) {
          finish_result(
            goal_handle,
            false,
            ExecuteTask::Goal::STAGE_FAILED,
            ExecuteTask::Result::ERROR_EXECUTION_FAILED,
            error_message);
          clear_current_task(nullptr);
          return;
        }
      }

      stop_gripper_hold();
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_MOVING_HOME,
        "Classification place completed: moving arm home");
      if (!execute_named_arm_target(
          parameters_.arm_home_named_target,
          "home after classification place",
          error_message))
      {
        finish_result(
          goal_handle,
          false,
          ExecuteTask::Goal::STAGE_MOVING_HOME,
          ExecuteTask::Result::ERROR_EXECUTION_FAILED,
          error_message.empty() ?
          "Failed to move arm home after classification place." :
          error_message);
        clear_current_task(nullptr);
        return;
      }

      finish_result(
        goal_handle,
        true,
        ExecuteTask::Goal::STAGE_DONE,
        ExecuteTask::Result::ERROR_NONE,
        "Classification place task completed.");
      clear_current_task(nullptr);
    } catch (const std::exception & exception) {
      stop_gripper_hold();
      detach_pick_object_from_gazebo_link_attacher();
      detach_pick_object_from_moveit();
      finish_result(
        goal_handle,
        false,
        ExecuteTask::Goal::STAGE_FAILED,
        ExecuteTask::Result::ERROR_EXECUTION_FAILED,
        std::string("Classification place task failed: ") + exception.what());
      clear_current_task(nullptr);
    }
  }

  bool execute_pre_place_alignment(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::string & error_message)
  {
    if (!parameters_.pre_place.enabled) {
      return true;
    }

    const auto initial_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);

    if (parameters_.pre_place.coarse_base_first) {
      const auto current_joint_values = arm_move_group_->getCurrentJointValues();
      const auto joint_names = arm_move_group_->getJointNames();
      const auto joint_it = std::find(
        joint_names.begin(),
        joint_names.end(),
        parameters_.pre_place.base_joint_name);
      if (joint_it == joint_names.end()) {
        error_message =
          "Pre-place base joint '" + parameters_.pre_place.base_joint_name + "' not found.";
        return false;
      }
      const std::size_t joint_index =
        static_cast<std::size_t>(std::distance(joint_names.begin(), joint_it));
      if (joint_index >= current_joint_values.size()) {
        error_message =
          "Pre-place base joint index is out of range for current joint state.";
        return false;
      }

      auto current_state = arm_move_group_->getCurrentState(5.0);
      if (!current_state) {
        error_message = "Failed to read current robot state for coarse pre-place alignment.";
        return false;
      }

      const double current_base_angle = current_joint_values[joint_index];
      const double current_object_angle =
        std::atan2(initial_pose.pose.position.y, initial_pose.pose.position.x);
      const double current_object_radius =
        std::hypot(initial_pose.pose.position.x, initial_pose.pose.position.y);
      const double desired_object_angle =
        choose_good_enough_object_angle(
        current_object_angle,
        current_object_radius,
        parameters_.place_bin,
        parameters_.pickup_object.radius) + parameters_.pre_place.heading_offset;
      const double desired_base_angle =
        current_base_angle + normalize_angle(desired_object_angle - current_object_angle);
      const auto & bounds =
        current_state->getRobotModel()->getVariableBounds(parameters_.pre_place.base_joint_name);
      const double target_base_angle = choose_nearest_bounded_angle(
        current_base_angle,
        desired_base_angle,
        bounds);

      if (std::abs(target_base_angle - current_base_angle) >= parameters_.pre_place.min_delta) {
        std::vector<double> coarse_joint_values = current_joint_values;
        coarse_joint_values[joint_index] = target_base_angle;

        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          "Coarsely rotating base toward place bin");

        arm_move_group_->clearPoseTargets();
        arm_move_group_->setStartStateToCurrentState();
        arm_move_group_->setJointValueTarget(coarse_joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan coarse_plan;
        const bool coarse_success = static_cast<bool>(arm_move_group_->plan(coarse_plan));
        if (!coarse_success) {
          error_message = "Failed to plan coarse base rotation toward place bin.";
          return false;
        }

        if (!execute_arm_plan(
            coarse_plan,
            "coarse base rotation toward place bin",
            error_message))
        {
          return false;
        }
      }
    }

    if (std::abs(parameters_.pre_place.retract_joint3_delta) >= 1e-6 ||
      std::abs(parameters_.pre_place.retract_joint4_delta) >= 1e-6)
    {
      auto retract_joint_values = arm_move_group_->getCurrentJointValues();
      const auto joint_names = arm_move_group_->getJointNames();

      const auto joint3_it = std::find(
        joint_names.begin(),
        joint_names.end(),
        parameters_.pre_place.retract_joint3_name);
      const auto joint4_it = std::find(
        joint_names.begin(),
        joint_names.end(),
        parameters_.pre_place.retract_joint4_name);

      if (joint3_it != joint_names.end() &&
        static_cast<std::size_t>(std::distance(joint_names.begin(), joint3_it)) <
        retract_joint_values.size())
      {
        const std::size_t joint3_index =
          static_cast<std::size_t>(std::distance(joint_names.begin(), joint3_it));
        retract_joint_values[joint3_index] += parameters_.pre_place.retract_joint3_delta;
      }

      if (joint4_it != joint_names.end() &&
        static_cast<std::size_t>(std::distance(joint_names.begin(), joint4_it)) <
        retract_joint_values.size())
      {
        const std::size_t joint4_index =
          static_cast<std::size_t>(std::distance(joint_names.begin(), joint4_it));
        retract_joint_values[joint4_index] += parameters_.pre_place.retract_joint4_delta;
      }

      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_MOVING_PLACE,
        "Retracting elbow and wrist toward release pose");

      arm_move_group_->clearPoseTargets();
      arm_move_group_->setStartStateToCurrentState();
      arm_move_group_->setJointValueTarget(retract_joint_values);

      moveit::planning_interface::MoveGroupInterface::Plan retract_plan;
      const bool retract_success = static_cast<bool>(arm_move_group_->plan(retract_plan));
      if (!retract_success) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to plan elbow/wrist retract step. Continuing without it.");
      } else {
        if (!execute_arm_plan(
            retract_plan,
            "elbow/wrist retract toward release pose",
            error_message))
        {
          RCLCPP_WARN(
            get_logger(),
            "Failed to execute elbow/wrist retract step: %s. Continuing without it.",
            error_message.c_str());
          error_message.clear();
        }
      }
    }

    if (!parameters_.pre_place.hover_after_base) {
      return true;
    }

    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    target_pose.header.frame_id = parameters_.planning_frame;

    const double bin_top_z =
      parameters_.place_bin.center.z + 0.5 * parameters_.place_bin.size.z;
    const double safe_hover_z = std::max(
      current_pose.pose.position.z,
      bin_top_z + parameters_.pickup_object.height + parameters_.pre_place.hover_margin_z);

    const double current_xy_angle = std::atan2(
      current_pose.pose.position.y,
      current_pose.pose.position.x);
    const double current_radius = std::hypot(
      current_pose.pose.position.x,
      current_pose.pose.position.y);
    const double desired_hover_radius = std::max(
      0.0,
      std::hypot(parameters_.place_bin.center.x, parameters_.place_bin.center.y) +
      parameters_.pre_place.hover_margin_xy);
    const double target_radius = std::min(current_radius, desired_hover_radius);
    target_pose.pose.position.x = target_radius * std::cos(current_xy_angle);
    target_pose.pose.position.y = target_radius * std::sin(current_xy_angle);
    target_pose.pose.position.z = safe_hover_z;

    tf2::Quaternion target_orientation;
    target_orientation.setRPY(
      parameters_.place_target.orientation.roll,
      parameters_.place_target.orientation.pitch,
      parameters_.place_target.orientation.yaw);
    target_pose.pose.orientation.x = target_orientation.x();
    target_pose.pose.orientation.y = target_orientation.y();
    target_pose.pose.orientation.z = target_orientation.z();
    target_pose.pose.orientation.w = target_orientation.w();

    const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    const double dz = target_pose.pose.position.z - current_pose.pose.position.z;

    tf2::Quaternion current_orientation(
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z,
      current_pose.pose.orientation.w);
    const double orientation_alignment =
      std::clamp(std::abs(current_orientation.dot(target_orientation)), 0.0, 1.0);
    const double orientation_delta = 2.0 * std::acos(orientation_alignment);

    if (std::sqrt(dx * dx + dy * dy + dz * dz) < parameters_.pre_place.min_delta &&
      orientation_delta < 0.05)
    {
      return true;
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_MOVING_PLACE,
      "Moving above place bin center");

    arm_move_group_->clearPoseTargets();
    arm_move_group_->setStartStateToCurrentState();
    arm_move_group_->setPoseTarget(target_pose, parameters_.hand_frame);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool plan_success = static_cast<bool>(arm_move_group_->plan(plan));
    if (!plan_success) {
      arm_move_group_->clearPoseTargets();
      RCLCPP_WARN(
        get_logger(),
        "Failed to plan move above place bin center. Continuing with coarse pre-place alignment only.");
      error_message.clear();
      return true;
    }

    const bool execute_success = execute_arm_plan(
      plan,
      "move above place bin center",
      error_message);
    arm_move_group_->clearPoseTargets();
    if (!execute_success) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to execute move above place bin center: %s. "
        "Continuing with coarse pre-place alignment only.",
        error_message.c_str());
      error_message.clear();
      return true;
    }

    return true;
  }

  bool execute_direct_release_fallback(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::string & error_message)
  {
    if (goal_handle->is_canceling()) {
      error_message = "Task canceled";
      return false;
    }

    stop_gripper_hold();

    if (parameters_.gripper_open_joint7 >= -0.5) {
      publish_gripper_target(parameters_.gripper_open_joint7, 0.35);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
      if (!execute_named_gripper_target(
          parameters_.gripper_open_named_target,
          "opening for direct release fallback",
          error_message))
      {
        return false;
      }
    }

    detach_pick_object_from_gazebo_link_attacher();
    detach_pick_object_from_moveit();

    const double retreat_distance = std::max(0.0, parameters_.place_target.direct_release_retreat_z);
    if (retreat_distance <= 1e-6) {
      return true;
    }

    auto current_state = arm_move_group_->getCurrentState(5.0);
    if (!current_state) {
      RCLCPP_WARN(
        get_logger(),
        "Object released, but failed to query current arm state for retreat. "
        "Treating retreat as best-effort complete.");
      error_message.clear();
      return true;
    }

    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    geometry_msgs::msg::Pose retreat_pose = current_pose.pose;
    retreat_pose.position.z += retreat_distance;

    moveit_msgs::msg::RobotTrajectory trajectory_message;
    moveit_msgs::msg::MoveItErrorCodes moveit_error_code;
    const double achieved_fraction = arm_move_group_->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{retreat_pose},
      std::max(parameters_.cartesian_step_size, 0.001),
      parameters_.cartesian_jump_threshold,
      trajectory_message,
      false,
      &moveit_error_code);
    if (achieved_fraction < 0.99) {
      RCLCPP_WARN(
        get_logger(),
        "Object released, but direct-release retreat Cartesian path only achieved fraction %.3f. "
        "Keeping the release result and skipping further retreat.",
        achieved_fraction);
      error_message.clear();
      return true;
    }

    robot_trajectory::RobotTrajectory retreat_trajectory(
      arm_move_group_->getRobotModel(),
      parameters_.arm_group_name);
    retreat_trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        retreat_trajectory,
        parameters_.cartesian_velocity_scaling,
        parameters_.cartesian_acceleration_scaling))
    {
      RCLCPP_WARN(
        get_logger(),
        "Object released, but failed to time-parameterize direct-release retreat. "
        "Keeping the release result.");
      error_message.clear();
      return true;
    }

    retreat_trajectory.getRobotTrajectoryMsg(trajectory_message);
    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_MOVING_PLACE,
      "Retreating upward after direct release");

    if (!execute_arm_robot_trajectory(
        trajectory_message,
        "direct-release upward retreat",
        error_message))
    {
      RCLCPP_WARN(
        get_logger(),
        "Object released, but upward retreat failed: %s. "
        "Keeping the release result.",
        error_message.c_str());
      error_message.clear();
      return true;
    }

    return true;
  }

  bool get_ordered_fixed_place_positions(
    std::vector<FixedPlacePoseCandidate> & ordered_candidates,
    std::string & error_message)
  {
    const auto candidates =
      get_parameter("place_target.fixed_pose_candidates_xyz").as_double_array();
    if (candidates.empty() || candidates.size() % 3 != 0) {
      error_message =
        "Parameter 'place_target.fixed_pose_candidates_xyz' must contain one or more xyz triples.";
      return false;
    }

    const int64_t index = get_parameter("place_target.fixed_pose_index").as_int();
    const int64_t candidate_count = static_cast<int64_t>(candidates.size() / 3);
    if (index < 0 || index >= candidate_count) {
      error_message =
        "Parameter 'place_target.fixed_pose_index' is out of range. Valid range is [0, " +
        std::to_string(candidate_count - 1) + "].";
      return false;
    }

    ordered_candidates.clear();
    ordered_candidates.reserve(static_cast<std::size_t>(candidate_count));
    for (int64_t candidate_index = 0; candidate_index < candidate_count; ++candidate_index) {
      const int64_t ordered_index =
        candidate_index == 0 ? index :
        (candidate_index <= index ? candidate_index - 1 : candidate_index);
      const std::size_t offset = static_cast<std::size_t>(ordered_index) * 3;
      ordered_candidates.push_back(FixedPlacePoseCandidate{
        ordered_index,
        XYZ{candidates[offset], candidates[offset + 1], candidates[offset + 2]}});
    }

    std::ostringstream summary;
    for (std::size_t i = 0; i < ordered_candidates.size(); ++i) {
      const auto & candidate = ordered_candidates[i];
      if (i > 0) {
        summary << " -> ";
      }
      summary << "#" << candidate.index
              << "=(" << std::fixed << std::setprecision(3)
              << candidate.position.x << ", "
              << candidate.position.y << ", "
              << candidate.position.z << ")";
    }
    RCLCPP_INFO(
      get_logger(),
      "Fixed place pose order: %s",
      summary.str().c_str());
    return true;
  }

  bool execute_direct_visual_pick(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::string & error_message)
  {
    const int64_t max_refresh_rounds = std::max<int64_t>(
      0,
      parameters_.vision_target.max_empty_grasp_target_refresh_count);
    int64_t refresh_round = 0;
    std::vector<double> forward_probe_schedule;
    constexpr double kProbeEpsilon = 1e-6;
    for (const double configured_probe :
      parameters_.vision_target.grasp_forward_probe_distance_candidates)
    {
      const double bounded_probe = std::max(0.0, configured_probe);
      const auto existing = std::find_if(
        forward_probe_schedule.begin(),
        forward_probe_schedule.end(),
        [bounded_probe](double candidate) {
          return std::abs(candidate - bounded_probe) < kProbeEpsilon;
        });
      if (existing == forward_probe_schedule.end()) {
        forward_probe_schedule.push_back(bounded_probe);
      }
    }
    if (forward_probe_schedule.empty()) {
      forward_probe_schedule.push_back(0.0);
    }
    while (rclcpp::ok()) {
      TaskParameters round_parameters = parameters_;
      const std::size_t forward_probe_index =
        std::min<std::size_t>(
        static_cast<std::size_t>(std::max<int64_t>(0, refresh_round)),
        forward_probe_schedule.empty() ? 0U : forward_probe_schedule.size() - 1U);
      const double round_forward_probe =
        forward_probe_schedule.empty() ? 0.0 : std::max(0.0, forward_probe_schedule[forward_probe_index]);
      if (refresh_round > 0) {
        const XYZ retry_bias = parameters_.vision_target.refresh_retry_target_position_bias;
        const XYZ retry_target_center{
          parameters_.pickup_object.center.x + retry_bias.x,
          parameters_.pickup_object.center.y + retry_bias.y,
          parameters_.pickup_object.center.z + retry_bias.z};
        round_parameters =
          build_visual_style_pick_parameters_for_center(parameters_, retry_target_center);
        round_parameters =
          apply_visual_grasp_forward_probe(round_parameters, round_forward_probe);
        std::optional<GraspTarget> current_visual_target;
        {
          std::lock_guard<std::mutex> lock(vision_target_mutex_);
          current_visual_target = latest_grasp_target_;
        }
        RCLCPP_INFO(
          get_logger(),
          "Applying refreshed-target retry bias on round %ld/%ld: raw_target=(%.4f, %.4f, %.4f) "
          "base_bias=(%.4f, %.4f, %.4f) retry_bias=(%.4f, %.4f, %.4f) "
          "effective_target=(%.4f, %.4f, %.4f)",
          refresh_round,
          max_refresh_rounds,
          current_visual_target ? current_visual_target->pose.position.x : std::numeric_limits<double>::quiet_NaN(),
          current_visual_target ? current_visual_target->pose.position.y : std::numeric_limits<double>::quiet_NaN(),
          current_visual_target ? current_visual_target->pose.position.z : std::numeric_limits<double>::quiet_NaN(),
          parameters_.vision_target.target_position_bias.x,
          parameters_.vision_target.target_position_bias.y,
          parameters_.vision_target.target_position_bias.z,
          retry_bias.x,
          retry_bias.y,
          retry_bias.z,
          round_parameters.pickup_object.center.x,
          round_parameters.pickup_object.center.y,
          round_parameters.pickup_object.center.z);
        RCLCPP_INFO(
          get_logger(),
          "Refresh round %ld/%ld will add forward probe %.4fm along approach direction.",
          refresh_round,
          max_refresh_rounds,
          round_forward_probe);
      } else {
        round_parameters =
          apply_visual_grasp_forward_probe(round_parameters, round_forward_probe);
      }

      const auto ranked_candidates = build_visual_style_pick_candidates(round_parameters);
      std::vector<TaskParameters> direct_candidates;
      if (!ranked_candidates.empty()) {
        direct_candidates.push_back(ranked_candidates.front());
      } else {
        direct_candidates.push_back(round_parameters);
      }
      RCLCPP_INFO(
        get_logger(),
        "Direct visual pick will use only the best candidate from the current visual target "
        "(refresh round %ld/%ld).",
        refresh_round,
        max_refresh_rounds);
      const auto pick_outcome = execute_visual_style_pick_to_lift(
        goal_handle,
        direct_candidates,
        "direct visual pick best-candidate",
        refresh_round < max_refresh_rounds,
        error_message);
      if (pick_outcome == VisualStylePickOutcome::kSuccess) {
        break;
      }
      if (pick_outcome == VisualStylePickOutcome::kRefreshTarget &&
        refresh_round < max_refresh_rounds)
      {
        ++refresh_round;
        RCLCPP_INFO(
          get_logger(),
          "Retrying direct visual pick with refreshed target (%ld/%ld).",
          refresh_round,
          max_refresh_rounds);
        continue;
      }
      if (pick_outcome == VisualStylePickOutcome::kRefreshTarget) {
        error_message =
          "Visual target refreshed after empty grasp, but the refresh retry budget was exhausted.";
      }
      return false;
    }

    if (!execute_pre_place_alignment(goal_handle, error_message)) {
      return false;
    }

    if (parameters_.place_target.release_after_pre_place) {
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
        "Releasing object from pre-place pose");
      return execute_direct_release_fallback(goal_handle, error_message);
    }

    std::vector<FixedPlacePoseCandidate> fixed_place_candidates;
    if (!get_ordered_fixed_place_positions(fixed_place_candidates, error_message)) {
      return false;
    }

    tf2::Quaternion fixed_place_orientation;
    fixed_place_orientation.setRPY(
      parameters_.place_target.orientation.roll,
      parameters_.place_target.orientation.pitch,
      parameters_.place_target.orientation.yaw);
    fixed_place_orientation.normalize();

    std::string last_candidate_error;
    for (std::size_t candidate_i = 0; candidate_i < fixed_place_candidates.size(); ++candidate_i) {
      const auto & fixed_place_candidate = fixed_place_candidates[candidate_i];

      geometry_msgs::msg::PoseStamped fixed_place_pose;
      fixed_place_pose.header.frame_id = parameters_.planning_frame;
      fixed_place_pose.pose.position.x = fixed_place_candidate.position.x;
      fixed_place_pose.pose.position.y = fixed_place_candidate.position.y;
      fixed_place_pose.pose.position.z = fixed_place_candidate.position.z;
      fixed_place_pose.pose.orientation.x = fixed_place_orientation.x();
      fixed_place_pose.pose.orientation.y = fixed_place_orientation.y();
      fixed_place_pose.pose.orientation.z = fixed_place_orientation.z();
      fixed_place_pose.pose.orientation.w = fixed_place_orientation.w();

      std::string candidate_error;
      if (execute_arm_pose_goal(
          goal_handle,
          fixed_place_pose,
          "Moving to fixed place pose candidate " +
          std::to_string(candidate_i + 1) + "/" +
          std::to_string(fixed_place_candidates.size()) +
          " (index " + std::to_string(fixed_place_candidate.index) + ")",
          ExecuteTask::Goal::STAGE_MOVING_PLACE,
          candidate_error))
      {
        publish_feedback(
          goal_handle,
          ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
          "Releasing object at fixed place pose");
        if (!execute_direct_release_fallback(goal_handle, error_message)) {
          return false;
        }
        return true;
      }

      last_candidate_error = candidate_error;
      RCLCPP_WARN(
        get_logger(),
        "Failed fixed place pose candidate %zu/%zu (index %ld): %s",
        candidate_i + 1,
        fixed_place_candidates.size(),
        fixed_place_candidate.index,
        candidate_error.c_str());
    }

    error_message = last_candidate_error.empty() ?
      "Failed to reach all fixed place pose candidates." :
      "Failed to reach all fixed place pose candidates. Last error: " + last_candidate_error;
    if (parameters_.place_target.allow_direct_release_fallback) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to reach any fixed place pose candidate. Releasing object from current pre-place pose instead.");
      error_message.clear();
      publish_feedback(
        goal_handle,
        ExecuteTask::Goal::STAGE_OPENING_GRIPPER,
        "All fixed place candidates failed, releasing object from current pre-place pose");
      return execute_direct_release_fallback(goal_handle, error_message);
    }
    return false;
  }

  void set_current_task(mtc::Task * task)
  {
    std::lock_guard<std::mutex> lock(current_task_mutex_);
    current_task_ = task;
  }

  void clear_current_task(mtc::Task * task)
  {
    std::lock_guard<std::mutex> lock(current_task_mutex_);
    if (task == nullptr || current_task_ == task) {
      current_task_ = nullptr;
      active_goal_ = false;
    }
  }

  void run_autostart_task()
  {
    autostart_timer_->cancel();
    RCLCPP_INFO(
      get_logger(),
      "autostart_task_type=%ld is configured. Send an action goal to '%s' to execute it.",
      parameters_.autostart_task_type,
      parameters_.action_name.c_str());
  }

  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    bool have_arm_joints[6] = {false, false, false, false, false, false};

    for (std::size_t index = 0; index < message->name.size() && index < message->position.size(); ++index) {
      if (message->name[index].size() == 6 && message->name[index].rfind("joint", 0) == 0) {
        const char joint_index = message->name[index][5];
        if (joint_index >= '1' && joint_index <= '6') {
          have_arm_joints[static_cast<std::size_t>(joint_index - '1')] = true;
        }
      }

      if (message->name[index] != "joint7") {
        continue;
      }

      std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
      latest_joint7_position_ = message->position[index];
      have_joint7_position_ = true;
    }

    bool complete_arm_state = true;
    for (const bool have_joint : have_arm_joints) {
      complete_arm_state = complete_arm_state && have_joint;
    }
    if (complete_arm_state) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      latest_arm_joint_state_received_at_ = now();
      have_arm_joint_state_ = true;
    }
  }

  void handle_grasp_target(const GraspTarget::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(vision_target_mutex_);
    latest_grasp_target_ = *message;
    latest_grasp_target_received_at_ = now();
  }

  void handle_target_valid(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(vision_target_mutex_);
    latest_target_valid_received_ = true;
    latest_target_valid_ = message->data;
  }

  void handle_classification_grasp_target(const GraspTarget::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(classification_target_mutex_);
    latest_classification_grasp_target_ = *message;
    latest_classification_grasp_target_received_at_ = now();
  }

  void handle_classification_target_valid(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(classification_target_mutex_);
    latest_classification_target_valid_ = message->data;
  }

  bool wait_for_visual_target_after(
    const rclcpp::Time & not_before,
    double timeout_sec,
    std::string & error_message)
  {
    const double bounded_timeout_sec = std::max(0.0, timeout_sec);
    const auto deadline = now() + rclcpp::Duration::from_seconds(bounded_timeout_sec);
    std::string latest_usable_error_message;

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(vision_target_mutex_);
        if (latest_grasp_target_) {
          if (is_visual_target_usable_locked(&latest_usable_error_message)) {
            if ((latest_grasp_target_received_at_ - not_before).seconds() >= 0.0) {
              return true;
            }
          } else {
            error_message = latest_usable_error_message;
          }
        } else {
          error_message = "No visual target has been received yet.";
        }
      }

      if (now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    {
      std::lock_guard<std::mutex> lock(vision_target_mutex_);
      if (latest_grasp_target_ && is_visual_target_usable_locked(&latest_usable_error_message)) {
        RCLCPP_WARN(
          get_logger(),
          "No post-home visual target arrived within %.2f s; using the latest still-valid target.",
          bounded_timeout_sec);
        return true;
      }
    }

    if (error_message.empty()) {
      if (!latest_usable_error_message.empty()) {
        error_message = latest_usable_error_message;
      } else {
        error_message = "Timed out waiting for a usable visual target after moving home.";
      }
    }
    return false;
  }

  bool wait_for_fresh_visual_target_after(
    const rclcpp::Time & not_before,
    double timeout_sec,
    std::string & error_message)
  {
    const double bounded_timeout_sec = std::max(0.0, timeout_sec);
    const auto deadline = now() + rclcpp::Duration::from_seconds(bounded_timeout_sec);

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(vision_target_mutex_);
        if (latest_grasp_target_ &&
          (latest_grasp_target_received_at_ - not_before).seconds() >= 0.0)
        {
          if (is_visual_target_usable_locked(&error_message)) {
            return true;
          }
        } else {
          error_message = "No fresh visual target has been received after moving home.";
        }
      }

      if (now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (error_message.empty()) {
      error_message = "Timed out waiting for a fresh visual target after moving home.";
    }
    return false;
  }

  bool visual_target_changed_enough(
    const GraspTarget & previous_target,
    const GraspTarget & current_target) const
  {
    const double dx = current_target.pose.position.x - previous_target.pose.position.x;
    const double dy = current_target.pose.position.y - previous_target.pose.position.y;
    const double dz = current_target.pose.position.z - previous_target.pose.position.z;
    const double position_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double dsx = current_target.size.x - previous_target.size.x;
    const double dsy = current_target.size.y - previous_target.size.y;
    const double dsz = current_target.size.z - previous_target.size.z;
    const double size_delta = std::sqrt(dsx * dsx + dsy * dsy + dsz * dsz);

    double yaw_delta = 0.0;
    if (previous_target.has_yaw != current_target.has_yaw) {
      yaw_delta = M_PI;
    } else if (previous_target.has_yaw && current_target.has_yaw) {
      yaw_delta = std::abs(normalize_angle(current_target.yaw - previous_target.yaw));
    }

    constexpr double kMinPositionDeltaMeters = 0.0;
    constexpr double kMinSizeDeltaMeters = 0.0;
    constexpr double kMinYawDeltaRad = 0.0;
    return position_delta >= kMinPositionDeltaMeters ||
           size_delta >= kMinSizeDeltaMeters ||
           yaw_delta >= kMinYawDeltaRad;
  }

  bool visual_target_matches_previous_object(
    const GraspTarget & previous_target,
    const GraspTarget & current_target,
    std::string * reason = nullptr) const
  {
    const std::string previous_class = normalize_class_name(previous_target.class_name);
    const std::string current_class = normalize_class_name(current_target.class_name);
    if (previous_class != current_class) {
      if (reason != nullptr) {
        *reason =
          "Fresh visual target belongs to a different class than the previously attempted object.";
      }
      return false;
    }

    const double dx = current_target.pose.position.x - previous_target.pose.position.x;
    const double dy = current_target.pose.position.y - previous_target.pose.position.y;
    const double dz = current_target.pose.position.z - previous_target.pose.position.z;
    const double xy_delta = std::hypot(dx, dy);
    const double xyz_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double kMaxSameObjectXyDeltaMeters = std::max(
      0.0,
      parameters_.vision_target.refresh_same_object_max_xy_delta);
    const double kMaxSameObjectXyzDeltaMeters = std::max(
      0.0,
      parameters_.vision_target.refresh_same_object_max_xyz_delta);
    const double kMaxSameObjectZDeltaMeters = std::max(
      0.0,
      parameters_.vision_target.refresh_same_object_max_z_delta);
    if (xy_delta > kMaxSameObjectXyDeltaMeters ||
      xyz_delta > kMaxSameObjectXyzDeltaMeters ||
      std::abs(dz) > kMaxSameObjectZDeltaMeters)
    {
      if (reason != nullptr) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(4)
                << "Fresh visual target drifted too far from the previous object: "
                << "delta_xyz=(" << dx << ", " << dy << ", " << dz
                << "), xy_delta=" << xy_delta << ", xyz_delta=" << xyz_delta << ".";
        *reason = message.str();
      }
      return false;
    }
    return true;
  }

  bool wait_for_changed_visual_target_after(
    const rclcpp::Time & not_before,
    const std::optional<GraspTarget> & previous_target,
    double timeout_sec,
    std::string & error_message)
  {
    const double bounded_timeout_sec = std::max(0.0, timeout_sec);
    const auto start_time = now();
    const auto deadline = start_time + rclcpp::Duration::from_seconds(bounded_timeout_sec);
    bool received_unchanged_fresh_target = false;
    bool received_mismatched_fresh_target = false;
    std::string last_wait_reason = "Waiting for a changed visual target after retreat.";
    auto next_status_log_time = start_time;

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(vision_target_mutex_);
        if (latest_grasp_target_ &&
          (latest_grasp_target_received_at_ - not_before).seconds() >= 0.0)
        {
          if (is_visual_target_usable_locked(&error_message)) {
            if (!previous_target) {
              return true;
            }
            std::string match_reason;
            if (!visual_target_matches_previous_object(
                *previous_target, *latest_grasp_target_, &match_reason))
            {
              received_mismatched_fresh_target = true;
              error_message = match_reason;
              last_wait_reason = match_reason;
            } else if (visual_target_changed_enough(*previous_target, *latest_grasp_target_)) {
              return true;
            } else {
              received_unchanged_fresh_target = true;
              error_message =
                "A fresh visual target arrived, but its pose/yaw/size did not change enough "
                "from the previous target.";
              last_wait_reason = error_message;
            }
          } else if (!error_message.empty()) {
            last_wait_reason = error_message;
          }
        } else {
          error_message = "No changed visual target has been received after retreat.";
          last_wait_reason = error_message;
        }
      }

      const auto current_time = now();
      if (current_time >= next_status_log_time) {
        const double elapsed_sec = (current_time - start_time).seconds();
        RCLCPP_INFO(
          get_logger(),
          "Still waiting for refreshed visual target after retreat: elapsed=%.2fs/%.2fs, reason=%s",
          elapsed_sec,
          bounded_timeout_sec,
          last_wait_reason.c_str());
        next_status_log_time = current_time + rclcpp::Duration::from_seconds(0.5);
      }

      if (current_time >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (error_message.empty()) {
      if (received_mismatched_fresh_target) {
        error_message =
          "Timed out waiting for a changed visual target after retreat; fresh targets arrived "
          "but they matched a different object than the one just attempted.";
      } else if (received_unchanged_fresh_target) {
        error_message =
          "Timed out waiting for a changed visual target after retreat; only equivalent targets arrived.";
      } else {
        error_message = "Timed out waiting for a changed visual target after retreat.";
      }
    }
    RCLCPP_WARN(
      get_logger(),
      "Stopped waiting for refreshed visual target after retreat after %.2fs: %s",
      (now() - start_time).seconds(),
      error_message.c_str());
    return false;
  }

  bool prepare_visual_pick_target(std::string & error_message)
  {
    std::lock_guard<std::mutex> lock(vision_target_mutex_);
    if (!parameters_.vision_target.enabled) {
      error_message = "Visual target mode is not enabled in configuration.";
      return false;
    }

    if (!is_visual_target_usable_locked(&error_message)) {
      return false;
    }

    parameters_.pickup_object.center.x =
      latest_grasp_target_->pose.position.x + parameters_.vision_target.target_position_bias.x;
    parameters_.pickup_object.center.y =
      latest_grasp_target_->pose.position.y + parameters_.vision_target.target_position_bias.y;
    parameters_.pickup_object.center.z =
      latest_grasp_target_->pose.position.z + parameters_.vision_target.target_position_bias.z;
    const double size_x = latest_grasp_target_->size.x > 1e-6 ?
      latest_grasp_target_->size.x : parameters_.vision_target.default_target_size.x;
    const double size_y = latest_grasp_target_->size.y > 1e-6 ?
      latest_grasp_target_->size.y : parameters_.vision_target.default_target_size.y;
    const double size_z = latest_grasp_target_->size.z > 1e-6 ?
      latest_grasp_target_->size.z : parameters_.vision_target.default_target_size.z;
    const double collision_scale_xy = positive_or(parameters_.vision_target.collision_scale_xy, 1.0);
    const double collision_scale_z = positive_or(parameters_.vision_target.collision_scale_z, 1.0);
    parameters_.pickup_object.radius = 0.5 * std::max(size_x, size_y) * collision_scale_xy;
    parameters_.pickup_object.height = size_z * collision_scale_z;

    if (latest_grasp_target_->has_yaw) {
      std::vector<double> yaw_candidates;
      yaw_candidates.reserve(parameters_.vision_target.yaw_candidate_offsets.size());
      for (const double offset : parameters_.vision_target.yaw_candidate_offsets) {
        yaw_candidates.push_back(normalize_angle(latest_grasp_target_->yaw + offset));
      }
      if (yaw_candidates.empty()) {
        yaw_candidates.push_back(normalize_angle(latest_grasp_target_->yaw));
      }
      parameters_.top_down_yaw_candidates = yaw_candidates;
    }

    const auto grasp_strategy =
      normalize_class_name(parameters_.vision_target.grasp_strategy);
    if (parameters_.vision_target.compute_grasp_offsets && grasp_strategy == "radial_side") {
      const double center_norm_xy = std::hypot(
        parameters_.pickup_object.center.x,
        parameters_.pickup_object.center.y);
      XYZ approach_direction{1.0, 0.0, 0.0};
      if (center_norm_xy > 1e-6) {
        approach_direction.x = parameters_.pickup_object.center.x / center_norm_xy;
        approach_direction.y = parameters_.pickup_object.center.y / center_norm_xy;
        approach_direction.z = 0.0;
      }
      if (parameters_.vision_target.lock_lateral_offsets_to_zero) {
        approach_direction.x = approach_direction.x >= 0.0 ? 1.0 : -1.0;
        approach_direction.y = 0.0;
        approach_direction.z = 0.0;
      }
      const double radial_yaw =
        normalize_angle(std::atan2(approach_direction.y, approach_direction.x));
      parameters_.grasp_orientation.yaw = radial_yaw;

      const double clearance = std::max(0.0, parameters_.vision_target.grasp_clearance);
      const double pregrasp_distance =
        std::max(0.0, parameters_.vision_target.pregrasp_distance);
      const double grasp_radius = parameters_.pickup_object.radius + clearance;
      const double grasp_z =
        0.5 * parameters_.pickup_object.height + parameters_.vision_target.grasp_z_offset;

      parameters_.approach_direction = approach_direction;
      parameters_.grasp_target_offset = {
        -approach_direction.x * grasp_radius,
        -approach_direction.y * grasp_radius,
        grasp_z};
      parameters_.pregrasp_offset = {
        parameters_.grasp_target_offset.x - approach_direction.x * pregrasp_distance,
        parameters_.grasp_target_offset.y - approach_direction.y * pregrasp_distance,
        grasp_z};

      parameters_.top_down_grasp_target_x_candidates =
        make_single_candidate(parameters_.grasp_target_offset.x);
      parameters_.top_down_grasp_target_y_candidates =
        make_single_candidate(parameters_.grasp_target_offset.y);
      parameters_.top_down_grasp_target_z_candidates =
        make_single_candidate(parameters_.grasp_target_offset.z);
      parameters_.top_down_pregrasp_x_candidates =
        make_single_candidate(parameters_.pregrasp_offset.x);
      parameters_.top_down_pregrasp_y_candidates =
        make_single_candidate(parameters_.pregrasp_offset.y);
      parameters_.top_down_pregrasp_z_candidates =
        make_single_candidate(parameters_.pregrasp_offset.z);
      parameters_.top_down_approach_direction_candidates = {
        approach_direction.x,
        approach_direction.y,
        approach_direction.z};
      parameters_.top_down_roll_candidates =
        make_single_candidate(parameters_.grasp_orientation.roll);
      parameters_.top_down_pitch_candidates =
        make_single_candidate(parameters_.grasp_orientation.pitch);
      parameters_.top_down_yaw_candidates.clear();
      append_unique_angle_candidate(parameters_.top_down_yaw_candidates, radial_yaw);
      for (const double offset : parameters_.vision_target.yaw_candidate_offsets) {
        append_unique_angle_candidate(parameters_.top_down_yaw_candidates, radial_yaw + offset);
      }

      if (!has_cartesian_distance(parameters_.approach_min_distance, parameters_.approach_max_distance) &&
        pregrasp_distance > 1e-6)
      {
        parameters_.approach_min_distance = 0.75 * pregrasp_distance;
        parameters_.approach_max_distance = pregrasp_distance;
      }

      RCLCPP_INFO(
        get_logger(),
        "Visual grasp geometry: raw_target=(%.4f, %.4f, %.4f) "
        "bias=(%.4f, %.4f, %.4f) target=(%.4f, %.4f, %.4f) size=(%.4f, %.4f, %.4f) "
        "collision_radius=%.4f collision_height=%.4f grasp_offset=(%.4f, %.4f, %.4f) "
        "pregrasp_offset=(%.4f, %.4f, %.4f) approach=(%.4f, %.4f, %.4f) "
        "orientation_rpy=(%.4f, %.4f, %.4f) approach_distance=[%.4f, %.4f]",
        latest_grasp_target_->pose.position.x,
        latest_grasp_target_->pose.position.y,
        latest_grasp_target_->pose.position.z,
        parameters_.vision_target.target_position_bias.x,
        parameters_.vision_target.target_position_bias.y,
        parameters_.vision_target.target_position_bias.z,
        parameters_.pickup_object.center.x,
        parameters_.pickup_object.center.y,
        parameters_.pickup_object.center.z,
        size_x,
        size_y,
        size_z,
        parameters_.pickup_object.radius,
        parameters_.pickup_object.height,
        parameters_.grasp_target_offset.x,
        parameters_.grasp_target_offset.y,
        parameters_.grasp_target_offset.z,
        parameters_.pregrasp_offset.x,
        parameters_.pregrasp_offset.y,
        parameters_.pregrasp_offset.z,
        parameters_.approach_direction.x,
        parameters_.approach_direction.y,
        parameters_.approach_direction.z,
        parameters_.grasp_orientation.roll,
        parameters_.grasp_orientation.pitch,
        parameters_.grasp_orientation.yaw,
        parameters_.approach_min_distance,
        parameters_.approach_max_distance);
    }

    auto node_handle = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
    factory_ = std::make_unique<TaskFactory>(node_handle, parameters_);
    return true;
  }

  bool is_visual_target_usable_locked(std::string * error_message) const
  {
    if (!latest_grasp_target_) {
      if (error_message != nullptr) {
        *error_message = "No visual target has been received yet.";
      }
      return false;
    }

    if ((parameters_.vision_target.require_valid_signal ||
      parameters_.vision_target.require_single_target) &&
      !latest_target_valid_received_)
    {
      if (error_message != nullptr) {
        *error_message = "No visual target validity signal has been received yet.";
      }
      return false;
    }

    if (parameters_.vision_target.require_valid_signal && !latest_target_valid_) {
      if (error_message != nullptr) {
        *error_message = "Latest visual target is currently marked invalid.";
      }
      return false;
    }

    if (parameters_.vision_target.require_single_target && !latest_target_valid_) {
      if (error_message != nullptr) {
        *error_message =
          "Single-target gating rejected the latest visual target because fusion marked it invalid.";
      }
      return false;
    }

    const auto age = (now() - latest_grasp_target_received_at_).seconds();
    if (age > parameters_.vision_target.target_timeout_sec) {
      if (error_message != nullptr) {
        *error_message = "Latest visual target is stale.";
      }
      return false;
    }

    if (latest_grasp_target_->header.frame_id != parameters_.planning_frame) {
      if (error_message != nullptr) {
        *error_message = "Latest visual target is not expressed in the planning frame.";
      }
      return false;
    }

    if (latest_grasp_target_->confidence < parameters_.vision_target.min_target_confidence) {
      if (error_message != nullptr) {
        *error_message = "Latest visual target confidence is below threshold.";
      }
      return false;
    }

    const auto & position = latest_grasp_target_->pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
      if (error_message != nullptr) {
        *error_message = "Latest visual target position contains a non-finite value.";
      }
      return false;
    }

    const auto & workspace_min = parameters_.vision_target.workspace_min;
    const auto & workspace_max = parameters_.vision_target.workspace_max;
    if (position.x < workspace_min.x || position.x > workspace_max.x ||
      position.y < workspace_min.y || position.y > workspace_max.y ||
      position.z < workspace_min.z || position.z > workspace_max.z)
    {
      if (error_message != nullptr) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(4)
                << "Latest visual target position is outside workspace bounds: position=("
                << position.x << ", " << position.y << ", " << position.z << ") bounds=[("
                << workspace_min.x << ", " << workspace_min.y << ", " << workspace_min.z
                << "), (" << workspace_max.x << ", " << workspace_max.y << ", "
                << workspace_max.z << ")].";
        *error_message = message.str();
      }
      return false;
    }

    const std::string normalized = normalize_class_name(latest_grasp_target_->class_name);
    const auto allowed = std::find_if(
      parameters_.vision_target.allowed_target_classes.begin(),
      parameters_.vision_target.allowed_target_classes.end(),
      [&normalized](const std::string & candidate) {
        return normalize_class_name(candidate) == normalized;
      });
    if (allowed == parameters_.vision_target.allowed_target_classes.end()) {
      if (error_message != nullptr) {
        *error_message = "Latest visual target class is not allowed.";
      }
      return false;
    }

    return true;
  }

  GripperCloseOutcome execute_visual_style_gripper_close(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const std::string & feedback_message,
    const std::string & action_label,
    std::string & error_message)
  {
    if (goal_handle->is_canceling()) {
      error_message = "Task canceled";
      return GripperCloseOutcome::kFailed;
    }

    publish_feedback(
      goal_handle,
      ExecuteTask::Goal::STAGE_CLOSING_GRIPPER,
      feedback_message);
    if (parameters_.use_contact_aware_gripper_close && parameters_.gripper_close_joint7 >= 0.0) {
      return execute_contact_aware_gripper_close(goal_handle, error_message);
    }

    if (parameters_.gripper_close_joint7 >= -0.5) {
      publish_gripper_target(parameters_.gripper_close_joint7, 0.50);
    } else if (!execute_named_gripper_target(
        parameters_.gripper_close_named_target,
        action_label,
        error_message))
    {
      return GripperCloseOutcome::kFailed;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    return GripperCloseOutcome::kContact;
  }

  bool execute_solution_with_contact_aware_gripper_close(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    const mtc::SolutionBase & solution,
    std::string & error_message,
    bool close_gripper_on_contact = true)
  {
    std::vector<ExecutableTrajectoryStep> steps;
    collect_executable_trajectory_steps(solution, steps);
    if (steps.empty()) {
      error_message = "MTC solution did not contain executable trajectories.";
      return false;
    }

    bool has_lift_step = false;
    std::ostringstream step_list;
    step_list << "Executable MTC trajectory steps:";
    for (const auto & step : steps) {
      step_list << "\n  stage='" << step.stage_name
                << "' joints=" << step.trajectory.joint_trajectory.joint_names.size()
                << " points=" << step.trajectory.joint_trajectory.points.size();
      if (stage_name_contains(step.stage_name, "lift")) {
        has_lift_step = true;
      }
    }
    RCLCPP_INFO_STREAM(get_logger(), step_list.str());
    if (!has_lift_step) {
      RCLCPP_WARN(get_logger(), "No explicit 'lift' step was found in the executable MTC sub-trajectories");
    }

    for (const auto & step : steps) {
      if (goal_handle->is_canceling()) {
        error_message = "Task canceled";
        return false;
      }

      publish_feedback(
        goal_handle,
        feedback_stage_for_step(step.stage_name),
        "Executing stage '" + step.stage_name + "'");

      if (stage_name_contains(step.stage_name, "open gripper")) {
        stop_gripper_hold();
        detach_pick_object_from_gazebo_link_attacher();
        detach_pick_object_from_moveit();
      }

      if (close_gripper_on_contact && stage_name_contains(step.stage_name, "close gripper")) {
        const auto close_outcome = execute_contact_aware_gripper_close(goal_handle, error_message);
        if (close_outcome == GripperCloseOutcome::kFailed) {
          return false;
        }
        if (!attach_pick_object_to_moveit(error_message)) {
          return false;
        }
        if (parameters_.enable_gazebo_attachment &&
          !attach_pick_object_with_gazebo_link_attacher(error_message))
        {
          return false;
        }
        continue;
      }

      if (close_gripper_on_contact && stage_name_contains(step.stage_name, "lift")) {
        if (!execute_explicit_lift(goal_handle, error_message)) {
          return false;
        }
        continue;
      }

      const auto & trajectory = step.trajectory;
      if (trajectory_targets_gripper(trajectory)) {
        const auto execution_result = gripper_move_group_->execute(trajectory);
        if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          error_message =
            "Failed to execute stage '" + step.stage_name + "' with MoveIt error code " +
            std::to_string(execution_result.val);
          return false;
        }
      } else if (!execute_arm_robot_trajectory(trajectory, step.stage_name, error_message)) {
        error_message = "Failed to execute stage '" + step.stage_name + "': " + error_message;
        return false;
      }
    }

    return true;
  }

  bool execute_explicit_lift(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::string & error_message,
    const std::optional<XYZ> & escape_approach_direction = std::nullopt)
  {
    if (goal_handle->is_canceling()) {
      error_message = "Task canceled";
      return false;
    }

    const double direction_norm = std::sqrt(
      parameters_.lift_direction.x * parameters_.lift_direction.x +
      parameters_.lift_direction.y * parameters_.lift_direction.y +
      parameters_.lift_direction.z * parameters_.lift_direction.z);
    if (direction_norm <= 1e-9) {
      error_message = "Lift direction is zero-length.";
      return false;
    }

    const double lift_distance =
      std::max(parameters_.lift_min_distance, parameters_.lift_max_distance);
    if (lift_distance <= 1e-6) {
      return true;
    }

    auto current_state = arm_move_group_->getCurrentState(5.0);
    if (!current_state) {
      error_message = "Failed to query current arm state before explicit lift.";
      return false;
    }

    const auto current_pose = arm_move_group_->getCurrentPose(parameters_.hand_frame);
    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    target_pose.position.x += (parameters_.lift_direction.x / direction_norm) * lift_distance;
    target_pose.position.y += (parameters_.lift_direction.y / direction_norm) * lift_distance;
    target_pose.position.z += (parameters_.lift_direction.z / direction_norm) * lift_distance;

    moveit_msgs::msg::RobotTrajectory trajectory_message;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double achieved_fraction = arm_move_group_->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target_pose},
      std::max(parameters_.cartesian_step_size, 0.001),
      parameters_.cartesian_jump_threshold,
      trajectory_message,
      false,
      &error_code);
    const bool vertical_path_complete =
      is_complete_cartesian_path_fraction(achieved_fraction) &&
      error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    const bool must_use_escape_fallback =
      parameters_.vision_target.postgrasp_escape_enabled && escape_approach_direction.has_value();
    const double achieved_lift_distance = achieved_fraction * lift_distance;
    if (!vertical_path_complete) {
      const double required_lift_distance =
        std::max(parameters_.lift_min_distance, 0.0);
      const std::string vertical_error =
        "Explicit lift Cartesian path is incomplete (fraction " +
        std::to_string(achieved_fraction) + ", MoveIt error code " +
        std::to_string(error_code.val) + ", distance " +
        std::to_string(achieved_lift_distance) + " m, required at least " +
        std::to_string(required_lift_distance) + " m)";
      if (!must_use_escape_fallback &&
        achieved_lift_distance + 1e-6 >= required_lift_distance)
      {
        RCLCPP_WARN(
          get_logger(),
          "Explicit lift Cartesian path is incomplete (fraction %.6f, MoveIt error code %d), but "
          "that still yields %.4f m "
          "which meets the required minimum %.4f m. Executing the partial lift.",
          achieved_fraction,
          error_code.val,
          achieved_lift_distance,
          required_lift_distance);
      } else if (!must_use_escape_fallback) {
        error_message = vertical_error;
        return false;
      } else {
        const auto escape_pose = make_planning_frame_postgrasp_escape_pose(
          current_pose,
          *escape_approach_direction,
          parameters_.vision_target.postgrasp_escape_retreat_distance,
          lift_distance);
        if (!escape_pose) {
          error_message = vertical_error + "; postgrasp escape target is invalid; vertical trajectory was not executed";
          return false;
        }

        moveit_msgs::msg::RobotTrajectory escape_trajectory_message;
        moveit_msgs::msg::MoveItErrorCodes escape_error_code;
        const double escape_fraction = arm_move_group_->computeCartesianPath(
          std::vector<geometry_msgs::msg::Pose>{escape_pose->pose},
          std::max(parameters_.cartesian_step_size, 0.001),
          parameters_.cartesian_jump_threshold,
          escape_trajectory_message,
          parameters_.vision_target.postgrasp_escape_avoid_collisions,
          &escape_error_code);
        const double retreat_distance = parameters_.vision_target.postgrasp_escape_retreat_distance;
        RCLCPP_INFO(
          get_logger(),
          "Postgrasp escape fallback offset=(%.4f, %.4f, %.4f), fraction=%.6f, error_code=%d, avoid_collisions=%s; vertical trajectory was not executed",
          escape_pose->pose.position.x - current_pose.pose.position.x,
          escape_pose->pose.position.y - current_pose.pose.position.y,
          escape_pose->pose.position.z - current_pose.pose.position.z,
          escape_fraction,
          escape_error_code.val,
          parameters_.vision_target.postgrasp_escape_avoid_collisions ? "true" : "false");
        if (!is_complete_cartesian_path_fraction(escape_fraction) ||
          escape_error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          error_message = vertical_error + "; postgrasp escape Cartesian path is incomplete (fraction " +
            std::to_string(escape_fraction) + ", MoveIt error code " +
            std::to_string(escape_error_code.val) + ", retreat " +
            std::to_string(retreat_distance) + " m, avoid_collisions=" +
            (parameters_.vision_target.postgrasp_escape_avoid_collisions ? "true" : "false") +
            "); vertical trajectory was not executed";
          return false;
        }

        trajectory_message = std::move(escape_trajectory_message);
        RCLCPP_WARN(
          get_logger(),
          "Explicit vertical lift is incomplete (fraction %.6f, MoveIt error code %d); executing full postgrasp escape fallback. Vertical trajectory was not executed.",
          achieved_fraction,
          error_code.val);
      }
    }

    robot_trajectory::RobotTrajectory lift_trajectory(
      arm_move_group_->getRobotModel(),
      parameters_.arm_group_name);
    lift_trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        lift_trajectory,
        parameters_.cartesian_velocity_scaling,
        parameters_.cartesian_acceleration_scaling))
    {
      error_message = "Failed to time-parameterize explicit lift trajectory.";
      return false;
    }

    lift_trajectory.getRobotTrajectoryMsg(trajectory_message);

    if (!execute_arm_robot_trajectory(
        trajectory_message,
        "explicit lift",
        error_message))
    {
      error_message = "Explicit lift execution failed: " + error_message;
      return false;
    }

    return true;
  }

  GripperCloseOutcome execute_contact_aware_gripper_close(
    const std::shared_ptr<GoalHandleExecuteTask> & goal_handle,
    std::string & error_message)
  {
    constexpr double kCloseStepJoint7 = 0.0040;
    constexpr double kContactPreloadJoint7 = 0.0040;
    constexpr double kReachTolerance = 0.0006;
    constexpr double kProgressEpsilon = 0.0002;
    constexpr double kCommandDurationSec = 0.40;
    constexpr double kStepTimeoutSec = 1.2;
    constexpr double kStallTimeoutSec = 0.60;
    constexpr double kPollIntervalSec = 0.02;

    if (!wait_for_joint7_state(kStepTimeoutSec, error_message)) {
      return GripperCloseOutcome::kFailed;
    }

    const double final_target = normalize_direct_gripper_target(parameters_.gripper_close_joint7);
    double current_position = latest_joint7_position();

    while (final_target - current_position > kReachTolerance) {
      if (goal_handle->is_canceling()) {
        error_message = "Task canceled";
        return GripperCloseOutcome::kFailed;
      }

      const double commanded_target = std::min(final_target, current_position + kCloseStepJoint7);
      publish_gripper_target(commanded_target, kCommandDurationSec);

      const auto step_started_at = std::chrono::steady_clock::now();
      auto last_progress_time = step_started_at;
      double last_progress_position = current_position;
      bool reached_command = false;
      bool stalled = false;

      while (std::chrono::duration<double>(std::chrono::steady_clock::now() - step_started_at).count() <
        kStepTimeoutSec)
      {
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(kPollIntervalSec)));

        const double observed_position = latest_joint7_position();
        if (observed_position >= commanded_target - kReachTolerance) {
          current_position = observed_position;
          reached_command = true;
          break;
        }

        if (observed_position - last_progress_position > kProgressEpsilon) {
          last_progress_position = observed_position;
          last_progress_time = std::chrono::steady_clock::now();
        } else if (
          std::chrono::duration<double>(std::chrono::steady_clock::now() - last_progress_time).count() >=
          kStallTimeoutSec)
        {
          current_position = observed_position;
          stalled = true;
          break;
        }
      }

      if (reached_command) {
        continue;
      }

      if (stalled) {
        const double hold_target =
          std::min(final_target, current_position + kContactPreloadJoint7);
        start_gripper_hold(hold_target);
        if (!wait_for_gripper_hold_engaged(hold_target, 0.5, error_message)) {
          return GripperCloseOutcome::kFailed;
        }
        RCLCPP_INFO(
          get_logger(),
          "Stopped gripper close after contact-like stall: target=%.4f, contact=%.4f, hold=%.4f",
          commanded_target,
          current_position,
          hold_target);
        return GripperCloseOutcome::kContact;
      }

      error_message =
        "Gripper close step timed out before reaching its target or detecting a stable contact.";
      return GripperCloseOutcome::kFailed;
    }

    start_gripper_hold(final_target);
    if (!wait_for_gripper_hold_engaged(final_target, 0.5, error_message)) {
      return GripperCloseOutcome::kFailed;
    }
    RCLCPP_WARN(
      get_logger(),
      "Gripper reached its fully closed target without contact-like stall; this may be an empty grasp.");
    return GripperCloseOutcome::kFullyClosed;
  }

  bool wait_for_joint7_state(double timeout_sec, std::string & error_message)
  {
    const auto started_at = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count() <
      timeout_sec)
    {
      {
        std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
        if (have_joint7_position_) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    error_message = "Timed out waiting for joint7 state feedback.";
    return false;
  }

  double latest_joint7_position()
  {
    std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
    return latest_joint7_position_;
  }

  bool has_fresh_arm_joint_state(double max_age_sec, double * age_sec = nullptr)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!have_arm_joint_state_) {
      return false;
    }

    const double age = (now() - latest_arm_joint_state_received_at_).seconds();
    if (age_sec != nullptr) {
      *age_sec = age;
    }
    return age <= std::max(0.0, max_age_sec);
  }

  bool wait_for_fresh_arm_joint_state(
    double timeout_sec,
    double max_age_sec,
    std::string & error_message)
  {
    const auto deadline = now() + rclcpp::Duration::from_seconds(std::max(0.0, timeout_sec));
    double last_age_sec = 0.0;

    while (rclcpp::ok()) {
      if (has_fresh_arm_joint_state(max_age_sec, &last_age_sec)) {
        return true;
      }

      if (now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!have_arm_joint_state_) {
        error_message =
          "Piper joint state preflight failed: no complete /joint_states sample for joints 1-6. "
          "The Piper driver may still be restarting or CAN may require manual recovery.";
        return false;
      }
      last_age_sec = (now() - latest_arm_joint_state_received_at_).seconds();
    }

    std::ostringstream message;
    message << "Piper joint state preflight failed: latest /joint_states sample is "
            << std::fixed << std::setprecision(3) << last_age_sec
            << "s old, max allowed is " << std::setprecision(3)
            << std::max(0.0, max_age_sec)
            << "s. The Piper driver may still be restarting or CAN may require manual recovery.";
    error_message = message.str();
    return false;
  }

  double normalize_direct_gripper_target(double joint7_target)
  {
    // MoveIt gripper semantics in this workspace are: open is negative, close is zero.
    // Preserve compatibility with older positive-open configs by mirroring them here.
    if (joint7_target > 0.0) {
      return -std::clamp(joint7_target, 0.0, 0.035);
    }
    return std::clamp(joint7_target, -0.040, 0.0);
  }

  bool execute_named_gripper_target(
    const std::string & named_target,
    const std::string & action_label,
    std::string & error_message)
  {
    gripper_move_group_->setStartStateToCurrentState();
    if (!gripper_move_group_->setNamedTarget(named_target)) {
      error_message = "Failed to set gripper " + action_label + " target.";
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    if (!static_cast<bool>(gripper_move_group_->plan(gripper_plan))) {
      error_message = "Failed to plan gripper " + action_label + ".";
      return false;
    }

    const auto gripper_execute_result = gripper_move_group_->execute(gripper_plan);
    if (gripper_execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error_message = "Failed to execute gripper " + action_label + ".";
      return false;
    }
    return true;
  }

  bool execute_named_arm_target(
    const std::string & named_target,
    const std::string & action_label,
    std::string & error_message)
  {
    if (named_target == "zero") {
      std::string direct_error_message;
      if (execute_direct_arm_joint_target(
          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          8.0,
          20.0,
          action_label,
          direct_error_message))
      {
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "Direct arm zero execution for '%s' failed: %s. Falling back to MoveIt named target.",
        action_label.c_str(),
        direct_error_message.c_str());
    }

    arm_move_group_->clearPoseTargets();
    arm_move_group_->setStartStateToCurrentState();
    if (!arm_move_group_->setNamedTarget(named_target)) {
      error_message = "Failed to set arm " + action_label + " target '" + named_target + "'.";
      arm_move_group_->clearPoseTargets();
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    if (!static_cast<bool>(arm_move_group_->plan(arm_plan))) {
      error_message = "Failed to plan arm " + action_label + " target '" + named_target + "'.";
      arm_move_group_->clearPoseTargets();
      return false;
    }

    arm_move_group_->clearPoseTargets();
    if (!execute_arm_plan(arm_plan, action_label + " target '" + named_target + "'", error_message)) {
      return false;
    }
    return true;
  }

  bool execute_direct_arm_joint_target(
    const std::vector<double> & joint_positions,
    double duration_sec,
    double goal_time_tolerance_sec,
    const std::string & action_label,
    std::string & error_message)
  {
    if (joint_positions.size() != 6U) {
      error_message =
        "Direct arm target for '" + action_label + "' must contain exactly 6 joint values.";
      return false;
    }

    if (!arm_direct_trajectory_client_) {
      error_message = "Direct arm trajectory client is not initialized.";
      return false;
    }

    if (!arm_direct_trajectory_client_->wait_for_action_server(std::chrono::seconds(3))) {
      error_message = "Arm controller action server is not available for '" + action_label + "'.";
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    trajectory_msgs::msg::JointTrajectoryPoint target_point;
    target_point.positions = joint_positions;
    target_point.time_from_start = rclcpp::Duration::from_seconds(
      positive_or(duration_sec, 8.0));
    goal.trajectory.points.push_back(std::move(target_point));
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(
      positive_or(goal_time_tolerance_sec, 20.0));

    auto goal_handle_future = arm_direct_trajectory_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      error_message =
        "Timed out while sending direct arm target for '" + action_label + "'.";
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      error_message = "Arm controller rejected direct target for '" + action_label + "'.";
      return false;
    }

    auto result_future = arm_direct_trajectory_client_->async_get_result(goal_handle);
    const auto timeout = std::chrono::duration<double>(
      positive_or(duration_sec, 8.0) + positive_or(goal_time_tolerance_sec, 20.0) + 5.0);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      arm_direct_trajectory_client_->async_cancel_goal(goal_handle);
      error_message =
        "Timed out waiting for direct arm target '" + action_label + "' to finish.";
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
      error_message =
        "Direct arm target '" + action_label + "' did not finish successfully.";
      return false;
    }

    if (wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      error_message =
        "Direct arm target '" + action_label + "' failed: " +
        wrapped_result.result->error_string;
      return false;
    }

    return true;
  }

  bool execute_arm_joint_trajectory_direct(
    const trajectory_msgs::msg::JointTrajectory & joint_trajectory,
    const std::string & action_label,
    std::string & error_message)
  {
    if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty()) {
      error_message =
        "Direct arm trajectory for '" + action_label + "' is empty.";
      return false;
    }

    if (!arm_direct_trajectory_client_) {
      error_message = "Direct arm trajectory client is not initialized.";
      return false;
    }

    if (!arm_direct_trajectory_client_->wait_for_action_server(std::chrono::seconds(3))) {
      error_message =
        "Arm controller action server is not available for '" + action_label + "'.";
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = joint_trajectory;
    const double goal_time_tolerance_sec =
      positive_or(parameters_.arm_trajectory_goal_time_tolerance_sec, 8.0);
    goal.goal_time_tolerance =
      rclcpp::Duration::from_seconds(goal_time_tolerance_sec);

    auto goal_handle_future = arm_direct_trajectory_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      error_message =
        "Timed out while sending direct arm trajectory for '" + action_label + "'.";
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      error_message =
        "Arm controller rejected direct arm trajectory for '" + action_label + "'.";
      return false;
    }

    auto result_future = arm_direct_trajectory_client_->async_get_result(goal_handle);
    const double nominal_duration_sec =
      rclcpp::Duration(goal.trajectory.points.back().time_from_start).seconds();
    const auto timeout = std::chrono::duration<double>(
      positive_or(nominal_duration_sec, 8.0) +
      goal_time_tolerance_sec +
      positive_or(parameters_.arm_trajectory_result_timeout_padding_sec, 8.0));
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      arm_direct_trajectory_client_->async_cancel_goal(goal_handle);
      error_message =
        "Timed out waiting for direct arm trajectory '" + action_label + "' to finish.";
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
      error_message =
        "Direct arm trajectory '" + action_label + "' did not finish successfully.";
      return false;
    }

    if (wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      error_message =
        "Direct arm trajectory '" + action_label + "' failed: " +
        wrapped_result.result->error_string;
      return false;
    }

    return true;
  }

  bool execute_arm_robot_trajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & action_label,
    std::string & error_message)
  {
    if (parameters_.prefer_direct_arm_trajectory_execution &&
      !trajectory.joint_trajectory.points.empty())
    {
      return execute_arm_joint_trajectory_direct(
        trajectory.joint_trajectory,
        action_label,
        error_message);
    }

    const auto execution_result = arm_move_group_->execute(trajectory);
    if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error_message =
        "Failed to execute arm trajectory for '" + action_label +
        "' with MoveIt error code " + std::to_string(execution_result.val) + ".";
      return false;
    }
    return true;
  }

  bool execute_arm_plan(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::string & action_label,
    std::string & error_message)
  {
    return execute_arm_robot_trajectory(plan.trajectory_, action_label, error_message);
  }

  void publish_gripper_target(double joint7_target, double duration_sec)
  {
    constexpr double kGripperCommandEffort = 3.0;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto & joint_trajectory = plan.trajectory_.joint_trajectory;
    joint_trajectory.joint_names = {"joint7"};

    auto start_point = trajectory_msgs::msg::JointTrajectoryPoint();
    start_point.positions = {std::clamp(latest_joint7_position(), -0.040, 0.0)};
    start_point.effort = {kGripperCommandEffort};
    start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    joint_trajectory.points.push_back(std::move(start_point));

    auto target_point = trajectory_msgs::msg::JointTrajectoryPoint();
    target_point.positions = {normalize_direct_gripper_target(joint7_target)};
    target_point.effort = {kGripperCommandEffort};
    target_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    joint_trajectory.points.push_back(std::move(target_point));

    const auto execute_result = gripper_move_group_->execute(plan);
    if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to execute gripper target through MoveIt bridge, error code %d",
        execute_result.val);
    }
  }

  void start_gripper_hold(double joint7_target)
  {
    {
      std::lock_guard<std::mutex> lock(gripper_hold_mutex_);
      gripper_hold_target_ = std::clamp(joint7_target, -0.040, 0.0);
      gripper_hold_active_ = true;
    }
    publish_active_gripper_hold();
  }

  void stop_gripper_hold()
  {
    {
      std::lock_guard<std::mutex> lock(gripper_hold_mutex_);
      gripper_hold_active_ = false;
    }
  }

  bool gripper_hold_active()
  {
    std::lock_guard<std::mutex> lock(gripper_hold_mutex_);
    return gripper_hold_active_;
  }

  bool wait_for_gripper_hold_engaged(
    double hold_target,
    double timeout_sec,
    std::string & error_message)
  {
    const auto started_at = std::chrono::steady_clock::now();
    double last_position = latest_joint7_position();

    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count() <
      timeout_sec)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      const double observed_position = latest_joint7_position();

      if (observed_position >= hold_target - 0.0008) {
        return true;
      }

      // In this workspace, gripper closing moves joint7 toward zero from negative values.
      if (observed_position > last_position + 0.0001) {
        last_position = observed_position;
        continue;
      }
    }

    const double observed_position = latest_joint7_position();
    RCLCPP_WARN(
      get_logger(),
      "Gripper hold did not settle onto target before lift: hold_target=%.4f observed=%.4f. "
      "Continuing with best-effort hold.",
      hold_target,
      observed_position);
    error_message.clear();
    return true;
  }

  void publish_active_gripper_hold()
  {
    double hold_target = 0.0;
    {
      std::lock_guard<std::mutex> lock(gripper_hold_mutex_);
      if (!gripper_hold_active_) {
        return;
      }
      hold_target = gripper_hold_target_;
    }
    publish_gripper_target(hold_target, 0.40);
  }

  bool attach_pick_object_to_moveit(std::string & error_message)
  {
    const bool attached =
      parameters_.touch_links.empty() ?
      arm_move_group_->attachObject(parameters_.pickup_object.id, parameters_.hand_frame) :
      arm_move_group_->attachObject(
      parameters_.pickup_object.id,
      parameters_.hand_frame,
      parameters_.touch_links);
    if (!attached) {
      error_message =
        "Gripper contact succeeded, but failed to attach the object into MoveIt's planning scene.";
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Attached '%s' to '%s' for manual pick execution",
      parameters_.pickup_object.id.c_str(),
      parameters_.hand_frame.c_str());
    return true;
  }

  void detach_pick_object_from_moveit()
  {
    arm_move_group_->detachObject(parameters_.pickup_object.id);
  }

  bool attach_pick_object_with_gazebo_link_attacher(std::string & error_message)
  {
#if !defined(HAVE_LINKATTACHER_MSGS)
    (void)error_message;
    return true;
#else
    if (!parameters_.enable_gazebo_attachment) {
      return true;
    }

    if (!gazebo_attach_link_client_) {
      error_message = "Official Gazebo link attacher client is unavailable.";
      return false;
    }
    if (!gazebo_attach_link_client_->wait_for_service(std::chrono::seconds(2))) {
      error_message = "Timed out waiting for the /ATTACHLINK service.";
      return false;
    }

    auto request = std::make_shared<AttachLink::Request>();
    request->model1_name = parameters_.gazebo_attach_robot_model;
    request->link1_name = parameters_.gazebo_attach_robot_link;
    request->model2_name = parameters_.gazebo_attach_object_model;
    request->link2_name = parameters_.gazebo_attach_object_link;

    auto future = gazebo_attach_link_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      error_message = "Timed out waiting for the /ATTACHLINK response.";
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      error_message = "Gazebo link attacher failed: " + response->message;
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
      gazebo_block_attached_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Official Gazebo link attacher latched '%s/%s' to '%s/%s'",
      request->model2_name.c_str(),
      request->link2_name.c_str(),
      request->model1_name.c_str(),
      request->link1_name.c_str());
    return true;
#endif
  }

  void detach_pick_object_from_gazebo_link_attacher()
  {
#if !defined(HAVE_LINKATTACHER_MSGS)
    return;
#else
    if (!parameters_.enable_gazebo_attachment || !gazebo_detach_link_client_) {
      return;
    }

    bool attached = false;
    {
      std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
      attached = gazebo_block_attached_;
    }
    if (!attached) {
      return;
    }

    if (!gazebo_detach_link_client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(
        get_logger(),
        "Skipping Gazebo detach because /DETACHLINK is unavailable");
      return;
    }

    auto request = std::make_shared<DetachLink::Request>();
    request->model1_name = parameters_.gazebo_attach_robot_model;
    request->link1_name = parameters_.gazebo_attach_robot_link;
    request->model2_name = parameters_.gazebo_attach_object_model;
    request->link2_name = parameters_.gazebo_attach_object_link;

    auto future = gazebo_detach_link_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Timed out waiting for /DETACHLINK");
      return;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(
        get_logger(),
        "Gazebo link detacher reported failure: %s",
        response->message.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
      gazebo_block_attached_ = false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Official Gazebo link attacher detached '%s/%s' from '%s/%s'",
      request->model2_name.c_str(),
      request->link2_name.c_str(),
      request->model1_name.c_str(),
      request->link1_name.c_str());
#endif
  }

  bool gazebo_link_attached()
  {
#if !defined(HAVE_LINKATTACHER_MSGS)
    return false;
#else
    std::lock_guard<std::mutex> lock(gazebo_attachment_mutex_);
    return gazebo_block_attached_;
#endif
  }

  TaskParameters parameters_;
  std::unique_ptr<TaskFactory> factory_;
  std::unique_ptr<SceneManager> scene_manager_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  std::shared_ptr<rclcpp::AsyncParametersClient> target_fusion_parameter_client_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr gazebo_callback_group_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_direct_trajectory_client_;
#if defined(HAVE_LINKATTACHER_MSGS)
  rclcpp::Client<AttachLink>::SharedPtr gazebo_attach_link_client_;
  rclcpp::Client<DetachLink>::SharedPtr gazebo_detach_link_client_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<GraspTarget>::SharedPtr grasp_target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_valid_subscription_;
  rclcpp::Subscription<GraspTarget>::SharedPtr classification_grasp_target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr classification_target_valid_subscription_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  std::mutex current_task_mutex_;
  std::mutex gazebo_attachment_mutex_;
  std::mutex gripper_hold_mutex_;
  std::mutex vision_target_mutex_;
  std::mutex classification_target_mutex_;
  std::mutex joint_state_mutex_;
  mtc::Task * current_task_{nullptr};
  bool active_goal_{false};
  bool gazebo_block_attached_{false};
  bool have_arm_joint_state_{false};
  bool have_joint7_position_{false};
  bool gripper_hold_active_{false};
  double gripper_hold_target_{0.0};
  double latest_joint7_position_{0.0};
  bool latest_target_valid_received_{false};
  bool latest_target_valid_{false};
  bool latest_classification_target_valid_{false};
  std::optional<GraspTarget> latest_grasp_target_;
  std::optional<GraspTarget> latest_classification_grasp_target_;
  rclcpp::Time latest_arm_joint_state_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_grasp_target_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_classification_grasp_target_received_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace piper_mtc_tasks

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<piper_mtc_tasks::PickPlaceServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
