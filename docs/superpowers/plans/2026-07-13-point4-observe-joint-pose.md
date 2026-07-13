# Point 4 Observe Joint Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use test-driven development and execute each task in order. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the Piper arm to the recorded point-4 camera observation joint pose before locating classification boxes, without changing the global visual-pick observe pose.

**Architecture:** Add an optional six-value vector under `classification_place`. Load it through the existing task-parameter API and execute it with the existing direct six-axis trajectory helper immediately before the box-target loop. An empty vector preserves existing behavior.

**Tech Stack:** ROS 2 Humble, C++17, rclcpp, FollowJointTrajectory, ament_cmake_gtest, YAML.

---

### Task 1: Parameter regression

**Files:**
- Create: `manipulation/piper_mtc_tasks/test/test_task_parameters.cpp`
- Modify: `manipulation/piper_mtc_tasks/CMakeLists.txt`
- Modify: `manipulation/piper_mtc_tasks/include/piper_mtc_tasks/task_parameters.hpp`
- Modify: `manipulation/piper_mtc_tasks/src/task_factory.cpp`

- [ ] Write a GTest that declares task parameters, sets `classification_place.observe_joint_positions` to six recorded values, loads `TaskParameters`, and asserts exact order and values.
- [ ] Build/run the test and confirm it fails because the parameter and struct field do not exist.
- [ ] Add `std::vector<double> observe_joint_positions` to `ClassificationPlaceConfig`, declare an empty default parameter, and load it with `as_double_array()`.
- [ ] Rebuild/run the test and confirm it passes.

### Task 2: Classification execution

**Files:**
- Modify: `manipulation/piper_mtc_tasks/src/pick_place_server.cpp:4095`
- Modify: `manipulation/piper_mtc_tasks/config/real_pick_task.yaml:178`

- [ ] Add a source/config regression assertion that the classification path calls `execute_direct_arm_joint_target` before the box-target loop and the real config contains exactly six recorded values.
- [ ] Run it and confirm failure before implementation.
- [ ] Before locating box classes, execute the optional joint target using the existing classification direct-joint duration and tolerance; fail the action if the move fails.
- [ ] Configure `[0.002599156, 1.108618532, -0.067578056, -0.046627812, -0.957291832, 0.094616256]` in `real_pick_task.yaml` only; leave the global `observe_pose` and vision-test class contract unchanged.
- [ ] Run focused tests and confirm they pass.

### Task 3: Build verification

**Files:**
- Verify only.

- [ ] Run `colcon build --packages-select piper_mtc_tasks --symlink-install`.
- [ ] Run the focused GTest and package tests.
- [ ] Confirm installed YAML contains the six recorded values and `git diff --check` succeeds.
