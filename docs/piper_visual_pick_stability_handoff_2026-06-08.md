# Piper Visual Pick Stability Handoff

Date: 2026-06-08
Workspace: `~/venom_ws/src/venom_vnv`
Working branch: `arm-multi-yolo-pipeline`

## User Goal

The user asked for a health check on the Piper manipulation tasks, especially whether the arm can still plan feasible solutions reliably for the current visual pick pipeline.

## Scope Of This Slice

This slice only touched the Piper visual pick stability path:

- `perception/grasp_target_fusion`
- `manipulation/piper_mtc_tasks`

It does not include unrelated dirty changes already present in the workspace.

## What Was Investigated

The following areas were reviewed:

- visual pick launch wiring
- grasp target fusion timing and target filtering
- `pick_place_server` visual target gating
- reachability and IK offline probes
- current real-pick parameter defaults

Offline checks that were run:

- `task_pose_reachability_probe.launch.py`
- `ik_frame_sweep.launch.py`
- targeted package build of:
  - `grasp_target_fusion`
  - `piper_mtc_tasks`

## Main Findings Before The Fix

### 1. Single-target gating was not enforced end-to-end

`vision_target.require_single_target` existed in task config, but the manipulation server only checked freshness, confidence, class, frame, and workspace bounds.

This meant multi-object scenes could still proceed as long as fusion published one "best" target.

### 2. Eye-in-hand target timing was too loose

The fusion node could use detection stamps, but TF lookup still used the latest transform instead of the detection time.

For a moving arm-mounted camera, this increases target drift risk after moving to the observe pose.

### 3. The default IK sweep was misaligned with the real setup

The default sweep launch/config was still biased toward the older non-prefixed frame setup and old scene assumptions.

As a result, the default probe could report all-fail results that did not reflect the current Piper visual pick stack.

### 4. Real visual pick pregrasp margin was too tight

The real pick config had:

- `require_valid_signal: false`
- `pregrasp_distance: 0.0`

This left very little geometric slack for noisy depth or minor object-height error.

## Code Changes Made

### A. Time-aligned grasp target publishing

File:

- `perception/grasp_target_fusion/grasp_target_fusion/grasp_target_fusion_node.py`

Changes:

- use the detection timestamp for TF lookup
- stamp projected camera points with the same effective time
- publish `GraspTarget.header.stamp` using the same target time

Effect:

- reduces eye-in-hand pose drift caused by querying TF "now" instead of at detection time

### B. Stronger single-target gating defaults in fusion config

File:

- `perception/grasp_target_fusion/config/grasp_target_fusion.yaml`

Changes:

- set `use_detection_header_stamp: true`
- set `require_single_target: true` for pick pipelines
- keep classification pipeline multi-target capable

Effect:

- pick target fusion now rejects ambiguous multi-target frames by default

### C. Real pick task now requires valid-signal gating and a small pregrasp buffer

File:

- `manipulation/piper_mtc_tasks/config/real_pick_task.yaml`

Changes:

- set `vision_target.require_valid_signal: true`
- set `vision_target.pregrasp_distance: 0.03`

Effect:

- manipulation no longer accepts fusion output unless fusion explicitly marks it usable
- adds a modest approach buffer before direct grasp

### D. Manipulation server now treats validity signal as required for single-target gating

File:

- `manipulation/piper_mtc_tasks/src/pick_place_server.cpp`

Changes:

- track whether `/perception/pick/target_valid` has ever been received
- reject visual pick execution if:
  - a validity signal has not been received yet
  - single-target gating is enabled and fusion marks the current target invalid

Effect:

- closes the gap where config said "require a single target" but the server could still proceed without honoring fusion validity

### E. IK sweep aligned to prefixed real Piper frames

Files:

- `manipulation/piper_mtc_tasks/src/ik_frame_sweep.cpp`
- `manipulation/piper_mtc_tasks/launch/ik_frame_sweep.launch.py`
- `manipulation/piper_mtc_tasks/config/ik_frame_sweep.yaml`
- `manipulation/piper_mtc_tasks/config/ik_frame_sweep_sim_can.yaml`

Changes:

- add explicit `planning_frame`
- switch hand frame defaults to `piper_gripper_grasp_center`
- load MoveIt configs with `link_prefix`
- place collision objects in the configured planning frame
- make the default sweep launch use `ik_frame_sweep_sim_can.yaml`

Effect:

- default IK sweep now reflects the current prefixed Piper stack instead of producing misleading false negatives

## Validation Results

### Python / Launch Sanity

Passed:

- `python3 -m py_compile perception/grasp_target_fusion/grasp_target_fusion/grasp_target_fusion_node.py`
- `python3 -m py_compile manipulation/piper_mtc_tasks/launch/ik_frame_sweep.launch.py`
- `python3 -m py_compile perception/grasp_target_fusion/launch/real_pick_vision.launch.py`

### Targeted Build

Passed:

```bash
source /opt/ros/humble/setup.bash
source ~/venom_ws/install/setup.bash
colcon build --packages-select grasp_target_fusion piper_mtc_tasks \
  --build-base /tmp/colcon_build_check \
  --install-base /tmp/colcon_install_check
```

### Offline Reachability Probe

Passed:

- `task_pose_reachability_probe`: `10 cases, 0 failures`

Interpretation:

- key observe / platform grasp / lift / release-retreat poses remain reachable

### Offline IK Sweep

After alignment fixes, the default sweep now reports:

- `Found 20 valid candidates`

Interpretation:

- the default sweep is no longer falsely reporting zero feasible grasp candidates for the current prefixed setup

## What Was Not Fully Verified

Full Gazebo end-to-end acceptance was not completed in this environment.

Observed blocker:

- `gzserver` exits under the current sandbox because it cannot access local interface information

Therefore:

- offline reachability and build checks are trustworthy
- full simulated action execution still needs to be rerun in a normal workstation environment

## Recommended Next Steps

1. Run `real_pick_vision.launch.py` on the real robot and confirm the visual target no longer flickers between multiple detections.
2. Watch `/perception/pick/target_valid` while moving to the observe pose and verify it drops on ambiguous scenes.
3. If real depth noise still causes occasional grasp misses, expand direct visual pick fallback to try a small pitch candidate set in addition to yaw offsets.
4. After hardware confirmation, consider updating docs or README examples to mention the stronger validity gating requirement.
