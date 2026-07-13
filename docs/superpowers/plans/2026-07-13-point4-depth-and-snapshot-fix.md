# Point 4 Depth and Snapshot Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use test-driven development and execute each task in order. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent point 4 classification from selecting distant background depth and remove the invalid standalone TEB snapshot restore.

**Architecture:** Keep the existing fusion and mission architecture. Correct the classification-only depth sampling policy so the near box surface wins over farther background pixels, run color and aligned depth at matching resolution, and make the skip-navigation verification mission self-contained.

**Tech Stack:** ROS 2, Python launch/YAML configuration, pytest.

---

### Task 1: Classification depth regression

**Files:**
- Create: `perception/grasp_target_fusion/test/test_classification_depth_config.py`
- Modify: `perception/grasp_target_fusion/config/grasp_target_fusion.yaml:63`
- Modify: `perception/grasp_target_fusion/launch/real_pick_vision.launch.py:101`

- [ ] **Step 1: Write the failing configuration test**

Load `grasp_target_fusion.yaml`, assert classification uses bbox sampling with percentile `20.0`, and demonstrate with a synthetic ROI containing 25% box pixels at `0.55 m` and 75% background pixels at `1.80 m` that the configured percentile selects the foreground. Parse the launch AST and assert the default depth and color profiles are both `640x480x15`.

- [ ] **Step 2: Run the test and verify RED**

Run: `python3 -m pytest perception/grasp_target_fusion/test/test_classification_depth_config.py -q`

Expected: FAIL because the percentile is `80.0` and color profile is `424x240x15`.

- [ ] **Step 3: Apply the minimal configuration fix**

Set:

```yaml
bbox_depth_sample_percentile: 20.0
```

and:

```python
DeclareLaunchArgument("color_profile", default_value="640x480x15")
```

- [ ] **Step 4: Run the test and verify GREEN**

Run: `python3 -m pytest perception/grasp_target_fusion/test/test_classification_depth_config.py -q`

Expected: PASS.

### Task 2: Standalone point 4 mission

**Files:**
- Modify: `venom_mission_commander/test/test_arm_task_client.py:1558`
- Modify: `venom_mission_commander/config/verify_point4_classify_place.yaml:25`

- [ ] **Step 1: Write the failing mission test**

Change the verification assertion to require only the point-4 perception/classification tasks and explicitly assert no `ros_parameters` restore task exists in the standalone skip-navigation mission.

- [ ] **Step 2: Run the test and verify RED**

Run: `python3 -m pytest venom_mission_commander/test/test_arm_task_client.py::test_verify_point4_classify_place_matches_craic_arm_mission -q`

Expected: FAIL because `restore_teb_after_point_4_approach` is still present.

- [ ] **Step 3: Remove the orphan restore**

Delete the `restore_teb_after_point_4_approach` task from `verify_point4_classify_place.yaml`. Do not add a snapshot creator because this mission skips the navigation segment that temporarily changes TEB.

- [ ] **Step 4: Run the test and verify GREEN**

Run the same targeted pytest command and expect PASS.

### Task 3: Focused verification

**Files:**
- Verify only; no additional changes expected.

- [ ] **Step 1: Run package-level tests**

Run both new/changed targeted tests, then the relevant `venom_mission_commander` configuration tests.

- [ ] **Step 2: Check Python and YAML syntax**

Run `python3 -m compileall` for the changed Python launch/test files and load both changed YAML files with `yaml.safe_load`.

- [ ] **Step 3: Review the final diff**

Confirm only the plan, two configuration changes, one launch default, and regression tests changed; preserve unrelated submodule modifications.
