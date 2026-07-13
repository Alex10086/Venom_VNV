import ast
from pathlib import Path

import numpy as np
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_DIR / "config" / "grasp_target_fusion.yaml"
LAUNCH_PATH = PACKAGE_DIR / "launch" / "real_pick_vision.launch.py"


def launch_argument_defaults():
    tree = ast.parse(LAUNCH_PATH.read_text(encoding="utf-8"))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                defaults[node.args[0].value] = keyword.value.value
    return defaults


def test_classification_bbox_depth_uses_foreground_percentile():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    parameters = config["classify_grasp_target_fusion"]["ros__parameters"]

    assert parameters["use_bbox_depth_sample"] is True
    assert parameters["bbox_depth_sample_percentile"] == 20.0

    roi_depths = [0.55] * 25 + [1.80] * 75
    sampled_depth = np.percentile(
        roi_depths,
        parameters["bbox_depth_sample_percentile"],
    )
    assert sampled_depth == 0.55


def test_classification_target_loss_and_ray_depth_settings():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    parameters = config["classify_grasp_target_fusion"]["ros__parameters"]

    assert parameters["target_loss_grace_sec"] == 0.75
    assert parameters["target_ray_depth_offset_m"] == 0.0
    assert parameters["subscribe_single_detection_topic"] is False


def test_camera_profiles_default_to_aligned_resolution():
    defaults = launch_argument_defaults()

    assert defaults["depth_profile"] == "640x480x15"
    assert defaults["color_profile"] == "640x480x15"
