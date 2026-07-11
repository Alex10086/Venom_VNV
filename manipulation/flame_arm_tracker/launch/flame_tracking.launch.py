import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("flame_arm_tracker"),
        "config",
        "flame_tracking.yaml",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Flame tracking parameter YAML.",
    )
    use_yolo_arg = DeclareLaunchArgument(
        "use_yolo",
        default_value="true",
        description="Use YOLO detector instead of HSV color detector.",
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="false",
        description="Automatically enable flame tracking after startup.",
    )
    auto_enable_delay_arg = DeclareLaunchArgument(
        "auto_enable_delay_sec",
        default_value="3.0",
        description="Delay before calling /flame_arm_tracker/set_enabled when auto_enable is true.",
    )
    yolo_enabled_arg = DeclareLaunchArgument(
        "yolo_enabled",
        default_value="true",
        description="Load the flame YOLO model at startup.",
    )

    color_detector_node = Node(
        package="flame_arm_tracker",
        executable="flame_color_detector",
        name="flame_color_detector",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        condition=UnlessCondition(LaunchConfiguration("use_yolo")),
    )

    yolo_detector_node = Node(
        package="flame_arm_tracker",
        executable="flame_yolo_detector",
        name="flame_yolo_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"enabled": ParameterValue(LaunchConfiguration("yolo_enabled"), value_type=bool)},
        ],
        condition=IfCondition(LaunchConfiguration("use_yolo")),
    )

    tracker_node = Node(
        package="flame_arm_tracker",
        executable="flame_arm_tracker",
        name="flame_arm_tracker",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    auto_enable_call = TimerAction(
        period=LaunchConfiguration("auto_enable_delay_sec"),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/flame_arm_tracker/set_enabled",
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("auto_enable")),
            )
        ],
    )

    return LaunchDescription([
        params_arg,
        use_yolo_arg,
        auto_enable_arg,
        auto_enable_delay_arg,
        yolo_enabled_arg,
        color_detector_node,
        yolo_detector_node,
        tracker_node,
        auto_enable_call,
    ])
