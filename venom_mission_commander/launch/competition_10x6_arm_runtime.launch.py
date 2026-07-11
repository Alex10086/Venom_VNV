"""Launch the full no-human-intervention runtime for the 10x6 arm mission."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_port = LaunchConfiguration('can_port')
    use_nav = LaunchConfiguration('use_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_config = LaunchConfiguration('mission_config')
    yolo_device = LaunchConfiguration('yolo_device')
    digit_model_path = LaunchConfiguration('digit_model_path')
    classification_yolo_model_path = LaunchConfiguration('classification_yolo_model_path')

    return LaunchDescription([
        DeclareLaunchArgument('can_port', default_value='can1'),
        DeclareLaunchArgument('use_nav', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'mission_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('venom_mission_commander'),
                'config',
                'competition_10x6_arm_mission.yaml',
            ]),
        ),
        DeclareLaunchArgument('yolo_device', default_value=''),
        DeclareLaunchArgument(
            'digit_model_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'venom_ws',
                'models',
                'yolo',
                'yolo_26_detect_digit.pt',
            ]),
        ),
        DeclareLaunchArgument(
            'classification_yolo_model_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('grasp_target_fusion'),
                'models',
                'box_best.pt',
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('grasp_target_fusion'),
                    'launch',
                    'real_pick_vision.launch.py',
                ])
            ),
            launch_arguments={
                'can_port': can_port,
                'launch_piper_control': 'true',
                'launch_moveit_stack': 'true',
                'launch_yolo_detector': 'true',
                'launch_yolo_bridge': 'true',
                'pick_yolo_enabled': 'false',
                'launch_classification_yolo_detector': 'true',
                'launch_classification_yolo_bridge': 'true',
                'classification_yolo_enabled': 'false',
                'classification_yolo_model_path': classification_yolo_model_path,
                'launch_color_box_detector': 'true',
                'launch_flame_tracking': 'true',
                'flame_use_yolo': 'true',
                'flame_yolo_enabled': 'false',
                'launch_repeat_visual_pick': 'false',
                'target_class': 'black_block',
                'yolo_allowed_classes': 'black_block,golden_block',
                'yolo_min_confidence': '0.7',
            }.items(),
        ),
        Node(
            package='yolo_detector',
            executable='yolo_node',
            name='digit_yolo_detector',
            output='screen',
            parameters=[{
                'model_path': digit_model_path,
                'image_topic': '/camera/d435i/color/image_raw',
                'output_topic': '/perception/digit_detections',
                'annotated_image_topic': '/perception/debug/yolo_result',
                'publish_annotated_image': True,
                'confidence_threshold': 0.25,
                'device': yolo_device,
                'enabled': False,
            }],
        ),
        Node(
            package='printed_number_reader',
            executable='printed_number_reader_node',
            name='printed_number_reader',
            output='screen',
            parameters=[{
                'reader_mode': 'yolo',
                'detections_topic': '/perception/digit_detections',
                'image_topic': '/perception/debug/yolo_result',
                'service_name': '/perception/read_printed_number',
                'expected_digits': 4,
                'min_confidence': 0.25,
                'stable_frames': 1,
                'max_detection_age_sec': 2.0,
                'max_image_age_sec': 2.0,
                'save_success_image': True,
                'success_image_dir': '/tmp/venom_meter_images',
                'success_image_encoding': 'bgr8',
                'success_image_wait_sec': 0.5,
            }],
        ),
        Node(
            package='venom_mission_commander',
            executable='mission_commander',
            name='mission_commander',
            output='screen',
            parameters=[{
                'mission_config': mission_config,
                'use_nav': ParameterValue(use_nav, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'nav2_wait_mode': 'bt_navigator',
                'navigator_ready_timeout_sec': 30.0,
            }],
        ),
    ])
