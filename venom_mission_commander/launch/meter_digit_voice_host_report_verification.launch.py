"""Launch D435i camera, YOLO reader, success-image saving, and commander."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


D435I_COLOR_IMAGE_TOPIC = '/camera/camera/color/image_raw'
SUCCESS_IMAGE_DIR = '/tmp/venom_meter_images'


def generate_launch_description():
    """Generate launch description for integrated verification."""
    mission_config = PathJoinSubstitution([
        FindPackageShare('venom_mission_commander'),
        'config',
        'meter_digit_voice_host_report_verification_mission.yaml',
    ])

    camera_namespace = LaunchConfiguration('camera_namespace')
    camera_name = LaunchConfiguration('camera_name')
    serial_no = LaunchConfiguration('serial_no')
    usb_port_id = LaunchConfiguration('usb_port_id')
    color_profile = LaunchConfiguration('rgb_camera.color_profile')
    color_auto_exposure = LaunchConfiguration('rgb_camera.enable_auto_exposure')
    enable_depth = LaunchConfiguration('enable_depth')
    camera_log_level = LaunchConfiguration('camera_log_level')
    camera_output = LaunchConfiguration('camera_output')
    image_topic = LaunchConfiguration('image_topic')

    model_path = LaunchConfiguration('model_path')
    detections_topic = LaunchConfiguration('detections_topic')
    annotated_image_topic = LaunchConfiguration('annotated_image_topic')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    yolo_device = LaunchConfiguration('yolo_device')

    success_image_dir = LaunchConfiguration('success_image_dir')
    success_image_wait_sec = LaunchConfiguration('success_image_wait_sec')

    realsense_camera = GroupAction(
        forwarding=False,
        launch_configurations={
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'serial_no': serial_no,
            'usb_port_id': usb_port_id,
            'log_level': camera_log_level,
            'output': camera_output,
            'enable_color': 'true',
            'rgb_camera.color_profile': color_profile,
            'rgb_camera.color_format': 'RGB8',
            'rgb_camera.enable_auto_exposure': color_auto_exposure,
            'enable_depth': enable_depth,
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_motion': 'false',
            'enable_rgbd': 'false',
            'pointcloud.enable': 'false',
            'publish_tf': 'false',
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py',
                    ])
                ),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
        ),
        DeclareLaunchArgument(
            'serial_no',
            default_value="''",
        ),
        DeclareLaunchArgument(
            'usb_port_id',
            default_value="''",
        ),
        DeclareLaunchArgument(
            'rgb_camera.color_profile',
            default_value='640x480x15',
        ),
        DeclareLaunchArgument(
            'rgb_camera.enable_auto_exposure',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value=D435I_COLOR_IMAGE_TOPIC,
        ),
        DeclareLaunchArgument('camera_log_level', default_value='warn'),
        DeclareLaunchArgument('camera_output', default_value='screen'),
        DeclareLaunchArgument(
            'model_path',
            default_value=(
                '/home/venom/venom_ws/models/yolo/yolo_26_detect_digit.pt'
            ),
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/perception/verification/digit_detections',
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/perception/verification/digit_yolo_result',
        ),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('yolo_device', default_value='cpu'),
        DeclareLaunchArgument('success_image_dir', default_value=SUCCESS_IMAGE_DIR),
        DeclareLaunchArgument('success_image_wait_sec', default_value='0.5'),
        realsense_camera,
        Node(
            package='yolo_detector',
            executable='yolo_node',
            name='digit_yolo_meter_voice_host_report_verification',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'image_topic': image_topic,
                'output_topic': detections_topic,
                'annotated_image_topic': annotated_image_topic,
                'publish_annotated_image': True,
                'confidence_threshold': ParameterValue(
                    confidence_threshold,
                    value_type=float,
                ),
                'device': yolo_device,
            }],
        ),
        Node(
            package='printed_number_reader',
            executable='printed_number_reader_node',
            name='printed_number_reader_meter_voice_host_report_verification',
            output='screen',
            parameters=[{
                'reader_mode': 'yolo',
                'detections_topic': detections_topic,
                'image_topic': annotated_image_topic,
                'service_name': '/perception/verification/read_printed_number',
                'expected_digits': 4,
                'min_confidence': 0.25,
                'stable_frames': 1,
                'max_detection_age_sec': 2.0,
                'max_image_age_sec': 2.0,
                'save_success_image': True,
                'success_image_dir': success_image_dir,
                'success_image_encoding': 'bgr8',
                'success_image_wait_sec': ParameterValue(
                    success_image_wait_sec,
                    value_type=float,
                ),
            }],
        ),
        Node(
            package='venom_mission_commander',
            executable='mission_commander',
            name='mission_commander_meter_voice_host_report_verification',
            output='screen',
            parameters=[{
                'mission_config': mission_config,
                'use_nav': False,
                'use_sim_time': False,
            }],
        ),
    ])
