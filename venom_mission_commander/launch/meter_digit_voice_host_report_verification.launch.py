"""Launch camera, YOLO reader, success-image saving, and commander."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


STABLE_VIDEO_DEVICE = '/dev/v4l/by-id/usb-SunplusIT_Inc_FHD_Webcam_01.00.00-video-index0'  # noqa: E501
SUCCESS_IMAGE_DIR = '/tmp/venom_meter_images'


def generate_launch_description():
    """Generate launch description for integrated verification."""
    mission_config = PathJoinSubstitution([
        FindPackageShare('venom_mission_commander'),
        'config',
        'meter_digit_voice_host_report_verification_mission.yaml',
    ])

    video_device = LaunchConfiguration('video_device')
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    pixel_format = LaunchConfiguration('pixel_format')
    output_encoding = LaunchConfiguration('output_encoding')
    image_size = LaunchConfiguration('image_size')
    camera_log_level = LaunchConfiguration('camera_log_level')

    model_path = LaunchConfiguration('model_path')
    detections_topic = LaunchConfiguration('detections_topic')
    annotated_image_topic = LaunchConfiguration('annotated_image_topic')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    yolo_device = LaunchConfiguration('yolo_device')

    success_image_dir = LaunchConfiguration('success_image_dir')
    success_image_wait_sec = LaunchConfiguration('success_image_wait_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value=STABLE_VIDEO_DEVICE,
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/perception/verification/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/perception/verification/camera_info',
        ),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('output_encoding', default_value='bgr8'),
        DeclareLaunchArgument('image_size', default_value='[640, 480]'),
        DeclareLaunchArgument('camera_log_level', default_value='warn'),
        DeclareLaunchArgument(
            'model_path',
            default_value=(
                '/home/alex/venom_ws/models/yolo/yolo_26_detect_digit.pt'
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
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='meter_voice_host_report_camera',
            output='screen',
            ros_arguments=['--log-level', camera_log_level],
            parameters=[{
                'video_device': video_device,
                'camera_frame_id': camera_frame_id,
                'pixel_format': pixel_format,
                'output_encoding': output_encoding,
                'image_size': ParameterValue(image_size, value_type=List[int]),
            }],
            remappings=[
                ('image_raw', image_topic),
                ('camera_info', camera_info_topic),
            ],
        ),
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
