"""Launch the meter digit YOLO reader and WAV voice verification chain."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


STABLE_VIDEO_DEVICE = '/dev/v4l/by-id/usb-SunplusIT_Inc_FHD_Webcam_01.00.00-video-index0'  # noqa: E501


def generate_launch_description():
    """Generate launch description for meter digit voice verification."""
    mission_config = PathJoinSubstitution([
        FindPackageShare('venom_mission_commander'),
        'config',
        'meter_digit_voice_verification_mission.yaml',
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
        DeclareLaunchArgument('pixel_format', default_value='MJPG'),
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
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='meter_voice_verification_camera',
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
            name='digit_yolo_meter_voice_verification',
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
            name='printed_number_reader_meter_voice_verification',
            output='screen',
            parameters=[{
                'reader_mode': 'yolo',
                'detections_topic': detections_topic,
                'service_name': '/perception/verification/read_printed_number',
                'expected_digits': 4,
                'min_confidence': 0.25,
                'stable_frames': 1,
                'max_detection_age_sec': 2.0,
            }],
        ),
        Node(
            package='venom_mission_commander',
            executable='mission_commander',
            name='mission_commander_meter_voice_verification',
            output='screen',
            parameters=[{
                'mission_config': mission_config,
                'use_nav': False,
                'use_sim_time': False,
            }],
        ),
    ])
