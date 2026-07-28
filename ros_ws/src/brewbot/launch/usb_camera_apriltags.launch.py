#!/usr/bin/env python3
"""Launch AprilTag detection and semantic tracking for the USB camera.

The RealSense camera is intentionally started separately with the fixed command:
    ros2 run realsense2_camera realsense2_camera_node
"""
#ros2 launch brewbot usb_camera_apriltags.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    startup_delay = LaunchConfiguration("startup_delay")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    target_frame = LaunchConfiguration("target_frame")
    config_file = LaunchConfiguration("config_file")

    # apriltag_ros names its image input "image_rect". For this first working
    # version we remap the RealSense raw colour image directly, matching the
    # already tested fixed- and arm-camera pipelines.
    detector = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        namespace="usb_camera",
        name="apriltag",
        output="screen",
        parameters=[config_file],
        remappings=[
            ("image_rect", image_topic),
            ("camera_info", camera_info_topic),
            ("detections", "/usb_camera/detections"),
        ],
    )

    # Reuse the parameterized tracker. Its executable name is historical; the
    # YAML controls the USB-camera topics, semantic names and TF child frames.
    tracker = Node(
        package="brewbot",
        executable="fixed_camera_tag_tracker",
        namespace="usb_camera",
        name="tag_tracker",
        output="screen",
        parameters=[
            config_file,
            {"target_frame": target_frame},
        ],
    )

    perception_nodes = TimerAction(
        period=startup_delay,
        actions=[detector, tracker],
    )

    default_config = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "config", "usb_camera_apriltags.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "startup_delay",
                default_value="1.0",
                description="Seconds before detector and tracker start.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
                description="USB / RealSense colour image topic.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
                description="CameraInfo matching the USB colour stream.",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="",
                description=(
                    "Frame for semantic PoseStamped outputs. Empty uses the "
                    "camera frame from the detection message."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="USB-camera AprilTag and tracker parameters.",
            ),
            perception_nodes,
        ]
    )
