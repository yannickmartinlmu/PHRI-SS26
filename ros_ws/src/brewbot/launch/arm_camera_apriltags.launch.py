#!/usr/bin/env python3
"""Launch Kinova arm-camera AprilTag detection and the Brewbot tracker."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_camera = LaunchConfiguration("start_camera")
    robot_ip = LaunchConfiguration("robot_ip")
    startup_delay = LaunchConfiguration("startup_delay")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    target_frame = LaunchConfiguration("target_frame")
    config_file = LaunchConfiguration("config_file")

    # Kinova's launch publishes the color stream under /camera/color by
    # default. Depth is disabled because AprilTag detection only needs RGB.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kinova_vision"),
                    "launch",
                    "kinova_vision.launch.py",
                ]
            )
        ),
        launch_arguments={
            "device": robot_ip,
            "launch_color": "true",
            "launch_depth": "false",
        }.items(),
        condition=IfCondition(start_camera),
    )

    # For the first working version, use the raw color image directly, just
    # as in the currently working fixed-camera pipeline. The apriltag_ros
    # subscription is named image_rect, hence the remapping below.
    detector = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        namespace="arm_camera",
        name="apriltag",
        output="screen",
        parameters=[config_file],
        remappings=[
            ("image_rect", image_topic),
            ("camera_info", camera_info_topic),
            ("detections", "/arm_camera/detections"),
        ],
    )

    # Reuse the already implemented, parameterized tracker executable.
    # Its executable name still says fixed_camera, but its topics, tag frames
    # and output namespace are entirely controlled by this YAML file.
    tracker = Node(
        package="brewbot",
        executable="fixed_camera_tag_tracker",
        namespace="arm_camera",
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
        [
            FindPackageShare("brewbot"),
            "config",
            "arm_camera_apriltags.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description=(
                    "Start kinova_vision. Set false when the camera stream "
                    "is already running."
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.163.18.200",
                description="IP address of the Kinova robot/vision module.",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="4.0",
                description=(
                    "Seconds before starting detector and tracker. The "
                    "Kinova camera sometimes needs time to connect."
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="Kinova arm-camera RGB image topic.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/color/camera_info",
                description="CameraInfo matching the Kinova RGB stream.",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="",
                description=(
                    "Frame for wrapper PoseStamped outputs. Empty uses the "
                    "image frame, normally camera_color_frame."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Arm-camera detector and tracker parameters.",
            ),
            camera_launch,
            perception_nodes,
        ]
    )
