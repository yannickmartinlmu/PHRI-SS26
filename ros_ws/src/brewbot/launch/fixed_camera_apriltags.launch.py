#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_camera = LaunchConfiguration("start_camera")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    config_file = LaunchConfiguration("config_file")

    # Optional: Azure-Kinect-Treiber mitstarten.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("azure_kinect_ros2_driver"),
                    "launch",
                    "k4a_device_launch.py",
                ]
            )
        ),
        condition=IfCondition(start_camera),
    )

    # Eigentliche AprilTag-Erkennung.
    #
    # Der Eingang des Nodes heißt intern image_rect.
    # Für unseren ersten Test verbinden wir ihn aber direkt mit image_raw,
    # weil dieser Weg bereits nachweislich funktioniert.
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        namespace="fixed_camera",
        name="apriltag",
        output="screen",
        parameters=[config_file],
        remappings=[
            ("image_rect", image_topic),
            ("camera_info", camera_info_topic),
            ("detections", "/fixed_camera/detections"),
        ],
    )

    # Unser Wrapper-/Tracker-Node.
    tag_tracker_node = Node(
        package="brewbot",
        executable="fixed_camera_tag_tracker",
        namespace="fixed_camera",
        name="tag_tracker",
        output="screen",
        parameters=[config_file],
    )

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("brewbot"),
            "config",
            "fixed_camera_apriltags.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="false",
                description=(
                    "Azure Kinect ebenfalls starten. "
                    "False verwenden, wenn die Kamera bereits läuft."
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/k4a/rgb/image_raw",
                description="RGB-Bildtopic der fest montierten Kamera.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/k4a/rgb/camera_info",
                description="Passendes CameraInfo-Topic.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Gemeinsame Konfiguration für Detector und Tracker.",
            ),
            camera_launch,
            apriltag_node,
            tag_tracker_node,
        ]
    )