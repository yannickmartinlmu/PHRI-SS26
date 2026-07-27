#!/usr/bin/env python3
#Launch Kinova arm-camera gesture recognition

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
    max_color_pub_rate = LaunchConfiguration("max_color_pub_rate")
    startup_delay = LaunchConfiguration("startup_delay")
    image_topic = LaunchConfiguration("image_topic")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")

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
            "max_color_pub_rate": max_color_pub_rate,
        }.items(),
        condition=IfCondition(start_camera),
    )

    recognizer = Node(
        package="brewbot",
        executable="arm_camera_gesture_recognizer",
        namespace="arm_camera",
        name="gesture_recognizer",
        output="screen",
        parameters=[
            config_file,
            {
                "image_topic": image_topic,
                "model_path": model_path,
            },
        ],
    )

    delayed_recognizer = TimerAction(
        period=startup_delay,
        actions=[recognizer],
    )

    default_config = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "config", "arm_camera_gestures.yaml"]
    )
    default_model = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "models", "gesture_recognizer.task"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description=(
                    "Start kinova_vision. Set false when another launch file "
                    "already owns the camera stream."
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.163.18.200",
                description="IP address of the Kinova robot/vision module.",
            ),
            DeclareLaunchArgument(
                "max_color_pub_rate",
                default_value="10.0",
                description="Maximum Kinova color publication rate in Hz.",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="4.0",
                description="Seconds before starting gesture recognition.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="Kinova arm-camera RGB image topic.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Gesture recognizer parameter file.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="MediaPipe gesture_recognizer.task file.",
            ),
            camera_launch,
            delayed_recognizer,
        ]
    )
