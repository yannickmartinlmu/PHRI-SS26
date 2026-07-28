#!/usr/bin/env python3
"""Launch gesture recognition for the USB / RealSense camera.

The camera is intentionally started separately with:
    ros2 run realsense2_camera realsense2_camera_node
"""
#ros2 launch brewbot usb_camera_gestures.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    startup_delay = LaunchConfiguration("startup_delay")
    image_topic = LaunchConfiguration("image_topic")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")

    recognizer = Node(
        package="brewbot",
        executable="arm_camera_gesture_recognizer",
        namespace="usb_camera",
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
        [FindPackageShare("brewbot"), "config", "usb_camera_gestures.yaml"]
    )
    default_model = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "models", "gesture_recognizer.task"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "startup_delay",
                default_value="1.0",
                description="Seconds before gesture recognition starts.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
                description="USB / RealSense colour image topic.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="USB-camera gesture recognizer parameters.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="MediaPipe gesture_recognizer.task file.",
            ),
            delayed_recognizer,
        ]
    )
