#!/usr/bin/env python3
"""Launch gesture + person recognition for the fixed kitchen camera (Azure Kinect).

The camera is started separately, as with the other two:
    ros2 launch azure_kinect_ros_driver driver.launch.py
"""
#ros2 launch brewbot kitchen_camera_gestures.launch.py
#ros2 topic echo /kitchen_camera/person_present
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
    person_model_path = LaunchConfiguration("person_model_path")

    # The namespace is what separates this camera's person_present from the PC
    # webcam's — the node publishes that topic under a relative name.
    recognizer = Node(
        package="brewbot",
        executable="arm_camera_gesture_recognizer",
        namespace="kitchen_camera",
        name="gesture_recognizer",
        output="screen",
        parameters=[
            config_file,
            {
                "image_topic": image_topic,
                "model_path": model_path,
                "person_model_path": person_model_path,
            },
        ],
    )

    delayed_recognizer = TimerAction(
        period=startup_delay,
        actions=[recognizer],
    )

    default_config = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "config", "kitchen_camera_gestures.yaml"]
    )
    default_model = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "models", "gesture_recognizer.task"]
    )
    default_person_model = PathJoinSubstitution(
        [FindPackageShare("brewbot"), "models", "efficientdet_lite0.tflite"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "startup_delay",
                default_value="1.0",
                description="Seconds before recognition starts.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/k4a/rgb/image_raw",
                description="Kinect colour image topic.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Kitchen-camera recognizer parameters.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="MediaPipe gesture_recognizer.task file.",
            ),
            DeclareLaunchArgument(
                "person_model_path",
                default_value=default_person_model,
                description="MediaPipe efficientdet_lite0.tflite file.",
            ),
            delayed_recognizer,
        ]
    )
