#!/usr/bin/env python3
"""Start USB-camera AprilTags and gestures together.

This launch does not start the RealSense node. Start the camera separately with:
    ros2 run realsense2_camera realsense2_camera_node
"""
#ros2 launch brewbot usb_camera_perception.launch.py
#ros2 topic echo /usb_camera/detections | grep --line-buffered -E 'family:|id:|hamming:|decision_margin:'

#ros2 topic echo /brewbot/perception/usb_camera/visible_ids
#ros2 topic echo /brewbot/perception/usb_camera/Ntag_7/visible
#ros2 topic echo /brewbot/perception/usb_camera/Ntag_7/pose

#ros2 topic echo /brewbot/perception/usb_camera/gesture
#ros2 topic echo /brewbot/perception/usb_camera/gesture_event
#ros2 topic echo /brewbot/perception/usb_camera/open_palm
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_apriltags = LaunchConfiguration("start_apriltags")
    start_gestures = LaunchConfiguration("start_gestures")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    target_frame = LaunchConfiguration("target_frame")

    apriltags = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("brewbot"),
                    "launch",
                    "usb_camera_apriltags.launch.py",
                ]
            )
        ),
        launch_arguments={
            "startup_delay": "1.0",
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "target_frame": target_frame,
        }.items(),
        condition=IfCondition(start_apriltags),
    )

    gestures = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("brewbot"),
                    "launch",
                    "usb_camera_gestures.launch.py",
                ]
            )
        ),
        launch_arguments={
            "startup_delay": "1.0",
            "image_topic": image_topic,
        }.items(),
        condition=IfCondition(start_gestures),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_apriltags", default_value="true"),
            DeclareLaunchArgument("start_gestures", default_value="true"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
            ),
            DeclareLaunchArgument("target_frame", default_value=""),
            apriltags,
            gestures,
        ]
    )
