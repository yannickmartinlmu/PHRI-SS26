#!/usr/bin/env python3
#Start the Kinova camera once, then AprilTags and hand gestures together

#This launch file expects the previously arm_camera_apriltags.launch.py to be present in the brewbot package.

#ros2 launch brewbot arm_camera_gestures.launch.py start_camera:=false
#ros2 topic echo /brewbot/perception/arm_camera/gesture
#ros2 topic echo /brewbot/perception/arm_camera/thumbs_up
#ros2 topic echo /brewbot/perception/arm_camera/gesture_event
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_camera = LaunchConfiguration("start_camera")
    start_apriltags = LaunchConfiguration("start_apriltags")
    start_gestures = LaunchConfiguration("start_gestures")
    robot_ip = LaunchConfiguration("robot_ip")
    max_color_pub_rate = LaunchConfiguration("max_color_pub_rate")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    use_apriltag_rectification = LaunchConfiguration(
        "use_apriltag_rectification"
    )
    rectified_image_topic = LaunchConfiguration(
        "rectified_image_topic"
    )

    target_frame = LaunchConfiguration("target_frame")

    camera = IncludeLaunchDescription(
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

    apriltags = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("brewbot"),
                    "launch",
                    "arm_camera_apriltags.launch.py",
                ]
            )
        ),
        launch_arguments={
            "start_camera": "false",
            "robot_ip": robot_ip,
            "startup_delay": "4.0",
            "use_rectification": use_apriltag_rectification,
            "raw_image_topic": image_topic,
            "rectified_image_topic": rectified_image_topic,
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
                    "arm_camera_gestures.launch.py",
                ]
            )
        ),
        launch_arguments={
            "start_camera": "false",
            "robot_ip": robot_ip,
            "startup_delay": "4.0",
            "image_topic": image_topic,
        }.items(),
        condition=IfCondition(start_gestures),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_apriltags", default_value="true"),
            DeclareLaunchArgument("start_gestures", default_value="true"),
            DeclareLaunchArgument("robot_ip", default_value="10.163.18.200"),
            DeclareLaunchArgument("max_color_pub_rate", default_value="10.0"),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument("target_frame", default_value=""),
            DeclareLaunchArgument(
                "use_apriltag_rectification",
                default_value="false",
                description=(
                "Rectify the RGB image before AprilTag detection."
                ),
            ),
            DeclareLaunchArgument(
                "rectified_image_topic",
                default_value="/arm_camera/image_rect",
            ),
            camera,
            apriltags,
            gestures,
        ]
    )
