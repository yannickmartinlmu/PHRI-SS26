#!/usr/bin/env python3
#ros2 launch brewbot fixed_camera_apriltags.launch.py start_camera:=false
#ros2 topic echo /brewbot/perception/fixed_camera/visible_ids | grep --line-buffered -E 'family:|id:|hamming:|decision_margin:'
#ros2 topic echo /brewbot/perception/fixed_camera/visible_ids
#ros2 topic echo /brewbot/perception/fixed_camera/Ntag_7/visible
#ros2 topic echo /brewbot/perception/fixed_camera/Ntag_7/pose
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_camera = LaunchConfiguration("start_camera")
    use_rectification = LaunchConfiguration("use_rectification")

    raw_image_topic = LaunchConfiguration("raw_image_topic")
    rectified_image_topic = LaunchConfiguration("rectified_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    config_file = LaunchConfiguration("config_file")
    rectify_queue_size = LaunchConfiguration("rectify_queue_size")

    # Wenn use_rectification=true:
    #     apriltag_ros bekommt image_rect.
    #
    # Wenn use_rectification=false:
    #     apriltag_ros bekommt direkt image_raw.
    apriltag_input_topic = IfElseSubstitution(
        use_rectification,
        if_value=rectified_image_topic,
        else_value=raw_image_topic,
    )

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("brewbot"),
            "config",
            "fixed_camera_apriltags.yaml",
        ]
    )

    # Optionaler Start des Azure-Kinect-Treibers.
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

    # Wird nur gestartet, wenn use_rectification=true ist.
    rectify_node = Node(
        package="image_proc",
        executable="rectify_node",
        namespace="fixed_camera",
        name="rectify",
        output="screen",
        parameters=[
            {
                "image_transport": "raw",
                "queue_size": rectify_queue_size,
            }
        ],
        remappings=[
            ("image", raw_image_topic),
            ("camera_info", camera_info_topic),
            ("image_rect", rectified_image_topic),
        ],
        condition=IfCondition(use_rectification),
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        namespace="fixed_camera",
        name="apriltag",
        output="screen",
        parameters=[config_file],
        remappings=[
            # Der interne Eingang heißt immer image_rect.
            # Das tatsächliche Topic wird oben ausgewählt.
            ("image_rect", apriltag_input_topic),
            ("camera_info", camera_info_topic),
            ("detections", "/fixed_camera/detections"),
        ],
    )

    tag_tracker_node = Node(
        package="brewbot",
        executable="fixed_camera_tag_tracker",
        namespace="fixed_camera",
        name="tag_tracker",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="false",
                description="Start the Azure Kinect driver.",
            ),
            DeclareLaunchArgument(
                "use_rectification",
                default_value="false",
                description=(
                    "Start image_proc and use the rectified image "
                    "for AprilTag detection."
                ),
            ),
            DeclareLaunchArgument(
                "raw_image_topic",
                default_value="/k4a/rgb/image_raw",
                description="Unrectified RGB image topic.",
            ),
            DeclareLaunchArgument(
                "rectified_image_topic",
                default_value="/fixed_camera/image_rect",
                description="Rectified image produced by image_proc.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/k4a/rgb/camera_info",
                description="Camera calibration topic.",
            ),
            DeclareLaunchArgument(
                "rectify_queue_size",
                default_value="5",
                description=(
                    "Queue used to synchronize image and CameraInfo "
                    "inside image_proc."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="AprilTag and tracker configuration.",
            ),
            camera_launch,
            rectify_node,
            apriltag_node,
            tag_tracker_node,
        ]
    )