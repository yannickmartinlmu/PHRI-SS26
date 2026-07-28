#!/usr/bin/env python3
#ros2 launch brewbot arm_camera_apriltags.launch.py start_camera:=false
#ros2 topic echo /arm_camera/detections | grep --line-buffered -E 'family:|id:|hamming:|decision_margin:'
#ros2 topic echo /brewbot/perception/arm_camera/visible_ids
#ros2 topic echo /brewbot/perception/arm_camera/Ntag_7/visible
#ros2 topic echo /brewbot/perception/arm_camera/Ntag_7/pose
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
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    start_camera = LaunchConfiguration("start_camera")
    robot_ip = LaunchConfiguration("robot_ip")
    max_color_pub_rate = LaunchConfiguration("max_color_pub_rate")
    startup_delay = LaunchConfiguration("startup_delay")

    use_rectification = LaunchConfiguration("use_rectification")
    raw_image_topic = LaunchConfiguration("raw_image_topic")
    rectified_image_topic = LaunchConfiguration("rectified_image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    rectify_queue_size = LaunchConfiguration("rectify_queue_size")

    target_frame = LaunchConfiguration("target_frame")
    config_file = LaunchConfiguration("config_file")

    # Bei true verwendet apriltag_ros das entzerrte Bild.
    # Bei false wird direkt das Rohbild verwendet.
    apriltag_input_topic = IfElseSubstitution(
        use_rectification,
        rectified_image_topic,
        raw_image_topic,
    )

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("brewbot"),
            "config",
            "arm_camera_apriltags.yaml",
        ]
    )

    # Optionaler Start der Kinova-Kamera.
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

    # Wird nur bei use_rectification:=true gestartet.
    rectify_node = Node(
        package="image_proc",
        executable="rectify_node",
        namespace="arm_camera",
        name="rectify",
        output="screen",
        parameters=[
            {
                "image_transport": "raw",
                "queue_size": ParameterValue(
                    rectify_queue_size,
                    value_type=int,
                ),
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
        namespace="arm_camera",
        name="apriltag",
        output="screen",
        parameters=[config_file],
        remappings=[
            # Der interne Eingang heißt immer image_rect.
            # Das tatsächliche Eingangstopic wird oben ausgewählt.
            ("image_rect", apriltag_input_topic),
            ("camera_info", camera_info_topic),
            ("detections", "/arm_camera/detections"),
        ],
    )

    tag_tracker_node = Node(
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

    # Gibt der Kinova-Kamera Zeit zum Starten.
    perception_nodes = TimerAction(
        period=startup_delay,
        actions=[
            rectify_node,
            apriltag_node,
            tag_tracker_node,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description=(
                    "Start kinova_vision. Set false when the camera "
                    "is already running."
                ),
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="10.163.18.200",
                description="IP address of the Kinova vision module.",
            ),
            DeclareLaunchArgument(
                "max_color_pub_rate",
                default_value="10.0",
                description="Maximum RGB publication rate in Hz.",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="4.0",
                description=(
                    "Seconds before starting the perception nodes."
                ),
            ),
            DeclareLaunchArgument(
                "use_rectification",
                default_value="false",
                description=(
                    "Start image_proc and use its rectified image "
                    "for AprilTag detection."
                ),
            ),
            DeclareLaunchArgument(
                "raw_image_topic",
                default_value="/camera/color/image_raw",
                description="Raw Kinova RGB image topic.",
            ),
            DeclareLaunchArgument(
                "rectified_image_topic",
                default_value="/arm_camera/image_rect",
                description="Rectified image generated by image_proc.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/color/camera_info",
                description="CameraInfo matching the RGB stream.",
            ),
            DeclareLaunchArgument(
                "rectify_queue_size",
                default_value="5",
                description=(
                    "Synchronization queue size used by image_proc."
                ),
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="",
                description=(
                    "Frame for wrapper PoseStamped outputs. "
                    "Empty uses the camera image frame."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="AprilTag detector and tracker configuration.",
            ),
            camera_launch,
            perception_nodes,
        ]
    )