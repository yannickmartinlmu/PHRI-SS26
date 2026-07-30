# Everything the conversation needs, no arm. Sensors are fake by default so this
# runs on a laptop; fake_sensors:=false swaps in the real E4 + chest strap on-site.
#
# `trigger` is deliberately absent — run it by hand in its own terminal, it is the
# thing you type into.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(name, condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("brewbot"), "launch", name])),
        condition=condition,
    )


def generate_launch_description():
    fake = LaunchConfiguration("fake_sensors")
    return LaunchDescription([
        DeclareLaunchArgument("fake_sensors", default_value="true"),

        Node(package="brewbot", executable="asr_vosk",            name="asr_vosk"),
        Node(package="brewbot", executable="llm",                 name="llm"),
        Node(package="brewbot", executable="tts",                 name="tts"),
        Node(package="brewbot", executable="light_actuator",      name="light_actuator"),
        Node(package="brewbot", executable="state_estimator",     name="state_estimator"),
        Node(package="brewbot", executable="interaction_manager", name="interaction_manager"),

        _include("sensor_fakes.launch.py", IfCondition(fake)),
        _include("sensors.launch.py", UnlessCondition(fake)),
    ])
