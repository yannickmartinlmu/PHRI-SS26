# Everything the conversation needs, no arm. Sensors are fake by default so this
# runs on a laptop; fake_sensors:=false swaps in the real E4 + chest strap on-site.
#
# `trigger` rides along with the real sensors — with a live band its /user_arrived
# and /user_spike paths are the whole point. It loses its keyboard either way
# (stdin belongs to launch), so exclude_trigger:=true when you want to type at it
# from its own terminal.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution,
)
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
    exclude_trigger = LaunchConfiguration("exclude_trigger")
    return LaunchDescription([
        DeclareLaunchArgument("fake_sensors", default_value="true"),
        DeclareLaunchArgument("exclude_trigger", default_value="false"),
        # Default is the lab PC; llm_host:=http://localhost:11434 for a laptop
        # running its own ollama. No trailing slash — ollama 307s on `//api/...`
        # and urllib will not follow a 307 on POST.
        DeclareLaunchArgument("llm_host", default_value="http://10.163.18.109:11434"),

        Node(package="brewbot", executable="asr_vosk",            name="asr_vosk"),
        Node(package="brewbot", executable="llm",                 name="llm",
             parameters=[{"host": LaunchConfiguration("llm_host")}]),
        Node(package="brewbot", executable="tts",                 name="tts"),
        Node(package="brewbot", executable="light_actuator",      name="light_actuator"),
        Node(package="brewbot", executable="state_estimator",     name="state_estimator"),
        Node(package="brewbot", executable="interaction_manager", name="interaction_manager"),
        Node(package="brewbot", executable="trigger",             name="trigger",
             condition=IfCondition(AndSubstitution(
                 NotSubstitution(fake), NotSubstitution(exclude_trigger)))),

        _include("sensor_fakes.launch.py", IfCondition(fake)),
        _include("sensors.launch.py", UnlessCondition(fake)),
    ])
