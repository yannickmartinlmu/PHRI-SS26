# Everything the conversation needs, no arm. sensors:=fake (default) runs on a
# laptop; sensors:=real swaps in the E4 + chest strap on-site; sensors:=none brings
# the dialog stack up with nothing feeding it — for driving the arm by hand.
#
# `trigger` rides along with the real sensors — with a live band its /user_arrived
# and /user_spike paths are the whole point. It loses its keyboard either way
# (stdin belongs to launch), so exclude_trigger:=true when you want to type at it
# from its own terminal.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, EqualsSubstitution, LaunchConfiguration, NotSubstitution,
    PathJoinSubstitution,
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
    sensors = LaunchConfiguration("sensors")
    real = EqualsSubstitution(sensors, "real")
    exclude_trigger = LaunchConfiguration("exclude_trigger")
    return LaunchDescription([
        DeclareLaunchArgument("sensors", default_value="fake",
                              choices=["fake", "real", "none"]),
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
        # Only talks to Home Assistant once a goal arrives, so starting it on a
        # laptop that cannot reach HA costs nothing.
        Node(package="brewbot", executable="coffee_machine_actuator",
             name="coffee_machine_actuator"),
        Node(package="brewbot", executable="state_estimator",     name="state_estimator"),
        Node(package="brewbot", executable="interaction_manager", name="interaction_manager"),
        Node(package="brewbot", executable="trigger",             name="trigger",
             condition=IfCondition(AndSubstitution(
                 real, NotSubstitution(exclude_trigger)))),

        _include("sensor_fakes.launch.py",
                 IfCondition(EqualsSubstitution(sensors, "fake"))),
        _include("sensors.launch.py", IfCondition(real)),
    ])
