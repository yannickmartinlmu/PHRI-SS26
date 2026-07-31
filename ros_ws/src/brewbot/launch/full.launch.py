# The whole stack. Defaults are the on-site case: real arm, fake sensors off the
# lab PC's drivers. sim_arm:=true adds the mock Kinova + elmo_sim for the laptop.
#
# `trigger` comes up with the real sensors (see interaction.launch.py);
# exclude_trigger:=true to keep it in its own terminal.
#
# arm:=false sensors:=none is the by-hand case: the whole dialog stack stays up so
# the arm can call /ask_for_water and friends, but nothing drives it — run
# arm_controller yourself and poke it directly.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _source(name):
    return PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("brewbot"), "launch", name]))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_arm", default_value="false"),
        DeclareLaunchArgument("arm", default_value="true"),
        DeclareLaunchArgument("sensors", default_value="fake",
                              choices=["fake", "real", "none"]),
        DeclareLaunchArgument("exclude_trigger", default_value="false"),
        DeclareLaunchArgument("llm_host", default_value="http://10.163.18.109:11434"),

        IncludeLaunchDescription(
            _source("interaction.launch.py"),
            # Args do not propagate into an include on their own.
            launch_arguments={
                "sensors": LaunchConfiguration("sensors"),
                "exclude_trigger": LaunchConfiguration("exclude_trigger"),
                "llm_host": LaunchConfiguration("llm_host"),
            }.items(),
        ),
        IncludeLaunchDescription(
            _source("sim.launch.py"),
            condition=IfCondition(LaunchConfiguration("sim_arm")),
        ),

        Node(package="brewbot", executable="arm_controller", name="arm_controller",
             condition=IfCondition(LaunchConfiguration("arm"))),
    ])
