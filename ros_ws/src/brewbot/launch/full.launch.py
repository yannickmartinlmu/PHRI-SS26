# The whole stack. Defaults are the on-site case: real arm, fake sensors off the
# lab PC's drivers. sim_arm:=true adds the mock Kinova + elmo_sim for the laptop.
#
# Still manual, on purpose: `trigger` (own terminal) and coffee_machine_actuator.

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
        DeclareLaunchArgument("fake_sensors", default_value="true"),

        IncludeLaunchDescription(
            _source("interaction.launch.py"),
            # Args do not propagate into an include on their own.
            launch_arguments={
                "fake_sensors": LaunchConfiguration("fake_sensors")
            }.items(),
        ),
        IncludeLaunchDescription(
            _source("sim.launch.py"),
            condition=IfCondition(LaunchConfiguration("sim_arm")),
        ),

        Node(package="brewbot", executable="arm_controller", name="arm_controller"),
    ])
