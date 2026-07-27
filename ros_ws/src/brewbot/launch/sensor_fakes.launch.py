from launch import LaunchDescription
from launch_ros.actions import Node

from brewbot.sensor_fakes import PRESETS


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="brewbot",
            executable="sensor_fakes",
            name=f"sensor_fake_{name}",
            parameters=[{"sensor": name}],
        )
        for name in PRESETS
    ])
