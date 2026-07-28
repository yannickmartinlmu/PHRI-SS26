# Real sensors: the chest strap owns /heartrate, the E4 does everything else.
# The E4's BVP-derived HR is too imprecise to trust, so it is switched off here.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="brewbot", executable="sensor_hr", name="sensor_hr"),
        Node(
            package="brewbot",
            executable="sensor_e4",
            name="sensor_e4",
            parameters=[{"publish_hr": False}],
        ),
    ])
