# web_ui drives arm_controller's bring_drink action directly — arm testing without
# speaking to it. Pair with sim.launch.py on a laptop.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="brewbot", executable="web_ui",         name="web_ui"),
        Node(package="brewbot", executable="arm_controller", name="arm_controller"),
    ])
