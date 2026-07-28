from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

MOVEIT_PKG = "kinova_gen3_6dof_robotiq_2f_85_moveit_config"


def generate_launch_description():
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(MOVEIT_PKG), "launch", "robot.launch.py"])),
        launch_arguments={
            "robot_ip": "192.168.1.10",
            "use_fake_hardware": "true",
        }.items(),
    )
    return LaunchDescription([
        driver,
        Node(package="brewbot", executable="elmo_sim", name="elmo_sim"),
    ])
