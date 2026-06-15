from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="brewbot", executable="asr_vosk",            name="asr_vosk"),
        Node(package="brewbot", executable="nlp",                 name="nlp"),
        Node(package="brewbot", executable="tts",                 name="tts"),
        Node(package="brewbot", executable="web_ui",              name="web_ui"),
        Node(package="brewbot", executable="sensor_hr",           name="sensor_hr"),
        Node(package="brewbot", executable="state_estimator",     name="state_estimator"),
        Node(package="brewbot", executable="interaction_manager", name="interaction_manager"),
    ])
