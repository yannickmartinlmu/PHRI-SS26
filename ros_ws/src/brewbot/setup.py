from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'brewbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yannick',
    maintainer_email='martiyan7@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'asr_vosk = brewbot.asr_vosk:main',
            'asr_whisper = brewbot.asr_whisper:main',
            'web_ui = brewbot.web_ui:main',
            'nlp = brewbot.nlp:main',
            'sensor_hr = brewbot.sensor_hr:main',
            'interaction_manager = brewbot.interaction_manager:main',
            'tts = brewbot.tts:main',
            'state_estimator = brewbot.state_estimator:main',
            'arm_controller = brewbot.arm_controller:main',
            'elmo_sim = brewbot.elmo_sim:main',
            (
                "fixed_camera_tag_tracker = "
                "brewbot.fixed_camera_tag_tracker:main"
            ),
        ],
    },
)
