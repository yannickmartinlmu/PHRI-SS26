#!/usr/bin/env python3
"""
 Fake sensor publisher — no hardware, no bluetooth
 If you want specific values sent forever, use:
    ros2 topic pub -r1 /heartrate std_msgs/msg/Int32 "{data: 110}"
    ros2 topic pub -r1 /skin_temp std_msgs/msg/Float32 "{data: 36.0}"
"""
import itertools
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

PRESETS = {
    "hr":        ("/heartrate", Int32,   [60, 72, 85, 110, 72]),
    "hrv":       ("/hrv",       Float32, [55.0, 48.0, 42.0, 50.0]),
    "eda":       ("/eda",       Float32, [0.5, 1.2, 3.0, 0.8]),
    "skin_temp": ("/skin_temp", Float32, [33.0, 34.5, 36.0, 31.0]),
    "motion":    ("/motion",    Float32, [0.1, 0.3, 2.0, 0.2]),
}


class FakeSensorNode(Node):

    def __init__(self):
        super().__init__("sensor_fakes")

        name = self.declare_parameter("sensor", "hr").value
        if name not in PRESETS:
            raise ValueError(f"unknown sensor '{name}', pick one of {list(PRESETS)}")

        topic, msg_type, values = PRESETS[name]
        self._msg_type = msg_type
        self._values = itertools.cycle(values)
        self._pub = self.create_publisher(msg_type, topic, 10)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(f"Fake sensor '{name}' publishing to {topic}")

    def _tick(self):
        msg = self._msg_type()
        msg.data = next(self._values)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = FakeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
