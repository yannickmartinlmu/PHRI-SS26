#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty

# Same interface strings the arm_controller drives (ELMO_SET / ELMO_GET there).
ELMO_GET = "/elmo/id1/{axis}/position/get"
ELMO_SET = "/elmo/id1/{axis}/position/set"
ELMO_STOP = "/elmo/id1/{axis}/stop"

PUBLISH_HZ = 20.0

AXES = {
    #            start,  speed (m/s)
    "carriage": (0.85, 0.20),
    "lift": (0.35, 0.10),
}


class ElmoSim(Node):

    def __init__(self):
        super().__init__("elmo_sim")
        self._pos = {}
        self._target = {}
        self._speed = {}
        self._pub = {}
        for axis, (start, speed) in AXES.items():
            self._pos[axis] = start
            self._target[axis] = start
            self._speed[axis] = speed
            self._pub[axis] = self.create_publisher(
                Float32, ELMO_GET.format(axis=axis), 10)
            self.create_subscription(
                Float32, ELMO_SET.format(axis=axis),
                lambda msg, a=axis: self._on_set(a, msg), 10)
            self.create_subscription(
                Empty, ELMO_STOP.format(axis=axis),
                lambda msg, a=axis: self._on_stop(a), 10)

        self._dt = 1.0 / PUBLISH_HZ
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(f"Elmo sim up — axes {list(AXES)} @ {PUBLISH_HZ} Hz")

    def _on_set(self, axis, msg):
        self._target[axis] = msg.data
        self.get_logger().info(f"[{axis}] target -> {msg.data}")

    def _on_stop(self, axis):
        self._target[axis] = self._pos[axis]   # freeze where we are
        self.get_logger().info(f"[{axis}] stop @ {self._pos[axis]:.3f}")

    def _tick(self):
        for axis in self._pos:
            step = self._speed[axis] * self._dt
            remaining = self._target[axis] - self._pos[axis]
            if abs(remaining) <= step:
                self._pos[axis] = self._target[axis]   # arrive, never overshoot
            else:
                self._pos[axis] += step if remaining > 0 else -step
            self._pub[axis].publish(Float32(data=float(self._pos[axis])))


def main():
    rclpy.init()
    node = ElmoSim()
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
