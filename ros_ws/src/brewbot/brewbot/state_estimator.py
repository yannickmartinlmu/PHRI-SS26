#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32
from brewbot_interfaces.action import SuggestDrink

THRESHOLD = 10        # BPM deviation to trigger a suggestion
COOLDOWN = 60.0       # seconds to wait before suggesting again


class StateDeciderNode(Node):

    def __init__(self):
        super().__init__("state_estimator")

        self.declare_parameter("base_hr", 70)
        self._base_hr = self.get_parameter("base_hr").get_parameter_value().integer_value

        self._action_client = ActionClient(self, SuggestDrink, "suggest_drink")

        self._sub = self.create_subscription(
            Int32, "/heartrate", self._on_heartrate, 10
        )

        self._suggesting = False
        self._cooldown_timer = None

        self.get_logger().info(
            f"State decider ready (base HR: {self._base_hr} BPM, threshold: ±{THRESHOLD})"
        )

    def _on_heartrate(self, msg):
        hr = msg.data
        deviation = hr - self._base_hr

        self.get_logger().debug(f"[HR] {hr} BPM (deviation: {deviation:+d})")

        if self._suggesting:
            self.get_logger().debug("[HR] Suggestion already in progress, skipping")
            return

        if deviation <= -THRESHOLD:
            self.get_logger().info(f"[HR] {hr} BPM — low, suggesting coffee")
            self._send_suggestion("coffee")
        elif deviation >= THRESHOLD:
            self.get_logger().info(f"[HR] {hr} BPM — high, suggesting tea")
            self._send_suggestion("tea")

    def _send_suggestion(self, drink):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Suggestion handler not available")
            return

        self._suggesting = True
        goal = SuggestDrink.Goal(drink=drink)

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[SUGGEST] Goal rejected")
            self._start_cooldown()
            return

        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"[SUGGEST] Done — accepted={result.accepted}")
        self._start_cooldown()

    def _start_cooldown(self):
        self.get_logger().info(f"[COOLDOWN] Waiting {COOLDOWN}s before next suggestion")
        self._cooldown_timer = self.create_timer(COOLDOWN, self._end_cooldown)

    def _end_cooldown(self):
        self._cooldown_timer.cancel()
        self._cooldown_timer = None
        self._suggesting = False
        self.get_logger().info("[COOLDOWN] Done, ready for next suggestion")


def main():
    rclpy.init()
    node = StateDeciderNode()
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
