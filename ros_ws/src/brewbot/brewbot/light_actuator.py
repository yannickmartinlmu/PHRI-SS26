#!/usr/bin/env python3
"""Philips Hue light actuator backed by Home Assistant.

Provides a tiny start/stop blink service. 
Interaction manager calls for blinking while the arm waits under faucet and
stop it in a finally block once the user is done or wait times out.
"""

import threading
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool

from brewbot.home_assistant import HomeAssistantClient, HomeAssistantError

ENTITY_ID = "light.spot_4"
BLINK_SERVICE = "/lab_light/blink"


class LightActuator(Node):
    def __init__(self):
        super().__init__("light_actuator")
        cb = ReentrantCallbackGroup()

        # color value is the current color set in home assistant, could additionally set the color if necessary
        self.declare_parameter("entity_id", ENTITY_ID)
        self.declare_parameter("request_timeout", 5.0)
        self.declare_parameter("blink_interval", 0.5)
        self.declare_parameter("flash", "short")
        self.declare_parameter("brightness", 255)   # max value 255
        self.declare_parameter("restore_on_stop", True)

        self._entity_id = self.get_parameter("entity_id").value
        self._blink_interval = float(self.get_parameter("blink_interval").value)
        self._flash = self.get_parameter("flash").value
        self._brightness = int(self.get_parameter("brightness").value)
        self._restore_on_stop = bool(self.get_parameter("restore_on_stop").value)
        timeout = self.get_parameter("request_timeout").value

        self._ha = HomeAssistantClient(timeout=timeout)
        self._lock = threading.Lock()
        self._blinking = False
        self._saved_state = None
        self._timer = None

        self.create_service(
            SetBool,
            BLINK_SERVICE,
            self._on_blink,
            callback_group=cb,
        )
        self.get_logger().info(f"Light actuator ready ({self._entity_id})")

    def _on_blink(self, request, response):
        try:
            if request.data:
                self._start_blinking()
                response.success = True
                response.message = "blinking"
            else:
                self._stop_blinking()
                response.success = True
                response.message = "stopped"
        except Exception as exc:
            self.get_logger().error(f"[light] blink request failed: {exc}")
            response.success = False
            response.message = str(exc)
        return response

    def _start_blinking(self):
        with self._lock:
            if self._blinking:
                return
            if self._restore_on_stop:
                self._saved_state = self._read_state()
            self._blinking = True
            self._flash_once()
            self._timer = self.create_timer(self._blink_interval, self._on_timer)
            self.get_logger().info("[light] blinking started")

    def _stop_blinking(self):
        with self._lock:
            if not self._blinking:
                return
            self._blinking = False
            if self._timer is not None:
                self._timer.cancel()
                self.destroy_timer(self._timer)
                self._timer = None
            if self._restore_on_stop:
                self._restore_saved_state()
            self._saved_state = None
            self.get_logger().info("[light] blinking stopped")

    def _on_timer(self):
        with self._lock:
            if not self._blinking:
                return
            try:
                self._flash_once()
            except Exception as exc:
                self.get_logger().error(f"[light] flash failed: {exc}")

    def _flash_once(self):
        body = {
            "entity_id": self._entity_id,
            "flash": self._flash,
            "brightness": self._brightness,
        }
        self._ha.call_service("light", "turn_on", body)

    def _read_state(self):
        try:
            return self._ha.get_state(self._entity_id)
        except HomeAssistantError as exc:
            self.get_logger().warn(f"[light] could not save previous state: {exc}")
            return None

    def _restore_saved_state(self):
        state = self._saved_state
        if not isinstance(state, dict):
            self._ha.call_service("light", "turn_off", {"entity_id": self._entity_id})
            return

        if state.get("state") != "on":
            self._ha.call_service("light", "turn_off", {"entity_id": self._entity_id})
            return

        attributes = state.get("attributes") or {}
        body = {"entity_id": self._entity_id}
        for key in ("brightness", "rgb_color", "color_temp_kelvin"):
            if key in attributes:
                body[key] = attributes[key]
        self._ha.call_service("light", "turn_on", body)


def main():
    rclpy.init()
    try:
        node = LightActuator()
    except HomeAssistantError as exc:
        print(f"light_actuator: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
