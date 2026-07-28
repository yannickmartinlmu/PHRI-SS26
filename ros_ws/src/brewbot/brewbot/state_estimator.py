#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Float32
from brewbot_interfaces.action import SuggestDrink

# Fixed thresholds, no per-person calibration yet. These become
# baseline-relative once a calibration step exists. Tune on real sensors.
HR_THRESHOLD = 10       # BPM above base_hr → aroused
SKIN_TEMP_HOT = 35.0    # °C skin temp → hot   (typical resting skin ~33)
SKIN_TEMP_COLD = 30.0   # °C skin temp → cold
MOTION_ACTIVE = 1.5     # accel magnitude (g, gravity-subtracted) → moving
FATIGUE_HRV_FRAC = 0.8  # HRV dropped below 80% of session start → fatigued
COOLDOWN = 60.0         # seconds between suggestions


# Pure decision tree — no ROS, so the demo() below actually tests it.
# Order matters: resolve thermal + motion BEFORE the arousal axis, else "hot",
# "active" and "stressed" all fight over the same HR/EDA spike. None = signal
# not available yet (sensor not wired) → that branch is skipped.
def decide(hr, hrv, eda, temp, motion, base_hr, hrv_baseline):
    # 1. thermal axis — the only signals that actually sense temperature
    if temp is not None:
        if temp >= SKIN_TEMP_HOT:
            return "hot", "water"
        if temp <= SKIN_TEMP_COLD:
            return "cold", "hot chocolate"

    # 2. motion — arousal is from moving, not stress
    if motion is not None and motion >= MOTION_ACTIVE:
        return "active", "water"

    # 3. arousal axis — high HR (HRV↓ / EDA↑ reinforce once tuned)
    if hr is not None and hr - base_hr >= HR_THRESHOLD:
        return "stressed", "tea"

    # 4. fatigue — a TREND, not a single reading: HRV sagging over the session
    #    while calm, not warm, not moving. Distinguishes tired from stressed.
    if hrv is not None and hrv_baseline is not None and hrv <= hrv_baseline * FATIGUE_HRV_FRAC:
        return "fatigued", "coffee"

    # 5. baseline → no proactive offer
    return "calm", None


class StateDeciderNode(Node):

    def __init__(self):
        super().__init__("state_estimator")

        self.declare_parameter("base_hr", 70)
        self._base_hr = self.get_parameter("base_hr").get_parameter_value().integer_value

        self._action_client = ActionClient(self, SuggestDrink, "suggest_drink")

        # Latest reading per signal; None until its sensor publishes.
        self._hr = None
        self._hrv = None
        self._eda = None
        self._temp = None
        self._motion = None
        self._hrv_baseline = None  # First HRV seen = session start; a
                                   # real calibration replaces this later.

        # HR is the live heartbeat that drives re-evaluation; the rest are
        # scaffolded — subscriptions exist, decide() skips them while None.
        self.create_subscription(Int32, "/heartrate", self._on_heartrate, 10)
        self.create_subscription(Float32, "/hrv", self._on_hrv, 10)
        self.create_subscription(Float32, "/eda", self._on_eda, 10)
        self.create_subscription(Float32, "/skin_temp", self._on_temp, 10)
        self.create_subscription(Float32, "/motion", self._on_motion, 10)

        self._suggesting = False
        self._cooldown_timer = None

        self.get_logger().info(
            f"State decider ready (base HR: {self._base_hr} BPM, threshold: ±{HR_THRESHOLD})"
        )

    # --- signal caches ---
    def _on_hrv(self, msg):
        self._hrv = msg.data
        if self._hrv_baseline is None:
            self._hrv_baseline = msg.data

    def _on_eda(self, msg):
        self._eda = msg.data

    def _on_temp(self, msg):
        self._temp = msg.data

    def _on_motion(self, msg):
        self._motion = msg.data

    def _on_heartrate(self, msg):
        self._hr = msg.data

        if self._suggesting:
            return

        state, drink = decide(
            self._hr, self._hrv, self._eda, self._temp, self._motion,
            self._base_hr, self._hrv_baseline,
        )
        self.get_logger().debug(f"[STATE] {state} (HR {self._hr})")

        if drink is not None:
            self.get_logger().info(f"[STATE] {state} → suggesting {drink}")
            self._send_suggestion(drink)

    def _send_suggestion(self, drink):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("Suggestion handler not available")
            return

        self._suggesting = True
        future = self._action_client.send_goal_async(SuggestDrink.Goal(drink=drink))
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


def demo():
    b = 70
    # thermal wins over everything, even a stress-level HR
    assert decide(90, None, None, 36.0, None, b, None) == ("hot", "water")
    assert decide(50, None, None, 28.0, None, b, None) == ("cold", "hot chocolate")
    # motion beats arousal
    assert decide(90, None, None, 33.0, 2.0, b, None) == ("active", "water")
    # arousal only when not hot/cold/moving
    assert decide(85, None, None, 33.0, 0.2, b, None) == ("stressed", "tea")
    # fatigue = HRV trend, needs a baseline; calm HR
    assert decide(70, 40.0, None, 33.0, 0.0, b, 60.0) == ("fatigued", "coffee")
    # HRV present but not yet dropped → still calm
    assert decide(70, 58.0, None, 33.0, 0.0, b, 60.0) == ("calm", None)
    # nothing wired → calm, no offer
    assert decide(70, None, None, None, None, b, None) == ("calm", None)
    print("decide() self-check passed")


def main():
    if "--selfcheck" in sys.argv:
        demo()
        return
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
