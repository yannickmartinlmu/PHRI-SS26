#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Float32
from brewbot_interfaces.srv import GetUserState
from brewbot.drinks import MENU

# Fixed thresholds, no per-person calibration yet. These become
# baseline-relative once a calibration step exists. Tune on real sensors.
HR_THRESHOLD = 10       # BPM above base_hr → aroused
SKIN_TEMP_HOT = 35.0    # °C skin temp → hot   (typical resting skin ~33)
SKIN_TEMP_COLD = 30.0   # °C skin temp → cold
MOTION_ACTIVE = 1.5     # accel magnitude (g, gravity-subtracted) → moving
FATIGUE_HRV_FRAC = 0.8  # HRV dropped below 80% of session start → fatigued

SPIKE_PERIOD = 1.0      # s between unsolicited re-checks
SPIKE_STREAK = 3        # consecutive agreeing samples before we believe it.
                        # HR comes from BVP autocorrelation — one noisy sample
                        # crossing the threshold must not start a conversation.
SPIKE_COOLDOWN = 300.0  # s — at most one unsolicited offer per 5 min


# ponytail: level-crossing IS the spike. decide() already encodes "elevated",
# so this only watches its output go falsy → truthy. Real spike math (rolling
# window, z-score) would add a second untuned threshold set fighting the first;
# add it only if crossing proves too blunt on real sensors.
# streak counts samples INCLUDING this one, so == fires exactly once per
# episode: no re-fire while the user simply stays stressed.
def spike_due(streak, since_last):
    return streak == SPIKE_STREAK and since_last > SPIKE_COOLDOWN


# Pure decision tree — no ROS, so the demo() below actually tests it.
# Every drink it names must be a MENU key; demo() asserts that, because a
# suggestion nothing downstream can make is worse than no suggestion.
# Order matters: resolve thermal + motion BEFORE the arousal axis, else "hot",
# "active" and "stressed" all fight over the same HR/EDA spike. None = signal
# not available yet (sensor not wired) → that branch is skipped.
def decide(hr, hrv, eda, temp, motion, base_hr, hrv_baseline):
    # 1. thermal axis — the only signals that actually sense temperature
    if temp is not None:
        if temp >= SKIN_TEMP_HOT:
            return "hot", "water"
        if temp <= SKIN_TEMP_COLD:
            return "cold", "tea"

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

    # 5. baseline → no proactive offer. "" not None: this is the service
    #    response field, and both are falsy at the caller.
    return "calm", ""


# Passive observer: subscriptions cache, the service answers. It never
# initiates anything — deciding WHEN to interrupt the user is a policy question
# that belongs in whichever node saw the triggering event (arrival, gesture),
# because that node is the one that knows if we're already mid-conversation.
class StateDeciderNode(Node):

    def __init__(self):
        super().__init__("state_estimator")

        self.declare_parameter("base_hr", 70)
        self._base_hr = self.get_parameter("base_hr").get_parameter_value().integer_value

        # Latest reading per signal; None until its sensor publishes.
        self._hr = None
        self._hrv = None
        self._eda = None
        self._temp = None
        self._motion = None
        self._hrv_baseline = None  # First HRV seen = session start; a
                                   # real calibration replaces this later.

        # Not every sensor is wired yet; decide() skips a signal while it is None.
        self.create_subscription(Int32, "/heartrate", self._on_heartrate, 10)
        self.create_subscription(Float32, "/hrv", self._on_hrv, 10)
        self.create_subscription(Float32, "/eda", self._on_eda, 10)
        self.create_subscription(Float32, "/skin_temp", self._on_temp, 10)
        self.create_subscription(Float32, "/motion", self._on_motion, 10)

        self.create_service(GetUserState, "/get_user_state", self._on_query)

        # The one thing this node does unprompted. Still not policy: it reports
        # "something changed", trigger decides whether to interrupt.
        self._spike_pub = self.create_publisher(Empty, "/user_spike", 10)
        self._streak = 0
        self._last_spike = float("-inf")   # not 0.0: monotonic() is uptime, so
                                           # 0.0 would mute the first spike on a
                                           # freshly booted machine
        self.create_timer(SPIKE_PERIOD, self._recheck)

        self.get_logger().info(
            f"State estimator ready (base HR: {self._base_hr} BPM, threshold: ±{HR_THRESHOLD})"
        )

    # --- signal caches ---
    # Last value wins, forever. A dead sensor answers with a stale
    # reading rather than None; the TTS drop warning is what surfaces that.
    # Timestamp each signal and age it out here if that stops being enough.
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

    def _decide(self):
        return decide(
            self._hr, self._hrv, self._eda, self._temp, self._motion,
            self._base_hr, self._hrv_baseline,
        )

    # 1 Hz, cheap: decide() is pure arithmetic on five cached floats.
    def _recheck(self):
        state, drink = self._decide()
        self._streak = self._streak + 1 if drink else 0
        now = time.monotonic()
        if spike_due(self._streak, now - self._last_spike):
            self._last_spike = now
            self.get_logger().info(f"[SPIKE] {state} → {drink}")
            self._spike_pub.publish(Empty())

    # --- the only thing anyone can ask this node ---
    def _on_query(self, request, response):
        response.state, response.drink = self._decide()
        self.get_logger().info(
            f"[QUERY] {response.state} → {response.drink or 'no suggestion'}"
        )
        return response


def demo():
    b = 70
    # thermal wins over everything, even a stress-level HR
    assert decide(90, None, None, 36.0, None, b, None) == ("hot", "water")
    assert decide(50, None, None, 28.0, None, b, None) == ("cold", "tea")
    # motion beats arousal
    assert decide(90, None, None, 33.0, 2.0, b, None) == ("active", "water")
    # arousal only when not hot/cold/moving
    assert decide(85, None, None, 33.0, 0.2, b, None) == ("stressed", "tea")
    # fatigue = HRV trend, needs a baseline; calm HR
    assert decide(70, 40.0, None, 33.0, 0.0, b, 60.0) == ("fatigued", "coffee")
    # HRV present but not yet dropped → still calm
    assert decide(70, 58.0, None, 33.0, 0.0, b, 60.0) == ("calm", "")
    # nothing wired → calm, no offer
    assert decide(70, None, None, None, None, b, None) == ("calm", "")
    # "no suggestion" must be falsy at the caller, not the string "None"
    assert not decide(70, None, None, None, None, b, None)[1]
    # Nothing may be suggested that the arm cannot fetch — the drift guard.
    for case in [(90, None, None, 36.0, None, b, None),
                 (50, None, None, 28.0, None, b, None),
                 (90, None, None, 33.0, 2.0, b, None),
                 (85, None, None, 33.0, 0.2, b, None),
                 (70, 40.0, None, 33.0, 0.0, b, 60.0)]:
        drink = decide(*case)[1]
        assert drink in MENU, f"decide() suggested '{drink}', not on the menu"

    # One spike per episode, not one per sample.
    assert not spike_due(SPIKE_STREAK - 1, 999)   # not convinced yet
    assert spike_due(SPIKE_STREAK, 999)           # the edge
    assert not spike_due(SPIKE_STREAK + 1, 999)   # still stressed ≠ new spike
    assert not spike_due(SPIKE_STREAK, 10)        # cooldown still running
    assert spike_due(SPIKE_STREAK, float("inf"))  # boot: must not be muted
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
