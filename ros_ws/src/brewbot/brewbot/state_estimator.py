#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Float32
from brewbot_interfaces.srv import GetUserState
from brewbot.drinks import MENU

# Tune on real sensors. Temp/HR are absolute; EDA is baseline-relative because
# absolute skin conductance varies ~100x between people (E4 range 0.01–10 uS),
# so a fixed uS threshold classifies the wearer, not their state.
HR_THRESHOLD = 10       # BPM above base_hr → aroused
SKIN_TEMP_HOT = 35.0    # °C skin temp → hot   (resting skin ~33, so ±2)
SKIN_TEMP_COLD = 31.0   # °C skin temp → cold

# /motion is INSTANTANEOUS |accel| (g, gravity removed) — one wrist flick
# spikes it. decide() wants "is this person active", so the node keeps a slow
# mean and this threshold applies to THAT, which is a much smaller number than
# a raw peak. Guessed: seated ~0.01-0.05, walking ~0.1-0.3. Read the [QUERY]
# log in the lab and set it for real.
MOTION_ACTIVE = 0.15
MOTION_ALPHA = 0.02     # EMA on 1 Hz samples → ~50 s of history

# ponytail: EMA baseline, not session-start. Dry E4 electrodes need minutes to
# equilibrate, so the first reading is electrode settling, not the person.
# Ceiling: an EMA is a high-pass — it sees CHANGES, and a slow all-session
# drift gets absorbed into the baseline and never fires. Fine here; a real
# calibration step is the upgrade.
EDA_ALPHA = 0.005       # ~200 s of history at 1 Hz
EDA_RISE_FRAC = 1.15    # 15% over own baseline → sympathetic arousal
EDA_DROP_FRAC = 0.85    # 15% under → sympathetic tone sagging

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
# "active" and "stressed" all fight over the same HR/EDA rise. None = signal
# not available yet (sensor not wired) → that branch is skipped.
#
# No HRV: RMSSD needs clean beat-to-beat intervals, and the E4's wrist PPG does
# not survive a user walking around a lab (see sensor_e4). There is no /hrv
# publisher, so that branch only ever fired against sensor_fakes.
def decide(hr, eda, temp, motion, base_hr, eda_base):
    # 1. thermal axis — the only signals that actually sense temperature
    if temp is not None:
        if temp >= SKIN_TEMP_HOT:
            return "hot", "water"
        if temp <= SKIN_TEMP_COLD:
            return "cold", "tea"

    # 2. motion — arousal is from moving, not stress
    if motion is not None and motion >= MOTION_ACTIVE:
        return "active", "water"

    # 3. arousal axis. Either signal alone is enough: heat drives EDA and
    #    movement drives HR, but both confounds are already excluded above, so
    #    OR costs fewer misses here than it costs false positives.
    #    `eda_base` truthiness covers None AND 0.0 — an unworn band reads ~0,
    #    and every ratio against 0 is true.
    if hr is not None and hr - base_hr >= HR_THRESHOLD:
        return "stressed", "tea"
    if eda is not None and eda_base and eda >= eda_base * EDA_RISE_FRAC:
        return "stressed", "tea"

    # 4. low arousal — sympathetic tone sagging while calm, still and not warm.
    #    Those three are implied by reaching this line, so no extra conditions.
    if eda is not None and eda_base and eda <= eda_base * EDA_DROP_FRAC:
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

        # 80, not a textbook 70: this is a lab, the user is mostly on their feet.
        self.declare_parameter("base_hr", 80)
        self._base_hr = self.get_parameter("base_hr").get_parameter_value().integer_value

        # Latest reading per signal; None until its sensor publishes.
        self._hr = None
        self._eda = None
        self._temp = None
        self._motion = None   # smoothed, not raw — see _on_motion
        self._eda_base = None

        # Not every sensor is wired yet; decide() skips a signal while it is None.
        self.create_subscription(Int32, "/heartrate", self._on_heartrate, 10)
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
    def _on_eda(self, msg):
        self._eda = msg.data
        # Tonic baseline: what "normal" is for THIS wearer. Seeded on the first
        # sample so it does not spend minutes climbing away from zero.
        self._eda_base = msg.data if self._eda_base is None else (
            self._eda_base + EDA_ALPHA * (msg.data - self._eda_base)
        )

    def _on_temp(self, msg):
        self._temp = msg.data

    def _on_motion(self, msg):
        # Smooth here, not in sensor_e4: the sensor should publish what it
        # measures, and "active" is a decision-layer notion.
        self._motion = msg.data if self._motion is None else (
            self._motion + MOTION_ALPHA * (msg.data - self._motion)
        )

    def _on_heartrate(self, msg):
        self._hr = msg.data

    def _decide(self):
        return decide(
            self._hr, self._eda, self._temp, self._motion,
            self._base_hr, self._eda_base,
        )

    # Thresholds above are guesses until someone reads real numbers off a real
    # wrist. Riding along on the two lines that already log means no new spam.
    def _inputs(self):
        def f(v):
            return "-" if v is None else f"{v:.2f}"
        return (f"hr={f(self._hr)} eda={f(self._eda)}/{f(self._eda_base)} "
                f"temp={f(self._temp)} motion={f(self._motion)}")

    # 1 Hz, cheap: decide() is pure arithmetic on five cached floats.
    def _recheck(self):
        state, drink = self._decide()
        self._streak = self._streak + 1 if drink else 0
        now = time.monotonic()
        if spike_due(self._streak, now - self._last_spike):
            self._last_spike = now
            self.get_logger().info(f"[SPIKE] {state} → {drink}  {self._inputs()}")
            self._spike_pub.publish(Empty())

    # --- the only thing anyone can ask this node ---
    def _on_query(self, request, response):
        response.state, response.drink = self._decide()
        self.get_logger().info(
            f"[QUERY] {response.state} → {response.drink or 'no suggestion'}  {self._inputs()}"
        )
        return response


def demo():
    b, e = 80, 2.0     # base HR, EDA baseline (uS)
    #        hr,  eda, temp, motion, base, eda_base
    # thermal wins over everything, even a stress-level HR and EDA
    assert decide(95, 4.0, 36.0, None, b, e) == ("hot", "water")
    assert decide(50, 4.0, 30.0, None, b, e) == ("cold", "tea")
    # 31/35 are the edges themselves, not just past them
    assert decide(80, None, 31.0, None, b, e) == ("cold", "tea")
    assert decide(80, None, 35.0, None, b, e) == ("hot", "water")
    assert decide(80, None, 33.0, None, b, e) == ("calm", "")
    # motion beats arousal — and it is the SMOOTHED value being compared
    assert decide(95, 4.0, 33.0, 0.3, b, e) == ("active", "water")
    # a raw wrist-flick peak is what the smoothing is meant to swallow
    assert decide(80, None, 33.0, 0.02, b, e) == ("calm", "")
    # arousal on either signal alone, once not hot/cold/moving
    assert decide(95, None, 33.0, 0.01, b, e) == ("stressed", "tea")
    assert decide(80, 2.5, 33.0, 0.01, b, e) == ("stressed", "tea")
    # low arousal → fatigued. Calm/still/not-warm is implied by the ordering.
    assert decide(80, 1.5, 33.0, 0.01, b, e) == ("fatigued", "coffee")
    # inside the deadband → no offer
    assert decide(80, 2.1, 33.0, 0.01, b, e) == ("calm", "")
    # an unworn band reads ~0 uS: every ratio against a 0 baseline is true, so
    # this must NOT come out as arousal.
    assert decide(80, 0.0, 33.0, 0.01, b, 0.0) == ("calm", "")
    # nothing wired → calm, no offer
    assert decide(None, None, None, None, b, None) == ("calm", "")
    # "no suggestion" must be falsy at the caller, not the string "None"
    assert not decide(80, None, None, None, b, None)[1]
    # Nothing may be suggested that the arm cannot fetch — the drift guard.
    for case in [(95, 4.0, 36.0, None, b, e),
                 (50, 4.0, 30.0, None, b, e),
                 (95, 4.0, 33.0, 0.3, b, e),
                 (95, None, 33.0, 0.01, b, e),
                 (80, 2.5, 33.0, 0.01, b, e),
                 (80, 1.5, 33.0, 0.01, b, e)]:
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
