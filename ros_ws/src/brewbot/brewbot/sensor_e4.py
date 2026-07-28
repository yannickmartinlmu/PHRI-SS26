#!/usr/bin/env python3
# Empatica E4 → the scalar topics state_estimator expects.
# The E4 has no HR characteristic, it streams a 64 Hz BVP waveform, so the
# pulse extraction lives here: py_e4lib stays a pure BLE driver, and
# /heartrate stays the swappable seam any other HR sensor can feed.
#
# Tuned against a 30 s reference capture (py-e4lib/examples/record.py). Re-record
# and re-check the numbers in HeartRate's docstring before changing constants.

import asyncio
import math
import sys
import threading
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

# py-e4lib ships with the repo but sits outside the workspace, and `ros2 run`
# executes under /usr/bin/python3, which never sees the venv — so pip installing
# it doesn't help. Search upward for the directory instead of counting "..":
# colcon copies this file into build/ and install/ at different depths.
for _root in Path(__file__).resolve().parents:
    if (_root / "py-e4lib").is_dir():
        sys.path.insert(0, str(_root / "py-e4lib"))
        break
else:
    raise ImportError("py-e4lib not found above " + __file__ + " — is the repo intact?")

from py_e4lib import E4Client  # noqa: E402  (needs the sys.path line above)

SAMPLE_RATE = 64.0    # Hz, E4 BVP nominal. Measured 64.4 Hz over a 26 s capture.
                      # If HR reads consistently a few % off, tune here — the
                      # rest of the math is scale-free.
WINDOW_S = 8.0        # s of BVP per HR estimate. Shorter reacts faster and is
                      # noisier; 6 and 10 both work, 8 was the best compromise
                      # on the reference recording.
WARMUP_S = 5.0        # s of BVP to throw away. py_e4lib's FIR buffers and Kalman
                      # start at zero and the output pins at -630 for ~3 s; that
                      # transient has more energy than the pulse and swamps it.
BASELINE_S = 2.0      # s rolling-mean window used to re-centre the signal
MIN_BPM, MAX_BPM = 40.0, 120.0   # search range. Seated user, not an athlete
                                 # mid-set — widen if that stops being true.
OCTAVE_TOL = 0.8      # a lag of 2T correlates nearly as well as T, and taking
                      # the global max lands on it and halves the rate. Accept
                      # the FIRST peak within this fraction of the best instead.
MIN_QUALITY = 0.05    # normalised peak below this is noise, not a pulse
PUBLISH_HZ = 1.0      # BLE streams at 4–64 Hz; decide() must not run that fast
RECONNECT_DELAY = 5.0 # s, the E4's 0x08 supervision-timeout drops are routine


class HeartRate:
    """BVP waveform → BPM, by autocorrelation over a sliding window.

    Beat-by-beat detection was tried first and does not survive this signal.
    py_e4lib's FIR chain is a low-pass (all coefficients positive, summing to 1),
    so it removes HF noise but leaves the E4's wrist PPG with several zero
    crossings per beat at an amplitude of only ~10 units. Replaying a 26 s
    recording through it gave intervals of 0.96, 0.64, 1.22, 0.90, 0.88, 1.00,
    0.61, 0.51 s against a true 1.03 s: single beats split in two, pairs merged
    into one, HR swinging 43–134 against a true 56–60. No refractory value fixes
    that — the spurious crossing lands at ~0.5 s, and a window wide enough to
    reject it also rejects real beats above 120 BPM.

    Autocorrelation asks a different question — what period does this window
    repeat at — and is indifferent to waveform shape, baseline drift and the
    dicrotic notch. On the same recording it holds 55–64 BPM. Stdlib only.
    """

    def __init__(self):
        self._n = 0
        self._mean_buf = deque(maxlen=int(SAMPLE_RATE * BASELINE_S))
        self._buf = deque(maxlen=int(SAMPLE_RATE * WINDOW_S))
        self._min_lag = int(SAMPLE_RATE * 60.0 / MAX_BPM)
        self._max_lag = int(SAMPLE_RATE * 60.0 / MIN_BPM)

    def feed(self, values):
        for v in values:
            self._n += 1
            self._mean_buf.append(v)
            if self._n < SAMPLE_RATE * WARMUP_S:
                continue
            # Rolling mean high-passes the signal. Autocorrelation measures how
            # a window correlates with itself, so any DC offset or slow drift
            # correlates with everything and buries the pulse.
            self._buf.append(v - sum(self._mean_buf) / len(self._mean_buf))

    def bpm(self):
        """BPM as a float, or None while warming up or when the signal is noise."""
        if len(self._buf) < self._buf.maxlen:
            return None

        # The rolling mean leaves a constant residual on a sloping baseline, and
        # a constant correlates equally at every lag — which drags the peak down
        # to the shortest one and reports MAX_BPM. Re-zero the window itself.
        d = list(self._buf)
        m = sum(d) / len(d)
        d = [x - m for x in d]

        c0 = sum(x * x for x in d) / len(d)
        if c0 <= 0.0:
            return None

        # ponytail: O(window x lags) ~ 30k float ops per second in pure Python.
        # Measured well under the 1 s budget. If it ever matters, decimate to
        # 32 Hz first — the pulse is under 2 Hz, so nothing is lost.
        corr = [sum(d[i] * d[i + lag] for i in range(len(d) - lag)) / (len(d) - lag) / c0
                for lag in range(self._min_lag, self._max_lag)]

        best = max(corr)
        if best < MIN_QUALITY:
            return None

        # First local maximum within OCTAVE_TOL of the best, not the global best.
        k = next(i for i, c in enumerate(corr)
                 if c >= OCTAVE_TOL * best
                 and (i == 0 or c >= corr[i - 1])
                 and (i == len(corr) - 1 or c >= corr[i + 1]))

        lag = self._min_lag + k
        # Parabolic interpolation: whole-sample lags quantise HR to ~1 BPM steps
        # around 60, which shows up as a visible stair-step on the topic.
        if 0 < k < len(corr) - 1:
            a, b, c = corr[k - 1], corr[k], corr[k + 1]
            denom = a - 2 * b + c
            if denom:
                lag += 0.5 * (a - c) / denom

        return 60.0 * SAMPLE_RATE / lag


class E4SensorNode(Node):

    def __init__(self):
        super().__init__("sensor_e4")

        self._address = self.declare_parameter("address", "").value

        # The BVP-derived HR below is the weakest number this node produces.
        # Set false to let a dedicated HR sensor own /heartrate (see
        # sensors.launch.py) — this also skips the 64 Hz BVP subscription
        # entirely, not just the publish.
        self._publish_hr = self.declare_parameter("publish_hr", True).value

        # ponytail: no /hrv publisher. RMSSD is defined on beat-to-beat
        # differences, and this signal can't give trustworthy individual beats
        # (see HeartRate) — RMSSD amplifies exactly that error, which is why it
        # read ~450 ms against a real 20–100. state_estimator latches the first
        # /hrv forever as its baseline, so a wrong number is worse than none.
        # Restore this if the BVP decode is ever fixed, not before.
        self._hr_pub = (self.create_publisher(Int32, "/heartrate", 10)
                        if self._publish_hr else None)
        self._eda_pub = self.create_publisher(Float32, "/eda", 10)
        self._temp_pub = self.create_publisher(Float32, "/skin_temp", 10)
        self._motion_pub = self.create_publisher(Float32, "/motion", 10)

        # ponytail: BLE thread writes these, timer thread reads them. Plain
        # attribute assignment and deque.append are GIL-atomic; add a lock only
        # if this ever grows into a read-modify-write.
        self._beats = HeartRate()
        self._eda = None
        self._temp = None
        self._motion_peak = None

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        threading.Thread(target=lambda: asyncio.run(self._ble_task()), daemon=True).start()
        self.get_logger().info("E4 node started, connecting to sensor...")

    # --- BLE callbacks: store only, never publish (they run at 4–64 Hz) ---
    def _on_bvp(self, values):
        self._beats.feed(values)

    def _on_gsr(self, values):
        self._eda = values[-1]

    def _on_temp(self, values):
        self._temp = values[-1]

    def _on_acc(self, values):
        # Gravity-subtracted magnitude in g. Peak over the publish interval, not
        # mean — a mean averages arm-waving back down to resting.
        for x, y, z in values:
            m = abs(math.sqrt(x * x + y * y + z * z) / 64.0 - 1.0)
            self._motion_peak = m if self._motion_peak is None else max(self._motion_peak, m)

    # --- the one place anything reaches ROS ---
    def _tick(self):
        if self._publish_hr:
            bpm = self._beats.bpm()
            if bpm is not None:
                self._hr_pub.publish(Int32(data=round(bpm)))

        if self._eda is not None:
            self._eda_pub.publish(Float32(data=float(self._eda)))

        if self._temp is not None:
            self._temp_pub.publish(Float32(data=float(self._temp)))

        if self._motion_peak is not None:
            self._motion_pub.publish(Float32(data=float(self._motion_peak)))
            self._motion_peak = None

    async def _ble_task(self):
        while rclpy.ok():
            try:
                client = E4Client(self._address) if self._address else await E4Client.find()
                async with client:
                    if self._publish_hr:
                        client.enable_bvp(self._on_bvp)
                    client.enable_gsr(self._on_gsr)
                    client.enable_temp(self._on_temp)
                    client.enable_acc(self._on_acc)
                    await client.start()
                    self.get_logger().info("E4 streaming")
                    while rclpy.ok() and client.connected:
                        await asyncio.sleep(1.0)
                self.get_logger().warn("E4 link lost")
            except Exception as e:
                self.get_logger().warn(f"E4 connection failed: {e}")
            if rclpy.ok():
                await asyncio.sleep(RECONNECT_DELAY)


def demo():
    n = int(SAMPLE_RATE * 60)

    def ppg(bpm, drift=0.0):
        """A pulse shape, not a sine: a second harmonic puts a dicrotic notch in
        each beat, which is the feature that broke zero-crossing detection.
        Optional drift is the wandering baseline the low-pass leaves behind."""
        f = bpm / 60.0
        return [math.sin(2 * math.pi * f * (i / SAMPLE_RATE))
                + 0.6 * math.sin(4 * math.pi * f * (i / SAMPLE_RATE) + 1.0)
                + drift * (i / SAMPLE_RATE)
                for i in range(n)]

    # The notch must not read as a second beat, and drift must not hide the pulse.
    d = HeartRate()
    d.feed(ppg(72.0))
    assert 70.0 <= d.bpm() <= 74.0, d.bpm()

    drifting = HeartRate()
    drifting.feed(ppg(72.0, drift=5.0))
    assert 70.0 <= drifting.bpm() <= 74.0, drifting.bpm()

    # Octave guard: 36 BPM correlates nearly as well as 72 and must lose.
    assert d.bpm() > 50.0, d.bpm()

    # Warm-up and noise gates: no readings out of thin air.
    assert HeartRate().bpm() is None

    flat = HeartRate()
    flat.feed([0.0] * n)
    assert flat.bpm() is None, flat.bpm()

    # Under WARMUP_S + WINDOW_S of signal there is nothing to report yet.
    short = HeartRate()
    short.feed(ppg(72.0)[:int(SAMPLE_RATE * (WARMUP_S + WINDOW_S - 1))])
    assert short.bpm() is None, short.bpm()

    # Arriving in 11-sample BLE packets must give the same answer as one blob.
    chunked = HeartRate()
    samples = ppg(72.0)
    for i in range(0, len(samples), 11):
        chunked.feed(samples[i:i + 11])
    assert abs(chunked.bpm() - d.bpm()) < 0.1, (chunked.bpm(), d.bpm())

    print("HeartRate self-check passed")


def main():
    if "--selfcheck" in sys.argv:
        demo()
        return
    rclpy.init()
    node = E4SensorNode()
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
