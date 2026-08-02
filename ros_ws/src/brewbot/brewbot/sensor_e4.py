#!/usr/bin/env python3
# Empatica E4 → the scalar topics state_estimator expects.
# The E4 has no HR characteristic, only a 64 Hz BVP waveform, so the pulse
# extraction lives here: py_e4lib stays a pure BLE driver and /heartrate stays
# the seam any other HR sensor can feed.
# Constants are tuned against a 30 s reference capture (py-e4lib/examples/record.py).

import asyncio
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Float32

# py-e4lib ships with the repo but sits outside the workspace, and `ros2 run`
# executes under /usr/bin/python3, which never sees the venv. Search upward
# rather than counting "..": colcon copies this file in at different depths.
for _root in Path(__file__).resolve().parents:
    if (_root / "py-e4lib").is_dir():
        sys.path.insert(0, str(_root / "py-e4lib"))
        break
else:
    raise ImportError("py-e4lib not found above " + __file__ + " — is the repo intact?")

from py_e4lib import E4Client  # noqa: E402  (needs the sys.path line above)

SAMPLE_RATE = 64.0    # Hz, E4 BVP nominal. Measured 64.4 Hz over a 26 s capture;
                      # tune here if HR reads a few % off, the rest is scale-free.
WINDOW_S = 8.0        # s of BVP per HR estimate. 6 and 10 both work, 8 was the
                      # best compromise on the reference recording.
WARMUP_S = 5.0        # s of BVP to throw away: py_e4lib's FIR and Kalman start at
                      # zero and the output pins at -630 for ~3 s.
BASELINE_S = 2.0      # s rolling-mean window used to re-centre the signal
MIN_BPM, MAX_BPM = 40.0, 120.0   # search range. Seated user, not an athlete.
OCTAVE_TOL = 0.8      # a lag of 2T correlates nearly as well as T, so the global
                      # max halves the rate. Take the FIRST peak within this
                      # fraction of the best instead.
MIN_QUALITY = 0.05    # normalised peak below this is noise, not a pulse
PUBLISH_HZ = 1.0      # BLE streams at 4–64 Hz; decide() must not run that fast
RECONNECT_DELAY = 1.0 # s between scan attempts. find() already blocks ~10 s, so
                      # this only sets how briefly the radio idles in between.
ARRIVAL_GAP = 120.0   # s offline before a reconnect counts as an ARRIVAL rather
                      # than the link bouncing. Longer than the worst observed
                      # 0x08 drop+rescan, shorter than a coffee break.


class HeartRate:
    """BVP waveform → BPM, by autocorrelation over a sliding window.

    Beat-by-beat detection does not survive this signal. py_e4lib's FIR chain is
    a low-pass, so the wrist PPG keeps several zero crossings per beat at ~10
    units amplitude: replaying a 26 s recording gave intervals of 0.96, 0.64,
    1.22, 0.90, 0.88, 1.00, 0.61, 0.51 s against a true 1.03 s, and HR swinging
    43–134 against a true 56–60. No refractory value fixes that — the spurious
    crossing lands at ~0.5 s.

    Autocorrelation asks what period the window repeats at, so it is indifferent
    to waveform shape, drift and the dicrotic notch. Same recording: 55–64 BPM.
    Stdlib only.
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
            # Rolling mean high-passes the signal: any DC offset or slow drift
            # correlates with everything and buries the pulse.
            self._buf.append(v - sum(self._mean_buf) / len(self._mean_buf))

    def bpm(self):
        """BPM as a float, or None while warming up or when the signal is noise."""
        if len(self._buf) < self._buf.maxlen:
            return None

        # The rolling mean leaves a constant residual on a sloping baseline, and
        # a constant correlates equally at every lag — which drags the peak to
        # the shortest one and reports MAX_BPM. Re-zero the window itself.
        d = list(self._buf)
        m = sum(d) / len(d)
        d = [x - m for x in d]

        c0 = sum(x * x for x in d) / len(d)
        if c0 <= 0.0:
            return None

        # O(window x lags) ~ 30k float ops per second in pure Python, measured
        # well under the 1 s budget. Decimate to 32 Hz if it ever matters — the
        # pulse is under 2 Hz, so nothing is lost.
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
        # around 60, which shows up as a stair-step on the topic.
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

        # The BVP-derived HR is the weakest number this node produces. False lets
        # a dedicated HR sensor own /heartrate (see sensors.launch.py) and skips
        # the 64 Hz BVP subscription entirely, not just the publish.
        self._publish_hr = self.declare_parameter("publish_hr", True).value

        # No /hrv publisher: RMSSD is defined on beat-to-beat differences, which
        # this signal cannot give (see HeartRate), and it amplifies exactly that
        # error — it read ~450 ms against a real 20–100. state_estimator latches
        # the first /hrv forever, so a wrong number is worse than none.
        self._hr_pub = (self.create_publisher(Int32, "/heartrate", 10)
                        if self._publish_hr else None)
        self._eda_pub = self.create_publisher(Float32, "/eda", 10)
        self._temp_pub = self.create_publisher(Float32, "/skin_temp", 10)
        self._motion_pub = self.create_publisher(Float32, "/motion", 10)

        # Wearing the band IS the arrival signal. An event, not a stream, so
        # subscribers act once instead of each owning the same edge detection.
        self._arrival_pub = self.create_publisher(Empty, "/user_arrived", 10)
        self._last_seen = float("-inf")   # never seen. Not 0.0: monotonic() is
                                          # uptime, so a launch within ARRIVAL_GAP
                                          # of boot would mute the first connect

        # BLE thread writes these, timer thread reads them. Plain assignment and
        # deque.append are GIL-atomic; add a lock only for a read-modify-write.
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
                    if time.monotonic() - self._last_seen > ARRIVAL_GAP:
                        self.get_logger().info("[arrival] /user_arrived")
                        self._arrival_pub.publish(Empty())
                    while rclpy.ok() and client.connected:
                        self._last_seen = time.monotonic()
                        await asyncio.sleep(1.0)
                self.get_logger().warn("E4 link lost")
            except Exception as e:
                self.get_logger().warn(f"E4 connection failed: {e}")
            if rclpy.ok():
                await asyncio.sleep(RECONNECT_DELAY)


def demo():
    n = int(SAMPLE_RATE * 60)

    def ppg(bpm, drift=0.0):
        """A pulse shape, not a sine: the second harmonic puts a dicrotic notch
        in each beat, which is what broke zero-crossing detection. Optional drift
        is the wandering baseline the low-pass leaves behind."""
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
