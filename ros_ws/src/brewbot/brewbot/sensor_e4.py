#!/usr/bin/env python3
# Empatica E4 → the scalar topics state_estimator expects.
# The E4 has no HR characteristic, it streams a 64 Hz BVP waveform, so the
# beat detection lives here: py_e4lib stays a pure BLE driver, and
# /heartrate stays the swappable seam any other HR sensor can feed.

import asyncio
import math
import statistics
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

SAMPLE_RATE = 64.0    # Hz, E4 BVP nominal. Real crystal drifts — if HR reads
                      # consistently a few % off, tune here, not in the math.
REFRACTORY = 0.3      # s between beats → caps at 200 BPM, kills the dicrotic notch
MIN_IBI, MAX_IBI = 0.3, 2.0   # sanity gate, 30–200 BPM
HR_WINDOW = 5         # IBIs to median for a BPM reading
HRV_WINDOW = 30       # IBIs for RMSSD — also the warm-up before /hrv is published
PUBLISH_HZ = 1.0      # BLE streams at 4–64 Hz; decide() must not run that fast
RECONNECT_DELAY = 5.0 # s, the E4's 0x08 supervision-timeout drops are routine


class BeatDetector:
    """BVP waveform → inter-beat intervals → HR and RMSSD.

    py_e4lib's FIR+Kalman chain has already band-passed the signal, so a rising
    zero-crossing plus a refractory window is enough — no scipy, no numpy.
    Sample times come from a sample counter, not the wall clock, so BLE packet
    jitter can't smear the intervals.
    """

    def __init__(self):
        self._n = 0
        self._mean_buf = deque(maxlen=int(SAMPLE_RATE))
        self._prev = 0.0
        self._last_beat = None
        self._ibis = deque(maxlen=HRV_WINDOW)

    def feed(self, values):
        for v in values:
            self._mean_buf.append(v)
            self._n += 1

            # Rolling mean re-centres the signal; the Kalman output isn't
            # guaranteed zero-mean, and a drifting baseline would fake crossings.
            centered = v - sum(self._mean_buf) / len(self._mean_buf)

            rising = self._prev <= 0.0 < centered
            # Sub-sample crossing time. Without it, beats snap to the 15.6 ms
            # sample grid and that quantization alone fakes ~12 ms of RMSSD —
            # a quarter of a real HRV reading.
            frac = -self._prev / (centered - self._prev) if rising else 0.0
            t = (self._n - 1 + frac) / SAMPLE_RATE

            settled = self._last_beat is None or t - self._last_beat >= REFRACTORY
            if rising and settled:
                if self._last_beat is not None:
                    ibi = t - self._last_beat
                    # Also swallows the one bogus interval after a reconnect.
                    if MIN_IBI <= ibi <= MAX_IBI:
                        self._ibis.append(ibi)
                self._last_beat = t

            self._prev = centered

    def hr(self):
        """BPM, or None until enough beats. Median, so one dropped beat
        doesn't swing the reading the way a mean would."""
        if len(self._ibis) < 3:
            return None
        recent = list(self._ibis)[-HR_WINDOW:]
        return round(60.0 / statistics.median(recent))

    def hrv(self):
        """RMSSD in ms, or None during warm-up.

        state_estimator latches the FIRST /hrv it sees as the session baseline,
        so nothing is published until the window is full — a garbage first value
        would poison the fatigue branch for the whole session.
        """
        if len(self._ibis) < HRV_WINDOW:
            return None
        ibis = list(self._ibis)
        diffs = [b - a for a, b in zip(ibis, ibis[1:])]
        return math.sqrt(sum(d * d for d in diffs) / len(diffs)) * 1000.0


class E4SensorNode(Node):

    def __init__(self):
        super().__init__("sensor_e4")

        self._address = self.declare_parameter("address", "").value

        self._hr_pub = self.create_publisher(Int32, "/heartrate", 10)
        self._hrv_pub = self.create_publisher(Float32, "/hrv", 10)
        self._eda_pub = self.create_publisher(Float32, "/eda", 10)
        self._temp_pub = self.create_publisher(Float32, "/skin_temp", 10)
        self._motion_pub = self.create_publisher(Float32, "/motion", 10)

        # ponytail: BLE thread writes these, timer thread reads them. Plain
        # attribute assignment and deque.append are GIL-atomic; add a lock only
        # if this ever grows into a read-modify-write.
        self._beats = BeatDetector()
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
        hr = self._beats.hr()
        if hr is not None:
            self._hr_pub.publish(Int32(data=hr))

        hrv = self._beats.hrv()
        if hrv is not None:
            self._hrv_pub.publish(Float32(data=hrv))

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
    # 72 BPM sine at the real sample rate, 60 s.
    d = BeatDetector()
    bpm = 72.0
    n = int(SAMPLE_RATE * 60)
    d.feed([math.sin(2 * math.pi * (bpm / 60.0) * (i / SAMPLE_RATE)) for i in range(n)])
    assert d.hr() == 72, d.hr()
    assert d.hrv() < 2.0, d.hrv()   # perfectly regular → near-zero RMSSD

    # Warm-up gates: no readings out of thin air.
    empty = BeatDetector()
    assert empty.hr() is None and empty.hrv() is None

    # Nonsense input must not invent a heart rate.
    flat = BeatDetector()
    flat.feed([0.0] * n)
    assert flat.hr() is None, flat.hr()

    # A 4 Hz "pulse" is faster than any heart — refractory window must reject it.
    fast = BeatDetector()
    fast.feed([math.sin(2 * math.pi * 4.0 * (i / SAMPLE_RATE)) for i in range(n)])
    assert fast.hr() is None or fast.hr() <= 200, fast.hr()

    print("BeatDetector self-check passed")


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
