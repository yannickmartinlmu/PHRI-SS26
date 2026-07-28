# examples/basic_usage.py

import asyncio
import time
from py_e4lib import E4Client

# ponytail: per-packet print() blocks the BLE event loop (BVP ~64 Hz ×4 sensors).
# Throttle to 1 line/sec/sensor and show pkts/s so you can SEE the rate. If the
# disconnects stop now, event-loop starvation from terminal I/O was the cause.
_last = {}
_count = {}


def _log(name, msg):
    _count[name] = _count.get(name, 0) + 1
    now = time.monotonic()
    if now - _last.get(name, 0) >= 1.0:
        print(f"[{name}] {_count[name]:3d} pkts/s | {msg}")
        _last[name] = now
        _count[name] = 0


def on_bvp(values):
    """Handle BVP data."""
    avg = sum(values) / len(values)
    _log("BVP", f"avg: {avg:.2f}")


def on_gsr(values):
    """Handle GSR/EDA data."""
    avg = sum(values) / len(values)
    _log("EDA", f"avg: {avg:.3f} µS")


def on_temp(values):
    """Handle temperature data."""
    avg = sum(values) / len(values)
    _log("Temp", f"avg: {avg:.2f}°C")


def on_acc(values):
    """Handle accelerometer data."""
    # values are (x, y, z) tuples, divide by 64.0 for g-force
    avg_x = sum(x for x, _, _ in values) / len(values) / 64.0
    avg_y = sum(y for _, y, _ in values) / len(values) / 64.0
    avg_z = sum(z for _, _, z in values) / len(values) / 64.0
    _log("ACC", f"X={avg_x:.2f}g, Y={avg_y:.2f}g, Z={avg_z:.2f}g")


async def main():
    client = await E4Client.find()
    client.enable_bvp(on_bvp)
    client.enable_gsr(on_gsr)
    client.enable_temp(on_temp)
    client.enable_acc(on_acc)

    # 0x08 RF drops are inherent to the E4 — reconnect instead of fighting them.
    # ponytail: reconnects by cached address; re-scan on repeated failure if the
    # watch is power-cycling, not just dropping. Not needed until it happens.
    # Back off on repeated failures so a watch that's simply OFF doesn't spin the
    # controller into a firmware wedge (2s, 4s, 8s ... capped at 30s).
    fails = 0
    while True:
        try:
            await client.connect()
            await client.start()
            fails = 0
            while client.connected:
                await asyncio.sleep(1)
        except Exception as e:
            print(f"stream error: {e}")
        fails += 1
        delay = min(2 * 2 ** (fails - 1), 30)
        print(f"reconnecting in {delay}s... (Ctrl+C to quit)")
        await asyncio.sleep(delay)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
