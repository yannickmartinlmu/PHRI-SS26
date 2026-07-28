#!/usr/bin/env python3
# Record every E4 sample to CSV for offline analysis.
#
# The other examples average each packet into one number, which destroys the BVP
# waveform and hides packet boundaries. This keeps both: one row per sample, plus
# the raw packet bytes for BVP and EDA so a suspect parser can be re-tested
# offline without another session on the wrist.
#
# Usage: python3 examples/record.py [seconds] [outfile]

import asyncio
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from py_e4lib import E4Client  # noqa: E402

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("e4_recording.csv")


async def main():
    t0 = time.monotonic()

    with OUT.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "sensor", "i", "a", "b", "c"])

        # One timestamp per packet, plus the sample's index within it. The E4
        # doesn't tell us intra-packet timing, so don't invent it here — record
        # what arrived and let the analysis decide how to space the samples.
        def row(sensor, values, triple=False):
            t = f"{time.monotonic() - t0:.4f}"
            for i, v in enumerate(values):
                w.writerow([t, sensor, i, *(v if triple else (v, "", ""))])

        client = await E4Client.find()
        client.enable_bvp(lambda v: row("BVP", v))
        client.enable_gsr(lambda v: row("EDA", v))
        client.enable_temp(lambda v: row("TEMP", v))
        client.enable_acc(lambda v: row("ACC", v, triple=True))

        # Raw bytes as well: if the decoder is what's wrong, the decoded values
        # can't show it. Instance attributes shadow the bound methods that
        # start() hands to start_notify, so this has to happen before start().
        for name, attr in (("BVP_RAW", "_handle_bvp"), ("EDA_RAW", "_handle_gsr")):
            def tap(sender, data, _orig=getattr(client, attr), _name=name):
                w.writerow([f"{time.monotonic() - t0:.4f}", _name, 0, data.hex(), "", ""])
                _orig(sender, data)
            setattr(client, attr, tap)

        async with client:
            await client.start()
            print(f"Recording {DURATION:.0f}s to {OUT} — sit still, arm relaxed.")
            await asyncio.sleep(DURATION)

    print(f"Wrote {OUT} ({OUT.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print(f"\nStopped early — {OUT} still has whatever was recorded.")
