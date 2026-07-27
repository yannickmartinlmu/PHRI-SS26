#!/usr/bin/env python3
"""Download Google's official MediaPipe Gesture Recognizer model bundle."""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
    "gesture_recognizer/float16/latest/gesture_recognizer.task"
)


def main() -> int:
    package_root = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "brewbot"
    default_output = package_root / "models" / "gesture_recognizer.task"

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=default_output)
    args = parser.parse_args()

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".part")

    print(f"Downloading {MODEL_URL}")
    print(f"       to {output}")
    try:
        urllib.request.urlretrieve(MODEL_URL, temporary)
        temporary.replace(output)
    except Exception as exc:
        temporary.unlink(missing_ok=True)
        print(f"Download failed: {exc}", file=sys.stderr)
        return 1

    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    print(f"Downloaded {output.stat().st_size / 1024 / 1024:.1f} MiB")
    print(f"SHA256: {digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
