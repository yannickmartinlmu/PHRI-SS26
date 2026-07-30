#!/usr/bin/env python3
"""Download the MediaPipe models the perception nodes need.

gesture_recognizer.task  hands  — thumbs up/down, open palm
efficientdet_lite0.tflite people — presence only, so the arm knows which way to
                                   look. COCO detector, we allowlist "person".
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path

MODEL_URLS = {
    "gesture_recognizer.task": (
        "https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
        "gesture_recognizer/float16/latest/gesture_recognizer.task"
    ),
    "efficientdet_lite0.tflite": (
        "https://storage.googleapis.com/mediapipe-models/object_detector/"
        "efficientdet_lite0/float32/latest/efficientdet_lite0.tflite"
    ),
}


def main() -> int:
    package_root = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "brewbot"

    parser = argparse.ArgumentParser()
    parser.add_argument("--models-dir", type=Path, default=package_root / "models")
    args = parser.parse_args()

    models_dir = args.models_dir.expanduser().resolve()
    models_dir.mkdir(parents=True, exist_ok=True)

    for name, url in MODEL_URLS.items():
        output = models_dir / name
        temporary = output.with_suffix(output.suffix + ".part")

        print(f"Downloading {url}")
        print(f"       to {output}")
        try:
            urllib.request.urlretrieve(url, temporary)
            temporary.replace(output)
        except Exception as exc:
            temporary.unlink(missing_ok=True)
            print(f"Download failed: {exc}", file=sys.stderr)
            return 1

        digest = hashlib.sha256(output.read_bytes()).hexdigest()
        print(f"Downloaded {output.stat().st_size / 1024 / 1024:.1f} MiB")
        print(f"SHA256: {digest}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
