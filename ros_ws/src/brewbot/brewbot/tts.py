#!/usr/bin/env python3

import os
import queue
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- espeak-ng backend ---
# Install: sudo apt install espeak-ng

def speak_espeak(text: str):
    subprocess.run(["espeak-ng", text], check=True, capture_output=True)


# --- Piper backend ---
# Install: pip install piper-tts
# Models:  https://github.com/rhasspy/piper/blob/master/VOICES.md
# Download a model .onnx + .onnx.json, set PIPER_MODEL_PATH below

PIPER_MODEL_PATH = os.path.expanduser("~/piper/en_GB-alan-medium.onnx")

_piper_voice = None

def speak_piper(text: str):
    global _piper_voice
    if _piper_voice is None:
        from piper import PiperVoice
        _piper_voice = PiperVoice.load(PIPER_MODEL_PATH)

    import wave
    import numpy as np

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        tmp_path = f.name

    try:
        with wave.open(tmp_path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(_piper_voice.config.sample_rate)
            for chunk in _piper_voice.synthesize(text):
                audio_int16 = (chunk.audio_float_array * 32767).astype(np.int16)
                wav_file.writeframes(audio_int16.tobytes())
        result = subprocess.run(["aplay", tmp_path], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"aplay failed (rc={result.returncode}): {result.stderr.strip()}")
    finally:
        os.unlink(tmp_path)


# --- Active backend ---
# Swap speak_espeak for speak_piper once Piper is installed and model is downloaded
_speak = speak_piper


class TtsNode(Node):

    def __init__(self):
        super().__init__("tts")

        self._queue = queue.Queue()

        self._worker = threading.Thread(
            target=self._speak_worker, daemon=True
        )
        self._worker.start()

        self._sub = self.create_subscription(
            String, "/tts_text", self._on_text, 10
        )

        self.get_logger().info("TTS node ready")

    def _on_text(self, msg):
        self.get_logger().info(f"[TTS] Queued: '{msg.data}'")
        self._queue.put(msg.data)

    def _speak_worker(self):
        while True:
            text = self._queue.get()
            try:
                _speak(text)
            except FileNotFoundError as e:
                self.get_logger().error(f"TTS binary not found: {e}")
            except Exception as e:
                self.get_logger().error(f"TTS failed: {e}")


def main():
    rclpy.init()
    node = TtsNode()
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
